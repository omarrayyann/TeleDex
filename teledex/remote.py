"""Direct internet connections between the phone and a Session on different networks.

Both sides learn their public UDP address from a STUN server, swap addresses through the
TeleDex rendezvous server (see ``signal/``), then send to each other at the same time so
their routers let the packets through ("hole punching"). Poses then flow straight from the
phone over UDP; the rendezvous server only sees a few small messages per session.

Packets are ``MAGIC | kind | sequence | payload | HMAC``, signed with a key that only the
QR code carries, so nobody who lacks it can inject poses.
"""
import asyncio
import base64
import hashlib
import hmac
import ipaddress
import json
import os
import secrets
import socket
import struct
import time

import websockets

# Rendezvous server used when none is given; set TELEDEX_SIGNAL_URL to override
SIGNAL_URL = os.environ.get("TELEDEX_SIGNAL_URL", "https://teledex-signal.teledex.workers.dev")
STUN_SERVERS = (("stun.l.google.com", 19302), ("stun.cloudflare.com", 3478))
STUN_COOKIE = 0x2112A442
STUN_REFRESH = 20.0  # seconds between STUN lookups, which also keep the router mapping open

MAGIC = b"TDX1"
PUNCH, POSE, VIBRATE, PING = 1, 2, 3, 4
HEADER_BYTES = len(MAGIC) + 9  # magic, kind (1 byte), sequence (8 bytes)
MAC_BYTES = 16

PUNCH_INTERVAL = 0.1  # seconds between hole-punching packets
PUNCH_DURATION = 15.0  # seconds to keep punching after learning the phone's addresses
PING_INTERVAL = 1.0  # seconds between keepalives to a connected phone
TIMEOUT = 3.0  # seconds without packets before the phone counts as disconnected


def pack(key, kind, seq, payload=b""):
    """Sign and frame one packet."""
    body = MAGIC + struct.pack(">BQ", kind, seq) + payload
    return body + hmac.new(key, body, hashlib.sha256).digest()[:MAC_BYTES]


def unpack(key, packet):
    """Return ``(kind, seq, payload)`` for a correctly signed packet, else None."""
    if len(packet) < HEADER_BYTES + MAC_BYTES or not packet.startswith(MAGIC):
        return None
    body, mac = packet[:-MAC_BYTES], packet[-MAC_BYTES:]
    if not hmac.compare_digest(mac, hmac.new(key, body, hashlib.sha256).digest()[:MAC_BYTES]):
        return None
    kind, seq = struct.unpack(">BQ", body[len(MAGIC) : HEADER_BYTES])
    return kind, seq, body[HEADER_BYTES:]


def stun_request():
    """A STUN binding request and its transaction id."""
    transaction = os.urandom(12)
    return transaction, struct.pack(">HHI", 0x0001, 0, STUN_COOKIE) + transaction


def parse_stun(packet, transaction):
    """The ``(ip, port)`` a STUN binding response reports for us, or None."""
    if len(packet) < 20:
        return None
    kind, length, cookie = struct.unpack(">HHI", packet[:8])
    if kind != 0x0101 or cookie != STUN_COOKIE or packet[8:20] != transaction:
        return None
    i = 20
    while i + 4 <= min(len(packet), 20 + length):
        attribute, size = struct.unpack(">HH", packet[i : i + 4])
        value = packet[i + 4 : i + 4 + size]
        if size >= 8 and value[1] == 1:  # IPv4
            if attribute in (0x0020, 0x8020):  # XOR-MAPPED-ADDRESS
                port = struct.unpack(">H", value[2:4])[0] ^ (STUN_COOKIE >> 16)
                ip = struct.unpack(">I", value[4:8])[0] ^ STUN_COOKIE
                return socket.inet_ntoa(struct.pack(">I", ip)), port
            if attribute == 0x0001:  # MAPPED-ADDRESS
                return socket.inet_ntoa(value[4:8]), struct.unpack(">H", value[2:4])[0]
        i += 4 + size + (-size % 4)
    return None


def is_stun(packet):
    return len(packet) >= 20 and packet[4:8] == struct.pack(">I", STUN_COOKIE)


def is_ipv6(ip):
    return ":" in ip


def global_ipv6():
    """This computer's preferred public IPv6 address, or None.

    IPv6 needs no address translation, so it works on networks whose shared IPv4
    translator (NAT64, carrier-grade NAT) blocks hole punching.
    """
    try:
        with socket.socket(socket.AF_INET6, socket.SOCK_DGRAM) as probe:
            probe.connect(("2001:4860:4860::8888", 53))  # no packet is sent; this only picks a route
            ip = probe.getsockname()[0]
    except OSError:
        return None
    return None if ip.startswith(("fe80", "::1", "fc", "fd")) else ip


def reachable_ipv4(ip):
    """Whether a local IPv4 address can be reached from another device at all.

    192.0.0.0/29 is the address macOS and iOS invent on IPv6-only networks (464XLAT).
    """
    return not ip.startswith(("127.", "192.0.0.", "169.254."))


class RemoteLink(asyncio.DatagramProtocol):
    """The computer's side of a direct internet connection, run on the Session's event loop."""

    def __init__(self, session, signal_url):
        self.session = session
        self.signal_url = signal_url.rstrip("/")
        self.sid = secrets.token_urlsafe(12)
        self.key = secrets.token_bytes(16)
        self.key_text = base64.urlsafe_b64encode(self.key).rstrip(b"=").decode()

        self.transport = None  # IPv4 socket
        self.transport6 = None  # IPv6 socket, on the same port
        self.public = None  # our (ip, port) as the internet sees it
        self.ready = False  # registered with the rendezvous server
        self.connected = False
        self.peer = None  # the phone's address once a signed packet arrives from it
        self.last_seen = 0.0

        self._socket = None
        self._tasks = []
        self._sent_candidates = None
        self._phone_candidates = []
        self._punch_until = 0.0
        self._stun = {}
        self._seq = 0
        self._last_pose_seq = -1
        self._ipv6 = None
        self._ipv6_checked = 0.0

    # -- Lifecycle -----------------------------------------------------------------

    async def start(self, port):
        loop = asyncio.get_running_loop()
        self.transport, _ = await loop.create_datagram_endpoint(lambda: self, local_addr=("0.0.0.0", port))
        try:
            sock6 = socket.socket(socket.AF_INET6, socket.SOCK_DGRAM)
            sock6.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_V6ONLY, 1)
            sock6.bind(("::", port))
            self.transport6, _ = await loop.create_datagram_endpoint(lambda: self, sock=sock6)
        except OSError as error:
            if self.session.debug:
                print(f"[DEBUG] No IPv6 socket: {error}")
        self._tasks = [loop.create_task(t) for t in (self._stun_loop(), self._signal_loop(), self._tick_loop())]

    async def close(self):
        for task in self._tasks:
            task.cancel()
        await asyncio.gather(*self._tasks, return_exceptions=True)
        if self._socket is not None:
            await self._socket.close()
        for transport in (self.transport, self.transport6):
            if transport is not None:
                transport.close()
        self.connected = False

    def candidates(self):
        """Addresses the phone may reach us at: public IPv6, public IPv4 (from STUN) and local IPv4."""
        port = self.transport.get_extra_info("sockname")[1] if self.transport else self.session.port
        found = []
        if self.transport6 is not None:
            now = time.monotonic()
            if now - self._ipv6_checked > 10.0:  # temporary IPv6 addresses rotate; recheck now and then
                self._ipv6, self._ipv6_checked = global_ipv6(), now
            if self._ipv6:
                found.append({"ip": self._ipv6, "port": port})
        if self.public:
            found.append({"ip": self.public[0], "port": self.public[1]})
        local = {"ip": self.session.ip_address, "port": port}
        if reachable_ipv4(local["ip"]) and local not in found:
            found.append(local)
        return found

    def same_network(self, ip):
        """Whether a phone address is on this computer's own network (so traffic stays local)."""
        try:
            address = ipaddress.ip_address(ip)
        except ValueError:
            return False
        if address.version == 4:
            return address.is_private and reachable_ipv4(ip)
        own = self._ipv6 or global_ipv6()
        return bool(own) and ipaddress.ip_network(f"{own}/64", strict=False) == ipaddress.ip_network(f"{ip}/64", strict=False)

    # -- Sending -------------------------------------------------------------------

    def _send(self, kind, payload, address):
        transport = self.transport6 if is_ipv6(address[0]) else self.transport
        if transport is not None:
            self._seq += 1
            transport.sendto(pack(self.key, kind, self._seq, payload), address[:2])

    def vibrate(self, message):
        if self.connected and self.peer:
            for _ in range(3):  # UDP may drop one; the app ignores repeats within a moment
                self._send(VIBRATE, message.encode(), self.peer)

    # -- Receiving -----------------------------------------------------------------

    def datagram_received(self, data, address):
        if is_stun(data):
            for transaction, server in list(self._stun.items()):
                mapped = parse_stun(data, transaction)
                if mapped:
                    del self._stun[transaction]
                    self.public = mapped
                    self._share_candidates()
            return

        packet = unpack(self.key, data)
        if packet is None:
            return
        kind, seq, payload = packet

        address = address[:2]  # IPv6 adds flow info and scope
        if not self.connected or address == self.peer or kind == PUNCH:
            self.peer = address
        self.last_seen = time.monotonic()
        if not self.connected:
            self.connected = True
            self._last_pose_seq = -1
            self.session._on_remote_connect(address)

        if kind == PUNCH:
            self._send(PUNCH, b"", address)
        elif kind == POSE:
            # Drop late, out-of-order poses; a big step back means the phone restarted its count
            if seq <= self._last_pose_seq and self._last_pose_seq - seq < 10_000:
                return
            self._last_pose_seq = seq
            try:
                self.session._on_message(json.loads(payload))
            except (ValueError, TypeError):
                pass

    def error_received(self, exc):
        if self.session.debug:
            print(f"[DEBUG] UDP error: {exc}")

    # -- Background loops ------------------------------------------------------------

    async def _stun_loop(self):
        loop = asyncio.get_running_loop()
        while True:
            for host, port in STUN_SERVERS:
                try:
                    infos = await loop.getaddrinfo(host, port, family=socket.AF_INET, type=socket.SOCK_DGRAM)
                except OSError:
                    continue
                transaction, request = stun_request()
                self._stun[transaction] = host
                self.transport.sendto(request, infos[0][4])
            await asyncio.sleep(STUN_REFRESH)
            self._stun.clear()

    async def _signal_loop(self):
        url = self.signal_url.replace("https://", "wss://", 1).replace("http://", "ws://", 1)
        url = f"{url}/v1/s/{self.sid}"
        while True:
            try:
                # Protocol pings keep the socket alive through idle-dropping networks and are not billed
                async with websockets.connect(url, open_timeout=10, ping_interval=30, ping_timeout=20) as ws:
                    self._socket = ws
                    self._sent_candidates = None
                    await asyncio.sleep(1.0)  # give STUN a moment so the first share is complete
                    await self._send_candidates()
                    self.ready = True
                    async for message in ws:
                        self._on_signal(message)
            except (OSError, websockets.WebSocketException, asyncio.TimeoutError) as error:
                if self.session.debug:
                    print(f"[DEBUG] Rendezvous connection failed: {error}")
            self._socket = None
            self.ready = False
            await asyncio.sleep(3.0)

    def _on_signal(self, message):
        try:
            msg = json.loads(message)
        except ValueError:
            return
        if msg.get("type") == "peer" and isinstance(msg.get("candidates"), list):
            self._phone_candidates = [
                (c["ip"], int(c["port"]))
                for c in msg["candidates"]
                if isinstance(c, dict) and isinstance(c.get("ip"), str) and "port" in c
            ]
            self._punch_until = time.monotonic() + PUNCH_DURATION

    def _share_candidates(self):
        if self._socket is not None:
            asyncio.get_running_loop().create_task(self._send_candidates())

    async def _send_candidates(self):
        candidates = self.candidates()
        if self._socket is None or not candidates or candidates == self._sent_candidates:
            return
        try:
            await self._socket.send(json.dumps({"type": "candidates", "candidates": candidates}))
            self._sent_candidates = candidates
        except websockets.WebSocketException:
            pass

    async def _tick_loop(self):
        last_ping = 0.0
        while True:
            await asyncio.sleep(PUNCH_INTERVAL)
            now = time.monotonic()
            if self.connected:
                if now - self.last_seen > TIMEOUT:
                    self.connected = False
                    self.peer = None
                    self.session._on_remote_disconnect()
                elif now - last_ping >= PING_INTERVAL:
                    self._send(PING, b"", self.peer)
                    last_ping = now
            elif self._phone_candidates and now < self._punch_until:
                for address in self._phone_candidates:
                    self._send(PUNCH, b"", address)
            # A refreshed local address (e.g. after Session.ip_address is re-detected) is shared too
            if self._socket is not None and self.candidates() != self._sent_candidates:
                await self._send_candidates()
