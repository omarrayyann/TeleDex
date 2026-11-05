"""Phones without the TeleDex app: the web page streams its pose over a WebRTC data channel.

On Android, the QR link opens a web page (orayyan.com/teledex/clip/) that tracks the phone with
WebXR. A browser can neither send raw UDP nor open a plain ws:// connection from an HTTPS page, so
the page connects with WebRTC: it posts an offer to the rendezvous server, which hands it to this
computer over the RemoteLink's WebSocket; the answer goes back the same way, and the poses then
travel peer to peer. Offers are signed with the session key from the QR link.

Needs aiortc: ``pip install teledex[web]``.
"""
import asyncio
import base64
import hashlib
import hmac
import json

try:
    from aiortc import RTCConfiguration, RTCIceServer, RTCPeerConnection, RTCSessionDescription
    HAS_AIORTC = True
except ImportError:
    HAS_AIORTC = False

ICE_SERVERS = ["stun:stun.l.google.com:19302", "stun:stun.cloudflare.com:3478"]
MAC_BYTES = 16


def sign(key, kind, sdp):
    """Base64url HMAC-SHA256 of an SDP, truncated like the UDP packets' MACs."""
    mac = hmac.new(key, f"teledex-rtc-{kind}\n{sdp}".encode(), hashlib.sha256).digest()[:MAC_BYTES]
    return base64.urlsafe_b64encode(mac).rstrip(b"=").decode()


class WebPeers:
    """Answers WebRTC offers from the TeleDex web page and feeds its poses to the Session."""

    def __init__(self, session, key):
        self.session = session
        self.key = key
        self._peers = set()
        self._channel = None  # data channel of the connected page, if any
        self._warned = False

    @property
    def connected(self):
        return self._channel is not None and self._channel.readyState == "open"

    def send(self, message):
        if self.connected:
            self._channel.send(message)

    async def answer(self, offer_id, sdp, mac, reply):
        """Handle one offer; ``reply`` is an async callable that sends a message to the rendezvous."""
        if not isinstance(sdp, str) or not isinstance(mac, str):
            return
        if not hmac.compare_digest(mac, sign(self.key, "offer", sdp)):
            print("[WARNING] Ignored a web connection offer with a bad signature")
            return
        if not HAS_AIORTC:
            if not self._warned:
                print("[INFO] A phone tried to connect from the web page; pip install teledex[web] to allow it")
                self._warned = True
            return

        peer = RTCPeerConnection(RTCConfiguration(iceServers=[RTCIceServer(urls=ICE_SERVERS)]))
        self._peers.add(peer)

        @peer.on("datachannel")
        def on_datachannel(channel):
            @channel.on("open")
            def on_open():
                self._opened(channel)

            @channel.on("message")
            def on_message(message):
                if isinstance(message, str):
                    try:
                        self.session._on_message(json.loads(message))
                    except (ValueError, TypeError):
                        pass

            @channel.on("close")
            def on_close():
                self._closed(channel)

            if channel.readyState == "open":
                self._opened(channel)

        @peer.on("connectionstatechange")
        async def on_state():
            if peer.connectionState in ("failed", "closed"):
                if self._channel is not None and peer.sctp and self._channel.transport is peer.sctp:
                    self._closed(self._channel)
                self._peers.discard(peer)
                await peer.close()

        await peer.setRemoteDescription(RTCSessionDescription(sdp=sdp, type="offer"))
        await peer.setLocalDescription(await peer.createAnswer())  # gathers all candidates first
        answer = peer.localDescription.sdp
        await reply({"type": "rtc-answer", "id": offer_id, "sdp": answer, "mac": sign(self.key, "answer", answer)})

    def _opened(self, channel):
        if self._channel is channel:
            return
        self._channel = channel
        self.session._on_web_connect()

    def _closed(self, channel):
        if self._channel is channel:
            self._channel = None
            self.session._on_web_disconnect()

    async def close(self):
        for peer in list(self._peers):
            await peer.close()
        self._peers.clear()
        self._channel = None
