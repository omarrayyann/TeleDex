// TeleDex rendezvous: lets a phone and a teledex Session on different networks swap their
// public UDP addresses so they can connect directly. Pose data never passes through here.
//
// The computer holds a WebSocket to /v1/s/<session id> and sends {"type": "candidates", ...};
// the phone POSTs its own candidates to the same path and gets the computer's back, while the
// computer is told the phone's over its socket. One Durable Object per session id.
//
// The TeleDex web page (Android, no app) cannot send raw UDP, so it uses WebRTC instead: it POSTs a
// signed offer to /v1/s/<session id>/rtc, which is handed to the computer over its socket, and the
// computer's answer is returned in the response. The page and computer then connect peer to peer.

const SESSION_PATH = /^\/v1\/s\/([A-Za-z0-9_-]{16,64})(\/rtc)?$/;
const MAX_BODY_BYTES = 2048;
const MAX_SDP_BYTES = 16384;
const MAX_CANDIDATES = 8;
const ANSWER_TIMEOUT_MS = 10000;
const CORS = {
  "access-control-allow-origin": "*",
  "access-control-allow-methods": "POST, OPTIONS",
  "access-control-allow-headers": "content-type",
};

function json(body, status = 200) {
  return new Response(JSON.stringify(body), {
    status,
    headers: { "content-type": "application/json", ...CORS },
  });
}

function isIPv4(ip) {
  const m = ip.match(/^(\d{1,3})\.(\d{1,3})\.(\d{1,3})\.(\d{1,3})$/);
  return Boolean(m) && m.slice(1).every((o) => Number(o) <= 255);
}

// Plain (non-scoped) IPv6 such as 2001:db8::1; the URL parser does the full check
function isIPv6(ip) {
  if (ip.length > 45 || !ip.includes(":") || !/^[0-9a-fA-F:.]+$/.test(ip)) return false;
  try {
    return new URL(`http://[${ip}]/`).hostname.length > 2;
  } catch {
    return false;
  }
}

// [{ip: "1.2.3.4", port: 1234}, {ip: "2001:db8::1", port: 1234}, ...] with valid ports
function validCandidates(value) {
  if (!Array.isArray(value) || value.length === 0 || value.length > MAX_CANDIDATES) return null;
  for (const c of value) {
    if (!c || typeof c.ip !== "string" || !(isIPv4(c.ip) || isIPv6(c.ip))) return null;
    if (!Number.isInteger(c.port) || c.port < 1 || c.port > 65535) return null;
  }
  return value.map((c) => ({ ip: c.ip, port: c.port }));
}

export class Rendezvous {
  constructor(state) {
    this.state = state;
    this.pending = new Map(); // offer id -> resolve(answer), while a web page waits
  }

  async fetch(request) {
    if (new URL(request.url).pathname.endsWith("/rtc")) return this.rtc(request);

    if (request.headers.get("Upgrade") === "websocket") {
      // The computer's socket; a new one replaces any previous (e.g. after a restart)
      for (const ws of this.state.getWebSockets()) ws.close(1000, "replaced");
      const [client, server] = Object.values(new WebSocketPair());
      this.state.acceptWebSocket(server);
      return new Response(null, { status: 101, webSocket: client });
    }

    if (request.method === "POST") {
      const text = await request.text();
      if (text.length > MAX_BODY_BYTES) return json({ error: "too large" }, 413);
      let candidates;
      try {
        candidates = validCandidates(JSON.parse(text).candidates);
      } catch {
        candidates = null;
      }
      if (!candidates) return json({ error: "bad candidates" }, 400);

      const host = await this.state.storage.get("host");
      const sockets = this.state.getWebSockets();
      if (!host || sockets.length === 0) return json({ error: "computer not online" }, 404);
      const message = JSON.stringify({ type: "peer", candidates });
      for (const ws of sockets) ws.send(message);
      return json({ candidates: host });
    }

    return json({ error: "not found" }, 404);
  }

  // A web page's WebRTC offer: pass it to the computer and return its answer
  async rtc(request) {
    if (request.method !== "POST") return json({ error: "not found" }, 404);
    const text = await request.text();
    if (text.length > MAX_SDP_BYTES) return json({ error: "too large" }, 413);
    let offer;
    try {
      offer = JSON.parse(text);
    } catch {
      offer = null;
    }
    if (!offer || typeof offer.sdp !== "string" || typeof offer.mac !== "string") {
      return json({ error: "bad offer" }, 400);
    }
    const sockets = this.state.getWebSockets();
    if (sockets.length === 0) return json({ error: "computer not online" }, 404);

    const id = crypto.randomUUID();
    const answer = new Promise((resolve) => {
      this.pending.set(id, resolve);
      setTimeout(() => resolve(null), ANSWER_TIMEOUT_MS);
    });
    const message = JSON.stringify({ type: "rtc-offer", id, sdp: offer.sdp, mac: offer.mac });
    for (const ws of sockets) ws.send(message);
    const reply = await answer;
    this.pending.delete(id);
    if (!reply) return json({ error: "computer did not answer" }, 504);
    return json({ sdp: reply.sdp, mac: reply.mac });
  }

  async webSocketMessage(ws, message) {
    if (typeof message !== "string" || message.length > MAX_SDP_BYTES) return;
    let msg;
    try {
      msg = JSON.parse(message);
    } catch {
      return;
    }
    if (msg.type === "rtc-answer") {
      const resolve = this.pending.get(msg.id);
      if (resolve && typeof msg.sdp === "string" && typeof msg.mac === "string") resolve(msg);
      return;
    }
    if (message.length > MAX_BODY_BYTES) return;
    const candidates = msg.type === "candidates" && validCandidates(msg.candidates);
    if (candidates) await this.state.storage.put("host", candidates);
  }

  async webSocketClose(ws) {
    const others = this.state.getWebSockets().filter((s) => s !== ws);
    if (others.length === 0) await this.state.storage.delete("host");
  }
}

export default {
  async fetch(request, env) {
    if (request.method === "OPTIONS") return new Response(null, { status: 204, headers: CORS });
    const match = new URL(request.url).pathname.match(SESSION_PATH);
    if (!match) return json({ service: "teledex rendezvous" }, 404);
    return env.RENDEZVOUS.get(env.RENDEZVOUS.idFromName(match[1])).fetch(request);
  },
};
