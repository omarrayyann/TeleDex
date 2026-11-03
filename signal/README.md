# TeleDex rendezvous

A Cloudflare Worker that lets the phone and a `teledex.Session(remote=True)` on different
networks swap public UDP addresses, so they can connect directly over the internet. Only a
few small messages per session pass through it; pose data goes straight between the devices.

Deploy once with a (free) Cloudflare account:

```bash
cd signal
npx wrangler login
npx wrangler deploy
```

Then use the printed `https://teledex-signal.<subdomain>.workers.dev` URL as the
`signal_url` (it is the default in `teledex/remote.py` and the app's `RemoteLink.swift`: https://teledex-signal.teledex.workers.dev).

Local testing: `npx wrangler dev` serves it at `http://localhost:8787`.
