# talk-key: session keys for 12-talk

A small Cloudflare Worker. It keeps our OpenAI API key, so the robot never has it. The robot asks it for a Realtime session key that works for 60 s, and uses that key to open its WebSocket to `api.openai.com`.

```
POST https://stackchan-talk.diegoromero.workers.dev/key
X-Robot-Token: <the robot's token>
```

The answer is plain text, two lines: the model (`gpt-realtime-2.1`) and the key (`ek_...`). Without the right token: 403. Any other path or method: 404.

The Worker sets the whole session: model, voice (`marin`), instructions, 24 kHz PCM in and out, and the server's voice detection with `interrupt_response` (full duplex: when you talk over the robot, its answer stops). The robot can't change any of it.

## Secrets

Set with `wrangler secret put`, never in git:

| Secret | What |
| --- | --- |
| `OPENAI_API_KEY` | Our OpenAI API key |
| `ROBOT_TOKEN` | A random token (48 hex characters). The same value is `TALK_TOKEN` in `m5stack/wifi.env`, which goes into the firmware at build time |

`wifi.env` also has `TALK_URL`. `scripts/build.sh` passes both to the build, as it does with the Wi-Fi settings.

## Deploy

```bash
cd workers/talk-key && npx wrangler deploy
```

## Risks

- The robot token is in the firmware. Someone with the robot (or its flash) can get session keys, and each talk costs money. To stop it, change `ROBOT_TOKEN`.
- Cloudflare's bot check blocks some clients (Python's `urllib` got error 1010). The ESP32's HTTP client and curl get through.
