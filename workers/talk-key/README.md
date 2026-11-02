# talk-key: the robot's bridge to GPT-Live

A small Cloudflare Worker for [12-talk](../../examples/stackchan/12-talk). It keeps our OpenAI API key, so the robot never has it.

GPT-Live has no short-lived keys for devices. So the robot opens a WebSocket to this Worker, and the Worker opens the session with OpenAI and passes the messages both ways:

```
robot  <-- WebSocket, X-Robot-Token -->  Worker  <-- WebSocket, API key -->  wss://api.openai.com/v1/live/sessions
```

```
GET wss://stackchan-talk.<account>.workers.dev/live
X-Robot-Token: <the robot's token>
```

Without the right token: 403. Any other path, or no WebSocket: 404.

The Worker starts the session itself (`session.start`), so the robot can't change it:

| Setting | Value |
| --- | --- |
| Voice model | `gpt-live-1`, full duplex: it listens while it speaks, and it stops when you talk over it |
| Voice | `vesper` |
| Audio | 16-bit PCM at 16 kHz both ways, the robot's own rate |
| Instructions | In Spanish: always speak Spanish, short answers, hand off what needs a search |
| Delegation | `gpt-6-luna` through Responses, with web search. Add its tools in `SESSION.delegation.responses.tools` in `src/index.js` |

From the robot, the Worker passes only audio (`session.input_audio.append`) and `session.close`. To the robot, it passes everything except the delegated model's own events (`response.event`). They are big, and the robot does not need them.

## Secrets

Set with `wrangler secret put`, never in git:

| Secret | What |
| --- | --- |
| `OPENAI_API_KEY` | Our OpenAI API key |
| `ROBOT_TOKEN` | A random token (48 hex characters). The same value is `TALK_TOKEN` in `m5stack/wifi.env`, which goes into the firmware at build time |

`wifi.env` also has `TALK_URL` (`wss://.../live`). `scripts/build.sh` passes both to the build, as it does with the Wi-Fi settings.

## Deploy

```bash
cd workers/talk-key && npx wrangler deploy
```

## Risks

- The robot token is in the firmware. Someone with the robot (or its flash) can open talks, and each talk costs money. To stop it, change `ROBOT_TOKEN`.
- Cloudflare's bot check blocks some clients (Python's `urllib` got error 1010). The ESP32's clients and curl get through.
- The Worker stays in the middle for the whole talk, which adds a little delay.
