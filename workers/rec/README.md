# rec: the robot's recordings in R2

A small Cloudflare Worker for [14-record](../../examples/stackchan/14-record). The robot uploads what it records, and the Mac app ([tools/scribe](../../tools/scribe)) lists it and downloads it as WAV.

```
robot --PUT pieces of 10 s, X-Robot-Token--> Worker --> R2 bucket stackchan-rec (EU)
Mac app <--WAV, Authorization: Bearer------- Worker <--
```

## API

The robot uses the header `X-Robot-Token` (the secret `REC_TOKEN`). It can only add:

| Request | What |
| --- | --- |
| `POST /rec` | Start a recording. Answer: `{"id": "20261107-121201-b0b5"}` (201) |
| `PUT /rec/<id>/<n>` | Piece `n` (0, 1, 2...): 16-bit PCM, 16 kHz, mono, up to 1 MB |
| `POST /rec/<id>/end` | The recording is over |

The Mac app uses `Authorization: Bearer <READ_TOKEN>`. It can read and delete:

| Request | What |
| --- | --- |
| `GET /rec` | `[{id, seconds, pieces, missing, ended}]`, newest first |
| `GET /rec/<id>.wav` | The whole recording as one WAV |
| `DELETE /rec/<id>` | Delete it |

A wrong or missing token gets 403. A bad id gets 400.

- The Worker names each recording from its own clock, in UTC, so the robot does not need to know the time.
- If a piece never came, the WAV has silence in its place. So the times after a gap stay right.
- Pieces are kept as they come (`rec/<id>/00000.pcm`...). The Worker joins them into one WAV when you ask for it.

## Storage and privacy

- The bucket `stackchan-rec` is in the EU jurisdiction.
- A lifecycle rule (`expire-30d`) deletes every object 30 days after upload. Delete a recording sooner from the app, or with `DELETE /rec/<id>`.
- One hour of sound is about 115 MB.

## Secrets

Set with `wrangler secret put`, never in git:

| Secret | What | Where else |
| --- | --- | --- |
| `REC_TOKEN` | The robot's token (48 hex characters) | `REC_TOKEN` in `m5stack/wifi.env`, with `REC_URL`. `scripts/build.sh` passes them to the build |
| `READ_TOKEN` | The Mac app's token (48 hex characters) | `~/.secrets/stackchan-rec.env` (`REC_URL`, `READ_TOKEN`). The app can import it into the keychain |

## Set up and deploy

```bash
cd workers/rec
npx wrangler r2 bucket create stackchan-rec -J eu
npx wrangler r2 bucket lifecycle add stackchan-rec expire-30d --expire-days 30 -J eu -y
npx wrangler deploy
npx wrangler secret put REC_TOKEN
npx wrangler secret put READ_TOKEN
```

To test it on your Mac, put test tokens in `.dev.vars` (ignored by git) and run `npx wrangler dev`. R2 is then a local copy.

## Risks

- The robot token is in the firmware. Someone with the robot (or its flash) can upload sound, but can't read or delete any. To stop it, change `REC_TOKEN`.
- The read token can read and delete every recording. Keep it on your Mac.
- The robot records the room. Tell people when it records (its face turns red), and delete what you don't need.
