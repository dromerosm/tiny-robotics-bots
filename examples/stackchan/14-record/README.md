# StackChan 14: record the room

12-talk's spoken conversation, plus a recording mode. Tap the robot's right eye and it records the room: the face turns red, and the sound goes to our Worker in pieces of 10 s. Tap the eye again to stop. The face stays pale red until the last piece is uploaded.

The face turns orange while the uploads fall behind: no network, or the Worker fails. It turns red again once they catch up. Red means "I record", orange means "I record, but the sound is not reaching R2 now". Later, the Mac app [Scribe](../../../tools/scribe) turns the recording into text and says who spoke when, all on the Mac.

The face, the audio, the echo canceller and the talk are 12-talk's as of commit `7ef1dae`. There is no talk while it records: a touch on the head does nothing.

| | |
| --- | --- |
| **Robot** | StackChan (CoreS3, ESP32-S3) |
| **Level** | Advanced |
| **Emulator** | ✅ Native `esp-emu`, without screen and audio: it checks that Wi-Fi and the server start |
| **Hardware used** | Both microphones (ES7210), speaker (AW88298, silent while it records), screen, touch, head touch |
| **Also needs** | The Worker in [workers/rec](../../../workers/rec), and `REC_URL` and `REC_TOKEN` in `wifi.env`. For talks, the same as 12-talk |

## How it works

```
4 inputs of the ES7210 -> audio task -> aec task: both mics, echo cancelled, averaged (talk.c, as in 12-talk)
                                            |  with no talk, the speaker plays silence
                                            v
                         take task: every 100 ms, into a 10 s piece in PSRAM (rec.c)
                                            |  6 pieces: 60 s can wait for the network
                                            v
                         upload task: HTTPS PUT /rec/<id>/<n> -> Worker -> R2 (EU)
```

- **Gain and high-pass.** While it records, the speaker is silent, so the microphones get 36 dB instead of 30 (the ES7210 goes to 37.5). A 2nd-order Butterworth high-pass at 90 Hz, in integer math, takes out hum, desk knocks and the DC offset. Both go back to normal when the recording ends.
- **The sound.** talk.c already runs the microphones and the echo canceller all the time. With no talk, the speaker plays silence, so the cleaned sound is simply the average of both microphones, at 16 kHz. The recorder reads it through `talk_tap()`, a small addition to 12-talk's talk.c. `POST /rec?src=mic` records microphone a as it is, to compare.
- **Upload.** The Worker gives each recording an id from its own clock, so the robot does not need the time. Each piece is one HTTPS PUT on a new connection. When the TCP send buffer (16 KB) is full, the write waits and tries again; it fails only after 10 s with no progress. A piece that fails is tried again, with a longer wait each time, up to 30 s.
- **Orange face.** `rec_behind()` is true while it records and the last request failed, or there is no id from the Worker after 5 s, or 2 pieces or more wait (20 s behind). It does not stay orange once it catches up: the seconds dropped are in `GET /rec`.
- **Gaps.** If all 6 pieces wait for the network, new sound is dropped, but its piece number is used anyway. The Worker puts silence where a piece never came, so the times after a gap stay right.
- **Limits.** A recording stops after 3 hours. The Worker deletes everything after 30 days.

## What we found on the way

| Try | Result |
| --- | --- |
| Upload task made with `xTaskCreate`, 8 KB stack | It never ran. With the echo canceller, the biggest free block of internal RAM is about 7 KB, and the failure was silent |
| **Its stack in PSRAM** (`xTaskCreatePinnedToCoreWithCaps`) | Works. Such a task can't delete itself, so it is made once and waits for each recording |
| The free pieces queue, empty at the start | All sound was dropped from the first second. Now it starts with all 6 pieces |
| One retry at the first piece of every recording | The connection left after the POST for the id was dead, and a failed read did not close it. Now any failed request closes it, and the first retry goes at once |
| On the second home network, the face often orange: about one retry per piece | Telemetry in `GET /rec` showed two causes. **1.** Writes that stopped with nothing written and no socket error: mbedtls says WANT_WRITE when the TCP send buffer is full, esp-tls stops, and `esp_http_client_write` returns 0 or less. Not an error: we now wait 20 ms and write the same bytes again. **2.** ECONNRESET on kept connections idle 6.6 to 6.9 s: the server closes them, and a piece comes every 10 s. Now each piece has its own connection |
| Wi-Fi power save off (`POST /rec?ps=0`) | No change: 101 KB/s on average both ways, three times what the sound needs (32 KB/s) |
| **All three fixes, 120 s on the second home network** | **0 retries, 0 failures, always on time.** 40 waits for the TCP buffer, the longest 53 ms |
| `esp-aes: Failed to allocate memory` in the log | Hardware AES copies each TLS record from PSRAM into a DMA buffer in internal RAM (up to 8 KB). While it records, the biggest free block is about 7 KB, so the copy often failed. Most of the "waits for the TCP buffer" above were these failures, not WANT_WRITE: the retry after 20 ms caught them the same way |
| Software AES (`CONFIG_MBEDTLS_HARDWARE_AES=n`) | Recording: errors 24 → 0, uploads 59 → 77 KB/s. **But it broke talks**: the echo canceller took 15 ms per frame instead of 10, real time fell to 274 of 312 frames, internal RAM to 1 KB, and the robot said "me he perdido". Undone |
| Outgoing TLS records of 2 KB instead of 4 KB | Still about 24 errors in 2 min. Undone |
| TCP send buffer back to the default (5760) | Uploads 91 → 78 KB/s. Lowest internal RAM in a talk: 1 KB (3 KB with 16 KB), so the buffer is not what fills internal RAM in talks. Kept at 16 KB |
| **Now** | TLS as in 12-talk. The AES errors stay while it records, but the write retry catches them: 0 failed pieces, about 90 KB/s near the router |

## Build

```bash
WIFI_ENV=../../../m5stack/wifi.env scripts/build.sh examples/stackchan/14-record
```

Output: `out/stackchan-14-record-esp32s3.bin` (merged image) and `examples/stackchan/14-record/build/record.bin` (app image for updates over Wi-Fi).

## Run it

```bash
curl -H "X-Token: <token>" --data-binary @examples/stackchan/14-record/build/record.bin http://stackchan.local/update
```

Then tap the right eye, or:

```bash
curl -X POST -H "X-Token: <token>" "http://stackchan.local/rec?start=1"
curl -X POST -H "X-Token: <token>" "http://stackchan.local/rec?stop=1"
```

| Endpoint | What |
| --- | --- |
| `GET /rec` | State, uploads on time or behind, id, seconds, pieces sent, pieces waiting, retries, seconds dropped, the last error. Then Wi-Fi signal and power save, upload speed (last, slowest, average), waits for the TCP buffer, and the last 12 failures with their step, bytes sent, errno, time, idle time and signal |
| `POST /rec?start=1`, `?stop=1` | Start or stop a recording |
| `POST /rec?gain=36&hpf=1` | Microphone gain while it records (dB, 30 in talks) and the 90 Hz high-pass. Only while it does not record |
| `POST /rec?ps=0\|1` | Wi-Fi power save off, or min (the default). It made no difference to uploads |
| `POST /rec?src=clean\|mic` | Record the cleaned sound (default) or microphone a as it is. Only while it does not record |
| `GET /talk`, `POST /talk`, `GET /face`, `POST /face` | As in 12-talk |

## Checked on the robot

On 2026-11-07, on USB power, with the Mac's speaker nearby playing 4 clips from Google FLEURS (Spanish, 52 s):

- 63 s recorded, 7 pieces uploaded, 0 s dropped, 1 retry (the dead connection in the table above, fixed since).
- The WAV from the Worker went through the Python pipeline in [tools/transcribe](../../../tools/transcribe). WER 1.7 % on 117 words. The only errors were "del clima" for "de clima" and "Horavec" for "Oravec".
- Internal RAM while it records: about 12 KB free, lowest 7 KB (13 KB in 12-talk's talks). It is tight: watch it if you add features.
- Internal RAM in a talk fell to 1 to 3 KB at some point (lowest seen). Talks still keep real time. The likely moment is the TLS handshake when a talk opens, which also loses about 50 frames of the echo canceller, as in 12-talk. Not measured yet.
- Real time kept: 312 of 312 frames every 10 s.

The orange face, on 2026-11-07: while it recorded, we changed the Worker's `REC_TOKEN` for 25 s, so the Worker refused the robot (403). `GET /rec` said "behind" within 25 s, with 2 pieces waiting. After we put the right token back, it was "on time" again in 20 s. All 7 pieces (66 s) reached R2, with no gaps and 0 s dropped.

Gain and high-pass, on 2026-11-07: a person talked near the robot for 80 s, recorded as A (30 dB, no filter) and then B (36 dB, high-pass):

| | A: 30 dB, no filter | B: 36 dB, high-pass |
| --- | --- | --- |
| Speech (90th percentile) | -49.8 dBFS | -44.1 dBFS |
| Noise floor (10th percentile) | -62.2 dBFS | -64.0 dBFS |
| **Signal to noise** | 12.4 dB | **19.9 dB** |
| Peak | -32.6 dBFS | -24.3 dBFS, nothing clipped |
| Energy below 90 Hz | -4.3 dB (37 % of it all) | -19.5 dB |

The filter was checked first in an exact simulation of its integer code: -26 dB at 20 Hz, -10.6 dB at 50 Hz, -3 dB at 90 Hz, 0 dB at 1 kHz, no DC, no overflow.

Not checked yet:

- A person talking in the room, and more than one person.
- A long recording (an hour), and battery use.
- `src=mic` against the cleaned sound, and recording with no echo canceller at all (for later).
