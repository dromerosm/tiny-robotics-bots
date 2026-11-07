# StackChan 14: record the room

12-talk's spoken conversation, plus a recording mode. Tap the robot's right eye and it records the room: the face turns red, and the sound goes to our Worker in pieces of 10 s. Tap the eye again to stop. The face stays pale red until the last piece is uploaded.

The face turns orange while the uploads fall behind: no network, or the Worker fails. It turns red again once they catch up. Red means "I record", orange means "I record, but the sound is not reaching R2 now". Later, the Mac app [Scribe](../../../tools/scribe) turns the recording into text and says who spoke when, all on the Mac.

The face, the audio, the echo canceller and the talk are 12-talk's as of commit `d193c00`. There is no talk while it records: a touch on the head does nothing.

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

- **The sound.** talk.c already runs the microphones and the echo canceller all the time. With no talk, the speaker plays silence, so the cleaned sound is simply the average of both microphones, at 16 kHz. The recorder reads it through `talk_tap()`, a small addition to 12-talk's talk.c. `POST /rec?src=mic` records microphone a as it is, to compare.
- **Upload.** The Worker gives each recording an id from its own clock, so the robot does not need the time. Each piece is one HTTPS PUT, on a connection that stays open. A piece that fails is tried again, with a longer wait each time, up to 30 s.
- **Orange face.** `rec_behind()` is true while it records and the last request failed, or there is no id from the Worker after 5 s, or 2 pieces or more wait (20 s behind). It does not stay orange once it catches up: the seconds dropped are in `GET /rec`.
- **Gaps.** If all 6 pieces wait for the network, new sound is dropped, but its piece number is used anyway. The Worker puts silence where a piece never came, so the times after a gap stay right.
- **Limits.** A recording stops after 3 hours. The Worker deletes everything after 30 days.

## What we found on the way

| Try | Result |
| --- | --- |
| Upload task made with `xTaskCreate`, 8 KB stack | It never ran. With the echo canceller, the biggest free block of internal RAM is about 7 KB, and the failure was silent |
| **Its stack in PSRAM** (`xTaskCreatePinnedToCoreWithCaps`) | Works. Such a task can't delete itself, so it is made once and waits for each recording |
| The free pieces queue, empty at the start | All sound was dropped from the first second. Now it starts with all 6 pieces |
| One retry at the first piece of every recording | The connection left after the POST for the id was dead, and a failed read did not close it. Now any failed request closes it, and the first retry goes at once: 0 retries since |

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
| `GET /rec` | State, uploads on time or behind, id, seconds, pieces sent, pieces waiting, retries, seconds dropped, the last error |
| `POST /rec?start=1`, `?stop=1` | Start or stop a recording |
| `POST /rec?src=clean\|mic` | Record the cleaned sound (default) or microphone a as it is. Only while it does not record |
| `GET /talk`, `POST /talk`, `GET /face`, `POST /face` | As in 12-talk |

## Checked on the robot

On 2026-11-07, on USB power, with the Mac's speaker nearby playing 4 clips from Google FLEURS (Spanish, 52 s):

- 63 s recorded, 7 pieces uploaded, 0 s dropped, 1 retry (the dead connection in the table above, fixed since).
- The WAV from the Worker went through the Python pipeline in [tools/transcribe](../../../tools/transcribe). WER 1.7 % on 117 words. The only errors were "del clima" for "de clima" and "Horavec" for "Oravec".
- Internal RAM while it records: about 14 KB free, lowest 8 KB (13 KB in 12-talk's talks).
- Real time kept: 312 of 312 frames every 10 s.

The orange face, on 2026-11-07: while it recorded, we changed the Worker's `REC_TOKEN` for 25 s, so the Worker refused the robot (403). `GET /rec` said "behind" within 25 s, with 2 pieces waiting. After we put the right token back, it was "on time" again in 20 s. All 7 pieces (66 s) reached R2, with no gaps and 0 s dropped.

Not checked yet:

- A person talking in the room, and more than one person.
- A long recording (an hour), and battery use.
- `src=mic` against the cleaned sound.
