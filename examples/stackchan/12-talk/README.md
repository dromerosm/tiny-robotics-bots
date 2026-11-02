# StackChan 12: talk with the robot

A spoken conversation with OpenAI's Realtime API. It is full duplex: the microphones stay open while the robot speaks, so you can cut it off. Echo cancelling runs on both microphones, so the robot does not hear itself.

Touch the head to start a talk, and touch it again to end it. A talk also ends by itself after 5 minutes. While the robot speaks, its mouth opens with the loudness of its voice.

The face is the one from [09-wifi](../09-wifi), without the Wi-Fi list. The head never moves, and there is no camera and no face model. Motors make noise in the microphones, and the echo canceller needs core 1 and the internal RAM.

| | |
| --- | --- |
| **Robot** | StackChan (CoreS3, ESP32-S3) |
| **Level** | Advanced |
| **Emulator** | ✅ Native `esp-emu`, without screen and audio: it checks that Wi-Fi and the server start |
| **Hardware used** | Speaker (AW88298), both microphones (ES7210), screen, touch, head touch |
| **Also needs** | The key Worker in [workers/talk-key](../../../workers/talk-key), and `TALK_URL` and `TALK_TOKEN` in `wifi.env` |

## How it works

```
speaker <- audio task (core 0) -> 4 inputs of the ES7210
                |  mic a, mic b, and what we played, delayed 79 samples
                v
           aec task (core 1): esp-sr echo canceller on both mics, then their average
                |
                v
           send task: every 100 ms, 16 -> 24 kHz, base64, WebSocket
                                                         |
           robot's voice <- 24 -> 16 kHz <- WebSocket <- Realtime API
```

- **The session key.** The robot asks our Worker for a key that works for 60 s (it sends its own token). Then it opens a WebSocket to `api.openai.com` with that key. The real API key never leaves the Worker.
- **Audio.** One I2S bus for both ways, at 16 kHz: the speaker as 32-bit stereo, the inputs as four 16-bit TDM slots (`tr_audio` in `tr_board`). Inputs 0 and 2 are the microphones. Input 1 carries the amplifier's output, and input 3 is empty.
- **Echo cancelling.** We use esp-sr's echo canceller (AEC) directly, in full duplex mode with its strongest non-linear stage, on both microphones at once. The reference is what we played, delayed so it lines up with the microphones.
- **Interruptions.** The server finds when you start talking. If the robot is speaking then, the server stops the answer and the robot drops what is left in its play buffer.

## What we found on the way

| Try | Result |
| --- | --- |
| esp-sr's whole audio front end (AFE), two microphones, no BSS | The output stayed empty: a fixed pattern, never audio |
| AFE with BSS (its two-microphone separation) | Real audio, but 65 to 76 ms of work for each 64 ms of sound, and about 100 ms of delay |
| Input 1 as the reference | Only about 8 dB of echo removed |
| What we played as the reference, not lined up | 2 to 6 dB |
| What we played, lined up (the tone test measures the delay) | 23 to 35 dB with the AFE |
| **The AEC alone on both microphones, filter 4, in internal RAM** | **40 to 53 dB, 12 ms per 32 ms frame (a third of core 1)** |
| The same with filter 2 | 6 dB |
| The same with its memory in PSRAM | 28 ms per 32 ms frame: too close to the limit |

Other fixes:

- **Write and read in turn.** The I2S buffers hold 512 frames each way. One whole 1024-sample write and then one read lost sound, so the loop fell behind.
- **No `double` in the sample loops.** The ESP32-S3 has no hardware for `double`, and those loops took about 20 ms per frame. They use integers now.
- **Our Worker's certificate.** Its chain ends in GTS Root R4, signed by GlobalSign Root CA, which is not in ESP-IDF's bundle. `CONFIG_MBEDTLS_CERTIFICATE_BUNDLE_CROSS_SIGNED_VERIFY` fixes it.
- **Internal RAM.** The AEC takes about 90 KB of it. With that, TLS ran out of memory (`esp-aes: Failed to allocate memory`). TLS, Wi-Fi and lwIP buffers now go to PSRAM, and so do most allocations above 2 KB (`CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL=2048`).

## Build

```bash
WIFI_ENV=../../../m5stack/wifi.env scripts/build.sh examples/stackchan/12-talk
```

Output: `out/stackchan-12-talk-esp32s3.bin` (merged image) and `examples/stackchan/12-talk/build/talk.bin` (app image for updates over Wi-Fi).

## Run it

```bash
curl -H "X-Token: <token>" --data-binary @examples/stackchan/12-talk/build/talk.bin http://stackchan.local/update
```

Then touch the head, or:

```bash
curl -X POST -H "X-Token: <token>" "http://stackchan.local/talk?start=1"
```

| Endpoint | What |
| --- | --- |
| `GET /talk` | State, levels per input, counts, the answer delay, timing, internal RAM, CPU per task |
| `POST /talk?start=1`, `?stop=1` | Start or end a talk |
| `POST /talk?test=lanes` | A 1 kHz tone. It shows when each input hears it, and sets the reference delay |
| `POST /talk?test=aec` | 6 s of noise bursts. It shows how much echo is removed |
| `POST /talk?delay=N`, `?volume=N`, `?send=mic` | Reference delay, speaker volume (40 at boot), or send microphone a with no echo cancelling, to compare |

## Checked on the robot

On 2026-11-02, on USB power:

- Real time: 312 of 312 frames every 10 s, no lost frames while it talks. AEC 12 ms per 32 ms frame, about 34 % of core 1.
- Noise test: 40 to 53 dB of echo removed.
- A talk: the session key in 2.7 s, the session open in 6 s. The robot answered twice. Its first sound came 735 and 919 ms after the end of the speech it heard. While it spoke, the microphones heard its voice at a level of 1200 to 1950, and the cleaned sound stayed at 18 to 33, the same as a quiet room. It never cut itself off.
- Internal RAM during a talk: 19 KB free, lowest 11 KB.

Not checked yet:

- Cutting the robot off while it speaks (barge-in).
- 72 frames (about 2 s) were lost while the session opened, never after. The TLS handshake likely takes the time.
- Battery current, and the cost of a 5-minute talk.

## Try next

- Test barge-in with a person, and a whole 5-minute talk.
- Make opening faster (6 s now): ask for the key before the touch, or keep the TLS session.
