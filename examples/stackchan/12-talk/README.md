# StackChan 12: talk with the robot

A spoken conversation with OpenAI's GPT-Live (`gpt-live-1`, voice `vesper`). It is full duplex: the microphones stay open while the robot speaks, so you can cut it off. Echo cancelling runs on both microphones, so the robot does not hear itself. When it needs to look something up, GPT-Live hands the task to `gpt-6-luna`, which can search the web.

Touch the head to start a talk, and touch it again to end it. A talk also ends by itself after 5 minutes. The face shows where the talk is: white eyes when it is off, yellow while it connects (3 to 4 s), and blue once the robot listens. While the robot speaks, its mouth opens with the loudness of its voice.

The face is the one from [09-wifi](../09-wifi), without the Wi-Fi list. The head never moves, and there is no camera and no face model. Motors make noise in the microphones, and the echo canceller needs core 1 and the internal RAM.

| | |
| --- | --- |
| **Robot** | StackChan (CoreS3, ESP32-S3) |
| **Level** | Advanced |
| **Emulator** | ✅ Native `esp-emu`, without screen and audio: it checks that Wi-Fi and the server start |
| **Hardware used** | Speaker (AW88298), both microphones (ES7210), screen, touch, head touch |
| **Also needs** | The Worker in [workers/talk-key](../../../workers/talk-key), and `TALK_URL` and `TALK_TOKEN` in `wifi.env` |

## How it works

```
speaker <- audio task (core 0) -> 4 inputs of the ES7210
                |  mic a, mic b, and what we played, delayed 79 samples
                v
           aec task (core 1): esp-sr echo canceller on both mics, then their average
                |
                v
           send task: every 40 ms, base64, WebSocket
                                             |
  robot's voice <- play buffer <- WebSocket <- our Worker <- GPT-Live (gpt-live-1)
                                                                |  hands off tasks
                                                                v
                                                     gpt-6-luna, web search
```

- **The Worker.** GPT-Live has no short-lived keys for devices. So the robot opens a WebSocket to our Worker with its own token, and the Worker opens the session with OpenAI and passes the messages both ways. The API key never leaves the Worker, and the Worker sets the model, the voice, the instructions and the delegation.
- **Audio.** One I2S bus for both ways, at 16 kHz: the speaker as 32-bit stereo, the inputs as four 16-bit TDM slots (`tr_audio` in `tr_board`). Inputs 0 and 2 are the microphones. Input 1 carries the amplifier's output, and input 3 is empty. GPT-Live also takes and sends 16 kHz, so there is no rate conversion.
- **Echo cancelling.** We use esp-sr's echo canceller (AEC) directly, in full duplex mode, on both microphones at once. The reference is what we played, delayed so it lines up with the microphones.
- **Interruptions.** GPT-Live itself stops when you talk over it. Its voice comes in real time, silence too, so the robot holds only a short buffer: 200 ms after the buffer runs empty, and never more than 500 ms.

## What we found on the way

| Try | Result |
| --- | --- |
| esp-sr's whole audio front end (AFE), two microphones, no BSS | The output stayed empty: a fixed pattern, never audio |
| AFE with BSS (its two-microphone separation) | Real audio, but 65 to 76 ms of work for each 64 ms of sound, and about 100 ms of delay |
| Input 1 as the reference | Only about 8 dB of echo removed |
| What we played as the reference, not lined up | 2 to 6 dB |
| What we played, lined up (the tone test measures the delay) | 23 to 35 dB with the AFE |
| **The AEC alone on both microphones, filter 4, in internal RAM** | **40 to 58 dB, 12 ms per 32 ms frame (a third of core 1)** |
| The same with filter 2 | 6 dB |
| The same with its memory in PSRAM | 28 ms per 32 ms frame: too close to the limit |
| Normal or aggressive non-linear stage | The same echo removed (46 to 53 dB). Normal cuts less of your voice when you talk over the robot |
| Speaker volume 45 or less | 54 to 58 dB in the noise test. From 48 up the microphones get near full scale, and only 17 to 25 dB |

Other fixes:

- **Write and read in turn.** The I2S buffers hold 512 frames each way. One whole 1024-sample write and then one read lost sound, so the loop fell behind.
- **No `double` in the sample loops.** The ESP32-S3 has no hardware for `double`, and those loops took about 20 ms per frame. They use integers now.
- **Our Worker's certificate.** Its chain ends in GTS Root R4, signed by GlobalSign Root CA, which is not in ESP-IDF's bundle. `CONFIG_MBEDTLS_CERTIFICATE_BUNDLE_CROSS_SIGNED_VERIFY` fixes it.
- **Internal RAM.** The AEC takes about 90 KB of it. With that, TLS ran out of memory (`esp-aes: Failed to allocate memory`). TLS, Wi-Fi and lwIP buffers now go to PSRAM, and so do most allocations above 2 KB (`CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL=2048`).
- **One stroke, many touches.** The head sensor gave about 12 touches in a second. The first one opened the talk and the next ones closed it. Now, after a touch, the next 3 s do nothing, and a touch while it connects does not end the talk.
- **Restart at the end of a talk.** The send task used the WebSocket while the session task freed it. A lock fixes it.
- **The Realtime API first.** We started with `gpt-realtime-2.1`. Its voice came much faster than real time, so after an interruption seconds of the old answer still arrived and played. GPT-Live sends in real time, opens faster (3 to 4 s instead of 6) and hands off to `gpt-6-luna` by itself.

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
| `POST /talk?delay=N`, `?volume=N`, `?nlp=0\|1\|2`, `?send=mic` | Reference delay, speaker volume (60 at boot), the AEC's non-linear stage, or send microphone a with no echo cancelling, to compare |

## Checked on the robot

On 2026-11-02, on USB power:

- Real time: 312 of 312 frames every 10 s. AEC 12 ms per 32 ms frame, about 34 % of core 1.
- Talks with a person, in Spanish. The first words came 0.5 to 1.3 s after the person stopped. Questions about today's news went to `gpt-6-luna`: the robot said "Un momento, lo reviso", and the answer came 10 to 20 s later.
- While the robot spoke, the cleaned sound stayed at the level of a quiet room. It never cut itself off.
- Cutting the robot off works. The person found it a bit slow before we sent every 40 ms (it was 100 ms) and moved to the normal non-linear stage.
- Internal RAM during a talk: about 18 KB free, lowest 13 KB.
- Volume 60 with speech: the robot never cut itself off, although the noise test says 60 is too loud for the echo canceller. Speech is quieter than the test noise.
- A touch ends the talk with no restart.
- The robot's voice comes at the real rate: 26.0 s received in 27.3 s, with a fixed delay of about 1.2 s at the start. The play buffer still runs empty now and then (25 times in 100 s, `gaps` in `GET /talk`), nearly always in silence. The person heard no cuts. A bigger buffer would make cutting the robot off slower.

Not checked yet:

- About 45 frames (1.5 s) are lost while the session opens, never after. The TLS handshake likely takes the time.
- Battery current, and the cost of a 5-minute talk.
