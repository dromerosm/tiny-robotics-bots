# StackChan 15: follow a face, then talk

[14-record](../14-record), plus a head that follows your face while nobody talks to the robot. The face tracking comes from [11-track](../11-track) and the talk from [12-talk](../12-talk). Here they are in the same firmware.

They can't run at the same time. Motors make noise in the microphones. The face finder and the echo canceller need the same core, and both need internal RAM. So they take turns:

- **No talk, no recording.** The camera and the face model run. The head follows the biggest face, and looks around when it sees none. The eyes and the mouth lead the head.
- **A talk or a recording starts.** The finder stops, the camera and the model give their memory back, and the servos lose their power. The head stays where it was: on you. Then the echo canceller starts to work.
- **It ends.** The echo canceller rests, and the head follows again about 0.6 s later.

The speaker is also louder than in 14-record: volume 75 instead of 60, with less microphone gain (21 dB instead of 30).

| | |
| --- | --- |
| **Robot** | StackChan (CoreS3, ESP32-S3) |
| **Level** | Advanced |
| **Emulator** | ✅ Native `esp-emu`, without screen, camera, servos and audio: it checks that Wi-Fi and the server start |
| **Hardware used** | Camera (GC0308), two head servos, both microphones (ES7210), speaker (AW88298), 8 MB PSRAM, screen, touch, head touch |
| **Also needs** | The same as 14-record: the Workers in [workers/talk-key](../../../workers/talk-key) and [workers/rec](../../../workers/rec), and their URLs and tokens in `wifi.env` |

## What you learn

- How two features that don't fit together can share a small chip by taking turns
- Why internal RAM, not total RAM, is the limit on an ESP32-S3 with PSRAM, and how to measure it
- What can move to PSRAM (task stacks, small allocations) and what can't (the echo canceller, DMA buffers)
- Why a louder speaker needs less microphone gain

## How it works

```
no talk:   camera -> finder task (core 1): esp-dl face model -> where the face is
                                                |
           face task: track_step() -> head servos, eyes and mouth on the screen

a talk:    talk.c calls the quiet hook -> track_quiet(): finder stops, camera and
           model freed, servo power off -> the echo canceller works on core 1
```

- **track.c** is 11-track's finder and head moves, without the record of finds and the glances. One task owns the camera and the model. It starts them when the face task wants to follow, and stops and frees them when `track_quiet()` asks.
- **talk.c** calls a hook before each talk, test or recording. The hook is `track_quiet()`. It returns only when the camera is off and the servos have no power, so no motor is heard in the talk.
- **The echo canceller** takes its memory once, at boot, and keeps it. It works only during a talk. With no talk, the "cleaned" sound is the average of both microphones as they are, and the recorder gets that.
- **One microphone.** The echo canceller works on microphone a only. On both microphones it takes 136 KB of internal RAM, and on one 92 KB. Those 44 KB are what the camera and the model need.
- **The camera view.** Tap the left eye: the screen shows the camera, 320 x 240, with a box on each face the finder sees (green for the one the head follows) and the center of the frame. It is there to check that following works. With it open the finder works at full speed. It also works with following off: the camera runs, and the head stays still. A tap closes it. A talk, a recording or sleep closes it too, because the camera stops.
- **The page** at `http://stackchan.local/app` makes its Face buttons from the lines of `GET /face`: **Start a talk**, **End the talk**, **Record**, **Stop recording**, **Follow faces**, **Don't follow**, **Blink**, and the views **Face**, **Status** and **Faces** (the camera view).
- **`POST /face?follow=0`** stops following. The camera and the model give their memory back, unless the camera view is open. `follow=1` starts it again.
- **Sleep.** The robot does not sleep by itself, but "Sleep now" on the page (`POST /sleep?now=1`) works. The finder, the camera, the servos and the screen stop, as in 11-track. The speaker, the microphones and I2S stop too (`tr_audio_pause()`), so the CPU can use light sleep. A touch wakes the robot. So does a talk or a recording started from the page.

## What we found on the way

The problem was internal RAM. This is what we tried, in order:

| Try | Result |
| --- | --- |
| Free the echo canceller after each talk, and make it again for the next one | The robot restarted at the first talk. After a while the biggest free block of internal RAM is 34 KB, the new echo canceller does not fit, and esp-sr crashes instead of returning an error |
| Keep the echo canceller from boot (136 KB), follow in what is left | 23 KB free with no camera, 6 KB with it, lowest 0 KB. Too little |
| The camera writes its frames straight into PSRAM (`CONFIG_CAMERA_PSRAM_DMA`) | No 32 KB buffer in internal RAM, but the camera timed out on most frames. Undone |
| Allocations above 256 bytes go to PSRAM (2 KB in 14-record) | Only a few KB more. Kept |
| Stacks of the face, finder, send and talk tasks in PSRAM | 22 KB of stacks out of internal RAM. Kept |
| **The echo canceller on one microphone** | **92 KB instead of 136 KB. Now it all fits** |
| The camera's buffer in internal RAM, 8 KB instead of 32 KB | No timeouts, and the finder is as fast as in 11-track. Kept |

### Sleep

At first, "asleep" used more power than 14-record awake: 146 mA. Only the screen and the servos were off. The camera, the finder and the audio kept running, and the finder turned the servos on again for each look.

| Try | Result |
| --- | --- |
| A sleep hook stops the finder, the audio task stops the audio, light sleep on (`CONFIG_PM_ENABLE`) | 45 to 55 mA asleep |
| Light sleep with the CPU powered down (the default) | It takes about 18 KB of internal RAM to save the CPU's state: 19 KB free while following, not 35 KB. Off (`CONFIG_PM_POWER_DOWN_CPU_IN_LIGHT_SLEEP=n`). The current asleep stayed in the same range |
| The face task started the camera at the moment sleep cut its power | The camera failed, and following stayed off. `track_step()` now checks again after it takes the lock, and a camera that worked before gets another try after 10 s |
| The reference delay of the echo canceller (79 samples in 12-talk) | After each wake the time from the speaker to the microphones is different: 13 to 140 samples. With 79 and a real delay of 13, only 8 dB of echo went away. Now the delay is 0, so the reference always comes first: 53 to 72 dB once the filter has learned again |

### The face model did not load

esp-sr 2.6.1 gives esp-dl a list of the operations its own models need, and then esp-dl builds only what is on a list. The face model needs others, so it failed to load: `Do not support Concat, please implement and register it first`. `main/face-req.yml` adds the face model's list.

### The speaker and the microphones

More volume alone made things worse. The microphones clip on the robot's own voice, and then the echo canceller can't remove it. `POST /talk?test=aec` plays noise bursts and says how much echo is gone:

| Volume | Microphone gain | Microphone a (rms) | Echo removed |
| --- | --- | --- | --- |
| 60 | 30 dB (14-record) | 15300 | 22 to 33 dB |
| 75 | 24 dB | 15300 | 22 dB |
| **75** | **21 dB** | **10500** | **66 dB** |
| 80 | 21 dB | 20300 | 13 dB |
| 85 | 18 dB | 10300 | 69 dB |
| 100 | 18 dB | 16000 | 21 dB |
| 100 | 12 dB | 8700 | 56 dB |

The echo canceller works while microphone a stays below about 11000 rms. Above 15000 it does not. Each 10 points of volume are about 4 dB. So 14-record's own setting was already at the edge. This example uses 75 and 21 dB. `POST /talk?volume=85&gain=18` tries a louder pair without a new build.

## Build

```bash
WIFI_ENV=../../../m5stack/wifi.env scripts/build.sh examples/stackchan/15-follow
```

Output: `out/stackchan-15-follow-esp32s3.bin` (merged image) and `examples/stackchan/15-follow/build/follow.bin` (app image for updates over Wi-Fi).

## Run it

```bash
curl -H "X-Token: <token>" --data-binary @examples/stackchan/15-follow/build/follow.bin http://stackchan.local/update
```

Stand in front of the robot: the head turns to you. Tap the left eye to see what the camera and the finder see. Touch the head to talk, and touch it again to end the talk. Tap the right eye to record. The page has a button for each of these.

| Endpoint | What |
| --- | --- |
| `GET /face` | The view on the screen, the talk, the recording, following on or off, and the finder: faces, time per find, finds per second, and frames per second on the screen while the camera view is open |
| `POST /face?view=face\|status\|track` | Change the screen. `track` is the camera view |
| `POST /face?follow=0\|1` | Stop or start following faces |
| `POST /talk?volume=75` | Speaker volume, 0 to 100 |
| `POST /talk?gain=21` | Microphone gain in dB, to try. A recording sets it back to 21 when it ends |
| `GET /talk` | As in 12-talk, plus "echo canceller: at work" or "resting" |
| `POST /sleep?now=1`, `?wake=1` | Sleep now, or wake (tr_web). The page has buttons for both |
| The rest of `/talk`, `/rec` and `/face` | As in 14-record |

## Checked on the robot

On 2026-11-11, on battery:

| | Internal RAM free | Notes |
| --- | --- | --- |
| Following, no face | 34 KB (biggest block 13 KB) | 2.0 finds per second, 82 to 88 ms per find, no camera timeouts |
| In a talk | 54 KB (biggest block 31 KB) | 14-record had 18 KB here |
| Lowest since boot | 19 KB | 14-record: 1 to 5 KB |

- A talk of 17 s from `POST /talk?start=1`: the session opened in 3.7 s, 312 of 312 frames every 10 s, 0 frames lost by the echo canceller, 6.5 ms per frame (10.5 ms on two microphones).
- The camera and the model stop in 7 ms and start again in 0.6 s. Three stops and starts in a row gave the same free memory each time.
- After the talk, the finder was back by itself.
- Echo removed at the new volume and gain, three runs after boot: 55, 47 and 33 dB. After a sleep and a wake: 9 dB in the first run, while the filter learns again, then 56 and 62 dB.
- Battery current, the robot's own 20 s average, at 3.6 to 3.7 V: 191 mA awake and following with no face in sight, 45 to 55 mA asleep (three sleeps of 130 to 150 s).
- Awake again in about 0.2 s. Four sleeps and wakes in a row, each just after an echo test: the finder came back every time.
- A talk started from the page while the robot slept: it woke, and the talk kept real time (312 of 312 frames).
- Every request the page's buttons send answered 200 with the token, and 403 without it. A recording of 14 s: 2 pieces uploaded, 0 retries, 0 s dropped.
- The camera view from the page: 7 to 10 finds per second with it open, about 100 ms per find, and 6 to 10 frames per second on the screen (`GET /face` says both). 33 to 35 KB of internal RAM free. A talk and a sleep closed it. With following off it opened too, and the servos stayed without power.

Not checked yet:

- A person in front of the camera: that the head follows, and that it stays on the person during the talk.
- A real talk at the new gain. The microphones give 9 dB less, so check that the robot still understands you from where you sit.
- The sound of the robot's own voice at volume 75, by ear, and 85 with 18 dB.
- How a recording sounds, and its text. The recorder now gets both microphones with no echo canceller in between.
- The camera view by eye, and the boxes on a real face. The numbers say frames reach the screen, but nobody looked at it yet.
- A long run, and battery use in a talk. Awake it uses more than 14-record (191 mA against about 130 mA): the camera, the face model and the head moves.
- Why asleep is 45 to 55 mA and not 11-track's 32 mA. The amplifier and the ES7210 only go to standby.

## Try next

- Follow with the eyes only during a talk. The camera would have to run next to the echo canceller: check the RAM first.
- Free more internal RAM, and try the echo canceller on both microphones again.
