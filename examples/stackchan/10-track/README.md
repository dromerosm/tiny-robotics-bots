# StackChan 10: follow a face

[09-wifi](../09-wifi) plus a camera that finds faces, and a head that follows them. The robot looks for faces about 12 times per second on the second core. When it sees one, the head turns toward it, and the eyes and mouth move toward the person first and come back to the center as the head catches up. Following is on at boot.

Tap the left eye to check it: the screen shows what the camera sees, with a green box on the face, a blue cross at the center and the blue dead zone around it. The head keeps following while this view is open.

Everything else stays as in 09: the face, the Wi-Fi list on the right eye, the status screen on the mouth, sleep after a minute with no touch on battery ([tr_board](../../../components/tr_board#sleep)), updates over Wi-Fi ([tr_ota](../../../components/tr_ota)) and the page at `http://stackchan.local/app` ([tr_web](../../../components/tr_web)).

| | |
| --- | --- |
| **Robot** | StackChan (CoreS3, ESP32-S3) |
| **Level** | Advanced |
| **Emulator** | ✅ Native `esp-emu`, without screen, camera and servos: it checks that Wi-Fi and the server start |
| **Hardware used** | Camera (GC0308), 8 MB PSRAM, screen, touch, two head servos, PY32, sensors, Wi-Fi |

## What you learn

- Running a neural network on the ESP32-S3 with Espressif's [esp-dl](https://github.com/espressif/esp-dl) (`espressif/human_face_detect`)
- Calling a C++ library from a C example through a small wrapper (`face.cpp`, `face.h`)
- What makes the model fast or slow on this robot: the PSRAM speed matters much more than the CPU speed

## Build

```bash
WIFI_ENV=../../../m5stack/wifi.env scripts/build.sh examples/stackchan/10-track
```

Output: `out/stackchan-10-track-esp32s3.bin` (merged image) and `examples/stackchan/10-track/build/track.bin` (app image for updates over Wi-Fi). The first build downloads esp-dl and the model.

## Run it

Send it over Wi-Fi to a robot that runs another `tr_ota` example:

```bash
curl -H "X-Token: <token>" --data-binary @examples/stackchan/10-track/build/track.bin http://stackchan.local/update
```

| Tap | Screen | How to leave |
| --- | --- | --- |
| The left eye | **Faces**: the camera, 320 x 240, with a green box on the best face and yellow on up to 3 more. A red X in the corner shows how to leave | A tap anywhere, or a touch on the head |
| The right eye | **Wi-Fi** (as in 09) | A tap on the top bar |
| The mouth | **Status** (as in 09) | A tap on the top bar |

`POST /face?view=track` (token needed) opens it too, and the page has a **Faces** button. The open view keeps the robot awake.

| Request | What it does |
| --- | --- |
| `GET /face` | As in 09, plus `following:` (off, on with no face, or on with a face) and `track:` (faces, time to find them, finds and frames per second) |
| `POST /face?follow=0` or `follow=1` | Stop or start following. Token needed. The page has **Follow faces** and **Don't follow** buttons |
| `GET /track.csv` | The record of what following did (see below) |

`/log` gets a line every 10 s with the faces and the speed, `head: home (boot)` when the head goes home at boot, and a line for each head move from the page or `curl`.

## The record (GET /track.csv)

To check following later, the robot keeps the last 16384 events in PSRAM (about 20 minutes): every find with a face, the first find after the face is gone, `seen` and `lost` when following starts and stops, each head move (axis, where the servo was, the goal, the move time), and `home` at boot. A restart clears it.

The first lines of our test:

```
ms,event,faces,cx,cy,box_w,score,err_x,err_y,axis,from_deg,goal_deg,move_ms
1452,home,,,,,,,,,,,
34009,find,1,96,108,121,0.99,-0.40,-0.10,,,,
34049,seen,,,,,,,,,,,
34051,move,,,,,,,,yaw,0.6,-6.6,180
34101,find,1,117,123,111,0.61,-0.27,0.03,,,,
34114,move,,,,,,,,yaw,0.9,-3.9,150
34207,find,1,119,125,103,0.94,-0.26,0.04,,,,
```

In our test on 2026-11-01 (34 s, moving left and right, then still), it had 286 finds with a face (8.4 per second, with the camera view open) and 137 moves. The face stayed close to the center: the error was 0.09 or less half of the time (the dead zone is 0.15), and 0.28 or less 90 % of the time.

## Expected output

On our robot, on 2026-11-01, with a face about 50 cm in front:

```
I (846665) track: 1 face(s), find 88 ms, 3.6 frames/s   (before the faster screen)
I (846665) track: best: 102,65 to 199,204, score 0.89
I (855315) track: 1 face(s), find 90 ms, 3.6 frames/s
I (855315) track: best: 118,54 to 224,193, score 0.98
```

The box followed the face as it moved. The score was 0.89 to 0.99.

## Memory and speed

Measured on our robot on 2026-11-01:

| | Value |
| --- | --- |
| App size | 2.26 MB (09-wifi: 1.01 MB). esp-dl, the camera driver and the model add about 1.25 MB. 2.68 MB of the 4.94 MB app slot stay free |
| Model | MSR + MNP, int8, 190 KB in flash. It loads in 0.6 s, the first time the view opens |
| Internal RAM free | 96 KB after the model loaded (lowest 53 KB, while it loaded). The second screen band (10 KB) and the finder task's stack (6 KB) are part of this |
| PSRAM free | 7.3 MB of 8 MB. The model and the 3 camera frames (150 KB each) use about 1 MB |
| Time to find faces | 87 ms with no face, 95 to 107 ms with one (on core 1, while core 0 shows frames) |
| Finds | 11.1 per second with no face, 9.2 to 10.3 with one |
| Screen | the same frames, about 10 per second |
| Battery current with the view open | about 138 mA (average) |

What made it faster:

| Change | Time to find faces |
| --- | --- |
| First build: CPU 160 MHz, code for debugging, PSRAM 40 MHz | 130 ms |
| CPU 240 MHz and code built for speed (`CONFIG_COMPILER_OPTIMIZATION_PERF`) | 122 ms |
| PSRAM at 80 MHz (`CONFIG_SPIRAM_SPEED_80M`) | 73 ms |

The model reads its weights and data from PSRAM, so the PSRAM speed was the real limit. Espressif gives 39 ms for this model, on an ESP32-S3 at 240 MHz with octal PSRAM. The CoreS3 has quad PSRAM.

Then the rest of each frame. We timed each part on the robot:

| Part of a frame | Before | After |
| --- | --- | --- |
| Wait for the camera | 0 ms | 0 ms |
| Find faces | 75 ms | 80 ms |
| Copy the frame to the screen | 106 ms | 32 ms |
| Pauses to read the touch | 59 ms | one 10 ms tick |
| **Frames per second** | **4.1** | **8.2** |

- **The screen.** `tr_lcd` sends the screen in bands of 16 rows through a DMA buffer. With one buffer, it filled a band, sent it and waited, one band at a time. Now it has two: one goes out on SPI while the next one is filled. 32 ms is close to the limit of SPI at 40 MHz for 320 x 240 pixels (31 ms). This also helps the camera in [08-face](../08-face). It costs 10 KB more of internal DMA memory.
- **The touch.** The view read the touch 2 times with 30 ms pauses after each frame. Now it reads it once per frame at the top of the loop, as the face does. A tap still closes the view.

Then we split the work over the two cores. What matters is how often the robot finds the face, because the head will follow it. The screen is only there to check.

| | One core | Two cores, screen first | Two cores, finder first |
| --- | --- | --- | --- |
| Finds per second (no face) | 8.2 | 6.6 | **11.1** |
| Screen, frames per second | 8.2 | 13.3 | 11.1 |
| Time to find faces (no face) | 80 ms | 92 ms | 87 ms |

- **Screen first** (our first try): the face task took each camera frame and showed it, and copied one for the finder when it was free. The screen was smooth, but the finder needs a bit more than one screen frame, so it got only every second frame (13.3 / 2 = 6.6). Before that, both tasks took frames from the camera: the face task got nearly all of them, the finder found faces 0.4 times per second, and the camera timed out (`cam_hal: Failed to get frame: timeout`).
- **Finder first** (now): the finder, on core 1, is the only task that takes frames from the camera, one after the other. When the face task is free, the finder hands it the frame with its boxes through a queue of one. The face task draws the boxes, shows the frame and gives it back to the camera. When the face task is busy, the finder gives the frame back itself. So the screen never slows the finder, and the boxes on the screen are exactly the ones found in that frame, with no copy.
- **3 camera frames.** With 2, the finder often waited about 18 ms for a new frame (9.9 finds per second). With 3 (`tr_cam`, 150 KB more PSRAM) a new frame is ready when it asks: 11.1.
- **Finding is a bit slower on two cores** (80 to 87 ms): both cores, the camera and the screen use the PSRAM at the same time.
- The box follows the face and a tap closes the view (checked on the robot). For the head, about 10 finds per second is enough: a servo move takes at least 100 ms.

## Following

For each find, the error is how far the face is from the center of the frame, from -1 to 1. Inside the dead zone (0.15) nothing moves. Outside it, the head gets a new goal: where it is now, read from the servo, plus 18 degrees of yaw (12 of pitch) times the error. The move time is 25 ms per degree, between 150 and 600 ms. The servos take a new goal while they still move, so the head turns smoothly. The torque goes off 2 s after the last move.

We tried three ways on the robot:

| Way | What we saw |
| --- | --- |
| A step on top of the last goal, with every find | It found the face, but swung around it. Each find still saw the head where it was 0.1 to 0.3 s before, so the steps added up |
| One step, then a pause until the head stops (250 + 150 ms) | It centered the face, but in jumps |
| **A goal from where the head really is, with every find** | Smooth, and it centers the face. Finds made while the head turns give about the same goal, so the goals don't add up |

- **Eyes and mouth.** They move by the same error (40 pixels sideways, 25 up and down at an error of 1), halfway each frame. So they look at the person before the head arrives, and come back to the center as the error goes down. With no face for 1 s, they go back to the center.
- **Directions.** The signs (`YAW_SIGN`, `PITCH_SIGN`, `EYE_SIGN_X`) were checked on the robot: the head turns toward you, the eyes look at you. The camera image is not a mirror.
- **18 and 12 degrees** are guesses of half the camera's view, not measured. They worked on our robot.
- **Sleep.** A face in view counts as activity. With no one there for a minute, on battery, the robot sleeps as usual, and the finder stops.
- **Page moves win.** After a move from the page or `curl`, following waits 20 s, as looking around does.

## How it works

- **The model.** `face.cpp` makes one `HumanFaceDetect` (the MSR + MNP model) the first time the view opens. MSR finds candidate boxes in a smaller copy of the frame, and MNP checks each one at 48 x 48. esp-dl makes the smaller copy itself.
- **The frame.** The camera gives RGB565 with the high byte first, which is `DL_IMAGE_PIX_TYPE_RGB565BE` in esp-dl, so the frame goes to the model as it is. The face task draws the boxes into the same frame, and the frame goes to the screen with `tr_lcd_blit()`.
- **The two tasks.** See above. The finder runs while following is on or the view is open, and stops when the robot sleeps. Frames go to the screen only while the view is open.
- **The rest.** As in [09-wifi](../09-wifi).

## Try next

- Measure the camera's view angle, and use it instead of the 18 and 12 degree guesses.
- Turn the whole body toward the face when the head reaches the end of its yaw range.
- Look at the newest face when there are several, or at the biggest one (the closest).
