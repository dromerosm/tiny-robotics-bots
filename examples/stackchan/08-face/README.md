# StackChan 08: face and head

StackChan's face on the real screen. At boot the head goes home (straight ahead, pitch 5°) and stays still. From the page at `http://stackchan.local/app` you can make it look around on its own, stop it, make it blink, or move the head by hand. The eyes follow where the head really points, and they blink every few seconds.

The screen runs at 20 % to save power. After a minute with no touch, on battery, the robot sleeps: the screen and the servos go off, and a touch on the screen or the head wakes it. An open view (status, audio, camera) keeps it awake. See [tr_board](../../../components/tr_board#sleep).

It brings together what 05 to 07 built: updates over Wi-Fi ([tr_ota](../../../components/tr_ota)), the screen and the servos ([tr_board](../../../components/tr_board)), and the page ([tr_web](../../../components/tr_web)).

| | |
| --- | --- |
| **Robot** | StackChan (CoreS3, ESP32-S3) |
| **Level** | Intermediate |
| **Emulator** | ✅ Native `esp-emu`, without screen and servos: it checks that Wi-Fi and the server start. [04-face](../04-face) shows the same face in the browser emulator |
| **Hardware used** | Screen, touch, two head servos, PY32 (servo power, LEDs), microphones, camera, sensors, Wi-Fi |

## What you learn

- Drawing a whole screen with no flicker: a function gives the color of each pixel, and `tr_lcd_render()` sends the screen in bands of 16 rows
- Making the face follow the real head, by reading the servos a few times a second
- Moving on your own, but safely: slow glances in a small range, torque off at rest, and a pause when someone moves the head from the page

## Build

```bash
WIFI_ENV=../../../m5stack/wifi.env scripts/build.sh examples/stackchan/08-face
```

Output: `out/stackchan-08-face-esp32s3.bin` (merged image) and `examples/stackchan/08-face/build/face.bin` (app image for updates over Wi-Fi).

## Run it

Send it over Wi-Fi to a robot that runs another `tr_ota` example:

```bash
curl -H "X-Token: <token>" --data-binary @examples/stackchan/08-face/build/face.bin http://stackchan.local/update
```

Then open `http://stackchan.local/` in a browser. It goes to the page, which has a **Face** card.

| Request | What it does |
| --- | --- |
| `GET /face` | The views, looking around on or off, where the head points, and the sound direction |
| `POST /face?idle=0` or `idle=1` | Stop or start looking around. Token needed |
| `POST /face?blink=1` | Blink once. Token needed |

All of [07-head](../07-head)'s requests work too (`/head`, `/head/off`, `/i2c`, `/log`).

## The status screen

Tap the mouth on the screen. The status screen of [tr_web](../../../components/tr_web#status-screen) opens, one section per page: Battery (with the time left and the sleep state), Robot, Network (with the **update token**), Touch and motion, and Room. **PREV** and **NEXT** at the bottom change the section, and a tap on the top bar closes it. The numbers change every second. The head stays still while it is open.

The token is shown here because only someone who holds the robot can see it. `tr_ota_token()` gives it to the example; never log it or send it over the network.

## The other screens

| Tap | Screen | How to leave |
| --- | --- | --- |
| The mouth | Status (see above) | A tap on the top bar |
| The left eye | **Audio analyzer**: 32 bars from 100 Hz to 8 kHz on a log scale, green to red, with falling peak marks. The bottom line shows the level (dBFS), the strongest tone and the sound direction (experimental, see below) | A tap on the top bar, or a touch on the head |
| The right eye | **Camera**: what it sees, 320 x 240, as fast as it can | A tap anywhere, or a touch on the head. A red X in the corner shows it |

`POST /face?view=face|status|audio|camera` (token needed) changes the screen too, and the page has buttons for it.

The analyzer reads 512 samples at 16 kHz (32 ms), with a Hann window and a 512-point FFT (`audio.c`). Bars go up at once and fall slowly, so they are easy to read.

## Sound direction (experimental)

The analyzer also tries to tell where a sound comes from, with the two microphones. It shows "< 27", "front" or "35 >" on its bottom line, and `GET /face` shows a `sound:` line. `POST /face?calibrate=1` (or the page's **Calibrate sound** button) measures the offset between the two channels from the next 5 clear sounds straight in front, and keeps it in NVS.

How: both microphones in stereo at 16 kHz. When a sound rises 10 dB above the room's background, the example cross-correlates the two channels over 8 ms (±3 samples, refined with a parabola) and turns the delay into an angle: angle = asin(lag / 1.4). Every sound that passes the level gate writes one line to `/log`, also when it is dropped.

**It does not work reliably on our robot.** What we measured on 2026-10-31:

- The two microphone holes are side by side on the front, about 30 mm apart (measured by hand). So the largest delay is 87 µs: 1.4 samples at 16 kHz.
- Claps on the left gave a steady -0.6 samples after calibration (about -27°), 9 out of 9.
- Claps on the right gave 0, the same as claps in front. So the right side is not told apart.
- The loudest clap (-13 dBFS) gave the two channels as mirror images (match -0.99): clipping, or one microphone with its polarity inverted.
- The start of a sound is nearly always found at the first sample of the frame, so the 8 ms window does not really leave out the room's echoes yet.
- At 48 kHz (MCLK 12.3 MHz on G0) the robot stopped answering on the network for about 90 seconds while the microphones ran, twice. At 16 kHz this did not happen. So the example captures at 16 kHz.

Open lines are in the StackChan to-do list: check the polarity and wiring of the two microphones in the schematic, find the start of a sound across frames, and test with a steady tone.

## Expected output

On our robot, on 2026-10-31, a few seconds apart:

```
$ curl http://stackchan.local/face
looking around: on
head: yaw 16.9, pitch 12.2 degrees
$ curl http://stackchan.local/face
looking around: on
head: yaw -35.9, pitch 8.4 degrees
```

## How it works

- **At boot.** The head moves home in 1 s (yaw 0°, pitch 5°), then the torque goes off. Looking around is off until you turn it on.
- **Looking around.** Every 2 to 5 seconds the head moves to a random spot: yaw -40° to 40°, pitch 5° to 35°, in 0.9 s. Then the torque goes off. After a move from the page or `curl`, it waits 20 seconds before it looks around again.
- **The eyes.** The task reads both servos every 200 ms. The eyes move 0.5 pixel per degree of yaw and 0.6 pixel per degree of pitch, so they point where the head points. If the eyes look the opposite way to the head, flip the sign of `EYE_X_PER_DEG`.
- **Sleep.** When the robot sleeps, `tr_sleep_asleep()` is true and the face task waits in `tr_sleep_wait_awake()`. When it wakes, the servos get their power back if they had it, and the task draws the face again.
- **Drawing.** It only draws when the eyes move or blink. Each frame is the whole screen, sent in 15 bands of 16 rows. That takes about 40 ms, so there is no flicker.

## Try next

- Turn the head toward a touch on the head sensor (Si12T, `0x68`).
- Look toward the light with the LTR553, or follow a face with the camera.
