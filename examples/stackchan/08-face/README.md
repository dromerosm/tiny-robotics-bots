# StackChan 08: face and head

StackChan's face on the real screen, with a head that looks around on its own. The eyes follow where the head really points, and they blink every few seconds. From the page at `http://stackchan.local/app` you can stop it, start it, make it blink, or move the head by hand.

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
| `GET /face` | Looking around on or off, and where the head points |
| `POST /face?idle=0` or `idle=1` | Stop or start looking around. Token needed |
| `POST /face?blink=1` | Blink once. Token needed |

All of [07-head](../07-head)'s requests work too (`/head`, `/head/off`, `/i2c`, `/log`).

## The status screen

Tap the mouth on the screen. A status screen opens with the same facts as `http://stackchan.local/`, the sensors, and the **update token** on the last line. **UP** and **DOWN** at the bottom scroll, and **X** at the top right closes it. The numbers change every second. The head stays still while it is open.

The token is shown here because only someone who holds the robot can see it. `tr_ota_token()` gives it to the example; never log it or send it over the network.

## The other screens

| Tap | Screen | How to leave |
| --- | --- | --- |
| The mouth | Status (see above) | X at the top right |
| The left eye | **Audio analyzer**: 32 bars from 100 Hz to 8 kHz on a log scale, green to red, with falling peak marks. The bottom line shows the level (dBFS) and the strongest tone | X at the top right |
| The right eye | **Camera**: what it sees, 320 x 240, as fast as it can | A tap anywhere, or a touch on the head. A red X in the corner shows it |

`POST /face?view=face|status|audio|camera` (token needed) changes the screen too, and the page has buttons for it.

The analyzer reads 512 samples at 16 kHz (32 ms), with a Hann window and a 512-point FFT (`audio.c`). Bars go up at once and fall slowly, so they are easy to read.

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

- **Looking around.** Every 2 to 5 seconds the head moves to a random spot: yaw -40° to 40°, pitch 5° to 35°, in 0.9 s. Then the torque goes off. After a move from the page or `curl`, it waits 20 seconds before it looks around again.
- **The eyes.** The task reads both servos every 200 ms. The eyes move 0.5 pixel per degree of yaw and 0.6 pixel per degree of pitch, so they point where the head points. If the eyes look the opposite way to the head, flip the sign of `EYE_X_PER_DEG`.
- **Drawing.** It only draws when the eyes move or blink. Each frame is the whole screen, sent in 15 bands of 16 rows. That takes about 40 ms, so there is no flicker.

## Try next

- Turn the head toward a touch on the head sensor (Si12T, `0x68`).
- Look toward the light with the LTR553, or follow a face with the camera.
