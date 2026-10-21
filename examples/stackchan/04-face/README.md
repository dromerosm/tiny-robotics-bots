# StackChan 04: face

StackChan's face on the screen, with head moves on the two servos. You control it from the serial console. In the browser emulator you see the screen and the head next to the console.

| | |
| --- | --- |
| **Robot** | StackChan (CoreS3, ESP32-S3) |
| **Level** | Intermediate |
| **Emulator** | ✅ Browser (screen and servos) and native `esp-emu` (console only) |
| **Hardware used** | None. It only works in the emulator for now (see [tr_emu](../../../components/tr_emu)) |

## What you learn

- How to draw on a 320×240 RGB565 framebuffer
- How to redraw only when something changes, so the emulator stays fast
- How a drawing task and a console share state
- How to send commands with arguments to the firmware

## Build

```bash
scripts/build.sh examples/stackchan/04-face
```

Output: `out/stackchan-04-face-esp32s3.bin`

## Run it

- **Browser:** drop the `.bin` on https://robotics.diegoromero.es/emulator/ (chip ESP32-S3). The screen and the head servos appear above the console.
- **Native emulator:** `scripts/smoke.sh examples/stackchan/04-face`

## Commands

| Command | What it does |
| --- | --- |
| `look <pan> <tilt>` | Move the head. Pan goes from -180 to 180 (positive is right, seen from the front). Tilt goes from 5 to 85 (45 is the middle, higher looks up). Values outside these ranges are clamped. |
| `look left`, `right`, `up`, `down`, `center` | Ready-made positions |
| `center` | Look straight ahead (pan 0, tilt 45) |
| `idle on` / `idle off` | Look around on its own. It is on by default and starts 8 s after the last `look`. |
| `blink` | Blink once |

## Expected output

```
StackChan face ready. Try: look left, look 30 70, idle off
face> look 30 60
pan=30 tilt=60
```

The browser shows the eyes and mouth. They move a little toward where the head looks, and the eyes blink every few seconds.

## How it works

- `tr_emu_init()` puts a 150 KB framebuffer in internal RAM and prints `@tr emu=0x...`. The emulator page uses that address to find the screen and the servo angles. It also hides the line from the console.
- **`face_task`** runs at about 15 Hz. It handles blinks and idle glances, and redraws the face only when the angles or the eyes change.
- The **console** commands call `tr_servo_set()`, and the face follows on the next frame.

## Try next

- Add a `mood` command that changes the mouth.
- Make the head move slowly toward the target instead of jumping, the way a real servo does.
