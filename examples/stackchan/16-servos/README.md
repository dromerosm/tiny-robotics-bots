# StackChan 16: the head servos from the console

Move StackChan's head with commands on the serial console. It uses the real servo driver in [tr_board](../../../components/tr_board): serial bus packets on UART1 at 1 Mbaud, and the servo power through the PY32 in the body. The same image runs on the robot and in the [browser emulator](https://robotics.diegoromero.es/emulator/). There, a virtual part plays the body, so the head moves on the page.

| | |
| --- | --- |
| **Robot** | StackChan (CoreS3, ESP32-S3) |
| **Level** | Beginner |
| **Emulator** | ✅ In the browser, with the virtual body (PY32 and both servos). Native `esp-emu` too, with a board file (see below) |
| **Hardware used** | Two FEETECH SCS0009 servos (UART1), PY32 in the body (servo power), Wi-Fi for updates |

## What you learn

- How a servo bus works: one serial line, packets with an ID, and an answer from each servo
- Servo power through an IO expander, and why the driver waits until both servos answer
- How to test a driver without the robot: a virtual part answers like the hardware

## Build

```bash
scripts/build.sh examples/stackchan/16-servos
```

Output: `out/stackchan-16-servos-esp32s3.bin` (merged image) and `examples/stackchan/16-servos/build/servos.bin` (app image for updates over Wi-Fi).

## Run it

**In the browser.** Open the [emulator](https://robotics.diegoromero.es/emulator/), keep the chip on ESP32-S3 and drop `stackchan-16-servos-esp32s3.bin`. Type `move 30 60` in the console. The head servos appear next to the console and turn in one second.

**In native esp-emu.** Build the part with `scripts/build-parts.sh` (or take `stackchan-body.wasm` from the release), put it next to this `board.json` and run:

```json
{ "parts": { "body": { "module": "stackchan-body.wasm", "pins": { "TX": 7, "RX": 6, "SDA": 12, "SCL": 11 } } } }
```

```bash
esp-emu --chip esp32s3 --firmware out/stackchan-16-servos-esp32s3.bin --cosim-board board.json
```

Without `--cosim-board` there is no body, and the example says "no servos".

**On the robot.** Send it over Wi-Fi to a robot that runs another `tr_ota` example, then open the serial console over USB:

```bash
curl -H "X-Token: <token>" --data-binary @examples/stackchan/16-servos/build/servos.bin http://stackchan.local/update
```

| Command | What it does |
| --- | --- |
| `move <yaw> <pitch> [ms]` | Move both servos, in `ms` milliseconds (1000 by default), then read them |
| `center` | Look straight ahead (yaw 0, pitch 45) |
| `nod` | Say yes, then no |
| `read` | Angle, raw position, torque, voltage and temperature of both servos |
| `power on\|off` | Servo power. A move turns it on |
| `torque on\|off` | Hold the head, or let it turn freely |

## Safety

- Nothing moves at boot. The power stays off until the first move.
- `tr_board` keeps the pitch between 5° and 85° and the yaw between -90° and 90°, so a typo can't push the pitch servo against its end stop.
- Each move takes 100 ms to 5 s.

## Expected output

In the browser emulator (esp-emu 1.0.0), on 2026-11-13:

```
I (108) tr_servo: PY32 at 0x6F: ESP_OK, version 0x41
I (108) tr_servo: servo 1 zero 460
I (108) tr_servo: servo 2 zero 620
W (318) tr_servo: side LEDs did not answer

servos: body found (PY32), power off until the first move
servos> move 30 60
I (13928) tr_servo: servos answer 194 ms after power on
power on: ESP_OK
yaw: 30.0 deg (raw 556, zero 460), torque on, still, 5.0 V, 30 C
pitch: 60.0 deg (raw 812, zero 620), torque on, still, 5.0 V, 30 C
servos> nod
nod done
servos> torque off
torque off: ESP_OK
```

The warning about the side LEDs is expected in the emulator: they need the CoreS3's 5 V switch, which the virtual body does not have.

## How it works

`app_main()` starts `tr_ota`, then `tr_servo_init()`. That finds the PY32 at I2C address 0x6F and sets up UART1 (TX GPIO 6, RX GPIO 7, 1 Mbaud). The first move turns on VM_EN, the servo power, and waits until both servos answer a read. A move is one write to the goal registers: position, time and speed. A read asks for 8 bytes from register 56: position, speed, load, voltage and temperature.

In the browser, the emulator sends every byte on UART1 to the [stackchan-body](../../../parts/stackchan-body) part, a small WebAssembly module written in Rust. The part answers like the two servos (IDs 1 and 2) and moves them in emulated time. It also answers on I2C as the PY32, so the servos only answer while VM_EN is on, as on the robot. The page reads the angles from the part and draws the head.

## Try next

- Change the `nod` steps, or add a `shake` command.
- Turn the torque off and read the servos while you turn the head by hand (on the robot).
- [04-face](../04-face) draws a face that follows the head.
