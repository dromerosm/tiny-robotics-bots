# StackChan 07: move the head over Wi-Fi

Read and move StackChan's two head servos from your computer or phone. The robot serves a small page, `http://stackchan.local/app`, with the status, the servos and the I2C chips, and sliders to move the head. The same actions also work with `curl`. Like every example that runs on the robot, it keeps the updates over Wi-Fi from [tr_ota](../../../components/tr_ota).

| | |
| --- | --- |
| **Robot** | StackChan (CoreS3, ESP32-S3) |
| **Level** | Intermediate |
| **Emulator** | ✅ Native `esp-emu`, without the body: it says "no servos" and goes on with Wi-Fi |
| **Hardware used** | Two FEETECH SCS0009 servos (UART1), PY32 in the body (servo power), INA226 (battery), Wi-Fi |

## What you learn

- The FEETECH SCS protocol: packets, checksums, reading and writing servo registers
- Turning on servo power through the PY32 IO expander, and why it needs split I2C reads
- Safe limits for a robot that moves: clamped angles, slow moves, torque off at rest
- Adding your own endpoints and a web page to `tr_ota`'s server

## Build

```bash
WIFI_ENV=../../../m5stack/wifi.env scripts/build.sh examples/stackchan/07-head
```

Output: `out/stackchan-07-head-esp32s3.bin` (merged image) and `examples/stackchan/07-head/build/head.bin` (app image for updates over Wi-Fi).

## Run it

Send it over Wi-Fi to a robot that runs another `tr_ota` example:

```bash
curl -H "X-Token: <token>" --data-binary @examples/stackchan/07-head/build/head.bin http://stackchan.local/update
```

Then open `http://stackchan.local/app`, type the update token once (the page keeps it in your browser), and move the sliders. Or with `curl`:

| Request | What it does |
| --- | --- |
| `GET /head` | Both servos: angle, raw position, zero, torque, voltage, temperature |
| `POST /head?yaw=20&pitch=45&ms=1000` | Move. Token needed. Add `&hold=1` to keep the torque on after the move |
| `POST /head/off` | Torque and servo power off. Token needed |
| `POST /head/zero?yaw=436&pitch=585` | Save new zero positions (raw 0 to 1023) in NVS. Token needed |
| `GET /i2c` | The chips that answer on the internal I2C bus |
| `GET /app` | The page. A browser that opens `/` lands here |
| `GET /log` | The recent log, kept across restarts (from `tr_ota`). The page shows it too |

```bash
curl -X POST -H "X-Token: <token>" "http://stackchan.local/head?yaw=30&pitch=45&ms=1000"
```

## Safety

- Nothing moves at boot. Servo power stays off until the first move.
- Pitch stays between 5° and 85°, yaw between -90° and 90°. `tr_board` clamps any other value, so a typo can't push the pitch servo against its end stop.
- Each move takes 100 ms to 5 s (1 s by default).
- After a move the torque goes off again, unless you ask for `hold=1`. Then the servos don't fight a hand that turns the head.

## Expected output

On our robot, on 2026-10-31, after saving the factory zero positions:

```
$ curl -X POST -H "X-Token: <token>" "http://stackchan.local/head?yaw=45&ms=1500"
servo power: on
yaw  : 43.8 deg (raw 576, zero 436), torque off, still, load 0, 5.1 V, 24 C
pitch: 5.0 deg (raw 601, zero 585), torque off, still, load 0, 5.2 V, 23 C
```

| Asked | Reached |
| --- | --- |
| yaw 10, 0 | 9.1, 0.6 |
| yaw -45, 45 | -44.1, 43.8 |
| yaw -90, 90 | -89.1, 89.1 |
| pitch 15, 5 | 14.4, 5.0 |
| pitch 45, 85 | 44.4, 84.4 |
| pitch 120, yaw 200 (out of range) | clamped: 84.4, 89.1 |

The servos warmed from 22 °C to 25 °C over the test, and the voltage stayed at 5.0 to 5.2 V. The load reads 0 because we read it after the torque goes off.

## How it works

- **Servo power.** The PY32 (I2C `0x6F`) drives VM_EN on its pin 0. `tr_servo_init()` makes it an output with a pull-up and changes only that bit. The PY32 NACKs a read after a repeated start, so `tr_board` sends the register and reads it in two transfers, at 100 kHz. After power on it waits 500 ms. With 200 ms (the BSP's value) the first read timed out.
- **Protocol.** `FF FF id len instruction params checksum` at 1 Mbps on UART1 (TX G6, RX G7). 16-bit values go high byte first. A move writes goal position, time and speed (registers 42 to 47). A read gets position, speed, load, voltage and temperature (registers 56 to 63).
- **Angles.** `raw = zero + degrees × 3.2` (0.3125° per step, as in the BSP). The zero positions are in NVS, namespace `servo`, keys `zero_pos_1` (yaw, ID 1) and `zero_pos_2` (pitch, ID 2). The factory firmware and UiFlow use the same keys. Our unit's factory values are 436 and 585. We read them from the factory backup. The BSP's defaults are 460 and 620, about 7° and 11° away.

## Try next

- Put the face from [04-face](../04-face) on the real screen, and make the head follow it.
- Read more sensors on the page: light (LTR553), motion (BMI270), head touch (Si12T).
