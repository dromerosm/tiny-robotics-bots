# stackchan-body: StackChan's body as a virtual part

A [virtual part](https://github.com/espressif/esp-emulator/blob/main/docs/guides/PARTS.md) for Espressif's esp-emu (1.0.0 or later). It plays the pieces of StackChan's body that the firmware talks to, so the real servo driver in [tr_board](../../components/tr_board) runs unchanged in the emulator. The [browser emulator](https://robotics.diegoromero.es/emulator/) loads it for every ESP32-S3 image, and [16-servos](../../examples/stackchan/16-servos) shows it.

## What it models

| On the robot | In the part |
| --- | --- |
| PY32 IO expander, I2C address 0x6F | Version 0x41 at register 0x02. Bit 0 of registers 0x03 (output) and 0x05 (level) is VM_EN, the servo power. Every other register stores what is written |
| Two FEETECH SCS0009 servos on UART1, 1 Mbaud, IDs 1 (yaw) and 2 (pitch) | The SCS protocol: ping, read, write, reg write, action and sync write. Goal position, time and speed (register 42), position, speed, voltage and temperature (register 56), torque (40), moving (66) |
| Servo power | The servos answer only while VM_EN is on. Power off turns their torque off |
| A move | Takes the time the firmware asks for, in emulated time. With no time it follows the speed, or takes about 0.15 s for 60°. Writing a goal turns the torque on, as on the servo |

It leaves out: load, a hand that turns the head, temperature and voltage changes (always 5.0 V and 30 °C), the side LEDs, and the servo's limits and other settings. After a chip reset the body keeps its power and the servos stay where they are, as on the robot.

The page reads `state()`: `power`, `yaw` and `pitch` in degrees (from the default zero positions 460 and 620, 3.2 steps per degree), `yaw_raw`, `pitch_raw`, `yaw_torque`, `pitch_torque` and `moving`.

## Wiring

The part's own pin names. For StackChan:

```json
{ "parts": { "body": { "module": "stackchan-body.wasm", "pins": { "TX": 7, "RX": 6, "SDA": 12, "SCL": 11 } } } }
```

The part's TX goes to the chip's RX (GPIO 7), and its RX to the chip's TX (GPIO 6). SDA and SCL are the CoreS3's internal I2C bus.

## Build and test

```bash
scripts/build-parts.sh
```

It runs the host tests (`cargo test`), then builds `out/stackchan-body.wasm` for `wasm32-unknown-unknown`. It uses the local Rust toolchain if it has that target, otherwise the Rust Docker image. The version is `RUST_VERSION` in `tools.env`.

The part depends on Espressif's [esp-part](https://github.com/espressif/esp-emulator/tree/main/esp-part) crate (Apache-2.0), pinned to the commit that published esp-emu 1.0.0. The protocol and register map follow FEETECH's SCSCL driver, as M5Stack's StackChan-BSP uses it.
