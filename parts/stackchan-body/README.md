# stackchan-body: StackChan's body as a virtual part

A [virtual part](https://github.com/espressif/esp-emulator/blob/main/docs/guides/PARTS.md) for Espressif's esp-emu (1.0.0 or later). It plays the pieces of StackChan's body that the firmware talks to, so the real drivers in [tr_board](../../components/tr_board) and in M5Stack's StackChan firmware run unchanged in the emulator. The [browser emulator](https://robotics.diegoromero.es/emulator/) loads it for the StackChan board, and [16-servos](../../examples/stackchan/16-servos) shows it.

## What it models

| On the robot | In the part |
| --- | --- |
| PY32 IO expander, I2C address 0x6F | Version 0x41 at register 0x02. Bit 0 of registers 0x03 (output) and 0x05 (level) is VM_EN, the servo power. Every other register stores what is written |
| Two FEETECH SCS0009 servos on UART1, 1 Mbaud, IDs 1 (yaw) and 2 (pitch) | The SCS protocol: ping, read, write, reg write, action and sync write. Goal position, time and speed (register 42), position, speed, voltage and temperature (register 56), torque (40), moving (66) |
| Servo power | The servos answer only while VM_EN is on. Power off turns their torque off |
| The 12 side LEDs (PY32 pin 13) | Colors in LED memory (register 0x30, RGB565, low byte first), shown when bit 6 of register 0x24 is written |
| A move | Takes the time the firmware asks for, in emulated time. With no time it follows the speed, or takes about 0.15 s for 60°. Writing a goal turns the torque on, as on the servo |
| Si12T head touch, I2C address 0x68 | Registers 0x02 to 0x0F keep what is written, so StackChan's readbacks match. Register 0x10 gives the three zones, 2 bits each (0 to 3): front in bits 1:0, middle in 3:2, back in 5:4. Bits 7:6 are 0. Every other register reads 0x00 |
| INA226 battery monitor, I2C address 0x41, 10 mΩ shunt | 16-bit registers, high byte first. The register pointer stays between transactions. Bus voltage (0x02, 1.25 mV per bit) and shunt voltage (0x01, 2.5 µV per bit, so 0.25 mA per bit) come from the inputs. Config 0x00 (reset 0x4127), calibration 0x05, mask 0x06 and alert limit 0x07 can be written. Current 0x04 and power 0x03 follow the datasheet formulas. Maker ID 0x5449 at 0xFE, die ID 0x2260 at 0xFF |

**Which zone is which.** M5Stack's StackChan Arduino library says zone 0 is the front, 1 the middle and 2 the back, and "swipe forward" goes from front to back. The official firmware agrees: it calls a move from zone 0 to zone 2 a forward swipe.

**Why 0x68 reads 0x00.** M5Unified looks for an IMU at 0x68. It reads register 0x75 (an MPU6886 gives 0x19, 0x68 or 0x71) and register 0x00 (a BMI270 gives 0x24). The Si12T gives 0x00 at both, so M5Unified finds no IMU there and uses the BMI270 at 0x69.

**Only from the datasheet, not checked against code or the robot:** the INA226 config reset value 0x4127, the die ID 0x2260, the current and power formulas, bit 15 of calibration, and what happens on a read longer than 2 bytes (the part repeats the same register). The part also follows the INA226 mode: continuous modes follow the inputs, a triggered mode measures once, power-down keeps the last values. It does not model the conversion time or the averaging, so a new value shows at once. For the Si12T, no reset value is known (all start at 0x00), and the settings (sensitivity, sleep, soft reset) change nothing. The pointer moving on after each byte is a guess: the drivers read one byte at a time. Registers 0x11 and 0x12 (OUTPUT2 and OUTPUT3 in the driver) read 0x00, as outputs of channels that are not wired.

It leaves out: load, a hand that turns the head, temperature and voltage changes (always 5.0 V and 30 °C), and the servo's limits and other settings. The LEDs need 5 V from the CoreS3, and the [cores3](../cores3) part plays the chips that switch it. The page lights the LEDs only when that part says the 5 V is on. After a chip reset the body keeps its power and the servos stay where they are, as on the robot.

The page reads `state()`: `power`, `yaw` and `pitch` in degrees (from the default zero positions 460 and 620, 3.2 steps per degree), `yaw_raw`, `pitch_raw`, `yaw_torque`, `pitch_torque`, `moving`, and `leds`: the 12 shown colors as `rrggbb`, LED 0 first, separated by commas. Also `head`: the three touch zones, front first, such as `0,3,0`. And `battery_mv` and `battery_ma`: what the INA226 measures.

The page sets values with inputs:

| Input | What | Default |
| --- | --- | --- |
| `set:head_front=N`, `set:head_middle=N`, `set:head_back=N` | Touch on that zone, 0 (none) to 3 (strong) | 0 |
| `set:battery_mv=N` | Battery voltage in mV | 4000. The cores3 part's AXP2101 reports 4000 mV for the same battery. Our robot read 3.95 V at 82 % (tr_board README, 2026-11-01). StackChan has one 550 mAh cell |
| `set:battery_ma=N` | Battery current in mA. Positive: discharging, negative: charging, as tr_board reads it | 96. Our robot awake, screen at 20 %, servos powered (tr_board README, 2026-11-01) |

The SDK's `set_value` only accepts `-` in a name, so `set:head-front=3` and `set:battery-mv=3700` work too.

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
