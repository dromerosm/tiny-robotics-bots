# cores3: the CoreS3 board's chips as a virtual part

A [virtual part](https://github.com/espressif/esp-emulator/blob/main/docs/guides/PARTS.md) for Espressif's esp-emu (1.0.0 or later). It plays every chip on the internal I2C bus of M5Stack's CoreS3 board, which is StackChan's head (the StackChan Core) and the CoreS3 Lite. The [browser emulator](https://robotics.diegoromero.es/emulator/) loads it for both boards, next to [ili9342](../ili9342) (the screen) and, on StackChan, [stackchan-body](../stackchan-body).

Each chip follows its datasheet and the code of three drivers, so all three run unchanged:

- our [tr_board](../../components/tr_board),
- M5Stack's [M5Unified](https://github.com/m5stack/M5Unified) and [M5GFX](https://github.com/m5stack/M5GFX),
- the official [StackChan firmware](https://github.com/m5stack/StackChan) (xiaozhi, the Bosch BMI270 SensorAPI, esp_codec_dev, esp_cam_sensor).

The unit tests replay each driver's own sequences. M5Stack's StackChan firmware (UserDemo v1.5.1) boots on it: the RTC and the IMU start, the welcome screen comes up, and a touch opens the launcher.

## The chips

| Address | Chip | What it models |
| --- | --- | --- |
| 0x21 | GC0308 camera | The control bus only: chip ID 0x9B, page select, soft reset, readback. M5GFX reads the ID to tell a CoreS3 from a CoreS3 SE. No picture: the pixels use the camera's parallel bus, which a virtual part cannot reach |
| 0x23 | LTR-553ALS light and proximity | Standby, gain, integration time, light and proximity readings from the page |
| 0x34 | AXP2101 power | Chip ID, USB power and a battery, voltages, fuel gauge. DLDO1 is the screen's backlight |
| 0x36 | AW88298 speaker amplifier | 16-bit registers with readback, chip ID 0x1852, reset values from the datasheet. No sound |
| 0x38 | FT6336U touch | Panel ID (ILI9342C), one touch point from the page |
| 0x40 | ES7210 microphone ADC | Registers with readback and reset values from the datasheet. No audio |
| 0x51 | BM8563 RTC | A clock that runs in emulated time, alarm and timer flags |
| 0x58 | AW9523B IO expander | Pins, directions, and the interrupt on input change |
| 0x69 | BMI270 IMU, with a BMM150 behind it | The 8 KB config upload, accel and gyro at the firmware's range and rate, the magnetometer through the aux interface |

Other addresses get no answer. That matters: M5Unified looks for an IMU at 0x68 (the head touch in the StackChan body answers there, with zeros) and for a magnetometer at 0x10.

### Wires between the chips

- **Touch interrupt.** While a finger is down, the touch chip pulls AW9523 P1_2 low. The AW9523 then pulls its INTN low, on GPIO 21 (pin `INT`), until the firmware reads the port. M5GFX reads the touch only while GPIO 21 is low.
- **Body power.** StackChan's body gets 5 V when AW9523 P0_1 (BUS_OUT_EN) and P1_7 (BOOST_EN) are both high outputs. State `body_5v` says so; the side LEDs in [stackchan-body](../stackchan-body) light only then.

### Notes per chip

- **Touch.** Set the finger with `touch_x` and `touch_y` (screen pixels, 0-319 and 0-239), then `touch=1` to press and `touch=0` to lift. Values outside the screen go to the edge. When the finger lifts, the touch count goes straight to 0: all three drivers end the touch there.
- **IMU.** The firmware must load the 8 KB config file first, as on the real chip; until then there is no data. Set the tilt with `roll` and `pitch` and the compass heading with `yaw`, in degrees, and rotation with `gyro_x`, `gyro_y`, `gyro_z` in degrees per second. With no input the board lies flat, screen up: +1 g on Z. The magnetometer sees a fixed Earth field, 40 µT to the north and 30 µT down. The temperature is 25 °C. No FIFO and no motion features (step counter, tap).
- **Clock.** It starts at 2000-01-01 00:00:00 with the "lost power" bit set. `time=<Unix seconds>` sets it (the page sends your computer's time), and then the bit is clear. The alarm and timer set their flags, but no pin goes low.
- **Light and proximity.** Set `lux` and `proximity` (0-2047). At gain 1x, tr_board prints about the lux value you set. The real chip answers only when the camera has power; this one always answers.
- **Power.** `battery` sets the fuel gauge in percent. The voltages stay at 4.0 V (battery) and 5.0 V (USB, system).
- **Camera.** It always answers. A real camera needs AXP2101 ALDO3 and AW9523 P1_0 on first.

Values that come only from a datasheet, and not from any driver, are marked "datasheet, unconfirmed" in the code.

## Inputs and state

Inputs are `set:<name>=<value>`. The SDK takes only letters, digits and `-` in a name, so `touch-x` and `touch_x` both work.

| Input | Chip |
| --- | --- |
| `touch_x`, `touch_y`, `touch` | FT6336U |
| `roll`, `pitch`, `yaw`, `gyro_x`, `gyro_y`, `gyro_z` | BMI270, BMM150 |
| `time` | BM8563 |
| `lux`, `proximity` | LTR-553ALS |
| `battery` | AXP2101 |

`state()` has `body_5v`, `bus_out_en`, `boost_en`, `speaker_en`, `camera_en`, `lcd_reset` (AW9523), `backlight_mv`, `battery` (AXP2101), `touch`, `touch_x`, `touch_y`, `imu_roll`, `imu_pitch`, `imu_yaw`, `imu_init`, `imu_acc_range_g`, `rtc`, `lux`, `proximity`, `amp_on`, `amp_volume`, `mic_on` and `camera_id_reads`.

## Wiring

The CoreS3's internal I2C bus, and the AW9523's interrupt line:

```json
{ "parts": { "cores3": { "module": "cores3.wasm", "pins": { "SDA": 12, "SCL": 11, "INT": 21 } } } }
```

## Build and test

`scripts/build-parts.sh` runs the host tests and builds `out/cores3.wasm`, as for every part. It depends on Espressif's [esp-part](https://github.com/espressif/esp-emulator/tree/main/esp-part) crate (Apache-2.0), at the same commit as `stackchan-body`. The part allocates nothing on the heap when it is built (see the note in [ili9342](../ili9342)).
