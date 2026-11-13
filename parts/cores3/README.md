# cores3: the CoreS3 power chips as a virtual part

A [virtual part](https://github.com/espressif/esp-emulator/blob/main/docs/guides/PARTS.md) for Espressif's esp-emu (1.0.0 or later). It plays the two chips on the CoreS3 that [tr_board](../../components/tr_board) needs to give the body its 5 V. With it and [stackchan-body](../stackchan-body), the side LEDs light in the emulator. The [browser emulator](https://robotics.diegoromero.es/emulator/) loads both for every ESP32-S3 image, and [16-servos](../../examples/stackchan/16-servos) shows them (`led 255 80 0`).

## What it models

| On the robot | In the part |
| --- | --- |
| AW9523 IO expander, I2C address 0x58 | ID 0x23 at register 0x10. Output levels at 0x02/0x03, directions at 0x04/0x05 (1 = input, all inputs after reset). 0x00/0x01 read the pins: an output reads its level, an input reads high |
| The body's 5 V | On when P0_1 (BUS_OUT_EN) and P1_7 (BOOST_EN) are both outputs and high |
| AXP2101 power chip, I2C address 0x34 | USB power present (bit 5 of register 0x00), chip ID 0x4A at 0x03, battery 80 % at 0xA4 |

Every other register stores what is written. It is not a model of the chips' electronics: no screen, touch, camera or speaker enables, no real battery, no charging.

The page reads `state()`: `body_5v`, `bus_out_en` and `boost_en` (0 or 1).

## Wiring

The CoreS3's internal I2C bus:

```json
{ "parts": { "cores3": { "module": "cores3.wasm", "pins": { "SDA": 12, "SCL": 11 } } } }
```

## Build and test

`scripts/build-parts.sh` runs the host tests and builds `out/cores3.wasm`, as for every part. It depends on Espressif's [esp-part](https://github.com/espressif/esp-emulator/tree/main/esp-part) crate (Apache-2.0), at the same commit as `stackchan-body`.
