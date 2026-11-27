# ws2812: RGB LEDs as a virtual part

A [virtual part](https://github.com/espressif/esp-emulator/blob/main/docs/guides/PARTS.md) for Espressif's esp-emu (1.0.0 or later). It reads a chain of WS2812 RGB LEDs on one data pin, like the LED of the AtomS3 Lite (a WS2812C on GPIO 35). The [browser emulator](https://robotics.diegoromero.es/emulator/) loads it for the AtomS3 Lite board and draws the color.

It works with any driver that sends the standard WS2812 signal: the RMT bytes encoder in [01-homekit](../../examples/atoms3-lite/01-homekit), ESP-IDF's `led_strip`, and Arduino. It was tested with 01-homekit and with M5Stack's factory firmware for the AtomS3 Lite.

## What it models

The part watches every edge of its pin (`GpioClaim.watch_pins`) and reads each high pulse as one bit:

| Signal | In the part |
| --- | --- |
| High for less than 0.6 µs | A 0. The datasheet says 0.4 µs; ESP-IDF drivers often send 0.3 µs |
| High for 0.6 µs or more | A 1. The datasheet says 0.8 µs; ESP-IDF drivers often send 0.9 µs |
| High for less than 0.1 µs | Noise: counted in `bad`, and the frame starts again |
| High for more than 5 µs | An idle line (the pin floats high until the firmware drives it): the frame starts again |
| Low for 50 µs or more | The reset: the next bit goes to the first LED again |

Each LED takes 24 bits: green, red, then blue, most significant bit first. Up to 64 LEDs. A real LED shows its color at the reset; the part shows it as soon as its 24 bits are in, which looks the same on a page.

It drives nothing. It does not model brightness, heat or power.

The page reads `state()`:

| Key | Value |
| --- | --- |
| `leds` | The colors as `rrggbb`, comma separated, in chain order |
| `frames` | How many LED colors were set |
| `bad` | Noise pulses seen |

## Wiring

The AtomS3 Lite:

```json
{ "parts": { "ws2812": { "module": "ws2812.wasm", "pins": { "DIN": 35 } } } }
```

## Build and test

`scripts/build-parts.sh` runs the host tests and builds `out/ws2812.wasm`, as for every part. It depends on Espressif's [esp-part](https://github.com/espressif/esp-emulator/tree/main/esp-part) crate (Apache-2.0), at the same commit as the other parts.
