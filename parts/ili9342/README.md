# ili9342: an SPI screen as a virtual part

A [virtual part](https://github.com/espressif/esp-emulator/blob/main/docs/guides/PARTS.md) for Espressif's esp-emu (1.0.0 or later). It plays an ILI9342C screen, 320×240 on SPI, like the one in the CoreS3 (StackChan's head and the CoreS3 Lite). The [browser emulator](https://robotics.diegoromero.es/emulator/) loads it for both boards and draws what the firmware sends.

It reads the SPI traffic itself, so any driver works: M5GFX, ESP-IDF's `esp_lcd`, and our [tr_lcd](../../components/tr_board). It was tested with [06-status](../../examples/stackchan/06-status), which draws with `esp_lcd`.

## What it models

The part reads every SPI transfer to its chip select. The D/C pin, as it was when the transfer was clocked, says what the bytes are: low is a command, high is data.

| Command | In the part |
| --- | --- |
| 0x01 SWRESET | Back to the reset state: sleeping, display off, black |
| 0x10 / 0x11 SLPIN / SLPOUT | Sleep: the screen shows nothing |
| 0x28 / 0x29 DISPOFF / DISPON | Display off / on |
| 0x20 / 0x21 INVOFF / INVON | Color inversion (see below) |
| 0x2A / 0x2B CASET / RASET | The window to draw in, 4 bytes each |
| 0x2C / 0x3C RAMWR / RAMWRC | Pixels into the window, from its corner / from where they stopped |
| 0x36 MADCTL | Row and column order, row/column exchange, and BGR |
| 0x3A COLMOD | 16 bits a pixel (0x55) or 18 bits (0x66, 3 bytes a pixel) |

Other commands (power, gamma, timing) are taken and ignored. Reads answer zeros. M5GFX reads the panel ID by toggling the pins itself, not through the SPI peripheral, and only logs a warning when nothing answers.

Two facts about the CoreS3's panel, from the drivers:

- It is an IPS panel that shows the colors inverted unless the firmware sends INVON. M5GFX sets `invert = true`, and tr_lcd sends 0x21.
- Its subpixels are in BGR order, so MADCTL's BGR bit (0x08, what M5GFX and tr_lcd send) gives the right colors.

The backlight is not part of the screen: on the CoreS3 it is the AXP2101's DLDO1, in the [cores3](../cores3) part.

The page reads `state()`:

| Key | Value |
| --- | --- |
| `on` | 1 when the screen is awake and the display is on |
| `frames` | How many times a whole window was filled |
| `rows` | The first and last row that changed since the last read, such as `100,120` |
| `pixels` | Those whole rows, RGB565 little-endian, in base64, as the screen shows them |

## Wiring

The CoreS3:

```json
{ "parts": { "ili9342": { "module": "ili9342.wasm", "pins": { "CS": 3, "DC": 35 } } } }
```

## A note on memory

The framebuffer (150 KB) is allocated at the first pixel, not when the part is built. In esp-emu 1.0.0 the browser's `part-worker.js` reads `memory.buffer` and then calls `part_buffer()`, which builds the part. If building it grows the WebAssembly memory, that buffer is detached and the part fails to start ("Cannot perform Construct on a detached ArrayBuffer"). Every part here keeps its `Default` free of heap allocations for this reason.

## Build and test

`scripts/build-parts.sh` runs the host tests and builds `out/ili9342.wasm`, as for every part. It depends on Espressif's [esp-part](https://github.com/espressif/esp-emulator/tree/main/esp-part) crate (Apache-2.0), at the same commit as the other parts.
