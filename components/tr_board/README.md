# tr_board

StackChan board parts that several examples share. For now:

- The internal I2C bus (SDA G12, SCL G11). `tr_board_i2c()` makes it on the first call and returns the same bus later. Use it for every chip on that bus, so the firmware makes the bus only once.
- The battery monitor in the body: an INA226 at `0x41`, with a 10 mΩ shunt.
- The CoreS3 screen (320 x 240), with text in a 5x7 font. [06-status](../../examples/stackchan/06-status) uses it.

More parts (PY32, servos) come here when the examples need them.

## Use it

Add the components folder to the project's `CMakeLists.txt`, before `project()`:

```cmake
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../../components")
```

Require it in `main/CMakeLists.txt` with `PRIV_REQUIRES tr_board`, and:

```c
#include "tr_board.h"

tr_battery_t b;
if (tr_battery_read(&b) == ESP_OK) {
    printf("%d mV, %d mA\n", b.millivolts, b.milliamps); // positive mA: discharging
}

ESP_ERROR_CHECK(tr_ota_add_status(tr_board_status)); // battery line on http://stackchan.local/
```

`tr_battery_read()` returns `ESP_ERR_NOT_FOUND` when no INA226 answers at `0x41`, as in the emulator. Before it trusts the chip, it checks the maker ID (`0x5449`).

## Screen

```c
ESP_ERROR_CHECK(tr_lcd_init());               // only on the robot: the emulator has no screen
tr_lcd_fill(TR_LCD_BLACK);
tr_lcd_text(4, 40, 2, TR_LCD_WHITE, TR_LCD_BLACK, "hello");   // x, y, scale 1 or 2
tr_lcd_fill_rect(0, 0, TR_LCD_W, 30, TR_LCD_RGB(20, 60, 120));
```

`tr_lcd_init()`:

1. AW9523 (`0x58`): sets P0_0 (touch reset) and P1_1 (screen reset) high. It reads each register first and changes only those bits, plus port 0 push-pull (register `0x11`, bit 4), as the BSP sets it. We don't use Espressif's AW9523 driver: it sets all 16 pins to low outputs, and the other pins drive power lines of the CoreS3.
2. Touch chip (`0x38`): its firmware ID (register `0xA6`) is `0x12` on the ILI9342E screen (units from August 2026 on). Other values mean ILI9342C.
3. AXP2101 (`0x34`): backlight on DLDO1 at 3.3 V (register `0x99` = 28, then bit 7 of `0x90`).
4. SPI3 at 40 MHz: SCLK G36, MOSI G37, DC G35, CS G3, no MISO. The `esp_lcd_ili9341` driver, BGR order, colors inverted, and extra setup commands for the ILI9342E.

Drawing goes through one DMA buffer of 16 rows (10 KB). Each call first waits until the last band has been sent.

## INA226

From TI's datasheet. Registers are 16 bits, high byte first. The default setup (continuous, no averaging) is enough, so we write nothing.

| Register | What | Scale |
| --- | --- | --- |
| `0x01` | Shunt voltage, signed | 2.5 µV per bit. Over 10 mΩ, that is 0.25 mA per bit |
| `0x02` | Bus voltage | 1.25 mV per bit |
| `0xFE` | Maker ID | `0x5449` ("TI") |

## On our robot (2026-10-31)

| Power | Reading |
| --- | --- |
| USB plugged in | 3.51 V, 0 mA, steady over 6 reads |
| On battery, Wi-Fi on, screen off, servos off | 3.68 to 3.72 V, 113 to 131 mA, discharging |

With the screen on (06-status), on battery: about 240 mA.

On battery, the sign and the size of the current look right. On USB, the voltage reads lower than on battery, and there is no charge current. We don't know why. One guess: on USB the power path cuts the battery off, and the INA226 then measures another point. Don't trust the USB-plugged reading as the battery's charge yet.

## Third-party code

- `tr_lcd.c`: the screen power-up order, pins and ILI9342E setup commands follow Espressif's BSP for the M5Stack CoreS3 ([esp-bsp](https://github.com/espressif/esp-bsp), `bsp/m5stack_core_s3`, Apache-2.0).
- `font5x7.h`: the classic 5x7 font from [Adafruit-GFX-Library](https://github.com/adafruit/Adafruit-GFX-Library) (BSD license, full text in the file).
