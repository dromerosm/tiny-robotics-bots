# tr_board

StackChan board parts that several examples share. For now:

- The internal I2C bus (SDA G12, SCL G11). `tr_board_i2c()` makes it on the first call and returns the same bus later. Use it for every chip on that bus, so the firmware makes the bus only once.
- The battery monitor in the body: an INA226 at `0x41`, with a 10 mΩ shunt.
- The CoreS3 screen (320 x 240), with text in a 5x7 font. [06-status](../../examples/stackchan/06-status) uses it.
- The two head servos (FEETECH SCS0009 on UART1) and their power switch on the PY32. [07-head](../../examples/stackchan/07-head) uses them.
- `tr_board_scan()`: the chips that answer on the internal I2C bus.

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

## Servos

```c
ESP_ERROR_CHECK(tr_servo_init());            // finds the PY32; nothing moves
ESP_ERROR_CHECK(tr_servo_power(true));       // VM_EN on, then waits 500 ms
tr_servo_move(TR_SERVO_PITCH, 45, 1000);     // degrees from zero, in 1000 ms
tr_servo_state_t s;
tr_servo_read(TR_SERVO_PITCH, &s);           // s.degrees, s.raw, s.volts_x10, s.celsius...
tr_servo_torque(TR_SERVO_PITCH, false);
```

- Pitch is clamped to 5° to 85° and yaw to -90° to 90°. Move times are clamped to 100 ms to 5 s.
- The zero positions come from NVS (`servo`/`zero_pos_1` and `zero_pos_2`, as in the factory firmware), or the BSP defaults (460 and 620). `tr_servo_set_zero()` saves new ones.
- The PY32 NACKs a read after a repeated start. So `tr_board` writes the register and reads it in two transfers, at 100 kHz. With `i2c_master_transmit_receive()` the reads failed and gave wrong values.
- See [07-head](../../examples/stackchan/07-head) for the protocol and the test on our robot.

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
| USB plugged in, first time | 3.51 V, 0 mA, steady over 6 reads |
| USB plugged in, later | 4.12 V, 643 mA, charging |
| On battery, Wi-Fi on, screen off, servos off | 3.68 to 3.72 V, 113 to 131 mA, discharging |

With the screen on (06-status), on battery: about 240 mA.

On battery, the sign and the size of the current look right. Later, on USB, the INA226 showed a charge current (643 mA), so it does measure charging. We don't know why the first USB reading showed 0 mA at 3.51 V. Maybe the charger had not started yet.

## Third-party code

- `tr_lcd.c`: the screen power-up order, pins and ILI9342E setup commands follow Espressif's BSP for the M5Stack CoreS3 ([esp-bsp](https://github.com/espressif/esp-bsp), `bsp/m5stack_core_s3`, Apache-2.0).
- `font5x7.h`: the classic 5x7 font from [Adafruit-GFX-Library](https://github.com/adafruit/Adafruit-GFX-Library) (BSD license, full text in the file).
- `tr_servo.c`: the servo IDs, zero positions, angle scale and PY32 setup follow M5Stack's [StackChan-BSP](https://github.com/m5stack/StackChan-BSP) (MIT).
