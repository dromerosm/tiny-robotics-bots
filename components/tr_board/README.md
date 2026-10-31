# tr_board

StackChan board parts that several examples share. For now:

- The internal I2C bus (SDA G12, SCL G11). `tr_board_i2c()` makes it on the first call and returns the same bus later. Use it for every chip on that bus, so the firmware makes the bus only once.
- The battery monitor in the body: an INA226 at `0x41`, with a 10 mΩ shunt.
- The CoreS3 screen (320 x 240), with text in a 5x7 font. [06-status](../../examples/stackchan/06-status) uses it.
- The two head servos (FEETECH SCS0009 on UART1) and their power switch on the PY32. [07-head](../../examples/stackchan/07-head) uses them.
- The other sensors (`tr_sensors_*`): head touch, screen touch, motion, clock, power chip, light, and the ESP32-S3's own temperature.
- The microphones (ES7210, `tr_mic_*`), the camera (GC0308, `tr_cam_*`) and the 12 side LEDs (`tr_led_*`).
- `tr_board_aw9523_on()`: turn on single AW9523 pins without touching the others.
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
   Our unit has the ILI9342E (firmware ID `0x12`).
3. AXP2101 (`0x34`): backlight on DLDO1 at 3.3 V (register `0x99` = 28, then bit 7 of `0x90`).
4. SPI3 at 40 MHz: SCLK G36, MOSI G37, DC G35, CS G3, no MISO. The `esp_lcd_ili9341` driver, BGR order, colors inverted, and extra setup commands for the ILI9342E.

For pictures that change, like a face, `tr_lcd_render(fn, ctx)` draws the whole screen from a function that gives each pixel's color. It sends 15 bands of 16 rows, about 40 ms, with no flicker. [08-face](../../examples/stackchan/08-face) uses it.

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

## Sensors

```c
tr_sensors_init();                      // sets up the chips that answer; skips the others
char text[1024];
tr_sensors_status(text, sizeof(text));  // one or two lines per sensor
int x, y;
if (tr_touch_read(&x, &y)) { ... }      // a finger on the screen, in pixels
```

| Sensor | Chip, address | What we read | Source |
| --- | --- | --- | --- |
| Head touch | Si12T, `0x68` | Register `0x10`: three zones, 2 bits each (0 to 3). Which zone is front or back is not documented | StackChan-BSP |
| Screen touch | FT6336, `0x38` | Register `0x02`: number of points, then x and y of the first one | M5GFX |
| Motion | BMI270, `0x69` | Accel (±8 g) and gyro (±2000°/s) from `0x0C`, temperature from `0x22` | M5Unified |
| Clock | BM8563, `0x51` | Seconds to year from `0x02`, in BCD. Bit 7 of the seconds means it lost power | M5Unified |
| Power chip | AXP2101, `0x34` | Status `0x00`/`0x01`, battery % `0xA4`, battery `0x34`, USB `0x38` and system `0x3A` voltages | M5Unified |
| Light | LTR553, `0x23` | Brightness: two channels from `0x88`, shown as their mean (raw counts, not lux). Proximity: 11 bits from `0x8D`. It needs the camera's power, so `tr_sensors_init()` turns that on | M5CoreS3 (LTR5XX) |
| ESP32-S3 | built in | `temperature_sensor` driver, started on the first read | ESP-IDF |

- The BMI270 needs Bosch's 8 KB setup file at every start (`bmi270_config.h`). `tr_sensors_init()` sends it in 64-byte blocks.
- It also needs 2 ms after a soft reset. At FreeRTOS' 100 Hz, `vTaskDelay(pdMS_TO_TICKS(2))` waits 0 ticks, so the first setup failed. We use `esp_rom_delay_us()` for waits under 10 ms.
- The ESP32-S3's temperature sensor stops esp-emu at start, so it is started on the first read, not in `tr_sensors_init()`.

## Microphones, camera and LEDs

| Part | How | Power | Source |
| --- | --- | --- | --- |
| Microphones | ES7210 (`0x40`) on I2S0: MCLK G0, BCLK G34, WS G33, data G14. `esp_codec_dev` 1.5, mono, 16 bits, gain 30 dB. `tr_mic_start(rate)`, then `tr_mic_read()` | AW9523 P0_2 | Espressif CoreS3 BSP |
| Camera | GC0308 (`0x21`) on the DVP port: D0-D7 G39 G40 G41 G42 G15 G16 G48 G47, VSYNC G46, HREF G38, PCLK G45, no XCLK pin. `esp32-camera`, 320 x 240 RGB565, frames in PSRAM. `tr_cam_start()`, then `esp_camera_fb_get()` | AXP2101 ALDO3 (3.3 V), BLDO1, BLDO2, AW9523 P1_0 | Espressif CoreS3 BSP |
| Side LEDs | 12 WS2812 on PY32 pin 13. LED count in `0x24`, colors from `0x30` (RGB565, low byte first), bit 6 of `0x24` shows them. LEDs 0-5 left, 6-11 right | PY32 | StackChan-BSP |

- The camera driver shares our I2C bus (`sccb_i2c_port`), so the bus is fixed on port 0 (`TR_I2C_PORT`).
- Examples that use the camera turn on the CoreS3's 8 MB PSRAM (`CONFIG_SPIRAM`, quad mode). On our robot it gives 8.3 MB free.
- `tr_lcd_blit()` copies camera frames to the screen through the DMA band, since the frame is in PSRAM.

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
- `tr_sensors.c`: registers and setup from StackChan-BSP (Si12T), M5Unified (AXP2101, BM8563, BMI270) and M5GFX (FT6336), all MIT. `bmi270_config.h` is Bosch Sensortec's BMI270 config file (BSD-3-Clause), taken from M5Unified.
- `tr_servo.c`: the servo IDs, zero positions, angle scale and PY32 setup follow M5Stack's [StackChan-BSP](https://github.com/m5stack/StackChan-BSP) (MIT).
