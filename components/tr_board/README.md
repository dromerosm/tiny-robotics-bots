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
- Sleep (`tr_sleep_*`): after a minute with no touch, on battery, it turns off what nobody uses, and a touch wakes the robot. See [Sleep](#sleep).
- The battery's average use and the time left (`tr_battery_average_ma()`, `tr_battery_minutes_left()`).

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
   On a cold start this is the first time the touch chip runs. `tr_sensors_init()` may run before it and get no answer at `0x38`. It adds the touch anyway, so the touch works once `tr_lcd_init()` has taken it out of reset.
3. AXP2101 (`0x34`): backlight on DLDO1 at 3.3 V (register `0x99` = 28, then bit 7 of `0x90`). `tr_lcd_brightness(percent)` dims it later as M5GFX does: 20 % is 2.6 V and saves about 110 mA (see [09-wifi](../../examples/stackchan/09-wifi)). Every value from 1 % to 5 % gives 2.5 V, the lowest step. `tr_lcd_power(false)` puts the panel to sleep and turns DLDO1 off (see [Sleep](#sleep)).
4. SPI3 at 40 MHz: SCLK G36, MOSI G37, DC G35, CS G3, no MISO. The `esp_lcd_ili9341` driver, BGR order, colors inverted, and extra setup commands for the ILI9342E.

For pictures that change, like a face, `tr_lcd_render(fn, ctx)` draws the whole screen from a function that gives each pixel's color. It sends 15 bands of 16 rows, about 40 ms, with no flicker. [08-face](../../examples/stackchan/08-face) uses it.

Drawing goes through two DMA buffers of 16 rows (10 KB each): one band goes out on SPI while the next one is filled. Each call waits only when both are still being sent. With one buffer, copying a camera frame to the screen took 106 ms; with two it takes 32 ms, close to the 31 ms that SPI at 40 MHz needs for 320 x 240 pixels (see [10-track](../../examples/stackchan/10-track)).

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
tr_power_t p;
if (tr_power_read(&p) == ESP_OK) { ... } // the power chip as numbers: %, USB, voltages, state
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
| Microphones | ES7210 (`0x40`) on I2S0: MCLK G0, BCLK G34, WS G33, data G14. `esp_codec_dev` 1.5, 16 bits, gain 30 dB. `tr_mic_start(rate, channels)`: 1 gives MIC1, 2 gives MIC1 and MIC2 interleaved. Then `tr_mic_read()`. Both holes are on the front, side by side, about 30 mm apart | AW9523 P0_2 | Espressif CoreS3 BSP |
| Camera | GC0308 (`0x21`) on the DVP port: D0-D7 G39 G40 G41 G42 G15 G16 G48 G47, VSYNC G46, HREF G38, PCLK G45, no XCLK pin. `esp32-camera`, 320 x 240 RGB565, 3 frames in PSRAM (so a new one is ready when a slow reader asks, see 10-track). `tr_cam_start()`, then `esp_camera_fb_get()` | AXP2101 ALDO3 (3.3 V), BLDO1, BLDO2, AW9523 P1_0 | Espressif CoreS3 BSP |
| Side LEDs | 12 WS2812 on PY32 pin 13 (output, pull-up, push-pull). LED count in `0x24`, then 200 ms before the first show. Colors from `0x30` (RGB565, low byte first), all 12 in one I2C write; bit 6 of `0x24` shows them, written every time. LEDs 0-5 left, 6-11 right | The body's 5 V: AW9523 BUS_OUT_EN (P0_1), then BOOST_EN (P1_7), with P1_2 and P1_3 as inputs | StackChan factory firmware |

- The camera driver shares our I2C bus (`sccb_i2c_port`), so the bus is fixed on port 0 (`TR_I2C_PORT`).
- Examples that use the camera turn on the CoreS3's 8 MB PSRAM (`CONFIG_SPIRAM`, quad mode). On our robot it gives 8.3 MB free.
- **The LEDs need the body's 5 V.** Without BUS_OUT_EN and BOOST_EN they stay dark, even though the PY32 takes the colors. The boost draws from the battery. On a nearly empty battery (3.39 V at 390 mA) turning it on made the AXP2101 switch the robot off, three times. So `tr_led_show()` turns the 5 V on only while an LED is lit, and only with USB power or a battery at 3.6 V or more. It answers `ESP_ERR_INVALID_STATE` otherwise, and turns the 5 V off when all LEDs are off.
- `tr_board_reg_read()` and `tr_board_reg_write()` read and write single registers. `tr_web` serves them at `/regs` and `/reg` for debugging.
- **Keep the microphones at 16 kHz or less.** At 48 kHz (MCLK 12.3 MHz on G0) the robot stopped answering on the network for about 90 seconds, twice, and closing the analyzer did not help until a restart. **[inference]** The clock may disturb the Wi-Fi radio.
- Failed servo reads are not logged: callers poll the servos, and with the analyzer or camera running some reads miss their 10 ms reply time.
- `tr_lcd_blit()` copies camera frames to the screen through the DMA band, since the frame is in PSRAM.

## Sleep

`tr_sleep_start()` starts one small task (3 KB of stack). [tr_web](../tr_web) calls it, so every example that runs on the robot sleeps. Every 100 ms the task checks the touch (when no example did), once a second it reads the battery and the USB power, and it puts the robot to sleep or wakes it.

- **When:** after `CONFIG_TR_SLEEP_AFTER_S` seconds (60) with no activity, and only on battery. With USB power it never sleeps, and plugging USB in wakes it. `POST /sleep?after=N` changes the time until the next restart, and `0` turns sleep off.
- **Activity:** a touch on the screen or the head, a request that changes something (`POST /head`, `/leds`, `/face`), and a screen the example keeps open (`tr_sleep_activity()`). Reading the page (`GET`) is not activity, so an open page does not keep the robot awake.
- **Wake:** a touch on the screen or the head, `POST /sleep?wake=1`, a request that needs the screen or the head, or USB power. The touch that wakes the robot is not passed to the example (`tr_touch_read()` hides it until the finger lets go), so it does not open a screen by mistake.
- **Head touch:** only a zone that gets stronger counts. A hand that stays on the head (or a zone that stays high) does not keep the robot awake.

What goes off, in this order (wake does the opposite):

| Step | Bit | What |
| --- | --- | --- |
| Hooks | `0x02` | `tr_web`: torque and servo power off (PY32 VM_EN), side LEDs off, and so the body's 5 V boost too. They come back as they were. An example can add its own hook (`tr_sleep_add_hook()`) |
| Screen | `0x01` | Display Off (`0x28`) and Sleep In (`0x10`) to the panel, then the backlight off (DLDO1). Drawing does nothing while it is off. On wake: Sleep Out (`0x11`), 120 ms, Display On (`0x29`), then the backlight |
| Camera | `0x04` | The camera's power off: AXP2101 ALDO3, BLDO1, BLDO2 and AW9523 P1_0. Not while the camera runs. The light sensor shares it, so wake sets the light sensor up again |
| Motion | `0x08` | The BMI270 suspended (PWR_CTRL `0x7D` = 0, then PWR_CONF `0x7C` = 1) |
| CPU | `0x10` | The task gives back two power locks, so the CPU drops to 80 MHz and sleeps between ticks (automatic light sleep, `CONFIG_PM_ENABLE` and `CONFIG_FREERTOS_USE_TICKLESS_IDLE`). Wi-Fi stays connected in modem sleep. Only on the robot: without the power chip (as in the emulator) there is no light sleep |

**Pins in light sleep.** On the ESP32-S3, ESP-IDF turns every pin off during light sleep, to avoid a reset on wake when a pin gets a small electrostatic pulse (`ESP_SLEEP_GPIO_RESET_WORKAROUND`, which turns on `CONFIG_PM_SLP_DISABLE_GPIO`). Then the green LED next to the RST button blinked with every tick, because it follows GPIO0. `tr_sleep_start()` keeps the pins we use as they are, with `gpio_sleep_sel_dis()`: GPIO0, the internal I2C (G11, G12), the screen (G3, G35, G36, G37) and the servo UART (G6, G7). The other pins keep the workaround.

**While asleep**, `GET /sensors` says "off while the robot sleeps" for the motion sensor and the light sensor, instead of reading a chip that is off (the motion chip read -41 °C). `GET /head` shows the servos' last reading from before the sleep.

An example with a loop checks `tr_sleep_asleep()`, waits in `tr_sleep_wait_awake()` instead of drawing, and draws again after. `POST /sleep?now=1&steps=0x01` sleeps with only some steps, to measure each one.

### What we can't turn off

| Part | Why not |
| --- | --- |
| AXP2101 ALDO1 (1.8 V) and ALDO2 (3.3 V) | They feed the speaker amp (AW88298, `0x36`) and the microphones (ES7210, `0x40`). With ALDO2 off, the whole internal I2C bus stopped: every address answered the scan, and no chip could be read. It came back when we turned it on again. So they stay on |
| ALDO4 (3.3 V, the SD card) | Turning it off saved nothing we could measure (176 to 177 mA). We leave it as it is |
| AW9523 P0_2 (microphone and speaker enable) | Off made no difference we could measure |
| Screen touch (FT6336), head touch (Si12T), INA226 | They are needed to wake the robot and to measure it |
| Wi-Fi | The page and the updates must keep working |

### What each step saves

On our robot, on 2026-11-01, running [09-wifi](../../examples/stackchan/09-wifi) on battery (3.95 V, 82 %), with the face still. Each row adds one step to the row above (`POST /sleep?now=1&steps=...`). We waited 100 s after each change, then read the robot's own 20 s average once, so our requests barely count. The INA226 averages 64 samples per reading.

| State | Battery current | Saves |
| --- | --- | --- |
| Awake: screen at 20 %, servos powered (as after the boot move) | 96 mA | |
| + servos and LEDs off (`0x02`) | 65 mA | 31 mA |
| + screen off (`0x01`) | 56 mA | 9 mA |
| + camera power off (`0x04`) | 42 mA | 14 mA |
| + motion sensor suspended (`0x08`) | 43 to 47 mA (4 reads) | nothing we can measure |
| + light sleep and 80 MHz (`0x10`), all steps | 28 to 31 mA (4 reads) | about 15 mA |

So asleep the robot uses about 30 mA instead of 96 mA, 70 % less. With the 550 mAh battery that is about 18 hours instead of under 6. The robot's own guess said 4 h 50 min left awake and 12 to 16 h asleep. It only counts the charge that is left (82 %).

- With light sleep the current jumps: single reads go from 3 mA (asleep) to over 100 mA (Wi-Fi wakes up for the router's beacon). Only the average is useful.
- Servo power is the largest part. The servos draw about 30 mA even with the torque off, while VM_EN is on.
- The screen at 20 % costs only about 9 mA. The big saving was going from 100 % to 20 % (see [09-wifi](../../examples/stackchan/09-wifi)).
- The camera's power was on all the time, only for the light sensor. That is 14 mA.
- Waking takes about 0.72 s: 0.5 s for servo power and 0.12 s for the panel. Going to sleep takes 6 to 142 ms.
- The page still works while the robot sleeps. `GET /sleep` answered in 0.1 to 0.5 s (once 1.6 s).
- The sleep task had 1484 bytes of its 3 KB stack left after a sleep and a wake.

Before this, we measured 181 mA for 09-wifi at 20 %, with the old firmware. We don't know why it was so much higher. Maybe the Wi-Fi list was open, because it scans every 10 s.

## INA226

From TI's datasheet. Registers are 16 bits, high byte first. At the first read, `tr_board` writes its setup to the config register (`0x00` = `0x4727`): shunt and bus continuous, 1.1 ms each, average of 64, so a new value every 141 ms. The average smooths the Wi-Fi bursts, so a single read means more.

We write it every time the firmware starts, because the chip keeps its setup while the battery lasts. Once, after the I2C bus hung (see [Sleep](#what-we-cant-turn-off)), the INA226 gave the same reading (4.11 V, 77 mA) for minutes. Writing the setup again fixed it.

| Register | What | Scale |
| --- | --- | --- |
| `0x00` | Config: averaging, conversion times, mode | We write `0x4727` |
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
