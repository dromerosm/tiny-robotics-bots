# StackChan 06: status on the screen

StackChan shows its own status on the screen, one section per page: battery (with the time left and the sleep state), robot, network, touch and motion, and the room. **PREV** and **NEXT** at the bottom change the section, and the numbers change every second. It is the first example that draws on the real CoreS3 screen, and the first that sleeps: after a minute with no touch, on battery, the screen goes off, and a touch wakes it. Like every example that runs on the robot, it keeps the updates over Wi-Fi from [tr_ota](../../../components/tr_ota) and the page at `http://stackchan.local/app` from [tr_web](../../../components/tr_web).

| | |
| --- | --- |
| **Robot** | StackChan (CoreS3, ESP32-S3) |
| **Level** | Intermediate |
| **Emulator** | ✅ Native `esp-emu`, without the screen: it says "no screen" and goes on with Wi-Fi and `GET /`. It also goes to sleep after a minute, with nothing to turn off |
| **Hardware used** | Screen (ILI9342C or ILI9342E on SPI), screen and head touch, AXP2101 (backlight, power rails), AW9523 (screen reset), INA226 (battery), sensors, Wi-Fi |

## What you learn

- Powering the CoreS3 screen: backlight on the AXP2101, reset on the AW9523
- Telling the ILI9342C from the ILI9342E by the touch chip's firmware ID
- Drawing with `esp_lcd` over SPI and DMA, in bands of 16 rows
- Text with a 5x7 bitmap font, no graphics library
- Changing only some pins of an IO expander, so the other power lines stay as they are
- A shared status screen with sections and buttons (`tr_screen.h` in tr_web)
- Sleep: turn off what nobody uses and wake on a touch ([tr_board](../../../components/tr_board#sleep))

## Build

```bash
WIFI_ENV=../../../m5stack/wifi.env scripts/build.sh examples/stackchan/06-status
```

Output: `out/stackchan-06-status-esp32s3.bin` (merged image) and `examples/stackchan/06-status/build/status.bin` (app image for updates over Wi-Fi). See [tr_ota](../../../components/tr_ota) for `WIFI_ENV`.

## Run it

**On a robot that runs 05-ota or another tr_ota example**, over Wi-Fi:

```bash
curl -H "X-Token: <token>" --data-binary @examples/stackchan/06-status/build/status.bin http://stackchan.local/update
```

**First time, over USB:** `esptool --chip esp32s3 write-flash 0x0 out/stackchan-06-status-esp32s3.bin`. Then note the new update token on the serial port.

**Native emulator:** `scripts/smoke.sh examples/stackchan/06-status`

## Expected output

On the screen, the first section. The numbers are from our robot on battery, on 2026-11-01, put together from `GET /` that afternoon:

```
Battery 1/5                X
voltage    3.95 V
current    85 mA out
average    96 mA
charge     82 %, discharging
time left  4 h 50 min
USB        out
system     3.98 V
awake, sleeps in 42 s
PREV                  NEXT
```

The time left is green above 1 h, yellow above 20 min, and red below. The Network section has the update token on its last lines: only someone who holds the robot can see it. A tap on the top bar goes back to the first section.

After a minute with no touch, on battery, the screen goes off. A tap on the screen or a touch on the head turns it on again, and that tap does not change the section. With USB power it never sleeps.

## How it works

`tr_lcd_init()` in [tr_board](../../../components/tr_board) powers and starts the screen:

1. Sets the AW9523 pins P0_0 (touch reset) and P1_1 (screen reset) high. It reads each register first and changes only those two bits.
2. Reads the touch chip's firmware ID: `0x12` means ILI9342E, anything else ILI9342C.
3. Turns on the backlight: AXP2101 DLDO1 at 3.3 V.
4. Starts SPI3 at 40 MHz and the `esp_lcd_ili9341` driver, with extra setup commands for the ILI9342E.

Then `tr_status_open()` from [tr_web](../../../components/tr_web) draws the first section, and the loop reads the touch and calls `tr_status_update()` once a second. The screen pads each line with spaces, so the new text covers the old one with no flicker.

When the robot sleeps, `tr_sleep_asleep()` is true and the loop waits in `tr_sleep_wait_awake()`. Then it draws the first section again.

## What we saw on our robot (2026-10-31)

- The screen shows the status correctly.
- On battery, the current went from about 120 mA to about 240 mA with the screen on. The voltage read 3.22 V right after the start, against 3.71 V before. The battery may have been low. We have not measured this again yet.
- Once, `stackchan.local` did not resolve while the robot answered on its IP. If that happens, use the IP from the screen.

## Try next

- Show the face from [04-face](../04-face) on the real screen (that is [08-face](../08-face)).
- Add a section of your own to the status screen.
