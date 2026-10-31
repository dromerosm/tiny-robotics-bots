# StackChan 06: status on the screen

StackChan shows its own status on the screen, new every second: firmware, app slot, uptime, last restart, Wi-Fi, battery and free memory. It is the first example that draws on the real CoreS3 screen. Like every example that runs on the robot, it keeps the updates over Wi-Fi from [tr_ota](../../../components/tr_ota), so `http://stackchan.local/` shows the same status as text.

| | |
| --- | --- |
| **Robot** | StackChan (CoreS3, ESP32-S3) |
| **Level** | Intermediate |
| **Emulator** | ✅ Native `esp-emu`, without the screen: it says "no screen" and goes on with Wi-Fi and `GET /` |
| **Hardware used** | Screen (ILI9342C or ILI9342E on SPI), AXP2101 (backlight), AW9523 (screen reset), INA226 (battery), Wi-Fi |

## What you learn

- Powering the CoreS3 screen: backlight on the AXP2101, reset on the AW9523
- Telling the ILI9342C from the ILI9342E by the touch chip's firmware ID
- Drawing with `esp_lcd` over SPI and DMA, in bands of 16 rows
- Text with a 5x7 bitmap font, no graphics library
- Changing only some pins of an IO expander, so the other power lines stay as they are

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

On the screen, under a blue title bar "StackChan status" (network hidden):

```
fw   v0.2.0-12-gcd9c7d2-dirt
slot ota_0 valid
up   0:00:09
boot restart from software
wifi <network>
     -42 dBm, channel 11
ip   192.168.1.45
bat  3.22 V 240 mA out
mem  225 KB free
```

Green means good (the app is valid, Wi-Fi is up, the signal is strong). Yellow is a warning (the app is on trial, there is no Wi-Fi, the signal is weak). The battery turns red below 3.4 V. Lines longer than 26 characters are cut.

## How it works

`tr_lcd_init()` in [tr_board](../../../components/tr_board) powers and starts the screen:

1. Sets the AW9523 pins P0_0 (touch reset) and P1_1 (screen reset) high. It reads each register first and changes only those two bits.
2. Reads the touch chip's firmware ID: `0x12` means ILI9342E, anything else ILI9342C.
3. Turns on the backlight: AXP2101 DLDO1 at 3.3 V.
4. Starts SPI3 at 40 MHz and the `esp_lcd_ili9341` driver, with extra setup commands for the ILI9342E.

Then the loop draws each line with `tr_lcd_text()`. It pads each line with spaces, so the new text covers the old one with no flicker.

## What we saw on our robot (2026-10-31)

- The screen shows the status correctly.
- On battery, the current went from about 120 mA to about 240 mA with the screen on. The voltage read 3.22 V right after the start, against 3.71 V before. The battery may have been low. We have not measured this again yet.
- Once, `stackchan.local` did not resolve while the robot answered on its IP. If that happens, use the IP from the screen.

## Try next

- Turn the backlight down to save battery (`AXP_DLDO1_V` takes 0.5 to 3.3 V).
- Show the face from [04-face](../04-face) on the real screen.
