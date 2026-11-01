# StackChan 09: face and Wi-Fi networks

[08-face](../08-face) without the microphones and the camera. Like 08, the screen runs at 20 % and the robot sleeps after a minute with no touch, on battery (see [tr_board](../../../components/tr_board#sleep)). Tap the right eye and the robot lists the Wi-Fi networks it can see, with the signal and the security of each one. Tap the mouth for the status, one section per page, with a guess of how long the battery will last.

Everything else stays as in 08: updates over Wi-Fi ([tr_ota](../../../components/tr_ota)), `stackchan.local`, the page at `http://stackchan.local/app` ([tr_web](../../../components/tr_web)), the head that goes home at boot, and the status screen on the mouth.

| | |
| --- | --- |
| **Robot** | StackChan (CoreS3, ESP32-S3) |
| **Level** | Intermediate |
| **Emulator** | ✅ Native `esp-emu`, without screen and servos: it checks that Wi-Fi and the server start |
| **Hardware used** | Screen, touch, two head servos, PY32 (servo power, LEDs), sensors, Wi-Fi |

## What you learn

- Scanning for Wi-Fi networks while the robot stays on its own network, without blocking the face
- Dimming the backlight, and how much power that saves
- Leaving parts out of a shared component with a Kconfig option (`CONFIG_TR_WEB_CAMERA=n`)
- Guessing the time left on the battery from its charge and an average of the current

## Build

```bash
WIFI_ENV=../../../m5stack/wifi.env scripts/build.sh examples/stackchan/09-wifi
```

Output: `out/stackchan-09-wifi-esp32s3.bin` (merged image) and `examples/stackchan/09-wifi/build/wifi.bin` (app image for updates over Wi-Fi).

## Run it

Send it over Wi-Fi to a robot that runs another `tr_ota` example:

```bash
curl -H "X-Token: <token>" --data-binary @examples/stackchan/09-wifi/build/wifi.bin http://stackchan.local/update
```

| Tap | Screen | How to leave |
| --- | --- | --- |
| The mouth | **Status**, one section per page: Battery, Robot, Network (with the update token), Touch and motion, Room. **PREV** and **NEXT** change the section. The numbers change every second | A tap on the top bar |
| The right eye | **Wi-Fi**: one row per network, strongest first: name (13 characters), signal in dBm, security. A new scan every 10 s while it is open. **UP** and **DOWN** scroll | A tap on the top bar |

The colors show the security: red for open networks (and WEP, which is nearly open), yellow for old WPA, green for WPA3. WPA2 stays white.

| Request | What it does |
| --- | --- |
| `GET /face` | The views, looking around on or off, where the head points, and the last scan: name, signal, channel and security of each network. The battery's average and the time left are on `GET /` now, for every example |
| `POST /face?view=face\|status\|wifi` | Change the screen. Token needed |
| `POST /face?idle=0` or `idle=1`, `POST /face?blink=1` | As in 08. Token needed |

All of [07-head](../07-head)'s requests work too, except `/camera.jpg`. The page hides its camera card and the buttons for views this example does not have.

## Expected output

On our robot, on 2026-11-01, after `POST /face?view=wifi`:

```
$ curl http://stackchan.local/face
views: face status wifi
looking around: off
head: yaw 0.9, pitch 5.0 degrees
last view asked: wifi
Wi-Fi: 8 networks, 2 s ago
  MiFibra-7D0D                      -55 dBm  ch  1  WPA2
  MOVISTAR_7B00                     -79 dBm  ch  1  WPA2
  CCB                               -82 dBm  ch  6  WPA2
  ...
  MOVISTAR_BBC6                     -91 dBm  ch  1  WPA/2
```

## Power

The backlight is DLDO1 on the AXP2101. `tr_lcd_brightness(20)` sets it the way M5GFX does on the CoreS3: brightness 0..255 gives `(b + 641) >> 5` in register `0x99`, so 20 % is 2.6 V and 100 % is 3.3 V.

What we measured on the battery (INA226), with the face still, the head at rest and no USB:

| Backlight | Battery current |
| --- | --- |
| 100 % (3.3 V) | about 293 mA |
| 20 % (2.6 V) | about 184 mA |

So the screen at 20 % saves about 110 mA, a bit more than a third of what the robot uses at rest.

With the sleep from [tr_board](../../../components/tr_board#sleep) the robot uses about 30 mA after a minute with no touch, against 96 mA awake (measured on 2026-11-01 with this example; the table is in tr_board).

Without the camera the example also has no PSRAM, and the app is 1.02 MB instead of 1.12 MB. The light sensor still turns on the camera's power, because it shares that cable (see [tr_board](../../../components/tr_board)).

## Battery time left

This moved to [tr_board](../../../components/tr_board), so every example has it on `GET /`, the page and the status screen. StackChan's battery is 550 mAh (M5Stack's docs). The sleep task reads the current from the INA226 every second and keeps an average over about 20 s. The time left is 550 mAh times the charge from the power chip (`0xA4`), divided by that average. It starts again when you unplug USB.

It is a guess: it assumes the robot keeps using what it uses now, and that the power chip's % is right. A move of the head or the Wi-Fi list uses more. The status shows it in green above 1 h, yellow above 20 min, and red below. On our robot, at 86 % and 159 mA, it said 2 h 58 min.

## How it works

- **The scan.** `esp_wifi_scan_start(NULL, false)` does not block: the robot stays on its network and the face task goes on reading the touch. The scan takes about 2 s for all channels. `WIFI_EVENT_SCAN_DONE` sets a flag, and the face task takes up to 32 results and sorts them by signal. tr_ota's own scans (when it joins a network) are left alone.
- **The lists.** Status and Wi-Fi use the same screen code, `tr_screen.h` in [tr_web](../../../components/tr_web#status-screen): a title bar with X, a line under it, 8 rows, and two buttons at the bottom. Each row has its own color. Each status section fits in 8 rows, so PREV and NEXT change the whole page.
- **The rest.** As in [08-face](../08-face): the head goes home at boot, the eyes follow the servos, and nothing is drawn unless the face changes.

## Try next

- Show the signal of the network the robot is on in the status screen, and how it changes when you move it around the house.
