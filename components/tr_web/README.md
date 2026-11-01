# tr_web

The robot's web page and the head endpoints, on [tr_ota](../tr_ota)'s server. [07-head](../../examples/stackchan/07-head) and [08-face](../../examples/stackchan/08-face) use it.

## Use it

Require it in `main/CMakeLists.txt` with `PRIV_REQUIRES tr_web`, and call it after `tr_ota_start()`:

```c
ESP_ERROR_CHECK(tr_ota_start());
ESP_ERROR_CHECK(tr_web_start());   // adds the endpoints, looks for the servos; nothing moves
```

## Endpoints

| Request | What it does |
| --- | --- |
| `GET /app` | The page: status, head servos, I2C chips, log, sliders to move the head. A browser that opens `/` lands here |
| `GET /head` | Both servos: angle, raw position, zero, torque, voltage, temperature |
| `POST /head?yaw=20&pitch=45&ms=1000` | Move. Token needed. `&hold=1` keeps the torque on |
| `POST /head/off` | Torque and servo power off. Token needed |
| `POST /head/zero?yaw=436&pitch=585` | Save new zero positions (raw 0 to 1023). Token needed |
| `GET /i2c` | The chips that answer on the internal I2C bus |
| `GET /sensors` | Head touch, screen touch, motion, clock, power chip, light and chip temperature (from `tr_board`). The page shows it every second |
| `GET /leds`, `POST /leds?color=ff8800&side=left` | The side LEDs: `left`, `right` or `all`, color as RRGGBB, `000000` is off. Token needed to change them. `409` if the battery is too low for their 5 V: plug in USB |
| `GET /camera.jpg` | One camera frame as JPEG (320 x 240). Starts the camera the first time. Not there with `CONFIG_TR_WEB_CAMERA=n` |
| `GET /regs` | AXP2101, AW9523 and PY32 registers, for debugging the power setup |
| `POST /reg?chip=aw&reg=0x03&val=0x83` | Write one register (`axp`, `aw` or `py32`). Token needed. Logged as a warning in `/log` first, in case it resets the robot |

The page shows the facts in cards, as tables of names and values: **Battery**, **Robot**, **Network**, **Touch and motion** and **Room**, the same sections as the status screen of [09-wifi](../../examples/stackchan/09-wifi). It builds them from the text of `GET /`, `/sensors` and `/face`, every second. Below them are the **Head** (servos and the sliders), the side LEDs, the camera, the I2C chips and the log. Tables are built with text nodes only, so nothing the robot sends becomes HTML.

It shows a **Face** card when the example answers `GET /face` (as 08-face does), with buttons to switch the robot's screen. The `views:` line of `GET /face` says which buttons to show. A `battery:` line there adds the average current and the time left to the battery card. A Wi-Fi scan in it (09-wifi) gets its own card, strongest first, with open networks in red.

**Ping** in the network card sends 5 requests one after the other to `/favicon.ico` (the 0.5 KB robot icon) and shows the fastest, average and slowest answer. It measures the Wi-Fi link and the web server together.

An example without the camera sets `CONFIG_TR_WEB_CAMERA=n` (as [09-wifi](../../examples/stackchan/09-wifi) does). Then there is no `/camera.jpg`, the camera code stays out of the app, and the page hides its camera card: it asks with `HEAD /camera.jpg` and gets 404 instead of 405.

## For an example that moves the head too

- `tr_web_head_power()` turns on servo power once. The page and the example share that state.
- `tr_web_last_move_us()` gives the time of the last move from the page or `curl`. Wait a while after it, so you don't move the head away from where someone just put it.
- `tr_web_servos()` is false without the body, as in the emulator.
