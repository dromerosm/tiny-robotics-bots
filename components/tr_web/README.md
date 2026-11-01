# tr_web

The robot's web page, the head endpoints and the status screen, on [tr_ota](../tr_ota)'s server. Every StackChan example that runs on the robot uses it, from [05-ota](../../examples/stackchan/05-ota) to [09-wifi](../../examples/stackchan/09-wifi).

`tr_web_start()` also starts the sleep of [tr_board](../tr_board#sleep). When the robot sleeps, the servos and the side LEDs go off. When it wakes, they come back as they were. A request that moves the head or lights the LEDs wakes the robot first.

## Use it

Require it in `main/CMakeLists.txt` with `PRIV_REQUIRES tr_web`, and call it after `tr_ota_start()`:

```c
ESP_ERROR_CHECK(tr_ota_start());
ESP_ERROR_CHECK(tr_web_start());   // adds the endpoints, looks for the servos, starts sleep; nothing moves
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
| `GET /sleep` | Awake or asleep, the battery current, its average and the time left (the same lines as `GET /`) |
| `POST /sleep?now=1`, `wake=1`, `after=60` | Sleep now, wake now, or the seconds with no touch before sleep (`0`: never), until the next restart. Token needed. `now=1&steps=0x01` sleeps with only some steps, to measure each one (see [tr_board](../tr_board#sleep)) |
| `HEAD /camera.jpg` | 200 if the app has the camera, 404 if not. The page uses it to show or hide the camera card |
| `GET /regs` | AXP2101, AW9523 and PY32 registers, for debugging the power setup |
| `POST /reg?chip=aw&reg=0x03&val=0x83` | Write one register (`axp`, `aw` or `py32`). Token needed. Logged as a warning in `/log` first, in case it resets the robot |

The page shows the battery's average use, the time left and the sleep state in the **Battery** card, with **Sleep now** and **Wake up** buttons.

The page shows the facts in cards, as tables of names and values: **Battery**, **Robot**, **Network**, **Touch and motion** and **Room**, the same sections as the status screen on the robot (below). It builds them from the text of `GET /`, `/sensors` and `/face`, every second. Below them are the **Head** (servos and the sliders), the side LEDs, the camera, the I2C chips and the log. Tables are built with text nodes only, so nothing the robot sends becomes HTML.

It shows a **Face** card when the example answers `GET /face` (as 08-face does), with buttons to switch the robot's screen. The `views:` line of `GET /face` says which buttons to show. A `battery:` line there adds the average current and the time left to the battery card. A Wi-Fi scan in it (09-wifi) gets its own card, strongest first, with open networks in red.

**Ping** in the network card sends 5 requests one after the other to `/favicon.ico` (the 0.5 KB robot icon) and shows the fastest, average and slowest answer. It measures the Wi-Fi link and the web server together.

An example without the camera sets `CONFIG_TR_WEB_CAMERA=n` (as [09-wifi](../../examples/stackchan/09-wifi) does). Then there is no `GET /camera.jpg`, the camera code stays out of the app, and the page hides its camera card. The page asks with `HEAD /camera.jpg`: tr_web answers 200 with the camera and 404 without it, with no body. Before, the server's own 404 sent a short body, which an answer to `HEAD` must not have. Browsers then closed those connections with a reset, and `/log` filled with `error in recv : 104`.

## Many connections

Every request of the page gives up after 8 s, so one that never gets an answer does not stop the updates. The server closes the connection that waited longest when all of its sockets are in use (`lru_purge_enable` in tr_ota), because Safari opens several at once. Before, Safari on an iPad showed empty cards: the server could not accept a new connection (`error in accept (128)` in `/log`), and the page waited for that answer for ever. In a test with 3 pages polling and 8 idle connections open, all 300 requests got an answer.

## Errors in the log

When a request fails, tr_web answers with the error and writes one line to `/log`, such as `W tr_web: /head: 409, no servos`. A wrong or missing token gives `W tr_ota: /sleep: 403, wrong or missing X-Token`. The handlers then return `ESP_OK`, not `ESP_FAIL`. With `ESP_FAIL` the server closes the connection and only logs `uri handler execution failed`, with no reason.

## For an example that moves the head too

- `tr_web_head_power()` turns on servo power once. The page and the example share that state.
- `tr_web_last_move_us()` gives the time of the last move from the page or `curl`. Wait a while after it, so you don't move the head away from where someone just put it.
- `tr_web_servos()` is false without the body, as in the emulator.

## Status screen

`tr_screen.h` has the text screens of the robot: a title bar with X, 8 rows of text, and two buttons at the bottom. The status screen uses them: one section per page (Battery, Robot, Network with the update token, Touch and motion, Room), with PREV and NEXT. [06-status](../../examples/stackchan/06-status) shows it all the time. [08-face](../../examples/stackchan/08-face) and [09-wifi](../../examples/stackchan/09-wifi) open it with a tap on the mouth.

```c
#include "tr_screen.h"

tr_status_open();            // draw the first section
tr_status_update();          // new numbers: call it once a second
tr_status_tap(x, y);         // PREV or NEXT
if (tr_screen_on_bar(y)) {}  // a tap on the title bar: close the screen
```

The Battery section shows the time left and the sleep state ("awake, sleeps in 42 s"). 09-wifi also uses `tr_screen_row()` and `tr_screen_draw()` for its list of Wi-Fi networks.
