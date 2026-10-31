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

The page shows a **Face** card when the example answers `GET /face` (as 08-face does).

## For an example that moves the head too

- `tr_web_head_power()` turns on servo power once. The page and the example share that state.
- `tr_web_last_move_us()` gives the time of the last move from the page or `curl`. Wait a while after it, so you don't move the head away from where someone just put it.
- `tr_web_servos()` is false without the body, as in the emulator.
