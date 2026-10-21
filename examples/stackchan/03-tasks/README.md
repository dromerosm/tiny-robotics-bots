# StackChan 03: tasks

Three FreeRTOS tasks that work together the way a robot does: one reads a sensor, one filters the data and one reacts. The sensor is a simulated distance sensor (like the Unit ToF4M), so it runs without hardware.

```
sensor task  --(queue)-->  filter task  --(event group)-->  alert task
 20 Hz                     moving average                   "obstacle!" / "path clear"
```

| | |
| --- | --- |
| **Robot** | StackChan (CoreS3, ESP32-S3) |
| **Level** | Intermediate |
| **Emulator** | ✅ Browser and native `esp-emu` |
| **Hardware used** | None (simulated sensor) |

## What you learn

- Creating tasks with priorities and stack sizes
- Running a task at a fixed rate with `vTaskDelayUntil()`
- Passing data between tasks with a queue
- Signaling a condition with an event group
- A moving-average filter with a ring buffer

## Build

```bash
scripts/build.sh examples/stackchan/03-tasks
```

Output: `out/stackchan-03-tasks-esp32s3.bin`

## Run it

- **Browser:** drop the `.bin` on https://robotics.diegoromero.es/emulator/.
- **Native emulator:** `scripts/smoke.sh examples/stackchan/03-tasks`
- **Robot:** `esptool --chip esp32s3 write-flash 0x0 out/stackchan-03-tasks-esp32s3.bin`

## Expected output

The simulated object comes closer and goes away again every 6 seconds:

```
StackChan tasks demo: sensor -> filter -> alert
I (992) tasks: distance avg=1058 mm (last 1137 mm)
I (1992) tasks: distance avg=1178 mm (last 1155 mm)
I (2992) tasks: distance avg=785 mm (last 694 mm)
I (3992) tasks: distance avg=293 mm (last 250 mm)
W (3992) tasks: obstacle closer than 300 mm!
I (4992) tasks: distance avg=176 mm (last 198 mm)
I (5542) tasks: path clear
```

The numbers change a little on each run because the sensor adds random noise.

## How it works

- **`sensor_task`** (priority 5) makes a reading every 50 ms and sends it to the queue. It has the highest priority so it never misses its period.
- **`filter_task`** (priority 4) averages the last 8 readings, sets or clears `OBSTACLE_BIT` and logs once per second.
- **`alert_task`** (priority 3) sleeps until `OBSTACLE_BIT` is set, warns, then waits until it clears.

Tuning lives at the top of `main.c`: `SAMPLE_PERIOD_MS`, `AVERAGE_WINDOW` and `OBSTACLE_MM`.

## Try next

- Replace `simulated_distance()` with real readings from a Unit ToF4M (VL53L1X, I2C address 0x29).
- Add a second sensor task and fuse both readings.
