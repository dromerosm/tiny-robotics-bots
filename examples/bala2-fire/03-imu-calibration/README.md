# BALA2 Fire 03: IMU calibration

Calibrate the gyroscope every time the robot starts, then show the tilt angle on the Fire's screen. A gyroscope at rest never reads exactly 0. If you do not remove that offset, the angle you get from it drifts away, and the robot cannot balance.

| | |
| --- | --- |
| **Robot** | BALA2 Fire (M5Stack Fire, ESP32) |
| **Level** | Intermediate |
| **Emulator** | Yes, with simulated readings, in the [browser](https://robotics.diegoromero.es/emulator/) (pick **ESP32**) or with Espressif QEMU. No screen there: read the serial output. |
| **Hardware used** | MPU6886 IMU (I2C 0x68), screen |

## What you learn

- How to set up the MPU6886 and read all its data in one 14-byte I2C read
- How to measure the gyro offset: average many readings, and use the spread (standard deviation) to tell if the robot moved
- A complementary filter: mix the gyro (fast, but drifts) with the accelerometer (slow and noisy, but knows where "down" is)
- How to add a component from the ESP Component Registry (M5GFX, for the screen)

## Build

```bash
scripts/build.sh examples/bala2-fire/03-imu-calibration
```

Output: `out/bala2-fire-03-imu-calibration-esp32.bin`. The first build downloads M5GFX 0.2.36.

## Run it

- **Robot:** `esptool --chip esp32 write-flash 0x0 out/bala2-fire-03-imu-calibration-esp32.bin`. Do not touch the robot for the first 2 seconds after it starts. The screen shows the offsets, then the tilt angle, live. The same data goes to the serial port at 115200 baud.
- **Browser:** open https://robotics.diegoromero.es/emulator/, pick **ESP32 · BALA2 Fire** and drop the `.bin`. There is no IMU in the emulator, so the readings are simulated.
- **QEMU:** with ESP-IDF installed, run `idf.py -C examples/bala2-fire/03-imu-calibration qemu monitor`.

## Expected output

On our robot, on 2026-10-30, lying in its rest position:

```
I (594) imu: MPU6886 found at 0x68
calibrating, keep still (try 1 of 5)
calibrated after 1 tries: offset x=-151.9 y=12.5 z=67.8 raw, noise 12.3 12.2 12.0, tilt 82.57 deg, 39.9 C
tilt  +82.47 deg  accel  +82.54 deg  rate   -0.92 deg/s
tilt  +82.50 deg  accel  +82.66 deg  rate   -0.62 deg/s
```

In QEMU, the lines say `calibrated (sim)` and the tilt stays near -1.5 degrees.

## How it works

1. **Set up the IMU** with the same values as M5Stack's library: ±2000 deg/s (16.4 raw units per deg/s) and ±8 g (4096 raw units per g), 500 samples per second.
2. **Calibrate.** Read the gyro 1000 times, every 2 ms. The mean of each axis is its offset. If the standard deviation of any axis is 20 raw units or more (about 1.2 deg/s), the robot moved: show "It moved" and try again, up to 5 times.
3. **Filter.** Every 10 ms:

   ```
   rate  = (gyro_x - offset_x) / 16.4                 deg/s
   accel = atan2(ay, sqrt(ax² + az²))                  deg
   tilt  = 0.98 × (tilt + rate × 0.01) + 0.02 × accel
   ```

   BALA2 falls around the IMU's X axis. M5Stack's balancing app uses the same angle, with its balance point at -1.5 degrees by default. So 0 degrees should be close to upright. On our robot, lying down at rest, the tilt reads 82.6 degrees, which fits.

Each boot gives slightly different offsets, because the offset changes with temperature. That is why we measure it at each start and do not save it.

The screen also shows the offsets that the factory firmware saved on our unit (x -167, y 0, z 74), so you can compare them.

### Check on the robot

On 2026-10-30 we turned the robot by hand around the wheel axle: from lying on one side, through upright, to lying on the other side, and back, twice. Results:

- The tilt went from +84.5 to -82.8 degrees and passed through 0 near upright.
- The gyro and the accelerometer agreed on the direction in 63 of 66 moving samples. So the sign of `rate` is right.
- During fast moves the filter and the accelerometer differed by up to 13 degrees. While the robot moves, the accelerometer also feels that movement, not only gravity. That is why the filter trusts the gyro more.
- Back at rest, the tilt came back to 83.4 degrees, the same as the accelerometer. It did not drift.

## Try next

- Use this tilt instead of the simulation in [01-balance-sim](../01-balance-sim).
