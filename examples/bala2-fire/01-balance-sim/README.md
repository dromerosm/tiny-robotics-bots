# BALA2 Fire 01: balance-sim

A PID loop that balances a simulated two-wheeled robot. The physics of an inverted pendulum on wheels runs on the ESP32, so you can study the control loop before you touch the motors. Every 5 seconds the robot gets a push, and the loop has to catch it.

| | |
| --- | --- |
| **Robot** | BALA2 Fire (M5Stack Fire, ESP32) |
| **Level** | Intermediate |
| **Emulator** | Yes, in the [browser](https://robotics.diegoromero.es/emulator/) (pick **ESP32**) or with Espressif QEMU |
| **Hardware used** | None (simulated physics) |

## What you learn

- The model of a self-balancing robot: `theta'' = (g·sin θ − a·cos θ) / L`
- How P, I and D each help: `kp` beats gravity, `kd` damps the motion, `ki` removes offsets
- A fixed-rate 200 Hz control loop with `vTaskDelayUntil()` and 1 ms ticks
- Using a shared component ([tr_pid](../../../components/tr_pid)) from an example

## Build

```bash
scripts/build.sh examples/bala2-fire/01-balance-sim
```

Output: `out/bala2-fire-01-balance-sim-esp32.bin`

## Run it

- **Robot:** `esptool --chip esp32 write-flash 0x0 out/bala2-fire-01-balance-sim-esp32.bin`, then open a serial monitor at 115200 baud. The motors do not move: everything is simulated.
- **Browser:** open https://robotics.diegoromero.es/emulator/, pick **ESP32 · BALA2 Fire** and drop the `.bin`. It runs in QEMU, inside your browser.
- **QEMU:** with ESP-IDF installed, run `idf.py -C examples/bala2-fire/01-balance-sim qemu monitor`. See [Espressif's QEMU guide](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/tools/qemu.html).

## Expected output

Ten lines per second of telemetry, plus events. Excerpt:

```
BALA2 balance simulation: start tilt 10.0 deg
t=  0.00 s  tilt=  9.93 deg  rate= -24.79 deg/s  wheel accel=  5.24 m/s^2
t=  0.11 s  tilt=  2.03 deg  rate= -51.85 deg/s  wheel accel= -0.26 m/s^2
t=  0.20 s  tilt= -0.11 deg  rate=  -4.42 deg/s  wheel accel= -0.13 m/s^2
t=  0.30 s  tilt= -0.20 deg  rate=   0.30 deg/s  wheel accel= -0.04 m/s^2
I (...) balance: balanced at t=1.15 s
W (...) balance: push forward!
t=  5.01 s  tilt=  0.38 deg  rate=  86.17 deg/s  wheel accel= -0.01 m/s^2
t=  5.11 s  tilt=  0.79 deg  rate= -15.54 deg/s  wheel accel=  0.06 m/s^2
t=  5.21 s  tilt= -0.02 deg  rate=  -2.52 deg/s  wheel accel= -0.05 m/s^2
I (...) balance: balanced at t=6.13 s
```

These values come from the same model, gains and `tr_pid` code compiled on a PC. The ESP32 uses the same float math, so the numbers should match closely.

## Tuning

The gains are in `balance_task()`:

| Gain | Value | Effect if you raise it |
| --- | --- | --- |
| `kp` | 30 | Stronger push back toward upright. Must stay above g (9.81) or the robot falls. |
| `ki` | 5 | Removes a steady lean faster, but too much makes it swing. |
| `kd` | 1.5 | More damping. Too much makes it slow and jittery with real sensor noise. |

Try `kp = 9` (below g) to see it fall, or `kd = 0` to see it oscillate.

## From simulation to the real robot

Replace `simulate()` with:

1. the tilt angle from the Fire's MPU6886 IMU (fuse gyroscope and accelerometer, for example with a complementary filter), and
2. a speed command to the BALA2 base over I2C (address 0x3A).

Keep the loop rate and start with lower gains: real motors, gears and sensor noise behave differently from the model.

## Try next

- Add a cascade loop that also keeps the wheel position, so the robot does not drift.
- Log the data and plot it to tune the gains.
