# BALA2 Fire 06: balance

Keep BALA2 upright and move as little as possible. It starts from M5Stack's balancing app, and changes what our measurements and seven tests on the robot showed. It talks over Wi-Fi: you can watch it, change the gains while it runs, and send new firmware without a cable.

| | |
| --- | --- |
| **Robot** | BALA2 Fire (M5Stack Fire, ESP32) |
| **Level** | Advanced |
| **Emulator** | Yes, with a rough simulated robot, in the [browser](https://robotics.diegoromero.es/emulator/) (pick **ESP32**) or with Espressif QEMU. It checks the start and the safety stops, not the gains. No Wi-Fi there. |
| **Hardware used** | MPU6886 IMU, BALA2 base (motors, encoders), buttons, screen, Wi-Fi |

> **Safety:** the robot can move fast. Test it on the floor with at least 1 m free in front and behind, away from table edges. It is built to fall over rather than drive away (see [Safety](#safety)).

## What you learn

- Why a balancing law must fit the motors: a speed command is not a force command
- A PID on the tilt, plus an outer position loop
- How to make a motor dead band smooth, so the robot does not shake around zero
- Safety for a robot that moves: limits, a start check, stop rules, a watchdog, and a link check
- A Wi-Fi access point, a UDP link and over-the-air updates (OTA) with ESP-IDF

## Build

```bash
scripts/build.sh examples/bala2-fire/06-balance
```

Output: `out/bala2-fire-06-balance-esp32.bin` (merged image for the first flash and the emulator) and `examples/bala2-fire/06-balance/build/balance.bin` (app image for updates over Wi-Fi).

## Run it

**First time, over USB.** This also writes the new partition table, with two 3 MB app slots for updates:

```bash
esptool --chip esp32 write-flash 0x0 out/bala2-fire-06-balance-esp32.bin
```

Keep the robot lying still while it starts: it measures the gyro offset.

**Wi-Fi.** The robot makes its own network, `BALA2-xxxx`. The password is made at random on the first boot and saved in the robot. It is shown on the screen and the serial port, and it is not in the code. Join that network from your computer, then:

```bash
python3 examples/bala2-fire/06-balance/tools/bala2_link.py --log run.csv
```

It shows the telemetry (`t_ms,state,tilt,rate,speed,position,u,setpoint`, 50 times a second) and sends what you type as commands. It also sends a heartbeat. If the robot balances and hears nothing for 1 second, it stops. So closing the tool stops the robot.

**Updates over Wi-Fi.** With the robot lying down (it refuses while it balances):

```bash
python3 examples/bala2-fire/06-balance/tools/bala2_link.py --ota examples/bala2-fire/06-balance/build/balance.bin
```

The robot restarts and its network goes away for a few seconds. Your computer may join another known network meanwhile; turn off auto-join for that network while you test.

**Balance.**

1. Lay the robot down. Then stand it up on its wheels and hold it still and upright (tilt between -3 and +3 degrees on the screen) for 1 second.
2. The screen turns green: `balancing`. Let go gently, within 1 or 2 seconds.
3. To stop: button B, or type `stop`. Lay it down before the next try.

## How it works

### The control law

The tilt comes from a complementary filter (gyro plus accelerometer, as in [03](../03-imu-calibration)), with the gyro offset measured at every boot. 0 degrees is upright, and positive is leaning forward.

The base's motors behave as a **speed** command: the wheels reach the speed we ask for almost at once ([05](../05-speed-sweep)). So the law gives a wheel speed in m/s:

```
setpoint = -(kpos * position + kvel * speed)      degrees, at most 5
e        = tilt - setpoint                          rad
v_cmd    = kp*e + ki*SUM(e*dt) + kd*rate + kv*speed + kx*position
PWM      = v_cmd / 0.00118 + off * min(1, |v_cmd| / ramp)
```

- **PID on the tilt.** P and D catch the fall. I keeps the wheels speeding up while the robot still leans, until they get under it.
- **Position loop (`kpos`, `kvel`).** When the robot is ahead of where it started, or rolls forward, it is asked to lean back a little, so it comes back.
- **Motor line.** 0.00118 m/s per PWM unit is the slope we measured. `off` (31) jumps the motors' dead band. It grows from 0 over `ramp` m/s, so the gain near 0 stays low.
- The rate for the D term goes through a 15 Hz low-pass filter.

Start values, all can be changed while it runs:

| Name | Value | Unit | What it does |
| --- | --- | --- | --- |
| `kp` | 3.5 | m/s per rad | P on the tilt (about 52 PWM per degree) |
| `ki` | 10 | m/s per rad·s | I on the tilt |
| `kd` | 0.10 | m/s per rad/s | D on the tilt |
| `kv` | 0.3 | m/s per m/s | speed term, like M5Stack's |
| `kx` | 0.5 | m/s per m | position term, like M5Stack's, at most 0.094 m/s |
| `kpos` | 10 | degrees per m | position loop |
| `kvel` | 3 | degrees per m/s | position loop, speed part |
| `kt` | 0 | | slow trim of the balance point (off, see test 6) |
| `off` | 31 | PWM | dead band jump |
| `ramp` | 0.1 | m/s | how smoothly the jump comes in |
| `vmax` | 0.5 | m/s | speed limit (at most 0.55) |
| `max` | 500 | PWM | PWM limit (at most 600) |

Commands: the name and a value (`kd 0.07`), `?` to print them all, `stop`.

The gains came from a search on a simulated robot. We kept the sets that worked for many guesses of the center of mass height (5 to 16 cm), motor lag (30 to 150 ms) and extra delay (0 to 40 ms). Then we tuned them on the robot.

### Safety

The robot should rather fall over than drive away.

- Speed limit 0.5 m/s and PWM limit 500. Over Wi-Fi they can go up to 0.55 m/s and 600, never more.
- It only starts after it was lying down, and then held upright and still for 1 second. Then the speed limit ramps up over 1 second.
- It stops, and stays stopped, if the tilt goes past 25 degrees, if it moves more than 0.3 m from where it started, if it goes faster than 0.4 m/s for 0.3 s, if an I2C read fails, if the loop runs late, if the Wi-Fi link goes quiet for 1 second, or on button B.
- The base keeps the last speed it got ([04](../04-wheels)). So the robot sends 0 at boot, and repeats 0 while it waits. A stuck loop resets the chip (task watchdog, 1 s).
- It never starts if the IMU cannot be read at boot.
- Updates are refused while it balances, and the flash is erased one sector at a time, so the control loop is never stalled for long.

### Two cores

The control loop runs alone on core 1, at 200 Hz. The I2C bus is created on core 1 too, so its interrupt runs there. Wi-Fi, the screen and the log run on core 0.

## What we tried on the robot

Tests on 2026-10-30. Most tests changed one thing; test 7 changed two.

| Test | Law | Result |
| --- | --- | --- |
| 1 | M5Stack's: PWM from tilt, rate, speed, position | Did not shake, but leaned 5 degrees and rolled on at a fixed speed until the 0.3 m stop. A tilt only gave a speed, never the acceleration needed to get back under the robot. |
| 2 | Acceleration from LQR, added up into the speed | Shook at 12 Hz or more, then ran to the 0.3 m stop. Far too much gain for the real delays. |
| 3 | Same, softer gains | Once 18 s up, but shaking and drifting ±25 cm. |
| 4 | Plus a 15 Hz filter on the D term | The fast shake was gone, a 3 to 5 Hz rocking was left. |
| 5 | PID on the tilt (M5Stack's PD, plus I) | **40 s up, twice.** Fell when the slow balance point trim drifted to -5 degrees. |
| 6 | Trim off (`kt 0`) | Tilt held well (±1.5 degrees), but it crept forward to the 0.3 m stop. Still shaking. |
| 7 | Position loop, smoother dead band (`ramp` 0.03 → 0.1) | **48 s up until stopped by hand.** Tilt ±1.2 degrees, half the shaking, less than 18 cm of travel. |

What we learned:

- Fit the law to the actuator. These motors follow a speed, so a pure P on the tilt (M5Stack's law) leaves the robot leaning.
- A dead band jump that comes in too fast doubles the gain near zero, and the robot shakes around its balance point.
- Two slow integrators (the PID's I and the balance point trim) fight each other. A position loop with a P part is calmer.
- The simulated robot was good enough to show the problem in test 1, but not to set the final gains. The robot and live tuning over Wi-Fi did that.

## Expected output

On the robot (serial or Wi-Fi), lying down and then balancing:

```
gains kp 3.50 ki 10.0 kd 0.100 kv 0.30 kx 0.50 kpos 10.0 kvel 3.0 kt 0.000 off 31 ramp 0.100 vmax 0.50 max 500
t_ms,state,tilt,rate,speed,position,u,setpoint
28816,0,84.29,0.6,0.000,0.000,0,0.00
...
33720,1,-0.41,-4.3,0.007,0.173,75,-1.75
```

State: 0 waiting, 1 balancing, 2 stopped.

In QEMU, with the simulated robot:

```
W (...) balance: no IMU or base (emulator?), using a simulated robot
I (...) balance: start
sim: push
sim: done, balancing
```

## Try next

- A little shaking is left (about 3 to 4 changes of direction per second). Try `ramp 0.2`, then a lower `kd`.
- The robot sits about 8 cm ahead of where it started. Try a higher `kpos`.
- Move: add a speed setpoint to drive forward, and a turn term (left and right wheels get `v_cmd ± turn`).
