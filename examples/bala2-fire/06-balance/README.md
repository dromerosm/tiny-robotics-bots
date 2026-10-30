# BALA2 Fire 06: balance

Keep BALA2 upright and move as little as possible. It starts from M5Stack's balancing app, and changes what our measurements and nine tests on the robot showed. It talks over Wi-Fi: you can watch it, change the gains while it runs, and send new firmware without a cable.

| | |
| --- | --- |
| **Robot** | BALA2 Fire (M5Stack Fire, ESP32) |
| **Level** | Advanced |
| **Emulator** | Yes, with a rough simulated robot, in the [browser](https://robotics.diegoromero.es/emulator/) (pick **ESP32**) or with Espressif QEMU. It checks the start and the safety stops, not the gains. No Wi-Fi there. |
| **Hardware used** | MPU6886 IMU, BALA2 base (motors, encoders), IP5306 power chip, buttons, screen, Wi-Fi |

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
setpoint = -(kpos * position + kvel * speed) + trim   degrees, at most 5
trim    += kt * position * dt                           slow, at most 5
e        = tilt - setpoint                              rad
v_cmd    = kp*e + ki*SUM(e*dt) + kd*rate + kv*speed + kx*position
turn     = -kh * heading                                m/s, at most 0.1
PWM      = (v_cmd -+ turn) / 0.00118 + off * min(1, |v| / ramp)   left, right
```

- **PID on the tilt.** P and D catch the fall. I keeps the wheels speeding up while the robot still leans, until they get under it.
- **Position loop (`kpos`, `kvel`, `kt`).** When the robot is ahead of where it started, or rolls forward, it is asked to lean back a little, so it comes back. The slow `kt` part removes what is left, so it returns to its start point.
- **Heading hold (`kh`).** The heading mixes the gyro Z axis (good over short times, does not care if a wheel slips) with the difference between the two encoders (does not drift). The left motor is a little faster, and pushes turn the robot too, so the right wheel gets a little more or less speed to keep the start heading. On our unit, gyro Z has the opposite sign to the encoders (`ys -1`).
- **Motor line.** 0.00118 m/s per PWM unit is the slope we measured. `off` (31) jumps the motors' dead band. It grows from 0 over `ramp` m/s, so the gain near 0 stays low.
- The rate for the D term goes through a 15 Hz low-pass filter. An optional notch filter (`nf`, `nq`) can remove one frequency from `v_cmd`; it is off by default (see test 12).

Start values, all can be changed while it runs:

| Name | Value | Unit | What it does |
| --- | --- | --- | --- |
| `kp` | 3.5 | m/s per rad | P on the tilt (about 52 PWM per degree) |
| `ki` | 10 | m/s per rad·s | I on the tilt |
| `kd` | 0.07 | m/s per rad/s | D on the tilt |
| `kv` | 0.3 | m/s per m/s | speed term, like M5Stack's |
| `kx` | 0.5 | m/s per m | position term, like M5Stack's, at most 0.094 m/s |
| `kpos` | 10 | degrees per m | position loop |
| `kvel` | 3 | degrees per m/s | position loop, speed part |
| `kt` | -0.05 | rad per m·s | slow return to the start point (with `kpos` and `kvel`; alone it fought the PID, test 6) |
| `kh` | 0.1 | m/s per rad | heading hold |
| `khr` | 0 | m/s per rad/s | heading rate damping |
| `ys` | -1 | | sign of the gyro Z axis |
| `nf`, `nq` | 0, 2 | Hz, | notch filter frequency (0 = off) and Q |
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
- It never starts on a nearly empty battery (IP5306 level under 25%), unless it is charging. Once, after about 4 hours of tests on battery, the robot switched off in the middle of a test.
- Updates are refused while it balances, and the flash is erased one sector at a time, so the control loop is never stalled for long.

### Two cores

The control loop runs alone on core 1, at 200 Hz. The I2C bus is created on core 1 too, so its interrupt runs there. Wi-Fi, the screen and the log run on core 0.

## What we tried on the robot

Tests on 2026-10-30. Most tests changed one thing; test 7 changed two. From test 7 on, the robot was also pushed by hand during the test.

| Test | Law | Result |
| --- | --- | --- |
| 1 | M5Stack's: PWM from tilt, rate, speed, position | Did not shake, but leaned 5 degrees and rolled on at a fixed speed until the 0.3 m stop. A tilt only gave a speed, never the acceleration needed to get back under the robot. |
| 2 | Acceleration from LQR, added up into the speed | Shook at 12 Hz or more, then ran to the 0.3 m stop. Far too much gain for the real delays. |
| 3 | Same, softer gains | Once 18 s up, but shaking and drifting ±25 cm. |
| 4 | Plus a 15 Hz filter on the D term | The fast shake was gone, a 3 to 5 Hz rocking was left. |
| 5 | PID on the tilt (M5Stack's PD, plus I) | **40 s up, twice.** Fell when the slow balance point trim drifted to -5 degrees. |
| 6 | Trim off (`kt 0`) | Tilt held well (±1.5 degrees), but it crept forward to the 0.3 m stop. Still shaking. |
| 7 | Position loop, smoother dead band (`ramp` 0.03 → 0.1) | **48 s up while we pushed it, until stopped by hand.** Tilt ±1.2 degrees, half the shaking, less than 18 cm of travel. |
| 8 | `ramp` 0.2 | The fine shaking felt a little smaller, but it swayed more and drifted to the 0.3 m stop twice. Back to 0.1. |
| 9 | `kd` 0.10 → 0.07 | **Best so far: 35, 45 and 54 s, with pushes.** Changes of direction down from 3.6 to about 2 per second, less than 15 cm of travel. Left alone, it stood fully still: tilt ±0.03 degrees, rate ±1 deg/s. |
| 10 | `kd` 0.05, position I (`kt` -0.05), heading measured only | Returns to its start point (mean position 1 to 2 cm, was 8). `kd` 0.05 shook more: back to 0.07. It turned ±19 degrees in 40 to 50 s, both ways. Gyro Z had the opposite sign to the encoders. |
| 11 | Heading hold on (`kh` 0.1, `ys` -1) | Heading held: +2 degrees at the end, 9.7 at most. The shaking is a resonance: 75% of it at 6 to 10 Hz, peak 8.5 Hz. After a push it took 2 to 10 s to stop. |
| 12 | Notch at 8.5 Hz, and a stiffer position loop (`kvel` 10, `kpos` 15, `kv` 0.1) | Much worse: strong rocking at 5.3 Hz. The 8.5 Hz peak was gone. Not finished: the battery ran out. |

What we learned:

- Fit the law to the actuator. These motors follow a speed, so a pure P on the tilt (M5Stack's law) leaves the robot leaning.
- A dead band jump that comes in too fast doubles the gain near zero, and the robot shakes around its balance point.
- Two slow integrators (the PID's I and the balance point trim) fight each other. A position loop with a P part is calmer.
- The simulated robot was good enough to show the problem in test 1, but not to set the final gains. The robot and live tuning over Wi-Fi did that.
- From test 11 data: the wheel follows the command with about 20 ms delay and an 80 ms lag, and with the robot's weight it reaches about 20% less speed than with no load (about 0.44 m/s at PWM 500). To stop after a push, the robot must first lean back. With 5 degrees of lean it can brake at about 0.86 m/s², so from 0.3 m/s the best stop is about 0.5 s and 5 to 8 cm.

## Expected output

On the robot (serial or Wi-Fi), lying down and then balancing:

```
gains kp 3.50 ki 10.0 kd 0.070 kv 0.30 kx 0.50 kpos 10.0 kvel 3.0 kt -0.050 off 31 ramp 0.100 kh 0.10 khr 0.00 ys -1 nf 0.0 nq 2.0 vmax 0.50 max 500
t_ms,state,tilt,rate,speed,position,u,setpoint,heading_enc,heading,yaw_rate,turn,battery,charging
9677,0,84.75,-0.3,0.000,0.000,0,0.00,0.0,0.0,0.0,0.000,0,1
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

- Stop faster after a push: raise `kvel` in small steps (5, 7) and check the shaking at each step.
- Find where the 8.5 Hz shake comes from (gear play, or the loop itself) before filtering it again.
- Move: add a speed setpoint to drive forward, and a turn term (left and right wheels get `v_cmd ± turn`).
