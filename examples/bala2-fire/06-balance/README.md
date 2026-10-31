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
- The rate for the D term goes through a low-pass filter (`rf`, 0.45, about 19 Hz). An optional notch filter (`nf`, `nq`) can remove one frequency from `v_cmd`; it is off by default (see test 12).

Start values, all can be changed while it runs:

| Name | Value | Unit | What it does |
| --- | --- | --- | --- |
| `kp` | 3.5 | m/s per rad | P on the tilt (about 52 PWM per degree) |
| `ki` | 10 | m/s per rad·s | I on the tilt |
| `kd` | 0.07 | m/s per rad/s | D on the tilt |
| `rf` | 0.45 | share per tick | low-pass filter on the rate for D (0.32 is about 12 Hz, 0.45 about 19 Hz) |
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
| `ramp` | 0.3 | m/s | how smoothly the jump comes in |
| `vmax` | 0.5 | m/s | speed limit (at most 0.55) |
| `max` | 500 | PWM | PWM limit (at most 600) |

Commands: the name and a value (`kd 0.07`), `?` to print them all, `stop`. And three for tests:

- `rec 3`: record 3 s (at most 5) of every control tick, at 200 Hz, then send it as `rec,...` lines. The normal telemetry runs at 50 Hz, and its UDP packets arrive in bursts.
- `fall`: the next start is a fall test, not a balance. Hold the robot upright and still, and let go. The wheels hold their place, and it records 4 s at 200 Hz while the robot falls over. From how fast the tilt grows we get the robot's pendulum length (see [A physical model](#a-physical-model)).
- `law 1`: balance with M5Stack's factory law instead of ours, to compare them with the same telemetry and limits; `law 0` goes back. See `m5law.h` for what is the same and what is not.

The gains came from a search on a simulated robot. We kept the sets that worked for many guesses of the center of mass height (5 to 16 cm), motor lag (30 to 150 ms) and extra delay (0 to 40 ms). Then we tuned them on the robot.

### Safety

The robot should rather fall over than drive away.

- Speed limit 0.5 m/s and PWM limit 500. Over Wi-Fi they can go up to 0.55 m/s and 600, never more.
- It only starts after it was lying down, and then held upright and still for 1 second. Then the speed limit ramps up over 1 second.
- It stops, and stays stopped, if the tilt goes past 25 degrees, if it moves more than 0.3 m from where it started, if it goes faster than 0.4 m/s for 0.3 s, if an I2C read fails, if the loop runs late, if the Wi-Fi link goes quiet for 1 second, or on button B.
- The base keeps the last speed it got ([04](../04-wheels)). So the robot sends 0 at boot, and repeats 0 while it waits. A stuck loop resets the chip (task watchdog, 1 s).
- It never starts if the IMU cannot be read at boot.
- It never starts on a nearly empty battery (IP5306 level under 25%), unless it is charging. Once, after about 4 hours of tests on battery, the robot switched off in the middle of a test.
- Updates are refused while it balances, and the flash is erased one sector at a time, so the control loop is never stalled for long. The update loop also pauses for 1 tick after each 4 KB block. Without it, on fast Wi-Fi, core 0's idle task never ran and the watchdog reset the robot in the middle of the update.

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

Tests on 2026-10-31, with the 200 Hz recorder (`rec 5`). Each setting got two windows of 5 s, with the robot left alone. Two windows with the same gains differ by about ±25%, so small improvements are hard to see.

| Test | Change | Result |
| --- | --- | --- |
| 13 | Our law against the factory law (`law 1`) | The factory law could not stand: twice it leaned forward 2 to 6 degrees, sped up to 0.34 m/s, and the 0.3 m stop ended it after about 2 s. So we compare our law only with itself. |
| 14 | `kd` 0.05, 0.07, 0.09 and `kp` 2.5, 4.5 | The shake peak moved with the gains: 6 Hz at `kd` 0.05, 8 Hz at 0.07, 9 Hz at 0.09. So it comes from the loop, not from the gears. At `kp` 4.5 it shook most (54 deg/s), at 2.5 least. |
| 15 | `kp` 2.0 to 3.5, then pushes at 2.5 and 3.5 | Lower `kp` halves the 4 to 10 Hz part and the motor reversals, but the robot sways more (tilt 1.5 degrees at 2.0, 0.85 at 3.5). After a push, `kp` 2.5 took about 1 s longer to stop. We kept 3.5. |
| 16 | `rf` 0.2, 0.32, 0.45, 0.6 (new command) | 0.2 shook most (up to 47 deg/s). 0.45 shook least (20 to 24 deg/s, 7 to 11 reversals per second, was 13 to 15). 0.6 was no better. The peak stayed at 7.5 to 8.5 Hz. |
| 17 | `ramp` 0.1, 0.2, 0.3, 0.5 | A softer dead band jump moved the peak down to 5.5 to 7 Hz and halved the motor reversals. The robot swayed a little more (tilt 1.15 against 1.0 degrees). No drift to the 0.3 m stop this time (compare test 8). |
| 18 | `rf` 0.45 with `ramp` 0.3, against the old values, with pushes | **Better on all counts:** 41 against 71 deg/s of shaking, 6 against 17 cm of travel after a push, and it stopped sooner. The pushes were by hand, and those on the old values were a little stronger, so the exact numbers are not reliable. These are now the start values. |

What we learned:

- Fit the law to the actuator. These motors follow a speed, so a pure P on the tilt (M5Stack's law) leaves the robot leaning.
- A dead band jump that comes in too fast doubles the gain near zero, and the robot shakes around its balance point.
- Two slow integrators (the PID's I and the balance point trim) fight each other. A position loop with a P part is calmer.
- The simulated robot was good enough to show the problem in test 1, but not to set the final gains. The robot and live tuning over Wi-Fi did that.
- The shake (tests 14 to 18) is a mode of the loop: its frequency follows `kp` and `kd`. Every change that lowers the gain near zero speed (`kp`, `ramp`) or cuts the delay (`rf`) helps a little, but none removes it. Near zero, the dead band jump adds about 37% gain, and that is where the motors reverse 10 to 15 times per second.
- From test 11 data: the wheel follows the command with about 20 ms delay and an 80 ms lag, and with the robot's weight it reaches about 20% less speed than with no load (about 0.44 m/s at PWM 500). To stop after a push, the robot must first lean back. With 5 degrees of lean it can brake at about 0.86 m/s², so from 0.3 m/s the best stop is about 0.5 s and 5 to 8 cm.

## A physical model

The control needs one number above all: the pendulum length L = I / (m·d), where m is the mass, d the height of the center of mass over the axle, and I the moment of inertia around the axle. For small tilts, a robot whose wheels do not move falls as

```
tilt'' = (g / L) · tilt      so      tilt(t) ≈ tilt0 · e^(λt),   λ = √(g / L)
```

The `fall` test measures λ from the 200 Hz recording, and so L. A scale and a pencil give m and d (lay the robot on the pencil and find where it balances). A string pendulum gives I as a check: hang the robot by a point at a known distance from its center of mass and time the swing with the IMU.

### Fall test results

On 2026-10-31 we ran `fall` four times over Wi-Fi, with a full battery. One run did not count: the robot was held for the whole 4 s, so the recording ended before it fell. Let go as soon as the screen says "let go". The other three:

| Run | Fell | λ (accel fit) | λ (energy fit) | L = g/λ² |
|---|---|---|---|---|
| 1 | forward | 10.40 1/s | 10.36 1/s | 91 mm |
| 3 | back | 10.31 1/s | 9.83 1/s | 92 mm |
| 4 | forward | 10.43 1/s | 10.34 1/s | 90 mm |

**λ = 10.4 1/s and L = 91 mm**, the same on both sides. The accel fit is a straight line through tilt'' against sin(tilt), for tilts from 3 to 35 degrees. The energy fit uses tilt'² = 2λ²(1 − cos tilt). It assumes the fall starts at exactly 0 degrees, so it is a little off when the true balance point is not 0. We use the accel fit. The first 0.1 s after release grows more slowly, because the hand is still on the robot. We leave it out.

The encoders did not move during the falls (x stayed at 0). The gearbox holds the wheels to the body, so the whole robot rolls over like one rigid body and turns around the wheel's contact point, not around the axle. So this L is measured from the floor, not from the axle. That is the number the control needs. To split it into m, d and I we still need the scale and the pencil.

What it means for the control: a tilt grows e times every 96 ms and doubles every 67 ms. The motors need about 20 ms to start and have an 80 ms lag, which is the same order of time. So the loop has very little phase margin. Tests 14 to 18 confirmed it: the 6 to 8 Hz shake comes from the loop, not from gear play.

## Expected output

On the robot (serial or Wi-Fi), lying down and then balancing:

```
gains kp 3.50 ki 10.0 kd 0.070 rf 0.45 kv 0.30 kx 0.50 kpos 10.0 kvel 3.0 kt -0.050 off 31 ramp 0.300 kh 0.10 khr 0.00 ys -1 nf 0.0 nq 2.0 law 0 vmax 0.50 max 500
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
- Cut the loop's delay at 6 to 8 Hz: predict the tilt about 20 ms ahead from the rate, or model the motor lag in the law. Small gain changes are done (tests 14 to 18).
- Test pushes with a fixed push (a pendulum or a weight on a string), so two settings get the same push.
- Move: add a speed setpoint to drive forward, and a turn term (left and right wheels get `v_cmd ± turn`).
