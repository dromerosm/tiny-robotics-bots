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
- Wi-Fi with ESP-IDF: join the strongest home network or make its own, mDNS (`bala2.local`), a UDP link, a page to control the robot from a phone, and over-the-air updates (OTA) with a token

## Build

```bash
WIFI_ENV=../../../m5stack/wifi.env scripts/build.sh examples/bala2-fire/06-balance
```

`WIFI_ENV` is optional. It points to a file outside the repo with up to two home networks:

```
SSID_01=first-network
PWD_SSID_01=its-password
SSID_02=second-network
PWD_SSID_02=its-password
```

`scripts/build.sh` passes these lines to the build, and the [bala2_link](../../../components/bala2_link) component reads them. The passwords end up in `sdkconfig`, `build/` and `out/`, which git ignores. ESP-IDF reads them only when `sdkconfig` is made, so delete `sdkconfig` after you change the file. Without `WIFI_ENV` (as in CI), the robot always makes its own network.

Output: `out/bala2-fire-06-balance-esp32.bin` (merged image for the first flash and the emulator) and `examples/bala2-fire/06-balance/build/balance.bin` (app image for updates over Wi-Fi).

## Run it

**First time, over USB.** This also writes the new partition table, with two 3 MB app slots for updates:

```bash
esptool --chip esp32 write-flash 0x0 out/bala2-fire-06-balance-esp32.bin
```

Keep the robot lying still while it starts: it measures the gyro offset.

**Wi-Fi.** At boot the robot scans for the networks from `WIFI_ENV` and joins the one with the strongest signal. If none is in range, or it can't join in 10 s, it makes its own network, `BALA2-xxxx`, at 192.168.4.1. Either way it is `bala2.local` (mDNS), and the screen shows the network and its address.

The robot also has a **key**: 12 random characters, made on the first boot and saved in the robot. It is shown on the screen and the serial port, and it is not in the code. It is the password of the robot's own network and the token for updates.

Be on the same network as the robot, then:

```bash
python3 examples/bala2-fire/06-balance/tools/bala2_link.py --log run.csv
```

The tool finds the robot as `bala2.local`, or at 192.168.4.1 (`--host` picks another address). It shows the telemetry (`t_ms,state,tilt,rate,speed,position,u,setpoint`, about 40 to 50 times a second) and sends what you type as commands. It also sends a heartbeat. If the robot hears nothing for 1 second, it stops the servo and keeps balancing in place. So closing the tool does not make the robot fall. To stop it, type `stop` or press button B.

Wi-Fi, the updates and the web server are not in this example. They are in the shared component [bala2_link](../../../components/bala2_link), so every BALA2 firmware can have them. The numbers you can change, and the 200 Hz samples, come from a second one, [bala2_mon](../../../components/bala2_mon). A new firmware is on trial after an update: if it fails three starts, the robot goes back to the one before (see the component's README).

**From a phone.** Open `http://bala2.local/` (or the address on the robot's screen) in the phone's browser, on the same network. The page shows the state, the tilt, the speed and the battery, and it has:

- a big **STOP** button (it also stops the servo),
- **−** and **+** to move the balance point by 0.25 degrees,
- a slider for the servo, from -100% to +100%, and a button to stop it,
- a box for every number you can change (the Tuning card), filled from the list the robot sends,
- a slider for the screen's backlight. It starts at 20%, which is easy to read indoors and saves battery. `bl 50` sets it over UDP. The value is not saved.

Under the controls it shows what the robot reads, twice a second:

- **Balance:** tilt rate, asked tilt, the balance point saved in flash, motor command, position, heading and turn rate.
- **Motion sensor (MPU6886):** acceleration and rotation on the three axes, raw, and the chip's temperature.
- **Wheels:** both encoder counts, and how many sensor reads failed since the start.
- **Buttons:** A, B and C light up while you press them.
- **Link and system:** network, address, signal, firmware build, the slot it runs from, time on, and free memory.

The last card lists what the robot has and this firmware does not use: the magnetometer, the microphone, the speaker, the microSD card, Bluetooth, Grove port A and the servo headers S1 to S3.

The page is one file, `main/app.html`, built into the firmware. It asks the robot for its state twice a second (`GET /state`, JSON) and sends the same commands as the tool (`POST /cmd`). Unlike the tool, it has no heartbeat: if you close the page, the robot and the servo go on as they were. So stop them first. `GET /info` says which firmware is running. The page's icon is the same file as on StackChan's page (`components/tr_ota/favicon.svg`).

**Updates over Wi-Fi.** With the robot lying down (it refuses while it balances), put the key from its screen in `BALA2_TOKEN`, not on the command line:

```bash
read -s BALA2_TOKEN && export BALA2_TOKEN
python3 examples/bala2-fire/06-balance/tools/bala2_link.py --ota examples/bala2-fire/06-balance/build/balance.bin
```

Without the right token the robot answers 401, and it does not stop the motors. After the update it restarts. On its own network, the network goes away for a few seconds, and your computer may join another known network meanwhile.

**Balance.**

1. Lay the robot down. Then stand it up on its wheels and hold it where it does not pull to either side. Keep it still for 1 second (within 1.5 degrees, and less than 15 degrees from upright).
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
v_cmd    = kp*e + ki*SUM(e*dt) + kd*rate
turn     = -kh * heading                                m/s, at most 0.1
PWM      = (v_cmd -+ turn) / 0.00118 + off * min(1, |v| / ramp)   left, right
```

- **PID on the tilt.** P and D catch the fall. I keeps the wheels speeding up while the robot still leans, until they get under it.
- **Position loop (`kpos`, `kvel`, `kt`).** When the robot is ahead of where it started, or rolls forward, it is asked to lean back a little, so it comes back. The slow `kt` part removes what is left, so it returns to its start point.
- **Heading hold (`kh`).** The heading mixes the gyro Z axis (good over short times, does not care if a wheel slips) with the difference between the two encoders (does not drift). The left motor is a little faster, and pushes turn the robot too, so the right wheel gets a little more or less speed to keep the start heading. On our unit, gyro Z has the opposite sign to the encoders (test 10).
- **Motor line.** 0.00118 m/s per PWM unit is the slope we measured. `off` (31) jumps the motors' dead band. It grows from 0 over `ramp` m/s, so the gain near 0 stays low.
- **Balance point.** It is not 0, and it moves a lot with the load: -1.5 degrees for the bare robot, +8 to +9.4 with the servo on the back, and -3.5 to +0.7 with it on top (test 24). So the robot takes it from your hand: the mean tilt of the second you hold it still before the start becomes the balance point. A person feels where the robot stops pulling, and in test 27 that was within 0.3 degrees of the true point. The position loop takes care of the rest. No saved value and no buttons are needed after the load changes. Buttons A and C, `bp` and the phone page still move it while it runs. The robot also learns the point while it balances (the mean tilt of each calm 10 s block, from the third block on) and saves the last one to flash when the run ends. That value is the start value after a restart and shows on the phone page, but the hand's angle replaces it at each start.
- **Tilt filter (`gw`).** The gyro's share of the tilt, per tick. The accelerometer also feels the wheels speed up, and a low `gw` lets that into the tilt. 0.995 shook less than 0.98 in tests 20 and 21, but not in test 22.
- **Wheel position loop (`kw`).** Off by default. It adds up the command into the place where the wheels should be, and pushes harder while they are behind. It cuts the slow sway but adds to the fast shake (tests 20 and 21).
- The rate for the D term goes through a low-pass filter (`rf`, 0.45, about 19 Hz).

Start values, all can be changed while it runs:

| Name | Value | Unit | What it does |
| --- | --- | --- | --- |
| `kp` | 3.5 | m/s per rad | P on the tilt (about 52 PWM per degree) |
| `ki` | 10 | m/s per rad·s | I on the tilt |
| `kd` | 0.07 | m/s per rad/s | D on the tilt |
| `rf` | 0.45 | share per tick | low-pass filter on the rate for D (0.32 is about 12 Hz, 0.45 about 19 Hz) |
| `kpos` | 18 | degrees per m | position loop |
| `kvel` | 8 | degrees per m/s | position loop, speed part |
| `kt` | -0.05 | rad per m·s | slow return to the start point (with `kpos` and `kvel`; alone it fought the PID, test 6) |
| `kh` | 0.1 | m/s per rad | heading hold |
| `off` | 31 | PWM | dead band jump |
| `ramp` | 0.3 | m/s | how smoothly the jump comes in |
| `vmax` | 0.5 | m/s | speed limit (at most 0.55) |
| `max` | 500 | PWM | PWM limit (at most 600) |
| `gw` | 0.995 | share per tick | gyro share in the tilt filter (0.98 follows the accelerometer in about 0.25 s, 0.995 in about 1 s) |
| `kw` | 0 | 1/s | wheel position loop (0 = off): up to 0.1 m/s more when the wheels are behind the command |

Commands: the name and a value (`kd 0.07`), `?` to print them all, `stop`. And four more:

- `bp 9.4`: set the balance point in degrees (from -15 to 15). The telemetry shows it as the setpoint while the robot waits.
- `servo 1700`: turn the servo on the base's S4 header (see [The servo](#the-servo)). It is not a test, and it stays until you change it.
- `rec 3`: send 3 s (at most 60) of every control tick, at 200 Hz, as `rec,...` lines. They come as the robot takes them, about 20 ms later, and `rec_end lost 0` closes them. The normal telemetry runs at 50 Hz, and its UDP packets arrive in bursts.
- `fall`: the next start is a fall test, not a balance. Hold the robot upright and still, and let go. The wheels hold their place, and it records 4 s at 200 Hz while the robot falls over. From how fast the tilt grows we get the robot's pendulum length (see [A physical model](#a-physical-model)).

The gains came from a search on a simulated robot. We kept the sets that worked for many guesses of the center of mass height (5 to 16 cm), motor lag (30 to 150 ms) and extra delay (0 to 40 ms). Then we tuned them on the robot.

### Safety

The robot should rather fall over than drive away.

- Speed limit 0.5 m/s and PWM limit 500. Over Wi-Fi they can go up to 0.55 m/s and 600, never more.
- It only starts after it was lying down, and then held upright and still for 1 second. Then the speed limit ramps up over 1 second.
- It stops, and stays stopped, if the tilt goes past 25 degrees, if it moves more than 0.3 m from where it started, if it goes faster than 0.4 m/s for 0.3 s, if the I2C bus fails three times in a row, if the loop runs late, or on button B.
- The base keeps the last speed it got ([04](../04-wheels)). So the robot sends 0 at boot, and repeats 0 while it waits. A stuck loop resets the chip (task watchdog, 1 s).
- It never starts if the IMU cannot be read at boot.
- If the computer goes quiet for 1 second, or the home Wi-Fi drops, the robot keeps balancing in place and the screen says `balancing, link lost`. It used to cut the motors, and a healthy robot fell over for it. The position loop holds it where it started, and the stop rules above still work. The robot joins the network again by itself.
- Wi-Fi power saving is off. With it on, the robot answered pings late (up to 1.7 s) and lost one packet in three. One second without a packet stops the balance, so a slow link can stop it for no reason.
- It never starts on a nearly empty battery (IP5306 level under 25%), unless it is charging. Once, after about 4 hours of tests on battery, the robot switched off in the middle of a test.
- Updates need the token and are refused while it balances or runs a fall test. The robot does not start while an update runs. The flash is erased one sector at a time, so the control loop is never stalled for long. Erasing and writing keeps core 0 busy for the whole update, so its idle task does not run, and the task watchdog reset the robot in the middle of the update (twice; the backtrace was in `esp_ota_write`). So during an update the watchdog stops watching core 0's idle task. The control task stays watched. An upload that stalls ends after 30 s (three receive timeouts in a row).
- On the home network, any computer on that network can send UDP commands and gets the telemetry (the robot answers whoever talked last). Commands can change gains and limits, but never start the robot: that needs your hands. Use the robot's own network if you don't trust the other devices.

### The servo

One servo of the M5Stack Servo Kit 360° is plugged into the base's **S4** header. It is a speed servo: it turns without end and cannot hold an angle. The base makes its 50 Hz signal; we only write the pulse width to the base (I2C 0x3A, register 0x36, in microseconds).

| Command | What the servo does |
| --- | --- |
| `servo 1500` | stops |
| `servo 1700` | turns one way; the further from 1500, the faster |
| `servo 1300` | turns the other way |

The value is kept between 500 and 2500. `?` shows it at the end of the gains line.

- The base keeps the last pulse it got, like the motor speed. So the robot sends 1500 at boot, and button B and `stop` send it too.
- If a computer was talking to the robot and goes quiet for 1 second, the servo stops. The balance goes on.
- Only the control task talks to the base. A command just stores the value, and the control task sends it on its next tick, when it changes.
- Each servo stops at a slightly different pulse. Ours stands still at 1500 (checked by eye). If yours creeps, try values near it.
- It works while the robot balances, but that is not measured: a turning servo shakes the body a little and uses the same battery as the motors (up to 750 mA if it is blocked).

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

Test on 2026-11-12: one run of 92 s with the start values, left alone and pushed by hand, with seven 200 Hz recordings.

| Test | Change | Result |
| --- | --- | --- |
| 19 | None, only measured | The shake peak was at 6.4 to 8.8 Hz, with 50 to 74% of the rate's power between 5 and 10 Hz. **The command was under PWM 55 for 33 to 67% of the time, and there the wheels mostly stand still:** 71% of the time under PWM 20, 57% at 20 to 40, 30% at 40 to 55, and 5% at 75 to 95. At the shake frequency the wheels gave 0.37 to 0.43 mm/s per PWM unit, a third of the measured line, with only 25 to 30 ms of delay. The accelerometer added 0.13 to 0.21 degrees (RMS) of false tilt between 5 and 10 Hz, to a true tilt of 0.23 to 0.64. Replaying the filter on the same data with `gw` 0.995 gave 0.08 to 0.10. The mean tilt was about -1.5 degrees. The run ended after a push: the command stayed at the speed limit for 0.35 s with only 2 to 4 degrees of tilt, and the 0.4 m/s stop ended it. |

Tests 20 and 21, the same day: the robot was left alone and a script changed one setting about every 25 s, with two 200 Hz recordings of 5 s each. The robot stood through each test without a stop (about 3 and 4 minutes). "Shake" is the rate between 5 and 10 Hz, "sway" is the tilt under 2 Hz, both RMS.

| Test | Setting | Shake (deg/s) | Sway (degrees) |
| --- | --- | --- | --- |
| 20, 21 | Start values of that day (`gw` 0.98), 10 recordings | 12.5 to 28.1, mean 16.9 | 0.82 to 1.05 |
| 20, 21 | `gw` 0.995, 6 recordings | 0.2 to 14.7, mean 9.2 | 0.06 to 0.87 |
| 20 | Dither of 30 and 50 PWM at 50 Hz, opposite on the two wheels | 13.7 to 21.3 | 0.68 to 0.96 |
| 20 | `kw` 10 | 25.0, 30.4 | 0.49, 0.52 |
| 20 | `kw` 20 | 42.1, 38.5 | 0.27, 0.31 |
| 21 | `kw` 10 with `kp` 2.5 | 17.2, 22.8 | 0.84, 0.87 |
| 21 | `kw` 10 with `kp` 2.5 and `gw` 0.995 | 14.4, 14.6 | 0.94, 0.82 |
| 21 | `kw` 20 with `kp` 2.5 and `gw` 0.995 | 13.4, 19.0 | 0.34, 0.62 |

- `gw` 0.995 became the start value after these two tests: 9.2 against 16.9 deg/s on average. Test 22 did not repeat it (see below), so the gain is not sure.
- The dither changed nothing, so it is not in the code.
- `kw` cuts the slow sway to a half or a third, but the shake grows with it. With a lower `kp` and `gw` 0.995, the shake is back to where it was and part of the sway gain is left. It stays off: no clear gain yet.
- Most of the robot's movement is the slow sway, about 1 degree under 2 Hz, not the shake.

Test 22, the same day, with `gw` 0.995 as the start value and the simpler code. Same method, three recordings per setting, one run of about 5 minutes without a stop. It was also the first start from lying down with `gw` 0.995, and it started well.

| Setting | Shake (deg/s) | Sway (degrees) | Travel (mm) |
| --- | --- | --- | --- |
| Start values, 9 recordings | 11.0 to 20.9, mean 15.4 | 0.83 to 1.00 | 8 to 22 |
| `kw` 5 | 21.8, 21.4, 17.5 | 0.86, 0.85, 0.86 | 11 to 13 |
| `kw` 10 | 20.7, 23.3, 22.3 | 0.59, 0.64, 0.67 | 7 to 9 |
| `kw` 15 | 28.0, 25.9, 26.4 | 0.50, 0.36, 0.23 | 5 to 6 |
| `kx` 0 and `kv` 0, with `kpos` 18 and `kvel` 8 | 14.9, 12.9, 8.8 | 0.93, 0.78, 0.87 | 5 to 10 |

- With `gw` 0.995 the shake was 15.4 deg/s on average, about the same as with 0.98 in tests 20 and 21. Over all three tests it is 12.9 (15 recordings) against 16.9 (10 recordings). So 0.995 is a little better at most.
- `kw` is the same trade as before: the sway and the travel go down, the shake goes up. `kw` 5 only adds shake.
- `kx` and `kv` can go: the position loop does the same job when `kpos` and `kvel` take their share. Not tested with pushes here, so the start values did not change yet (see test 23).

Test 23, the same day: pushes by hand, about one every 10 s. The script changed between the two settings every 25 s, and the person pushing did not know which one was on. All pushes were soft (0.09 to 0.17 m/s at the peak), and none stopped the robot.

| Setting | Pushes | Travel (mm) | Travel per m/s of push (mm) | Time to stop (s) |
| --- | --- | --- | --- | --- |
| `kx` 0.5, `kv` 0.3, `kpos` 10, `kvel` 3 | 5 | 82 | 565 | 2.6 (0.2 to 5.6) |
| `kx` 0, `kv` 0, `kpos` 18, `kvel` 8 | 4 | 61 | 453 | 3.4 (0.4 to 5.8) |

No clear difference with so few pushes, and the new setting was not worse. So `kx` and `kv` are gone, and `kpos` 18 and `kvel` 8 are the start values. Hard pushes, close to the 0.4 m/s stop, are not tested.

A check of the final firmware, started from lying down and left alone, six recordings: shake 10.5 to 17.8 deg/s (mean 13.9), sway 0.89 to 1.03 degrees, travel 7 to 14 mm. About the same as the old start values in test 22, with less travel.

That run ended after 80 s with one hard push by hand. The tilt jumped 3 degrees in 0.1 s, the command stayed at the speed limit (PWM 455, 0.50 m/s) and the tilt kept growing to 10 degrees. The 0.4 m/s stop ended it 0.4 s later, 19 cm ahead. The old start values failed the same way in test 19, so one push does not tell the two apart. The battery read 25% under load.

Test 24, the same day: the servo moved from the base to the body.

- **Servo on the back.** The robot ran away to the servo's side and fell in 1 to 2 s, every time. We tried balance points from -1.75 to +13.5 degrees, and none helped. We do not know the cause. The battery read 0% in most of these runs, and in some the charging cable was plugged in, so it may not be the servo alone.
- **The charging cable holds the robot up.** With the cable plugged in, the robot stood at about 6.5 degrees with the motors doing nothing. It looks like a balance, but the wheels moved less than 1 mm in a minute and the command stayed under the dead band. Do not test with the cable in.
- **Servo on top, cable out.** The robot balanced. It started at +1.5 degrees, the trim brought it to -3.2 in about 20 s, and the mean tilt over the last minute was -3.4 to -3.5. Shake 15.6 deg/s, travel 3 cm. The battery read 0% here too, so the shake needs a new check on a full battery. In the next run, 20 minutes later, the point was +0.7. We do not know what changed (the servo was not fixed in place, or the cable still hung from the robot).
- **Servo on the back again, cable out.** The robot balanced, with a mean tilt of 9.4 to 9.6 degrees over 50 s and shake of 12.8 to 16.9 deg/s. So the servo on the back was not the problem in the first runs. The cable and the battery were, and the start window too: the robot only starts within 3 degrees of its balance point, and that point was far off.
- **The robot restarts when the charging cable goes in or out.** It happened every time. The balance point then went back to its start value. This is why the robot now learns the point and saves it.

Test 25, the same day: Bluetooth. The ESP32 in the Fire has it, but this firmware does not use it. A test build with NimBLE (224 KB larger) scanned for 5 s with Wi-Fi on and found 33 devices (131 adverts, best signal -38 dBm), then advertised as `BALA2` with no error. Wi-Fi telemetry went on during the scan. We did not check from a phone that the advert is seen. The test code is not in the repo.

Test 27, the same day: start from the hand's angle. Servo on the back, cable out, no buttons. Two starts, held at 8.4 and 8.2 degrees. The mean tilt later in each run was 8.1 and 7.9, so the hand was 0.3 degrees off. The robot moved 0.5 cm and 4.3 cm in the first 3 s (8 cm in test 24, with a 4 degree error), and balanced for 177 s and 46 s. After the first run it saved 7.59 as the learned point.

Test 28, the same day: the link code moved to the `bala2_link` component. Four updates over Wi-Fi with it, and the page, the UDP link and the saved balance point all worked (after a restart the robot started with the saved 7.59). Three things came up:

- **A wrong key restarted the robot.** The first version left the connection open after a 401, the server read the 1 MB body as new requests, and the watchdog restarted the robot. Now the connection closes at once. A wrong key, no key and a bad image give 401, 401 and 400, with no restart.
- **One start as "sim", with no Wi-Fi.** After one update the robot did not find the base or the IMU and started the simulated robot, which has no Wi-Fi. A reset fixed it. A restart can cut an I2C transfer in two and leave the data line held. The start now tries 5 times, with a bus reset between tries.
- **Smaller.** The app went from 1,083,456 to 1,053,616 bytes: IPv6 is off, and only the components this firmware uses are built.

Test 29, the same day: the commands and the 200 Hz samples moved to the `bala2_mon` component. A firmware now names its numbers once, and the commands, the list, the state for the page and the boxes on the page come from that. The samples no longer wait in memory to be sent later: they go out as they come. Robot lying still, home network: a command and its answer took 13 ms (median), 86 ms at worst. A 3 s take gave 600 of 600 samples, none lost, each about 18 ms behind (median), 74 ms at worst. Every number now has a range: `kp 999` gives 20.

Test 30, the same day: what was left to check, with the two components in place. Servo on the back, cable out.

- **Rollback.** An app that fails on purpose at each start went in over Wi-Fi. 12 s later the robot was back on the app before it, alone.
- **It still balances.** Three runs of 26 s, 8 s and over 60 s, each ended by hand. Shake 13 deg/s in the long one, as before.
- **A bad hold at the start.** Held about 3 degrees back from the balance point (5.9 against 8 to 9), twice: the robot rolled 11 cm and came back. Held about 3 degrees forward (11.5): it rolled 12 cm and came back. So the start takes a hand that is 3 degrees off, at the cost of about 11 cm.
- **The link while it balances.** A command and its answer took 9 ms (median), 36 ms at worst. Four takes of 10 s at 200 Hz lost no sample.
- **The robot's own network.** An app built with no home networks made `BALA2-xxxx`. A phone joined it with the key from the screen and opened the page at `http://192.168.4.1/`. It was also the first time the page ran on a real phone. That app was made to stay on trial, and two presses of reset brought the normal one back, on the home network.

Test 31, the same day: a lost link no longer stops the balance. With the robot balancing, a computer sent a heartbeat for 8 s with the servo turning (1600), then went quiet for 20 s. The servo went back to 1500 within 2 s. The robot kept balancing all the time, within 3 cm of where it started, and the screen and the page said `balancing, link lost`. When the heartbeat came back, so did `balancing`.

Test 26, the same day: the lights. Two small green LEDs are lit on the lower part of the robot. We tried to drive them as SK6812 LEDs on GPIO 15, where a normal Fire has its two LED bars (10 LEDs), with state colors and a brightness command. They stayed green. So they are not those LEDs. The BALA2 base takes the place of the Fire's LED base, and the green lights belong to the base itself: its schematic shows a system LED on the base's own chip (STM32 pin PA15). The base's firmware is not published and no register for the lights is known, so this firmware cannot change them. The LED code is not in the repo.

Removed from the code on 2026-11-12, to keep the control loop small: the factory law (`law 1`, test 13), the notch filter (`nf` and `nq`, test 12), the gains `khr` and `ys`, which never changed, and M5Stack's two terms `kx` and `kv` (tests 22 and 23). They are in the git history, up to commit 17b83ac.

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
gains kp 3.50 ki 10.0 kd 0.070 rf 0.45 kpos 18.0 kvel 8.0 kt -0.050 off 31 ramp 0.300 kh 0.10 vmax 0.50 max 500 gw 0.995 kw 0.0 servo 1500
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

- The learned balance point is saved (test 27), but nobody checked that a restart loads it. With the start from the hand's angle it matters little. Decide if it stays.
- Start from the hand's angle with the servo on top, and with a hold more than 3 degrees off, to find where the start fails.
- Find why the robot restarts when the charging cable goes in or out.
- Stop faster after a push: raise `kvel` in small steps (10, 12) and check the shaking at each step.
- Push the robot hard, close to the 0.4 m/s stop, with the new position loop (`kpos` 18, `kvel` 8, no `kx` and `kv`). Test 23 only had soft pushes. To go back, use commit 17b83ac.
- The slow sway (about 1 degree, under 2 Hz) is most of the robot's movement, and it comes from the motors' dead band. `kw` cuts it but adds shake (tests 20 to 22). Find a way to get one without the other.
- Check `gw` again with more recordings: 0.995 was better in tests 20 and 21 and the same in test 22.
- Before test 23 the robot restarted once while it was handled (its clock went from 548 s back to 10 s). Nobody saw why. If it happens again, read the serial port to see if it was a crash or the watchdog.
- Cut the loop's delay at 6 to 8 Hz: predict the tilt about 20 ms ahead from the rate, or model the motor lag in the law. Small gain changes are done (tests 14 to 18).
- Test pushes with a fixed push (a pendulum or a weight on a string), so two settings get the same push.
- Move: add a speed setpoint to drive forward, and a turn term (left and right wheels get `v_cmd ± turn`).
