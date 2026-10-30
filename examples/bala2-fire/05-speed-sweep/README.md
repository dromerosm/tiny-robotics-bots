# BALA2 Fire 05: speed sweep

Measure the real speed of the wheels. The base takes a speed value from -1023 to 1023, but that value is a PWM duty, not a speed. This example steps both wheels through many values and uses the encoders to measure the turns per minute (rpm) at each one. It shows the lowest value that makes a wheel move, the top speed, and whether the curve is a straight line.

| | |
| --- | --- |
| **Robot** | BALA2 Fire (M5Stack Fire, ESP32) |
| **Level** | Intermediate |
| **Emulator** | Yes, with a simple motor model, in the [browser](https://robotics.diegoromero.es/emulator/) (pick **ESP32**) or with Espressif QEMU |
| **Hardware used** | BALA2 base (I2C 0x3A): motors and encoders. Screen |

> **Safety:** the wheels reach full speed. Run it with the robot lying down or held in the air. Standing on its wheels, it would drive away.

## What you learn

- How to measure a speed with an encoder: count for a fixed time, then divide
- Why a motor needs more power to start than to keep turning (static friction)
- How to turn rpm into meters per second, and wheel speeds into a turn rate

## Build

```bash
scripts/build.sh examples/bala2-fire/05-speed-sweep
```

Output: `out/bala2-fire-05-speed-sweep-esp32.bin`

## Run it

- **Robot:** `esptool --chip esp32 write-flash 0x0 out/bala2-fire-05-speed-sweep-esp32.bin`. After a 5 second countdown, the wheels start. It takes about 2 minutes. Read the results on the serial port at 115200 baud.
- **Browser:** open https://robotics.diegoromero.es/emulator/, pick **ESP32 · BALA2 Fire** and drop the `.bin`. There is no base, so a motor model stands in for it.
- **QEMU:** with ESP-IDF installed, run `idf.py -C examples/bala2-fire/05-speed-sweep qemu monitor`.

It makes three passes, both wheels at the same time:

1. `forward-up`: 0 to 1023. Shows the lowest value that makes a still wheel start.
2. `forward-down`: 1023 to 0. Shows the lowest value that keeps a turning wheel moving.
3. `backward-up`: 0 to -1023. The same as pass 1, the other way.

Steps are 10 apart up to 200, then 50 apart, then 1023. At each step the wheels get 300 ms to settle, then we count for 700 ms. Each line is CSV, ready for a spreadsheet:

```
step,pass,speed,left_rpm,right_rpm
step,forward-up,400,186.7,179.4
```

## Results on our robot

Measured on 2026-10-30, robot lying down, wheels free (no load), battery charging over USB.

| Speed value | Left rpm | Right rpm | Left m/s |
| --- | --- | --- | --- |
| 50 to 80 | 0 | 0 | 0 |
| 100 | 35.3 | 32.0 | 0.08 |
| 200 | 84.9 | 81.4 | 0.20 |
| 400 | 186.7 | 179.4 | 0.44 |
| 600 | 290.8 | 279.4 | 0.69 |
| 800 | 395.5 | 380.6 | 0.93 |
| 1023 | 489.6 | 492.9 | 1.15 |

Forward, going up from 0. m/s uses the wheel diameter, 45 mm.

**Start and stop**

- A still wheel starts at **90 to 100**.
- A wheel that is already turning keeps going down to **50 to 60**. Below that, it stops.
- The gap between the two is static friction in the gearbox: starting takes more force than keeping it turning.

**Top speed**

About **490 to 498 rpm** at 1023, forward and backward. That is **1.15 to 1.17 m/s** with no load. M5Stack's spec says 530 rpm with no load. Under the robot's weight, and with a lower battery, it will be less.

**The curve is a straight line.** From 100 up, a good fit is:

```
rpm ≈ 0.50 × speed − 15        (error under 5 rpm from 200 to 1023)
m/s ≈ 0.00118 × speed − 0.036
```

**The wheels are not the same.** Below about 800, the left wheel turns 3 to 5% faster than the right one for the same value. If you send the same value to both, the robot turns slowly to the right. To drive straight, use the encoders and correct each wheel.

## From wheel speed to robot motion

Our measurements: wheel diameter D = 45 mm, tire width 17 mm, distance between the wheel centers b = 77 mm.

| What | Formula | Value |
| --- | --- | --- |
| Wheel turn | π × D | 141.4 mm |
| One encoder count | π × D / 420 | 0.34 mm |
| Wheel speed | rpm / 60 × π × D | m/s = rpm × 0.002356 |
| Robot speed | (v_left + v_right) / 2 | |
| Turn rate | (v_right − v_left) / b | rad/s, positive = turns left |
| Turn radius | (b / 2) × (v_left + v_right) / (v_right − v_left) | from the robot's center |

Some useful cases:

- **Spin in place** (one wheel forward, the other backward, same speed): each wheel moves on a circle of 38.5 mm radius. A full turn of the robot is 242 mm per wheel, about **1.7 wheel turns, or 718 encoder counts**. That is **2 counts per degree**.
- **Pivot on one wheel** (the other one stopped): the turning radius is b = 77 mm, so one robot turn is 1436 counts on the moving wheel.
- **Fastest spin with no load:** 2 × 1.16 / 0.077 = 30 rad/s, about 4.8 turns per second. In practice the robot must balance at the same time, so it will be much slower.

The tires are 17 mm wide, so the real contact point can be anywhere across that width. While turning, the real track can be between 60 and 94 mm. So turn rates from these formulas can be off by up to about 20%, most likely less. Measure a real turn to correct b.

## Try next

- Run it with the robot standing and held by hand over the table, wheels just touching, to see the effect of load.
- Store the start value (100) and the slope (0.5 rpm per unit) in a balancing program. Add the start value to small commands so the wheels react right away.
