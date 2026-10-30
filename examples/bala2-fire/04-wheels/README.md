# BALA2 Fire 04: wheels

Turn each wheel on with the Fire's buttons, and watch the encoders count. Button A starts the left wheel, C starts the right wheel, and B in the middle stops both. This is the first example that moves the robot's motors.

| | |
| --- | --- |
| **Robot** | BALA2 Fire (M5Stack Fire, ESP32) |
| **Level** | Beginner |
| **Emulator** | Yes, as a short demo with simulated encoders, in the [browser](https://robotics.diegoromero.es/emulator/) (pick **ESP32**) or with Espressif QEMU |
| **Hardware used** | BALA2 base (I2C 0x3A): motors and encoders. Buttons A, B, C. Screen |

> **Safety:** the robot does not balance in this example. Run it with the robot lying down or held in the air. Standing on its wheels, it would drive away.

## What you learn

- How the Fire talks to the BALA2 base: one I2C write for both motor speeds, one I2C read for both encoders
- How to read the Fire's buttons and catch each press once
- How to check the direction of a motor and an encoder on the real robot

## Build

```bash
scripts/build.sh examples/bala2-fire/04-wheels
```

Output: `out/bala2-fire-04-wheels-esp32.bin`

## Run it

- **Robot:** `esptool --chip esp32 write-flash 0x0 out/bala2-fire-04-wheels-esp32.bin`. Press the buttons. The screen shows each wheel's state, speed, encoder count and turns. The same data goes to the serial port at 115200 baud.
- **Browser:** open https://robotics.diegoromero.es/emulator/, pick **ESP32 · BALA2 Fire** and drop the `.bin`. There is no base and there are no buttons, so it runs a demo: left wheel on, right wheel on, stop.
- **QEMU:** with ESP-IDF installed, run `idf.py -C examples/bala2-fire/04-wheels qemu monitor`.

## Expected output

On our robot, on 2026-10-30, lying down:

```
I (522) wheels: BALA2 base found at 0x3a
A: left wheel on
 left  400 enc   -1589 | right    0 enc       0
 left  400 enc   -2907 | right    0 enc       0
B: stop
C: right wheel on
 left    0 enc   -5398 | right  400 enc   -1112
 left    0 enc   -5398 | right  400 enc   -2376
B: stop
```

In QEMU, the lines start with `(sim)` and end with `demo done (sim)`.

## How it works

The base has an STM32 that drives the two motors. The register map is not in M5Stack's docs. We took it from their code (Arduino `bala.cpp` and UiFlow2 `bala2.py`):

| Register | Bytes | Content |
| --- | --- | --- |
| `0x00` | write 4 | Speed of motor 1 (left), then motor 2 (right). `int16`, big-endian, -1023 to 1023 |
| `0x10` | read 8 | Encoder of motor 1 (left), then motor 2 (right). `int32`, big-endian |

The speed is a PWM duty, not a real speed. The example uses 400.

The buttons are on GPIO 39 (A), 38 (B) and 37 (C). They read 0 while pressed. The Fire has pull-up resistors on them, so we only set the pins as inputs. Every 20 ms we read them. A change from 1 to 0 counts as one press.

### What we measured

Checked on our robot by pressing the buttons and looking at the wheels:

- Motor 1 is the **left** wheel, motor 2 the **right** wheel. Each button moves only its own wheel.
- A **positive** speed turns the left wheel **forward** (toward the screen side). Then its encoder counts **down**. The right encoder counts down too, so we expect the same direction there. We did not look at that wheel to confirm it.
- At speed 400, a wheel turns about 1300 counts per second. With 420 counts per turn, that is about 3 turns per second (185 rpm), with no load.
- After B, the wheels coast for less than a second.

M5Stack's two libraries do not agree on the signs. Arduino sends the speed as it is, which matches what we saw. UiFlow2 flips the speed, and its docs say the encoder counts up when the wheel goes forward. On our robot it counts down.

## Try next

- Press A twice to change the left wheel's direction.
- Use the encoders to stop each wheel after exactly one turn (420 counts).
