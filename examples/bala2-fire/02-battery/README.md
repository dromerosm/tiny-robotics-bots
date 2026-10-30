# BALA2 Fire 02: battery

Read the battery level and see if BALA2 Fire is charging. The M5Stack Fire has an IP5306 power chip. It charges the battery in the BALA2 base from the Fire's USB-C port, and it tells us over I2C how full the battery is.

| | |
| --- | --- |
| **Robot** | BALA2 Fire (M5Stack Fire, ESP32) |
| **Level** | Beginner |
| **Emulator** | Yes, with simulated values, in the [browser](https://robotics.diegoromero.es/emulator/) (pick **ESP32**) or with Espressif QEMU |
| **Hardware used** | IP5306 power chip (I2C 0x75) |

## What you learn

- How to use the ESP-IDF I2C master driver (`driver/i2c_master.h`): create a bus, probe an address, read a register
- How to read single bits from a register
- How to fall back to simulated data when the hardware is not there

## Build

```bash
scripts/build.sh examples/bala2-fire/02-battery
```

Output: `out/bala2-fire-02-battery-esp32.bin`

## Run it

- **Robot:** `esptool --chip esp32 write-flash 0x0 out/bala2-fire-02-battery-esp32.bin`, then open a serial monitor at 115200 baud. The serial monitor needs the USB cable. That cable also charges the battery, so here you only see "charging" or "full". To see "on battery", show the state on the screen (see Try next).
- **Browser:** open https://robotics.diegoromero.es/emulator/, pick **ESP32 · BALA2 Fire** and drop the `.bin`. There is no IP5306 in the emulator, so you see simulated values.
- **QEMU:** with ESP-IDF installed, run `idf.py -C examples/bala2-fire/02-battery qemu monitor`.

## Expected output

In QEMU:

```
W (1835) battery: no IP5306 at 0x75 (emulator?), using simulated values
battery (sim):  25%  charging
battery (sim):  50%  charging
battery (sim):  75%  charging
battery (sim): 100%  full
```

On the robot, with USB connected:

```
I (291) battery: IP5306 found at 0x75
battery:  50%  charging
battery:  50%  charging
```

## How it works

The IP5306 on the Fire is a special version with I2C. The register map comes from M5Stack's Arduino library (`src/utility/Power.cpp`):

| Register | Bits | Meaning |
| --- | --- | --- |
| `0x70` | bit 3 | 1 = charging |
| `0x71` | bit 3 | 1 = charge full |
| `0x78` | bits 7-4 | Level: `0x0` = 100%, `0x8` = 75%, `0xC` = 50%, `0xE` = 25%, other = 0% |

At boot, `power_init()` probes address 0x75. If nothing answers, as in the emulator, `simulate_power()` makes up values and the output says `(sim)`. `read_power()` is the only function that touches the chip.

Limits:

- The level comes in 25% steps. The chip does not give the voltage.
- The steps depend on the voltage, and the voltage is higher while charging. So the level can jump when you plug or unplug the cable.

## Try next

- Show the level on the Fire's screen.
- In a balancing program, stop the motors when the level falls to 0%.
