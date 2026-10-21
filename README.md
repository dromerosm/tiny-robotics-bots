# Tiny Robotics examples

[![CI](https://github.com/dromerosm/tiny-robotics-bots/actions/workflows/ci.yml/badge.svg)](https://github.com/dromerosm/tiny-robotics-bots/actions/workflows/ci.yml)
[![License: MIT](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
[![ESP-IDF v6.1](https://img.shields.io/badge/ESP--IDF-v6.1-red.svg)](https://docs.espressif.com/projects/esp-idf/en/v6.1/)
[![Website](https://img.shields.io/badge/website-robotics.diegoromero.es-orange.svg)](https://robotics.diegoromero.es)

**Website:** [robotics.diegoromero.es](https://robotics.diegoromero.es) · **Try the examples in your browser:** [ESP32-S3 Emulator](https://robotics.diegoromero.es/emulator/)

Small, focused ESP-IDF examples for two M5Stack robots: **StackChan** and **BALA2 Fire**. Each example teaches one idea, builds with one command and, for StackChan, runs in your browser without the robot.

They are part of [Tiny Robotics](https://robotics.diegoromero.es), a personal project to learn these robots inside out and push them to their limits.

## Contents

- [Robots](#robots)
- [Examples](#examples)
- [Quick start](#quick-start)
- [Run without the robot](#run-without-the-robot)
- [Flash a real robot](#flash-a-real-robot)
- [Repository layout](#repository-layout)
- [Contributing](#contributing)
- [License and trademarks](#license-and-trademarks)

## Robots

| Robot | Controller | Chip | Emulator |
| --- | --- | --- | --- |
| [StackChan](https://docs.m5stack.com/en/StackChan) | CoreS3 | ESP32-S3 | Yes, in the [browser](https://robotics.diegoromero.es/emulator/) and with native `esp-emu` |
| [BALA2 Fire](https://docs.m5stack.com/en/app/Bala2-Fire) | M5Stack Fire | ESP32 | Not yet in esp-emu. Use the robot or [Espressif QEMU](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/tools/qemu.html) |

## Examples

Examples are numbered per robot, from simplest to most advanced. Start with `01`.

### StackChan

| # | Example | What you learn | Emulator |
| --- | --- | --- | --- |
| 01 | [hello](examples/stackchan/01-hello) | Boot, print to the serial console, read commands from UART | ✅ |
| 02 | [console](examples/stackchan/02-console) | An interactive command line with ESP-IDF's `console` component | ✅ |
| 03 | [tasks](examples/stackchan/03-tasks) | FreeRTOS tasks, queues and event groups: read, filter, react | ✅ |

### BALA2 Fire

| # | Example | What you learn | Emulator |
| --- | --- | --- | --- |
| 01 | [balance-sim](examples/bala2-fire/01-balance-sim) | A PID loop that balances a simulated two-wheeled robot | QEMU |

### Shared components

| Component | What it is |
| --- | --- |
| [tr_pid](components/tr_pid) | Portable PID controller with anti-windup and derivative on measurement. Unit-tested on the host. |

## Quick start

You need [Docker](https://docs.docker.com/get-docker/) and Git. ESP-IDF runs inside Docker, so you don't need to install it.

```bash
git clone https://github.com/dromerosm/tiny-robotics-bots.git
cd tiny-robotics-bots
scripts/build.sh examples/stackchan/01-hello
```

The build writes one merged firmware image:

```
out/stackchan-01-hello-esp32s3.bin
```

Already have ESP-IDF v6.1 installed? Run `. $IDF_PATH/export.sh` first and `scripts/build.sh` uses it instead of Docker.

To build everything: `scripts/build-all.sh`.

## Run without the robot

### In the browser

1. Open **https://robotics.diegoromero.es/emulator/**.
2. Keep **ESP32-S3** selected and drop the `.bin` from `out/`.
3. It starts on its own. Type commands in the input under the console.

The firmware runs in your browser and is never uploaded. More details, including the limits, are in [docs/emulator.md](docs/emulator.md).

### On your computer

```bash
scripts/install-esp-emu.sh                  # installs the pinned esp-emu in .tools/
scripts/smoke.sh examples/stackchan/01-hello  # runs it and checks the output
.tools/bin/esp-emu --chip esp32s3 --firmware out/stackchan-01-hello-esp32s3.bin
```

## Flash a real robot

Connect the robot over USB-C and flash the merged image at offset 0:

```bash
pip install esptool
esptool --chip esp32s3 write-flash 0x0 out/stackchan-01-hello-esp32s3.bin
```

Use `--chip esp32` for BALA2 Fire. Then open a serial monitor at 115200 baud, for example `python -m serial.tools.miniterm /dev/ttyACM0 115200`.

> **Servo safety (StackChan):** keep the tilt servo between 5° and 85°, as M5Stack recommends. Don't turn the head by hand while the servos are powered.

## Repository layout

```
examples/
  stackchan/NN-name/     ESP-IDF project for StackChan (ESP32-S3)
  bala2-fire/NN-name/    ESP-IDF project for BALA2 Fire (ESP32)
components/              Shared, portable components used by the examples
tests/host/              Unit tests that run on your computer (no ESP-IDF)
scripts/                 build, build-all, smoke, test-host, format, install-esp-emu
docs/                    Emulator guide and example guidelines
tools.env                Pinned ESP-IDF image and esp-emu version
```

Every example folder has:

| File | Purpose |
| --- | --- |
| `README.md` | What it does, what you learn, how to run it, expected output |
| `CMakeLists.txt`, `main/` | The ESP-IDF project |
| `sdkconfig.defaults` | Target chip and the settings the example needs |
| `smoke.env` | What CI expects to see when it runs the example in the emulator |

## Continuous integration

On every push and pull request, CI:

1. checks C formatting (`clang-format` 18.1.8),
2. runs the host unit tests,
3. builds every example with the pinned ESP-IDF image,
4. runs each StackChan example in `esp-emu` and checks its output.

Tagged releases (`v*`) publish every firmware image, with SHA-256 checksums, on the [Releases](https://github.com/dromerosm/tiny-robotics-bots/releases) page.

## Contributing

Ideas, fixes and new examples are welcome. Please read [CONTRIBUTING.md](CONTRIBUTING.md) and the [example guidelines](docs/writing-examples.md) first. This project follows a [code of conduct](CODE_OF_CONDUCT.md). To report a security problem, see [SECURITY.md](SECURITY.md).

## License and trademarks

The code in this repository is released under the [MIT License](LICENSE).

This is a personal learning project. It is not affiliated with, sponsored by or endorsed by M5Stack or Espressif. M5Stack, StackChan, BALA2 and related product names belong to M5Stack Technology Co., Ltd. Espressif, ESP32 and ESP-IDF are trademarks of Espressif Systems. They are used here only to identify the hardware and tools.
