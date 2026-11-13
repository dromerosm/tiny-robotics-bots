# Tiny Robotics examples

[![CI](https://github.com/dromerosm/tiny-robotics-bots/actions/workflows/ci.yml/badge.svg)](https://github.com/dromerosm/tiny-robotics-bots/actions/workflows/ci.yml)
[![License: MIT](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
[![ESP-IDF v6.1](https://img.shields.io/badge/ESP--IDF-v6.1-red.svg)](https://docs.espressif.com/projects/esp-idf/en/v6.1/)
[![Website](https://img.shields.io/badge/website-robotics.diegoromero.es-orange.svg)](https://robotics.diegoromero.es)

**Website:** [robotics.diegoromero.es](https://robotics.diegoromero.es) · **Try the examples in your browser:** [ESP32 Emulator](https://robotics.diegoromero.es/emulator/)

Small, focused ESP-IDF examples for two M5Stack robots: **StackChan** and **BALA2 Fire**. Each example teaches one idea, builds with one command and runs in your browser without the robot.

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
| [BALA2 Fire](https://docs.m5stack.com/en/app/Bala2-Fire) | M5Stack Fire | ESP32 | Yes, in the [browser](https://robotics.diegoromero.es/emulator/) (QEMU) and with [Espressif QEMU](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/tools/qemu.html) |

## Examples

Examples are numbered per robot, from simplest to most advanced. Start with `01`.

### StackChan

| # | Example | What you learn | Emulator |
| --- | --- | --- | --- |
| 01 | [hello](examples/stackchan/01-hello) | Boot, print to the serial console, read commands from UART | ✅ |
| 02 | [console](examples/stackchan/02-console) | An interactive command line with ESP-IDF's `console` component | ✅ |
| 03 | [tasks](examples/stackchan/03-tasks) | FreeRTOS tasks, queues and event groups: read, filter, react | ✅ |
| 04 | [face](examples/stackchan/04-face) | Draw a face on the screen and move the head servos from the console | ✅ |
| 05 | [ota](examples/stackchan/05-ota) | Join the home Wi-Fi and update over the air, with rollback | ✅ (Wi-Fi in esp-emu, no updates) |
| 06 | [status](examples/stackchan/06-status) | Show the robot's status on the real screen, one section per page, and sleep when nobody uses it | ✅ (no screen in esp-emu) |
| 07 | [head](examples/stackchan/07-head) | Read and move the head servos over Wi-Fi, from a page the robot serves | ✅ (no servos in esp-emu) |
| 08 | [face](examples/stackchan/08-face) | The face on the real screen, a head that looks around, an audio analyzer and the camera, all from the page too | ✅ (no screen or servos in esp-emu) |
| 09 | [wifi](examples/stackchan/09-wifi) | 08 without microphones and camera, the screen at 20 %, and the Wi-Fi networks around in the right eye | ✅ (no screen or servos in esp-emu) |
| 10 | [track](examples/stackchan/10-track) | 09 plus face tracking: esp-dl finds faces on the second core about 12 times per second, the head follows the biggest face and searches when there is none, and the eyes look at the person | ✅ (no screen, camera or servos in esp-emu) |
| 11 | [track](examples/stackchan/11-track) | 10 on less battery: fewer finds and no servo power while it searches, the camera off while it sleeps (32 mA asleep, 136 mA searching) | ✅ (no screen, camera or servos in esp-emu) |
| 12 | [talk](examples/stackchan/12-talk) | Talk with the robot through OpenAI's GPT-Live, full duplex: speaker and both microphones at once, and esp-sr's echo canceller on both microphones (40 to 58 dB of echo removed). It hands searches to `gpt-6-luna`. The API key stays in a Cloudflare Worker | ✅ (no screen or audio in esp-emu) |
| 13 | [home](examples/stackchan/13-home) | 12's talk plus the plugs on the home network (Tuya 3.3 and 3.4, no cloud): a Home screen on the left eye, switch them by voice, and `/home` on the web | ✅ (no screen or audio; it reads the real plugs through the emulator's network) |
| 14 | [record](examples/stackchan/14-record) | 12's talk plus a recording mode: tap the right eye and the robot records the room (36 dB gain, 90 Hz high-pass) and uploads it to R2 in pieces of 10 s, through a Cloudflare Worker. The face turns orange if the uploads fall behind. The Mac app [Scribe](tools/scribe) turns it into text with speakers | ✅ (no screen or audio in esp-emu) |
| 15 | [follow](examples/stackchan/15-follow) | 14 plus 11's face tracking, by turns: the head follows your face while nobody talks, and for a talk or a recording the camera, the face model and the servos stop. The echo canceller works on one microphone, so it all fits in internal RAM. The speaker is louder, with less microphone gain | ✅ (no screen, camera, servos or audio in esp-emu) |

### BALA2 Fire

| # | Example | What you learn | Emulator |
| --- | --- | --- | --- |
| 01 | [balance-sim](examples/bala2-fire/01-balance-sim) | A PID loop that balances a simulated two-wheeled robot | ✅ |
| 02 | [battery](examples/bala2-fire/02-battery) | Read the battery level and charging state from the IP5306 over I2C | ✅ (simulated values) |
| 03 | [imu-calibration](examples/bala2-fire/03-imu-calibration) | Calibrate the gyro at each boot and show the tilt on the screen | ✅ (simulated readings) |
| 04 | [wheels](examples/bala2-fire/04-wheels) | Turn each wheel on with the buttons and read the encoders | ✅ (demo) |
| 05 | [speed-sweep](examples/bala2-fire/05-speed-sweep) | Measure real wheel speed (rpm, m/s) for each speed value | ✅ (motor model) |
| 06 | [balance](examples/bala2-fire/06-balance) | Stay upright with a PID and a position loop; tune and update over Wi-Fi | ✅ (simulated robot, no Wi-Fi) |

### Shared components

| Component | What it is |
| --- | --- |
| [tr_pid](components/tr_pid) | Portable PID controller with anti-windup and derivative on measurement. Unit-tested on the host. |
| [tr_emu](components/tr_emu) | Virtual screen and head servos for StackChan. The browser emulator shows them next to the console. Emulator only for now. |
| [tr_ota](components/tr_ota) | Home Wi-Fi, updates over Wi-Fi and rollback for StackChan. Every example that runs on the robot calls `tr_ota_start()`. |
| [tr_board](components/tr_board) | StackChan board parts: screen, touch, head servos, side LEDs, microphones, camera, battery and the other sensors. Also sleep: after a minute with no touch, on battery, it turns off what nobody uses, and a touch wakes it. |
| [tr_web](components/tr_web) | The robot's web page (`http://stackchan.local/app`), the head and sleep endpoints, and the status screen on the robot. Every example that runs on the robot uses it. |

### Workers and Mac tools

| Folder | What it is |
| --- | --- |
| [workers/talk-key](workers/talk-key) | The robot's bridge to GPT-Live, for 12-talk to 15-follow. The OpenAI key stays in the Worker. It can also ask Diego's Second Brain, read-only. |
| [workers/rec](workers/rec) | The robot's recordings (14-record) in R2, in the EU, deleted after 30 days. |
| [tools/transcribe](tools/transcribe) | Speech to text and who spoke when on a Mac, in Python (mlx-whisper, pyannote), with benchmarks on public Spanish data. |
| [tools/scribe](tools/scribe) | A native Mac app for the same job, in Swift (WhisperKit, FluidAudio). It lists the robot's recordings in R2. |

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
2. Pick the chip (**ESP32-S3** for StackChan, **ESP32** for BALA2 Fire) and drop the `.bin` from `out/`.
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
