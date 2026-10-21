# Running examples without the robot

StackChan examples run in Espressif's [ESP Emulator](https://github.com/espressif/esp-emulator) (esp-emu), which emulates the ESP32-S3: CPU, memory, flash, UART, GPIO, timers and more. BALA2 Fire uses the original ESP32, which esp-emu does not support. The browser emulator runs it in Espressif's QEMU instead.

## Browser

1. Build an example: `scripts/build.sh examples/stackchan/01-hello` (or `examples/bala2-fire/01-balance-sim`).
2. Open https://robotics.diegoromero.es/emulator/.
3. Pick the chip (**ESP32-S3** for StackChan, **ESP32** for BALA2 Fire) and drop the `.bin` from `out/`. It starts on its own.
4. Type commands in the input under the console and press Enter.

The firmware runs in your browser (WebAssembly) and is never uploaded.

## Native

```bash
scripts/install-esp-emu.sh
.tools/bin/esp-emu --chip esp32s3 --firmware out/stackchan-01-hello-esp32s3.bin
```

Useful options:

| Option | Use |
| --- | --- |
| `--timeout 20s` | Stop after a while |
| `--exit-on "text"` | Stop when the output contains `text` |
| `--inject "ping\n" --inject-on "help"` | Type into UART0 when some text appears |
| `--gdb 1234` | Debug with `xtensa-esp32s3-elf-gdb` |

`scripts/smoke.sh <example>` wraps these with the values in the example's `smoke.env`. That is what CI runs.

## Limits

- **Board peripherals:** both engines model the chip, not the board (CoreS3 or Fire). There is no screen, camera, touch, speaker, IMU, servos or motors. Firmware that waits for them will stop or fail at start-up. Examples in this repo avoid them or simulate them. For StackChan, the [`tr_emu`](../components/tr_emu) component gives a virtual screen and head servos that the browser emulator shows.
- **Original ESP32:** native esp-emu does not support it yet. BALA2 Fire examples run in the browser (QEMU) or in [Espressif QEMU](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/tools/qemu.html) on your computer. In the browser it needs a recent Chrome, Edge or Firefox, and does not show cores, GPIO or speed.
- **Network:** the browser version has no real network.
- **Speed:** the ESP32-S3 engine in the browser is 2 to 5 times slower than native. Timing-sensitive code may behave differently than on the robot.
- **Flash size:** the ESP32-S3 engine reports 2 MB; a real CoreS3 has 16 MB. The ESP32 (QEMU) has 16 MB of flash and 4 MB of PSRAM.
