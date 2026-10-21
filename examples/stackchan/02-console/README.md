# StackChan 02: console

An interactive command line on the serial port, built with ESP-IDF's `console` component. It gives you `help`, command registration and argument parsing for free.

| | |
| --- | --- |
| **Robot** | StackChan (CoreS3, ESP32-S3) |
| **Level** | Beginner |
| **Emulator** | ✅ Browser and native `esp-emu` |
| **Hardware used** | UART0 (USB serial) only |

## What you learn

- Creating a REPL (read-eval-print loop) with `esp_console_new_repl_uart()`
- Registering commands with `esp_console_cmd_register()`
- Reading heap usage and uptime
- Restarting the chip from software

## Build

```bash
scripts/build.sh examples/stackchan/02-console
```

Output: `out/stackchan-02-console-esp32s3.bin`

## Run it

- **Browser:** drop the `.bin` on https://robotics.diegoromero.es/emulator/ and type `help`.
- **Native emulator:** `scripts/smoke.sh examples/stackchan/02-console`
- **Robot:** flash it with `esptool --chip esp32s3 write-flash 0x0 out/stackchan-02-console-esp32s3.bin`. A terminal that supports escape sequences (for example `idf.py monitor` or PuTTY) also gives you history and tab completion.

## Expected output

```
StackChan console ready.
stackchan> heap
free heap: 385368 bytes (minimum since boot: 385368)
stackchan> echo hi StackChan
hi StackChan
stackchan>
```

In the emulator you also see a note that the terminal does not support escape sequences. That is expected: line editing is off, but commands work.

## Commands

| Command | What it does |
| --- | --- |
| `help` | Lists all commands |
| `info` | Chip and flash details |
| `heap` | Free heap and the minimum since boot |
| `uptime` | Time since boot |
| `echo <text>` | Prints the text back |
| `restart` | Restarts the chip |

## How it works

Each command is a function with the usual `int cmd(int argc, char **argv)` signature, listed in `register_commands()`. The REPL runs in its own task, reads lines from UART0, splits them into arguments and calls the matching function.

## Try next

- Add a command with typed arguments using `argtable3` (included in ESP-IDF), for example `look <pan> <tilt>` for a future servo example.
- Continue with [03-tasks](../03-tasks).
