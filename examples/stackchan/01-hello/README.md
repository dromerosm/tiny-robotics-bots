# StackChan 01: hello

Your first StackChan program. It boots, prints details about the chip, sends a heartbeat every 5 seconds and answers simple commands typed on the serial console.

| | |
| --- | --- |
| **Robot** | StackChan (CoreS3, ESP32-S3) |
| **Level** | Beginner |
| **Emulator** | ✅ Browser and native `esp-emu` |
| **Hardware used** | UART0 (USB serial) only |

## What you learn

- The structure of an ESP-IDF project (`CMakeLists.txt`, `main/`, `sdkconfig.defaults`)
- `app_main()`, logging with `ESP_LOGI` and printing with `printf`
- Reading the chip and flash details at run time
- Reading bytes from UART0 with the UART driver and building a simple command parser
- Running a second FreeRTOS task (the heartbeat)

## Build

From the repository root:

```bash
scripts/build.sh examples/stackchan/01-hello
```

Output: `out/stackchan-01-hello-esp32s3.bin`

## Run it

- **Browser:** open https://robotics.diegoromero.es/emulator/ and drop the `.bin`. Type `help` in the console input.
- **Native emulator:** `scripts/smoke.sh examples/stackchan/01-hello`
- **Robot:** `esptool --chip esp32s3 write-flash 0x0 out/stackchan-01-hello-esp32s3.bin`, then open a serial monitor at 115200 baud.

## Expected output

```
Hello from StackChan!
chip: esp32s3, 2 core(s), revision 0.0, 2 MB flash
free heap: 391504 bytes, uptime: 33 ms
Type a command and press Enter (try: help)
I (42) hello: heartbeat 1
> ping
pong
> info
chip: esp32s3, 2 core(s), revision 0.0, 2 MB flash
free heap: 389348 bytes, uptime: 40 ms
I (5042) hello: heartbeat 2
```

The emulator reports 2 MB of flash; a real CoreS3 has 16 MB.

## Commands

| Command | Reply |
| --- | --- |
| `ping` | `pong` |
| `info` | Chip, flash, free heap and uptime |
| `help` | List of commands |

## How it works

`app_main()` installs the UART0 driver, prints the banner and starts the `heartbeat` task. Then it loops on `uart_read_bytes()`, collects characters until Enter and passes the line to `handle()`. Commands are case-insensitive.

## Try next

- Add a `led` command once you know which GPIO drives an LED on your board.
- Continue with [02-console](../02-console), which replaces this hand-written parser with ESP-IDF's console component.
