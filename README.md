# Tiny Robotics bots

Code for the two robots of the [Tiny Robotics](https://robotics.diegoromero.es) project: StackChan and BALA2 Fire.

Each project builds into one firmware image that you can try without the robot in the [browser emulator](https://robotics.diegoromero.es/emulator/).

## Projects

| Project | Robot | Chip | What it does |
| --- | --- | --- | --- |
| [`stackchan/hello`](stackchan/hello) | StackChan | ESP32-S3 | Prints a heartbeat and answers `ping`, `info` and `help` on the serial console |

## Try a project in the emulator

1. Build it (you only need Docker):
   ```bash
   scripts/build.sh stackchan/hello
   ```
2. Open https://robotics.diegoromero.es/emulator/, keep **ESP32-S3** selected and drop `out/stackchan-hello-esp32s3.bin`.
3. Type `help` in the console input and press Enter.

You can also download the images from the latest run of the **Build firmware** workflow (artifact `firmware`).

## Layout

```
stackchan/<project>/    ESP-IDF projects for StackChan (ESP32-S3)
bala2-fire/<project>/   ESP-IDF projects for BALA2 Fire (ESP32)
scripts/build.sh        Build one project into out/<robot>-<project>-<chip>.bin
idf.env                 ESP-IDF Docker image (keep in sync with the workflow)
```

A project is any folder with a `CMakeLists.txt` and a `sdkconfig.defaults` that sets `CONFIG_IDF_TARGET`. CI builds all of them.

## Disclaimer

Personal learning project. Not affiliated with M5Stack or Espressif. M5Stack, StackChan and BALA2 belong to M5Stack Technology Co., Ltd.; Espressif, ESP32 and ESP-IDF are trademarks of Espressif Systems.
