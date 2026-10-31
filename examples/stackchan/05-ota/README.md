# StackChan 05: updates over Wi-Fi (OTA)

StackChan joins your home Wi-Fi and takes new firmware over the network, so you only need the USB cable once. You can set two networks. At boot it scans and joins the one with the strongest signal. A new app that does not work goes back to the old one by itself (rollback).

| | |
| --- | --- |
| **Robot** | StackChan (CoreS3, ESP32-S3) |
| **Level** | Intermediate |
| **Emulator** | ✅ Native `esp-emu`: it joins the emulator's own Wi-Fi network and starts the web server. You can't send an update to it from your computer |
| **Hardware used** | Wi-Fi only |

## What you learn

- Joining a Wi-Fi network as a station, and picking the strongest of several networks from a scan
- Over-the-air updates (OTA) with two app slots and `esp_ota_*`
- Rollback: an app on trial marks itself valid only when it can take the next update
- A small web server with `esp_http_server`, and a name on the network with mDNS (`stackchan.local`)
- Keeping Wi-Fi passwords out of git

## Build

The Wi-Fi networks come from a file outside this repo, for example:

```sh
SSID_01=first-network
PWD_SSID_01=its-password
SSID_02=second-network
PWD_SSID_02=its-password
```

Pass it with `WIFI_ENV`. `scripts/build.sh` gives its lines to the build as environment variables, and the `Kconfig` of [`tr_ota`](../../../components/tr_ota) reads them:

```bash
WIFI_ENV=../../../m5stack/wifi.env scripts/build.sh examples/stackchan/05-ota
```

Output: `out/stackchan-05-ota-esp32s3.bin` (merged image for the first flash) and `examples/stackchan/05-ota/build/ota.bin` (app image for updates over Wi-Fi).

The passwords end up in `sdkconfig`, `build/` and `out/`. Git ignores all three. Do not share those files.

Without `WIFI_ENV` (as in CI), the robot looks for esp-emu's built-in network, `myssid`.

## Run it

**First time, over USB.** This also writes the new partition table, with two 5 MB app slots, the same as the factory firmware. It erases the factory firmware, so back it up first (see the StackChan notes, section 12).

```bash
esptool --chip esp32s3 write-flash 0x0 out/stackchan-05-ota-esp32s3.bin
```

Open the serial port and note the **update token**. It is made at random on the first boot and saved in the robot. It is shown only on the USB port, and it is not in the code. A new flash over USB erases it and makes a new one.

**Updates over Wi-Fi.** Your computer must be on the same network:

```bash
curl http://stackchan.local/
curl -H "X-Token: <token>" --data-binary @examples/stackchan/05-ota/build/ota.bin http://stackchan.local/update
```

The robot answers `ok, restarting` and is back after about 4 seconds, running from the other slot.

**Native emulator:** `scripts/smoke.sh examples/stackchan/05-ota`

## Expected output

On the robot, after the first flash over USB (token and network hidden):

```
ota v0.2.0-8-g64097a7-dirty, built Oct 31 2026 11:54:50, running from ota_0 (valid)
update token: <token>
Wi-Fi: joining <network> (-35 dBm)
Wi-Fi: robot at 192.168.1.45, http://stackchan.local/
update with: curl -H 'X-Token: <token>' --data-binary @build/ota.bin http://stackchan.local/update
I (5951) ota: ready
```

After an update over Wi-Fi:

```
$ curl -H "X-Token: <token>" --data-binary @examples/stackchan/05-ota/build/ota.bin http://stackchan.local/update
ok, restarting
$ curl http://stackchan.local/
ota v0.2.0-8-g64097a7-dirty, built Oct 31 2026 11:56:55, running from ota_1 (valid)
```

## How it works

All the code is in the [`tr_ota`](../../../components/tr_ota) component. This example only calls `tr_ota_start()`. Every example that runs on the robot uses it the same way, so the robot can always take the next update.

### Wi-Fi

A task scans, keeps the known network with the strongest signal and joins it. When the link drops, it scans again, so the robot can move to the other network. With no known network in range, it scans every 5 seconds.

### Updates

`POST /update` checks the `X-Token` header first and writes nothing without it. Then it writes the body to the slot that does not run (`esp_ota_get_next_update_partition`). `esp_ota_end()` checks the image (chip, size, checksum) before the robot boots from it.

With a wrong token, the robot closes the connection while curl is still sending the file. Then curl may print nothing and exit with code 56. Nothing is written either way.

### Rollback

`CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE` is on. A new app boots once "on trial". It marks itself valid only when it has Wi-Fi and the web server runs. Then it can always take the next update. If it crashes or restarts before that, the bootloader boots the old app. If it has no Wi-Fi within 60 seconds, it goes back to the old app by itself.

An app flashed over USB is not on trial. It keeps trying to join Wi-Fi forever.

We tested it on the robot on 2026-10-31:

| Test | What we sent | Result |
| --- | --- | --- |
| Normal update | A new build of this example, 883 KB | Took 6.8 s. Back after about 4 s on the other slot, marked valid |
| App that crashes | This example with `abort()` at the top of `app_main()` | Back on the old app and slot |
| App with no Wi-Fi | This example built for a network that does not exist | Back on the old app and slot after 67 s (60 s on trial, plus boot and Wi-Fi) |

To repeat them, change a copy of the example. Do not commit the broken copies.

### Security

Anyone on your network can read `GET /`. Only someone with the token can update. The token goes over plain HTTP, so someone who reads your Wi-Fi traffic can see it. That is fine at home, not on a shared network. A next step would be signed images (`CONFIG_SECURE_SIGNED_APPS_NO_SECURE_BOOT`).

## Try next

- Send the logs over Wi-Fi as well. Then you need the cable only to recover the robot.
- Add a `restart` endpoint, and a page that shows the free heap and the signal strength.
