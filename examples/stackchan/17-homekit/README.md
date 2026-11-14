# StackChan 17: a HomeKit bridge for the plugs

The robot puts the Tuya plugs at home into Apple's Home app. Each outlet shows up as its own accessory, so you can switch it from an iPhone, ask Siri, or use it in an automation. With an Apple TV as home hub it also works from outside the house.

This is [13-home](../13-home) with everything but the plugs taken out: no talk, no audio, no face. The robot still talks to the plugs over the local network, with no cloud, through [`tr_tuya`](../../../components/tr_tuya). HomeKit's protocol (HAP) comes from Espressif's [esp-homekit-sdk](https://github.com/espressif/esp-homekit-sdk).

| | |
| --- | --- |
| **Robot** | StackChan (CoreS3, ESP32-S3) |
| **Level** | Advanced |
| **Emulator** | ✅ Native `esp-emu`, without screen. It checks that Wi-Fi and HomeKit start. Pairing needs the real robot |
| **Hardware used** | Screen, touch, head touch, head servos (only to rest them) |
| **Also needs** | `TUYA_DEVICES` in the env file, and an iPhone or iPad with the Home app |

## What you learn

- A HomeKit bridge on an ESP32-S3: one accessory for the robot and one bridged accessory per outlet.
- How to use an SDK written for ESP-IDF 5 on ESP-IDF 6 without changing its files.
- How to find Tuya devices by their UDP broadcasts, so a new IP address does not break anything.

## The devices

They come from one line in the env file passed to the build (`WIFI_ENV`), never from git. It is the same line as in 13-home:

```
TUYA_DEVICES=Salon,34,192.168.1.33,<id>,<key-hex>,1:Main:main,7:USB:usb;Personal,33,...,1:AP:Access_Point:ap,2:TV:TV
```

One device per `;`: name, protocol version (33 or 34), IP, device ID, local key in hex, then one `dp:label:name` per outlet. In Home, an outlet is called `<device> <name>`, for example "Personal TV". You can rename it there.

`~/GitHub/home-iot` makes the line from its `devices.json` and the keys in `~/.secrets`:

```bash
~/GitHub/home-iot/.venv/bin/python ~/GitHub/home-iot/scripts/stackchan_env.py
```

### The IP is only a first guess

Each device says where it is in a UDP broadcast on port 6667, every few seconds. The robot listens. When a device of ours shows a new address, the robot uses it from then on. So you do not need fixed addresses in the router.

### The access point outlet

`:ap` marks the outlet that powers the Wi-Fi access point. While the robot is on `CONFIG_HOME_AP_SSID` (`home-ap` by default), off for that outlet is refused: the robot would lose its network, and nobody could switch the outlet on again from Home. On any other network, that outlet behaves like the rest.

## Build

The firmware needs one env file with the Wi-Fi networks and the plugs. Build it outside git, with mode 600:

```bash
E=$(mktemp) && chmod 600 $E
cat ../../../m5stack/wifi.env > $E && echo >> $E
~/GitHub/home-iot/.venv/bin/python ~/GitHub/home-iot/scripts/stackchan_env.py >> $E
WIFI_ENV=$E scripts/build.sh examples/stackchan/17-homekit
rm $E
```

Output: `out/stackchan-17-homekit-esp32s3.bin` and `examples/stackchan/17-homekit/build/homekit.bin`.

The first build downloads the HomeKit SDK from GitHub. `main/idf_component.yml` pins it to one commit.

## Run it

On the robot, send `build/homekit.bin` over Wi-Fi as in [05-ota](../05-ota) (`POST /update`). At boot the head goes to its rest position and the servos switch off.

To add it to Home:

1. On the robot's screen, tap the **QR** tab (the last one). It shows a QR code, and under it the setup code as text. The code is made at random on the first boot. It is also printed on the USB port.
2. On the iPhone, on the same Wi-Fi: Home, **+**, **Add Accessory**, and scan the QR code. If it does not scan, tap **More options**, pick **StackChan Bridge** and type the code.
3. Home warns that the accessory is not certified. That is expected: the open SDK has no Apple certificate. Tap **Add Anyway**.
4. Home then asks for a room and a name for each outlet.

The screen goes off after 30 seconds. Touch the screen or the head to switch it on again. That touch only wakes the screen.

### The screen

One tab per device, and the QR tab. A device's tab has one button per outlet: green is on, grey is off. Tap a button to switch the outlet. It stays dark until the plug answers, and Home shows the change too. The line at the bottom has the device's IP and its power, or the reason it does not answer.

| Endpoint | What |
| --- | --- |
| `GET /home` | How many controllers are paired, and every device: its IP now, each outlet on or off, and the power |
| `POST /home?unpair=1` | Forget every pairing and restart (token needed). Then remove the bridge in Home and add it again |

In the emulator, build with no env file, so the robot joins esp-emu's own network:

```bash
scripts/build.sh examples/stackchan/17-homekit
scripts/smoke.sh examples/stackchan/17-homekit
```

## Expected output

On the robot on 2026-11-14, from `GET /log`:

```
I (2130) plugs: 3 devices
I (2210) bridge: HomeKit bridge with 8 outlets, 0 paired
I (5600) tr_ota: Wi-Fi: robot at 192.168.1.54, http://stackchan.local/
I (5840) plugs: Salon at 192.168.1.33 answers
I (6300) plugs: Trabajo at 192.168.1.36 answers
```

And from a build with a wrong IP for one device, on purpose:

```
Trabajo (192.168.1.199): no connection (off, busy, or wrong IP)
I (9631) plugs: Trabajo moved from 192.168.1.199 to 192.168.1.36
I (10281) plugs: Trabajo at 192.168.1.36 answers
```

From a Mac on the same network, `dns-sd -B _hap._tcp` lists `StackChan Bridge`.

## How it works

```
Home app -- HAP, port 8080 --> bridge.c -- order --> queue --> worker task --> tr_tuya --> plug
plug's button, phone app ----> worker reads every 15 s ------> bridge.c ----> Home app
plug's UDP broadcast --------> find task -- new IP ---> queue --> worker task
```

- **HomeKit's task never waits on the network.** A write from Home puts an order on a queue and answers at once. The worker task switches the outlet and reads the device again. Then the bridge tells Home what is true, so a failed order shows up as the old state.
- **Home learns of changes made elsewhere.** The worker reads every device each 15 s (3 s while one does not answer). When an outlet is not as Home thinks, the bridge sends an event.
- **A plug that does not answer** gives a read error. Home shows "No Response" for its outlets.
- **One accessory per outlet**, not one per power strip. Each outlet gets its own name, tile and room.
- **The same accessory after a restart.** Each outlet has a number that the SDK keeps in NVS, under a key made from the device ID and the data point.
- **Only the worker changes the device table.** The find task sends a new IP to the worker through the queue.
- **Two web servers.** The robot's page stays on port 80 (`tr_ota`). HomeKit has its own on port 8080.
- **`stackchan.local` still works.** The SDK sets the mDNS name to "MyHost". The bridge sets it back.
- **No sleep.** A bridge must answer at any time, also on the battery.

### The SDK on ESP-IDF 6

The SDK is written for ESP-IDF 5. Three settings in `CMakeLists.txt` and one in `sdkconfig.defaults` make it build on ESP-IDF 6.1, with no change to its files:

| Problem | Fix |
| --- | --- |
| The old Wi-Fi interface names (`ESP_IF_WIFI_STA`) are gone | Two compile definitions for `esp_hap_core` |
| A prototype with `()` is not valid C23 | `esp_hap_core` builds as C17 |
| mbedTLS 4 moved `aes.h` and the SHA headers to `mbedtls/private/` | `idf6/mbedtls/*.h` include them from there, after the build config |
| With hardware SHA, mbedTLS's own SHA-512 is not built | `CONFIG_MBEDTLS_HARDWARE_SHA=n` |

## Checked so far

- The firmware on the robot: it starts, joins Wi-Fi, reads the three real plugs, and announces `_hap._tcp` (seen from a Mac). The HomeKit port answers 470 to a client that is not paired, as it should.
- A device with a wrong IP in the build: found by its broadcast in a few seconds.
- `tr_tuya_broadcast()` against the real broadcasts, from a Mac, and in the host tests.
- In esp-emu: Wi-Fi and HomeKit start (the smoke test).
- Pairing: the Home app added the bridge on 2026-11-14 (`GET /home` says `1 paired`), and the pairing stayed after two updates over Wi-Fi.
- The screen on the robot, by its owner on 2026-11-14: the tabs show well and the buttons switch the outlets.

Not checked yet:

- Switching an outlet from the Home app.
- The Apple TV as home hub: Siri, automations, and control from outside.
- The 30 s screen timeout and the head's rest move: nobody reported on them.
- Discovery from the other home network. The tests ran with the robot on `home-ap`.
- A whole day running: memory, and the power strip with the phone app open.

## Try next

- Pair it and check the list above.
- The power of each device. HomeKit has no standard field for watts, so Home would not show it.
- The same firmware on a plain ESP32-S3 board with no screen, as a bridge that is always on.
