# bala2_link

Wi-Fi, updates over Wi-Fi (OTA) and a small web server for BALA2 Fire firmware. It is one component, because the three parts need each other: the update comes in through the web server, the web server needs Wi-Fi, and all three share one key.

It is for the BALA2 Fire only (ESP32). StackChan has its own module, [tr_ota](../tr_ota). The two robots share one file, the page icon.

[06-balance](../../examples/bala2-fire/06-balance) uses it.

## What the robot does with it

- **Wi-Fi.** At boot it scans for the home networks set at build time and joins the strongest one. If none is in range, or it can't join in 10 s, it makes its own network: `BALA2-xxxx`, at 192.168.4.1. Either way it is `bala2.local`.
- **Key.** 12 random characters, made on the first boot and kept in the robot. It is the password of the robot's own network and the token for updates. It is never in the code. Show it on the screen with `link_password()`.
- **UDP link, port 4210.** Send a line to the robot (a command, or `hb` as a heartbeat) and it sends its telemetry back to you.
- **Updates.** `POST /update` with the app image and the header `X-Token: <key>`.
- **Web server.** `GET /` gives the firmware's page, `GET /state` its state as JSON, `POST /cmd` takes a command line, and `GET /info` says which firmware runs.

## Use it

1. In the project's `CMakeLists.txt`, before `project()`:

   ```cmake
   set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../../components")
   set(COMPONENTS main)
   ```

   The second line builds only what the firmware uses. Without it, StackChan's modules are built too.

2. Require it in `main/CMakeLists.txt` with `PRIV_REQUIRES bala2_link`.

3. Add these lines to `sdkconfig.defaults`:

   ```
   CONFIG_ESPTOOLPY_FLASHSIZE_16MB=y
   CONFIG_PARTITION_TABLE_CUSTOM=y
   CONFIG_PARTITION_TABLE_CUSTOM_FILENAME="../../../components/bala2_link/partitions.csv"
   CONFIG_LWIP_IPV6=n
   CONFIG_MDNS_ENABLE_CONSOLE_CLI=n
   ```

4. Call it at boot:

   ```c
   #include "bala2_link.h"

   void app_main(void)
   {
       link_guard(); // first: see "A new app is on trial"
       // ... start the robot ...
       const link_callbacks_t cb = {.on_command = handle_command};
       if (link_start(&cb) != ESP_OK) {
           // no link: the robot still works
       }
   }
   ```

Every field of `link_callbacks_t` can be left out. With no page, `GET /` gives the info line. With no `state_json` or `on_command`, `/state` and `/cmd` are not there.

`link_start()` takes up to about 15 s (the scan and the join). It never stops the firmware: on an error it returns it, and the robot goes on without a link.

## Home networks

The networks come from a file outside the repo, the same one StackChan uses:

```sh
WIFI_ENV=~/wifi.env scripts/build.sh examples/bala2-fire/06-balance
```

The file has `SSID_01`, `PWD_SSID_01`, `SSID_02` and `PWD_SSID_02`. With none set, as in CI, the robot makes its own network. A network with a password must be WPA2 or better: the robot does not join an open network with the same name.

## Partitions

`partitions.csv` has two app slots of 3 MB each. It is the table already on our robot. A new table can only go in over USB, because an update over Wi-Fi writes only the app. So do not change the offsets unless you plan to flash over USB. The last 6 MB of the flash are free.

## Safety for a robot that moves

- **Heartbeat.** Once a computer has talked to the robot over UDP, `link_alive()` turns false if it is quiet for 1 second. A control loop calls it and stops the motors. The web page does not start this rule.
- **No update while it moves.** `can_update()` returns false, and the robot answers 409. `link_updating()` is true while an update is written, so the firmware can refuse to start: flash writes stall both cores for a few milliseconds at a time.
- **Core 0 only.** Wi-Fi, the web server and the UDP task run on core 0. A control loop on core 1 only calls `link_alive()` and `link_updating()`, which read one word each.
- **A bad request does no harm.** A wrong key gives 401, an image that is too large or empty gives 400, and the connection closes at once.

## A new app is on trial

The Fire's bootloader has no rollback, and a new bootloader can only go in over USB. So the app does it:

1. The update writes `trial = 1` before the restart.
2. `link_guard()` adds 1 at each start of the new app.
3. When the link has been up for 15 s, the app sets `trial` back to 0. `GET /info` shows `on trial` until then.
4. A new app that gets to its third start without that sets the old app to boot again, and restarts.

So a new app that crashes, or cannot bring the link up, gives way to the old one. Pressing reset three times does the same by hand.

Two limits:

- It cannot help when the new app crashes before `link_guard()` runs. Call it first.
- It counts starts. A new app that hangs with no link and never restarts stays there until someone presses reset.

## What we checked on the robot (2026-11-12)

- Wi-Fi join, the page, `/state`, `/cmd`, UDP telemetry and commands, and four updates in a row.
- A new app showed `on trial` and was accepted after 15 s.
- A wrong key, no key and a bad image gave 401, 401 and 400, and the robot did not restart.
- Not checked: a real rollback (an app that fails three starts), and the robot's own network.
