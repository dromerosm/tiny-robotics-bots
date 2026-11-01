# tr_ota

Home Wi-Fi, updates over Wi-Fi (OTA) and rollback for StackChan examples. [05-ota](../../examples/stackchan/05-ota) shows it on its own.

Every example that runs on the robot needs it. An app sent over Wi-Fi boots "on trial" and must mark itself valid. If it does not, the robot goes back to the old app on the next restart. An app without this component also can't take the next update.

## Use it

1. Add the components folder to the project's `CMakeLists.txt`, before `project()`:

   ```cmake
   set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../../components")
   ```

2. Require it in `main/CMakeLists.txt` with `PRIV_REQUIRES tr_ota`.

3. Add these lines to `sdkconfig.defaults`. They set 16 MB flash, the two app slots and rollback:

   ```
   CONFIG_ESPTOOLPY_FLASHSIZE_16MB=y
   CONFIG_PARTITION_TABLE_CUSTOM=y
   CONFIG_PARTITION_TABLE_CUSTOM_FILENAME="../../../components/tr_ota/partitions.csv"
   CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y
   ```

4. Call it once at boot:

   ```c
   #include "tr_ota.h"

   ESP_ERROR_CHECK(tr_ota_start());
   ```

`tr_ota_start()` returns at once. It does not wait for Wi-Fi, so the example keeps running with or without a network. It sets up NVS, `esp_netif` and the default event loop if the example has not done it yet.

## Wi-Fi networks

Two networks are set at build time from a file outside the repo:

```sh
SSID_01=first-network
PWD_SSID_01=its-password
SSID_02=second-network
PWD_SSID_02=its-password
```

```bash
WIFI_ENV=path/to/wifi.env scripts/build.sh examples/stackchan/<example>
```

`scripts/build.sh` passes the file's lines to the build as environment variables, and the component's `Kconfig` reads them. The passwords end up in `sdkconfig`, `build/` and `out/`, which git ignores. Without `WIFI_ENV` (as in CI), the robot joins esp-emu's built-in network, `myssid`.

At boot the robot scans and joins the known network with the strongest signal. When the link drops, it scans again.

## On the network

| | |
| --- | --- |
| Name | `stackchan.local` (mDNS) |
| `GET /` | Status as plain text: firmware, slot and its state, uptime, why it last restarted, free memory, Wi-Fi (network, signal, channel, IP), plus lines the example adds |
| `GET /log` | The last 4 KB of `ESP_LOG` lines, oldest first. Kept in RTC memory, so they survive a restart, a crash or a rollback (not a power cut). Each boot adds a line `--- boot, last restart: ... ---` |
| `GET /favicon.ico`, `GET /apple-touch-icon.png` | The robot icon: an SVG for the browser tab, and a 180 x 180 PNG for Safari, which asks for it (also as `apple-touch-icon-precomposed.png`) when you save the page to the home screen. `touch-icon.py` draws the PNG from the same shapes as `favicon.svg` |
| `POST /update` | A new app image (`build/<project>.bin`, not the merged image), with the header `X-Token: <token>` |

An example adds its own endpoints with `tr_ota_add_uri()`, up to 16. Endpoints that change the robot call `tr_ota_token_ok(req)` first: it checks `X-Token`, sends the 403 answer and writes a line to `/log`. The handler then returns `ESP_OK`, because the answer is sent. With `ESP_FAIL` the server closes the connection and logs only `uri handler execution failed`, with no reason. `tr_ota_send_error()` sends other errors the same way. [07-head](../../examples/stackchan/07-head) does this.

An example adds its own lines to `GET /` with `tr_ota_add_status(fn)`, up to 4. `fn` writes text that ends in `\n`. It runs in the web server's task each time someone asks. For example, `tr_ota_add_status(tr_board_status)` adds the battery (see [tr_board](../tr_board)).

A browser that opens `/` goes to `/app` when the example has a page there. `curl` still gets the text.

`GET /log` only has `ESP_LOG` lines, not `printf` output. The token is printed with `printf` on purpose, so it never reaches `/log`, which anyone on the network can read. The Wi-Fi driver's own logs are cut to warnings, so the 4 KB hold more useful lines. The log does show the network name.

The token is made at random on the first boot and saved in NVS. It is shown only on the USB port. A new flash over USB erases it and makes a new one.

```bash
curl -H "X-Token: <token>" --data-binary @examples/stackchan/<example>/build/<project>.bin http://stackchan.local/update
```

## Rollback

An app on trial is marked valid once it has Wi-Fi, because the web server already runs. With no Wi-Fi within 60 seconds, it goes back to the old app. If it crashes before that, the bootloader goes back on the next boot. An app flashed over USB is not on trial.

Tested on the robot on 2026-10-31, with this component inside `05-ota`: an update was back in 6 s and marked valid. A build for a network that does not exist was back on the old app after 67 s.

## stackchan.local and IPv6

The robot has an IPv6 link-local address (`fe80::...`) next to its IPv4 address. Without it, mDNS answered the IPv6 question (AAAA) with no IPv6 address and without saying there is none. macOS (curl, ssh) then waited 5 s on every lookup of `stackchan.local`. Safari did not wait, because it uses the IPv4 address as soon as it has it. Now the lookup takes about 7 ms, also while the robot sleeps, and the requests go over IPv6.

The examples set `CONFIG_LWIP_IPV6_DUP_DETECT_ATTEMPTS=0`. With the duplicate address check, the address never became valid on our network. **[inference]** The router may send the robot's own check back to it. The address comes from the robot's MAC address, so a duplicate is very unlikely.
