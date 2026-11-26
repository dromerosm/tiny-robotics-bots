# AtomS3 Lite 01: a HomeKit bridge for the plugs

A small AtomS3 Lite puts the Tuya plugs at home into Apple's Home app. Each outlet shows up as its own accessory, so you can switch it from an iPhone, ask Siri, or use it in an automation. With an Apple TV as home hub it also works from outside the house.

This is StackChan's [17-homekit](../../stackchan/17-homekit) without the screen, the touch and the servos. The bridge was on StackChan first. It moved here to free the robot, and to have the bridge in a small box that is always on.

| | |
| --- | --- |
| **Board** | AtomS3 Lite (ESP32-S3FN8: 8 MB flash, no PSRAM) |
| **Level** | Advanced |
| **Emulator** | ✅ Native `esp-emu`. It checks that Wi-Fi and HomeKit start. Pairing needs the real board |
| **Hardware used** | RGB LED (G35), button (G41) |
| **Also needs** | `TUYA_DEVICES` in the env file, and an iPhone or iPad with the Home app |

## What you learn

- The same HomeKit bridge on a board with no PSRAM and no screen.
- A WS2812 LED with ESP-IDF's RMT bytes encoder, with no extra library.
- How to show a HomeKit setup code safely when there is no screen: a page that opens only after a press on the button.
- How to move a paired HomeKit accessory to new hardware, so Home keeps its rooms, names and automations.

## What changed from 17-homekit

The plugs, HomeKit, the energy, the API and `/panel` are the same code (`plugs.c`, `bridge.c`, `energy.c`, `api.c`, `info.c`, `json.c`, `panel.html`). See [17-homekit](../../stackchan/17-homekit) for how they work. The changes:

| | 17-homekit (StackChan) | 01-homekit (Atom) |
| --- | --- | --- |
| Name on the network | `stackchan.local` | `bridge.local` (`CONFIG_TR_OTA_HOSTNAME`) |
| In Home | StackChan Bridge | Atom Bridge |
| Flash | 16 MB, two 5 MB app slots | 8 MB, two 3 MB app slots (`partitions.csv`) |
| PSRAM | 8 MB | None |
| Setup code and QR | On the screen, QR tab | On `GET /pair`, after a press on the button |
| Update token | On the screen and the USB port | On the USB port only |
| `/panel` asks for the token | With `prompt()` | With a field on the page: browsers inside apps have no `prompt()` |
| Energy per day on `/panel` | Units in each cell | Plain numbers, units in the header, so it fits a phone |
| State | Screen tabs | The LED |
| Built | Every shared component | Only `main` and what it needs (`COMPONENTS`) |

The app is 1.1 MB. On the board, with the three plugs read and one controller paired, the lowest free memory since boot was 176 KB.

## Build

The firmware needs one env file with the Wi-Fi networks and the plugs. Build it outside git, with mode 600:

```bash
E=$(mktemp) && chmod 600 $E
cat ../../../m5stack/wifi.env > $E && echo >> $E
python3 ~/GitHub/home-iot/scripts/stackchan_env.py >> $E
WIFI_ENV=$E scripts/build.sh examples/atoms3-lite/01-homekit
rm $E
```

Output: `out/atoms3-lite-01-homekit-esp32s3.bin` (to flash over USB) and `examples/atoms3-lite/01-homekit/build/homekit.bin` (to send over Wi-Fi).

In the emulator, build with no env file, so the board joins esp-emu's own network:

```bash
scripts/build.sh examples/atoms3-lite/01-homekit
scripts/smoke.sh examples/atoms3-lite/01-homekit
```

## Run it

The first time, flash over USB. The board uses the chip's own USB, so esptool puts it in download mode by itself:

```bash
esptool -p /dev/cu.usbmodem101 write-flash 0 out/atoms3-lite-01-homekit-esp32s3.bin
```

This erases NVS: the pairing, the setup code, the energy history and the update token. After that, send updates over Wi-Fi, as in StackChan's [05-ota](../../stackchan/05-ota):

```bash
curl -H "X-Token: $TOKEN" --data-binary @examples/atoms3-lite/01-homekit/build/homekit.bin http://bridge.local/update
```

The update token is printed on the USB port at boot. It is never in `GET /log`.

### The LED

| Color | Means |
| --- | --- |
| Red | No Wi-Fi |
| Blue | Wi-Fi, but not paired with Home |
| Blue, blinking | The pairing page is open |
| Amber | Paired, but a plug does not answer |
| Dim green | All good |

A press on the button flashes it white. The colors are dim on purpose: the board is on all day.

### Add it to Home

1. Press the button once. The LED blinks blue.
2. Within two minutes, open `http://bridge.local/pair` on the iPhone, on the same Wi-Fi. It shows the QR code and the setup code.
3. In Home: **+**, **Add Accessory**, and scan the code. Or tap **More options**, pick **Atom Bridge** and type the code.
4. Home warns that the accessory is not certified. That is expected: the open SDK has no Apple certificate. Tap **Add Anyway**.

Without a press, `/pair` answers 403 and shows nothing. While the bridge is paired it only says so: HomeKit takes a new pairing only after `POST /home?unpair=1`. So someone on the network cannot read the code. They would need to press the button too.

The setup code is made at random on the first boot and kept in NVS. It is also printed on the USB port.

## Move the bridge from StackChan, and keep Home as it is

Done on 2026-11-26. Home kept the outlets' rooms, names and automations, and did not ask to add anything again.

HomeKit knows an accessory by an ID and a key pair, not by its hardware. The SDK keeps them in NVS, so the Atom can take them over:

| Namespace | What |
| --- | --- |
| `hap_main` | The accessory ID, its key pair (`ltska`, `ltpka`), the configuration number, and the number of each outlet |
| `hap_ctrl` | The paired controllers (the iPhone's admin key) |
| `bridge` | The setup code and the setup id |
| `energy` | The energy history (not needed, but good to keep) |

The rest of StackChan's NVS stays on StackChan: Wi-Fi calibration, the servos' zero positions and its update token.

Two bridges with the same identity must never run at the same time. Steps:

1. Read StackChan's NVS over USB and leave it stopped in download mode:
   `esptool -p <stackchan-port> --after no-reset read-flash 0x9000 0x4000 stackchan-nvs.bin`
2. Keep only those namespaces, in a new image the size of the Atom's NVS. In the ESP-IDF container:
   `python nvs_move.py stackchan-nvs.bin out hap_main hap_ctrl bridge energy`, then
   `python $IDF_PATH/components/nvs_flash/nvs_partition_generator/nvs_partition_gen.py generate out/nvs.csv atom-nvs.bin 0x6000`.
   `nvs_move.py` prints only names and sizes, never values. Read the new image back with the same script and compare the files, to check it.
3. Restart StackChan (`esptool ... run`) and send it `POST /home?unpair=1`. That makes it forget its pairings and its identity. It only changes StackChan: Home is not told anything.
4. Flash the Atom without letting it boot in between: `erase-flash`, then the image at 0, then `atom-nvs.bin` at `0x9000`, all with `--after no-reset`. If it booted with an empty NVS, it would make a new identity.
5. Reset the Atom. Its log says `HomeKit bridge with 8 outlets, 1 paired`. `dns-sd -L "Atom Bridge" _hap._tcp` shows `sf=0` (paired).

The Atom's accessory has firmware revision 1.1, where 17-homekit had 1.0. When the SDK sees a new revision it raises the configuration number (`c#` in mDNS). Home then reads the accessories again and learns the new name, model and serial.

To go back, write `stackchan-nvs.bin` to StackChan at `0x9000` (and stop the Atom first). Both dumps hold HomeKit's private key: keep them out of git, with mode 600.

## Expected output

On the board on 2026-11-26, from `GET /log`, after the move:

```
I (466) tr_ota: homekit v0.5.0-13-gfb27ae7-dirty, built Nov 26 2026 08:02:21, running from ota_0 (valid)
I (676) bridge: HomeKit bridge with 8 outlets, 1 paired
I (7736) tr_ota: Wi-Fi: robot at 192.168.1.49, http://bridge.local/
I (10026) plugs: Salon at 192.168.1.33 answers
I (12026) plugs: Trabajo at 192.168.1.36 answers
I (13526) plugs: Personal at 192.168.1.34 answers
I (13536) plugs: first read: 3 of 3 devices answer
```

## Checked so far

- The firmware on the board: it starts, joins Wi-Fi, reads the three real plugs and announces `_hap._tcp` with `sf=0`.
- The move from StackChan: the Atom has the same identity and its one controller. The Home app shows the 8 outlets with their rooms and names, and switching an outlet works (seen by the owner on 2026-11-26).
- The LED: dim green with everything working (seen by the owner).
- `/pair` while paired: it says so, and shows no code.
- The API, with a script from a Mac on 2026-11-26 (109 checks, all passed): every endpoint, the 3 devices and 8 outlets, bad input (400), unknown paths (404), every bad form of the token (403, also on `/update` and `/home?unpair=1`), no CORS, the token never in an answer, and 40 requests 8 at a time (all answered in 2.2 s, no restart, 154 KB free at the lowest).
- `/panel` in a browser, light and dark, at desktop and phone width: plugs, energy and the bridge show well, and a wrong token says so. The Apple TV shows as the home hub, connected to the Atom.
- In esp-emu: Wi-Fi and HomeKit start (the smoke test).

Not checked yet:

- `/pair` with the button, and adding the bridge from zero with its QR code.
- Switching an outlet from `/panel` or the API.
- The other LED colors (red, blue, amber).
- A whole day running: memory, and the Wi-Fi signal. The board joined `MOVISTAR_7B00` at -74 dBm, which is weak.
- The energy history kept across the move, against the numbers on StackChan.

## Try next

- Put the board where the Wi-Fi is stronger, and watch the lowest free memory for a day.
- A long press on the button to forget the pairings, as a way back without the token.
