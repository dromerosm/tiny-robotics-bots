# StackChan 13: the plugs at home

12-talk's spoken conversation, plus the smart plugs on the home network. Tap the robot's left eye and you get a Home screen with one button per outlet and the power each device draws. In a talk you can ask for it out loud: "apaga la tele", "¿cuánto está gastando el trabajo?". The robot talks to the plugs directly over the local network, with no cloud, through [`tr_tuya`](../../../components/tr_tuya).

The face, the audio, the echo canceller and the talk are 12-talk's as of commit `7ef1dae`: GPT-Live (gpt-live-1) through our Worker, gpt-6-luna for searches, and the face's color for the talk's state.

| | |
| --- | --- |
| **Robot** | StackChan (CoreS3, ESP32-S3) |
| **Level** | Advanced |
| **Emulator** | ✅ Native `esp-emu`, without screen or audio. Built with `TUYA_DEVICES`, it reads the real plugs through the emulator's network |
| **Hardware used** | Speaker, both microphones, screen, touch, head touch |
| **Also needs** | `TUYA_DEVICES` in the env file, and for the voice the talk Worker with `worker-home.patch` |

## What you learn

- Tuya's local protocol (3.3 and 3.4) on the robot, with PSA crypto and plain sockets.
- Function tools that the robot runs for GPT-Live's delegated model (gpt-6-luna), next to luna's own `web_search`.
- A safety check that depends on the robot's own Wi-Fi.

## The devices

They come from one line in the env file passed to the build (`WIFI_ENV`), never from git:

```
TUYA_DEVICES=Salon,34,192.168.1.33,<id>,<key-hex>,1:Main:main,7:USB:usb;Trabajo,...;Personal,33,...,1:AP:Access_Point:ap,2:TV:TV,3:Mac:MacBook_Air,7:USB:usb
```

One device per `;`: name, protocol version (33 or 34), IP, device ID, local key in hex, then one `dp:label:name` per outlet. `label` goes on the screen button, `name` is for the voice and the web (`_` for a space). `~/GitHub/home-iot` makes the line from its `devices.json` and the keys in `~/.secrets`:

```bash
~/GitHub/home-iot/.venv/bin/python ~/GitHub/home-iot/scripts/stackchan_env.py
```

### The access point outlet

`:ap` marks the outlet that powers the Wi-Fi access point. If the robot is on `CONFIG_HOME_AP_SSID` (`HOME_AP_SSID` in the env file, next to `TUYA_DEVICES`) and you switch that outlet off, the robot loses its own network. So, only while it is on that network, it asks first:

- **Screen:** the first tap turns the button yellow ("Sure?"). A second tap within 4 s switches it off.
- **Voice:** the tool answers `needs_confirmation`. The model asks you, and calls again with `confirmed=true` only after a clear yes.
- **Web:** `409 Conflict` until you add `confirm=1`.

On any other network, that outlet behaves like the rest.

## Build

The firmware needs one env file with everything: Wi-Fi, the talk, and the plugs. Build it outside git, with mode 600:

```bash
E=$(mktemp) && chmod 600 $E
cat ../../../m5stack/wifi.env > $E && echo >> $E
~/GitHub/home-iot/.venv/bin/python ~/GitHub/home-iot/scripts/stackchan_env.py >> $E
WIFI_ENV=$E scripts/build.sh examples/stackchan/13-home
rm $E
```

Output: `out/stackchan-13-home-esp32s3.bin` and `examples/stackchan/13-home/build/home.bin`.

### The voice

The robot opens `TALK_URL?profile=home` (`TALK_URL` is the Worker's `wss://.../live`, as in 12-talk). With that profile the Worker:

- adds two function tools to gpt-6-luna's tools: `get_home_status` and `set_switch` (device, switch, on, confirmed), and tells gpt-live-1 to hand plug requests to luna;
- passes to the robot luna's finished function calls: `response.event` with a `response.output_item.done` whose item is a `function_call`. Every other `response.event` is still dropped;
- takes from the robot `response.item.create` (only with a `function_call_output`) and `response.create`.

Without the profile, the Worker does what it did for 12-talk. The change is in `worker-home.patch`, against the Worker of `7ef1dae`:

```bash
git apply examples/stackchan/13-home/worker-home.patch
cd workers/talk-key && npx wrangler deploy
```

On the robot, the tools task in `talk.c` runs each call with `home_tool()`, sends its output, then `response.create`: without it luna does not go on.

## Run it

In the emulator, with no Wi-Fi networks in the env file so it joins esp-emu's own network (`myssid`):

```bash
E=$(mktemp) && chmod 600 $E
~/GitHub/home-iot/.venv/bin/python ~/GitHub/home-iot/scripts/stackchan_env.py > $E
WIFI_ENV=$E scripts/build.sh examples/stackchan/13-home && rm $E
.tools/bin/esp-emu --chip esp32s3 --firmware out/stackchan-13-home-esp32s3.bin --timeout 30s
```

On the robot, as for 12-talk (`POST /update` with `build/home.bin`). Then tap the left eye, or:

```bash
curl http://stackchan.local/home
curl -X POST -H "X-Token: <token>" "http://stackchan.local/home?dev=Personal&sw=TV&on=0"
```

| Endpoint | What |
| --- | --- |
| `GET /home` | Every device: each outlet on or off, and the power now |
| `POST /home?dev=..&sw=..&on=0\|1` | Switch one outlet (token needed). `sw` takes the name or the label (`TV`, `MacBook+Air`, `usb`). Add `confirm=1` for the access point outlet |
| `POST /face?view=home` | Open the Home screen |

## Expected output

In esp-emu on 2026-11-02, reading the real plugs through the emulator's network:

```
I (1771) home: 3 devices
I (3971) tr_ota: Wi-Fi: robot at 192.168.4.2, http://stackchan.local/
I (5151) home: first read:
Salon: main on, usb on, 0.0 W.
Trabajo: main on, usb on, 13.7 W.
Personal: Access Point off, TV on, MacBook Air on, usb off, 0.0 W.
```

## How it works

```
left eye tap -> Home screen (face task) -- taps --> job queue --> home task --> tr_tuya --> plug
voice: gpt-live-1 -> gpt-6-luna -- function call (via Worker) --> tools task --> home_tool() --> tr_tuya
web:   POST /home ---------------------------------> set_by_name() --> tr_tuya
```

- **Nobody waits on the network in the face task.** A tap puts a job on a queue. The home task (its stack in PSRAM) switches the outlet and reads the device again. The button stays dark until the device answers.
- **Function calls** (a `response.event` carrying a `function_call`) go to a tools task in `talk.c`, also with its stack in PSRAM. It runs `home_tool()`, sends the result (`response.item.create` with a `function_call_output`), then `response.create` so luna goes on. It holds 12-talk's WebSocket lock while it sends.
- **A status is the device's own answer.** A device may first push part of its state (the power strip sends its voltage alone). `tr_tuya` merges those pushes in but waits for the reply to its query, or the screen would show outlets as unknown.
- **One connection at a time, never kept open.** The power strip takes a single connection. While the phone app holds it, the strip refuses the robot ("no answer" on the screen).
- **Fresh numbers** every 5 s while the Home screen is open, and on every tool call. Nothing in between: no polling while you don't look.
- **Internal RAM** stays for the echo canceller: tr_tuya takes one 3 KB block in PSRAM per call, and both new tasks have their stacks in PSRAM (`CONFIG_FREERTOS_TASK_CREATE_ALLOW_EXT_MEM`).

## Checked so far

- `tr_tuya` from a Mac against the three real devices: status and switching, 3.3 and 3.4.
- The patched Worker (Node): profile=home adds the tools and keeps 12-talk's session as it was; only function-call events pass.
- In esp-emu: the firmware starts, reads `TUYA_DEVICES`, waits for Wi-Fi and reads the three real plugs through the emulator's network (PSA crypto and lwIP, as on the robot).

Not checked yet (nothing has been flashed to the robot):

- The Home screen and its taps, including the "Sure?" step (esp-emu has no screen or touch).
- The voice tools in a real talk: the Worker patch is not deployed, and esp-emu has no audio. The patched Worker's session and filters pass a Node test; the event shapes come from OpenAI's delegation guide and are not yet checked against a real call.
- `GET /home` and `POST /home` (esp-emu can't forward a port to the robot's web server).
- Internal RAM during a talk while the plugs are being read.

## Try next

- Apply and deploy the Worker patch, flash it, and check the items above.
- How long a plug command takes by voice (gpt-live-1 -> luna -> robot -> luna -> gpt-live-1).
- Whether luna keeps the `needs_confirmation` from one delegation to the next.
- A button for a whole device (all outlets off at once).
- Show a day's energy: the plugs' dp 17 adds up energy between reports.
