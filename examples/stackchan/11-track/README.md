# StackChan 11: follow a face, on less battery

[10-track](../10-track) with less power use. It does the same: the head follows the biggest face, the eyes and mouth lead it, and the head searches the room when there is no face. 10-track stays as it was, so we can go back to it if this one fails.

Anything not listed below works as in [10-track](../10-track): the camera view on the left eye, `GET /face`, `GET /track.csv`, following, searching, and the low-battery guard.

| | |
| --- | --- |
| **Robot** | StackChan (CoreS3, ESP32-S3) |
| **Level** | Advanced |
| **Emulator** | ✅ Native `esp-emu`, without screen, camera and servos: it checks that Wi-Fi and the server start |
| **Hardware used** | As in 10-track |

## What changed

| Change | Why |
| --- | --- |
| **Fewer finds with no face.** With no face for 1 s and the camera view closed, the finder waits 400 ms after each find: about 2 finds per second instead of 12. A face brings back full speed at once | While it searches, the head stays still 3 s at each spot, so 2 finds per second are enough. Each find takes about 80 ms of CPU at 240 MHz |
| **Servo power off at each search spot.** With no face, once the head stops, the servo power goes off too, not only the torque. While it follows a face, only the torque goes off, so the next move starts at once | The servos use power even with the torque off |
| **The camera stops before sleep.** A sleep hook stops the finder and the camera (`tr_cam_stop()`), and then the sleep step cuts the camera's power. After waking, the camera starts again | In 10-track, the camera's power stayed on while the robot slept |
| **The servos must answer after power on.** `tr_servo_power(true)` now waits until both servos answer, up to 2 s, instead of a fixed 500 ms | On battery they answer after about 860 ms. With 500 ms the first search move got no answer, and the head never moved |
| **The move home at boot checks the battery.** Below 3.6 V, on battery, the head does not go home either | The guard covered following and searching, but not this first move |
| **The finder's stack in `/log`.** Each 10 s line says how much of its 6 KB stack is left | To check that 6 KB is enough |

## Build

```bash
WIFI_ENV=../../../m5stack/wifi.env scripts/build.sh examples/stackchan/11-track
```

Output: `out/stackchan-11-track-esp32s3.bin` (merged image) and `examples/stackchan/11-track/build/track11.bin` (app image for updates over Wi-Fi).

## Run it

```bash
curl -H "X-Token: <token>" --data-binary @examples/stackchan/11-track/build/track11.bin http://stackchan.local/update
```

## Battery current

Measured on our robot on 2026-11-02: on battery (3.8 to 3.9 V), no USB, sleep turned off for the awake rows (`POST /sleep?after=0`). Each value is the robot's own 20 s average (`GET /sleep`) after 100 s in that state.

| State | 10-track | 11-track |
| --- | --- | --- |
| Following a face (moving a bit) | 181 mA | 167 mA |
| Searching, nobody there | 180 mA | **136 mA** |
| Following off, face on the screen | 135 mA | not measured (no change there) |
| Asleep | 63 mA | **32 mA** |

- **Asleep** is now the same as [09-wifi](../09-wifi) (about 30 mA). The robot sleeps most of the time, so this saves the most battery.
- **Searching** uses 44 mA less: fewer finds, and no servo power while the head is still. While still at a search spot it read 93 mA.
- **Following off** still has the camera on, at 135 mA. Stopping it there too is in the to-do.

## Checked on the robot

On 2026-11-02:

- It follows a face as smoothly as 10-track.
- With no face it searches, with about 2 finds per second, and it finds the face again. The servos answer about 857 ms after each power on.
- Asleep in 4 ms. Awake in 215 ms, and the camera is back 430 ms later: about 0.65 s in all. Four sleep and wake cycles left the free memory the same (internal 95 KB, PSRAM 6879 KB).
- The low-battery guard, with a test build that set the limit at 4000 mV (the battery was at 3.9 V): `/log` said `battery below 4000 mV: the head stays still`, `/track.csv` had a `battery_low` line, and the head stayed still with a face in front.
- The finder has 4400 bytes of its 6 KB stack left. 6 KB is enough.
- Not tried yet: two faces at once.

## How the camera stops safely

The camera must not stop while a task still holds one of its frames. Two tasks can hold one: the finder (while it finds faces in it) and the face task (while it shows it). So:

- the finder holds a lock (`cam_lock`) from the moment it takes a frame until it hands it over or gives it back;
- the face task says when it holds a frame for the screen (`showing`);
- before sleep, the hook stops the finder, takes the lock, waits until the screen has no frame, gives back a frame still waiting for the screen, and only then stops the camera.

## Try next

- Stop the camera when following is off and the view is closed (135 mA now).
- Try two faces at once.
