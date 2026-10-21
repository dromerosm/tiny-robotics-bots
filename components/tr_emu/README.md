# tr_emu

A virtual screen and head servos for StackChan firmware that runs in the [browser emulator](https://robotics.diegoromero.es/emulator/).

The emulator can't see the real SPI screen (ILI9342C) or the serial bus servos (Feetech SCS on UART1). So with this component the firmware draws into a framebuffer in RAM and writes the servo angles to a small struct. The emulator page reads both and shows them next to the console.

It only works in the emulator. It doesn't drive the real screen or servos yet.

## Use it

Add the components folder to your project's `CMakeLists.txt`, before `project()`:

```cmake
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../../components")
```

Then require it in `main/CMakeLists.txt` with `PRIV_REQUIRES tr_emu`, and:

```c
#include "tr_emu.h"

ESP_ERROR_CHECK(tr_emu_init());              // once, at boot

tr_screen_fill(TR_RGB(0, 0, 0));
tr_screen_fill_circle(90, 93, 12, TR_RGB(255, 255, 255));
tr_screen_present();                         // the page shows the new frame

tr_servo_set(30.0f, 60.0f);                  // pan, tilt in degrees
```

[04-face](../../examples/stackchan/04-face) is a full example.

## Ranges

- **Pan:** -180° to 180°. 0 is straight ahead, and positive is right, seen from the front.
- **Tilt:** 5° to 85°, the safe range on the real robot (the servo itself turns 0° to 90°). 45 is the middle, and higher looks up. The page warns if firmware without `tr_emu` limits goes outside this range.

`tr_servo_set()` clamps values to these ranges.

## How the page finds it

`tr_emu_init()` prints one line at boot:

```
@tr emu=0x3fc94c20
```

That address holds `tr_emu_shared_t` (see `tr_emu.h`): a magic number, the framebuffer address and size, a frame counter, and the pan and tilt targets. The page reads it about 15 times per second. When the frame counter changes, it reads the framebuffer as well. It also hides `@tr` lines from the console.

## Limits

- **Memory:** the framebuffer takes 150 KB of internal RAM. The browser engine has no PSRAM.
- **No tearing protection:** if you draw while the page reads, you may see half a frame for one refresh.
- **No servo model:** the firmware sets a target and the page draws the head there. There is no speed, torque or feedback.
- **Not covered:** touch, camera, IMU, sound.
