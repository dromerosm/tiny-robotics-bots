# tr_board

StackChan board parts that several examples share. For now:

- The internal I2C bus (SDA G12, SCL G11). `tr_board_i2c()` makes it on the first call and returns the same bus later. Use it for every chip on that bus, so the firmware makes the bus only once.
- The battery monitor in the body: an INA226 at `0x41`, with a 10 mΩ shunt.

More parts (AXP2101, AW9523, PY32) come here when the screen and servo examples need them.

## Use it

Add the components folder to the project's `CMakeLists.txt`, before `project()`:

```cmake
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../../components")
```

Require it in `main/CMakeLists.txt` with `PRIV_REQUIRES tr_board`, and:

```c
#include "tr_board.h"

tr_battery_t b;
if (tr_battery_read(&b) == ESP_OK) {
    printf("%d mV, %d mA\n", b.millivolts, b.milliamps); // positive mA: discharging
}

ESP_ERROR_CHECK(tr_ota_add_status(tr_board_status)); // battery line on http://stackchan.local/
```

`tr_battery_read()` returns `ESP_ERR_NOT_FOUND` when no INA226 answers at `0x41`, as in the emulator. Before it trusts the chip, it checks the maker ID (`0x5449`).

## INA226

From TI's datasheet. Registers are 16 bits, high byte first. The default setup (continuous, no averaging) is enough, so we write nothing.

| Register | What | Scale |
| --- | --- | --- |
| `0x01` | Shunt voltage, signed | 2.5 µV per bit. Over 10 mΩ, that is 0.25 mA per bit |
| `0x02` | Bus voltage | 1.25 mV per bit |
| `0xFE` | Maker ID | `0x5449` ("TI") |

## On our robot (2026-10-31)

| Power | Reading |
| --- | --- |
| USB plugged in | 3.51 V, 0 mA, steady over 6 reads |
| On battery, Wi-Fi on, screen off, servos off | 3.68 to 3.72 V, 113 to 131 mA, discharging |

On battery, the sign and the size of the current look right. On USB, the voltage reads lower than on battery, and there is no charge current. We don't know why. One guess: on USB the power path cuts the battery off, and the INA226 then measures another point. Don't trust the USB-plugged reading as the battery's charge yet.
