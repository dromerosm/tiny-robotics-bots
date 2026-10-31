// tr_board: StackChan board parts shared by several examples.
//
// For now: the internal I2C bus (SDA G12, SCL G11) and the battery monitor in
// the body (INA226 at 0x41, 10 mOhm shunt). More parts (AXP2101, AW9523,
// PY32) come here when the screen and servo examples need them.
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include "driver/i2c_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// The internal I2C bus. Made on the first call; later calls return the same bus.
esp_err_t tr_board_i2c(i2c_master_bus_handle_t *bus);

typedef struct {
    int millivolts;
    int milliamps; // positive: the battery discharges; negative: it charges
} tr_battery_t;

// Read the body battery. ESP_ERR_NOT_FOUND if the INA226 does not answer (as in the emulator).
esp_err_t tr_battery_read(tr_battery_t *battery);

// One line about the battery, for a status page. Fits tr_ota_add_status().
void tr_board_status(char *text, size_t size);

#ifdef __cplusplus
}
#endif
