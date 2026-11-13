// tof: a Unit ToF4M distance sensor (VL53L1X) on the Fire's Grove port A. That
// port is the robot's own I2C bus, so the control task reads the sensor
// between its other reads, and never waits for it.
#pragma once
#include <stdbool.h>
#include "driver/i2c_master.h"
#ifdef __cplusplus
extern "C" {
#endif

// Look for the sensor on the bus and start it: long distance mode, one
// measure every 100 ms, up to 4 m. False if there is none, or it does not
// start. It takes up to 0.4 s, so call it before the control loop runs.
bool tof_init(i2c_master_bus_handle_t bus);

// Ask for a new measure, without waiting. 1: there is one, and *mm has the
// distance, or -1 when the sensor saw nothing it trusts (nothing in range).
// 0: no new measure yet. -1: the sensor did not answer.
int tof_poll(int *mm);

// How wide the sensor looks, and where. It has 16 x 16 detectors and uses a
// square of them: `size` is its side (4 to 16; 16 is the whole view, 27
// degrees in M5Stack's docs), and `col` and `row` are the column and the row
// of its center (8 and 7 are the middle). One detector is about 1.7 degrees.
// Which of the two is up and down depends on how the unit is mounted. A value
// that does not fit is brought inside the range. It takes about 1 ms; false
// if the sensor did not answer.
bool tof_set_zone(int size, int col, int row);

#ifdef __cplusplus
}
#endif
