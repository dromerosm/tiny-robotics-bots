// tr_board: functions the parts of this component share, not for examples.
#pragma once

#include <stdbool.h>
#include "esp_err.h"

// tr_sensors tells tr_sleep about each touch read. source 0 is the screen,
// 1 the head. down: the screen is touched, or a head zone got stronger.
// Returns false when the example must not see this touch: it woke the robot,
// and the finger is still there.
bool tr_sleep_touch(int source, bool down);

// The sleep steps of tr_sensors and tr_cam. Called from the sleep task.
void tr_sensors_sleep(bool asleep, bool camera, bool motion);
esp_err_t tr_cam_power_off(void);
bool tr_cam_started(void);
