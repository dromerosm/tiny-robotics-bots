// radar: a screen for the distance sensor on port A. The head turns from side
// to side, and the screen shows what the sensor finds at each angle, with the
// distance and the angle on top.
#pragma once

#include <stdint.h>

// Start a new picture. Nothing moves until radar_step().
void radar_open(void);
// Once per frame, from the face task, while the view is open: the next step
// of the head and the new picture. now is esp_timer_get_time().
void radar_step(int64_t now);
// The head goes back to the middle and the servos go off (0.7 s).
void radar_close(void);
