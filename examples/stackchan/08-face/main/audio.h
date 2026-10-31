// The audio analyzer screen (audio.c).
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

// Start the microphones (once) and draw the empty screen with its title bar.
esp_err_t audio_open(void);
// Read 32 ms of sound and draw the bars. Takes about 60 ms.
esp_err_t audio_frame(void);

// The last sound direction: lag in samples at 16 kHz between the two
// microphones, minus the calibrated offset (positive: MIC1 heard it first), how well they matched
// (0..1), its level, and how long ago. False if no sound was loud enough yet.
bool audio_direction(float *lag, float *match, float *db, int64_t *age_us);
// The angle in degrees from straight ahead for a lag (negative: left, seen
// from the front of the robot; two microphones cannot tell front from back).
float audio_angle(float lag);
// "< left", "front" or "right >" for a lag, seen from the front of the robot.
const char *audio_side(float lag);

// Measure the channels' fixed offset from the next 5 clear sounds, which must
// come from straight in front, and keep it in NVS.
void audio_calibrate(void);
// The offset in use (samples at 16 kHz), and sounds still to take (0: done).
void audio_calibration(float *offset, int *sounds_left);
