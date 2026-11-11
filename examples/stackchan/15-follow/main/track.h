// track: the head follows the biggest face the camera sees, and looks around
// when there is none. From 11-track, without the camera view and the record
// of finds. It can stop and give its memory back: see track_quiet().
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

// Make the finder task. Nothing runs yet: track_step() starts the camera.
esp_err_t track_init(void);
// Stop for a talk or a recording: the finder stops the camera and frees the
// face model, and the servos lose their power, so the microphones hear no
// motor. Returns when all of that is done (about half a second). From any
// task. track_step() starts everything again once it is called with
// quiet = false.
void track_quiet(void);
// Once per frame, from the face task. quiet: a talk or a recording is on,
// so the head stays still. now is esp_timer_get_time().
void track_step(int64_t now, bool quiet);
// How far the face is from the center of the camera frame, -1 to 1 on each
// axis, for the eyes. 0 with no face.
void track_eyes(float *x, float *y);
// One line for GET /face. Returns its length.
int track_status(char *text, size_t size);
