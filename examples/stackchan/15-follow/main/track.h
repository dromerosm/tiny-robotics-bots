// track: the head follows the biggest face the camera sees, and looks around
// when there is none. From 11-track, without the record of finds. It can stop and give its memory
// back: see track_quiet().
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
// Once per frame, from the face task. quiet: the camera is not wanted (a
// talk or a recording is on, or nobody follows and the camera view is
// closed). move: the head may follow the face and look around; with false
// the finder still finds faces, for the camera view and the eyes. now is
// esp_timer_get_time().
void track_step(int64_t now, bool quiet, bool move);
// The camera view. track_show(true): the finder hands its frames to the
// screen, and works at full speed. track_frame(): from the face task, in a
// loop while the view is open. It shows the next frame, 320 x 240, with a
// box on each face (green for the one the head follows) and the center, and
// waits at most one tick for it. Nothing to show while the finder is stopped.
void track_show(bool on);
void track_frame(void);
// How far the face is from the center of the camera frame, -1 to 1 on each
// axis, for the eyes. 0 with no face.
void track_eyes(float *x, float *y);
// One line for GET /face. Returns its length.
int track_status(char *text, size_t size);
