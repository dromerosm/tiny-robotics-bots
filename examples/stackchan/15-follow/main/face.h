// face: find faces in a camera frame, with Espressif's esp-dl
// (human_face_detect, the MSR + MNP model). A small C wrapper, so main.c
// stays in C.
#pragma once

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FACE_MAX 4 // we keep the biggest ones

typedef struct {
    int x0, y0, x1, y1; // the box, in pixels of the frame
    float score;        // 0..1
} face_box_t;

// Load the model (the first call takes a while). ESP_ERR_NO_MEM if it does not fit.
esp_err_t face_start(void);
// Free the model again. face_start() loads it again.
void face_stop(void);
// Look for faces in an RGB565 frame (high byte first, as the camera gives it).
// Returns how many it put in boxes, biggest first (the closest person).
int face_find(const uint8_t *rgb565, int w, int h, face_box_t *boxes, int max);

#ifdef __cplusplus
}
#endif
