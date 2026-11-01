// events: a record of what face tracking did, to check it later. The finds
// with a face, when a face is found or lost, and every head move, with the
// time. Kept in PSRAM (16384 events, about 20 minutes of following), and
// served as CSV at GET /track.csv. A restart clears it.
#pragma once

#include "esp_err.h"
#include "esp_http_server.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t events_start(void);
// A find: how many faces, the best one's center and width, its score, and
// the error (-1 to 1) from the center of the frame.
void events_find(int faces, int cx, int cy, int box_w, float score, float err_x, float err_y);
// A head move: axis 'Y' (yaw) or 'P' (pitch), from where the servo was to the goal.
void events_move(char axis, float from, float goal, int ms);
// Something else: 'S' a face is seen (following starts), 'L' the face is
// lost, 'H' the head goes home (at boot).
void events_note(char kind);
// GET /track.csv
esp_err_t events_csv(httpd_req_t *req);

#ifdef __cplusplus
}
#endif
