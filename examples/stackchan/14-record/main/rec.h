// rec: record the room and upload it to our Worker (workers/rec), in pieces
// of 10 s. See rec.c.
#pragma once

#include <stdbool.h>
#include "esp_err.h"
#include "esp_http_server.h"

// Start or stop a recording. Not while a talk is open.
esp_err_t rec_start(void);
void rec_stop(void);
// True from rec_start() until the last piece is uploaded (or given up).
bool rec_active(void);
// True while it still takes sound (false once stopped, while it uploads).
bool rec_listening(void);
// GET /rec and POST /rec?start=1|stop=1|src=clean|mic (see rec.c).
esp_err_t rec_get(httpd_req_t *req);
esp_err_t rec_post(httpd_req_t *req);
