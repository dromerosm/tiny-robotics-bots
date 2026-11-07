// home: the plugs on the home network (tr_tuya), on the screen, by voice and
// on the web. See home.c.
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"
#include "esp_http_server.h"

// Read the devices from CONFIG_HOME_DEVICES and start the worker task.
esp_err_t home_start(void);
// How many devices are set (0 when TUYA_DEVICES was empty at build time).
int home_count(void);

// The screen. open shows it and keeps the numbers fresh (every 5 s), close
// stops that. draw paints all of it; call it when home_dirty() says so.
// tap returns false when the tap closes the screen (the title bar).
void home_open(void);
void home_close(void);
void home_draw(void);
bool home_dirty(void);
bool home_tap(int x, int y);

// The voice: a tool call from the model (talk_set_tool).
void home_tool(const char *name, const char *args, char *out, size_t size);

// GET /home (the plugs, as text) and POST /home?dev=..&sw=..&on=0|1[&confirm=1].
esp_err_t home_get(httpd_req_t *req);
esp_err_t home_post(httpd_req_t *req);
