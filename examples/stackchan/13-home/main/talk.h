// talk: a spoken conversation with OpenAI's Realtime API, full duplex, with
// echo cancelling on both microphones (esp-sr). See talk.c.
#pragma once

#include <stdbool.h>
#include "esp_err.h"
#include "esp_http_server.h"

// Start the audio (speaker and microphones) and the echo canceller. It
// runs from then on; with no talk, the speaker plays silence.
esp_err_t talk_start(void);
// Start or end a talk (a session with the Realtime API).
void talk_open(void);
void talk_close(void);
// True while a talk is open or opening.
bool talk_active(void);
// True once the server listens (the session has started).
bool talk_live(void);
// The loudness of the robot's voice now, 0 to 1, for the mouth.
float talk_level(void);
// The function that runs a tool call from the model: name, its arguments as
// JSON, and a buffer for the result (text the model reads). It runs in the
// talk's tools task.
typedef void (*talk_tool_fn)(const char *name, const char *args, char *out, size_t size);
void talk_set_tool(talk_tool_fn fn);
// GET /talk and POST /talk?start=1|stop=1|test=lanes|test=aec (see talk.c).
esp_err_t talk_get(httpd_req_t *req);
esp_err_t talk_post(httpd_req_t *req);
