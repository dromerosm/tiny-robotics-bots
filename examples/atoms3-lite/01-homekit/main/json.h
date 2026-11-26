// json: a small JSON writer that sends a chunked HTTP answer as it goes, so no
// handler needs a buffer as big as its answer. It can also write to any sink,
// such as a buffer for an upload. See json.c.
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include "esp_http_server.h"

// Takes n bytes of the text. NULL and 0 at the end.
typedef esp_err_t (*jw_sink_t)(void *ctx, const char *data, size_t n);

typedef struct {
    jw_sink_t sink;
    void *ctx;
    char buf[512];
    size_t n;
    esp_err_t err; // the first send error; after it nothing more is sent
} jw_t;

// Start an answer: sets the JSON content type.
void jw_begin(jw_t *w, httpd_req_t *req);
// Start writing to a sink instead.
void jw_begin_sink(jw_t *w, jw_sink_t sink, void *ctx);
// Text as it is (for punctuation and numbers).
void jw_raw(jw_t *w, const char *text);
// printf, as it is. Up to 127 bytes per call.
void jw_fmt(jw_t *w, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
// A JSON string, in quotes and escaped. NULL writes null.
void jw_str(jw_t *w, const char *text);
// A number in tenths (123 -> 12.3), or null when it is negative (not known).
void jw_tenths(jw_t *w, long value);
// Send what is left and end the answer.
esp_err_t jw_end(jw_t *w);
