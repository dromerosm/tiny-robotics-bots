// json: see json.h.
#include "json.h"

#include <stdarg.h>
#include <stdio.h>

static void flush(jw_t *w)
{
    if (w->n && w->err == ESP_OK) {
        w->err = httpd_resp_send_chunk(w->req, w->buf, (ssize_t)w->n);
    }
    w->n = 0;
}

static void put(jw_t *w, char c)
{
    if (w->n == sizeof(w->buf)) {
        flush(w);
    }
    w->buf[w->n++] = c;
}

void jw_begin(jw_t *w, httpd_req_t *req)
{
    w->req = req;
    w->n = 0;
    w->err = ESP_OK;
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
}

void jw_raw(jw_t *w, const char *text)
{
    while (*text) {
        put(w, *text++);
    }
}

void jw_fmt(jw_t *w, const char *fmt, ...)
{
    char text[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(text, sizeof(text), fmt, args);
    va_end(args);
    jw_raw(w, text);
}

void jw_str(jw_t *w, const char *text)
{
    if (!text) {
        jw_raw(w, "null");
        return;
    }
    put(w, '"');
    for (const unsigned char *p = (const unsigned char *)text; *p; p++) {
        if (*p == '"' || *p == '\\') {
            put(w, '\\');
            put(w, (char)*p);
        } else if (*p < 0x20) {
            char esc[8];
            snprintf(esc, sizeof(esc), "\\u%04x", *p);
            jw_raw(w, esc);
        } else {
            put(w, (char)*p); // UTF-8 goes through as it is
        }
    }
    put(w, '"');
}

void jw_tenths(jw_t *w, long value)
{
    if (value < 0) {
        jw_raw(w, "null");
    } else {
        jw_fmt(w, "%ld.%ld", value / 10, value % 10);
    }
}

esp_err_t jw_end(jw_t *w)
{
    flush(w);
    if (w->err == ESP_OK) {
        w->err = httpd_resp_send_chunk(w->req, NULL, 0);
    }
    return ESP_OK; // a client that went away is not the server's error
}
