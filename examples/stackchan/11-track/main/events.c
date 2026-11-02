// events: see events.h.
#include "events.h"

#include <stdio.h>
#include <string.h>
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"

#define MAX_EVENTS 16384

typedef struct {
    uint32_t ms;     // since boot
    char kind;       // F find, M move, S seen, L lost, G search look, B battery low, H home
    char axis;       // M: 'Y' or 'P'
    int8_t faces;    // F
    int16_t cx, cy;  // F: the best face's center, in frame pixels
    int16_t box_w;   // F: its width
    int16_t move_ms; // M
    float a, b, c;   // F: score, err_x, err_y. M: from, goal
} event_t;

static event_t *ring;  // in PSRAM
static uint32_t count; // events written since boot; the ring keeps the last MAX_EVENTS
static portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;

esp_err_t events_start(void)
{
    if (!ring) {
        ring = heap_caps_calloc(MAX_EVENTS, sizeof(event_t), MALLOC_CAP_SPIRAM);
    }
    return ring ? ESP_OK : ESP_ERR_NO_MEM;
}

static void add(const event_t *e)
{
    if (!ring) {
        return;
    }
    taskENTER_CRITICAL(&lock);
    ring[count % MAX_EVENTS] = *e;
    ring[count % MAX_EVENTS].ms = esp_timer_get_time() / 1000;
    count++;
    taskEXIT_CRITICAL(&lock);
}

void events_find(int faces, int cx, int cy, int box_w, float score, float err_x, float err_y)
{
    const event_t e = {.kind = 'F',
                       .faces = faces,
                       .cx = cx,
                       .cy = cy,
                       .box_w = box_w,
                       .a = score,
                       .b = err_x,
                       .c = err_y};
    add(&e);
}

void events_move(char axis, float from, float goal, int ms)
{
    const event_t e = {.kind = 'M', .axis = axis, .move_ms = ms, .a = from, .b = goal};
    add(&e);
}

void events_note(char kind)
{
    const event_t e = {.kind = kind};
    add(&e);
}

// One line of CSV per event. The columns that don't apply stay empty.
static int line(const event_t *e, char *out, size_t size)
{
    switch (e->kind) {
        case 'F':
            return snprintf(out, size, "%lu,find,%d,%d,%d,%d,%.2f,%.2f,%.2f,,,,\n",
                            (unsigned long)e->ms, e->faces, e->cx, e->cy, e->box_w, e->a, e->b,
                            e->c);
        case 'M':
            return snprintf(out, size, "%lu,move,,,,,,,,%s,%.1f,%.1f,%d\n", (unsigned long)e->ms,
                            e->axis == 'Y' ? "yaw" : "pitch", e->a, e->b, e->move_ms);
        default:
            return snprintf(out, size, "%lu,%s,,,,,,,,,,,\n", (unsigned long)e->ms,
                            e->kind == 'S'   ? "seen"
                            : e->kind == 'L' ? "lost"
                            : e->kind == 'G' ? "search"
                            : e->kind == 'B' ? "battery_low"
                                             : "home");
    }
}

esp_err_t events_csv(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/csv; charset=utf-8");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=\"track.csv\"");
    httpd_resp_sendstr_chunk(
        req, "ms,event,faces,cx,cy,box_w,score,err_x,err_y,axis,from_deg,goal_deg,move_ms\n");
    if (!ring) {
        return httpd_resp_sendstr_chunk(req, NULL);
    }
    taskENTER_CRITICAL(&lock);
    const uint32_t end = count;
    taskEXIT_CRITICAL(&lock);
    const uint32_t start = end > MAX_EVENTS ? end - MAX_EVENTS : 0;
    static char buf[2048]; // the web server has one task, so one buffer is enough
    int at = 0;
    for (uint32_t i = start; i < end; i++) {
        event_t e;
        taskENTER_CRITICAL(&lock);
        e = ring[i % MAX_EVENTS];
        taskEXIT_CRITICAL(&lock);
        if (at > (int)sizeof(buf) - 96) {
            if (httpd_resp_send_chunk(req, buf, at) != ESP_OK) {
                return ESP_OK; // the browser went away
            }
            at = 0;
        }
        at += line(&e, buf + at, sizeof(buf) - at);
    }
    if (at) {
        httpd_resp_send_chunk(req, buf, at);
    }
    return httpd_resp_sendstr_chunk(req, NULL);
}
