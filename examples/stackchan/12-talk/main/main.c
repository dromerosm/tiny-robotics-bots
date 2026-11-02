// 12-talk: a spoken conversation with OpenAI's Realtime API, full duplex,
// with echo cancelling on both microphones (talk.c). The face is 09-wifi's,
// without the Wi-Fi list, and the head never moves: no motor noise in the
// microphones, and no camera, no face model, so the echo canceller has core 1
// and the RAM. The mouth opens with the robot's voice.
//
// A touch on the head starts a talk, and another one ends it. So does
// POST /talk?start=1 or stop=1. A talk ends by itself after 5 minutes.
//
//   GET  /face                  the screen and the talk, in short
//   POST /face?blink=1          blink once (token needed)
//   POST /face?view=face|status change the screen (token needed)
//   GET  /talk, POST /talk      see talk.c
//
// Tap the mouth on the screen: the status screen (tr_screen), one section per
// page: battery, robot, network (with the update token), touch and motion,
// and the room. PREV and NEXT change the section, X closes it.
//
// The audio runs all the time, so the robot does not sleep (light sleep with
// I2S running is not tested yet).
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "talk.h"
#include "tr_board.h"
#include "tr_ota.h"
#include "tr_screen.h"
#include "tr_web.h"

#define FRAME_MS   66 // about 15 frames per second
#define BG         TR_LCD_RGB(0, 0, 0)
#define FG         TR_LCD_RGB(255, 255, 255)
// The face's color says where the talk is: white off, yellow while it
// connects (3 to 4 s), blue once the robot listens.
#define CONNECTING TR_LCD_RGB(255, 200, 0)
#define LISTENING  TR_LCD_RGB(80, 200, 255)
#define BRIGHTNESS 20 // screen backlight, in %
#define TOUCH_GAP_US (3 * 1000 * 1000)

typedef enum { VIEW_FACE, VIEW_STATUS, VIEW_COUNT } view_t;
static const char *view_names[] = {"face", "status"};
static volatile view_t want_view = VIEW_FACE; // set by POST /face?view=..
static volatile bool view_request;
static volatile bool blink_now;

typedef struct {
    bool closed;
    uint16_t color;
    int mouth; // extra mouth height in pixels, while the robot speaks
} face_t;

// The color of one pixel of the face: two eyes and a mouth on black.
static uint16_t face_pixel(int x, int y, void *ctx)
{
    const face_t *f = ctx;
    for (int i = 0; i < 2; i++) {
        const int cx = i ? 225 : 95, cy = 95;
        if (f->closed) {
            if (abs(x - cx) <= 18 && abs(y - cy) <= 3) {
                return f->color;
            }
        } else if ((x - cx) * (x - cx) + (y - cy) * (y - cy) <= 16 * 16) {
            return f->color;
        }
    }
    if (x >= 120 && x < 200 && y >= 160 - f->mouth / 2 && y < 165 + f->mouth / 2) {
        return f->color;
    }
    return BG;
}

// The mouth, with some room around it for a finger.
static bool on_mouth(int x, int y)
{
    return x >= 90 && x < 230 && y >= 135 && y < 200;
}

static void face_task(void *arg)
{
    const bool screen = tr_lcd_init() == ESP_OK;
    if (screen) {
        tr_lcd_brightness(BRIGHTNESS);
    }
    int64_t next_blink = esp_timer_get_time() + 2000000, blink_end = 0, next_update = 0;
    face_t drawn = {.mouth = -1};
    view_t open = VIEW_FACE;
    bool was_down = false;
    int tap_x = 0, tap_y = 0;
    float mouth = 0;
    int64_t touch_ok_at = 0;
    TickType_t wake = xTaskGetTickCount();

    for (;;) {
        const int64_t now = esp_timer_get_time();

        // A touch on the head starts or ends a talk. One stroke gives many
        // touches (12 in a second), so after each one the next TOUCH_GAP_US
        // do nothing.
        if (tr_head_touch() && now >= touch_ok_at) {
            touch_ok_at = now + TOUCH_GAP_US;
            if (talk_live()) {
                ESP_LOGI("face", "head touch: end the talk");
                talk_close();
            } else if (talk_active()) {
                ESP_LOGI("face", "head touch: still connecting, the talk stays");
            } else {
                ESP_LOGI("face", "head touch: start a talk");
                talk_open();
            }
        }

        // A tap is a touch that ends: act on where the finger was last.
        int x, y;
        const bool down = screen && tr_touch_read(&x, &y);
        if (down) {
            tap_x = x;
            tap_y = y;
        }
        const bool tap = was_down && !down;
        was_down = down;
        if (view_request) {
            view_request = false;
            open = want_view;
            drawn.mouth = -1;
            if (open == VIEW_STATUS) {
                tr_status_open();
                next_update = now + 1000000;
            }
        }
        if (tap && open == VIEW_FACE && on_mouth(tap_x, tap_y)) {
            open = VIEW_STATUS;
            tr_status_open();
            next_update = now + 1000000;
        } else if (tap && open == VIEW_STATUS) {
            if (tr_screen_on_bar(tap_y)) {
                open = VIEW_FACE;
                drawn.mouth = -1; // draw the face again
            } else {
                tr_status_tap(tap_x, tap_y); // PREV or NEXT
                next_update = now + 1000000;
            }
        }
        if (open == VIEW_STATUS) {
            if (now >= next_update) { // new numbers every second
                tr_status_update();
                next_update = now + 1000000;
            }
            vTaskDelayUntil(&wake, pdMS_TO_TICKS(FRAME_MS));
            continue;
        }

        if (blink_now || now >= next_blink) {
            blink_now = false;
            blink_end = now + 150000;
            next_blink = now + 2500000 + esp_random() % 3000000;
        }
        mouth += (talk_level() * 30.0f - mouth) * 0.5f;
        const face_t f = {
            .closed = now < blink_end,
            .color = talk_live() ? LISTENING : talk_active() ? CONNECTING : FG,
            .mouth = (int)lroundf(mouth) & ~1,
        };
        if (screen && (f.closed != drawn.closed || f.color != drawn.color || f.mouth != drawn.mouth)) {
            tr_lcd_render(face_pixel, (void *)&f);
            drawn = f;
        }
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(FRAME_MS));
    }
}

static esp_err_t face_get(httpd_req_t *req)
{
    static char text[128];
    snprintf(text, sizeof(text), "views: face status\nlast view asked: %s\ntalk: %s\n",
             view_names[want_view], talk_active() ? "on" : "off");
    return httpd_resp_sendstr(req, text);
}

static esp_err_t face_post(httpd_req_t *req)
{
    if (!tr_ota_token_ok(req)) {
        return ESP_OK;
    }
    char query[64], value[8];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        if (httpd_query_key_value(query, "blink", value, sizeof(value)) == ESP_OK) {
            blink_now = true;
        }
        if (httpd_query_key_value(query, "view", value, sizeof(value)) == ESP_OK) {
            for (int v = 0; v < VIEW_COUNT; v++) {
                if (strcmp(value, view_names[v]) == 0) {
                    want_view = v;
                    view_request = true;
                }
            }
        }
    }
    return face_get(req);
}

void app_main(void)
{
    ESP_ERROR_CHECK(tr_ota_add_status(tr_board_status));
    ESP_ERROR_CHECK(tr_ota_start());
    ESP_ERROR_CHECK(tr_web_start());
    const httpd_uri_t get = {.uri = "/face", .method = HTTP_GET, .handler = face_get};
    const httpd_uri_t post = {.uri = "/face", .method = HTTP_POST, .handler = face_post};
    const httpd_uri_t talk_g = {.uri = "/talk", .method = HTTP_GET, .handler = talk_get};
    const httpd_uri_t talk_p = {.uri = "/talk", .method = HTTP_POST, .handler = talk_post};
    ESP_ERROR_CHECK(tr_ota_add_uri(&get));
    ESP_ERROR_CHECK(tr_ota_add_uri(&post));
    ESP_ERROR_CHECK(tr_ota_add_uri(&talk_g));
    ESP_ERROR_CHECK(tr_ota_add_uri(&talk_p));
    if (talk_start() != ESP_OK) {
        ESP_LOGW("talk", "no talk (no audio, as in the emulator)");
    } else {
        tr_sleep_set_after(0);
    }
    xTaskCreate(face_task, "face", 6144, NULL, 5, NULL);
}
