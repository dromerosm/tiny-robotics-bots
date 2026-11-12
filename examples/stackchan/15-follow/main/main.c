// 15-follow: 14-record, plus a head that follows your face (track.c, from
// 11-track) while nobody talks to the robot.
//
// The face finder and the echo canceller need the same core, a talk needs
// the internal RAM the camera uses, and motors make noise in the
// microphones. So they take turns:
// - with no talk and no recording, the camera and the face model run, the
//   head follows the biggest face, and it looks around when there is none.
//   The eyes and mouth lead the head. The echo canceller rests;
// - when a talk or a recording starts, the finder stops, the camera and the
//   model give their memory back, and the servos lose their power. Then the
//   echo canceller works (talk.c). The head stays where it was: on you;
// - when it ends, the echo canceller rests and following starts again.
//
// The rest is 14-record. A touch on the head starts a talk with GPT-Live,
// and another one ends it. So does POST /talk?start=1 or stop=1. A talk ends
// by itself after 5 minutes. Tap the right eye to record the room (rec.c),
// and tap it again to stop. No talk while it records.
//
// The face's color: white off, yellow while a talk connects, blue while the
// robot listens, red while it records, orange if the uploads fall behind,
// pale red while the last pieces go up.
//
//   GET  /face                  the screen, the talk, the recording and the
//                               finder, in short. The page at /app makes
//                               its Face buttons from these lines
//   POST /face?blink=1          blink once (token needed)
//   POST /face?follow=0|1       stop or start following faces (token needed)
//   POST /face?view=face|status|track|radar   change the screen (token needed)
//   GET  /talk, POST /talk      see talk.c
//   GET  /rec, POST /rec        see rec.c
//
// Tap the left eye: the camera on the screen, 320 x 240, with a box on each
// face the finder sees, green for the one the head follows, and the center
// of the frame. It shows that following works. A tap closes it, and so does
// a talk or a recording, because the camera stops then.
//
// Tap above the eyes: the radar (radar.c). The head turns from side to side
// and the screen shows the distance the sensor on port A finds at each angle.
// A tap closes it, and so does a talk or a recording.
//
// Tap the mouth on the screen: the status screen (tr_screen), one section per
// page: battery, robot, network (with the update token), touch and motion,
// and the room. PREV and NEXT change the section, X closes it.
//
// The robot does not sleep by itself (tr_sleep_set_after(0)), but "Sleep
// now" on the page, or POST /sleep?now=1, works: the finder, the camera, the
// servos, the screen, the speaker and the microphones stop, and the CPU uses
// light sleep. A touch wakes it. A talk or a recording wakes it too.
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/task.h"
#include "radar.h"
#include "rec.h"
#include "talk.h"
#include "track.h"
#include "tr_board.h"
#include "tr_ota.h"
#include "tr_screen.h"
#include "tr_web.h"

#define FRAME_MS 66 // about 15 frames per second
#define BG       TR_LCD_RGB(0, 0, 0)
#define FG       TR_LCD_RGB(255, 255, 255)
// The face's color says where the talk is: white off, yellow while it
// connects (3 to 4 s), blue once the robot listens.
#define CONNECTING   TR_LCD_RGB(255, 200, 0)
#define LISTENING    TR_LCD_RGB(80, 200, 255)
#define RECORDING    TR_LCD_RGB(255, 40, 40)   // red while it records
#define BEHIND       TR_LCD_RGB(255, 120, 0)   // orange: it records, but the uploads fall behind
#define UPLOADING    TR_LCD_RGB(255, 140, 140) // pale red: stopped, still uploading
#define BRIGHTNESS   20                        // screen backlight, in %
#define TOUCH_GAP_US (3 * 1000 * 1000)
#define EYE_FOLLOW_X 40.0f // pixels the eyes and mouth move with the face at the frame's edge
#define EYE_FOLLOW_Y 25.0f

typedef enum { VIEW_FACE, VIEW_STATUS, VIEW_TRACK, VIEW_RADAR, VIEW_COUNT } view_t;
static const char *view_names[] = {"face", "status", "track", "radar"};
static volatile view_t want_view = VIEW_FACE; // set by POST /face?view=..
static volatile bool view_request;
static volatile view_t shown = VIEW_FACE; // the view on the screen now, for GET /face
static volatile bool blink_now;
static volatile bool follow = true; // POST /face?follow=0 stops the finder

typedef struct {
    int dx, dy; // the eyes and mouth look at the face the head follows
    bool closed;
    uint16_t color;
    int mouth; // extra mouth height in pixels, while the robot speaks
} face_t;

// The color of one pixel of the face: two eyes and a mouth on black.
static uint16_t face_pixel(int x, int y, void *ctx)
{
    const face_t *f = ctx;
    for (int i = 0; i < 2; i++) {
        const int cx = (i ? 225 : 95) + f->dx, cy = 95 + f->dy;
        if (f->closed) {
            if (abs(x - cx) <= 18 && abs(y - cy) <= 3) {
                return f->color;
            }
        } else if ((x - cx) * (x - cx) + (y - cy) * (y - cy) <= 16 * 16) {
            return f->color;
        }
    }
    if (x >= 120 + f->dx && x < 200 + f->dx && y >= 160 + f->dy - f->mouth / 2 &&
        y < 165 + f->dy + f->mouth / 2) {
        return f->color;
    }
    return BG;
}

// The left eye, with room for a finger.
static bool on_left_eye(int x, int y)
{
    return x >= 40 && x < 160 && y >= 45 && y < 135;
}

// The right eye (on the screen's right), with room for a finger.
static bool on_right_eye(int x, int y)
{
    return x >= 185 && x < 265 && y >= 55 && y < 135;
}

// Above the eyes.
static bool on_forehead(int y)
{
    return y < 40;
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
    float mouth = 0, eye_x = 0, eye_y = 0;
    int64_t touch_ok_at = 0;
    TickType_t wake = xTaskGetTickCount();

    for (;;) {
        shown = open;
        // Asleep: nothing to draw and nobody to follow. Wait, then draw the
        // face again. The touch that wakes the robot does nothing else.
        if (tr_sleep_asleep()) {
            track_show(false); // the camera view closes too
            tr_sleep_wait_awake();
            open = VIEW_FACE;
            was_down = false;
            drawn.mouth = -1;
            touch_ok_at = esp_timer_get_time() + 500000;
            wake = xTaskGetTickCount();
            continue;
        }
        const int64_t now = esp_timer_get_time();

        // A touch on the head starts or ends a talk. One stroke gives many
        // touches (12 in a second), so after each one the next TOUCH_GAP_US
        // do nothing.
        if (tr_head_touch() && now >= touch_ok_at) {
            touch_ok_at = now + TOUCH_GAP_US;
            if (rec_active()) {
                ESP_LOGI("face", "head touch: recording, no talk");
            } else if (talk_live()) {
                ESP_LOGI("face", "head touch: end the talk");
                talk_close();
            } else if (talk_active()) {
                ESP_LOGI("face", "head touch: still connecting, the talk stays");
            } else {
                ESP_LOGI("face", "head touch: start a talk");
                talk_open();
            }
        }

        // The head follows a face, but not during a talk or a recording.
        // The camera runs while the head follows, or the camera view is
        // open. When neither wants it any more, it stops and gives its
        // memory back.
        const bool quiet = talk_quiet() || rec_active();
        // The radar moves the head itself: no camera and no finder then.
        const bool camera = (follow && open != VIEW_RADAR) || open == VIEW_TRACK;
        static bool camera_was;
        if (camera_was && !camera && !quiet) {
            track_quiet();
        }
        camera_was = camera;
        track_step(now, quiet || !camera, follow);

        // A tap is a touch that ends: act on where the finger was last.
        int x, y;
        const bool down = screen && tr_touch_read(&x, &y);
        if (down) {
            tap_x = x;
            tap_y = y;
        }
        const bool tap = was_down && !down;
        was_down = down;
        // The camera view: every frame the finder took, with a box on each
        // face. A tap closes it. So does a talk or a recording, because the
        // camera stops then.
        if (open == VIEW_TRACK && !view_request) {
            if (tap || quiet) {
                ESP_LOGI("face", "camera view closed");
                track_show(false);
                open = VIEW_FACE;
                drawn.mouth = -1; // draw the face again
            } else {
                track_frame();
                vTaskDelay(1); // one tick for the tasks below ours
            }
            wake = xTaskGetTickCount();
            continue;
        }
        if (view_request) {
            view_request = false;
            track_show(false);
            if (open == VIEW_RADAR && want_view != VIEW_RADAR) {
                radar_close();
            }
            const bool new_radar = want_view == VIEW_RADAR && open != VIEW_RADAR;
            open = want_view;
            drawn.mouth = -1;
            if (open == VIEW_STATUS) {
                tr_status_open();
                next_update = now + 1000000;
            }
            if (new_radar) {
                radar_open();
                continue; // the camera stops first, at the top of the loop
            }
        }
        if (open == VIEW_FACE && tap && on_forehead(tap_y) && !quiet) {
            ESP_LOGI("face", "radar open");
            open = VIEW_RADAR;
            radar_open();
            continue;
        }
        // The radar: a tap closes it. So does a talk or a recording, because
        // the servos stop then.
        if (open == VIEW_RADAR) {
            if (tap || quiet) {
                ESP_LOGI("face", "radar closed");
                radar_close();
                open = VIEW_FACE;
                drawn.mouth = -1; // draw the face again
            } else {
                radar_step(now);
            }
            vTaskDelayUntil(&wake, pdMS_TO_TICKS(FRAME_MS));
            continue;
        }
        if (open == VIEW_FACE && tap && on_left_eye(tap_x, tap_y) && !quiet) {
            open = VIEW_TRACK;
        }
        if (open == VIEW_TRACK) { // from the tap, or from the page
            if (quiet) {
                open = VIEW_FACE; // no camera now
            } else {
                ESP_LOGI("face", "camera view open");
                track_show(true);
            }
            continue;
        }
        if (tap && open == VIEW_FACE && on_right_eye(tap_x, tap_y)) {
            if (rec_listening()) {
                ESP_LOGI("face", "right eye: stop recording");
                rec_stop();
            } else if (!talk_active()) {
                ESP_LOGI("face", "right eye: record");
                rec_start();
            }
        } else if (tap && open == VIEW_FACE && on_mouth(tap_x, tap_y)) {
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
        // The eyes and mouth look at the face, and come back to the center
        // as the head catches up. Halfway each frame: smooth, but quick.
        float to_x, to_y;
        track_eyes(&to_x, &to_y);
        eye_x += (to_x * EYE_FOLLOW_X - eye_x) * 0.5f;
        eye_y += (to_y * EYE_FOLLOW_Y - eye_y) * 0.5f;
        const face_t f = {
            .dx = (int)lroundf(eye_x),
            .dy = (int)lroundf(eye_y),
            .closed = now < blink_end,
            .color = rec_behind()      ? BEHIND
                     : rec_listening() ? RECORDING
                     : rec_active()    ? UPLOADING
                     : talk_live()     ? LISTENING
                     : talk_active()   ? CONNECTING
                                       : FG,
            .mouth = (int)lroundf(mouth) & ~1,
        };
        if (screen && (f.dx != drawn.dx || f.dy != drawn.dy || f.closed != drawn.closed ||
                       f.color != drawn.color || f.mouth != drawn.mouth)) {
            tr_lcd_render(face_pixel, (void *)&f);
            drawn = f;
        }
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(FRAME_MS));
    }
}

// Before sleep, in the sleep task: the finder stops the camera, so the sleep
// step can cut its power, and the servos stay off. track_step() starts it
// all again after the robot wakes.
static void on_sleep(bool going_to_sleep)
{
    if (going_to_sleep) {
        track_quiet();
    }
}

// The distance sensor on port A needs the 5 V of the Grove ports. Asleep, it
// goes off to save the battery.
static void port_sleep(bool going_to_sleep)
{
    if (tr_port_power(!going_to_sleep) != ESP_OK) {
        ESP_LOGW("port", "no 5 V for the Grove ports");
    }
}

static esp_err_t face_get(httpd_req_t *req)
{
    // The page (tr_web) shows a pair of buttons for each of the lines
    // "talk:", "recording:" and "following:", and fills the button of the
    // mode the robot is in. "view:" is the screen now, also after a tap.
    static char text[320];
    const int n = snprintf(text, sizeof(text),
                           "views: face status track radar\nview: %s\ntalk: %s\nrecording: %s\n"
                           "following: %s\n",
                           view_names[shown],
                           talk_live()     ? "on, the robot listens"
                           : talk_active() ? "connecting"
                                           : "off",
                           rec_behind()      ? "on, the uploads are behind"
                           : rec_listening() ? "on"
                           : rec_active()    ? "stopped, the last pieces go up"
                                             : "off",
                           follow ? "on" : "off");
    track_status(text + n, sizeof(text) - n);
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
        if (httpd_query_key_value(query, "follow", value, sizeof(value)) == ESP_OK) {
            follow = value[0] != '0'; // the face task stops the camera if nobody needs it
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
    const httpd_uri_t rec_g = {.uri = "/rec", .method = HTTP_GET, .handler = rec_get};
    const httpd_uri_t rec_p = {.uri = "/rec", .method = HTTP_POST, .handler = rec_post};
    ESP_ERROR_CHECK(tr_ota_add_uri(&rec_g));
    ESP_ERROR_CHECK(tr_ota_add_uri(&rec_p));
    port_sleep(false);
    tr_sleep_add_hook(port_sleep);
    if (track_init() == ESP_OK) {
        talk_set_quiet_hook(track_quiet);
        tr_sleep_add_hook(on_sleep);
    } else {
        ESP_LOGW("track", "no memory for the finder task: the head does not follow");
    }
    if (talk_start() != ESP_OK) {
        ESP_LOGW("talk", "no talk (no audio, as in the emulator)");
    } else {
        tr_sleep_set_after(0);
    }
    // The face task's stack in PSRAM too (see track.c).
    xTaskCreatePinnedToCoreWithCaps(face_task, "face", 6144, NULL, 5, NULL, tskNO_AFFINITY,
                                    MALLOC_CAP_SPIRAM);
}
