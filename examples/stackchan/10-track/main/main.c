// 10-track: 09-wifi plus a camera that finds faces. Step one of face
// tracking: it finds faces and shows them, and the head does not move yet.
//
// Tap the left eye: the camera on the screen, 320 x 240, with a green box on
// each face it finds (Espressif's esp-dl, human_face_detect, the MSR + MNP
// model, in face.cpp). A tap anywhere, or a touch on the head, goes back to
// the face. Every 2 s /log gets a line with the faces, the time to find them
// and the frames per second, and GET /face has the same as "track:".
//
// Everything else is as in 09-wifi:
//
// The eyes follow where the head really points: the example reads both servos
// a few times a second. At boot the head goes home and stays still. With
// "look around" on (POST /face?idle=1), it glances somewhere new every few
// seconds, slowly and in a small range. A move from the page or curl pauses
// that for IDLE_PAUSE_US.
//
//   GET  /face                  idle on or off, where the head looks, last scan, faces
//   POST /face?follow=0|1       stop or start following faces (token needed)
//   GET  /track.csv             the record of finds and head moves (events.c)
//   POST /face?idle=0|1         stop or start looking around (token needed)
//   POST /face?blink=1          blink once (token needed)
//   POST /face?view=face|status|wifi|track   change the screen (token needed)
//
// Tap the mouth on the screen: the status screen (tr_screen), one section per
// page: battery (with a guess of the time left and the sleep state), robot,
// network (with the update token), touch and motion, and the room. PREV and
// NEXT change the section, X closes it. The token is shown only here, on the
// robot itself, never on the network.
//
// After a minute with no touch, on battery, the robot sleeps (tr_board): the
// screen and the servos go off and this task waits. A touch on the screen or
// the head wakes it. An open view keeps it awake.
//
// Tap the right eye: the Wi-Fi networks the robot sees, strongest first, with
// their signal and security. Red is open (or WEP), yellow old WPA, green WPA3.
// The list scans again every SCAN_EVERY_US. UP and DOWN scroll, X closes it.
// GET /face lists the last scan too.
//
// The page at http://stackchan.local/app (tr_web) shows a card for this.
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_heap_caps.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "events.h"
#include "face.h"
#include "tr_board.h"
#include "tr_ota.h"
#include "tr_screen.h"
#include "tr_web.h"

#define FRAME_MS      66                 // about 15 frames per second
#define IDLE_PAUSE_US (20 * 1000 * 1000) // after a move from the page
#define GLANCE_MS     900                // a slow move
#define YAW_RANGE     40                 // idle glances stay within +-40 degrees
#define PITCH_LOW     5                  // and between 5 and 35 degrees of pitch
#define PITCH_HIGH    35
#define BG            TR_LCD_RGB(0, 0, 0)
#define FG            TR_LCD_RGB(255, 255, 255)
// How far the eyes move on the screen per degree of yaw. The sign makes the
// eyes look the way the head turns; check it on the robot, and flip it if the
// eyes look the other way.
#define EYE_X_PER_DEG (-0.5f)
#define BRIGHTNESS    20 // screen backlight, in %
// Following a face. The error is how far the face is from the center of the
// camera frame, from -1 (one edge) to 1 (the other edge).
#define FOLLOW_DEAD  0.15f // no move while the face is this close to the center
#define FOLLOW_YAW   18.0f // degrees of yaw from the center to the edge of the frame (a guess)
#define FOLLOW_PITCH 12.0f // the same for pitch (a guess)
#define MS_PER_DEG   25    // move time per degree: about 40 degrees per second
#define MOVE_MIN_MS  150
#define MOVE_MAX_MS  600
#define YAW_SIGN     1.0f    // flip if the head turns away from the face
#define PITCH_SIGN   (-1.0f) // flip if the head looks away up or down
#define EYE_FOLLOW_X 40.0f   // pixels the eyes and mouth move at an error of 1
#define EYE_FOLLOW_Y 25.0f
#define EYE_SIGN_X   1.0f                     // flip if the eyes look away from the face
#define LOST_US      (1000 * 1000)            // no face for this long: the face is lost
#define RELEASE_US   (2000 * 1000)            // torque off after this long with no move
#define CENTER_FG    TR_LCD_RGB(90, 160, 255) // the center and dead zone in the camera view
// Searching. The camera sees about 47 x 37 degrees (57.6 degrees across,
// from M5Stack's GC0308 camera specs, and the GC0308's 3.4 um pixels), a
// quarter of the head's yaw range. So with no face, the head looks around:
// each look goes far enough to see a new part of the room.
#define SEARCH_AFTER_US   (5000 * 1000) // no face for this long: start searching
#define SEARCH_YAW        70            // looks stay within +-70 degrees of yaw
#define SEARCH_PITCH_LOW  5             // and 5 to 40 degrees of pitch
#define SEARCH_PITCH_HIGH 40
#define SEARCH_MIN_YAW    33            // a new look turns at least this far (0.7 x 47)
#define SEARCH_MIN_PITCH  26            // or tilts at least this far (0.7 x 37)
#define SEARCH_MAX_STEP   60            // but no look turns more than this (calm, not jumpy)
#define SEARCH_MS_PER_DEG 50            // looks turn at about 20 degrees per second
#define SEARCH_STAY_US    (3000 * 1000) // then it stays still this long (about 30 finds)
// Below this, on battery, the head does not move. Servo moves on a nearly
// empty battery made the power chip cut the power: the robot restarted
// again and again (at 28 % charge, after following and searching).
#define MOVE_MIN_MV   3600
#define SCAN_EVERY_US (10 * 1000 * 1000) // while the Wi-Fi list is open
#define MAX_APS       32

typedef enum { VIEW_FACE, VIEW_STATUS, VIEW_WIFI, VIEW_TRACK, VIEW_COUNT } view_t;
static const char *view_names[] = {"face", "status", "wifi", "track"};
static volatile view_t want_view = VIEW_FACE; // set by POST /face?view=..
static volatile bool view_request;

// Off at boot: the head goes home and stays still. The page can turn it on.
static volatile bool idle = false;
// On at boot: the head follows a face (POST /face?follow=0 stops it).
static volatile bool follow = true;
static volatile bool blink_now;
static volatile float head_yaw, head_pitch; // read from the servos

typedef struct {
    int dx, dy;
    bool closed;
} face_t;

// The color of one pixel of the face: two eyes and a mouth on black.
static uint16_t face_pixel(int x, int y, void *ctx)
{
    const face_t *f = ctx;
    for (int i = 0; i < 2; i++) {
        const int cx = (i ? 225 : 95) + f->dx, cy = 95 + f->dy;
        if (f->closed) {
            if (abs(x - cx) <= 18 && abs(y - cy) <= 3) {
                return FG;
            }
        } else if ((x - cx) * (x - cx) + (y - cy) * (y - cy) <= 16 * 16) {
            return FG;
        }
    }
    if (x >= 120 + f->dx && x < 200 + f->dx && y >= 160 + f->dy && y < 165 + f->dy) {
        return FG;
    }
    return BG;
}

// Read where the head points, when the servos have power.
static void read_head(void)
{
    tr_servo_state_t s;
    if (tr_servo_read(TR_SERVO_YAW, &s) == ESP_OK) {
        head_yaw = s.degrees;
    }
    if (tr_servo_read(TR_SERVO_PITCH, &s) == ESP_OK) {
        head_pitch = s.degrees;
    }
}

// Home: straight ahead, pitch at its low safe end. One slow move, then the
// face task takes the torque off.
#define HOME_YAW   0.0f
#define HOME_PITCH 5.0f
#define HOME_MS    1000

static void go_home(void)
{
    ESP_LOGI("face", "head: home (boot)");
    events_note('H');
    if (tr_web_head_power() == ESP_OK) {
        tr_servo_move(TR_SERVO_YAW, HOME_YAW, HOME_MS);
        tr_servo_move(TR_SERVO_PITCH, HOME_PITCH, HOME_MS);
    }
}

static void glance(void)
{
    const float yaw = (float)((int)(esp_random() % (2 * YAW_RANGE + 1)) - YAW_RANGE);
    const float pitch = (float)(PITCH_LOW + esp_random() % (PITCH_HIGH - PITCH_LOW + 1));
    if (tr_web_head_power() == ESP_OK) {
        tr_servo_move(TR_SERVO_YAW, yaw, GLANCE_MS);
        tr_servo_move(TR_SERVO_PITCH, pitch, GLANCE_MS);
    }
}

#define ROWS   TR_SCREEN_ROWS
#define GREEN  TR_SCREEN_GREEN
#define YELLOW TR_SCREEN_YELLOW
#define RED    TR_SCREEN_RED

// Wi-Fi scan. It runs in the background (about 2 s, all channels) while the
// robot stays on its network; the driver tells us when it is done.
static wifi_ap_record_t aps[MAX_APS];
static uint16_t ap_count;
static volatile bool scanning, scan_done;
static int64_t scanned_at = -1;

static void on_scan_done(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (scanning) { // not tr_ota's own scan
        scanning = false;
        scan_done = true;
    }
}

static void start_scan(void)
{
    scanning = esp_wifi_scan_start(NULL, false) == ESP_OK;
}

static const char *security(wifi_auth_mode_t m)
{
    switch (m) {
        case WIFI_AUTH_OPEN:
            return "open";
        case WIFI_AUTH_WEP:
            return "WEP";
        case WIFI_AUTH_WPA_PSK:
            return "WPA";
        case WIFI_AUTH_WPA_WPA2_PSK:
            return "WPA/2";
        case WIFI_AUTH_WPA2_PSK:
            return "WPA2";
        case WIFI_AUTH_WPA2_ENTERPRISE:
            return "WPA2-E";
        case WIFI_AUTH_WPA2_WPA3_PSK:
            return "WPA2/3";
        case WIFI_AUTH_WPA3_PSK:
            return "WPA3";
        case WIFI_AUTH_WPA3_ENT_192:
            return "WPA3-E";
        case WIFI_AUTH_OWE:
            return "OWE";
        case WIFI_AUTH_WAPI_PSK:
            return "WAPI";
        default:
            return "other";
    }
}

// Red for open networks (and WEP, which is nearly open), yellow for old WPA,
// green for WPA3, white for WPA2 and the rest.
static uint16_t security_color(wifi_auth_mode_t m)
{
    if (m == WIFI_AUTH_OPEN || m == WIFI_AUTH_WEP) {
        return RED;
    }
    if (m == WIFI_AUTH_WPA_PSK || m == WIFI_AUTH_WPA_WPA2_PSK) {
        return YELLOW;
    }
    if (m == WIFI_AUTH_WPA3_PSK || m == WIFI_AUTH_WPA2_WPA3_PSK || m == WIFI_AUTH_WPA3_ENT_192 ||
        m == WIFI_AUTH_OWE) {
        return GREEN;
    }
    return FG;
}

static int stronger_first(const void *a, const void *b)
{
    return ((const wifi_ap_record_t *)b)->rssi - ((const wifi_ap_record_t *)a)->rssi;
}

// Take the scan results, strongest first.
static void take_scan(void)
{
    ap_count = MAX_APS;
    if (esp_wifi_scan_get_ap_records(&ap_count, aps) != ESP_OK) {
        ap_count = 0;
    }
    qsort(aps, ap_count, sizeof(aps[0]), stronger_first);
    scanned_at = esp_timer_get_time();
}

// One row per network: name (13 characters), signal in dBm, security.
static void build_wifi(void)
{
    tr_screen_clear();
    if (scanned_at < 0) {
        tr_screen_row(FG, "scanning...");
        return;
    }
    tr_screen_row(FG, "%u networks, dBm", ap_count);
    for (int i = 0; i < ap_count; i++) {
        const char *name = aps[i].ssid[0] ? (const char *)aps[i].ssid : "(hidden)";
        tr_screen_row(security_color(aps[i].authmode), "%-13.13s %4d %s", name, aps[i].rssi,
                      security(aps[i].authmode));
    }
}

// The left eye, with some room around it.
static bool on_left_eye(int x, int y)
{
    return x >= 40 && x < 160 && y >= 45 && y < 135;
}

// The right eye, with some room around it.
static bool on_right_eye(int x, int y)
{
    return x >= 160 && x < 280 && y >= 45 && y < 135;
}

// The mouth, with some room around it for a finger.
static bool on_mouth(int x, int y)
{
    return x >= 90 && x < 230 && y >= 135 && y < 200;
}

// A red box with an X at the top right of a camera frame, so it is clear how
// to leave. The frame is RGB565, high byte first. (As in 08-face.)
static void draw_exit_mark(camera_fb_t *fb)
{
    const uint16_t red = TR_LCD_RGB(200, 40, 40), white = TR_LCD_WHITE;
    for (int y = 0; y < 32 && y < (int)fb->height; y++) {
        for (int x = fb->width - 40; x < (int)fb->width; x++) {
            const int dx = x - (fb->width - 40), d1 = dx - y - 4, d2 = dx + y - 36;
            const bool cross =
                dx > 8 && dx < 32 && y > 4 && y < 28 && (abs(d1) <= 1 || abs(d2) <= 1);
            const uint16_t c = cross ? white : red;
            uint8_t *p = fb->buf + ((size_t)y * fb->width + x) * 2;
            p[0] = c >> 8;
            p[1] = c & 0xFF;
        }
    }
}

// Faces. Two tasks run at the same time:
// - the finder, on the second core (core 1), takes each camera frame, finds
//   the faces and keeps their boxes. Finding is the point (the head will
//   follow the face), so it takes the frames one after the other;
// - when the face task is free, the finder hands it the frame and its boxes,
//   and the face task shows them (for checking) and gives the frame back to
//   the camera. When it is busy, the finder gives the frame back itself.
// So the screen shows exactly what was found, and never slows the finder.
static bool track_ready;
static volatile bool track_on;   // the finder works (follow is on, or the view is open)
static volatile bool track_show; // the camera view is open: frames go to the screen
static TaskHandle_t finder;
static QueueHandle_t to_show; // one frame for the screen, with its boxes

typedef struct {
    camera_fb_t *fb;
    int n;
    face_box_t boxes[FACE_MAX];
} found_frame_t;

// For GET /face and /log, over windows of 2 s. fps values are x 10.
static volatile int track_faces, track_ms, track_fps10, show_fps10, show_ms;
static volatile face_box_t track_best;

// The last find, for following: the center of the biggest face and the frame
// size. seq goes up with each find.
typedef struct {
    uint32_t seq;
    int n, cx, cy, w, h;
} target_t;
static portMUX_TYPE target_lock = portMUX_INITIALIZER_UNLOCKED;
static target_t target;

// A box of 2-pixel lines on an RGB565 frame (high byte first).
static void draw_box(camera_fb_t *fb, const face_box_t *b, uint16_t color)
{
    const int w = fb->width, h = fb->height;
    for (int y = b->y0; y <= b->y1; y++) {
        for (int x = b->x0; x <= b->x1; x++) {
            const bool edge = x - b->x0 < 2 || b->x1 - x < 2 || y - b->y0 < 2 || b->y1 - y < 2;
            if (edge && x >= 0 && x < w && y >= 0 && y < h) {
                uint8_t *p = fb->buf + ((size_t)y * w + x) * 2;
                p[0] = color >> 8;
                p[1] = color & 0xFF;
            }
        }
    }
}

// The finder: core 1, only while the view is open.
static void finder_task(void *arg)
{
    int finds = 0, ms_sum = 0;
    int64_t window = esp_timer_get_time();
    for (;;) {
        if (!track_on) {
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // wait for the view
            finds = ms_sum = 0;
            window = esp_timer_get_time();
            continue;
        }
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            continue; // the camera timed out; try again
        }
        found_frame_t f = {.fb = fb};
        const int64_t t0 = esp_timer_get_time();
        f.n = face_find(fb->buf, fb->width, fb->height, f.boxes, FACE_MAX);
        const int ms = (esp_timer_get_time() - t0) / 1000;
        track_faces = f.n;
        if (f.n > 0) {
            track_best = f.boxes[0];
        }
        // The record: every find with a face, and the first one without.
        static int last_n;
        if (f.n > 0) {
            const face_box_t *b = &f.boxes[0];
            const int cx = (b->x0 + b->x1) / 2, cy = (b->y0 + b->y1) / 2;
            events_find(f.n, cx, cy, b->x1 - b->x0, b->score,
                        (cx - fb->width / 2.0f) / (fb->width / 2.0f),
                        (cy - fb->height / 2.0f) / (fb->height / 2.0f));
        } else if (last_n > 0) {
            events_find(0, 0, 0, 0, 0, 0, 0);
        }
        last_n = f.n;
        taskENTER_CRITICAL(&target_lock);
        target.seq++;
        target.n = f.n;
        target.w = fb->width;
        target.h = fb->height;
        if (f.n > 0) {
            target.cx = (f.boxes[0].x0 + f.boxes[0].x1) / 2;
            target.cy = (f.boxes[0].y0 + f.boxes[0].y1) / 2;
        }
        taskEXIT_CRITICAL(&target_lock);
        // To the screen if the view is open and free; if not, back to the camera.
        if (!track_show || xQueueSend(to_show, &f, 0) != pdTRUE) {
            esp_camera_fb_return(fb);
        }
        finds++;
        ms_sum += ms;
        const int64_t now = esp_timer_get_time();
        if (now - window >= 10000000) { // every 10 s: /log holds only 4 KB
            track_ms = ms_sum / finds;
            track_fps10 = (int)(finds * 10000000LL / (now - window));
            ESP_LOGI("track", "%d face(s), find %d ms, %d.%d finds/s, show %d.%d frames/s (%d ms)",
                     f.n, track_ms, track_fps10 / 10, track_fps10 % 10, show_fps10 / 10,
                     show_fps10 % 10, show_ms);
            if (f.n > 0) {
                ESP_LOGI("track", "best: %d,%d to %d,%d, score %.2f", f.boxes[0].x0, f.boxes[0].y0,
                         f.boxes[0].x1, f.boxes[0].y1, f.boxes[0].score);
            }
            finds = ms_sum = 0;
            window = now;
        }
    }
}

// Start the camera, load the model and start the finder, once. Says in /log
// how long it took and how much memory is left.
static esp_err_t track_start(void)
{
    if (track_ready) {
        return ESP_OK;
    }
    const int64_t t0 = esp_timer_get_time();
    if (tr_cam_start() != ESP_OK) {
        ESP_LOGW("track", "no camera");
        return ESP_FAIL;
    }
    if (face_start() != ESP_OK) {
        ESP_LOGW("track", "no memory for the face model");
        return ESP_ERR_NO_MEM;
    }
    to_show = xQueueCreate(1, sizeof(found_frame_t));
    if (!to_show ||
        xTaskCreatePinnedToCore(finder_task, "finder", 6144, NULL, 4, &finder, 1) != pdPASS) {
        ESP_LOGW("track", "no memory for the finder task");
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI("track", "ready in %lld ms; free: internal %u KB (lowest %u KB), PSRAM %u KB",
             (esp_timer_get_time() - t0) / 1000,
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024,
             (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL) / 1024,
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024);
    track_ready = true;
    return ESP_OK;
}

// Start or stop the finder, and send its frames to the screen or not. A frame
// still waiting for the screen goes back to the camera when the view closes.
static void track_set(bool on, bool show)
{
    track_show = show;
    if (!show) {
        found_frame_t f;
        if (to_show && xQueueReceive(to_show, &f, 0) == pdTRUE) {
            esp_camera_fb_return(f.fb);
        }
    }
    if (on && !track_on) {
        track_on = true;
        xTaskNotifyGive(finder);
    } else if (!on) {
        track_on = false;
        track_faces = 0;
    }
}

// Show the next frame the finder handed over, with its boxes. Waits up to
// one tick for it, so the touch is still read often.
static void track_frame(void)
{
    static int shows, us_sum;
    static int64_t window;
    found_frame_t f;
    if (xQueueReceive(to_show, &f, 1) != pdTRUE) {
        return;
    }
    const int64_t t0 = esp_timer_get_time();
    for (int i = 0; i < f.n; i++) {
        draw_box(f.fb, &f.boxes[i], i == 0 ? GREEN : YELLOW); // green: the biggest, followed
    }
    // The center and the dead zone, to check that the head centers the face.
    const int cx = f.fb->width / 2, cy = f.fb->height / 2;
    const int dx = (int)(FOLLOW_DEAD * cx), dy = (int)(FOLLOW_DEAD * cy);
    const face_box_t dead = {cx - dx, cy - dy, cx + dx, cy + dy, 0};
    const face_box_t cross_h = {cx - 8, cy, cx + 8, cy + 1, 0},
                     cross_v = {cx, cy - 8, cx + 1, cy + 8, 0};
    draw_box(f.fb, &dead, CENTER_FG);
    draw_box(f.fb, &cross_h, CENTER_FG);
    draw_box(f.fb, &cross_v, CENTER_FG);
    draw_exit_mark(f.fb);
    tr_lcd_blit(0, 0, f.fb->width, f.fb->height, f.fb->buf);
    esp_camera_fb_return(f.fb);
    shows++;
    us_sum += esp_timer_get_time() - t0;
    const int64_t now = esp_timer_get_time();
    if (window == 0) {
        window = now;
    } else if (now - window >= 2000000) {
        show_fps10 = (int)(shows * 10000000LL / (now - window));
        show_ms = us_sum / 1000 / shows;
        shows = us_sum = 0;
        window = now;
    }
}

// The status or Wi-Fi screen.
static void open_list(view_t v, int64_t now, int64_t *next_update)
{
    if (v == VIEW_WIFI) {
        if (!scanning) {
            start_scan();
        }
        build_wifi();
        tr_screen_draw("Wi-Fi", "UP", "DOWN", 0);
        *next_update = now + SCAN_EVERY_US;
    } else {
        tr_status_open();
        *next_update = now + 1000000;
    }
}

// Follow the face. For each new find: the error is how far the face is
// from the center of the frame (-1 to 1). Outside the dead zone, the head
// gets a new goal: where it is now (read from the servo) plus the angle of
// the error. Finds made while the head turns give about the same goal, so
// the goals don't add up, and the head turns smoothly with no pauses. (Steps
// on top of the last commanded goal made it swing; steps with a pause after
// each one made it jump.) The eyes and mouth move toward the face by
// the same error, so they lead the head, and they come back to the center
// as the head turns and the error goes down. Returns true if the head moved.
static float err_x, err_y; // the last error, 0 when there is no face
static float follow_yaw, follow_pitch;
static bool following;         // a face was seen in the last LOST_US
static int64_t last_seen = -1; // when, or -1
static uint32_t seen_seq;

// True if the battery can take a head move: USB power, or the battery at
// MOVE_MIN_MV or more. Read at most once a second; says so in /log when it
// changes.
static bool power_for_moves(int64_t now)
{
    static int64_t next_check;
    static bool ok = true;
    if (now >= next_check) {
        next_check = now + 1000000;
        tr_power_t p;
        tr_battery_t b;
        const bool usb = tr_power_read(&p) == ESP_OK && p.usb;
        const bool now_ok = usb || tr_battery_read(&b) != ESP_OK || b.millivolts >= MOVE_MIN_MV;
        if (now_ok != ok) {
            ok = now_ok;
            if (ok) {
                ESP_LOGI("face", "battery ok again: the head moves");
            } else {
                ESP_LOGW("face", "battery below %d mV: the head stays still", MOVE_MIN_MV);
                events_note('B');
            }
        }
    }
    return ok;
}

// Move time for a turn of this many degrees: about the same speed for small
// and big turns.
static int move_ms(float degrees)
{
    const int ms = (int)(fabsf(degrees) * MS_PER_DEG);
    return ms < MOVE_MIN_MS ? MOVE_MIN_MS : ms > MOVE_MAX_MS ? MOVE_MAX_MS : ms;
}

static bool follow_step(int64_t now)
{
    if (!follow || !track_on) {
        following = false;
        err_x = err_y = 0;
        return false;
    }
    taskENTER_CRITICAL(&target_lock);
    const target_t t = target;
    taskEXIT_CRITICAL(&target_lock);
    bool moved = false;
    if (t.seq != seen_seq) {
        seen_seq = t.seq;
        if (t.n > 0 && t.w > 0 && t.h > 0) {
            if (!following) { // start from where the head is now
                follow_yaw = head_yaw;
                follow_pitch = head_pitch;
                events_note('S');
            }
            following = true;
            last_seen = now;
            tr_sleep_activity(); // someone is there: stay awake
            err_x = (t.cx - t.w / 2.0f) / (t.w / 2.0f);
            err_y = (t.cy - t.h / 2.0f) / (t.h / 2.0f);
            const int64_t last_move = tr_web_last_move_us();
            const bool paused = last_move >= 0 && now - last_move < IDLE_PAUSE_US;
            if (!paused && tr_web_servos() && power_for_moves(now) &&
                (fabsf(err_x) > FOLLOW_DEAD || fabsf(err_y) > FOLLOW_DEAD) &&
                tr_web_head_power() == ESP_OK) {
                tr_servo_state_t st;
                if (fabsf(err_x) > FOLLOW_DEAD) {
                    if (tr_servo_read(TR_SERVO_YAW, &st) == ESP_OK) {
                        follow_yaw = st.degrees;
                    }
                    const float goal =
                        fminf(fmaxf(follow_yaw + YAW_SIGN * err_x * FOLLOW_YAW, -80.0f), 80.0f);
                    tr_servo_move(TR_SERVO_YAW, goal, move_ms(goal - follow_yaw));
                    events_move('Y', follow_yaw, goal, move_ms(goal - follow_yaw));
                    follow_yaw = goal;
                }
                if (fabsf(err_y) > FOLLOW_DEAD) {
                    if (tr_servo_read(TR_SERVO_PITCH, &st) == ESP_OK) {
                        follow_pitch = st.degrees;
                    }
                    const float goal =
                        fminf(fmaxf(follow_pitch + PITCH_SIGN * err_y * FOLLOW_PITCH, TR_PITCH_MIN),
                              TR_PITCH_MAX);
                    tr_servo_move(TR_SERVO_PITCH, goal, move_ms(goal - follow_pitch));
                    events_move('P', follow_pitch, goal, move_ms(goal - follow_pitch));
                    follow_pitch = goal;
                }
                moved = true;
            }
        }
    }
    if (following && now - last_seen > LOST_US) {
        following = false; // lost: the eyes go back to the center
        events_note('L');
        err_x = err_y = 0;
    }
    return moved;
}

// Search for a face: with following on and no face for SEARCH_AFTER_US, look
// at a random spot 33 to 60 degrees away (far enough to see a new part of the
// room, near enough to stay calm), turn there slowly, stay SEARCH_STAY_US,
// then look somewhere else. (With 1.2 s stays, 40 degrees per second and any
// spot in the range, it moved too much and too fast.) A face found on the way takes over at once.
// Searching is not activity, so with nobody there the robot still sleeps.
static int64_t next_look;

static bool search_step(int64_t now, int64_t follow_since)
{
    const int64_t quiet_since = last_seen > follow_since ? last_seen : follow_since;
    const int64_t last_move = tr_web_last_move_us();
    const bool paused = last_move >= 0 && now - last_move < IDLE_PAUSE_US;
    if (!follow || !track_on || following || paused || !tr_web_servos() ||
        now - quiet_since < SEARCH_AFTER_US || now < next_look || !power_for_moves(now)) {
        return false;
    }
    if (tr_web_head_power() != ESP_OK) {
        return false;
    }
    tr_servo_state_t st;
    float yaw = head_yaw, pitch = head_pitch;
    if (tr_servo_read(TR_SERVO_YAW, &st) == ESP_OK) {
        yaw = st.degrees;
    }
    if (tr_servo_read(TR_SERVO_PITCH, &st) == ESP_OK) {
        pitch = st.degrees;
    }
    float to_yaw, to_pitch;
    for (int tries = 0;; tries++) { // a spot far enough to show a new part of the room
        to_yaw = (float)((int)(esp_random() % (2 * SEARCH_YAW + 1)) - SEARCH_YAW);
        to_pitch =
            (float)(SEARCH_PITCH_LOW + esp_random() % (SEARCH_PITCH_HIGH - SEARCH_PITCH_LOW + 1));
        const float dyaw = fabsf(to_yaw - yaw), dpitch = fabsf(to_pitch - pitch);
        const bool new_place = dyaw >= SEARCH_MIN_YAW || dpitch >= SEARCH_MIN_PITCH;
        if ((new_place && dyaw <= SEARCH_MAX_STEP && dpitch <= SEARCH_MAX_STEP) || tries >= 20) {
            break;
        }
    }
    // Slower than following: both axes arrive together, at about 20 degrees
    // per second for the longer one.
    const float far = fmaxf(fabsf(to_yaw - yaw), fabsf(to_pitch - pitch));
    const int ms = (int)(far * SEARCH_MS_PER_DEG) < 400 ? 400 : (int)(far * SEARCH_MS_PER_DEG);
    events_note('G');
    tr_servo_move(TR_SERVO_YAW, to_yaw, ms);
    tr_servo_move(TR_SERVO_PITCH, to_pitch, ms);
    events_move('Y', yaw, to_yaw, ms);
    events_move('P', pitch, to_pitch, ms);
    next_look = now + ms * 1000LL + SEARCH_STAY_US;
    return true;
}

static void face_task(void *arg)
{
    const bool screen = tr_lcd_init() == ESP_OK;
    if (screen) {
        tr_lcd_brightness(BRIGHTNESS);
    } else {
        printf("no screen, the face only moves the head\n");
    }
    int64_t next_blink = esp_timer_get_time() + 2000000;
    int64_t blink_end = 0, next_glance = esp_timer_get_time() + 3000000, release_at = 0;
    if (tr_web_servos()) {
        go_home();
        release_at = esp_timer_get_time() + (HOME_MS + 300) * 1000; // then torque off
    }
    int64_t next_read = 0;
    face_t drawn = {.dx = 1000};
    view_t open = VIEW_FACE; // VIEW_STATUS or VIEW_WIFI: a list is on the screen
    bool was_down = false;
    int top = 0, tap_x = 0, tap_y = 0;
    int64_t next_update = 0;
    TickType_t wake = xTaskGetTickCount();
    // Since when following has been on with the robot awake: searching
    // starts SEARCH_AFTER_US after this, or after the last face.
    int64_t follow_since = esp_timer_get_time();
    bool was_following_on = follow;

    for (;;) {
        if (follow != was_following_on) {
            was_following_on = follow;
            follow_since = esp_timer_get_time();
        }
        // Asleep: nothing to draw. Wait, then draw the face again.
        if (tr_sleep_asleep()) {
            if (track_on) {
                track_set(false, false); // the finder stops too
            }
            tr_sleep_wait_awake();
            follow_since = esp_timer_get_time();
            open = VIEW_FACE;
            was_down = false;
            drawn.dx = 1000;
            next_read = 0;
            wake = xTaskGetTickCount();
            continue;
        }
        const int64_t now = esp_timer_get_time();

        // A tap is a touch that ends: act on where the finger was last.
        int x, y;
        const bool down = screen && tr_touch_read(&x, &y);
        if (down) {
            tap_x = x;
            tap_y = y;
        }
        const bool tap = was_down && !down;
        was_down = down;
        if (tap && open != VIEW_FACE) {
            ESP_LOGI("face", "tap at %d,%d", tap_x, tap_y); // to check the touch in /log
        }
        const bool on_x = tap && tr_screen_on_bar(tap_y);
        // A view asked for over HTTP: close whatever is open, then open it.
        if (view_request) {
            view_request = false;
            open = want_view;
            drawn.dx = 1000;
            top = 0;
            if (open != VIEW_FACE && open != VIEW_TRACK) {
                open_list(open, now, &next_update);
            }
        }
        // The left eye: the camera with the faces it finds.
        if (tap && open == VIEW_FACE && on_left_eye(tap_x, tap_y)) {
            open = VIEW_TRACK;
        }
        // The finder works while the head follows faces, or the camera view is
        // open; frames go to the screen only for the view.
        const bool want_show = open == VIEW_TRACK;
        if ((follow || want_show) && track_start() != ESP_OK) {
            follow = false; // no camera (as in the emulator): don't try again
            if (want_show) {
                open = VIEW_FACE;
                drawn.dx = 1000;
                continue;
            }
        }
        if (follow || want_show) {
            track_set(true, want_show);
        } else if (track_on) {
            track_set(false, false);
        }
        if (open == VIEW_TRACK) {
            tr_sleep_activity(); // an open view keeps the robot awake
            if (follow_step(now) || search_step(now, follow_since)) {
                release_at = now + RELEASE_US;
            }
            if (release_at && now >= release_at) {
                tr_servo_torque(TR_SERVO_YAW, false);
                tr_servo_torque(TR_SERVO_PITCH, false);
                release_at = 0;
            }
            track_frame();
            // Any tap on the screen (read once per frame, at the top of the
            // loop), or a touch on the head, goes back to the face.
            if (tap || tr_head_touch()) {
                ESP_LOGI("face", "track closed");
                open = VIEW_FACE;
                drawn.dx = 1000;
                vTaskDelay(pdMS_TO_TICKS(300)); // let go first
                was_down = false;
            }
            vTaskDelay(1); // one tick for the tasks below ours (web server, sleep)
            wake = xTaskGetTickCount();
            continue;
        }
        if (scan_done) {
            scan_done = false;
            take_scan();
            if (open == VIEW_WIFI) {
                build_wifi();
                top = tr_screen_scroll(top, 0);
                tr_screen_rows(top);
            }
        }
        if (tap && open == VIEW_FACE && (on_mouth(tap_x, tap_y) || on_right_eye(tap_x, tap_y))) {
            open = on_mouth(tap_x, tap_y) ? VIEW_STATUS : VIEW_WIFI;
            top = 0;
            open_list(open, now, &next_update);
        } else if (tap && open != VIEW_FACE) {
            if (on_x) {
                ESP_LOGI("face", "%s closed", view_names[open]);
                open = VIEW_FACE;
                drawn.dx = 1000; // draw the face again
            } else if (open == VIEW_STATUS) {
                tr_status_tap(tap_x, tap_y); // PREV or NEXT
                next_update = now + 1000000;
            } else if (tr_screen_button(tap_x, tap_y)) {
                top = tr_screen_scroll(top, tr_screen_button(tap_x, tap_y) * ROWS / 2);
                tr_screen_rows(top);
            }
        }
        if (open != VIEW_FACE) {
            tr_sleep_activity(); // an open list keeps the robot awake
            if (now >= next_update) {
                if (open == VIEW_STATUS) { // new numbers every second
                    tr_status_update();
                    next_update = now + 1000000;
                } else { // a new scan; the list changes when it is done
                    if (!scanning) {
                        start_scan();
                    }
                    next_update = now + SCAN_EVERY_US;
                }
            }
            vTaskDelayUntil(&wake, pdMS_TO_TICKS(FRAME_MS));
            continue; // the head stays still while a list is open
        }

        if (blink_now || now >= next_blink) {
            blink_now = false;
            blink_end = now + 150000;
            next_blink = now + 2500000 + esp_random() % 3000000;
        }
        if (follow_step(now) || search_step(now, follow_since)) {
            release_at = now + RELEASE_US; // torque off when it stops moving
        }
        const int64_t last_move = tr_web_last_move_us();
        const bool paused = last_move >= 0 && now - last_move < IDLE_PAUSE_US;
        if (tr_web_servos() && idle && !paused && !following && now >= next_glance) {
            glance();
            release_at = now + (GLANCE_MS + 300) * 1000;
            next_glance = now + 2000000 + esp_random() % 3000000;
        }
        // Torque off after each glance: quiet, cool, and safe for a hand.
        if (release_at && now >= release_at) {
            tr_servo_torque(TR_SERVO_YAW, false);
            tr_servo_torque(TR_SERVO_PITCH, false);
            release_at = 0;
        }
        if (tr_web_head_powered() && now >= next_read) {
            read_head();
            next_read = now + 200000;
        }

        // Following: the eyes and mouth look at the face and come back to the
        // center as the head catches up. Otherwise they point where the head points.
        static float eye_x, eye_y;
        const float to_x = follow ? EYE_SIGN_X * err_x * EYE_FOLLOW_X : head_yaw * EYE_X_PER_DEG;
        const float to_y = follow ? err_y * EYE_FOLLOW_Y : -(head_pitch - 20) * 0.6f;
        eye_x += (to_x - eye_x) * 0.5f; // halfway each frame: smooth, but quick
        eye_y += (to_y - eye_y) * 0.5f;
        const face_t f = {
            .dx = (int)lroundf(eye_x),
            .dy = (int)lroundf(eye_y),
            .closed = now < blink_end,
        };
        if (screen && (f.dx != drawn.dx || f.dy != drawn.dy || f.closed != drawn.closed)) {
            tr_lcd_render(face_pixel, (void *)&f);
            drawn = f;
        }
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(FRAME_MS));
    }
}

static esp_err_t face_get(httpd_req_t *req)
{
    static char text[2048];
    int n = snprintf(text, sizeof(text),
                     "views: face status wifi track\nfollowing: %s\nlooking around: %s\n"
                     "head: yaw %.1f, pitch %.1f degrees\nlast view asked: %s\n",
                     !follow     ? "off"
                     : following ? "on, a face is there"
                                 : "on, no face",
                     idle ? "on" : "off", head_yaw, head_pitch, view_names[want_view]);
    // The faces, while the track view is open. The Wi-Fi scan comes last,
    // because the page reads lines that start with two spaces as networks.
    if (!track_ready) {
        n += snprintf(text + n, sizeof(text) - n, "track: open the track view (left eye)\n");
    } else {
        const face_box_t b = track_best;
        n += snprintf(text + n, sizeof(text) - n,
                      "track: %d face(s), find %d ms, %d.%d finds/s, show %d.%d frames/s",
                      track_faces, track_ms, track_fps10 / 10, track_fps10 % 10, show_fps10 / 10,
                      show_fps10 % 10);
        n += track_faces
                 ? snprintf(text + n, sizeof(text) - n, ", best %d,%d to %d,%d, score %.2f\n", b.x0,
                            b.y0, b.x1, b.y1, b.score)
                 : snprintf(text + n, sizeof(text) - n, "\n");
    }
    if (scanned_at < 0) {
        snprintf(text + n, sizeof(text) - n, "Wi-Fi: no scan yet, open the Wi-Fi view\n");
        return httpd_resp_sendstr(req, text);
    }
    // aps may change under us while a scan ends: worst case one odd line.
    n += snprintf(text + n, sizeof(text) - n, "Wi-Fi: %u networks, %.0f s ago\n", ap_count,
                  (esp_timer_get_time() - scanned_at) / 1e6);
    for (int i = 0; i < ap_count && n < (int)sizeof(text) - 64; i++) {
        n += snprintf(text + n, sizeof(text) - n, "  %-32s %4d dBm  ch %2d  %s\n",
                      aps[i].ssid[0] ? (const char *)aps[i].ssid : "(hidden)", aps[i].rssi,
                      aps[i].primary, security(aps[i].authmode));
    }
    return httpd_resp_sendstr(req, text);
}

static esp_err_t face_post(httpd_req_t *req)
{
    if (!tr_ota_token_ok(req)) {
        return ESP_OK;
    }
    tr_sleep_wake(); // a change on the screen or the head: wake first
    char query[64], value[8];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        if (httpd_query_key_value(query, "follow", value, sizeof(value)) == ESP_OK) {
            follow = atoi(value) != 0;
        }
        if (httpd_query_key_value(query, "idle", value, sizeof(value)) == ESP_OK) {
            idle = atoi(value) != 0;
        }
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
    ESP_ERROR_CHECK(
        esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_SCAN_DONE, on_scan_done, NULL));
    const httpd_uri_t get = {.uri = "/face", .method = HTTP_GET, .handler = face_get};
    const httpd_uri_t post = {.uri = "/face", .method = HTTP_POST, .handler = face_post};
    ESP_ERROR_CHECK(tr_ota_add_uri(&get));
    ESP_ERROR_CHECK(tr_ota_add_uri(&post));
    const httpd_uri_t csv = {.uri = "/track.csv", .method = HTTP_GET, .handler = events_csv};
    ESP_ERROR_CHECK(tr_ota_add_uri(&csv));
    events_start();
    xTaskCreate(face_task, "face", 6144, NULL, 5, NULL);
}
