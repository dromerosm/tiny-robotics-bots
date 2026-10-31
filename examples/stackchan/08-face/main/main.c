// 08-face: StackChan's face on the real screen, and a head that looks around.
//
// The eyes follow where the head really points: the example reads both servos
// a few times a second. When idle, the head glances somewhere new every few
// seconds, slowly and in a small range. A move from the page or curl pauses
// that for IDLE_PAUSE_US.
//
//   GET  /face                  idle on or off, and where the head looks
//   POST /face?idle=0|1         stop or start looking around (token needed)
//   POST /face?blink=1          blink once (token needed)
//
// Tap the mouth on the screen: a status screen opens, with the same facts as
// the page, the sensors and the update token. UP and DOWN scroll, X closes it.
// The token is shown only here, on the robot itself, never on the network.
//
// The page at http://stackchan.local/app (tr_web) shows a card for this.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_http_server.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tr_board.h"
#include "tr_ota.h"
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

static volatile bool idle = true;
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

static void glance(void)
{
    const float yaw = (float)((int)(esp_random() % (2 * YAW_RANGE + 1)) - YAW_RANGE);
    const float pitch = (float)(PITCH_LOW + esp_random() % (PITCH_HIGH - PITCH_LOW + 1));
    if (tr_web_head_power() == ESP_OK) {
        tr_servo_move(TR_SERVO_YAW, yaw, GLANCE_MS);
        tr_servo_move(TR_SERVO_PITCH, pitch, GLANCE_MS);
    }
}

// The status screen.
#define COLS      26 // characters per row at scale 2
#define ROWS      8  // text rows between the two bars
#define BAR_H     28
#define MAX_LINES 48
#define TITLE_BG  TR_LCD_RGB(20, 60, 120)
#define BUTTON_BG TR_LCD_RGB(50, 50, 60)
#define EXIT_BG   TR_LCD_RGB(170, 40, 40)
static char lines[MAX_LINES][COLS + 1];
static int line_count;

// Status text, sensors and token, cut into rows of COLS characters.
static void build_status(void)
{
    static char text[1800];
    tr_ota_status_text(text, sizeof(text));
    size_t at = strlen(text);
    tr_sensors_status(text + at, sizeof(text) - at);
    at = strlen(text);
    snprintf(text + at, sizeof(text) - at, "update token: %s\n", tr_ota_token());

    line_count = 0;
    const char *p = text;
    while (*p && line_count < MAX_LINES) {
        const char *nl = strchr(p, '\n');
        size_t len = nl ? (size_t)(nl - p) : strlen(p);
        do {
            const size_t n = len > COLS ? COLS : len;
            memcpy(lines[line_count], p, n);
            lines[line_count][n] = 0;
            line_count++;
            p += n;
            len -= n;
        } while (len > 0 && line_count < MAX_LINES);
        p = nl ? nl + 1 : p;
    }
}

static void draw_rows(int top)
{
    char row[COLS + 1];
    for (int i = 0; i < ROWS; i++) {
        const char *src = top + i < line_count ? lines[top + i] : "";
        snprintf(row, sizeof(row), "%-*s", COLS, src);
        tr_lcd_text(4, BAR_H + 6 + i * 22, 2, FG, BG, row);
    }
}

static void draw_status_screen(int top)
{
    tr_lcd_fill(BG);
    tr_lcd_fill_rect(0, 0, TR_LCD_W, BAR_H, TITLE_BG);
    tr_lcd_text(8, 6, 2, FG, TITLE_BG, "Status");
    tr_lcd_fill_rect(TR_LCD_W - 56, 0, 56, BAR_H, EXIT_BG);
    tr_lcd_text(TR_LCD_W - 34, 6, 2, FG, EXIT_BG, "X");
    tr_lcd_fill_rect(0, TR_LCD_H - BAR_H, TR_LCD_W / 2 - 2, BAR_H, BUTTON_BG);
    tr_lcd_fill_rect(TR_LCD_W / 2 + 2, TR_LCD_H - BAR_H, TR_LCD_W / 2 - 2, BAR_H, BUTTON_BG);
    tr_lcd_text(64, TR_LCD_H - BAR_H + 6, 2, FG, BUTTON_BG, "UP");
    tr_lcd_text(216, TR_LCD_H - BAR_H + 6, 2, FG, BUTTON_BG, "DOWN");
    draw_rows(top);
}

// The mouth, with some room around it for a finger.
static bool on_mouth(int x, int y)
{
    return x >= 90 && x < 230 && y >= 135 && y < 200;
}

static void face_task(void *arg)
{
    const bool screen = tr_lcd_init() == ESP_OK;
    if (!screen) {
        printf("no screen, the face only moves the head\n");
    }
    int64_t next_blink = esp_timer_get_time() + 2000000;
    int64_t blink_end = 0, next_glance = esp_timer_get_time() + 3000000, release_at = 0;
    int64_t next_read = 0;
    face_t drawn = {.dx = 1000};
    bool status_open = false, was_down = false;
    int top = 0, tap_x = 0, tap_y = 0;
    int64_t next_status = 0;
    TickType_t wake = xTaskGetTickCount();

    for (;;) {
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
        if (tap && !status_open && on_mouth(tap_x, tap_y)) {
            status_open = true;
            top = 0;
            build_status();
            draw_status_screen(top);
            next_status = now + 1000000;
        } else if (tap && status_open) {
            if (tap_y < BAR_H && tap_x >= TR_LCD_W - 56) {
                status_open = false;
                drawn.dx = 1000; // draw the face again
            } else if (tap_y >= TR_LCD_H - BAR_H) {
                top += tap_x < TR_LCD_W / 2 ? -ROWS / 2 : ROWS / 2;
                top = top > line_count - ROWS ? line_count - ROWS : top;
                top = top < 0 ? 0 : top;
                draw_rows(top);
            }
        }
        if (status_open) {
            if (now >= next_status) { // new numbers every second
                build_status();
                draw_rows(top);
                next_status = now + 1000000;
            }
            vTaskDelayUntil(&wake, pdMS_TO_TICKS(FRAME_MS));
            continue; // the head stays still while the status is open
        }

        if (blink_now || now >= next_blink) {
            blink_now = false;
            blink_end = now + 150000;
            next_blink = now + 2500000 + esp_random() % 3000000;
        }
        const int64_t last_move = tr_web_last_move_us();
        const bool paused = last_move >= 0 && now - last_move < IDLE_PAUSE_US;
        if (tr_web_servos() && idle && !paused && now >= next_glance) {
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

        const face_t f = {
            .dx = (int)(head_yaw * EYE_X_PER_DEG),
            .dy = (int)(-(head_pitch - 20) * 0.6f),
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
    char text[128];
    snprintf(text, sizeof(text), "looking around: %s\nhead: yaw %.1f, pitch %.1f degrees\n",
             idle ? "on" : "off", head_yaw, head_pitch);
    return httpd_resp_sendstr(req, text);
}

static esp_err_t face_post(httpd_req_t *req)
{
    if (!tr_ota_token_ok(req)) {
        return ESP_FAIL;
    }
    char query[64], value[8];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        if (httpd_query_key_value(query, "idle", value, sizeof(value)) == ESP_OK) {
            idle = atoi(value) != 0;
        }
        if (httpd_query_key_value(query, "blink", value, sizeof(value)) == ESP_OK) {
            blink_now = true;
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
    ESP_ERROR_CHECK(tr_ota_add_uri(&get));
    ESP_ERROR_CHECK(tr_ota_add_uri(&post));
    xTaskCreate(face_task, "face", 6144, NULL, 5, NULL);
}
