// 09-wifi: 08-face without the microphones and the camera, with the screen at
// 20 % to use less power. The right eye shows the Wi-Fi networks around.
//
// The eyes follow where the head really points: the example reads both servos
// a few times a second. At boot the head goes home and stays still. With
// "look around" on (POST /face?idle=1), it glances somewhere new every few
// seconds, slowly and in a small range. A move from the page or curl pauses
// that for IDLE_PAUSE_US.
//
//   GET  /face                  idle on or off, where the head looks, battery, last scan
//   POST /face?idle=0|1         stop or start looking around (token needed)
//   POST /face?blink=1          blink once (token needed)
//   POST /face?view=face|status|wifi   change the screen (token needed)
//
// Tap the mouth on the screen: a status screen opens, one section per page:
// battery (with a guess of the time left), robot, network (with the update
// token), touch and motion, and the room. PREV and NEXT change the section,
// X closes it. The token is shown only here, on the robot itself, never on
// the network.
//
// Tap the right eye: the Wi-Fi networks the robot sees, strongest first, with
// their signal and security. Red is open (or WEP), yellow old WPA, green WPA3.
// The list scans again every SCAN_EVERY_US. UP and DOWN scroll, X closes it.
// GET /face lists the last scan too.
//
// The page at http://stackchan.local/app (tr_web) shows a card for this.
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_app_desc.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "esp_wifi.h"
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
#define BRIGHTNESS    20                 // screen backlight, in %
#define SCAN_EVERY_US (10 * 1000 * 1000) // while the Wi-Fi list is open
#define MAX_APS       32

typedef enum { VIEW_FACE, VIEW_STATUS, VIEW_WIFI, VIEW_COUNT } view_t;
static const char *view_names[] = {"face", "status", "wifi"};
static volatile view_t want_view = VIEW_FACE; // set by POST /face?view=..
static volatile bool view_request;

// Off at boot: the head goes home and stays still. The page can turn it on.
static volatile bool idle = false;
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

// The status screen.
#define COLS      26 // characters per row at scale 2
#define ROWS      8  // text rows between the two bars
#define BAR_H     28
#define MAX_LINES 48
#define TITLE_BG  TR_LCD_RGB(20, 60, 120)
#define BUTTON_BG TR_LCD_RGB(50, 50, 60)
#define EXIT_BG   TR_LCD_RGB(170, 40, 40)
#define GREEN     TR_LCD_RGB(80, 220, 100)
#define YELLOW    TR_LCD_RGB(240, 200, 60)
#define RED       TR_LCD_RGB(240, 80, 80)
#define LINE_FG   TR_LCD_RGB(110, 110, 130)
static char lines[MAX_LINES][COLS + 1]; // the status or the Wi-Fi list
static uint16_t line_color[MAX_LINES];
static int line_count;

// The battery: StackChan's is 550 mAh (M5Stack's docs). The time left is
// that capacity times the power chip's charge in %, over the current the
// INA226 measures, averaged over about 20 s. A guess: it assumes the robot
// keeps using what it uses now.
#define BATTERY_MAH 550
static float avg_ma; // above 0 while the battery discharges

// Every second, from the face task.
static void battery_tick(void)
{
    tr_battery_t b;
    if (tr_battery_read(&b) != ESP_OK) {
        return;
    }
    if (b.milliamps <= 0 || avg_ma <= 0) {
        avg_ma = b.milliamps; // charging, on USB, or a new start
    } else {
        avg_ma += (b.milliamps - avg_ma) * 0.05f;
    }
}

// Minutes left at the current use, or -1 if it is not discharging.
static int minutes_left(int percent)
{
    return avg_ma >= 5 ? (int)(BATTERY_MAH * percent / 100.0f / avg_ma * 60) : -1;
}

// The status, one section per page. PREV and NEXT change the section.
enum { SEC_BATTERY, SEC_ROBOT, SEC_NETWORK, SEC_TOUCH, SEC_ROOM, SEC_COUNT };
static const char *section_names[] = {"Battery", "Robot", "Network", "Touch, motion", "Room"};
static int section;

static void row(uint16_t color, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
static void row(uint16_t color, const char *fmt, ...)
{
    if (line_count >= MAX_LINES) {
        return;
    }
    va_list args;
    va_start(args, fmt);
    vsnprintf(lines[line_count], sizeof(lines[0]), fmt, args); // cut at COLS
    va_end(args);
    line_color[line_count++] = color;
}

// One line of text in rows of COLS, cut at spaces when it can. The next rows
// start with two spaces.
static void wrapped(const char *p, size_t len)
{
    int indent = 0;
    while (len > 0 && line_count < MAX_LINES) {
        size_t n = len;
        if (indent + n > COLS) {
            n = COLS - indent;
            for (size_t k = n; k > COLS / 2; k--) {
                if (p[k] == ' ') {
                    n = k;
                    break;
                }
            }
        }
        row(FG, "%*s%.*s", indent, "", (int)n, p);
        p += n;
        len -= n;
        while (len > 0 && *p == ' ') {
            p++;
            len--;
        }
        indent = 2;
    }
}

static void build_battery(void)
{
    tr_battery_t b;
    if (tr_battery_read(&b) == ESP_OK) {
        row(FG, "voltage    %d.%02d V", b.millivolts / 1000, b.millivolts % 1000 / 10);
        row(FG, "current    %d mA %s", abs(b.milliamps), b.milliamps < 0 ? "in" : "out");
        row(FG, "average    %.0f mA", avg_ma > 0 ? avg_ma : 0);
    } else {
        row(FG, "INA226: no answer");
    }
    tr_power_t p;
    if (tr_power_read(&p) != ESP_OK) {
        row(FG, "power chip: no answer");
        return;
    }
    row(FG, "charge     %d %%", p.percent);
    row(FG, "state      %s", p.state);
    const int m = minutes_left(p.percent);
    if (m >= 0) {
        row(m >= 60 ? GREEN : m >= 20 ? YELLOW : RED, "time left  %d h %02d min", m / 60, m % 60);
    } else {
        row(FG, "time left  -");
    }
    if (p.usb_mv) {
        row(FG, "USB        in, %d.%02d V", p.usb_mv / 1000, p.usb_mv % 1000 / 10);
    } else {
        row(FG, "USB        %s", p.usb ? "in" : "out");
    }
    row(FG, "system     %d.%02d V", p.system_mv / 1000, p.system_mv % 1000 / 10);
}

static void build_robot(void)
{
    const esp_app_desc_t *app = esp_app_get_description();
    const esp_partition_t *run = esp_ota_get_running_partition();
    row(FG, "%s on %s, %s", app->project_name, run ? run->label : "?", tr_ota_state());
    row(FG, "%s", app->version);
    row(FG, "built %s %.5s", app->date, app->time);
    const int64_t s = esp_timer_get_time() / 1000000;
    row(FG, "up %lld h %02lld min %02lld s", s / 3600, s / 60 % 60, s % 60);
    char text[64];
    const int n = snprintf(text, sizeof(text), "last restart: %s", tr_ota_last_restart());
    wrapped(text, n < (int)sizeof(text) ? n : sizeof(text) - 1);
    row(FG, "memory %lu KB free", (unsigned long)esp_get_free_heap_size() / 1024);
    row(FG, "lowest since boot %lu KB", (unsigned long)esp_get_minimum_free_heap_size() / 1024);
}

static void build_network(void)
{
    wifi_ap_record_t ap;
    if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
        row(FG, "%s", (const char *)ap.ssid);
        row(FG, "signal %d dBm, channel %d", ap.rssi, ap.primary);
    } else {
        row(FG, "Wi-Fi: not connected");
    }
    esp_netif_t *sta = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    esp_netif_ip_info_t ip;
    if (sta && esp_netif_get_ip_info(sta, &ip) == ESP_OK && ip.ip.addr) {
        row(FG, "IP " IPSTR, IP2STR(&ip.ip));
    }
    const char *host = NULL;
    if (sta && esp_netif_get_hostname(sta, &host) == ESP_OK && host) {
        row(FG, "%s.local", host);
    }
    row(FG, "%s", "");
    row(FG, "update token:");
    const char *t = tr_ota_token();
    wrapped(t, strlen(t));
}

// The sensors' own lines, for the touch and room sections. The power chip's
// lines are in the battery section.
static void build_sensors(bool touch)
{
    static const char *touch_lines[] = {"head touch", "screen touch", "accel", "gyro"};
    static char text[700];
    tr_sensors_status(text, sizeof(text));
    for (const char *p = text; *p;) {
        const char *nl = strchr(p, '\n');
        const size_t len = nl ? (size_t)(nl - p) : strlen(p);
        bool is_touch = false;
        for (int i = 0; i < 4; i++) {
            is_touch |= strncmp(p, touch_lines[i], strlen(touch_lines[i])) == 0;
        }
        if (is_touch == touch && strncmp(p, "power chip", 10) != 0) {
            wrapped(p, len);
        }
        p = nl ? nl + 1 : p + len;
    }
}

static void build_status(void)
{
    line_count = 0;
    switch (section) {
        case SEC_BATTERY:
            build_battery();
            break;
        case SEC_ROBOT:
            build_robot();
            break;
        case SEC_NETWORK:
            build_network();
            break;
        default:
            build_sensors(section == SEC_TOUCH);
            break;
    }
}

static void draw_rows(int top)
{
    char padded[COLS + 1];
    for (int i = 0; i < ROWS; i++) {
        const char *src = top + i < line_count ? lines[top + i] : "";
        snprintf(padded, sizeof(padded), "%-*s", COLS, src);
        const uint16_t c = top + i < line_count ? line_color[top + i] : FG;
        tr_lcd_text(4, BAR_H + 6 + i * 22, 2, c, BG, padded);
    }
}

// A title bar with X, a line under it, the rows, and two buttons at the bottom.
static void draw_screen(const char *title, const char *left, const char *right, int top)
{
    tr_lcd_fill(BG);
    tr_lcd_fill_rect(0, 0, TR_LCD_W, BAR_H, TITLE_BG);
    tr_lcd_text(8, 6, 2, FG, TITLE_BG, title);
    tr_lcd_fill_rect(TR_LCD_W - 56, 0, 56, BAR_H, EXIT_BG);
    tr_lcd_text(TR_LCD_W - 34, 6, 2, FG, EXIT_BG, "X");
    tr_lcd_fill_rect(0, BAR_H + 2, TR_LCD_W, 1, LINE_FG);
    tr_lcd_fill_rect(0, TR_LCD_H - BAR_H, TR_LCD_W / 2 - 2, BAR_H, BUTTON_BG);
    tr_lcd_fill_rect(TR_LCD_W / 2 + 2, TR_LCD_H - BAR_H, TR_LCD_W / 2 - 2, BAR_H, BUTTON_BG);
    tr_lcd_text(80 - 6 * (int)strlen(left), TR_LCD_H - BAR_H + 6, 2, FG, BUTTON_BG, left);
    tr_lcd_text(240 - 6 * (int)strlen(right), TR_LCD_H - BAR_H + 6, 2, FG, BUTTON_BG, right);
    draw_rows(top);
}

static void draw_status(void)
{
    char title[24];
    snprintf(title, sizeof(title), "%s %d/%d", section_names[section], section + 1, SEC_COUNT);
    draw_screen(title, "PREV", "NEXT", 0);
}

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
    line_count = 0;
    if (scanned_at < 0) {
        strcpy(lines[0], "scanning...");
        line_color[0] = FG;
        line_count = 1;
        return;
    }
    snprintf(lines[0], sizeof(lines[0]), "%u networks, dBm", ap_count);
    line_color[0] = FG;
    line_count = 1;
    for (int i = 0; i < ap_count && line_count < MAX_LINES; i++) {
        const char *name = aps[i].ssid[0] ? (const char *)aps[i].ssid : "(hidden)";
        snprintf(lines[line_count], sizeof(lines[0]), "%-13.13s %4d %s", name, aps[i].rssi,
                 security(aps[i].authmode));
        line_color[line_count] = security_color(aps[i].authmode);
        line_count++;
    }
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

// The status or Wi-Fi screen.
static void open_list(view_t v, int64_t now, int64_t *next_update)
{
    if (v == VIEW_WIFI) {
        if (!scanning) {
            start_scan();
        }
        build_wifi();
        draw_screen("Wi-Fi", "UP", "DOWN", 0);
        *next_update = now + SCAN_EVERY_US;
    } else {
        section = SEC_BATTERY;
        build_status();
        draw_status();
        *next_update = now + 1000000;
    }
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
    int64_t next_update = 0, next_battery = 0;
    TickType_t wake = xTaskGetTickCount();

    for (;;) {
        const int64_t now = esp_timer_get_time();
        if (now >= next_battery) {
            battery_tick();
            next_battery = now + 1000000;
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
        if (tap && open != VIEW_FACE) {
            ESP_LOGI("face", "tap at %d,%d", tap_x, tap_y); // to check the touch in /log
        }
        // The whole top bar closes a screen, not only the X: easier to hit.
        const bool on_x = tap && tap_y < BAR_H + 12;
        // A view asked for over HTTP: close whatever is open, then open it.
        if (view_request) {
            view_request = false;
            open = want_view;
            drawn.dx = 1000;
            top = 0;
            if (open != VIEW_FACE) {
                open_list(open, now, &next_update);
            }
        }
        if (scan_done) {
            scan_done = false;
            take_scan();
            if (open == VIEW_WIFI) {
                build_wifi();
                top = top > line_count - ROWS ? line_count - ROWS : top;
                top = top < 0 ? 0 : top;
                draw_rows(top);
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
            } else if (tap_y >= TR_LCD_H - BAR_H && open == VIEW_STATUS) {
                section = (section + (tap_x < TR_LCD_W / 2 ? SEC_COUNT - 1 : 1)) % SEC_COUNT;
                build_status();
                draw_status();
                next_update = now + 1000000;
            } else if (tap_y >= TR_LCD_H - BAR_H) {
                top += tap_x < TR_LCD_W / 2 ? -ROWS / 2 : ROWS / 2;
                top = top > line_count - ROWS ? line_count - ROWS : top;
                top = top < 0 ? 0 : top;
                draw_rows(top);
            }
        }
        if (open != VIEW_FACE) {
            if (now >= next_update) {
                if (open == VIEW_STATUS) { // new numbers every second
                    build_status();
                    draw_rows(top);
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
    static char text[2048];
    int n = snprintf(text, sizeof(text),
                     "views: face status wifi\nlooking around: %s\n"
                     "head: yaw %.1f, pitch %.1f degrees\nlast view asked: %s\n",
                     idle ? "on" : "off", head_yaw, head_pitch, view_names[want_view]);
    tr_power_t p;
    if (tr_power_read(&p) == ESP_OK) {
        const int m = minutes_left(p.percent);
        n += snprintf(text + n, sizeof(text) - n, "battery: %d %%, %s, average %.0f mA", p.percent,
                      p.state, avg_ma > 0 ? avg_ma : 0);
        n += m >= 0 ? snprintf(text + n, sizeof(text) - n, ", about %d h %02d min left\n", m / 60,
                               m % 60)
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
    xTaskCreate(face_task, "face", 6144, NULL, 5, NULL);
}
