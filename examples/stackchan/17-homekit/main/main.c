// 17-homekit: the robot as a HomeKit bridge for the Tuya plugs at home. The
// plugs show up in Apple's Home app, and an Apple TV as home hub gives them
// Siri, automations and control from outside the house.
//
// 13-home with everything but the plugs taken out: no talk, no audio, no
// face. plugs.c talks to the plugs (tr_tuya), bridge.c is HomeKit.
//
// While it is a bridge the robot does little else. At boot the head goes to
// its rest position and the servos switch off. The screen has one tab per
// device, with a button per outlet (green is on; a tap switches it), and a
// last tab with the QR code to add the bridge in the Home app. It switches
// itself off after SCREEN_S seconds. A touch on the screen or on the head
// switches it on again, and that touch does nothing else.
//
//   GET  /home            the bridge and the plugs, as text
//   POST /home?unpair=1   forget every pairing and restart (token needed)
#include <stdio.h>
#include <string.h>
#include "bridge.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "plugs.h"
#include "qrcode.h"
#include "tr_board.h"
#include "tr_ota.h"
#include "tr_screen.h"
#include "tr_web.h"

static const char *TAG = "main";

#define SCREEN_S   30 // seconds with no touch before the screen goes off
#define BRIGHTNESS 20 // screen backlight, in %
#define FRAME_MS   50

// The head to its rest position (looking ahead), then the servos off: a
// bridge does not move, and servos that hold a position use power and hum.
static void rest_head(void)
{
    if (tr_servo_init() != ESP_OK || tr_servo_power(true) != ESP_OK) {
        ESP_LOGW(TAG, "no head servos (no body, as in the emulator)");
        return;
    }
    tr_servo_move(TR_SERVO_YAW, 0, 1000);
    tr_servo_move(TR_SERVO_PITCH, TR_PITCH_MIN, 1000);
    vTaskDelay(pdMS_TO_TICKS(1300)); // let the move finish
    tr_servo_torque(TR_SERVO_YAW, false);
    tr_servo_torque(TR_SERVO_PITCH, false);
    tr_servo_power(false);
}

#define TAB_H   32 // the tabs, at the top
#define INFO_H  24 // one line of text, at the bottom
#define GAP     8
#define BTN_W   ((TR_LCD_W - 3 * GAP) / 2)
#define TAB_BG  TR_LCD_RGB(40, 40, 48)
#define SEL_BG  TR_LCD_RGB(20, 60, 120)
#define ON_BG   TR_LCD_RGB(40, 150, 70)
#define OFF_BG  TR_LCD_RGB(50, 50, 60)
#define BUSY_BG TR_LCD_RGB(25, 25, 30)

// One tab per device, and a last one with the QR code.
static int tab_count(void)
{
    return plugs_count() + 1;
}

// Text in the middle of a box w pixels wide: large when it fits, small when
// it does not, and cut when even small is too long.
static void text_center(int x, int w, int y, uint16_t fg, uint16_t bg, const char *text)
{
    char cut[54];
    const int scale = (int)strlen(text) * 12 <= w - 4 ? 2 : 1;
    snprintf(cut, sizeof(cut), "%.*s", (w - 4) / 6, text);
    tr_lcd_text(x + (w - (int)strlen(cut) * 6 * scale) / 2, y + (scale == 1 ? 4 : 0), scale, fg, bg,
                cut);
}

static void draw_tabs(int selected)
{
    const int w = TR_LCD_W / tab_count();
    for (int i = 0; i < tab_count(); i++) {
        const uint16_t bg = i == selected ? SEL_BG : TAB_BG;
        tr_lcd_fill_rect(i * w, 0, w - 1, TAB_H, bg);
        text_center(i * w, w - 1, 8, TR_SCREEN_FG, bg,
                    i < plugs_count() ? plugs_dev_name(i) : "QR");
    }
}

// Where the button of outlet sw is: two columns, as many rows as the device needs.
static void button_box(int dev, int sw, int *x, int *y, int *h)
{
    const int rows = (plugs_switches(dev) + 1) / 2;
    *h = (TR_LCD_H - TAB_H - INFO_H - (rows + 1) * GAP) / rows;
    if (*h > 72) {
        *h = 72;
    }
    *x = GAP + sw % 2 * (BTN_W + GAP);
    *y = TAB_H + GAP + sw / 2 * (*h + GAP);
}

// A device's tab: one button per outlet (green is on), then its IP and power.
// busy is the outlet that waits for the plug's answer after a tap, or -1.
static void draw_device(int dev, int busy)
{
    char info[54];
    tr_lcd_fill_rect(0, TAB_H, TR_LCD_W, TR_LCD_H - TAB_H, TR_LCD_BLACK);
    for (int sw = 0; sw < plugs_switches(dev); sw++) {
        int x, y, h;
        button_box(dev, sw, &x, &y, &h);
        const int on = sw == busy ? -1 : plugs_state(dev, sw);
        const uint16_t bg = on == 1 ? ON_BG : on == 0 ? OFF_BG : BUSY_BG;
        tr_lcd_fill_rect(x, y, BTN_W, h, bg);
        text_center(x, BTN_W, y + h / 2 - 18, TR_SCREEN_FG, bg, plugs_sw_name(dev, sw));
        text_center(x, BTN_W, y + h / 2 + 2, TR_SCREEN_FG, bg,
                    on == 1   ? "ON"
                    : on == 0 ? "OFF"
                              : "...");
    }
    plugs_info(dev, info, sizeof(info));
    text_center(0, TR_LCD_W, TR_LCD_H - INFO_H + 4, TR_SCREEN_FG, TR_LCD_BLACK, info);
}

// The last tab: the QR code for the Home app, black on white, and under it
// the same code as text. A phone needs the white border to read the QR code.
static void draw_qr_modules(esp_qrcode_handle_t qr)
{
    const int size = esp_qrcode_get_size(qr), area = TR_LCD_H - TAB_H - INFO_H;
    const int scale = area / (size + 2); // one module of border on each side
    const int x0 = (TR_LCD_W - size * scale) / 2, y0 = TAB_H + (area - size * scale) / 2;
    for (int y = 0; y < size; y++) {
        for (int x = 0; x < size; x++) {
            if (esp_qrcode_get_module(qr, x, y)) {
                tr_lcd_fill_rect(x0 + x * scale, y0 + y * scale, scale, scale, TR_LCD_BLACK);
            }
        }
    }
}

static void draw_setup(void)
{
    char line[40];
    tr_lcd_fill_rect(0, TAB_H, TR_LCD_W, TR_LCD_H - TAB_H, TR_LCD_WHITE);
    esp_qrcode_config_t cfg = ESP_QRCODE_CONFIG_DEFAULT();
    cfg.display_func = draw_qr_modules;
    if (esp_qrcode_generate(&cfg, bridge_qr_text()) != ESP_OK) {
        ESP_LOGW(TAG, "no QR code");
    }
    const int paired = bridge_paired();
    if (paired) {
        snprintf(line, sizeof(line), "%s  paired (%d)", bridge_code(), paired);
    } else {
        snprintf(line, sizeof(line), "%s  not paired", bridge_code());
    }
    text_center(0, TR_LCD_W, TR_LCD_H - INFO_H + 4, TR_LCD_BLACK, TR_LCD_WHITE, line);
}

// The outlet under a tap on a device's tab, or -1.
static int button_at(int dev, int tx, int ty)
{
    for (int sw = 0; sw < plugs_switches(dev); sw++) {
        int x, y, h;
        button_box(dev, sw, &x, &y, &h);
        if (tx >= x && tx < x + BTN_W && ty >= y && ty < y + h) {
            return sw;
        }
    }
    return -1;
}

static void screen_task(void *arg)
{
    rest_head();
    const bool screen = tr_lcd_init() == ESP_OK;
    if (!screen) {
        ESP_LOGW(TAG, "no screen (as in the emulator)");
        vTaskDelete(NULL);
    }
    tr_lcd_brightness(BRIGHTNESS);
    bool on = true, was_down = false, redraw = true;
    int tap_x = 0, tap_y = 0, drawn_paired = -1, busy = -1;
    // Not paired yet: start on the QR code. Paired: on the first device.
    int tab = bridge_paired() || plugs_count() == 0 ? 0 : plugs_count();
    int64_t off_at = esp_timer_get_time() + SCREEN_S * 1000000LL;
    unsigned drawn_reads = 0, busy_reads = 0;
    for (;;) {
        // A tap is a touch that ends: act on where the finger was last.
        int x, y;
        const bool down = tr_touch_read(&x, &y);
        if (down) {
            tap_x = x;
            tap_y = y;
        }
        bool tap = was_down && !down;
        was_down = down;
        const int64_t now = esp_timer_get_time();
        if (down || tap || tr_head_touch() > 0) {
            off_at = now + SCREEN_S * 1000000LL;
            if (!on) {
                on = true;
                tap = false; // this touch only wakes the screen
                was_down = false;
                tr_lcd_power(true);
                redraw = true;
            }
        } else if (on && now >= off_at) {
            on = false;
            tr_lcd_power(false);
        }
        if (on && tap && tap_y < TAB_H) {
            tab = tap_x / (TR_LCD_W / tab_count());
            tab = tab < tab_count() ? tab : tab_count() - 1;
            busy = -1;
            redraw = true;
        } else if (on && tap && tab < plugs_count()) {
            const int sw = button_at(tab, tap_x, tap_y);
            const int state = sw < 0 ? -1 : plugs_state(tab, sw);
            // The button stays dark until the plug has been read again.
            if (state >= 0 && plugs_set(tab, sw, state == 0)) {
                busy = sw;
                busy_reads = plugs_reads();
                redraw = true;
            }
        }
        if (busy >= 0 && plugs_reads() != busy_reads) {
            busy = -1;
        }
        if (on && (redraw || plugs_reads() != drawn_reads || bridge_paired() != drawn_paired)) {
            redraw = false;
            drawn_reads = plugs_reads();
            drawn_paired = bridge_paired();
            draw_tabs(tab);
            if (tab < plugs_count()) {
                draw_device(tab, busy);
            } else {
                draw_setup();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(FRAME_MS));
    }
}

static esp_err_t home_get(httpd_req_t *req)
{
    static char text[640];
    const int n = snprintf(text, sizeof(text), "homekit: %d paired\n", bridge_paired());
    plugs_text(text + n, sizeof(text) - (size_t)n);
    return httpd_resp_sendstr(req, text);
}

static esp_err_t home_post(httpd_req_t *req)
{
    if (!tr_ota_token_ok(req)) {
        return ESP_OK;
    }
    char query[32], value[4];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK &&
        httpd_query_key_value(query, "unpair", value, sizeof(value)) == ESP_OK) {
        bridge_unpair();
        return httpd_resp_sendstr(req, "pairings forgotten, restarting\n");
    }
    return home_get(req);
}

void app_main(void)
{
    ESP_ERROR_CHECK(tr_ota_add_status(tr_board_status));
    ESP_ERROR_CHECK(tr_ota_start());
    ESP_ERROR_CHECK(tr_web_start());
    // A bridge must answer Home at any time: no sleep, also on the battery.
    tr_sleep_set_after(0);
    const httpd_uri_t get = {.uri = "/home", .method = HTTP_GET, .handler = home_get};
    const httpd_uri_t post = {.uri = "/home", .method = HTTP_POST, .handler = home_post};
    ESP_ERROR_CHECK(tr_ota_add_uri(&get));
    ESP_ERROR_CHECK(tr_ota_add_uri(&post));
    if (plugs_start(bridge_fresh) != ESP_OK) {
        ESP_LOGW(TAG, "no plugs (set TUYA_DEVICES in the env file)");
    }
    if (bridge_start() != ESP_OK) {
        ESP_LOGE(TAG, "HomeKit did not start");
    }
    xTaskCreate(screen_task, "screen", 6144, NULL, 4, NULL);
}
