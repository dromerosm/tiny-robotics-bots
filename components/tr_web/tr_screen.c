// tr_screen: text screens on the robot, and the status screen. See
// include/tr_screen.h.
#include "tr_screen.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_app_desc.h"
#include "esp_netif.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "tr_ota.h"

#define MAX_LINES 48
#define BG        TR_LCD_RGB(0, 0, 0)
#define FG        TR_SCREEN_FG
#define BAR_H     TR_SCREEN_BAR_H
#define COLS      TR_SCREEN_COLS
#define ROWS      TR_SCREEN_ROWS
#define TITLE_BG  TR_LCD_RGB(20, 60, 120)
#define BUTTON_BG TR_LCD_RGB(50, 50, 60)
#define EXIT_BG   TR_LCD_RGB(170, 40, 40)
#define LINE_FG   TR_LCD_RGB(110, 110, 130)

static char lines[MAX_LINES][COLS + 1];
static uint16_t line_color[MAX_LINES];
static int line_count;

void tr_screen_clear(void)
{
    line_count = 0;
}

int tr_screen_count(void)
{
    return line_count;
}

static void vrow(uint16_t color, const char *fmt, va_list args)
{
    if (line_count < MAX_LINES) {
        vsnprintf(lines[line_count], sizeof(lines[0]), fmt, args); // cut at COLS
        line_color[line_count++] = color;
    }
}

void tr_screen_row(uint16_t color, const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    vrow(color, fmt, args);
    va_end(args);
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
        tr_screen_row(FG, "%*s%.*s", indent, "", (int)n, p);
        p += n;
        len -= n;
        while (len > 0 && *p == ' ') {
            p++;
            len--;
        }
        indent = 2;
    }
}

void tr_screen_rows(int top)
{
    char padded[COLS + 1];
    for (int i = 0; i < ROWS; i++) {
        const char *src = top + i < line_count ? lines[top + i] : "";
        snprintf(padded, sizeof(padded), "%-*s", COLS, src);
        const uint16_t c = top + i < line_count ? line_color[top + i] : FG;
        tr_lcd_text(4, BAR_H + 6 + i * 22, 2, c, BG, padded);
    }
}

void tr_screen_draw(const char *title, const char *left, const char *right, int top)
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
    tr_screen_rows(top);
}

int tr_screen_scroll(int top, int delta)
{
    top += delta;
    top = top > line_count - ROWS ? line_count - ROWS : top;
    return top < 0 ? 0 : top;
}

// The whole top bar closes a screen, not only the X: easier to hit.
bool tr_screen_on_bar(int y)
{
    return y < BAR_H + 12;
}

int tr_screen_button(int x, int y)
{
    return y < TR_LCD_H - BAR_H ? 0 : x < TR_LCD_W / 2 ? -1 : 1;
}

// The status, one section per page.
enum { SEC_BATTERY, SEC_ROBOT, SEC_NETWORK, SEC_TOUCH, SEC_ROOM, SEC_COUNT };
static const char *section_names[] = {"Battery", "Robot", "Network", "Touch, motion", "Room"};
static int section;

static void build_battery(void)
{
    tr_battery_t b;
    if (tr_battery_read(&b) == ESP_OK) {
        tr_screen_row(FG, "voltage    %d.%02d V", b.millivolts / 1000, b.millivolts % 1000 / 10);
        tr_screen_row(FG, "current    %d mA %s", abs(b.milliamps), b.milliamps < 0 ? "in" : "out");
        tr_screen_row(FG, "average    %.0f mA", tr_battery_average_ma());
    } else {
        tr_screen_row(FG, "INA226: no answer");
    }
    tr_power_t p;
    if (tr_power_read(&p) == ESP_OK) {
        tr_screen_row(FG, "charge     %d %%, %s", p.percent, p.state);
        const int m = tr_battery_minutes_left();
        if (m >= 0) {
            tr_screen_row(m >= 60   ? TR_SCREEN_GREEN
                          : m >= 20 ? TR_SCREEN_YELLOW
                                    : TR_SCREEN_RED,
                          "time left  %d h %02d min", m / 60, m % 60);
        } else {
            tr_screen_row(FG, "time left  -");
        }
        if (p.usb_mv) {
            tr_screen_row(FG, "USB        in, %d.%02d V", p.usb_mv / 1000, p.usb_mv % 1000 / 10);
        } else {
            tr_screen_row(FG, "USB        %s", p.usb ? "in" : "out");
        }
        tr_screen_row(FG, "system     %d.%02d V", p.system_mv / 1000, p.system_mv % 1000 / 10);
    } else {
        tr_screen_row(FG, "power chip: no answer");
    }
    char text[64];
    tr_sleep_status(text, sizeof(text));
    const char *s = text + strlen("sleep: ");
    tr_screen_row(FG, "%.*s", (int)strcspn(s, "\n"), s); // "awake, sleeps in 42 s"
}

static void build_robot(void)
{
    const esp_app_desc_t *app = esp_app_get_description();
    const esp_partition_t *run = esp_ota_get_running_partition();
    tr_screen_row(FG, "%s on %s, %s", app->project_name, run ? run->label : "?", tr_ota_state());
    tr_screen_row(FG, "%s", app->version);
    tr_screen_row(FG, "built %s %.5s", app->date, app->time);
    const int64_t s = esp_timer_get_time() / 1000000;
    tr_screen_row(FG, "up %lld h %02lld min %02lld s", s / 3600, s / 60 % 60, s % 60);
    char text[64];
    const int n = snprintf(text, sizeof(text), "last restart: %s", tr_ota_last_restart());
    wrapped(text, n < (int)sizeof(text) ? n : sizeof(text) - 1);
    tr_screen_row(FG, "memory %lu KB free", (unsigned long)esp_get_free_heap_size() / 1024);
    tr_screen_row(FG, "lowest since boot %lu KB",
                  (unsigned long)esp_get_minimum_free_heap_size() / 1024);
}

static void build_network(void)
{
    wifi_ap_record_t ap;
    if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
        tr_screen_row(FG, "%s", (const char *)ap.ssid);
        tr_screen_row(FG, "signal %d dBm, channel %d", ap.rssi, ap.primary);
    } else {
        tr_screen_row(FG, "Wi-Fi: not connected");
    }
    esp_netif_t *sta = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    esp_netif_ip_info_t ip;
    if (sta && esp_netif_get_ip_info(sta, &ip) == ESP_OK && ip.ip.addr) {
        tr_screen_row(FG, "IP " IPSTR, IP2STR(&ip.ip));
    }
    const char *host = NULL;
    if (sta && esp_netif_get_hostname(sta, &host) == ESP_OK && host) {
        tr_screen_row(FG, "%s.local", host);
    }
    tr_screen_row(FG, "%s", "");
    tr_screen_row(FG, "update token:");
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

static void draw_status(void)
{
    char title[24];
    snprintf(title, sizeof(title), "%s %d/%d", section_names[section], section + 1, SEC_COUNT);
    build_status();
    tr_screen_draw(title, "PREV", "NEXT", 0);
}

void tr_status_open(void)
{
    section = SEC_BATTERY;
    draw_status();
}

void tr_status_update(void)
{
    build_status();
    tr_screen_rows(0);
}

void tr_status_tap(int x, int y)
{
    const int b = tr_screen_button(x, y);
    if (b) {
        section = (section + (b < 0 ? SEC_COUNT - 1 : 1)) % SEC_COUNT;
        draw_status();
    }
}
