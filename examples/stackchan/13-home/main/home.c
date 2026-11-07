// home: see home.h.
//
// The devices come from CONFIG_HOME_DEVICES, set at build time from
// TUYA_DEVICES in wifi.env (never in git). One device per ";", fields by ",":
//
//   name,version,ip,id,key-hex,switch,switch,...
//   switch = dp:label:long_name[:ap]
//
// label goes on the screen button (up to 6 characters), long_name is for the
// voice and the web ("_" for a space). ":ap" marks the outlet that feeds the
// Wi-Fi access point: switching it off while the robot is on
// CONFIG_HOME_AP_SSID cuts the robot off, so it asks first (a second tap on
// the screen, a yes by voice, confirm=1 on the web).
//
// A worker task does the switching and the reading, so the face task never
// waits on the network. The voice's tool calls run in the talk's tools task.
// tr_tuya takes one call at a time.
#include "home.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "sdkconfig.h"
#include "tr_board.h"
#include "tr_ota.h"
#include "tr_screen.h"
#include "tr_tuya.h"

static const char *TAG = "home";

#define MAX_DEV    4
#define MAX_SW     5
#define REFRESH_US 5000000 // numbers on the open screen
#define CONFIRM_US 4000000 // time for the second tap
#define BAR_H      TR_SCREEN_BAR_H
#define TOP        32 // first device block
#define BLOCK_H    69
#define BTN_Y      24 // buttons, from the top of a block
#define BTN_H      40
#define BG         TR_LCD_RGB(0, 0, 0)
#define FG         TR_SCREEN_FG
#define TITLE_BG   TR_LCD_RGB(20, 60, 120)
#define EXIT_BG    TR_LCD_RGB(170, 40, 40)
#define ON_BG      TR_LCD_RGB(40, 150, 70)
#define OFF_BG     TR_LCD_RGB(50, 50, 60)
#define BUSY_BG    TR_LCD_RGB(25, 25, 30)
#define ASK_BG     TR_SCREEN_YELLOW

typedef struct {
    int dp;
    char label[8];
    char name[24];
    bool ap;
    int on; // 1, 0, or -1 while not known (or being switched)
} sw_t;

typedef struct {
    char name[16];
    char id[24];
    char ip[16];
    tr_tuya_dev_t dev;
    sw_t sw[MAX_SW];
    int nsw;
    int watts10; // dp 19, in tenths of a watt (-1: not known)
    int err;     // the last tr_tuya result, 0 when fine
} home_dev_t;

typedef struct {
    int dev; // -1: read all
    int sw;
    bool on;
} job_t;

static home_dev_t devs[MAX_DEV];
static int ndev;
static SemaphoreHandle_t mu; // devs[] between the worker, the tools and the face
static QueueHandle_t jobs;
static volatile bool screen_open, dirty;
static int ask_dev = -1, ask_sw = -1; // a switch waiting for its second tap
static int64_t ask_until;

// --- Devices -----------------------------------------------------------------

static void copy(char *out, size_t size, const char *in)
{
    snprintf(out, size, "%s", in ? in : "");
}

static bool parse_switch(home_dev_t *d, char *spec)
{
    char *save = NULL;
    const char *dp = strtok_r(spec, ":", &save), *label = strtok_r(NULL, ":", &save);
    const char *name = strtok_r(NULL, ":", &save), *flag = strtok_r(NULL, ":", &save);
    if (!dp || !label || !name || d->nsw == MAX_SW) {
        return false;
    }
    sw_t *s = &d->sw[d->nsw++];
    s->dp = atoi(dp);
    copy(s->label, sizeof(s->label), label);
    copy(s->name, sizeof(s->name), name);
    for (char *p = s->name; *p; p++) {
        *p = *p == '_' ? ' ' : *p;
    }
    s->ap = flag && strcmp(flag, "ap") == 0;
    s->on = -1;
    return s->dp > 0;
}

static void parse_devices(void)
{
    static char text[sizeof(CONFIG_HOME_DEVICES)];
    copy(text, sizeof(text), CONFIG_HOME_DEVICES);
    char *save_dev = NULL;
    for (char *entry = strtok_r(text, ";", &save_dev); entry && ndev < MAX_DEV;
         entry = strtok_r(NULL, ";", &save_dev)) {
        home_dev_t *d = &devs[ndev];
        memset(d, 0, sizeof(*d));
        char *save = NULL;
        const char *name = strtok_r(entry, ",", &save), *version = strtok_r(NULL, ",", &save);
        const char *ip = strtok_r(NULL, ",", &save), *id = strtok_r(NULL, ",", &save);
        const char *key = strtok_r(NULL, ",", &save);
        if (!name || !version || !ip || !id || !tr_tuya_key_from_hex(key, d->dev.key)) {
            ESP_LOGW(TAG, "skipped a device: bad entry in TUYA_DEVICES");
            continue;
        }
        copy(d->name, sizeof(d->name), name);
        copy(d->ip, sizeof(d->ip), ip);
        copy(d->id, sizeof(d->id), id);
        d->dev.id = d->id;
        d->dev.ip = d->ip;
        d->dev.version = atoi(version);
        bool ok = true;
        for (char *spec = strtok_r(NULL, ",", &save); spec; spec = strtok_r(NULL, ",", &save)) {
            ok = parse_switch(d, spec) && ok;
        }
        if (!ok || d->nsw == 0) {
            ESP_LOGW(TAG, "skipped %s: bad switch list", d->name);
            continue;
        }
        d->err = TR_TUYA_ERR_IO; // not read yet
        ndev++;
    }
}

// The robot is on the network that the ":ap" outlet feeds.
static bool on_guarded_wifi(void)
{
    wifi_ap_record_t ap;
    return CONFIG_HOME_AP_SSID[0] && esp_wifi_sta_get_ap_info(&ap) == ESP_OK &&
           strcmp((const char *)ap.ssid, CONFIG_HOME_AP_SSID) == 0;
}

// Read one device (tr_tuya_status) and keep what it says.
static void refresh(int i)
{
    tr_tuya_dp_t dps[24];
    const int n = tr_tuya_status(&devs[i].dev, dps, 24);
    xSemaphoreTake(mu, portMAX_DELAY);
    home_dev_t *d = &devs[i];
    d->err = n < 0 ? n : 0;
    for (int k = 0; k < d->nsw; k++) {
        d->sw[k].on = n < 0 ? -1 : tr_tuya_value(dps, n, d->sw[k].dp);
    }
    d->watts10 = n < 0 ? -1 : tr_tuya_value(dps, n, 19);
    xSemaphoreGive(mu);
    if (n < 0) {
        ESP_LOGW(TAG, "%s: %s", d->name, tr_tuya_err_text(n));
    }
    dirty = true;
}

static int set(int i, int k, bool on)
{
    const int r = tr_tuya_set(&devs[i].dev, devs[i].sw[k].dp, on);
    ESP_LOGI(TAG, "%s %s %s: %s", devs[i].name, devs[i].sw[k].name, on ? "on" : "off",
             tr_tuya_err_text(r));
    refresh(i);
    return r;
}

static void status_text(char *out, size_t size);

// True once the robot has an address on the home network.
static bool have_ip(void)
{
    esp_netif_ip_info_t ip;
    esp_netif_t *sta = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    return sta && esp_netif_get_ip_info(sta, &ip) == ESP_OK && ip.ip.addr != 0;
}

static void worker_task(void *arg)
{
    // The first read waits for Wi-Fi: before it, every device looks off.
    while (!have_ip()) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    for (int i = 0; i < ndev; i++) {
        refresh(i);
    }
    static char text[512];
    status_text(text, sizeof(text));
    ESP_LOGI(TAG, "first read:\n%s", text);
    job_t job;
    for (;;) {
        if (xQueueReceive(jobs, &job,
                          screen_open ? pdMS_TO_TICKS(REFRESH_US / 1000) : portMAX_DELAY) !=
            pdTRUE) {
            job.dev = -1; // the screen is open: fresh numbers
        }
        if (job.dev < 0) {
            for (int i = 0; i < ndev; i++) {
                refresh(i);
            }
        } else {
            set(job.dev, job.sw, job.on);
        }
    }
}

esp_err_t home_start(void)
{
    parse_devices();
    ESP_LOGI(TAG, "%d devices", ndev);
    if (ndev == 0) {
        return ESP_ERR_NOT_FOUND;
    }
    mu = xSemaphoreCreateMutex();
    jobs = xQueueCreate(8, sizeof(job_t));
    // Its stack in PSRAM: the echo canceller needs the internal RAM.
    return xTaskCreateWithCaps(worker_task, "home", 6144, NULL, 3, NULL, MALLOC_CAP_SPIRAM) ==
                   pdPASS
               ? ESP_OK
               : ESP_FAIL;
}

int home_count(void)
{
    return ndev;
}

// --- Names, for the voice and the web -------------------------------------

// Lower case, no spaces, and Spanish accents dropped (UTF-8): "Salón" and
// "salon" match, so do "MacBook Air" and "macbookair".
static void fold(char *out, size_t size, const char *in)
{
    size_t n = 0;
    for (const unsigned char *p = (const unsigned char *)in; *p && n + 1 < size; p++) {
        unsigned char ch = *p;
        if (ch == 0xC3 && p[1]) {
            const unsigned char x = *++p | 0x20; // lower case of the Latin-1 letter
            ch = x == 0xA1                ? 'a'
                 : x == 0xA9              ? 'e'
                 : x == 0xAD              ? 'i'
                 : x == 0xB3              ? 'o'
                 : x == 0xBA || x == 0xBC ? 'u'
                 : x == 0xB1              ? 'n'
                                          : '?';
        }
        if (ch != ' ' && ch != '_' && ch != '-') {
            out[n++] = (char)tolower(ch);
        }
    }
    out[n] = 0;
}

static bool same_name(const char *a, const char *b)
{
    char x[32], y[32];
    fold(x, sizeof(x), a);
    fold(y, sizeof(y), b);
    return x[0] && strcmp(x, y) == 0;
}

static int find_dev(const char *name)
{
    for (int i = 0; i < ndev; i++) {
        if (same_name(devs[i].name, name)) {
            return i;
        }
    }
    return -1;
}

static int find_sw(int i, const char *name)
{
    if (!name || !name[0]) {
        return devs[i].nsw == 2 ? 0 : -1; // a plug: its main outlet
    }
    for (int k = 0; k < devs[i].nsw; k++) {
        if (same_name(devs[i].sw[k].name, name) || same_name(devs[i].sw[k].label, name)) {
            return k;
        }
    }
    return -1;
}

// Everything, as short text: for the voice and GET /home.
static void status_text(char *out, size_t size)
{
    size_t n = 0;
    xSemaphoreTake(mu, portMAX_DELAY);
    for (int i = 0; i < ndev && n < size; i++) {
        const home_dev_t *d = &devs[i];
        n += snprintf(out + n, size - n, "%s:", d->name);
        if (d->err) {
            n += snprintf(out + n, size - n, " not answering (%s).\n", tr_tuya_err_text(d->err));
            continue;
        }
        for (int k = 0; k < d->nsw && n < size; k++) {
            n += snprintf(out + n, size - n, " %s %s,", d->sw[k].name,
                          d->sw[k].on == 1   ? "on"
                          : d->sw[k].on == 0 ? "off"
                                             : "unknown");
        }
        if (n < size) {
            if (d->watts10 < 0) {
                n += snprintf(out + n, size - n, " power unknown.\n");
            } else {
                n += snprintf(out + n, size - n, " %d.%d W.\n", d->watts10 / 10, d->watts10 % 10);
            }
        }
    }
    xSemaphoreGive(mu);
}

// Switch by name, with the access point check. Writes the result to out.
// Returns 0, or 1 when it needs a confirmation, or an error (< 0).
static int set_by_name(const char *dev, const char *sw, bool on, bool confirmed, char *out,
                       size_t size)
{
    const int i = find_dev(dev);
    if (i < 0) {
        int n = snprintf(out, size, "error: no device \"%s\". Devices:", dev);
        for (int j = 0; j < ndev; j++) {
            n += snprintf(out + n, size - n, " %s", devs[j].name);
        }
        return TR_TUYA_ERR_ARG;
    }
    const int k = find_sw(i, sw);
    if (k < 0) {
        int n = snprintf(out, size, "error: %s has no switch \"%s\". Switches:", devs[i].name, sw);
        for (int j = 0; j < devs[i].nsw; j++) {
            n += snprintf(out + n, size - n, " %s,", devs[i].sw[j].name);
        }
        return TR_TUYA_ERR_ARG;
    }
    if (!on && devs[i].sw[k].ap && !confirmed && on_guarded_wifi()) {
        snprintf(out, size,
                 "needs_confirmation: %s feeds the Wi-Fi access point the robot is on (%s). "
                 "Turning it off "
                 "cuts the robot off. Ask the person to confirm, and only after a clear yes call "
                 "again with "
                 "confirmed=true.",
                 devs[i].sw[k].name, CONFIG_HOME_AP_SSID);
        return 1;
    }
    const int r = set(i, k, on);
    if (r == 0) {
        snprintf(out, size, "done: %s %s is %s now.", devs[i].name, devs[i].sw[k].name,
                 on ? "on" : "off");
    } else {
        snprintf(out, size, "error: %s did not answer (%s).", devs[i].name, tr_tuya_err_text(r));
    }
    return r;
}

// --- The voice -----------------------------------------------------------------

static void arg_str(const char *json, const char *key, char *out, size_t size)
{
    char pat[24];
    snprintf(pat, sizeof(pat), "\"%s\"", key);
    out[0] = 0;
    const char *p = strstr(json, pat);
    if (p && (p = strchr(p + strlen(pat), '"'))) {
        size_t n = 0;
        for (p++; *p && *p != '"' && n + 1 < size; p++) {
            out[n++] = *p;
        }
        out[n] = 0;
    }
}

static bool arg_true(const char *json, const char *key)
{
    char pat[24];
    snprintf(pat, sizeof(pat), "\"%s\"", key);
    const char *p = strstr(json, pat);
    if (!p || !(p = strchr(p + strlen(pat), ':'))) {
        return false;
    }
    while (*++p == ' ') {
    }
    return strncmp(p, "true", 4) == 0;
}

void home_tool(const char *name, const char *args, char *out, size_t size)
{
    if (strcmp(name, "get_home_status") == 0) {
        for (int i = 0; i < ndev; i++) {
            refresh(i);
        }
        status_text(out, size);
        return;
    }
    if (strcmp(name, "set_switch") == 0) {
        char dev[32], sw[32];
        arg_str(args, "device", dev, sizeof(dev));
        arg_str(args, "switch", sw, sizeof(sw));
        set_by_name(dev, sw, arg_true(args, "on"), arg_true(args, "confirmed"), out, size);
        return;
    }
    snprintf(out, size, "error: no tool \"%s\".", name);
}

// --- The web ---------------------------------------------------------------------

esp_err_t home_get(httpd_req_t *req)
{
    static char text[768];
    status_text(text, sizeof(text));
    return httpd_resp_sendstr(req,
                              ndev ? text : "no devices (TUYA_DEVICES was empty at build time)\n");
}

esp_err_t home_post(httpd_req_t *req)
{
    if (!tr_ota_token_ok(req)) {
        return ESP_OK;
    }
    char query[128], dev[32] = "", sw[32] = "", on[4] = "", confirm[4] = "";
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        httpd_query_key_value(query, "dev", dev, sizeof(dev));
        httpd_query_key_value(query, "sw", sw, sizeof(sw));
        httpd_query_key_value(query, "on", on, sizeof(on));
        httpd_query_key_value(query, "confirm", confirm, sizeof(confirm));
    }
    for (char *p = sw; *p; p++) {
        *p = *p == '+' ? ' ' : *p; // "MacBook+Air"
    }
    if (!dev[0] || !on[0]) {
        tr_ota_send_error(req, "400 Bad Request", "use dev=..&sw=..&on=0|1\n");
        return ESP_OK;
    }
    static char text[256];
    const int r = set_by_name(dev, sw, on[0] == '1', confirm[0] == '1', text, sizeof(text) - 1);
    strcat(text, "\n");
    if (r == 1) {
        tr_ota_send_error(req, "409 Conflict", text);
        return ESP_OK;
    }
    return httpd_resp_sendstr(req, text);
}

// --- The screen ------------------------------------------------------------------

void home_open(void)
{
    screen_open = true;
    ask_dev = -1;
    dirty = true;
    const job_t job = {.dev = -1};
    xQueueSend(jobs, &job, 0);
}

void home_close(void)
{
    screen_open = false;
}

bool home_dirty(void)
{
    if (ask_dev >= 0 && esp_timer_get_time() > ask_until) {
        ask_dev = -1; // no second tap in time
        dirty = true;
    }
    const bool d = dirty;
    dirty = false;
    return d;
}

static int button_w(const home_dev_t *d)
{
    return (TR_LCD_W - 8 - (d->nsw - 1) * 4) / d->nsw;
}

static void text_center(int x, int w, int y, uint16_t fg, uint16_t bg, const char *text)
{
    tr_lcd_text(x + (w - 12 * (int)strlen(text)) / 2, y, 2, fg, bg, text);
}

void home_draw(void)
{
    tr_lcd_fill(BG);
    tr_lcd_fill_rect(0, 0, TR_LCD_W, BAR_H, TITLE_BG);
    tr_lcd_text(8, 6, 2, FG, TITLE_BG, "Home");
    tr_lcd_fill_rect(TR_LCD_W - 56, 0, 56, BAR_H, EXIT_BG);
    tr_lcd_text(TR_LCD_W - 34, 6, 2, FG, EXIT_BG, "X");
    xSemaphoreTake(mu, portMAX_DELAY);
    for (int i = 0; i < ndev; i++) {
        const home_dev_t *d = &devs[i];
        const int y0 = TOP + i * BLOCK_H;
        char watts[16];
        tr_lcd_text(8, y0 + 2, 2, FG, BG, d->name);
        if (d->err) {
            snprintf(watts, sizeof(watts), "no answer");
        } else {
            if (d->watts10 < 0) {
                snprintf(watts, sizeof(watts), "-- W");
            } else {
                snprintf(watts, sizeof(watts), "%d.%d W", d->watts10 / 10, d->watts10 % 10);
            }
        }
        tr_lcd_text(TR_LCD_W - 8 - 12 * (int)strlen(watts), y0 + 2, 2,
                    d->err ? TR_SCREEN_RED : TR_SCREEN_YELLOW, BG, watts);
        const int w = button_w(d);
        for (int k = 0; k < d->nsw; k++) {
            const sw_t *s = &d->sw[k];
            const int x = 4 + k * (w + 4);
            const bool ask = i == ask_dev && k == ask_sw;
            const uint16_t bg = ask ? ASK_BG : s->on == 1 ? ON_BG : s->on == 0 ? OFF_BG : BUSY_BG;
            tr_lcd_fill_rect(x, y0 + BTN_Y, w, BTN_H, bg);
            text_center(x, w, y0 + BTN_Y + 12, ask ? BG : FG, bg, ask ? "Sure?" : s->label);
        }
    }
    xSemaphoreGive(mu);
}

bool home_tap(int x, int y)
{
    if (tr_screen_on_bar(y)) {
        home_close();
        return false;
    }
    const int i = (y - TOP) / BLOCK_H, dy = (y - TOP) % BLOCK_H;
    if (y < TOP || i >= ndev || dy < BTN_Y - 4) {
        return true; // between the buttons
    }
    home_dev_t *d = &devs[i];
    const int w = button_w(d), k = (x - 4) / (w + 4);
    if (x < 4 || k >= d->nsw) {
        return true;
    }
    xSemaphoreTake(mu, portMAX_DELAY);
    const bool on = d->sw[k].on != 1;
    const bool asked = i == ask_dev && k == ask_sw && esp_timer_get_time() < ask_until;
    if (!on && d->sw[k].ap && !asked && on_guarded_wifi()) {
        ask_dev = i; // first tap: ask
        ask_sw = k;
        ask_until = esp_timer_get_time() + CONFIRM_US;
        xSemaphoreGive(mu);
        dirty = true;
        return true;
    }
    ask_dev = -1;
    d->sw[k].on = -1; // dark until the device says
    xSemaphoreGive(mu);
    const job_t job = {.dev = i, .sw = k, .on = on};
    xQueueSend(jobs, &job, 0);
    dirty = true;
    return true;
}
