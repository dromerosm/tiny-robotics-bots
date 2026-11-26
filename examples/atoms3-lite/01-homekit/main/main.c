// 01-homekit: an AtomS3 Lite as a HomeKit bridge for the Tuya plugs at home.
// The plugs show up in Apple's Home app, and an Apple TV as home hub gives
// them Siri, automations and control from outside the house.
//
// StackChan's 17-homekit without the screen, the touch and the servos.
// plugs.c talks to the plugs (tr_tuya), bridge.c is HomeKit, energy.c adds up
// the energy, api.c is the JSON API and /panel. atom.c is the board: the LED
// shows the state, and the button opens the pairing page.
//
// With no screen, the QR code to add the bridge in Home is on a page,
// GET /pair. The page shows it only for ATOM_PAIR_S seconds after a press on
// the button, and only while the bridge is not paired. So someone on the
// network cannot read the code: they would need to press the button too.
//
//   GET  /pair            the QR code and the setup code (after a press)
//   GET  /home            the bridge and the plugs, as text (token needed)
//   POST /home?unpair=1   forget every pairing and restart (token needed)
//   /api/... and /panel   the same and more as JSON, and a page (api.h)
#include <stdarg.h>
#include <stdio.h>
#include "api.h"
#include "atom.h"
#include "bridge.h"
#include "cloud.h"
#include "energy.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "info.h"
#include "plugs.h"
#include "qrcode.h"
#include "tr_ota.h"

static const char *TAG = "main";

// The plugs, their addresses and what is on say a lot about a home, so this
// needs the token, like everything else here that is not tr_ota's.
static esp_err_t home_get(httpd_req_t *req)
{
    if (!tr_ota_token_ok(req)) {
        return ESP_OK;
    }
    static char text[640];
    const int n = snprintf(text, sizeof(text), "homekit: %d paired\n", bridge_paired());
    plugs_text(text + n, sizeof(text) - (size_t)n);
    return httpd_resp_sendstr(req, text);
}

static esp_err_t home_post(httpd_req_t *req)
{
    char query[32], value[4];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK &&
        httpd_query_key_value(query, "unpair", value, sizeof(value)) == ESP_OK) {
        if (!tr_ota_token_ok(req)) {
            return ESP_OK;
        }
        bridge_unpair();
        return httpd_resp_sendstr(req, "pairings forgotten, restarting\n");
    }
    return home_get(req);
}

// The QR code as an SVG: one path, with a run of dark modules in a row as one
// rectangle. Sent in chunks, so no buffer holds the whole picture.
typedef struct {
    httpd_req_t *req;
    char buf[512];
    size_t len;
} chunks_t;

static void chunk_flush(chunks_t *c)
{
    if (c->len) {
        httpd_resp_send_chunk(c->req, c->buf, (ssize_t)c->len);
        c->len = 0;
    }
}

// Add text to the buffer. Each piece is short (under 200 bytes).
static void chunk(chunks_t *c, const char *fmt, ...)
{
    if (c->len + 200 > sizeof(c->buf)) {
        chunk_flush(c);
    }
    va_list args;
    va_start(args, fmt);
    c->len += (size_t)vsnprintf(c->buf + c->len, sizeof(c->buf) - c->len, fmt, args);
    va_end(args);
}

static void qr_svg(esp_qrcode_handle_t qr, void *arg)
{
    chunks_t *c = arg;
    const int size = esp_qrcode_get_size(qr), side = size + 4;
    // Two modules of white border: a phone needs it to read the code.
    chunk(c,
          "<svg xmlns='http://www.w3.org/2000/svg' viewBox='-2 -2 %d %d' width='264' height='264'",
          side, side);
    chunk(c,
          " shape-rendering='crispEdges'><rect x='-2' y='-2' width='%d' height='%d' fill='#fff'/>"
          "<path d='",
          side, side);
    for (int y = 0; y < size; y++) {
        for (int x = 0; x < size;) {
            int run = 0;
            while (x + run < size && esp_qrcode_get_module(qr, x + run, y)) {
                run++;
            }
            if (run) {
                chunk(c, "M%d %dh%dv1H%dz", x, y, run, x);
            }
            x += run ? run : 1;
        }
    }
    chunk(c, "'/></svg>");
}

#define PAGE_HEAD                                                                                  \
    "<!doctype html><meta charset=utf-8><meta name=viewport content='width=device-width'>"         \
    "<title>Pair the bridge</title><style>body{font:16px system-ui;margin:2em;text-align:center}"  \
    "code{font-size:1.6em}</style><h1>Atom Bridge</h1>"

// GET /pair: the QR code, only for a short time after a press on the button,
// and only while nothing is paired. No token: the press is the proof that the
// person is next to the bridge.
static esp_err_t pair_get(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_set_hdr(req, "Content-Security-Policy",
                       "default-src 'none'; style-src 'unsafe-inline'");
    const int paired = bridge_paired();
    if (paired) {
        char page[384];
        snprintf(page, sizeof(page),
                 PAGE_HEAD "<p>Paired with %d controller%s. To pair it again, forget the pairings "
                           "first (<code>POST /home?unpair=1</code> with the token).",
                 paired, paired == 1 ? "" : "s");
        return httpd_resp_sendstr(req, page);
    }
    if (!atom_pair_open()) {
        httpd_resp_set_status(req, "403 Forbidden");
        return httpd_resp_sendstr(req, PAGE_HEAD "<p>Press the button on the Atom once (the LED "
                                                 "blinks blue), then load this page again. It "
                                                 "stays open for two minutes.");
    }
    static chunks_t c;
    c.req = req;
    c.len = 0;
    httpd_resp_send_chunk(req, PAGE_HEAD "<p>", HTTPD_RESP_USE_STRLEN);
    esp_qrcode_config_t cfg = ESP_QRCODE_CONFIG_DEFAULT();
    cfg.display_func_with_cb = qr_svg;
    cfg.user_data = &c;
    if (esp_qrcode_generate(&cfg, bridge_qr_text()) != ESP_OK) {
        ESP_LOGW(TAG, "no QR code");
    }
    chunk_flush(&c);
    char tail[256];
    snprintf(tail, sizeof(tail),
             "<p><code>%s</code><p>In the Home app: <b>+</b>, <b>Add Accessory</b>, then scan the "
             "code. Or tap <b>More options</b> and type it.",
             bridge_code());
    httpd_resp_send_chunk(req, tail, HTTPD_RESP_USE_STRLEN);
    return httpd_resp_send_chunk(req, NULL, 0);
}

// A device was read: tell Home, and add up its energy.
static void fresh(int dev)
{
    bridge_fresh(dev);
    energy_fresh(dev);
}

void app_main(void)
{
    ESP_ERROR_CHECK(tr_ota_start());
    const httpd_uri_t uris[] = {
        {.uri = "/home", .method = HTTP_GET, .handler = home_get},
        {.uri = "/home", .method = HTTP_POST, .handler = home_post},
        {.uri = "/pair", .method = HTTP_GET, .handler = pair_get},
    };
    for (size_t i = 0; i < sizeof(uris) / sizeof(uris[0]); i++) {
        ESP_ERROR_CHECK(tr_ota_add_uri(&uris[i]));
    }
    info_init();
    if (plugs_start(fresh) != ESP_OK) {
        ESP_LOGW(TAG, "no plugs (set TUYA_DEVICES in the env file)");
    }
    energy_start();
    if (cloud_start() != ESP_OK) {
        ESP_LOGW(TAG, "no energy sends to the cloud");
    }
    ESP_ERROR_CHECK(api_start());
    if (bridge_start() != ESP_OK) {
        ESP_LOGE(TAG, "HomeKit did not start");
    }
    if (atom_start() != ESP_OK) {
        ESP_LOGW(TAG, "no LED or button");
    }
}
