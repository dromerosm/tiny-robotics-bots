// api: the bridge and the plugs as JSON, and a page that shows them. See api.h.
//
// Everything under /api/ needs the token (header X-Token), like /home: the
// plugs, their addresses and what is on say a lot about a home. /panel is
// only the page, with no data in it. It asks for the token once and keeps it
// in the browser.
//
// A browser cannot send X-Token to another site without asking first (a CORS
// preflight), and the bridge answers no preflight. So a page on another site
// cannot switch the plugs through a visitor's browser.
#include "api.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "bridge.h"
#include "cloud.h"
#include "energy.h"
#include "esp_app_desc.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "info.h"
#include "json.h"
#include "plugs.h"
#include "tr_ota.h"

extern const char panel_start[] asm("_binary_panel_html_start");

// GET /api/status: the bridge, HomeKit and the bridge.
static esp_err_t status_get(httpd_req_t *req)
{
    if (!tr_ota_token_ok(req)) {
        return ESP_OK;
    }
    static char text[640];
    char clock[24];
    jw_t w;
    jw_begin(&w, req);
    info_poll(); // the clients now; starts a search for Apple TVs now and then
    energy_clock_text(clock, sizeof(clock));
    jw_raw(&w, "{\"firmware\":");
    jw_str(&w, esp_app_get_description()->version);
    jw_fmt(&w, ",\"uptime_s\":%lld,\"heap_free\":%u,\"heap_min\":%u,\"clock\":",
           esp_timer_get_time() / 1000000, (unsigned)esp_get_free_heap_size(),
           (unsigned)esp_get_minimum_free_heap_size());
    jw_str(&w, clock[0] ? clock : NULL);
    jw_raw(&w, ",\"cloud\":");
    cloud_json(&w);
    jw_fmt(&w, ",\"homekit\":{\"paired\":%d,\"network\":", bridge_paired());
    info_json(&w);
    // The bridge's own status, the same lines as GET /.
    jw_raw(&w, "},\"robot\":[");
    tr_ota_status_text(text, sizeof(text));
    bool first = true;
    for (char *save = NULL, *line = strtok_r(text, "\n", &save); line;
         line = strtok_r(NULL, "\n", &save)) {
        jw_raw(&w, first ? "" : ",");
        jw_str(&w, line);
        first = false;
    }
    jw_raw(&w, "]}");
    return jw_end(&w);
}

// GET /api/plugs: every device, its outlets and its last reading.
static esp_err_t plugs_get(httpd_req_t *req)
{
    if (!tr_ota_token_ok(req)) {
        return ESP_OK;
    }
    jw_t w;
    jw_begin(&w, req);
    jw_fmt(&w, "{\"reads\":%u,\"devices\":[", plugs_reads());
    for (int i = 0; i < plugs_count(); i++) {
        char ip[16];
        int w10, v10, ma;
        plugs_dev_ip(i, ip, sizeof(ip));
        plugs_power(i, &w10, &v10, &ma);
        jw_raw(&w, i ? ",{\"name\":" : "{\"name\":");
        jw_str(&w, plugs_dev_name(i));
        jw_raw(&w, ",\"ip\":");
        jw_str(&w, ip);
        jw_raw(&w, ",\"error\":");
        jw_str(&w, plugs_error(i));
        jw_raw(&w, ",\"watts\":");
        jw_tenths(&w, w10);
        jw_raw(&w, ",\"volts\":");
        jw_tenths(&w, v10);
        jw_raw(&w, ",\"milliamps\":");
        if (ma < 0) {
            jw_raw(&w, "null");
        } else {
            jw_fmt(&w, "%d", ma);
        }
        jw_raw(&w, ",\"outlets\":[");
        for (int k = 0; k < plugs_switches(i); k++) {
            const int on = plugs_state(i, k);
            jw_raw(&w, k ? ",{\"name\":" : "{\"name\":");
            jw_str(&w, plugs_sw_name(i, k));
            jw_fmt(&w, ",\"on\":%s,\"ap\":%s}",
                   on < 0 ? "null"
                   : on   ? "true"
                          : "false",
                   plugs_sw_ap(i, k) ? "true" : "false");
        }
        jw_raw(&w, "]}");
    }
    jw_raw(&w, "]}");
    return jw_end(&w);
}

// The value of key when it is exactly one digit, or -1. Nothing else passes:
// atoi() would read "%00" or "1x" as a number.
static int query_digit(const char *query, const char *key)
{
    char value[4];
    if (httpd_query_key_value(query, key, value, sizeof(value)) != ESP_OK || value[0] < '0' ||
        value[0] > '9' || value[1]) {
        return -1;
    }
    return value[0] - '0';
}

// POST /api/plugs?dev=0&sw=1&on=1: switch an outlet (on=0 off, on=1 on).
// It answers 202 when the order is queued: GET /api/plugs shows the result
// once "reads" has grown.
static esp_err_t plugs_post(httpd_req_t *req)
{
    if (!tr_ota_token_ok(req)) {
        return ESP_OK;
    }
    char query[48];
    int dev = -1, sw = -1, on = -1;
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        dev = query_digit(query, "dev");
        sw = query_digit(query, "sw");
        on = query_digit(query, "on");
    }
    if (dev < 0 || dev >= plugs_count() || sw < 0 || sw >= plugs_switches(dev) || on < 0 ||
        on > 1) {
        tr_ota_send_error(req, "400 Bad Request", "{\"error\":\"dev, sw and on=0|1\"}\n");
        return ESP_OK;
    }
    if (!plugs_set(dev, sw, on == 1)) {
        tr_ota_send_error(req, "409 Conflict",
                          "{\"error\":\"refused: the access point outlet, or a full queue\"}\n");
        return ESP_OK;
    }
    httpd_resp_set_status(req, "202 Accepted");
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"queued\":true}\n");
}

// GET /api/energy: see energy.c.
static esp_err_t energy_get(httpd_req_t *req)
{
    if (!tr_ota_token_ok(req)) {
        return ESP_OK;
    }
    jw_t w;
    jw_begin(&w, req);
    energy_json(&w);
    return jw_end(&w);
}

static esp_err_t panel_get(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Content-Security-Policy",
                       "default-src 'self'; script-src 'unsafe-inline'; style-src "
                       "'unsafe-inline'; frame-ancestors 'none'");
    return httpd_resp_sendstr(req, panel_start);
}

esp_err_t api_start(void)
{
    const httpd_uri_t uris[] = {
        {.uri = "/api/status", .method = HTTP_GET, .handler = status_get},
        {.uri = "/api/plugs", .method = HTTP_GET, .handler = plugs_get},
        {.uri = "/api/plugs", .method = HTTP_POST, .handler = plugs_post},
        {.uri = "/api/energy", .method = HTTP_GET, .handler = energy_get},
        {.uri = "/panel", .method = HTTP_GET, .handler = panel_get},
    };
    for (size_t i = 0; i < sizeof(uris) / sizeof(uris[0]); i++) {
        esp_err_t err = tr_ota_add_uri(&uris[i]);
        if (err != ESP_OK) {
            return err;
        }
    }
    return ESP_OK;
}
