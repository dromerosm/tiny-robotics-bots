// cloud: see cloud.h.
//
// Once an hour, from minute 2 on, the bridge POSTs its /api/energy JSON (the
// last 48 hours and 62 days) to CONFIG_HOME_ENERGY_URL over HTTPS, with
// "Authorization: Bearer CONFIG_HOME_ENERGY_TOKEN". The Worker keeps the larger
// value of each slot, so a lost send needs no buffer: the next one has the same
// hours again. A failed send is tried again 10 minutes later.
//
// There is no PSRAM, and TLS takes about 40 KB while it runs. So the send runs
// in a task that is made for it and then deleted: its stack and the TLS memory
// are only used for the few seconds of the send.
#include "cloud.h"

#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "energy.h"
#include "esp_crt_bundle.h"
#include "esp_heap_caps.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

static const char *TAG = "cloud";

#define CHECK_US   (60 * 1000000LL)
#define RETRY_S    600
#define FROM_MIN   2     // minutes into the hour: the hour before is closed
#define STACK      8192
#define FIRST_BUF  8192

typedef struct {
    char *data;
    size_t len, cap;
} buf_t;

static volatile bool busy;     // a send task is running
static time_t sent_hour;       // the UTC hour of the last good send (0: none)
static time_t tried_at;        // when the last send started
// For GET /api/status. Written only by the send task.
static int last_http;          // the last HTTP status, or -1 for a network error
static int64_t last_try_s, last_ok_s; // uptime in seconds (0: never)
static unsigned sends, fails;
static unsigned heap_before, heap_low; // free heap before the last send, lowest since boot after it
static int last_len;

static esp_err_t buf_sink(void *ctx, const char *data, size_t n)
{
    buf_t *b = ctx;
    if (!data) {
        return ESP_OK;
    }
    if (b->len + n > b->cap) {
        size_t cap = b->cap ? b->cap : FIRST_BUF;
        while (cap < b->len + n) {
            cap *= 2;
        }
        char *bigger = realloc(b->data, cap);
        if (!bigger) {
            return ESP_ERR_NO_MEM;
        }
        b->data = bigger;
        b->cap = cap;
    }
    memcpy(b->data + b->len, data, n);
    b->len += n;
    return ESP_OK;
}

static void send_task(void *arg)
{
    const time_t hour = (time_t)(intptr_t)arg;
    buf_t b = {0};
    jw_t w;
    last_try_s = esp_timer_get_time() / 1000000;
    heap_before = (unsigned)esp_get_free_heap_size();

    jw_begin_sink(&w, buf_sink, &b);
    energy_json(&w);
    jw_end(&w);
    int status = -1;
    if (w.err != ESP_OK) {
        ESP_LOGW(TAG, "no memory for the JSON");
    } else {
        last_len = (int)b.len;
        const esp_http_client_config_t cfg = {
            .url = CONFIG_HOME_ENERGY_URL,
            .method = HTTP_METHOD_POST,
            .crt_bundle_attach = esp_crt_bundle_attach,
            .timeout_ms = 15000,
            .keep_alive_enable = false,
        };
        esp_http_client_handle_t c = esp_http_client_init(&cfg);
        if (c) {
            esp_http_client_set_header(c, "Content-Type", "application/json");
            esp_http_client_set_header(c, "Authorization", "Bearer " CONFIG_HOME_ENERGY_TOKEN);
            esp_http_client_set_post_field(c, b.data, (int)b.len);
            const esp_err_t err = esp_http_client_perform(c);
            if (err == ESP_OK) {
                status = esp_http_client_get_status_code(c);
            } else {
                ESP_LOGW(TAG, "send: %s", esp_err_to_name(err));
            }
            esp_http_client_cleanup(c);
        }
    }
    free(b.data);

    last_http = status;
    heap_low = (unsigned)esp_get_minimum_free_heap_size();
    if (status == 200) {
        sent_hour = hour;
        last_ok_s = esp_timer_get_time() / 1000000;
        sends++;
        ESP_LOGI(TAG, "sent %d bytes; heap %u before, %u lowest", last_len, heap_before, heap_low);
    } else {
        fails++;
        ESP_LOGW(TAG, "send failed (HTTP %d); again in %d s", status, RETRY_S);
    }
    busy = false;
    vTaskDelete(NULL);
}

static void check(void *arg)
{
    if (busy || !energy_clock_ok()) {
        return;
    }
    const time_t now = time(NULL);
    const time_t hour = now / 3600; // Spain's offsets are whole hours: the same hours
    if (hour == sent_hour || now % 3600 < FROM_MIN * 60 || (tried_at && now - tried_at < RETRY_S)) {
        return;
    }
    tried_at = now;
    busy = true;
    if (xTaskCreate(send_task, "cloud", STACK, (void *)(intptr_t)hour, 2, NULL) != pdPASS) {
        busy = false;
        fails++;
        ESP_LOGW(TAG, "no memory for the send task");
    }
}

esp_err_t cloud_start(void)
{
    if (!CONFIG_HOME_ENERGY_URL[0] || !CONFIG_HOME_ENERGY_TOKEN[0]) {
        ESP_LOGI(TAG, "no HOME_ENERGY_URL or token: the history stays on the bridge");
        return ESP_OK;
    }
    const esp_timer_create_args_t args = {.callback = check, .name = "cloud"};
    esp_timer_handle_t t;
    esp_err_t err = esp_timer_create(&args, &t);
    if (err == ESP_OK) {
        err = esp_timer_start_periodic(t, CHECK_US);
    }
    ESP_LOGI(TAG, "sends the energy to %s every hour", CONFIG_HOME_ENERGY_URL);
    return err;
}

void cloud_json(jw_t *w)
{
    if (!CONFIG_HOME_ENERGY_URL[0] || !CONFIG_HOME_ENERGY_TOKEN[0]) {
        jw_raw(w, "null");
        return;
    }
    const int64_t up = esp_timer_get_time() / 1000000;
    jw_raw(w, "{\"url\":");
    jw_str(w, CONFIG_HOME_ENERGY_URL);
    jw_fmt(w, ",\"sends\":%u,\"fails\":%u,\"last_http\":%d,\"last_bytes\":%d", sends, fails,
           last_try_s ? last_http : 0, last_len);
    jw_fmt(w, ",\"last_try_ago_s\":%lld,\"last_ok_ago_s\":%lld", last_try_s ? up - last_try_s : -1LL,
           last_ok_s ? up - last_ok_s : -1LL);
    jw_fmt(w, ",\"heap_before\":%u,\"heap_min_after\":%u}", heap_before, heap_low);
}
