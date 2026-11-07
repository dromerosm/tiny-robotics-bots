// rec: see rec.h.
//
// The sound comes from talk.c, which runs the microphones and the echo
// canceller all the time. With no talk the speaker plays silence, so the
// cleaned sound is both microphones averaged, at 16 kHz. Two tasks:
// - take (small stack, made at the start of a recording and gone at its
//   end): every 100 ms it moves the sound into a 10 s piece in PSRAM. When a
//   piece is full it goes to the upload queue.
// - upload: asks our Worker for a recording id, then PUTs each piece,
//   keeping the connection open. A piece that fails is tried again, with a
//   longer wait each time (up to 30 s). TLS needs an 8 KB stack, and with
//   the echo canceller the biggest free block of internal RAM is about 7 KB.
//   So its stack is in PSRAM. Such a task can't delete itself, so it is
//   made once and waits for each recording.
//
// There are POOL pieces, so 60 s of sound can wait for the network. When
// all are waiting, the new sound is dropped, but its piece number is used
// anyway. The Worker puts silence where a piece never came, so the times
// after a gap stay right.
//
//   GET  /rec                         state and counts
//   POST /rec?start=1|stop=1          start or stop (token needed)
//   POST /rec?src=clean|mic           the cleaned sound (default), or
//                                     microphone a as it is, to compare
//   POST /rec?ps=0|1                  Wi-Fi power save off, or min (default)
#include "rec.h"

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include "esp_crt_bundle.h"
#include "esp_heap_caps.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "talk.h"
#include "tr_ota.h"

static const char *TAG = "rec";

#define RATE         16000
#define PIECE        (RATE * 10) // samples per piece: 10 s, 320 KB
#define POOL         6           // pieces that can wait for the network: 60 s
#define TAKE_MS      100
#define MAX_REC_US   (3 * 3600 * 1000000LL) // a recording stops after 3 hours
#define MAX_WAIT_S   30
#define STALL_MAX_MS 10000 // a write with no progress for this long fails

typedef struct {
    int16_t *s;
    int n;   // samples in it
    int seq; // piece number
} piece_t;

static piece_t pool[POOL];
static QueueHandle_t free_q, full_q; // indices into pool[]
static volatile bool listening, active;
static volatile int source = 1; // 1 the cleaned sound, 2 microphone a
static char id[32];
static volatile int sent, retries, dropped_s, next_seq;
static int lost;              // samples dropped since the last skipped piece number
static volatile bool failing; // the last request to the Worker failed

// Telemetry for GET /rec: how fast each piece went up, and the last
// failures, to see why uploads fall behind.
typedef struct {
    int t_s;       // seconds since the recording started
    char phase;    // 'o' open, 'w' write, 'h' headers, 's' HTTP status
    int done, len; // bytes written before it failed, of len
    int err;       // errno from the transport, or the HTTP status
    int ms;        // how long the request took
    int idle_ms;   // how long the connection waited since the last request
    bool reused;   // the connection was already open
    int rssi;      // Wi-Fi signal then
} fail_t;
#define FAILS 12
static fail_t fails[FAILS];
static int fail_n;                           // total failures this recording
static volatile int up_last_kbs, up_min_kbs; // KB/s of the last and the slowest piece
static int64_t up_bytes, up_ms;              // for the average
static int64_t last_req_end;
static bool conn_open;
static int stalls, stall_max_ms; // writes that had to wait for room in the TCP buffer
static volatile int64_t since, last_ok;
static char last_error[96];

static TaskHandle_t uploader;

static void upload_task(void *arg);
static void upload_one(void);

static esp_err_t pool_init(void)
{
    if (free_q) {
        return ESP_OK;
    }
    free_q = xQueueCreate(POOL, sizeof(int));
    full_q = xQueueCreate(POOL + 1, sizeof(int)); // + the end mark (-1)
    for (int i = 0; i < POOL; i++) {
        pool[i].s = heap_caps_malloc(PIECE * 2, MALLOC_CAP_SPIRAM);
        if (!pool[i].s) {
            return ESP_ERR_NO_MEM;
        }
        xQueueSend(free_q, &i, 0);
    }
    if (xTaskCreatePinnedToCoreWithCaps(upload_task, "rec up", 8192, NULL, 4, &uploader,
                                        tskNO_AFFINITY, MALLOC_CAP_SPIRAM) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

static void take_task(void *arg)
{
    talk_tap(source);
    int cur = -1;
    static int16_t scrap[RATE / 10 * 2]; // sound with no piece to go to
    TickType_t wake = xTaskGetTickCount();
    while (listening) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(TAKE_MS));
        if (esp_timer_get_time() - since > MAX_REC_US) {
            ESP_LOGI(TAG, "3 hours: stop");
            listening = false;
        }
        for (;;) {
            if (cur < 0) {
                if (xQueueReceive(free_q, &cur, 0) == pdTRUE) {
                    pool[cur].n = 0;
                    pool[cur].seq = next_seq++;
                } else { // all pieces wait for the network: drop this sound
                    lost += talk_tap_read(scrap, sizeof(scrap) / 2);
                    if (lost >= PIECE) { // a whole piece lost: skip its number
                        lost -= PIECE;
                        next_seq++;
                        dropped_s += PIECE / RATE;
                    }
                    break;
                }
            }
            piece_t *p = &pool[cur];
            const int got = talk_tap_read(p->s + p->n, PIECE - p->n);
            p->n += got;
            if (p->n == PIECE) {
                xQueueSend(full_q, &cur, portMAX_DELAY);
                cur = -1;
                continue;
            }
            break;
        }
    }
    talk_tap(0);
    if (cur >= 0 && pool[cur].n > 0) {
        xQueueSend(full_q, &cur, portMAX_DELAY);
    } else if (cur >= 0) {
        xQueueSend(free_q, &cur, 0);
    }
    const int end = -1;
    xQueueSend(full_q, &end, portMAX_DELAY);
    vTaskDelete(NULL);
}

static int wifi_rssi(void)
{
    wifi_ap_record_t ap;
    return esp_wifi_sta_get_ap_info(&ap) == ESP_OK ? ap.rssi : 0;
}

static void note_fail(esp_http_client_handle_t c, char phase, int done, int len, int err,
                      int64_t t0, int idle_ms, bool reused)
{
    fail_t *f = &fails[fail_n++ % FAILS];
    *f = (fail_t){
        .t_s = (int)((esp_timer_get_time() - since) / 1000000),
        .phase = phase,
        .done = done,
        .len = len,
        .err = err,
        .ms = (int)((esp_timer_get_time() - t0) / 1000),
        .idle_ms = idle_ms,
        .reused = reused,
        .rssi = wifi_rssi(),
    };
    ESP_LOGW(TAG, "fail %c at %d s: %d/%d bytes, err %d, %d ms, idle %d ms, %s, %d dBm", f->phase,
             f->t_s, f->done, f->len, f->err, f->ms, f->idle_ms, f->reused ? "reused" : "new",
             f->rssi);
    failing = true;
}

// One request to the Worker. Returns the HTTP status, or -1. The body goes
// in 8 KB writes, so a failure says how far it got.
static int request(esp_http_client_handle_t c, const char *url, esp_http_client_method_t method,
                   const void *body, int len, char *out, int out_size)
{
    const int64_t t0 = esp_timer_get_time();
    const int idle_ms = last_req_end ? (int)((t0 - last_req_end) / 1000) : -1;
    const bool reused = conn_open;
    esp_http_client_set_url(c, url);
    esp_http_client_set_method(c, method);
    if (esp_http_client_open(c, len) != ESP_OK) {
        snprintf(last_error, sizeof(last_error), "connect failed");
        note_fail(c, 'o', 0, len, esp_http_client_get_errno(c), t0, idle_ms, reused);
        conn_open = false;
        return -1;
    }
    conn_open = true;
    // When the TCP send buffer is full, mbedtls answers WANT_WRITE: esp-tls
    // gives up with nothing written, and esp_http_client_write returns <= 0.
    // That is not an error. Wait a little and write the same bytes again (as
    // mbedtls asks). Fail only after STALL_MAX_MS with no progress, or a real
    // socket error. Before this, nearly every piece failed once on a slower
    // network, and the face turned orange.
    int done = 0;
    int64_t stuck_since = 0;
    while (done < len) {
        const int chunk = len - done < 8192 ? len - done : 8192;
        const int w = esp_http_client_write(c, (const char *)body + done, chunk);
        if (w > 0) {
            if (stuck_since) {
                const int ms = (int)((esp_timer_get_time() - stuck_since) / 1000);
                stall_max_ms = ms > stall_max_ms ? ms : stall_max_ms;
                stuck_since = 0;
            }
            done += w;
            continue;
        }
        const int err = esp_http_client_get_errno(c);
        const int64_t now = esp_timer_get_time();
        if (!stuck_since) {
            stuck_since = now;
            stalls++;
        }
        if ((err != 0 && err != EAGAIN) || now - stuck_since > STALL_MAX_MS * 1000LL) {
            snprintf(last_error, sizeof(last_error), "write failed (%d, errno %d)", w, err);
            note_fail(c, 'w', done, len, err ? err : w, t0, idle_ms, reused);
            esp_http_client_close(c);
            conn_open = false;
            return -1;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    if (esp_http_client_fetch_headers(c) < 0) {
        snprintf(last_error, sizeof(last_error), "no answer");
        note_fail(c, 'h', done, len, esp_http_client_get_errno(c), t0, idle_ms, reused);
        esp_http_client_close(c);
        conn_open = false;
        return -1;
    }
    const int status = esp_http_client_get_status_code(c);
    static char why[48];
    if (!out && status >= 300) { // keep the Worker's answer, to see why
        out = why;
        out_size = sizeof(why);
    }
    int got = 0;
    if (out) {
        got = esp_http_client_read_response(c, out, out_size - 1);
        out[got > 0 ? got : 0] = 0;
    } else {
        esp_http_client_flush_response(c, NULL);
    }
    // A new connection for each request. The server closes an idle one after
    // about 6.5 s, and a piece comes every 10 s, so a kept one was nearly
    // always dead (ECONNRESET at the first write). A TLS handshake per piece
    // costs much less than that.
    esp_http_client_close(c);
    conn_open = false;
    last_req_end = esp_timer_get_time();
    if (status >= 300) {
        snprintf(last_error, sizeof(last_error), "HTTP %d: %.40s", status, out ? out : "");
        note_fail(c, 's', done, len, status, t0, idle_ms, reused);
        return status;
    }
    failing = false;
    const int ms = (int)((last_req_end - t0) / 1000);
    if (len > 0 && ms > 0) {
        up_last_kbs = len / ms; // bytes per ms = KB/s
        up_min_kbs = up_min_kbs && up_min_kbs < up_last_kbs ? up_min_kbs : up_last_kbs;
        up_bytes += len;
        up_ms += ms;
    }
    return status;
}

static void upload_task(void *arg)
{
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // rec_start()
        upload_one();
        active = false;
    }
}

static void upload_one(void)
{
    static char url[192], answer[96];
    const esp_http_client_config_t cfg = {
        .url = CONFIG_REC_URL,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .timeout_ms = 15000,
        .keep_alive_enable = false,
    };
    esp_http_client_handle_t c = esp_http_client_init(&cfg);
    esp_http_client_set_header(c, "X-Robot-Token", CONFIG_REC_TOKEN);
    esp_http_client_set_header(c, "Content-Type", "application/octet-stream");

    // A recording id from the Worker, tried until it comes or the
    // recording ends with nothing to send.
    id[0] = 0;
    for (int wait = 1, tries = 0; !id[0]; tries++) {
        if (request(c, CONFIG_REC_URL, HTTP_METHOD_POST, NULL, 0, answer, sizeof(answer)) == 201) {
            const char *v = strstr(answer, "\"id\":\"");
            if (v && sscanf(v + 6, "%31[^\"]", id) == 1) {
                break;
            }
        }
        if (!listening && tries >= 10) {
            break; // stopped, and still no network: give up
        }
        vTaskDelay(pdMS_TO_TICKS(wait * 1000));
        wait = wait < MAX_WAIT_S ? wait * 2 : MAX_WAIT_S;
    }
    ESP_LOGI(TAG, "recording %s", id[0] ? id : "(no id)");

    for (int idx;;) {
        xQueueReceive(full_q, &idx, portMAX_DELAY);
        if (idx < 0) {
            break; // the end mark
        }
        piece_t *p = &pool[idx];
        // The first retry goes at once: it is nearly always a connection the
        // server closed (after the POST for the id), and a new one works.
        for (int wait = 0, tries = 0; id[0] && tries < 20; tries++) {
            snprintf(url, sizeof(url), "%s/%s/%d", CONFIG_REC_URL, id, p->seq);
            if (request(c, url, HTTP_METHOD_PUT, p->s, p->n * 2, NULL, 0) == 200) {
                sent++;
                last_ok = esp_timer_get_time();
                break;
            }
            retries++;
            vTaskDelay(pdMS_TO_TICKS(wait * 1000));
            wait = wait == 0 ? 1 : wait < MAX_WAIT_S ? wait * 2 : MAX_WAIT_S;
        }
        xQueueSend(free_q, &idx, 0);
    }
    if (id[0]) {
        snprintf(url, sizeof(url), "%s/%s/end", CONFIG_REC_URL, id);
        request(c, url, HTTP_METHOD_POST, NULL, 0, NULL, 0);
    }
    esp_http_client_cleanup(c);
    conn_open = false;
    last_req_end = 0;
    ESP_LOGI(TAG, "done: %d pieces sent, %d retries, %d s dropped", sent, retries, dropped_s);
}

esp_err_t rec_start(void)
{
    if (active || talk_active()) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!CONFIG_REC_URL[0] || !CONFIG_REC_TOKEN[0]) {
        snprintf(last_error, sizeof(last_error), "no REC_URL or REC_TOKEN in the build");
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t err = talk_start(); // the microphones and the echo canceller
    if (err == ESP_OK) {
        err = pool_init();
    }
    if (err != ESP_OK) {
        snprintf(last_error, sizeof(last_error), "no audio: %s", esp_err_to_name(err));
        return err;
    }
    sent = retries = dropped_s = next_seq = lost = 0;
    failing = false;
    id[0] = 0;
    fail_n = 0;
    up_last_kbs = up_min_kbs = 0;
    up_bytes = up_ms = 0;
    stalls = stall_max_ms = 0;
    last_error[0] = 0;
    since = esp_timer_get_time();
    active = listening = true;
    if (xTaskCreatePinnedToCore(take_task, "rec take", 3072, NULL, 5, NULL, 0) != pdPASS) {
        snprintf(last_error, sizeof(last_error), "no RAM for the take task");
        active = listening = false;
        return ESP_ERR_NO_MEM;
    }
    xTaskNotifyGive(uploader);
    ESP_LOGI(TAG, "start");
    return ESP_OK;
}

void rec_stop(void)
{
    listening = false;
}

bool rec_active(void)
{
    return active;
}

bool rec_listening(void)
{
    return listening;
}

bool rec_behind(void)
{
    if (!listening) {
        return false;
    }
    const bool no_id = !id[0] && esp_timer_get_time() - since > 5000000LL; // the id takes 1 to 2 s
    const bool queued = full_q && uxQueueMessagesWaiting(full_q) >= 2;     // 20 s or more behind
    return failing || no_id || queued; // dropping also means 2 or more waiting
}

esp_err_t rec_get(httpd_req_t *req)
{
    static char text[1800];
    const int secs = active ? (int)((esp_timer_get_time() - since) / 1000000) : 0;
    snprintf(text, sizeof(text),
             "recording: %s\nuploads: %s\nid: %s\nsource: %s\nseconds: %d\npieces sent: %d\n"
             "waiting to upload: %d\nretries: %d\nseconds dropped: %d\n"
             "last upload: %lld s ago\nlast error: %s\n",
             listening ? "on"
             : active  ? "stopped, still uploading"
                       : "off",
             !listening     ? "-"
             : rec_behind() ? "behind (orange face)"
                            : "on time",
             id[0] ? id : "-", source == 2 ? "microphone a as it is" : "the cleaned sound", secs,
             sent, full_q ? (int)uxQueueMessagesWaiting(full_q) : 0, retries, dropped_s,
             last_ok ? (esp_timer_get_time() - last_ok) / 1000000 : -1LL,
             last_error[0] ? last_error : "-");
    wifi_ps_type_t ps = WIFI_PS_NONE;
    esp_wifi_get_ps(&ps);
    int n = strlen(text);
    n += snprintf(
        text + n, sizeof(text) - n,
        "Wi-Fi: %d dBm, power save %s\nupload speed: last %d KB/s, slowest %d KB/s, "
        "average %d KB/s (the sound needs 32)\nwaits for the TCP buffer: %d, longest %d ms\n"
        "failures: %d\n",
        wifi_rssi(),
        ps == WIFI_PS_NONE        ? "off"
        : ps == WIFI_PS_MIN_MODEM ? "min"
                                  : "max",
        up_last_kbs, up_min_kbs, up_ms ? (int)(up_bytes / up_ms) : 0, stalls, stall_max_ms, fail_n);
    for (int i = fail_n > FAILS ? fail_n - FAILS : 0; i < fail_n && n < (int)sizeof(text) - 120;
         i++) {
        const fail_t *f = &fails[i % FAILS];
        n += snprintf(
            text + n, sizeof(text) - n,
            "  at %d s: %s, %d of %d bytes, err %d, %d ms, idle %d ms, %s, %d dBm\n", f->t_s,
            f->phase == 'o'   ? "connect"
            : f->phase == 'w' ? "write"
            : f->phase == 'h' ? "no answer"
                              : "HTTP",
            f->done, f->len, f->err, f->ms, f->idle_ms, f->reused ? "reused" : "new", f->rssi);
    }
    return httpd_resp_sendstr(req, text);
}

esp_err_t rec_post(httpd_req_t *req)
{
    if (!tr_ota_token_ok(req)) {
        return ESP_OK;
    }
    char query[64], value[8];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        if (httpd_query_key_value(query, "src", value, sizeof(value)) == ESP_OK && !active) {
            source = strcmp(value, "mic") == 0 ? 2 : 1;
        }
        if (httpd_query_key_value(query, "ps", value, sizeof(value)) == ESP_OK) {
            esp_wifi_set_ps(strcmp(value, "0") == 0 ? WIFI_PS_NONE : WIFI_PS_MIN_MODEM);
        }
        if (httpd_query_key_value(query, "start", value, sizeof(value)) == ESP_OK) {
            rec_start();
        }
        if (httpd_query_key_value(query, "stop", value, sizeof(value)) == ESP_OK) {
            rec_stop();
        }
    }
    return rec_get(req);
}
