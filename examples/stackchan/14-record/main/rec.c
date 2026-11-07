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
#include "rec.h"

#include <stdio.h>
#include <string.h>
#include "esp_crt_bundle.h"
#include "esp_heap_caps.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "talk.h"
#include "tr_ota.h"

static const char *TAG = "rec";

#define RATE       16000
#define PIECE      (RATE * 10) // samples per piece: 10 s, 320 KB
#define POOL       6           // pieces that can wait for the network: 60 s
#define TAKE_MS    100
#define MAX_REC_US (3 * 3600 * 1000000LL) // a recording stops after 3 hours
#define MAX_WAIT_S 30

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
static int lost; // samples dropped since the last skipped piece number
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

// One request to the Worker. Returns the HTTP status, or -1.
static int request(esp_http_client_handle_t c, const char *url, esp_http_client_method_t method,
                   const void *body, int len, char *out, int out_size)
{
    esp_http_client_set_url(c, url);
    esp_http_client_set_method(c, method);
    if (esp_http_client_open(c, len) != ESP_OK) {
        snprintf(last_error, sizeof(last_error), "connect failed");
        return -1;
    }
    if (len > 0 && esp_http_client_write(c, body, len) != len) {
        snprintf(last_error, sizeof(last_error), "write failed");
        esp_http_client_close(c);
        return -1;
    }
    if (esp_http_client_fetch_headers(c) < 0) {
        snprintf(last_error, sizeof(last_error), "no answer");
        esp_http_client_close(c);
        return -1;
    }
    const int status = esp_http_client_get_status_code(c);
    int got = 0;
    if (out) {
        got = esp_http_client_read_response(c, out, out_size - 1);
        out[got > 0 ? got : 0] = 0;
    } else {
        esp_http_client_flush_response(c, NULL);
    }
    if (status >= 300) {
        snprintf(last_error, sizeof(last_error), "HTTP %d at %s", status, url + strlen(url) - 12);
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
        .keep_alive_enable = true,
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

esp_err_t rec_get(httpd_req_t *req)
{
    static char text[512];
    const int secs = active ? (int)((esp_timer_get_time() - since) / 1000000) : 0;
    snprintf(text, sizeof(text),
             "recording: %s\nid: %s\nsource: %s\nseconds: %d\npieces sent: %d\n"
             "waiting to upload: %d\nretries: %d\nseconds dropped: %d\n"
             "last upload: %lld s ago\nlast error: %s\n",
             listening ? "on"
             : active  ? "stopped, still uploading"
                       : "off",
             id[0] ? id : "-", source == 2 ? "microphone a as it is" : "the cleaned sound", secs,
             sent, full_q ? (int)uxQueueMessagesWaiting(full_q) : 0, retries, dropped_s,
             last_ok ? (esp_timer_get_time() - last_ok) / 1000000 : -1LL,
             last_error[0] ? last_error : "-");
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
        if (httpd_query_key_value(query, "start", value, sizeof(value)) == ESP_OK) {
            rec_start();
        }
        if (httpd_query_key_value(query, "stop", value, sizeof(value)) == ESP_OK) {
            rec_stop();
        }
    }
    return rec_get(req);
}
