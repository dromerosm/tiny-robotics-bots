// talk: see talk.h.
//
// Four tasks:
// - audio (core 0): every frame (FRAME samples at 16 kHz) it plays the next
//   piece of the robot's voice (or silence) and reads the same amount from
//   the four ES7210 inputs. It passes both microphones and the reference
//   (what it played, delayed to line up with the microphones) to the echo
//   canceller;
// - aec (core 1): esp-sr's echo canceller (AEC) on both microphones, with
//   its non-linear stage for the echo that is left. The two cleaned
//   microphones are averaged into one sound;
// - send: during a talk, every SEND_MS it sends the cleaned sound to the
//   Realtime API at 24 kHz;
// - session: gets a session key from our key Worker, opens the WebSocket,
//   and closes it at the end. The WebSocket client's own task receives the
//   robot's voice (24 kHz), which goes to the play buffer at 16 kHz.
//
// Full duplex: the microphones stay open while the robot speaks. The server
// hears when the person starts talking (input_audio_buffer.speech_started)
// and stops the answer; the robot drops what is left in its play buffer.
//
// No AFE (esp-sr's whole audio front end) and no BSS (its two-microphone
// separation): together they needed more than all of core 1, and added
// about 100 ms. The AEC alone on both microphones takes about a third.
//
//   GET  /talk                 state, levels, counts, times
//   POST /talk?start=1|stop=1  open or end a talk (token needed)
//   POST /talk?test=lanes      a 1 kHz tone: the level and delay on each input
//   POST /talk?test=aec        6 s of noise bursts: how much echo is left
//   POST /talk?delay=N         reference delay in samples (the tone test
//                              sets it from what it measures)
//   POST /talk?send=aec|mic    send the cleaned sound, or microphone a as it is
//   POST /talk?volume=40       speaker volume, 0 to 100
#include "talk.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_aec.h"
#include "esp_crt_bundle.h"
#include "esp_heap_caps.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "esp_websocket_client.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "mbedtls/base64.h"
#include "sdkconfig.h"
#include "tr_board.h"
#include "tr_ota.h"

static const char *TAG = "talk";

#define RATE        16000
#define API_RATE    24000
#define RING        (RATE * 30)          // 30 s of the robot's voice
#define MIC_RING    (RATE * 2)           // 2 s of the cleaned sound, to send
#define MAX_TALK_US (5 * 60 * 1000000LL) // a talk ends after 5 minutes
#define SEND_MS     100                  // microphone sound per WebSocket message
#define RX_MAX      (256 * 1024)         // the biggest message we take
#define MAX_DELAY   2048                 // reference delay line, in samples
#define SLOTS       4                    // frames on their way to the AEC
#define VOLUME      40                   // at 70 the microphones clip
#define FILTER      4                    // AEC filter, in frames: 2 removed only 6 dB of echo

// The ES7210 inputs: 0 and 2 are the microphones (1 carries the amplifier's
// output, 3 is empty). As the reference, input 1 removed only about 8 dB
// of echo; what we played, lined up with the microphones, removed 23 to
// 35 dB. The delay is the time from our write to the sound in the
// microphones, a bit less so the reference comes first.
#define MIC_A 0
#define MIC_B 2
static volatile int ref_delay = 79; // measured by the tone test

static aec_handle_t *aec;
static int frame;                // samples per AEC frame
static QueueHandle_t to_aec;     // indices into slots[]
static int16_t *slots[SLOTS];    // mic a, mic b, reference: each FRAME samples
static bool started;
static volatile bool send_raw;   // send microphone a as it is, to compare

// The robot's voice, 16 kHz mono, in PSRAM. One writer (the WebSocket task),
// one reader (the audio task). The same kind of ring holds the cleaned
// sound (aec task to send task) and microphone a (audio task to send task).
typedef struct {
    int16_t *s;
    uint32_t size;
    volatile uint32_t in, out;
} ring_t;
static ring_t voice = {.size = RING}, clean = {.size = MIC_RING}, raw = {.size = MIC_RING};

static void ring_push(ring_t *r, const int16_t *s, int n)
{
    for (int i = 0; i < n; i++) {
        if (r->in - r->out >= r->size) {
            break; // full: drop the rest
        }
        r->s[r->in % r->size] = s[i];
        r->in++;
    }
}

static int ring_pop(ring_t *r, int16_t *s, int n)
{
    int got = 0;
    while (got < n && r->out != r->in) {
        s[got++] = r->s[r->out % r->size];
        r->out++;
    }
    return got;
}

static void ring_flush(ring_t *r)
{
    r->out = r->in;
}

// Linear resampling, for any ratio, in pieces. step = input rate / output rate.
typedef struct {
    float pos, step;
    int16_t prev;
} resampler_t;

static int resample(resampler_t *r, const int16_t *in, int n, int16_t *out)
{
    int k = 0;
    for (int i = 0; i < n; i++) {
        while (r->pos < 1.0f) {
            out[k++] = (int16_t)(r->prev + (in[i] - r->prev) * r->pos);
            r->pos += r->step;
        }
        r->pos -= 1.0f;
        r->prev = in[i];
    }
    return k;
}

// State and numbers for GET /talk.
typedef enum { T_OFF, T_OPENING, T_OPEN, T_CLOSING } tstate_t;
static volatile tstate_t state = T_OFF;
static volatile bool close_asked;
static volatile float level;                        // the robot's voice, 0..1
static volatile float rms_in[4], rms_play, rms_out; // over the last second
static volatile int frames10, aec_us, io_us;        // frames per 10 s, times per frame
static volatile int barge_ins, answers, deltas, sent_msgs, overruns;
static volatile int64_t talk_since, speech_stopped_at, first_audio_ms = -1;
static volatile int last_latency_ms = -1;
static char last_error[160];

// Tests (test=lanes and test=aec). Loudness per frame from the start: the
// four inputs, what we played and the cleaned sound.
typedef enum { TEST_NONE, TEST_TONE, TEST_BURSTS } test_t;
static volatile test_t test;
static volatile int test_pos; // samples since the test started
#define ENV 512
static uint16_t env_in[4][ENV], env_play[ENV], env_out[ENV];
static volatile int env_n, env_out_n;
static int t_onset[4], t_play_onset;

// Integer math in the sample loops: the ESP32-S3 has no hardware for double.
static uint16_t rms16(int64_t sq, int n)
{
    return (uint16_t)sqrtf((float)sq / n);
}

// ---------------------------------------------------------------------------
// Audio and echo cancelling

static void audio_task(void *arg)
{
    int16_t *play = heap_caps_malloc(frame * 2, MALLOC_CAP_SPIRAM);
    int16_t *in = heap_caps_malloc(frame * TR_AUDIO_INPUTS * 2, MALLOC_CAP_SPIRAM);
    int16_t *delay = heap_caps_calloc(MAX_DELAY, 2, MALLOC_CAP_SPIRAM);
    uint32_t delay_pos = 0;
    int slot = 0, n_sum = 0, count = 0;
    int64_t sum[4] = {0}, sum_play = 0, t_io = 0, window = esp_timer_get_time();
    for (;;) {
        // What to play: a test signal, the robot's voice, or silence.
        int got;
        if (test == TEST_TONE) { // 1 s quiet, then 1 s of 1 kHz
            for (int i = 0; i < frame; i++) {
                const int t = test_pos + i;
                play[i] = t >= RATE && t < 2 * RATE ? (int16_t)(8000 * sinf(2 * M_PI * 1000 * t / RATE)) : 0;
            }
            got = frame;
        } else if (test == TEST_BURSTS) { // 1 s quiet, then 0.5 s noise, 0.5 s quiet, x5
            for (int i = 0; i < frame; i++) {
                const int t = test_pos + i;
                const bool on = t >= RATE && ((t - RATE) / (RATE / 2)) % 2 == 0;
                play[i] = on ? (int16_t)((int)(esp_random() % 12001) - 6000) : 0;
            }
            got = frame;
        } else {
            got = ring_pop(&voice, play, frame);
        }
        if (got < frame) {
            memset(play + got, 0, (frame - got) * 2);
        }
        const int64_t t0 = esp_timer_get_time();
        tr_audio_write(play, frame);
        tr_audio_read(in, frame);
        t_io += esp_timer_get_time() - t0;

        // Both microphones and the reference, one after the other (the AEC
        // takes each channel as a block).
        int16_t *s = slots[slot];
        int64_t sq[4] = {0}, sq_play = 0;
        for (int i = 0; i < frame; i++) {
            delay[delay_pos % MAX_DELAY] = play[i];
            s[i] = in[i * TR_AUDIO_INPUTS + MIC_A];
            s[frame + i] = in[i * TR_AUDIO_INPUTS + MIC_B];
            s[2 * frame + i] = delay[(delay_pos + MAX_DELAY - ref_delay) % MAX_DELAY];
            delay_pos++;
            sq_play += play[i] * play[i];
            for (int l = 0; l < 4; l++) {
                const int v = in[i * TR_AUDIO_INPUTS + l];
                sq[l] += v * v;
            }
        }
        ring_push(&raw, s, frame);
        if (xQueueSend(to_aec, &slot, 0) == pdTRUE) {
            slot = (slot + 1) % SLOTS;
        } else {
            overruns++; // the AEC fell behind: this frame is lost
        }
        level = fminf(1.0f, sqrtf((float)sq_play / frame) / 6000.0f);

        if (test != TEST_NONE) {
            if (env_n < ENV) {
                for (int l = 0; l < 4; l++) {
                    env_in[l][env_n] = rms16(sq[l], frame);
                }
                env_play[env_n++] = rms16(sq_play, frame);
            }
            // Onset: the first sample 8 times above the quiet first second.
            static int64_t quiet[4];
            if (test_pos == (RATE / frame) * frame) {
                for (int l = 0; l < 4; l++) {
                    quiet[l] = 0;
                    for (int k = 0; k < RATE / frame; k++) {
                        quiet[l] += (int64_t)env_in[l][k] * env_in[l][k];
                    }
                    quiet[l] = quiet[l] / (RATE / frame) + 1;
                }
            }
            for (int i = 0; i < frame && test_pos >= RATE; i++) {
                if (t_play_onset < 0 && play[i] != 0) {
                    t_play_onset = test_pos + i;
                }
                for (int l = 0; l < 4; l++) {
                    const int v = in[i * TR_AUDIO_INPUTS + l];
                    if (t_onset[l] < 0 && (int64_t)v * v > 64 * quiet[l]) {
                        t_onset[l] = test_pos + i;
                    }
                }
            }
            test_pos += frame;
            const int len = test == TEST_TONE ? 2 * RATE : 6 * RATE;
            if (test_pos >= len) {
                // The tone sets the reference delay: where microphone a
                // heard it start, 16 samples early.
                if (test == TEST_TONE && t_onset[MIC_A] > 0 && t_play_onset > 0) {
                    const int d = t_onset[MIC_A] - t_play_onset - 16;
                    if (d > 0 && d < MAX_DELAY) {
                        ref_delay = d;
                    }
                }
                test = TEST_NONE;
            }
        }
        for (int l = 0; l < 4; l++) {
            sum[l] += sq[l];
        }
        sum_play += sq_play;
        n_sum += frame;
        if (n_sum >= RATE) { // every second
            for (int l = 0; l < 4; l++) {
                rms_in[l] = rms16(sum[l], n_sum);
                sum[l] = 0;
            }
            rms_play = rms16(sum_play, n_sum);
            sum_play = 0;
            n_sum = 0;
        }
        if (++count == 5 * RATE / frame) { // every 5 s
            const int64_t now = esp_timer_get_time();
            frames10 = count * 10 * 1000000LL / (now - window);
            io_us = t_io / count;
            count = 0;
            t_io = 0;
            window = now;
        }
        if (state == T_OPEN) {
            tr_sleep_activity(); // a talk keeps the robot awake
        }
    }
}

// The AEC on both microphones, then their average.
static void aec_task(void *arg)
{
    int16_t *out = heap_caps_aligned_alloc(16, frame * 2 * 2, MALLOC_CAP_INTERNAL);
    int16_t *mono = heap_caps_malloc(frame * 2, MALLOC_CAP_INTERNAL);
    int slot, n = 0, n_sum = 0;
    int64_t t_sum = 0, sum = 0;
    for (;;) {
        xQueueReceive(to_aec, &slot, portMAX_DELAY);
        int16_t *s = slots[slot];
        const int64_t t0 = esp_timer_get_time();
        aec_process(aec, s, s + 2 * frame, out);
        t_sum += esp_timer_get_time() - t0;
        if (++n == 64) {
            aec_us = t_sum / n;
            n = 0;
            t_sum = 0;
        }
        int64_t sq = 0;
        for (int i = 0; i < frame; i++) {
            mono[i] = (out[i] + out[frame + i]) / 2;
            sq += mono[i] * mono[i];
        }
        ring_push(&clean, mono, frame);
        if (test != TEST_NONE && env_out_n < ENV) {
            env_out[env_out_n++] = rms16(sq, frame);
        }
        sum += sq;
        n_sum += frame;
        if (n_sum >= RATE) {
            rms_out = rms16(sum, n_sum);
            sum = 0;
            n_sum = 0;
        }
    }
}

// ---------------------------------------------------------------------------
// The WebSocket session

static esp_websocket_client_handle_t ws;
static char *rx;     // one message, put together from its pieces
static int16_t *pcm; // decoded voice, 24 kHz
static int16_t *pcm16;
static resampler_t down = {.step = (float)API_RATE / RATE};

// A JSON string value after "key":", copied into out. Enough for our events.
static bool json_str(const char *msg, const char *key, char *out, size_t size)
{
    char pat[32];
    snprintf(pat, sizeof(pat), "\"%s\":\"", key);
    const char *p = strstr(msg, pat);
    if (!p) {
        return false;
    }
    p += strlen(pat);
    size_t n = 0;
    while (p[n] && p[n] != '"' && n + 1 < size) {
        n++;
    }
    memcpy(out, p, n);
    out[n] = 0;
    return true;
}

static void on_message(char *msg, int len)
{
    msg[len] = 0;
    char type[64];
    if (!json_str(msg, "type", type, sizeof(type))) {
        return;
    }
    if (strcmp(type, "response.output_audio.delta") == 0) {
        char *p = strstr(msg, "\"delta\":\"");
        if (!p) {
            return;
        }
        p += 9;
        char *end = strchr(p, '"');
        if (!end) {
            return;
        }
        size_t bytes = 0;
        if (mbedtls_base64_decode((unsigned char *)pcm, RX_MAX, &bytes, (unsigned char *)p, end - p) != 0) {
            return;
        }
        if (deltas++ == 0 || first_audio_ms < 0) {
            if (speech_stopped_at > 0) {
                last_latency_ms = (esp_timer_get_time() - speech_stopped_at) / 1000;
                ESP_LOGI(TAG, "first sound of the answer %d ms after you stopped", last_latency_ms);
            }
            first_audio_ms = (esp_timer_get_time() - talk_since) / 1000;
        }
        const int n = resample(&down, pcm, bytes / 2, pcm16);
        ring_push(&voice, pcm16, n);
    } else if (strcmp(type, "input_audio_buffer.speech_started") == 0) {
        if (voice.in != voice.out) {
            barge_ins++;
            ESP_LOGI(TAG, "you talked over the robot: its answer stops");
        }
        ring_flush(&voice);
    } else if (strcmp(type, "input_audio_buffer.speech_stopped") == 0) {
        speech_stopped_at = esp_timer_get_time();
        first_audio_ms = -1;
    } else if (strcmp(type, "response.done") == 0) {
        answers++;
    } else if (strcmp(type, "session.created") == 0) {
        ESP_LOGI(TAG, "session open in %lld ms", (esp_timer_get_time() - talk_since) / 1000);
    } else if (strcmp(type, "error") == 0) {
        char m[120] = "";
        json_str(msg, "message", m, sizeof(m));
        snprintf(last_error, sizeof(last_error), "%s", m);
        ESP_LOGW(TAG, "error from the API: %s", m);
    } else if (strcmp(type, "response.output_audio_transcript.done") == 0) {
        char t[200] = "";
        json_str(msg, "transcript", t, sizeof(t));
        ESP_LOGI(TAG, "robot: %s", t);
    }
}

static void ws_event(void *arg, esp_event_base_t base, int32_t id, void *event_data)
{
    esp_websocket_event_data_t *d = event_data;
    if (id == WEBSOCKET_EVENT_DATA) {
        if (d->op_code != 0x1 && d->op_code != 0x0) {
            return; // only text (and its continuations)
        }
        if (d->payload_offset + d->data_len > RX_MAX - 1) {
            return; // too big: skip it
        }
        memcpy(rx + d->payload_offset, d->data_ptr, d->data_len);
        if (d->payload_offset + d->data_len >= d->payload_len && d->fin) {
            on_message(rx, d->payload_len);
        }
    } else if (id == WEBSOCKET_EVENT_DISCONNECTED || id == WEBSOCKET_EVENT_CLOSED) {
        if (state == T_OPEN) {
            ESP_LOGW(TAG, "the API closed the talk");
            close_asked = true;
        }
    } else if (id == WEBSOCKET_EVENT_ERROR) {
        snprintf(last_error, sizeof(last_error), "WebSocket error");
    }
}

// A session key from our key Worker: two lines, the model and the key.
static esp_err_t get_key(char *model, size_t msize, char *key, size_t ksize)
{
    if (!CONFIG_TALK_URL[0] || !CONFIG_TALK_TOKEN[0]) {
        snprintf(last_error, sizeof(last_error), "no key server set (TALK_URL, TALK_TOKEN)");
        return ESP_ERR_INVALID_STATE;
    }
    esp_http_client_config_t cfg = {
        .url = CONFIG_TALK_URL,
        .method = HTTP_METHOD_POST,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .timeout_ms = 10000,
    };
    esp_http_client_handle_t c = esp_http_client_init(&cfg);
    esp_http_client_set_header(c, "X-Robot-Token", CONFIG_TALK_TOKEN);
    char body[160] = "";
    esp_err_t err = esp_http_client_open(c, 0);
    int status = 0, n = 0;
    if (err == ESP_OK) {
        esp_http_client_fetch_headers(c);
        status = esp_http_client_get_status_code(c);
        n = esp_http_client_read_response(c, body, sizeof(body) - 1);
    }
    esp_http_client_cleanup(c);
    if (err != ESP_OK || status != 200 || n <= 0) {
        snprintf(last_error, sizeof(last_error), "key server: %s, HTTP %d", esp_err_to_name(err), status);
        return ESP_FAIL;
    }
    body[n] = 0;
    char *nl = strchr(body, '\n');
    if (!nl) {
        return ESP_FAIL;
    }
    *nl = 0;
    snprintf(model, msize, "%s", body);
    char *k = nl + 1;
    k[strcspn(k, "\r\n")] = 0;
    snprintf(key, ksize, "%s", k);
    return ESP_OK;
}

static TaskHandle_t session;

static void session_task(void *arg)
{
    char model[48], key[96], url[128], header[128];
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (state != T_OPENING) {
            continue;
        }
        talk_since = esp_timer_get_time();
        last_error[0] = 0;
        if (get_key(model, sizeof(model), key, sizeof(key)) != ESP_OK) {
            ESP_LOGW(TAG, "no session key: %s", last_error);
            state = T_OFF;
            continue;
        }
        ESP_LOGI(TAG, "session key in %lld ms, model %s", (esp_timer_get_time() - talk_since) / 1000, model);
        snprintf(url, sizeof(url), "wss://api.openai.com/v1/realtime?model=%s", model);
        snprintf(header, sizeof(header), "Authorization: Bearer %s\r\n", key);
        const esp_websocket_client_config_t cfg = {
            .uri = url,
            .headers = header,
            .crt_bundle_attach = esp_crt_bundle_attach,
            .buffer_size = 8192,
            .task_stack = 6144,
            .network_timeout_ms = 10000,
            .disable_auto_reconnect = true,
        };
        ws = esp_websocket_client_init(&cfg);
        esp_websocket_register_events(ws, WEBSOCKET_EVENT_ANY, ws_event, NULL);
        close_asked = false;
        barge_ins = answers = deltas = sent_msgs = 0;
        speech_stopped_at = 0;
        last_latency_ms = -1;
        if (esp_websocket_client_start(ws) == ESP_OK) {
            state = T_OPEN;
        } else {
            snprintf(last_error, sizeof(last_error), "WebSocket did not start");
        }
        while (state == T_OPEN && !close_asked && esp_timer_get_time() - talk_since < MAX_TALK_US) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        state = T_CLOSING;
        esp_websocket_client_close(ws, pdMS_TO_TICKS(2000));
        esp_websocket_client_destroy(ws);
        ws = NULL;
        ring_flush(&voice);
        ESP_LOGI(TAG, "talk ended after %lld s: %d answers, %d interruptions, %d messages sent",
                 (esp_timer_get_time() - talk_since) / 1000000, answers, barge_ins, sent_msgs);
        state = T_OFF;
    }
}


// During a talk: the cleaned sound to the API, at 24 kHz, every SEND_MS.
static void send_task(void *arg)
{
    const int per_msg = RATE * SEND_MS / 1000; // samples at 16 kHz
    int16_t *mic = heap_caps_malloc(per_msg * 2, MALLOC_CAP_SPIRAM);
    int16_t *up = heap_caps_malloc((per_msg * API_RATE / RATE + 16) * 2, MALLOC_CAP_SPIRAM);
    const size_t b64_size = (per_msg * API_RATE / RATE + 16) * 2 * 4 / 3 + 128;
    char *msg = heap_caps_malloc(b64_size + 64, MALLOC_CAP_SPIRAM);
    resampler_t upr = {.step = (float)RATE / API_RATE};
    TickType_t wake = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(SEND_MS));
        ring_t *from = send_raw ? &raw : &clean;
        if (state != T_OPEN || !ws || !esp_websocket_client_is_connected(ws)) {
            ring_flush(&raw);
            ring_flush(&clean);
            continue;
        }
        ring_flush(send_raw ? &clean : &raw);
        while (from->in - from->out >= (uint32_t)per_msg) {
            ring_pop(from, mic, per_msg);
            const int have = resample(&upr, mic, per_msg, up);
            int len = snprintf(msg, 64, "{\"type\":\"input_audio_buffer.append\",\"audio\":\"");
            size_t olen = 0;
            mbedtls_base64_encode((unsigned char *)msg + len, b64_size, &olen, (unsigned char *)up, have * 2);
            len += olen;
            len += snprintf(msg + len, 8, "\"}");
            if (esp_websocket_client_send_text(ws, msg, len, pdMS_TO_TICKS(1000)) > 0) {
                sent_msgs++;
            }
        }
    }
}

// ---------------------------------------------------------------------------

esp_err_t talk_start(void)
{
    if (started) {
        return ESP_OK;
    }
    const int64_t t0 = esp_timer_get_time();
    esp_err_t err = tr_audio_start(RATE);
    if (err != ESP_OK) {
        snprintf(last_error, sizeof(last_error), "no audio: %s", esp_err_to_name(err));
        return err;
    }
    tr_audio_volume(VOLUME);
    // Full duplex mode, with the aggressive non-linear stage. Its memory
    // (about 90 KB) in internal RAM: in PSRAM it took 28 ms of each 32.
    aec_config_t cfg = {
        .mic_num = 2,
        .ref_num = 1,
        .out_num = 2,
        .filter_length = FILTER,
        .sample_rate = RATE,
        .caps = MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT,
        .mode = AEC_MODE_FD_LOW_COST,
        .nlp_level = AEC_NLP_LEVEL_AGGR,
    };
    aec = aec_create_from_config(&cfg);
    if (!aec) {
        snprintf(last_error, sizeof(last_error), "AEC create failed");
        return ESP_FAIL;
    }
    frame = aec_get_chunksize(aec);
    ESP_LOGI(TAG, "AEC: %s, frame %d", aec_get_config_string(aec), frame);
    voice.s = heap_caps_malloc(RING * 2, MALLOC_CAP_SPIRAM);
    clean.s = heap_caps_malloc(MIC_RING * 2, MALLOC_CAP_SPIRAM);
    raw.s = heap_caps_malloc(MIC_RING * 2, MALLOC_CAP_SPIRAM);
    rx = heap_caps_malloc(RX_MAX, MALLOC_CAP_SPIRAM);
    pcm = heap_caps_malloc(RX_MAX, MALLOC_CAP_SPIRAM);
    pcm16 = heap_caps_malloc(RX_MAX, MALLOC_CAP_SPIRAM);
    if (!voice.s || !clean.s || !raw.s || !rx || !pcm || !pcm16) {
        return ESP_ERR_NO_MEM;
    }
    to_aec = xQueueCreate(SLOTS - 2, sizeof(int)); // one being filled, one in the AEC
    for (int i = 0; i < SLOTS; i++) {
        slots[i] = heap_caps_aligned_alloc(16, frame * 3 * 2, MALLOC_CAP_SPIRAM);
        if (!slots[i]) {
            return ESP_ERR_NO_MEM;
        }
    }
    // The AEC on core 1, away from the screen, Wi-Fi and the web server.
    xTaskCreatePinnedToCore(aec_task, "aec", 4096, NULL, 6, NULL, 1);
    xTaskCreatePinnedToCore(audio_task, "audio", 4096, NULL, 7, NULL, 0);
    xTaskCreatePinnedToCore(send_task, "send", 4096, NULL, 5, NULL, 0);
    xTaskCreate(session_task, "talk", 6144, NULL, 4, &session);
    started = true;
    ESP_LOGI(TAG, "audio ready in %lld ms; free: internal %u KB, PSRAM %u KB", (esp_timer_get_time() - t0) / 1000,
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024,
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024);
    return ESP_OK;
}

void talk_open(void)
{
    if (talk_start() != ESP_OK || state != T_OFF) {
        return;
    }
    state = T_OPENING;
    xTaskNotifyGive(session);
}

void talk_close(void)
{
    if (state == T_OPEN) {
        close_asked = true;
    }
}

bool talk_active(void)
{
    return state != T_OFF;
}

float talk_level(void)
{
    return level;
}

static void start_test(test_t t)
{
    for (int l = 0; l < 4; l++) {
        t_onset[l] = -1;
    }
    t_play_onset = -1;
    env_n = env_out_n = 0;
    test_pos = 0;
    test = t;
}

// Echo left after the AEC, in dB, over the frames with sound: microphone a
// against the cleaned sound, taken at the same time (the AEC adds no delay).
// The first burst is left out, while the AEC learns.
static float echo_removed(void)
{
    int64_t mic = 0, out = 0;
    const int from = (RATE + RATE) / frame; // after the first burst
    for (int i = from; i < env_n && i < env_out_n; i++) {
        if (env_play[i] > 0) {
            mic += (int64_t)env_in[MIC_A][i] * env_in[MIC_A][i];
            out += (int64_t)env_out[i] * env_out[i];
        }
    }
    return 10 * log10f(((float)mic + 1) / ((float)out + 1));
}

esp_err_t talk_get(httpd_req_t *req)
{
    static char text[1024];
    static const char *names[] = {"off", "opening", "open", "closing"};
    int n = snprintf(text, sizeof(text),
                     "talk: %s%s%s\nreference delay %d samples, sending %s\n"
                     "levels (rms, last second): in %.0f %.0f %.0f %.0f, played %.0f, cleaned %.0f\n"
                     "answers %d, interruptions %d, messages sent %d, voice pieces %d, overruns %d\n"
                     "last answer started %d ms after you stopped\n"
                     "audio: %d frames per 10 s (real time: %d), AEC %d us, play+record %d us per %d us frame\n"
                     "internal RAM: free %u KB, lowest %u KB, biggest block %u KB\n",
                     names[state], last_error[0] ? ", last error: " : "", last_error, ref_delay,
                     send_raw ? "microphone a as it is" : "the cleaned sound", rms_in[0], rms_in[1], rms_in[2],
                     rms_in[3], rms_play, rms_out, answers, barge_ins, sent_msgs, deltas, overruns,
                     last_latency_ms, frames10, frame ? 10 * RATE / frame : 0, aec_us, io_us,
                     frame ? frame * 1000000 / RATE : 0,
                     (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024,
                     (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL) / 1024,
                     (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL) / 1024);
    if (env_n > 0 && test == TEST_NONE) {
        n += snprintf(text + n, sizeof(text) - n, "last test: onset delay per input (samples):");
        for (int l = 0; l < 4; l++) {
            n += snprintf(text + n, sizeof(text) - n, " %d",
                          t_onset[l] >= 0 && t_play_onset >= 0 ? t_onset[l] - t_play_onset : -1);
        }
        n += snprintf(text + n, sizeof(text) - n, "\n");
        if (env_out_n > (2 * RATE) / frame) {
            n += snprintf(text + n, sizeof(text) - n, "echo removed: %.1f dB\n", echo_removed());
        }
    }
    httpd_resp_sendstr_chunk(req, text);
    if (env_n > 0 && test == TEST_NONE) {
        // Loudness per frame, every second frame: microphone a, then cleaned.
        httpd_resp_sendstr_chunk(req, "loudness per frame, microphone a, then cleaned:\n");
        for (int k = 0; k < 2; k++) {
            const uint16_t *e = k ? env_out : env_in[MIC_A];
            const int m = k ? env_out_n : env_n;
            n = 0;
            for (int i = 0; i < m && n < (int)sizeof(text) - 8; i += 2) {
                n += snprintf(text + n, sizeof(text) - n, "%u ", e[i]);
            }
            snprintf(text + n, sizeof(text) - n, "\n");
            httpd_resp_sendstr_chunk(req, text);
        }
    }
    // CPU time per task since boot (FreeRTOS run-time stats).
    static char stats[2048];
    vTaskGetRunTimeStats(stats);
    httpd_resp_sendstr_chunk(req, "CPU per task since boot:\n");
    httpd_resp_sendstr_chunk(req, stats);
    return httpd_resp_sendstr_chunk(req, NULL);
}

esp_err_t talk_post(httpd_req_t *req)
{
    if (!tr_ota_token_ok(req)) {
        return ESP_OK;
    }
    char query[96], value[16];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        if (httpd_query_key_value(query, "delay", value, sizeof(value)) == ESP_OK) {
            const int d = atoi(value);
            ref_delay = d >= 0 && d < MAX_DELAY ? d : ref_delay;
        }
        if (httpd_query_key_value(query, "send", value, sizeof(value)) == ESP_OK) {
            send_raw = strcmp(value, "mic") == 0;
        }
        if (httpd_query_key_value(query, "volume", value, sizeof(value)) == ESP_OK) {
            tr_audio_volume(atoi(value));
        }
        if (httpd_query_key_value(query, "test", value, sizeof(value)) == ESP_OK) {
            if (talk_start() == ESP_OK && state == T_OFF) {
                start_test(strcmp(value, "aec") == 0 ? TEST_BURSTS : TEST_TONE);
                const int ms = strcmp(value, "aec") == 0 ? 6500 : 2500;
                vTaskDelay(pdMS_TO_TICKS(ms)); // wait for it, then answer with the result
            }
        }
        if (httpd_query_key_value(query, "start", value, sizeof(value)) == ESP_OK) {
            talk_open();
        }
        if (httpd_query_key_value(query, "stop", value, sizeof(value)) == ESP_OK) {
            talk_close();
        }
    }
    return talk_get(req);
}
