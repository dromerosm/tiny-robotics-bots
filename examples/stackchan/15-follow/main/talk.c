// talk: see talk.h. The same as 14-record's, but the echo canceller works
// only during a talk (or a test). It takes a third of core 1, and the face
// finder (track.c) needs that core. So before a talk, talk calls the quiet
// hook (the finder stops and frees its memory), and only then the echo
// canceller starts to work. With no talk, the "cleaned" sound is the average
// of both microphones as they are: the recorder gets that.
//
// The echo canceller's memory (92 KB of internal RAM, on one microphone) is
// taken once, at boot, and kept. We tried to free it after each talk and
// make it again for the next one. It does not work: later the biggest free
// block of internal RAM is 34 KB, the new echo canceller does not fit, and
// esp-sr crashes instead of returning an error.
//
// Four tasks:
// - audio (core 0): every frame (FRAME samples at 16 kHz) it plays the next
//   piece of the robot's voice (or silence) and reads the same amount from
//   the four ES7210 inputs. It passes both microphones and the reference
//   (what it played, delayed to line up with the microphones) to the echo
//   canceller;
// - aec (core 1): during a talk, esp-sr's echo canceller (AEC) on
//   microphone a, with its non-linear stage for the echo that is left.
//   With no talk, it only averages both microphones;
// - send: during a talk, every SEND_MS it sends the cleaned sound;
// - session: opens the WebSocket to our Worker (workers/talk-key), which
//   opens the session with GPT-Live (gpt-live-1) and passes the messages
//   both ways, and closes it at the end. The WebSocket client's own task
//   receives the robot's voice, which goes to the play buffer.
//
// Everything is 16 kHz, so there is no rate conversion. Full duplex: the
// microphones stay open while the robot speaks, and gpt-live-1 itself stops
// when you talk over it. Its voice comes in real time, so there is little
// to drop. When it needs to look something up, it hands the task to
// gpt-6-luna (set in the Worker).
//
// No AFE (esp-sr's whole audio front end) and no BSS (its two-microphone
// separation): together they needed more than all of core 1, and added
// about 100 ms. The AEC alone on both microphones took about a third.
//
//   GET  /talk                 state, levels, counts, times
//   POST /talk?start=1|stop=1  open or end a talk (token needed)
//   POST /talk?test=lanes      a 1 kHz tone: the level and delay on each input
//   POST /talk?test=aec        6 s of noise bursts: how much echo is left
//   POST /talk?delay=N         reference delay in samples, to try (0 by
//                              default; the tone test only measures)
//   POST /talk?send=aec|mic    send the cleaned sound, or microphone a as it is
//   POST /talk?volume=40       speaker volume, 0 to 100
//   POST /talk?gain=21         microphone gain in dB, to try (a recording
//                              sets it back to 21 when it ends)
//   POST /talk?nlp=0|1|2        how hard the AEC cuts what is left of the echo
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
#include "freertos/idf_additions.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "mbedtls/base64.h"
#include "sdkconfig.h"
#include "tr_board.h"
#include "tr_ota.h"

static const char *TAG = "talk";

#define RATE        16000
#define RING        RATE                 // 1 s of the robot's voice
#define MIC_RING    (RATE * 2)           // 2 s of the cleaned sound, to send
#define MAX_TALK_US (5 * 60 * 1000000LL) // a talk ends after 5 minutes
#define SEND_MS     40 // microphone sound per WebSocket message: short, so you are heard soon
#define RX_MAX      (64 * 1024) // the biggest message we take
// The robot's voice comes over the network in bursts. After the buffer runs
// empty, play waits for PREFILL (fewer gaps); above MAX_BUFFERED the oldest
// part goes (it would only delay the robot, and its stop when you cut in).
#define PREFILL      (RATE * 200 / 1000) // 120 ms left 14 gaps in 73 s
#define MAX_BUFFERED (RATE * 500 / 1000)
#define MAX_DELAY    2048 // reference delay line, in samples
#define SLOTS        4    // frames on their way to the AEC
// The speaker's volume and the microphones' gain go together. The echo
// canceller needs microphone a below about 11000 rms while the robot speaks.
// Above 15000 the microphone clips and the echo stays (13 to 22 dB removed).
// Measured with test=aec: 60 and 30 dB (14-record) gave 15300 rms and 22 to
// 33 dB; 75 and 21 dB gave 10500 rms and 66 dB; 85 and 18 dB, 10300 rms and
// 69 dB. Each 10 points of volume are about 4 dB.
#define VOLUME      75
#define MIC_GAIN_DB 21
#define FILTER      4 // AEC filter, in frames: 2 removed only 6 dB of echo

// The ES7210 inputs: 0 and 2 are the microphones (1 carries the amplifier's
// output, 3 is empty). As the reference, input 1 removed only about 8 dB
// of echo; what we played, lined up with the microphones, removed 23 to
// 35 dB. The delay is the time from our write to the sound in the
// microphones, a bit less so the reference comes first.
#define MIC_A    0
#define MIC_B    2
#define MIC_SLOT 0 // the AEC works on the first block of a slot: microphone a
// 0 here, not 79 as in 12-talk. The time from our write to the microphones
// changes each time the audio stops for sleep and starts again: the tone
// test gave 13 to 140 samples. A reference that comes after the echo is
// useless (8 dB removed), and one that comes up to 140 samples early is
// fine: 53 to 72 dB at delay 0, once the filter has learned again (the first
// seconds of the robot's voice after a wake).
static volatile int ref_delay = 0;

static aec_handle_t *aec;      // made once, in talk_start()
static volatile bool aec_want; // a talk or a test needs it: the aec task runs it
static volatile bool busy;     // from the quiet hook until the talk or test is over
static volatile int nlp_level = AEC_NLP_LEVEL_NORMAL;
static void (*quiet_hook)(void);
static int frame;             // samples per AEC frame
static QueueHandle_t to_aec;  // indices into slots[]
static int16_t *slots[SLOTS]; // mic a, mic b, reference: each FRAME samples
static bool started;
static volatile bool send_raw; // send microphone a as it is, to compare
static volatile int tap;       // 1: the recorder reads the cleaned sound, 2: microphone a

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

// State and numbers for GET /talk.
typedef enum { T_OFF, T_OPENING, T_OPEN, T_CLOSING } tstate_t;
static volatile tstate_t state = T_OFF;
static volatile bool close_asked;
static volatile float level;                        // the robot's voice, 0..1
static volatile float rms_in[4], rms_play, rms_out; // over the last second
static volatile int frames10, aec_us, io_us;        // frames per 10 s, times per frame
static volatile int answers, delegations, deltas, sent_msgs, overruns, skips, gaps;
static volatile uint32_t received;  // samples of the robot's voice since the talk opened
static volatile int64_t live_since; // when session.started came
static volatile int64_t talk_since;
static volatile int last_latency_ms = -1; // your last words to the robot's first words
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
        // Asleep with nothing to hear or say: stop the speaker, the
        // microphones and I2S until the robot wakes, so the CPU can sleep too.
        if (tr_sleep_asleep() && state == T_OFF && !busy && !tap && test == TEST_NONE) {
            tr_audio_pause(true);
            ESP_LOGI(TAG, "asleep: audio stopped");
            tr_sleep_wait_awake();
            tr_audio_pause(false);
            ESP_LOGI(TAG, "awake: audio on again");
            ring_flush(&raw);
            window = esp_timer_get_time();
            count = 0;
            t_io = 0;
            continue;
        }
        // What to play: a test signal, the robot's voice, or silence.
        int got;
        if (test == TEST_TONE) { // 1 s quiet, then 1 s of 1 kHz
            for (int i = 0; i < frame; i++) {
                const int t = test_pos + i;
                play[i] = t >= RATE && t < 2 * RATE
                              ? (int16_t)(8000 * sinf(2 * M_PI * 1000 * t / RATE))
                              : 0;
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
            static bool waiting = true;
            if (waiting && voice.in - voice.out >= PREFILL) {
                waiting = false;
            }
            got = waiting ? 0 : ring_pop(&voice, play, frame);
            if (!waiting && got < frame) {
                waiting = true;
                gaps += state == T_OPEN;
            }
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
                // The tone only reports the delay (GET /talk). It does not
                // set ref_delay any more: see ref_delay.
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

// Full duplex mode, with the normal non-linear stage: it removed as much
// echo as the aggressive one (46 to 51 dB), and it cuts less of your voice
// when you talk over the robot. Its memory in internal RAM: in PSRAM it took
// 28 ms of each 32. On microphone a only: with both microphones (as in
// 12-talk) it takes 136 KB of internal RAM, and then the camera and the face
// model do not fit. With one it takes 92 KB.
static aec_handle_t *aec_new(void)
{
    aec_config_t cfg = {
        .mic_num = 1,
        .ref_num = 1,
        .out_num = 1,
        .filter_length = FILTER,
        .sample_rate = RATE,
        .caps = MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT,
        .mode = AEC_MODE_FD_LOW_COST,
        .nlp_level = AEC_NLP_LEVEL_NORMAL,
    };
    return aec_create_from_config(&cfg);
}

// Stop the face finder (it gives core 1 and its memory back), then let the
// echo canceller work.
static void aec_get(void)
{
    busy = true;
    tr_sleep_wake(); // the audio task starts the speaker and the microphones again
    if (quiet_hook) {
        quiet_hook();
    }
    ESP_LOGI(TAG, "AEC works; free: internal %u KB (biggest block %u KB)",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024,
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL) / 1024);
    aec_want = true;
}

// The echo canceller rests again, and the finder may start.
static void aec_put(void)
{
    aec_want = false;
    busy = false;
}

// During a talk: the AEC on microphone a. With no talk: the average of both
// microphones as they are.
static void aec_task(void *arg)
{
    int16_t *out = heap_caps_aligned_alloc(16, frame * 2 * 2, MALLOC_CAP_INTERNAL);
    int16_t *mono = heap_caps_malloc(frame * 2, MALLOC_CAP_INTERNAL);
    int slot, n = 0, n_sum = 0;
    int64_t t_sum = 0, sum = 0;
    for (;;) {
        xQueueReceive(to_aec, &slot, portMAX_DELAY);
        int16_t *s = slots[slot];
        const int16_t *a = s, *b = s + frame; // the two microphones, as they are
        if (!aec_want) {
            aec_us = 0;
            n = 0;
            t_sum = 0;
        } else {
            static int nlp_set = AEC_NLP_LEVEL_NORMAL;
            if (nlp_set != nlp_level) {
                nlp_set = nlp_level;
                aec_set_nlp_level(aec, (aec_nlp_level_t)nlp_set);
            }
            const int64_t t0 = esp_timer_get_time();
            aec_process(aec, s + MIC_SLOT * frame, s + 2 * frame, out);
            t_sum += esp_timer_get_time() - t0;
            if (++n == 64) {
                aec_us = t_sum / n;
                n = 0;
                t_sum = 0;
            }
            a = b = out;
        }
        int64_t sq = 0;
        for (int i = 0; i < frame; i++) {
            mono[i] = (a[i] + b[i]) / 2;
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
// The WebSocket session, through our Worker to GPT-Live

static esp_websocket_client_handle_t ws;
// The send task uses ws while the session task may close and free it: both
// hold this lock. Without it, the robot restarted at the end of a talk.
static SemaphoreHandle_t ws_lock;
static char *rx;                   // one message, put together from its pieces
static int16_t *pcm;               // decoded voice, 16 kHz
static volatile bool live;         // session.started: the server takes audio
static int64_t heard_at;           // the last piece of your words the server wrote down
static char said[256], heard[256]; // the robot's words, and yours, as they come

// A JSON string value after "key":", copied into out. Enough for our events
// (the escapes stay as they are).
static bool json_str(const char *msg, const char *key, char *out, size_t size)
{
    char pat[32];
    snprintf(pat, sizeof(pat), "\"%s\":", key);
    const char *p = strstr(msg, pat);
    if (!p) {
        return false;
    }
    p += strlen(pat);
    while (*p == ' ') {
        p++;
    }
    if (*p++ != '"') {
        return false;
    }
    size_t n = 0;
    while (p[n] && !(p[n] == '"' && (n == 0 || p[n - 1] != '\\')) && n + 1 < size) {
        n++;
    }
    memcpy(out, p, n);
    out[n] = 0;
    return true;
}

// Words come in small pieces. Keep them, and log a line at the end of a
// sentence or when the line is full.
static void words(char *line, size_t size, const char *piece, const char *who)
{
    strncat(line, piece, size - strlen(line) - 1);
    const size_t n = strlen(line);
    if (n > 0 && (strchr(".?!", line[n - 1]) || n + 32 > size)) {
        ESP_LOGI(TAG, "%s:%s", who, line);
        line[0] = 0;
    }
}

static void on_message(char *msg, int len)
{
    msg[len] = 0;
    char type[48];
    if (!json_str(msg, "type", type, sizeof(type))) {
        return;
    }
    if (strcmp(type, "session.output_audio.delta") == 0) {
        // The voice comes in real time, silence too: the buffer stays short.
        // If it grows (the clocks differ a little), drop the oldest part.
        const char *p = strstr(msg, "\"delta\":");
        p = p ? strchr(p + 8, '"') : NULL;
        const char *end = p ? strchr(p + 1, '"') : NULL;
        size_t bytes = 0;
        if (!end || mbedtls_base64_decode((unsigned char *)pcm, RX_MAX, &bytes,
                                          (unsigned char *)p + 1, end - p - 1) != 0) {
            return;
        }
        deltas++;
        if (voice.in - voice.out > MAX_BUFFERED) {
            voice.out = voice.in - PREFILL;
            skips++;
        }
        ring_push(&voice, pcm, bytes / 2);
        received += bytes / 2;
    } else if (strcmp(type, "session.output_transcript.delta") == 0) {
        char t[160];
        if (json_str(msg, "delta", t, sizeof(t))) {
            if (heard_at > 0) {
                last_latency_ms = (esp_timer_get_time() - heard_at) / 1000;
                heard_at = 0;
                answers++;
            }
            words(said, sizeof(said), t, "robot");
        }
    } else if (strcmp(type, "session.input_transcript.delta") == 0) {
        char t[160];
        if (json_str(msg, "delta", t, sizeof(t))) {
            heard_at = esp_timer_get_time();
            words(heard, sizeof(heard), t, "you");
        }
    } else if (strcmp(type, "session.delegation.created") == 0) {
        delegations++;
        ESP_LOGI(TAG, "handed to gpt-6-luna");
    } else if (strcmp(type, "session.started") == 0) {
        live = true;
        live_since = esp_timer_get_time();
        received = 0;
        ESP_LOGI(TAG, "session open in %lld ms", (esp_timer_get_time() - talk_since) / 1000);
    } else if (strcmp(type, "session.closed") == 0) {
        char r[48] = "";
        json_str(msg, "reason", r, sizeof(r));
        ESP_LOGI(TAG, "the server closed the session: %s", r);
        close_asked = true;
    } else if (strcmp(type, "error") == 0) {
        char m[120] = "";
        json_str(msg, "message", m, sizeof(m));
        snprintf(last_error, sizeof(last_error), "%s", m);
        ESP_LOGW(TAG, "error from the API: %s", m);
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
        if (state == T_OPEN && !close_asked) {
            ESP_LOGW(TAG, "the connection closed");
            close_asked = true;
        }
    } else if (id == WEBSOCKET_EVENT_ERROR) {
        snprintf(last_error, sizeof(last_error), "WebSocket error");
    }
}

static TaskHandle_t session;

static void session_task(void *arg)
{
    char header[160];
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (state != T_OPENING) {
            continue;
        }
        talk_since = esp_timer_get_time();
        last_error[0] = 0;
        if (!CONFIG_TALK_URL[0] || !CONFIG_TALK_TOKEN[0]) {
            snprintf(last_error, sizeof(last_error), "no Worker set (TALK_URL, TALK_TOKEN)");
            state = T_OFF;
            continue;
        }
        // The finder stops and the echo canceller starts before the TLS
        // handshake, which needs internal RAM too.
        aec_get();
        snprintf(header, sizeof(header), "X-Robot-Token: %s\r\n", CONFIG_TALK_TOKEN);
        const esp_websocket_client_config_t cfg = {
            .uri = CONFIG_TALK_URL,
            .headers = header,
            .crt_bundle_attach = esp_crt_bundle_attach,
            .buffer_size = 8192,
            .task_stack = 6144,
            .network_timeout_ms = 10000,
            .disable_auto_reconnect = true,
        };
        ws = esp_websocket_client_init(&cfg);
        esp_websocket_register_events(ws, WEBSOCKET_EVENT_ANY, ws_event, NULL);
        close_asked = live = false;
        answers = delegations = deltas = sent_msgs = skips = gaps = 0;
        heard_at = 0;
        said[0] = heard[0] = 0;
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
        xSemaphoreTake(ws_lock, portMAX_DELAY);
        if (esp_websocket_client_is_connected(ws)) {
            static const char bye[] = "{\"type\":\"session.close\"}";
            esp_websocket_client_send_text(ws, bye, sizeof(bye) - 1, pdMS_TO_TICKS(1000));
        }
        esp_websocket_client_close(ws, pdMS_TO_TICKS(2000));
        esp_websocket_client_destroy(ws);
        ws = NULL;
        live = false;
        xSemaphoreGive(ws_lock);
        ring_flush(&voice);
        ESP_LOGI(TAG,
                 "talk ended after %lld s: %d answers, %d handed to gpt-6-luna, %d messages sent",
                 (esp_timer_get_time() - talk_since) / 1000000, answers, delegations, sent_msgs);
        aec_put();
        state = T_OFF;
    }
}

// During a talk: the cleaned sound to the server, every SEND_MS.
static void send_task(void *arg)
{
    const int per_msg = RATE * SEND_MS / 1000;
    int16_t *mic = heap_caps_malloc(per_msg * 2, MALLOC_CAP_SPIRAM);
    const size_t b64_size = per_msg * 2 * 4 / 3 + 16;
    char *msg = heap_caps_malloc(b64_size + 64, MALLOC_CAP_SPIRAM);
    TickType_t wake = xTaskGetTickCount();
    for (;;) {
        if (tr_sleep_asleep() && state == T_OFF) { // nothing to send: no wake every 40 ms
            tr_sleep_wait_awake();
            wake = xTaskGetTickCount();
        }
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(SEND_MS));
        ring_t *from = send_raw ? &raw : &clean;
        xSemaphoreTake(ws_lock, portMAX_DELAY);
        if (state != T_OPEN || !live || !ws || !esp_websocket_client_is_connected(ws)) {
            xSemaphoreGive(ws_lock);
            if (tap != 2) { // the recorder reads one of them
                ring_flush(&raw);
            }
            if (tap != 1) {
                ring_flush(&clean);
            }
            continue;
        }
        ring_flush(send_raw ? &clean : &raw);
        while (from->in - from->out >= (uint32_t)per_msg) {
            ring_pop(from, mic, per_msg);
            int len = snprintf(msg, 64, "{\"type\":\"session.input_audio.append\",\"audio\":\"");
            size_t olen = 0;
            mbedtls_base64_encode((unsigned char *)msg + len, b64_size, &olen, (unsigned char *)mic,
                                  per_msg * 2);
            len += olen;
            len += snprintf(msg + len, 8, "\"}");
            if (esp_websocket_client_send_text(ws, msg, len, pdMS_TO_TICKS(1000)) > 0) {
                sent_msgs++;
            }
        }
        xSemaphoreGive(ws_lock);
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
    tr_audio_gain(MIC_GAIN_DB);
    // The echo canceller's memory, once and for good (see the top of the file).
    const unsigned free0 = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    const unsigned big0 = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
    aec = aec_new();
    ESP_LOGI(TAG, "AEC takes %u KB of internal RAM; before it: free %u KB, biggest block %u KB",
             (free0 - (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL)) / 1024, free0 / 1024,
             big0 / 1024);
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
    if (!voice.s || !clean.s || !raw.s || !rx || !pcm) {
        return ESP_ERR_NO_MEM;
    }
    ws_lock = xSemaphoreCreateMutex();
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
    // These two wait on the network: their stacks go to PSRAM, to keep
    // internal RAM for the camera while the head follows a face.
    xTaskCreatePinnedToCoreWithCaps(send_task, "send", 4096, NULL, 5, NULL, 0, MALLOC_CAP_SPIRAM);
    xTaskCreatePinnedToCoreWithCaps(session_task, "talk", 6144, NULL, 4, &session, tskNO_AFFINITY,
                                    MALLOC_CAP_SPIRAM);
    started = true;
    ESP_LOGI(TAG, "audio ready in %lld ms; free: internal %u KB, PSRAM %u KB",
             (esp_timer_get_time() - t0) / 1000,
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024,
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024);
    return ESP_OK;
}

void talk_open(void)
{
    if (tap || talk_start() != ESP_OK || state != T_OFF) { // not while recording
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

bool talk_quiet(void)
{
    return state != T_OFF || busy || tap != 0;
}

void talk_set_quiet_hook(void (*hook)(void))
{
    quiet_hook = hook;
}

bool talk_live(void)
{
    return state == T_OPEN && live;
}

float talk_level(void)
{
    return level;
}

void talk_tap(int source)
{
    ring_flush(&clean);
    ring_flush(&raw);
    tap = source;
    if (source) {
        tr_sleep_wake();
    }
    if (source && quiet_hook) {
        quiet_hook(); // no motor noise in the recording
    }
}

int talk_tap_read(int16_t *s, int n)
{
    if (tap == 0 || state != T_OFF) {
        return 0;
    }
    return ring_pop(tap == 2 ? &raw : &clean, s, n);
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
    int n = snprintf(
        text, sizeof(text),
        "talk: %s%s%s\nreference delay %d samples, sending %s\n"
        "levels (rms, last second): in %.0f %.0f %.0f %.0f, played %.0f, cleaned %.0f\n"
        "answers %d, handed to gpt-6-luna %d, messages sent %d, voice pieces %d, AEC overruns %d, "
        "voice skips %d, gaps %d\nvoice received %.1f s in %.1f s, buffered now %d ms\nlast "
        "answer: first words %d ms after your last words\n"
        "audio: %d frames per 10 s (real time: %d), AEC %d us, play+record %d us per %d us frame\n"
        "internal RAM: free %u KB, lowest %u KB, biggest block %u KB\necho canceller: %s\n",
        names[state], last_error[0] ? ", last error: " : "", last_error, ref_delay,
        send_raw ? "microphone a as it is" : "the cleaned sound", rms_in[0], rms_in[1], rms_in[2],
        rms_in[3], rms_play, rms_out, answers, delegations, sent_msgs, deltas, overruns, skips,
        gaps, received / (float)RATE,
        live_since ? (esp_timer_get_time() - live_since) / 1e6f : 0.0f,
        (int)((voice.in - voice.out) * 1000 / RATE), last_latency_ms, frames10,
        frame ? 10 * RATE / frame : 0, aec_us, io_us, frame ? frame * 1000000 / RATE : 0,
        (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024,
        (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL) / 1024,
        (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL) / 1024,
        aec_want ? "at work" : "resting (no talk)");
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
        if (httpd_query_key_value(query, "nlp", value, sizeof(value)) == ESP_OK) {
            const int l = atoi(value); // 0 normal, 1 aggressive, 2 very aggressive
            if (l >= 0 && l <= 2) {
                nlp_level = l; // the aec task sets it
            }
        }
        if (httpd_query_key_value(query, "send", value, sizeof(value)) == ESP_OK) {
            send_raw = strcmp(value, "mic") == 0;
        }
        if (httpd_query_key_value(query, "volume", value, sizeof(value)) == ESP_OK) {
            tr_audio_volume(atoi(value));
        }
        if (httpd_query_key_value(query, "gain", value, sizeof(value)) == ESP_OK) {
            tr_audio_gain(atof(value)); // until the next recording ends
        }
        if (httpd_query_key_value(query, "test", value, sizeof(value)) == ESP_OK) {
            if (talk_start() == ESP_OK && state == T_OFF && !tap && !busy) {
                // The tests need the echo canceller, and no motor noise.
                aec_get();
                start_test(strcmp(value, "aec") == 0 ? TEST_BURSTS : TEST_TONE);
                const int ms = strcmp(value, "aec") == 0 ? 6500 : 2500;
                vTaskDelay(pdMS_TO_TICKS(ms)); // wait for it, then answer with the result
                aec_put();
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
