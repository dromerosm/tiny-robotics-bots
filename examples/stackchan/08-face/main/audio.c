// The audio analyzer screen: what the microphones hear, as 32 bars from
// 100 Hz to 8 kHz on a log scale, plus the level and the strongest tone.
//
// It also tries to find where a sound comes from. The two microphones hear it
// at slightly different times. The delay (in samples, at 16 kHz) is the shift
// that makes the two signals match best (cross-correlation), refined between
// samples with a parabola. Positive: MIC1 heard it first. The two channels
// also have a small fixed offset; audio_calibrate() measures it with sounds
// from the front and keeps it in NVS.
#include <math.h>
#include "esp_log.h"
#include "esp_timer.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "audio.h"
#include "nvs.h"
#include "tr_board.h"

#define MIC_RATE 16000 // 48 kHz (MCLK 12.3 MHz) blocked the robot's network for ~90 s
#define DECIM    1     // the bars use the same 16 kHz
#define RATE     (MIC_RATE / DECIM)
#define N        512         // FFT size at 16 kHz: 32 ms of sound, 31.25 Hz per bin
#define M        (N * DECIM) // samples per microphone per frame
#define BARS     32
#define TOP      30     // below the title bar
#define BOTTOM   212    // above the text line
#define FLOOR    -90.0f // dB at the bottom of a bar
#define CEIL     -20.0f // dB at the top
#define TITLE    TR_LCD_RGB(20, 60, 120)
#define EXIT_BG  TR_LCD_RGB(170, 40, 40)

static float re[N], im[N], window[N];
static int16_t samples[2 * M]; // MIC1, MIC2, MIC1, MIC2...
static float mic1[M], mic2[M];

// Measured on our robot at 16 kHz (2026-10-31): the two channels are offset
// by about -0.3 samples, and a sound from the left or right shifts that by only
// about 0.1 to 0.2. So the microphones are very close. More than MAX_LAG (at
// 16 kHz) is not possible, so larger shifts are echoes.
#define MAX_LAG   3
#define MIN_DB    -45.0f // quieter than this: no direction
#define ONSET_DB  10.0f  // a new sound must be this much above the room's background
#define MIN_MATCH 0.6f   // how well the shifted signals must match (0..1)
// The two holes are side by side on the front, about 30 mm apart (measured on
// our robot): sound needs at most 0.030 / 343 s = 87 us = 1.4 samples at
// 16 kHz to go from one to the other. A sound at angle a from straight ahead
// gives 1.4 * sin(a) samples.
#define FULL_LAG  1.4f
#define WINDOW    128    // 8 ms from the start of a sound: before room echoes
#define PRE_ONSET 11     // and a little before it
#define SIDE_DEG  15.0f  // closer to straight ahead than this is "front"
#define CAL_COUNT 5      // sounds from the front to measure the offset
static float lag_offset; // the channels' fixed offset, from NVS
static int cal_left;     // sounds still to take for the calibration
static float cal_sum;
static float floor_db = -60.0f; // the room's background, learned slowly
static volatile float dir_lag, dir_match, dir_db;
static volatile int64_t dir_time = -1;
static float bar_db[BARS], peak_db[BARS];
static int bar_lo[BARS], bar_hi[BARS]; // FFT bins of each bar
static bool ready;

// In-place radix-2 FFT.
static void fft(float *xr, float *xi)
{
    for (int i = 1, j = 0; i < N; i++) {
        int bit = N >> 1;
        for (; j & bit; bit >>= 1) {
            j ^= bit;
        }
        j ^= bit;
        if (i < j) {
            float t = xr[i];
            xr[i] = xr[j];
            xr[j] = t;
            t = xi[i];
            xi[i] = xi[j];
            xi[j] = t;
        }
    }
    for (int len = 2; len <= N; len <<= 1) {
        const float a = -2.0f * (float)M_PI / len;
        for (int i = 0; i < N; i += len) {
            for (int k = 0; k < len / 2; k++) {
                const float wr = cosf(a * k), wi = sinf(a * k);
                const int p = i + k, q = p + len / 2;
                const float tr = xr[q] * wr - xi[q] * wi, ti = xr[q] * wi + xi[q] * wr;
                xr[q] = xr[p] - tr;
                xi[q] = xi[p] - ti;
                xr[p] += tr;
                xi[p] += ti;
            }
        }
    }
}

esp_err_t audio_open(void)
{
    if (!ready) {
        esp_err_t err = tr_mic_start(MIC_RATE, 2);
        if (err != ESP_OK) {
            return err;
        }
        for (int i = 0; i < N; i++) {
            window[i] = 0.5f - 0.5f * cosf(2.0f * (float)M_PI * i / (N - 1)); // Hann
        }
        // Bars spaced on a log scale from 100 Hz to 8 kHz.
        for (int b = 0; b < BARS; b++) {
            const float f0 = 100.0f * powf(80.0f, (float)b / BARS);
            const float f1 = 100.0f * powf(80.0f, (float)(b + 1) / BARS);
            bar_lo[b] = (int)(f0 * N / RATE);
            bar_hi[b] = (int)(f1 * N / RATE);
            bar_hi[b] = bar_hi[b] <= bar_lo[b] ? bar_lo[b] + 1 : bar_hi[b];
            bar_hi[b] = bar_hi[b] > N / 2 ? N / 2 : bar_hi[b];
        }
        nvs_handle_t nvs;
        int32_t off = 0;
        if (nvs_open("audio", NVS_READONLY, &nvs) == ESP_OK) {
            nvs_get_i32(nvs, "lag_off", &off); // in thousandths of a sample
            nvs_close(nvs);
        }
        lag_offset = off / 1000.0f;
        ready = true;
    }
    for (int b = 0; b < BARS; b++) {
        bar_db[b] = peak_db[b] = FLOOR;
    }
    tr_lcd_fill(TR_LCD_BLACK);
    tr_lcd_fill_rect(0, 0, TR_LCD_W, 28, TITLE);
    tr_lcd_text(8, 6, 2, TR_LCD_WHITE, TITLE, "Audio");
    tr_lcd_fill_rect(TR_LCD_W - 56, 0, 56, 28, EXIT_BG);
    tr_lcd_text(TR_LCD_W - 34, 6, 2, TR_LCD_WHITE, EXIT_BG, "X");
    return ESP_OK;
}

static uint16_t bar_pixel(int x, int y, void *ctx)
{
    const int b = x / (TR_LCD_W / BARS);
    if (x % (TR_LCD_W / BARS) >= TR_LCD_W / BARS - 2) {
        return TR_LCD_BLACK; // gap between bars
    }
    const int h = BOTTOM - TOP;
    const float level = (bar_db[b] - FLOOR) / (CEIL - FLOOR);
    const float peak = (peak_db[b] - FLOOR) / (CEIL - FLOOR);
    const int from_bottom = BOTTOM - 1 - y;
    if (abs(from_bottom - (int)(peak * h)) <= 1 && peak > 0.02f) {
        return TR_LCD_WHITE; // the falling peak mark
    }
    if (from_bottom < (int)(level * h)) {
        const float f = (float)from_bottom / h; // green, then yellow, then red
        return f < 0.6f    ? TR_LCD_RGB(60, 200, 90)
               : f < 0.85f ? TR_LCD_RGB(240, 200, 40)
                           : TR_LCD_RGB(230, 60, 50);
    }
    return TR_LCD_BLACK;
}

// The delay between the two microphones, from their cross-correlation over
// -MAX_LAG..MAX_LAG samples, on the plain signals (mean removed), over the
// first 8 ms of a sound. About 900 multiplies, no extra memory.
static void find_direction(float db, int from, int len)
{
    float e1 = 0, e2 = 0, best = -1;
    int best_k = 0;
    static float corr[2 * MAX_LAG + 1];
    for (int i = from; i < from + len; i++) {
        e1 += mic1[i] * mic1[i];
        e2 += mic2[i] * mic2[i];
    }
    for (int k = -MAX_LAG; k <= MAX_LAG; k++) {
        float sum = 0;
        for (int i = from; i < from + len; i++) {
            sum += mic1[i] * mic2[i + k];
        }
        corr[k + MAX_LAG] = sum / sqrtf(e1 * e2 + 1e-12f);
        if (corr[k + MAX_LAG] > best) {
            best = corr[k + MAX_LAG];
            best_k = k;
        }
    }
    // One line per sound in /log, also for the ones we drop, to tune the gates.
    ESP_LOGI("audio", "sound %.0f dB (floor %.0f) at %d: match %.2f, raw lag %d", db, floor_db,
             from, best, best_k);
    if (best < MIN_MATCH) {
        return; // the two do not agree: noise, or the sound came from everywhere
    }
    float lag = best_k;
    if (best_k > -MAX_LAG && best_k < MAX_LAG) { // between samples: fit a parabola
        const float a = corr[best_k + MAX_LAG - 1], b = corr[best_k + MAX_LAG],
                    c = corr[best_k + MAX_LAG + 1];
        const float d = a - 2 * b + c;
        if (d < 0) {
            lag += 0.5f * (a - c) / d;
        }
    }
    if (cal_left > 0) { // calibrating: this sound came from the front
        cal_sum += lag;
        if (--cal_left == 0) {
            lag_offset = cal_sum / CAL_COUNT;
            nvs_handle_t nvs;
            if (nvs_open("audio", NVS_READWRITE, &nvs) == ESP_OK) {
                nvs_set_i32(nvs, "lag_off", (int32_t)lroundf(lag_offset * 1000));
                nvs_commit(nvs);
                nvs_close(nvs);
            }
        }
    }
    // MIC2 lagging MIC1 by `lag` samples means MIC1 heard it first.
    dir_lag = lag - lag_offset; // one reading per sound, not mixed with the last one
    dir_match = best;
    dir_db = db;
    dir_time = esp_timer_get_time();
}

void audio_calibrate(void)
{
    cal_sum = 0;
    cal_left = CAL_COUNT;
}

void audio_calibration(float *offset, int *sounds_left)
{
    *offset = lag_offset;
    *sounds_left = cal_left;
}

float audio_angle(float lag)
{
    const float x = lag / FULL_LAG;
    return asinf(x < -1 ? -1 : x > 1 ? 1 : x) * 180.0f / (float)M_PI;
}

const char *audio_side(float lag)
{
    // Seen from the front of the robot, MIC2 is on the left.
    const float a = audio_angle(lag);
    return a < -SIDE_DEG ? "< left" : a > SIDE_DEG ? "right >" : "front";
}

bool audio_direction(float *lag, float *match, float *db, int64_t *age_us)
{
    if (dir_time < 0) {
        return false;
    }
    *lag = dir_lag;
    *match = dir_match;
    *db = dir_db;
    *age_us = esp_timer_get_time() - dir_time;
    return true;
}

esp_err_t audio_frame(void)
{
    if (tr_mic_read(samples, 2 * M) != ESP_OK) {
        return ESP_FAIL;
    }
    double sum = 0, mean1 = 0, mean2 = 0;
    for (int i = 0; i < M; i++) {
        mic1[i] = samples[2 * i] / 32768.0f;
        mic2[i] = samples[2 * i + 1] / 32768.0f;
        mean1 += mic1[i];
        mean2 += mic2[i];
    }
    mean1 /= M;
    mean2 /= M;
    for (int i = 0; i < M; i++) {
        mic1[i] -= mean1;
        mic2[i] -= mean2;
    }
    // The bars: both microphones, 3 samples averaged to one (48 to 16 kHz).
    for (int i = 0; i < N; i++) {
        float v = 0;
        for (int j = 0; j < DECIM; j++) {
            v += mic1[i * DECIM + j] + mic2[i * DECIM + j];
        }
        v /= 2 * DECIM;
        sum += v * v;
        re[i] = v * window[i];
        im[i] = 0;
    }
    fft(re, im);

    int peak_bin = 1;
    float peak_power = 0;
    for (int k = 1; k < N / 2; k++) {
        const float p = re[k] * re[k] + im[k] * im[k];
        if (p > peak_power) {
            peak_power = p;
            peak_bin = k;
        }
    }
    for (int b = 0; b < BARS; b++) {
        float p = 0;
        for (int k = bar_lo[b]; k < bar_hi[b]; k++) {
            p += re[k] * re[k] + im[k] * im[k];
        }
        // Power to dB; 4/N^2 scales a full-scale sine to about 0 dB (Hann loses ~6 dB).
        const float db = 10.0f * log10f(p * 4.0f / ((float)N * N) + 1e-12f);
        bar_db[b] = db > bar_db[b] ? db : bar_db[b] * 0.7f + db * 0.3f; // fast up, slow down
        peak_db[b] = bar_db[b] > peak_db[b] ? bar_db[b] : peak_db[b] - 0.6f;
    }
    tr_lcd_render_rows(TOP, BOTTOM, bar_pixel, NULL);

    char line[32];
    const float rms_db = 10.0f * log10f((float)(sum / N) + 1e-12f);
    // Only the start of a sound (a clap, a knock, a word), well above the room.
    if (rms_db > MIN_DB && rms_db > floor_db + ONSET_DB) {
        // Where the sound starts in this frame: the first sample well above the
        // room's background. Only the next 8 ms count, before echoes arrive.
        const float limit = 4.0f * powf(10.0f, floor_db / 20.0f);
        int onset = -1;
        for (int i = MAX_LAG; i < M - MAX_LAG; i++) {
            if (fabsf(mic1[i]) > limit || fabsf(mic2[i]) > limit) {
                onset = i;
                break;
            }
        }
        int from = onset - PRE_ONSET;
        from = from < MAX_LAG ? MAX_LAG : from;
        if (onset >= 0 && from + WINDOW < M - MAX_LAG) {
            find_direction(rms_db, from, WINDOW);
        } else {
            ESP_LOGI("audio", "sound %.0f dB (floor %.0f): no clear start (onset %d)", rms_db,
                     floor_db, onset);
        }
    }
    floor_db =
        rms_db < floor_db ? 0.8f * floor_db + 0.2f * rms_db : 0.99f * floor_db + 0.01f * rms_db;
    float lag, match, db;
    int64_t age;
    if (audio_direction(&lag, &match, &db, &age) && age < 2000000) {
        const float a = audio_angle(lag);
        if (fabsf(a) < SIDE_DEG) {
            snprintf(line, sizeof(line), "%4.0fdB %4dHz front   ", rms_db, peak_bin * RATE / N);
        } else {
            snprintf(line, sizeof(line), "%4.0fdB %4dHz %s%2.0f%s  ", rms_db, peak_bin * RATE / N,
                     a < 0 ? "<" : " ", fabsf(a), a < 0 ? " " : ">");
        }
    } else {
        snprintf(line, sizeof(line), "%4.0fdB %4dHz  ---    ", rms_db, peak_bin * RATE / N);
    }
    tr_lcd_text(4, BOTTOM + 8, 2, TR_LCD_WHITE, TR_LCD_BLACK, line);
    return ESP_OK;
}
