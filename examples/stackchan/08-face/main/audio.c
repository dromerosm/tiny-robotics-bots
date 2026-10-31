// The audio analyzer screen: what the microphones hear, as 32 bars from
// 100 Hz to 8 kHz on a log scale, plus the level and the strongest tone.
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "audio.h"
#include "tr_board.h"

#define RATE    16000
#define N       512 // FFT size: 32 ms of sound, 31.25 Hz per bin
#define BARS    32
#define TOP     30     // below the title bar
#define BOTTOM  212    // above the text line
#define FLOOR   -90.0f // dB at the bottom of a bar
#define CEIL    -20.0f // dB at the top
#define TITLE   TR_LCD_RGB(20, 60, 120)
#define EXIT_BG TR_LCD_RGB(170, 40, 40)

static float re[N], im[N], window[N];
static int16_t samples[N];
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
        esp_err_t err = tr_mic_start(RATE);
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

esp_err_t audio_frame(void)
{
    if (tr_mic_read(samples, N) != ESP_OK) {
        return ESP_FAIL;
    }
    double sum = 0;
    for (int i = 0; i < N; i++) {
        const float v = samples[i] / 32768.0f;
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
    snprintf(line, sizeof(line), "%5.0f dBFS  peak %5d Hz", rms_db, peak_bin * RATE / N);
    tr_lcd_text(4, BOTTOM + 8, 2, TR_LCD_WHITE, TR_LCD_BLACK, line);
    return ESP_OK;
}
