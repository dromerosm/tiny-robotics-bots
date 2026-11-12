// The head goes from -80 to 80 degrees and back, 5 degrees every 150 ms. At
// each stop the last distance of the sensor becomes one dot. The sensor looks
// through a cone of up to 27 degrees, so a dot is not a sharp point.
//
// The picture is half a circle of 2 m, with a ring every 0.5 m. One function
// gives the color of each pixel (tr_lcd_render_rows), with whole numbers
// only, so a frame costs little and does not flicker.
#include "radar.h"
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tr_board.h"
#include "tr_web.h"

#define STEP_DEG 5
#define STEP_MS  150
#define YAW_MAX  80
#define BINS     (2 * YAW_MAX / STEP_DEG + 1)
#define RANGE_MM 2000
#define TOP      28  // the text is above this row
#define CX       160 // the robot on the screen
#define CY       236
#define R        152 // pixels for RANGE_MM
#define BG       TR_LCD_BLACK
#define GRID     TR_LCD_RGB(0, 110, 50)
#define SWEEP    TR_LCD_RGB(80, 255, 120)
#define DOT      TR_LCD_RGB(160, 255, 180)
#define TEXT     TR_LCD_RGB(80, 255, 120)

static int16_t dot_x[BINS], dot_y[BINS]; // where each angle's dot is; x < 0: none
static uint8_t row_has[TR_LCD_H];        // rows with a dot, to skip the others fast
static int bin, dir, sweep_x, sweep_y;   // the sweep line, as sin and cos times 256
static int64_t next_step;
static bool placed;

static int yaw_of(int b)
{
    return b * STEP_DEG - YAW_MAX;
}

static uint16_t pixel(int x, int y, void *ctx)
{
    const int dx = x - CX, dy = CY - y;
    if (dy < 0) {
        return BG;
    }
    if (row_has[y]) {
        for (int i = 0; i < BINS; i++) {
            if (dot_x[i] >= 0 && abs(x - dot_x[i]) <= 2 && abs(y - dot_y[i]) <= 2) {
                return DOT;
            }
        }
    }
    const int r2 = dx * dx + dy * dy;
    if (r2 > R * R + R) {
        return BG;
    }
    if (dx * sweep_x + dy * sweep_y > 0 && abs(dx * sweep_y - dy * sweep_x) <= 256) {
        return SWEEP;
    }
    for (int r = R / 4; r <= R; r += R / 4) { // the rings
        if (abs(r2 - r * r) <= r) {
            return GRID;
        }
    }
    // The ground line, and a line every 30 degrees (sin and cos times 256).
    if (dy == 0 || dx == 0 || abs(abs(dx) * 222 - dy * 128) <= 128 ||
        abs(abs(dx) * 128 - dy * 222) <= 128) {
        return GRID;
    }
    return BG;
}

void radar_open(void)
{
    for (int i = 0; i < BINS; i++) {
        dot_x[i] = -1;
    }
    bin = 0;
    dir = 1;
    placed = false;
    next_step = 0;
    tr_lcd_fill_rect(0, 0, TR_LCD_W, TOP, BG);
}

void radar_step(int64_t now)
{
    if (now < next_step) {
        return;
    }
    const bool servos = tr_web_servos() && tr_web_head_power() == ESP_OK;
    if (!placed) { // go to the start of the sweep first
        placed = true;
        if (servos) {
            tr_servo_move(TR_SERVO_YAW, yaw_of(bin), 600);
            tr_servo_move(TR_SERVO_PITCH, TR_PITCH_MIN, 600);
        }
        next_step = now + 800000;
        return;
    }
    // The head is at this angle now: its distance, then on to the next one.
    int mm = -1;
    const bool sensor = tr_tof_read(&mm) == ESP_OK;
    const float a = yaw_of(bin) * (float)M_PI / 180.0f;
    sweep_x = (int)lroundf(sinf(a) * 256);
    sweep_y = (int)lroundf(cosf(a) * 256);
    if (mm >= 0) {
        const int r = (mm > RANGE_MM ? RANGE_MM : mm) * R / RANGE_MM;
        dot_x[bin] = CX + r * sweep_x / 256;
        dot_y[bin] = CY - r * sweep_y / 256;
    } else {
        dot_x[bin] = -1;
    }
    for (int y = 0; y < TR_LCD_H; y++) {
        row_has[y] = 0;
    }
    for (int i = 0; i < BINS; i++) {
        for (int y = dot_y[i] - 2; dot_x[i] >= 0 && y <= dot_y[i] + 2; y++) {
            if (y >= 0 && y < TR_LCD_H) {
                row_has[y] = 1;
            }
        }
    }
    char text[16];
    if (!sensor) {
        snprintf(text, sizeof(text), "no sensor");
    } else if (mm < 0) {
        snprintf(text, sizeof(text), "  --- mm ");
    } else {
        snprintf(text, sizeof(text), "%5d mm ", mm);
    }
    tr_lcd_text(8, 6, 2, TEXT, BG, text);
    snprintf(text, sizeof(text), "%4d deg", yaw_of(bin));
    tr_lcd_text(TR_LCD_W - 8 - 8 * 12, 6, 2, TEXT, BG, text);
    tr_lcd_render_rows(TOP, TR_LCD_H, pixel, NULL);

    if (bin + dir < 0 || bin + dir >= BINS) {
        dir = -dir;
    }
    bin += dir;
    if (servos) {
        tr_servo_move(TR_SERVO_YAW, yaw_of(bin), STEP_MS);
    }
    next_step = now + STEP_MS * 1000;
}

void radar_close(void)
{
    if (tr_web_servos() && tr_web_head_powered()) {
        tr_servo_move(TR_SERVO_YAW, 0, 600);
        vTaskDelay(pdMS_TO_TICKS(700));
        tr_web_head_off();
    }
}
