#include "tr_emu.h"

#include <stdio.h>
#include <string.h>
#include "esp_heap_caps.h"

// Lives in internal RAM (.bss). The page finds it through the "@tr emu=" line.
static volatile tr_emu_shared_t s_shared;
static uint16_t *s_fb;

esp_err_t tr_emu_init(void)
{
    if (s_fb) {
        return ESP_OK;
    }
    // Internal RAM: the browser engine has no PSRAM.
    s_fb = heap_caps_malloc(TR_SCREEN_W * TR_SCREEN_H * sizeof(uint16_t),
                            MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!s_fb) {
        return ESP_ERR_NO_MEM;
    }
    memset(s_fb, 0, TR_SCREEN_W * TR_SCREEN_H * sizeof(uint16_t));

    s_shared.version = TR_EMU_VERSION;
    s_shared.size = sizeof(tr_emu_shared_t);
    s_shared.fb = (uint32_t)(uintptr_t)s_fb;
    s_shared.width = TR_SCREEN_W;
    s_shared.height = TR_SCREEN_H;
    s_shared.format = TR_EMU_FORMAT_RGB16;
    s_shared.frame = 0;
    s_shared.pan_mdeg = 0;
    s_shared.tilt_mdeg = 45000;
    s_shared.servo_seq = 0;
    // Magic last, so the page never sees a half-filled struct.
    s_shared.magic = TR_EMU_MAGIC;

    printf("@tr emu=%p\n", (void *)&s_shared);
    return ESP_OK;
}

uint16_t *tr_screen_fb(void)
{
    return s_fb;
}

void tr_screen_fill(uint16_t color)
{
    tr_screen_fill_rect(0, 0, TR_SCREEN_W, TR_SCREEN_H, color);
}

void tr_screen_fill_rect(int x, int y, int w, int h, uint16_t color)
{
    int x0 = x < 0 ? 0 : x;
    int y0 = y < 0 ? 0 : y;
    int x1 = x + w > TR_SCREEN_W ? TR_SCREEN_W : x + w;
    int y1 = y + h > TR_SCREEN_H ? TR_SCREEN_H : y + h;
    if (!s_fb || x0 >= x1 || y0 >= y1) {
        return;
    }
    for (int row = y0; row < y1; row++) {
        uint16_t *p = s_fb + row * TR_SCREEN_W + x0;
        for (int col = x0; col < x1; col++) {
            *p++ = color;
        }
    }
}

void tr_screen_fill_circle(int cx, int cy, int r, uint16_t color)
{
    // One horizontal line per row: the widest dx with dx^2 + dy^2 <= r^2.
    int dx = r;
    for (int dy = 0; dy <= r; dy++) {
        while (dx > 0 && dx * dx + dy * dy > r * r) {
            dx--;
        }
        tr_screen_fill_rect(cx - dx, cy - dy, 2 * dx + 1, 1, color);
        if (dy) {
            tr_screen_fill_rect(cx - dx, cy + dy, 2 * dx + 1, 1, color);
        }
    }
}

void tr_screen_present(void)
{
    s_shared.frame = s_shared.frame + 1;
}

static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : v > hi ? hi : v;
}

void tr_servo_set(float pan_deg, float tilt_deg)
{
    s_shared.pan_mdeg = (int32_t)(clampf(pan_deg, TR_PAN_MIN, TR_PAN_MAX) * 1000.0f);
    s_shared.tilt_mdeg = (int32_t)(clampf(tilt_deg, TR_TILT_MIN, TR_TILT_MAX) * 1000.0f);
    s_shared.servo_seq = s_shared.servo_seq + 1;
}

void tr_servo_get(float *pan_deg, float *tilt_deg)
{
    if (pan_deg) {
        *pan_deg = s_shared.pan_mdeg / 1000.0f;
    }
    if (tilt_deg) {
        *tilt_deg = s_shared.tilt_mdeg / 1000.0f;
    }
}
