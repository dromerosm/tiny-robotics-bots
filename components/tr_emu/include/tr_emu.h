// tr_emu: a virtual screen and head servos for StackChan firmware that runs in
// the browser emulator (https://robotics.diegoromero.es/emulator/).
//
// The emulator cannot see the real SPI screen or the serial bus servos. So the
// firmware draws into a framebuffer in RAM and writes the servo angles to a
// small shared struct. At boot it prints one line with the struct's address:
//
//   @tr emu=0x3fc9a000
//
// The emulator page reads the struct and the framebuffer with read_memory()
// and shows the screen and the head. The layout of tr_emu_shared_t is the
// contract with the page: change it only together with TR_EMU_VERSION.
//
// This component only works in the emulator. It does not drive the real
// ILI9342C screen or the SCS servos.
#pragma once

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TR_SCREEN_W 320
#define TR_SCREEN_H 240

// RGB565 color from 8-bit red, green and blue.
#define TR_RGB(r, g, b) ((uint16_t)((((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | ((b) >> 3)))

#define TR_EMU_MAGIC        0x4D455254u // "TREM" in memory order
#define TR_EMU_VERSION      1
#define TR_EMU_FORMAT_RGB16 1 // RGB565, little-endian 16-bit words

#define TR_PAN_MIN -180.0f // pan (yaw) in degrees, 0 = ahead, positive = right seen from the front
#define TR_PAN_MAX 180.0f
// Tilt servo angle in degrees, 45 = middle, higher = looks up. The servo turns 0 to 90, but
// 5 to 85 is the safe range on the real robot (docs/writing-examples.md), so we clamp to it.
#define TR_TILT_MIN 5.0f
#define TR_TILT_MAX 85.0f

typedef struct {
    uint32_t magic;   // TR_EMU_MAGIC
    uint16_t version; // TR_EMU_VERSION
    uint16_t size;    // sizeof(tr_emu_shared_t)
    uint32_t fb;      // framebuffer address
    uint16_t width;   // TR_SCREEN_W
    uint16_t height;  // TR_SCREEN_H
    uint32_t format;  // TR_EMU_FORMAT_RGB16
    uint32_t frame;   // +1 on each tr_screen_present()
    int32_t pan_mdeg; // pan target in thousandths of a degree
    int32_t tilt_mdeg;
    uint32_t servo_seq; // +1 on each tr_servo_set()
} tr_emu_shared_t;

// Allocate the framebuffer in internal RAM (150 KB), clear it and print the
// "@tr emu=" line. Call it once, before anything else in this header.
esp_err_t tr_emu_init(void);

// The framebuffer: TR_SCREEN_W * TR_SCREEN_H pixels, row by row.
uint16_t *tr_screen_fb(void);

void tr_screen_fill(uint16_t color);
// Shapes are clipped to the screen.
void tr_screen_fill_rect(int x, int y, int w, int h, uint16_t color);
void tr_screen_fill_circle(int cx, int cy, int r, uint16_t color);
// Tell the page that a new frame is ready. Draw first, then present.
void tr_screen_present(void);

// Set the head target. Values are clamped to the ranges above.
void tr_servo_set(float pan_deg, float tilt_deg);
void tr_servo_get(float *pan_deg, float *tilt_deg);

#ifdef __cplusplus
}
#endif
