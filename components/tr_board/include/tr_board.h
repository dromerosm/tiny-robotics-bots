// tr_board: StackChan board parts shared by several examples.
//
// The internal I2C bus (SDA G12, SCL G11), the battery monitor in the body
// (INA226 at 0x41, 10 mOhm shunt) and the CoreS3 screen. More parts (PY32,
// servos) come here when the examples need them.
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "driver/i2c_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// The internal I2C bus. Made on the first call; later calls return the same bus.
esp_err_t tr_board_i2c(i2c_master_bus_handle_t *bus);

// Find the chips on the internal I2C bus. Returns how many addresses it put in found.
int tr_board_scan(uint8_t *found, int max);

typedef struct {
    int millivolts;
    int milliamps; // positive: the battery discharges; negative: it charges
} tr_battery_t;

// Read the body battery. ESP_ERR_NOT_FOUND if the INA226 does not answer (as in the emulator).
esp_err_t tr_battery_read(tr_battery_t *battery);

// One line about the battery, for a status page. Fits tr_ota_add_status().
void tr_board_status(char *text, size_t size);

// The screen: 320 x 240, RGB565 colors.
#define TR_LCD_W            320
#define TR_LCD_H            240
#define TR_LCD_RGB(r, g, b) ((uint16_t)((((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | ((b) >> 3)))
#define TR_LCD_BLACK        TR_LCD_RGB(0, 0, 0)
#define TR_LCD_WHITE        TR_LCD_RGB(255, 255, 255)

// Power the screen and its backlight (AXP2101, AW9523), find which screen is
// fitted (ILI9342C or ILI9342E) and clear it. Only on the real robot: the
// emulator does not model the screen.
esp_err_t tr_lcd_init(void);

void tr_lcd_fill(uint16_t color);
void tr_lcd_fill_rect(int x, int y, int w, int h, uint16_t color);

// One line of text with the 5x7 font, on a filled background. Each character
// is 6 x 8 pixels at scale 1 and 12 x 16 at scale 2 (the largest). Text that
// does not fit on the right is cut.
void tr_lcd_text(int x, int y, int scale, uint16_t fg, uint16_t bg, const char *text);

// Draw the whole screen from a function that gives the color of each pixel.
// It goes out in bands of 16 rows, so there is no flicker: use it for
// pictures that change, like a face. About 40 ms per frame.
typedef uint16_t (*tr_lcd_pixel_fn)(int x, int y, void *ctx);
void tr_lcd_render(tr_lcd_pixel_fn pixel, void *ctx);

// The head servos: FEETECH SCS0009 on UART1 (TX G6, RX G7, 1 Mbps), power
// through the PY32 in the body. Angles in degrees from the zero position,
// which comes from NVS ("servo", "zero_pos_1" and "zero_pos_2", the keys the
// factory firmware uses) or the BSP defaults.
typedef enum { TR_SERVO_YAW = 1, TR_SERVO_PITCH = 2 } tr_servo_t; // the servo IDs

#define TR_YAW_LIMIT 90.0f // our limit, to keep the cables safe; the BSP allows 128
#define TR_PITCH_MIN 5.0f  // below 5 or above 85 the pitch servo can stall
#define TR_PITCH_MAX 85.0f

typedef struct {
    int raw;       // position, 0..1023
    float degrees; // from the zero position
    int zero;      // the zero position in use
    int load;      // signed, servo units
    int volts_x10; // supply voltage in 0.1 V
    int celsius;
    bool torque;
    bool moving;
} tr_servo_state_t;

// Find the PY32 and set up the UART. It does not power or move the servos.
// ESP_ERR_NOT_FOUND without the body (as in the emulator).
esp_err_t tr_servo_init(void);
// Servo power (VM_EN). Waits 500 ms after power on.
esp_err_t tr_servo_power(bool on);
esp_err_t tr_servo_read(tr_servo_t id, tr_servo_state_t *state);
// Move to an angle in ms milliseconds (100..5000). Out-of-range angles are clamped.
esp_err_t tr_servo_move(tr_servo_t id, float degrees, int ms);
esp_err_t tr_servo_torque(tr_servo_t id, bool on);
// Save a new zero position (raw) in NVS.
esp_err_t tr_servo_set_zero(tr_servo_t id, int raw);

#ifdef __cplusplus
}
#endif
