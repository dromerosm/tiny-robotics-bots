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

#ifdef __cplusplus
}
#endif
