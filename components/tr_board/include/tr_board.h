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
#include "esp_camera.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TR_I2C_PORT 0 // the internal bus; the camera driver shares it

// The internal I2C bus. Made on the first call; later calls return the same bus.
esp_err_t tr_board_i2c(i2c_master_bus_handle_t *bus);

// Find the chips on the internal I2C bus. Returns how many addresses it put in found.
int tr_board_scan(uint8_t *found, int max);

// Read or write one register of a chip on the internal bus (for debugging).
esp_err_t tr_board_reg_read(uint8_t addr, uint8_t reg, uint8_t *value);
esp_err_t tr_board_reg_write(uint8_t addr, uint8_t reg, uint8_t value);

// Set some AW9523 pins as outputs, high: port0 and port1 are bit masks
// (P0_0 = 0x01 ... P1_7 = 0x80 of port1). The other pins stay as they are.
esp_err_t tr_board_aw9523_on(uint8_t port0, uint8_t port1);
// Set some AW9523 outputs low again (the direction stays).
esp_err_t tr_board_aw9523_off(uint8_t port0, uint8_t port1);

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

// Backlight from 1 to 100 %, after tr_lcd_init. Less light uses less power.
esp_err_t tr_lcd_brightness(int percent);

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
// Draw w x h pixels that are already in the screen's byte order (RGB565,
// high byte first), as the camera gives them.
void tr_lcd_blit(int x, int y, int w, int h, const uint8_t *pixels);
// The same for rows y0 to y1 - 1 only, so text above or below stays.
void tr_lcd_render_rows(int y0, int y1, tr_lcd_pixel_fn pixel, void *ctx);

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

// The 12 side LEDs (0 to 5 on the left, 6 to 11 on the right), through the
// PY32. tr_servo_init() sets them up (all off). tr_led_set() only stores a
// color; tr_led_show() sends all 12 in one I2C transfer. The LEDs run on the
// body's 5 V (AW9523 BUS_OUT_EN and BOOST_EN): tr_led_show() turns it on while
// any LED is lit, only with USB power or a battery at 3.6 V or more, and off
// when all are off. ESP_ERR_INVALID_STATE means the battery is too low.
#define TR_LED_COUNT 12
esp_err_t tr_led_set(int index, uint8_t r, uint8_t g, uint8_t b);
esp_err_t tr_led_show(void);

// The other sensors on the I2C bus: head touch (Si12T), screen touch (FT6336),
// motion (BMI270), clock (BM8563), power chip (AXP2101), light (LTR553), and
// the ESP32-S3's own temperature. init sets them up; a chip that does not
// answer is skipped. status writes one or two lines per sensor.
esp_err_t tr_sensors_init(void);
void tr_sensors_status(char *text, size_t size);
// The power chip (AXP2101) as numbers. Needs tr_sensors_init().
typedef struct {
    bool battery;      // a battery is connected
    int percent;       // its fuel gauge, 0..100
    int millivolts;    // battery voltage, as the power chip measures it
    bool usb;          // USB power in
    int usb_mv;        // 0 when not known
    int system_mv;     // the system rail
    const char *state; // "standby", "charging" or "discharging"
} tr_power_t;
esp_err_t tr_power_read(tr_power_t *power);
// The first point touching the screen, in screen pixels. False if none.
// Needs tr_sensors_init().
bool tr_touch_read(int *x, int *y);
// The head touch zones, 2 bits each (zone 1 in bits 0-1). 0 = no touch.
int tr_head_touch(void);

// The two microphones (ES7210 on I2S: MCLK G0, BCLK G34, WS G33, data G14).
// start sets them up once, 16 bits, at sample_rate: channels 1 gives MIC1
// only, 2 gives MIC1 and MIC2 interleaved (L, R, L, R...). read waits for
// count samples (count / channels frames).
esp_err_t tr_mic_start(uint32_t sample_rate, int channels);
esp_err_t tr_mic_read(int16_t *samples, int count);

// The camera (GC0308 on the DVP port), 320 x 240 RGB565, frames in PSRAM
// when there is PSRAM. Powers it on the first call. Then take frames with
// esp_camera_fb_get() and give them back with esp_camera_fb_return().
esp_err_t tr_cam_start(void);
// Only the camera's power (AXP2101 ALDO3, BLDO1, BLDO2 and AW9523 P1_0).
// The light sensor needs it too.
esp_err_t tr_cam_power(void);

#ifdef __cplusplus
}
#endif
