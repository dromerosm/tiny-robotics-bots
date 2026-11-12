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

// The same for a Grove port of the body, given its two pins: port A is SDA G2
// and SCL G1, port B is G9 and G8, port C is G17 and G18. Returns -1 when a
// line of the port is held low (a unit with no power does that).
int tr_board_scan_port(int sda, int scl, uint8_t *found, int max);

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

// The battery for a status page: voltage and current, the average use and
// the time left, and the sleep state (tr_sleep_status). Fits tr_ota_add_status().
void tr_board_status(char *text, size_t size);

// The battery current averaged over about 20 s, in mA (0 when it does not
// discharge), and a guess of the minutes left at that use (-1 if not known).
// The sleep task updates both once a second, so they need tr_sleep_start().
float tr_battery_average_ma(void);
int tr_battery_minutes_left(void);

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
// Screen off: the panel sleeps (0x28, 0x10) and the backlight (DLDO1) turns
// off. Drawing does nothing until it is on again. On: the panel wakes (0x11,
// 120 ms, 0x29) and the backlight comes back at the same brightness. Without
// tr_lcd_init, off turns the backlight off and on does nothing.
esp_err_t tr_lcd_power(bool on);
// True after tr_lcd_init worked.
bool tr_lcd_ready(void);

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
// Servo power (VM_EN). After power on it waits until both servos answer, up
// to 2 s (ESP_ERR_TIMEOUT if they do not).
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

// The 5 V of the three Grove ports. It is the same line as the LEDs' 5 V, so
// the same battery rule holds. On: it stays on with all LEDs off. Off: it goes
// off when no LED is lit.
esp_err_t tr_port_power(bool on);

// The distance sensor on port A (Unit ToF4M, a VL53L1X), if there is one. It
// needs tr_port_power(true). The first call sets it up, and it measures about
// 10 times a second, up to 4 m. mm is the last distance, or -1 when the last
// measure was not good (nothing in range). ESP_ERR_NOT_FOUND: no sensor; the
// next look is 5 s later, so calling this often costs nothing.
esp_err_t tr_tof_read(int *mm);

// How wide the sensor looks. It has 16 x 16 detectors and uses a square of
// them around the middle: 16 is the widest view (27 degrees in M5Stack's
// docs), 4 the narrowest (15 degrees). A narrow view gets less light. The
// size is kept until the next restart, also with no sensor plugged in.
#define TR_TOF_ZONE_MIN 4
#define TR_TOF_ZONE_MAX 16
esp_err_t tr_tof_set_zone(int size);
// The same, with the square moved off the middle, to look a little to one
// side: its center as a column and a row of the 16 x 16 (the middle is column
// 8, row 7). The whole square must fit. One detector is about 1.7 degrees.
#define TR_TOF_ZONE_COL 8
#define TR_TOF_ZONE_ROW 7
esp_err_t tr_tof_set_zone_at(int size, int col, int row);
int tr_tof_zone(void);

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
// The head touch zones, 2 bits each (zone 1 in bits 0-1), when a zone got
// stronger since the last read; 0 otherwise. A zone can stay high with no hand
// on it, so only a rise counts as a touch.
int tr_head_touch(void);

// The two microphones (ES7210 on I2S: MCLK G0, BCLK G34, WS G33, data G14).
// start sets them up once, 16 bits, at sample_rate: channels 1 gives MIC1
// only, 2 gives MIC1 and MIC2 interleaved (L, R, L, R...). read waits for
// count samples (count / channels frames).
esp_err_t tr_mic_start(uint32_t sample_rate, int channels);
esp_err_t tr_mic_read(int16_t *samples, int count);

// Full duplex: the speaker and the microphones at the same time, for echo
// cancelling. One I2S bus, so not together with tr_mic_*. start opens both
// at sample_rate. read waits for count frames of TR_AUDIO_INPUTS samples
// (the ES7210's four inputs, in TDM slot order). write plays count mono
// samples and waits while the output buffers are full. Volume 0 to 100,
// microphone gain in dB.
#define TR_AUDIO_INPUTS 4
esp_err_t tr_audio_start(uint32_t sample_rate);
esp_err_t tr_audio_read(int16_t *frames, int count);
esp_err_t tr_audio_write(const int16_t *samples, int count);
esp_err_t tr_audio_volume(int percent);
esp_err_t tr_audio_gain(float db);
// Stop the speaker and the microphones (the amplifier and the ES7210 go to
// standby, and I2S stops, so the CPU can use light sleep), or start them
// again with the same rate, volume and gain. No read or write while paused.
esp_err_t tr_audio_pause(bool pause);

// The camera (GC0308 on the DVP port), 320 x 240 RGB565, frames in PSRAM
// when there is PSRAM. Powers it on the first call. Then take frames with
// esp_camera_fb_get() and give them back with esp_camera_fb_return().
esp_err_t tr_cam_start(void);
// Stop the camera (esp_camera_deinit) and free its frames, so the sleep step
// can cut its power. Give back every frame first, and take no new ones until
// tr_cam_start() again.
esp_err_t tr_cam_stop(void);
// True between tr_cam_start() and tr_cam_stop().
bool tr_cam_started(void);
// Only the camera's power (AXP2101 ALDO3, BLDO1, BLDO2 and AW9523 P1_0).
// The light sensor needs it too.
esp_err_t tr_cam_power(void);

// Sleep. After CONFIG_TR_SLEEP_AFTER_S seconds with no touch, on battery,
// the robot turns off what nobody uses: the screen, the servos and LEDs (with
// tr_web), the camera's power (and the light sensor), the motion sensor, and
// it lets the CPU slow down and sleep between ticks. Wi-Fi stays on, so the
// page and updates still work, a bit slower. A touch on the screen or the
// head wakes it, and the example does not see that touch. It never sleeps
// with USB power, and plugging USB in wakes it.
//
// tr_web_start() starts it. An example with a loop calls tr_sleep_asleep()
// and waits with tr_sleep_wait_awake() instead of drawing, and calls
// tr_sleep_activity() while someone uses a screen that is open.
#define TR_SLEEP_SCREEN 0x01 // panel asleep, backlight off, no drawing
#define TR_SLEEP_HOOKS  0x02 // the hooks: servos and LEDs off (tr_web), the example's own
#define TR_SLEEP_CAMERA                                                                            \
    0x04 // camera power off (ALDO3, BLDO1, BLDO2, AW9523 P1_0), light sensor too
#define TR_SLEEP_MOTION 0x08 // motion sensor (BMI270) suspended
#define TR_SLEEP_CPU    0x10 // light sleep and 80 MHz between ticks (CONFIG_PM_ENABLE)
#define TR_SLEEP_ALL    0x1F
esp_err_t tr_sleep_start(void);
bool tr_sleep_asleep(void);
void tr_sleep_wait_awake(void);
// Someone uses the robot: the count starts again.
void tr_sleep_activity(void);
// Wake now, and wait until it is awake (up to 3 s). Counts as activity.
esp_err_t tr_sleep_wake(void);
// Sleep now, with only some steps (to measure what each one saves).
esp_err_t tr_sleep_now(uint32_t steps);
// Seconds with no touch before it sleeps. 0 turns sleep off.
void tr_sleep_set_after(int seconds);
// fn(true) runs before the robot sleeps, fn(false) after it wakes, in the
// sleep task. Up to 4.
typedef void (*tr_sleep_fn)(bool going_to_sleep);
esp_err_t tr_sleep_add_hook(tr_sleep_fn fn);
// One line: "sleep: awake, sleeps in 42 s", "sleep: asleep for 3 min 10 s"...
void tr_sleep_status(char *text, size_t size);

#ifdef __cplusplus
}
#endif
