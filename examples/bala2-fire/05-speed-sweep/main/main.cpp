// 05-speed-sweep: measure how fast the wheels turn for each speed value.
//
// The base takes a speed from -1023 to 1023, but that is a PWM duty, not a
// real speed. To learn the real one, we step both wheels through many values
// and use the encoders to measure the turns per minute (rpm) at each step:
//
//     pass 1  0 -> 1023, forward   lowest value that makes a wheel start
//     pass 2  1023 -> 0, forward   lowest value that keeps it turning
//     pass 3  0 -> -1023, backward the same, the other way
//
// Small steps near 0 (where the motor starts) and bigger ones above 200.
// Each step: 300 ms to settle, then 700 ms of counting. About 2 minutes.
//
// Run it with the robot lying down or held in the air: the wheels turn at
// full speed, and standing up it would drive away.
//
// Output, one line per step, easy to paste into a spreadsheet:
//
//     step,<pass>,<speed>,<left rpm>,<right rpm>
//
// In the emulator there is no base, so a simple motor model stands in for it
// and the lines say "(sim)".
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include <M5GFX.h>

static const char *TAG = "sweep";

#define I2C_SDA     21
#define I2C_SCL     22
#define I2C_HZ      100000
#define I2C_TIMEOUT 50 // ms
#define BASE_ADDR   0x3A
#define REG_SPEED   0x00
#define REG_ENCODER 0x10

#define COUNTS_PER_TURN 420.0f
#define SETTLE_MS       300
#define MEASURE_MS      700
#define COUNTDOWN_S     5

enum { LEFT, RIGHT };

static M5GFX display;
static i2c_master_dev_handle_t base;
static bool sim;
static int16_t sim_speed;
static int32_t sim_encoder[2];

// --- Base ---------------------------------------------------------------

static bool base_init(void)
{
    i2c_master_bus_config_t bus_cfg = {};
    bus_cfg.i2c_port = I2C_NUM_0;
    bus_cfg.sda_io_num = (gpio_num_t)I2C_SDA;
    bus_cfg.scl_io_num = (gpio_num_t)I2C_SCL;
    bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_cfg.glitch_ignore_cnt = 7;
    bus_cfg.flags.enable_internal_pullup = true;
    i2c_master_bus_handle_t bus;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus));

    if (i2c_master_probe(bus, BASE_ADDR, I2C_TIMEOUT) != ESP_OK) {
        return false;
    }
    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address = BASE_ADDR;
    dev_cfg.scl_speed_hz = I2C_HZ;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &dev_cfg, &base));
    return true;
}

// Same speed for both wheels.
static void set_speed(int16_t s)
{
    if (sim) {
        sim_speed = s;
        return;
    }
    const uint8_t buf[5] = {REG_SPEED, (uint8_t)(s >> 8), (uint8_t)s, (uint8_t)(s >> 8),
                            (uint8_t)s};
    if (i2c_master_transmit(base, buf, sizeof(buf), I2C_TIMEOUT) != ESP_OK) {
        ESP_LOGE(TAG, "could not send the speed to the base");
    }
}

// Motor model for the emulator: does not start below 60, then a straight line
// up to 500 rpm at 1023. The encoder counts down for a positive speed, as on
// our robot. Advances the model by ms milliseconds.
static void simulate(int ms)
{
    const int mag = abs(sim_speed);
    const float rpm = mag < 60 ? 0 : 500.0f * (mag - 60) / (1023 - 60);
    const float counts = rpm / 60.0f * COUNTS_PER_TURN * ms / 1000.0f;
    for (int w = 0; w < 2; w++) {
        sim_encoder[w] -= (int32_t)(sim_speed > 0 ? counts : -counts);
    }
}

static bool read_encoders(int32_t enc[2])
{
    if (sim) {
        enc[LEFT] = sim_encoder[LEFT];
        enc[RIGHT] = sim_encoder[RIGHT];
        return true;
    }
    const uint8_t reg = REG_ENCODER;
    uint8_t b[8];
    if (i2c_master_transmit_receive(base, &reg, 1, b, sizeof(b), I2C_TIMEOUT) != ESP_OK) {
        ESP_LOGE(TAG, "could not read the encoders");
        return false;
    }
    enc[LEFT] = (int32_t)((uint32_t)b[0] << 24 | b[1] << 16 | b[2] << 8 | b[3]);
    enc[RIGHT] = (int32_t)((uint32_t)b[4] << 24 | b[5] << 16 | b[6] << 8 | b[7]);
    return true;
}

static void wait_ms(int ms)
{
    if (sim) {
        simulate(ms);
    }
    vTaskDelay(pdMS_TO_TICKS(ms));
}

// --- Screen -------------------------------------------------------------

static void screen_line(int y, uint32_t color, const char *fmt, ...)
{
    char buf[64];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    display.setTextColor(color, TFT_BLACK);
    display.setTextPadding(304);
    display.drawString(buf, 8, y);
}

// --- Sweep --------------------------------------------------------------

// Speed values for one pass, from low to high: every 10 up to 200, then
// every 50, then 1023.
static int build_steps(int16_t *steps)
{
    int n = 0;
    for (int s = 0; s <= 200; s += 10) {
        steps[n++] = s;
    }
    for (int s = 250; s <= 1000; s += 50) {
        steps[n++] = s;
    }
    steps[n++] = 1023;
    return n;
}

// Run one speed and return the rpm of both wheels (always positive).
static void measure(int16_t speed, float rpm[2])
{
    set_speed(speed);
    wait_ms(SETTLE_MS);
    int32_t a[2] = {}, b[2] = {};
    read_encoders(a);
    wait_ms(MEASURE_MS);
    read_encoders(b);
    for (int w = 0; w < 2; w++) {
        rpm[w] = abs(b[w] - a[w]) / COUNTS_PER_TURN * 60000.0f / MEASURE_MS;
    }
}

static void run_pass(const char *name, const int16_t *steps, int n, int sign, bool down)
{
    display.setFont(&fonts::DejaVu18);
    screen_line(40, TFT_YELLOW, "Pass: %s", name);
    display.setFont(&fonts::DejaVu12);
    for (int i = 0; i < n; i++) {
        const int16_t speed = sign * steps[down ? n - 1 - i : i];
        float rpm[2];
        measure(speed, rpm);
        printf("step%s,%s,%d,%.1f,%.1f\n", sim ? " (sim)" : "", name, speed, rpm[LEFT], rpm[RIGHT]);
        screen_line(80, TFT_WHITE, "speed %5d", speed);
        screen_line(100, TFT_GREEN, "left  %6.1f rpm", rpm[LEFT]);
        screen_line(120, TFT_GREEN, "right %6.1f rpm", rpm[RIGHT]);
        screen_line(150, TFT_DARKGREY, "step %d of %d", i + 1, n);
    }
}

extern "C" void app_main(void)
{
    display.init();
    display.setRotation(1);
    display.fillScreen(TFT_BLACK);
    display.setFont(&fonts::DejaVu18);
    display.setTextColor(TFT_WHITE, TFT_BLACK);
    display.drawString("BALA2 speed sweep", 8, 6);

    sim = !base_init();
    if (sim) {
        ESP_LOGW(TAG, "no BALA2 base at 0x%02x (emulator?), using a motor model", BASE_ADDR);
    } else {
        ESP_LOGI(TAG, "BALA2 base found at 0x%02x", BASE_ADDR);
    }
    set_speed(0);

    // Time to put the robot down before the wheels start.
    display.setFont(&fonts::DejaVu12);
    for (int s = COUNTDOWN_S; s > 0; s--) {
        screen_line(80, TFT_RED, "Wheels start in %d s. Keep the robot lying down.", s);
        printf("start in %d s\n", s);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    int16_t steps[48];
    const int n = build_steps(steps);
    printf("step,pass,speed,left_rpm,right_rpm\n");
    run_pass("forward-up", steps, n, 1, false);
    run_pass("forward-down", steps, n, 1, true);
    run_pass("backward-up", steps, n, -1, false);
    set_speed(0);

    printf("sweep done%s\n", sim ? " (sim)" : "");
    display.setFont(&fonts::DejaVu18);
    screen_line(40, TFT_GREEN, "Sweep done");
    display.setFont(&fonts::DejaVu12);
    screen_line(170, TFT_WHITE, "Results are on the serial port.");
}
