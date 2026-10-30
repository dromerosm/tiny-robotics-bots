// 04-wheels: drive each wheel with the Fire's buttons.
//
//     A (left button)    left wheel on
//     C (right button)   right wheel on
//     B (middle button)  stop both
//
// The motors belong to the BALA2 base. Its STM32 listens on I2C 0x3A. We use
// two registers, taken from M5Stack's code (Arduino bala.cpp, UiFlow2 bala2.py):
//
//     0x00  write 4 bytes: speed of motor 1 (left), then motor 2 (right),
//           int16 big-endian, -1023..1023 (a PWM duty, not a real speed)
//     0x10  read 8 bytes: encoder left, then right, int32 big-endian
//
// One wheel turn is 420 encoder counts (UiFlow2 docs).
//
// Checked on our robot: motor 1 is the left wheel, a positive speed turns the
// wheel forward (toward the screen side), and then the encoder counts DOWN.
// UiFlow2 says the opposite for both signs; Arduino matches the speed sign.
//
// Run it with the robot lying down or held in the air: it does not balance,
// so standing up it would drive away.
//
// In the emulator there is no base and the buttons do not exist, so the
// example runs a short demo with simulated encoders and says "(sim)".
#include <stdarg.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include <M5GFX.h>

static const char *TAG = "wheels";

#define I2C_SDA     21
#define I2C_SCL     22
#define I2C_HZ      100000 // what UiFlow2 uses; the base's maximum is not documented
#define I2C_TIMEOUT 50     // ms
#define BASE_ADDR   0x3A
#define REG_SPEED   0x00
#define REG_ENCODER 0x10

#define BTN_A GPIO_NUM_39 // the Fire's buttons: input only, pressed = 0
#define BTN_B GPIO_NUM_38
#define BTN_C GPIO_NUM_37

#define SPEED           400 // out of 1023
#define COUNTS_PER_TURN 420.0f
#define LOOP_MS         20 // button poll and encoder read
#define SHOW_EVERY      10 // screen and log every 200 ms

enum { LEFT, RIGHT };

static M5GFX display;
static i2c_master_dev_handle_t base;
static bool sim;
static int16_t speed[2]; // what we asked for
static int32_t encoder[2];

// --- Base (motors and encoders) -----------------------------------------

// Returns false if there is no base (as in the emulator).
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

static void set_speed(int16_t left, int16_t right)
{
    speed[LEFT] = left;
    speed[RIGHT] = right;
    if (sim) {
        return;
    }
    const uint8_t buf[5] = {REG_SPEED, (uint8_t)(left >> 8), (uint8_t)left, (uint8_t)(right >> 8),
                            (uint8_t)right};
    if (i2c_master_transmit(base, buf, sizeof(buf), I2C_TIMEOUT) != ESP_OK) {
        ESP_LOGE(TAG, "could not send the speed to the base");
    }
}

static void read_encoders(void)
{
    if (sim) {
        // Pretend the counts follow the speed: about 1 turn per second at 400,
        // counting down for a positive speed, as on the robot.
        for (int w = 0; w < 2; w++) {
            encoder[w] -= speed[w] * LOOP_MS * 420 / (400 * 1000);
        }
        return;
    }
    const uint8_t reg = REG_ENCODER;
    uint8_t b[8];
    if (i2c_master_transmit_receive(base, &reg, 1, b, sizeof(b), I2C_TIMEOUT) != ESP_OK) {
        ESP_LOGE(TAG, "could not read the encoders");
        return;
    }
    encoder[LEFT] = (int32_t)((uint32_t)b[0] << 24 | b[1] << 16 | b[2] << 8 | b[3]);
    encoder[RIGHT] = (int32_t)((uint32_t)b[4] << 24 | b[5] << 16 | b[6] << 8 | b[7]);
}

// --- Buttons ------------------------------------------------------------

static void buttons_init(void)
{
    // The Fire has pull-up resistors on these pins, and GPIO 34-39 have no
    // internal ones anyway.
    gpio_config_t cfg = {};
    cfg.pin_bit_mask = (1ULL << BTN_A) | (1ULL << BTN_B) | (1ULL << BTN_C);
    cfg.mode = GPIO_MODE_INPUT;
    ESP_ERROR_CHECK(gpio_config(&cfg));
}

// True once per press: the pin went from 1 (released) to 0 (pressed). Polling
// every 20 ms is slow enough to skip most contact bounce.
static bool pressed(gpio_num_t pin, int *last)
{
    const int now = gpio_get_level(pin);
    const bool down = *last == 1 && now == 0;
    *last = now;
    return down;
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

static void screen_init(void)
{
    display.fillScreen(TFT_BLACK);
    display.setFont(&fonts::DejaVu18);
    display.setTextColor(TFT_WHITE, TFT_BLACK);
    display.drawString(sim ? "BALA2 wheels (sim)" : "BALA2 wheels", 8, 6);

    // Labels above the three buttons.
    display.setFont(&fonts::DejaVu12);
    display.setTextDatum(textdatum_t::bottom_center);
    display.setTextColor(TFT_GREEN, TFT_BLACK);
    display.drawString("LEFT", 65, 236);
    display.setTextColor(TFT_RED, TFT_BLACK);
    display.drawString("STOP", 160, 236);
    display.setTextColor(TFT_GREEN, TFT_BLACK);
    display.drawString("RIGHT", 255, 236);
    display.setTextDatum(textdatum_t::top_left);
}

static void show(void)
{
    const char *names[2] = {"left ", "right"};
    display.setFont(&fonts::DejaVu18);
    for (int w = 0; w < 2; w++) {
        screen_line(50 + w * 60, speed[w] ? TFT_GREEN : TFT_DARKGREY, "%s  %s", names[w],
                    speed[w] ? "ON" : "off");
    }
    display.setFont(&fonts::DejaVu12);
    for (int w = 0; w < 2; w++) {
        screen_line(74 + w * 60, TFT_WHITE, "speed %5d   encoder %7ld   %+.2f turns", speed[w],
                    (long)encoder[w], encoder[w] / COUNTS_PER_TURN);
    }
    printf("%s left %4d enc %7ld | right %4d enc %7ld\n", sim ? "(sim)" : "", speed[LEFT],
           (long)encoder[LEFT], speed[RIGHT], (long)encoder[RIGHT]);
}

// --- Main ---------------------------------------------------------------

// In the emulator: left wheel 1 s, right wheel 1 s, stop.
static void sim_demo(int n)
{
    const int t = n * LOOP_MS;
    if (t == 500) {
        printf("demo: A, left wheel on\n");
        set_speed(SPEED, speed[RIGHT]);
    } else if (t == 1500) {
        printf("demo: C, right wheel on\n");
        set_speed(speed[LEFT], SPEED);
    } else if (t == 2500) {
        printf("demo: B, stop\n");
        set_speed(0, 0);
    } else if (t == 3000) {
        printf("demo done (sim)\n");
    }
}

extern "C" void app_main(void)
{
    display.init();
    display.setRotation(1);

    sim = !base_init();
    if (sim) {
        ESP_LOGW(TAG, "no BALA2 base at 0x%02x (emulator?), running a demo", BASE_ADDR);
    } else {
        ESP_LOGI(TAG, "BALA2 base found at 0x%02x", BASE_ADDR);
        buttons_init();
    }
    set_speed(0, 0); // never start with the motors on
    screen_init();

    int last_a = 1, last_b = 1, last_c = 1;
    TickType_t wake = xTaskGetTickCount();
    for (int n = 0;; n++) {
        if (sim) {
            sim_demo(n);
        } else {
            if (pressed(BTN_A, &last_a)) {
                printf("A: left wheel on\n");
                set_speed(SPEED, speed[RIGHT]);
            }
            if (pressed(BTN_C, &last_c)) {
                printf("C: right wheel on\n");
                set_speed(speed[LEFT], SPEED);
            }
            if (pressed(BTN_B, &last_b)) {
                printf("B: stop\n");
                set_speed(0, 0);
            }
        }
        read_encoders();
        if (n % SHOW_EVERY == 0) {
            show();
        }
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(LOOP_MS));
    }
}
