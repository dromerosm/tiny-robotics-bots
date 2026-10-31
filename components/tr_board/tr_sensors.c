// tr_sensors: StackChan's sensors on the internal I2C bus, read as text for a
// status page. See include/tr_board.h.
//
// Registers and setup follow M5Stack's code (MIT): StackChan-BSP for the head
// touch (Si12T), M5Unified for the power chip (AXP2101), clock (BM8563) and
// motion (BMI270), M5GFX for the screen touch (FT6336).
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "bmi270_config.h"
#include "driver/temperature_sensor.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "tr_board.h"

static const char *TAG = "tr_sensors";

#define TIMEOUT_MS 50

enum { HEAD_TOUCH, SCREEN_TOUCH, MOTION, CLOCK, POWER, LIGHT, COUNT };
static const struct {
    uint8_t addr;
    uint32_t hz;
} chip[COUNT] = {
    [HEAD_TOUCH] = {0x68, 100000}, [SCREEN_TOUCH] = {0x38, 100000}, [MOTION] = {0x69, 400000},
    [CLOCK] = {0x51, 400000},      [POWER] = {0x34, 400000},        [LIGHT] = {0x23, 400000},
};
static i2c_master_dev_handle_t dev[COUNT];
static bool motion_ready;
static temperature_sensor_handle_t chip_temp;
static SemaphoreHandle_t lock;

static esp_err_t wr(int c, uint8_t reg, uint8_t v)
{
    const uint8_t d[2] = {reg, v};
    return i2c_master_transmit(dev[c], d, 2, TIMEOUT_MS);
}

static esp_err_t rd(int c, uint8_t reg, uint8_t *buf, size_t n)
{
    return i2c_master_transmit_receive(dev[c], &reg, 1, buf, n, TIMEOUT_MS);
}

// Head touch (Si12T): the BSP's setup, low type, sensitivity level 3.
static esp_err_t head_touch_init(void)
{
    static const uint8_t zero_regs[] = {0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F}; // channels on
    for (size_t i = 0; i < sizeof(zero_regs); i++) {
        ESP_RETURN_ON_ERROR(wr(HEAD_TOUCH, zero_regs[i], 0x00), TAG, "Si12T");
    }
    ESP_RETURN_ON_ERROR(wr(HEAD_TOUCH, 0x09, 0x0F), TAG, "Si12T"); // reset, then run
    ESP_RETURN_ON_ERROR(wr(HEAD_TOUCH, 0x09, 0x07), TAG, "Si12T");
    ESP_RETURN_ON_ERROR(wr(HEAD_TOUCH, 0x08, 0x22), TAG, "Si12T"); // auto mode, 10 s recal
    for (uint8_t reg = 0x02; reg <= 0x06; reg++) {
        ESP_RETURN_ON_ERROR(wr(HEAD_TOUCH, reg, 0x33), TAG, "Si12T"); // sensitivity
    }
    return ESP_OK;
}

// Motion (BMI270): soft reset, load Bosch's 8 KB setup file, then accel,
// gyro and temperature on.
static esp_err_t motion_init(void)
{
    uint8_t id = 0;
    ESP_RETURN_ON_ERROR(rd(MOTION, 0x00, &id, 1), TAG, "BMI270 id");
    ESP_RETURN_ON_FALSE(id == 0x24, ESP_ERR_NOT_FOUND, TAG, "BMI270 id 0x%02X", id);
    // Busy waits, not vTaskDelay: at 100 Hz, pdMS_TO_TICKS(1..9) is 0 ticks.
    wr(MOTION, 0x7E, 0xB6); // soft reset; the chip may not ACK this
    esp_rom_delay_us(2000); // Bosch: 2 ms after a soft reset
    ESP_RETURN_ON_ERROR(wr(MOTION, 0x7C, 0x00), TAG, "power save off");
    esp_rom_delay_us(1000); // Bosch: 450 us after power save off
    ESP_RETURN_ON_ERROR(wr(MOTION, 0x59, 0x00), TAG, "init start");
    uint8_t buf[1 + 64];
    for (size_t i = 0; i < sizeof(bmi270_config); i += 64) {
        const uint8_t addr[3] = {0x5B, (uint8_t)((i / 2) & 0x0F), (uint8_t)((i / 2) >> 4)};
        ESP_RETURN_ON_ERROR(i2c_master_transmit(dev[MOTION], addr, 3, TIMEOUT_MS), TAG, "addr");
        buf[0] = 0x5E;
        memcpy(buf + 1, bmi270_config + i, 64);
        ESP_RETURN_ON_ERROR(i2c_master_transmit(dev[MOTION], buf, sizeof(buf), TIMEOUT_MS), TAG,
                            "config");
    }
    ESP_RETURN_ON_ERROR(wr(MOTION, 0x59, 0x01), TAG, "init end");
    uint8_t status = 0;
    for (int i = 0; i < 20 && !(status & 0x01); i++) {
        vTaskDelay(pdMS_TO_TICKS(10));
        rd(MOTION, 0x21, &status, 1);
    }
    ESP_RETURN_ON_FALSE(status & 0x01, ESP_ERR_INVALID_STATE, TAG, "BMI270 status 0x%02X", status);
    return wr(MOTION, 0x7D, 0x0E); // temperature, accel and gyro on
}

esp_err_t tr_sensors_init(void)
{
    if (lock) {
        return ESP_OK;
    }
    lock = xSemaphoreCreateMutex();
    i2c_master_bus_handle_t bus;
    ESP_RETURN_ON_ERROR(tr_board_i2c(&bus), TAG, "I2C");
    for (int c = 0; c < COUNT; c++) {
        if (i2c_master_probe(bus, chip[c].addr, TIMEOUT_MS) != ESP_OK) {
            ESP_LOGW(TAG, "no answer at 0x%02X", chip[c].addr);
            continue;
        }
        const i2c_device_config_t cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = chip[c].addr,
            .scl_speed_hz = chip[c].hz,
        };
        if (i2c_master_bus_add_device(bus, &cfg, &dev[c]) != ESP_OK) {
            dev[c] = NULL;
        }
    }
    if (dev[HEAD_TOUCH] && head_touch_init() != ESP_OK) {
        ESP_LOGW(TAG, "head touch setup failed");
    }
    if (dev[SCREEN_TOUCH]) {
        wr(SCREEN_TOUCH, 0xA4, 0x00); // polling mode, as M5GFX sets it
    }
    if (dev[POWER]) {
        wr(POWER, 0x30, 0x3F); // all ADC channels on, as M5Unified does
    }
    motion_ready = dev[MOTION] && motion_init() == ESP_OK;
    if (dev[MOTION] && !motion_ready) {
        ESP_LOGW(TAG, "motion sensor setup failed");
    }
    return ESP_OK;
}

static int add(char *text, size_t size, int at, const char *fmt, ...)
    __attribute__((format(printf, 4, 5)));
static int add(char *text, size_t size, int at, const char *fmt, ...)
{
    if (at >= (int)size) {
        return at;
    }
    va_list args;
    va_start(args, fmt);
    const int n = vsnprintf(text + at, size - at, fmt, args);
    va_end(args);
    return n < 0 ? at : at + n;
}

static int bcd(uint8_t v)
{
    return (v >> 4) * 10 + (v & 0x0F);
}

static int read14(int c, uint8_t reg)
{
    uint8_t b[2];
    return rd(c, reg, b, 2) == ESP_OK ? ((b[0] & 0x3F) << 8 | b[1]) : -1;
}

void tr_sensors_status(char *text, size_t size)
{
    int at = 0;
    text[0] = 0;
    if (!lock) {
        add(text, size, 0, "sensors: not started\n");
        return;
    }
    xSemaphoreTake(lock, portMAX_DELAY);

    uint8_t b[16];
    // Head touch: three zones, two bits each, 0 = none to 3 = strong. Which
    // zone is front or back is not documented.
    if (dev[HEAD_TOUCH] && rd(HEAD_TOUCH, 0x10, b, 1) == ESP_OK) {
        at = add(text, size, at, "head touch: zone 1 %d, zone 2 %d, zone 3 %d (0..3)\n", b[0] & 3,
                 b[0] >> 2 & 3, b[0] >> 4 & 3);
    } else {
        at = add(text, size, at, "head touch: no answer\n");
    }

    // Screen touch: number of points, then the first point.
    if (dev[SCREEN_TOUCH] && rd(SCREEN_TOUCH, 0x02, b, 5) == ESP_OK) {
        const int points = b[0] & 0x0F;
        if (points > 0 && points <= 2) {
            at = add(text, size, at, "screen touch: %d point(s), first at x %d, y %d\n", points,
                     (b[1] & 0x0F) << 8 | b[2], (b[3] & 0x0F) << 8 | b[4]);
        } else {
            at = add(text, size, at, "screen touch: none\n");
        }
    } else {
        at = add(text, size, at, "screen touch: no answer\n");
    }

    // Motion: accel +-8 g and gyro +-2000 deg/s (the chip's defaults), little-endian.
    if (motion_ready && rd(MOTION, 0x0C, b, 12) == ESP_OK) {
        int16_t v[6];
        for (int i = 0; i < 6; i++) {
            v[i] = (int16_t)(b[2 * i] | b[2 * i + 1] << 8);
        }
        at = add(text, size, at, "accel: x %+.2f, y %+.2f, z %+.2f g\n", v[0] * 8.0f / 32768,
                 v[1] * 8.0f / 32768, v[2] * 8.0f / 32768);
        at = add(text, size, at, "gyro: x %+.1f, y %+.1f, z %+.1f deg/s\n", v[3] * 2000.0f / 32768,
                 v[4] * 2000.0f / 32768, v[5] * 2000.0f / 32768);
        if (rd(MOTION, 0x22, b, 2) == ESP_OK) {
            at = add(text, size, at, "motion chip: %.1f C\n",
                     23.0f + (int16_t)(b[0] | b[1] << 8) / 512.0f);
        }
    } else {
        at = add(text, size, at, "motion: not ready\n");
    }

    // Clock (BM8563): seconds to year from register 0x02, in BCD.
    if (dev[CLOCK] && rd(CLOCK, 0x02, b, 7) == ESP_OK) {
        at = add(text, size, at, "clock: %04d-%02d-%02d %02d:%02d:%02d%s\n",
                 bcd(b[6]) + (b[5] & 0x80 ? 1900 : 2000), bcd(b[5] & 0x1F), bcd(b[3] & 0x3F),
                 bcd(b[2] & 0x3F), bcd(b[1] & 0x7F), bcd(b[0] & 0x7F),
                 b[0] & 0x80 ? " (not set: it lost power)" : "");
    } else {
        at = add(text, size, at, "clock: no answer\n");
    }

    // Power chip (AXP2101).
    uint8_t st[2], pct;
    if (dev[POWER] && rd(POWER, 0x00, st, 2) == ESP_OK && rd(POWER, 0xA4, &pct, 1) == ESP_OK) {
        static const char *dir[4] = {"standby", "charging", "discharging", "?"};
        at = add(text, size, at, "power chip: battery %s, %d%%, %.2f V, %s\n",
                 st[0] & 0x08 ? "present" : "not found", pct, read14(POWER, 0x34) / 1000.0f,
                 dir[st[1] >> 5 & 3]);
        const int vbus = read14(POWER, 0x38), vsys = read14(POWER, 0x3A);
        at = add(text, size, at, "power chip: USB %s", st[0] & 0x20 ? "in" : "out");
        if (st[0] & 0x20 && vbus >= 0 && vbus < 0x3FFF - 32) {
            at = add(text, size, at, ", %.2f V", vbus / 1000.0f);
        }
        at = add(text, size, at, ", system %.2f V\n", vsys / 1000.0f);
    } else {
        at = add(text, size, at, "power chip: no answer\n");
    }

    at = add(text, size, at, "light: %s\n",
             dev[LIGHT] ? "LTR553 found, not read yet" : "LTR553 does not answer at 0x23");

    // Started on the first read, not at boot: esp-emu does not model it and
    // stops there, which would stop the example too.
    if (!chip_temp) {
        const temperature_sensor_config_t tcfg = TEMPERATURE_SENSOR_CONFIG_DEFAULT(-10, 80);
        if (temperature_sensor_install(&tcfg, &chip_temp) == ESP_OK) {
            temperature_sensor_enable(chip_temp);
        }
    }
    float t;
    if (chip_temp && temperature_sensor_get_celsius(chip_temp, &t) == ESP_OK) {
        add(text, size, at, "ESP32-S3: %.1f C\n", t);
    }
    xSemaphoreGive(lock);
}

bool tr_touch_read(int *x, int *y)
{
    if (!lock || !dev[SCREEN_TOUCH]) {
        return false;
    }
    uint8_t b[5];
    xSemaphoreTake(lock, portMAX_DELAY);
    const bool ok = rd(SCREEN_TOUCH, 0x02, b, 5) == ESP_OK;
    xSemaphoreGive(lock);
    if (!ok || (b[0] & 0x0F) == 0 || (b[0] & 0x0F) > 2) {
        return false;
    }
    *x = (b[1] & 0x0F) << 8 | b[2];
    *y = (b[3] & 0x0F) << 8 | b[4];
    return true;
}
