// 02-battery: read the battery level and charging state of BALA2 Fire.
//
// The M5Stack Fire has an IP5306 power chip on its internal I2C bus (address
// 0x75). It charges the battery in the BALA2 base from the Fire's USB-C port.
// We read three registers (from the official M5Stack library, Power.cpp):
//
//     0x70 bit 3   1 = charging
//     0x71 bit 3   1 = charge full
//     0x78 bits 7-4 battery level in 25% steps
//
// The chip gives no voltage and no exact percentage.
//
// In the emulator there is no IP5306, so the example falls back to simulated
// values and marks them "(sim)". read_power() is the only function to swap.
#include <stdbool.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

static const char *TAG = "battery";

#define I2C_SDA       21 // Fire internal bus, shared with the IMU and the base
#define I2C_SCL       22
#define I2C_HZ        100000
#define I2C_TIMEOUT   50 // ms
#define IP5306_ADDR   0x75
#define REG_READ0     0x70
#define REG_READ1     0x71
#define REG_READ3     0x78
#define BIT_CHARGING  0x08
#define BIT_FULL      0x08
#define READ_EVERY_MS 2000

typedef struct {
    bool charging;
    bool full;
    int level; // 0, 25, 50, 75 or 100
} power_t;

static i2c_master_dev_handle_t ip5306;

// Same mapping as M5Stack's getBatteryLevel(): only the top 4 bits count.
static int level_from_reg(uint8_t reg)
{
    switch (reg & 0xF0) {
        case 0x00:
            return 100;
        case 0x80:
            return 75;
        case 0xC0:
            return 50;
        case 0xE0:
            return 25;
        default:
            return 0;
    }
}

static esp_err_t read_reg(uint8_t reg, uint8_t *val)
{
    return i2c_master_transmit_receive(ip5306, &reg, 1, val, 1, I2C_TIMEOUT);
}

// Returns false if the chip did not answer.
static bool read_power(power_t *p)
{
    uint8_t r0, r1, r3;
    if (read_reg(REG_READ0, &r0) != ESP_OK || read_reg(REG_READ1, &r1) != ESP_OK ||
        read_reg(REG_READ3, &r3) != ESP_OK) {
        return false;
    }
    p->charging = r0 & BIT_CHARGING;
    p->full = r1 & BIT_FULL;
    p->level = level_from_reg(r3);
    return true;
}

// Fake values for the emulator: the level goes up 25% per reading while
// "charging", until it is full.
static void simulate_power(power_t *p)
{
    static int level = 25;
    p->level = level;
    p->full = level == 100;
    p->charging = !p->full;
    if (level < 100) {
        level += 25;
    }
}

static const char *state_text(const power_t *p)
{
    if (p->full) {
        return "full";
    }
    return p->charging ? "charging" : "on battery";
}

// Look for the IP5306. Returns false if it is not there (as in the emulator).
static bool power_init(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus));

    if (i2c_master_probe(bus, IP5306_ADDR, I2C_TIMEOUT) != ESP_OK) {
        return false;
    }
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = IP5306_ADDR,
        .scl_speed_hz = I2C_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &dev_cfg, &ip5306));
    return true;
}

void app_main(void)
{
    const bool real = power_init();
    if (real) {
        ESP_LOGI(TAG, "IP5306 found at 0x%02x", IP5306_ADDR);
    } else {
        ESP_LOGW(TAG, "no IP5306 at 0x%02x (emulator?), using simulated values", IP5306_ADDR);
    }

    while (true) {
        power_t p;
        bool ok = real && read_power(&p);
        if (real && !ok) {
            ESP_LOGW(TAG, "IP5306 did not answer");
        } else {
            if (!real) {
                simulate_power(&p);
            }
            printf("battery%s: %3d%%  %s\n", real ? "" : " (sim)", p.level, state_text(&p));
        }
        vTaskDelay(pdMS_TO_TICKS(READ_EVERY_MS));
    }
}
