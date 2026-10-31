// tr_board: see include/tr_board.h.
#include "tr_board.h"

#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define I2C_SDA     12
#define I2C_SCL     11
#define I2C_HZ      400000
#define I2C_TIMEOUT 50 // ms

// INA226 (TI datasheet). Registers are 16 bits, high byte first.
#define INA226_ADDR      0x41
#define INA226_SHUNT     0x01 // signed, 2.5 uV per bit
#define INA226_BUS       0x02 // 1.25 mV per bit
#define INA226_MAKER     0xFE // 0x5449, "TI"
#define INA226_MAKER_TI  0x5449
#define SHUNT_MICRO_OHMS 10000 // 10 mOhm in the body (schematic)

static i2c_master_bus_handle_t bus;
static i2c_master_dev_handle_t ina226;
static bool ina226_checked;
static SemaphoreHandle_t lock;

esp_err_t tr_board_i2c(i2c_master_bus_handle_t *out)
{
    if (!bus) {
        const i2c_master_bus_config_t cfg = {
            .i2c_port = -1, // any free port
            .sda_io_num = I2C_SDA,
            .scl_io_num = I2C_SCL,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .flags.enable_internal_pullup = true,
        };
        esp_err_t err = i2c_new_master_bus(&cfg, &bus);
        if (err != ESP_OK) {
            return err;
        }
    }
    *out = bus;
    return ESP_OK;
}

static esp_err_t read16(uint8_t reg, uint16_t *value)
{
    uint8_t buf[2];
    esp_err_t err = i2c_master_transmit_receive(ina226, &reg, 1, buf, 2, I2C_TIMEOUT);
    *value = (uint16_t)(buf[0] << 8 | buf[1]);
    return err;
}

// Find the INA226 once. The status page and the example may call from two tasks.
static esp_err_t ina226_open(void)
{
    if (!lock) {
        lock = xSemaphoreCreateMutex();
    }
    xSemaphoreTake(lock, portMAX_DELAY);
    esp_err_t err = ESP_OK;
    if (!ina226_checked) {
        ina226_checked = true;
        i2c_master_bus_handle_t b;
        err = tr_board_i2c(&b);
        if (err == ESP_OK && i2c_master_probe(b, INA226_ADDR, I2C_TIMEOUT) != ESP_OK) {
            err = ESP_ERR_NOT_FOUND;
        }
        if (err == ESP_OK) {
            const i2c_device_config_t dev = {
                .dev_addr_length = I2C_ADDR_BIT_LEN_7,
                .device_address = INA226_ADDR,
                .scl_speed_hz = I2C_HZ,
            };
            err = i2c_master_bus_add_device(b, &dev, &ina226);
        }
        uint16_t maker = 0;
        if (err == ESP_OK && (read16(INA226_MAKER, &maker) != ESP_OK || maker != INA226_MAKER_TI)) {
            err = ESP_ERR_NOT_FOUND; // something else answers at 0x41
            i2c_master_bus_rm_device(ina226);
        }
        if (err != ESP_OK) {
            ina226 = NULL;
        }
    }
    if (!ina226 && err == ESP_OK) {
        err = ESP_ERR_NOT_FOUND;
    }
    xSemaphoreGive(lock);
    return err;
}

esp_err_t tr_battery_read(tr_battery_t *battery)
{
    esp_err_t err = ina226_open();
    if (err != ESP_OK) {
        return err;
    }
    uint16_t raw_bus, raw_shunt;
    xSemaphoreTake(lock, portMAX_DELAY);
    err = read16(INA226_BUS, &raw_bus);
    if (err == ESP_OK) {
        err = read16(INA226_SHUNT, &raw_shunt);
    }
    xSemaphoreGive(lock);
    if (err != ESP_OK) {
        return err;
    }
    battery->millivolts = raw_bus * 125 / 100;
    // 2.5 uV per bit over 10 mOhm = 0.25 mA per bit.
    const int32_t shunt_nanovolts = (int16_t)raw_shunt * 2500;
    battery->milliamps = shunt_nanovolts / SHUNT_MICRO_OHMS;
    return ESP_OK;
}

void tr_board_status(char *text, size_t size)
{
    tr_battery_t b;
    const esp_err_t err = tr_battery_read(&b);
    if (err == ESP_ERR_NOT_FOUND) {
        snprintf(text, size, "battery: no INA226 at 0x41 (emulator?)\n");
    } else if (err != ESP_OK) {
        snprintf(text, size, "battery: read failed (%s)\n", esp_err_to_name(err));
    } else {
        snprintf(text, size, "battery: %d.%02d V, %d mA (%s)\n", b.millivolts / 1000,
                 (b.millivolts % 1000) / 10, abs(b.milliamps),
                 b.milliamps > 0   ? "discharging"
                 : b.milliamps < 0 ? "charging"
                                   : "idle");
    }
}
