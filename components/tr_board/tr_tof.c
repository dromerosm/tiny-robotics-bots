// tr_tof: the distance sensor (Unit ToF4M, a VL53L1X) on Grove port A. See
// include/tr_board.h.
//
// The setup table, the registers and the start steps follow ST's VL53L1X
// Ultra Lite Driver (Copyright (c) 2018 STMicroelectronics, BSD 3-Clause).
// We keep the sensor's defaults: long distance mode, one measure every
// 100 ms.
#include <string.h>
#include "driver/i2c_master.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "tr_board.h"
#include "tr_board_priv.h"

static const char *TAG = "tr_tof";

#define PORT_A_SDA 2
#define PORT_A_SCL 1
#define ADDR       0x29
#define TIMEOUT_MS 50
#define RETRY_US   5000000 // no sensor: look again after this

#define REG_VHV_BOUND 0x0008
#define REG_CONFIG    0x002D // the table goes from here to 0x0087
#define REG_READY     0x0031 // bit 0: a new measure waits
#define REG_CLEAR     0x0086
#define REG_START     0x0087 // 0x40 starts, 0x00 stops
#define REG_STATUS    0x0089 // low 5 bits: 9 is a good measure
#define REG_DISTANCE  0x0096 // mm, 2 bytes
#define REG_BOOTED    0x00E5 // bit 0
#define REG_MODEL     0x010F // 0xEA

// Registers 0x2D to 0x87, as ST's driver writes them.
static const uint8_t config[] = {
    0x00, 0x01, 0x01, 0x01, 0x02, 0x00, 0x02, 0x08, 0x00, 0x08, 0x10, 0x01, 0x01, 0x00, 0x00, 0x00,
    0x00, 0xff, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x0b, 0x00, 0x00, 0x02, 0x0a, 0x21,
    0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0xc8, 0x00, 0x00, 0x38, 0xff, 0x01, 0x00, 0x08, 0x00,
    0x00, 0x01, 0xdb, 0x0f, 0x01, 0xf1, 0x0d, 0x01, 0x68, 0x00, 0x80, 0x08, 0xb8, 0x00, 0x00, 0x00,
    0x00, 0x0f, 0x89, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x0f, 0x0d, 0x0e, 0x0e, 0x00,
    0x00, 0x02, 0xc7, 0xff, 0x9B, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00,
};
_Static_assert(sizeof(config) == 0x87 - 0x2D + 1, "the table is 0x2D to 0x87");

static i2c_master_bus_handle_t bus;
static i2c_master_dev_handle_t dev;
static bool ready;
static int last_mm = -1;
static int64_t retry_at;
static SemaphoreHandle_t lock;

static esp_err_t wr(uint16_t reg, uint8_t v)
{
    const uint8_t d[3] = {reg >> 8, reg & 0xFF, v};
    return i2c_master_transmit(dev, d, 3, TIMEOUT_MS);
}

static esp_err_t rd(uint16_t reg, uint8_t *buf, size_t n)
{
    const uint8_t r[2] = {reg >> 8, reg & 0xFF};
    return i2c_master_transmit_receive(dev, r, 2, buf, n, TIMEOUT_MS);
}

// Remove the bus, so a scan can use the I2C port. The sensor keeps its setup.
static void release(void)
{
    if (dev) {
        i2c_master_bus_rm_device(dev);
        dev = NULL;
    }
    if (bus) {
        i2c_del_master_bus(bus);
        bus = NULL;
    }
    ready = false;
}

static esp_err_t start(void)
{
    const i2c_master_bus_config_t bus_cfg = {
        .i2c_port = 1, // port 0 is the internal bus
        .sda_io_num = PORT_A_SDA,
        .scl_io_num = PORT_A_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_cfg, &bus), TAG, "bus");
    // Quiet when nothing is there: the probe logs nothing for a plain "no".
    if (i2c_master_probe(bus, ADDR, 20) != ESP_OK) {
        return ESP_ERR_NOT_FOUND;
    }
    const i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = ADDR,
        .scl_speed_hz = 400000,
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(bus, &dev_cfg, &dev), TAG, "device");
    uint8_t b = 0;
    ESP_RETURN_ON_ERROR(rd(REG_BOOTED, &b, 1), TAG, "boot state");
    ESP_RETURN_ON_FALSE(b & 1, ESP_ERR_INVALID_STATE, TAG, "not booted yet");
    ESP_RETURN_ON_ERROR(rd(REG_MODEL, &b, 1), TAG, "model");
    ESP_RETURN_ON_FALSE(b == 0xEA, ESP_ERR_NOT_SUPPORTED, TAG, "model 0x%02X is not a VL53L1X", b);

    uint8_t table[2 + sizeof(config)] = {REG_CONFIG >> 8, REG_CONFIG & 0xFF};
    memcpy(table + 2, config, sizeof(config));
    ESP_RETURN_ON_ERROR(i2c_master_transmit(dev, table, sizeof(table), TIMEOUT_MS), TAG, "setup");
    // ST's start: one measure, then two settings that need it, then run.
    ESP_RETURN_ON_ERROR(wr(REG_CLEAR, 0x01), TAG, "clear");
    ESP_RETURN_ON_ERROR(wr(REG_START, 0x40), TAG, "start");
    for (int i = 0; i < 30; i++) {
        vTaskDelay(pdMS_TO_TICKS(10));
        if (rd(REG_READY, &b, 1) == ESP_OK && (b & 1)) {
            break;
        }
    }
    ESP_RETURN_ON_FALSE(b & 1, ESP_ERR_TIMEOUT, TAG, "no first measure");
    ESP_RETURN_ON_ERROR(wr(REG_CLEAR, 0x01), TAG, "clear");
    ESP_RETURN_ON_ERROR(wr(REG_START, 0x00), TAG, "stop");
    ESP_RETURN_ON_ERROR(wr(REG_VHV_BOUND, 0x09), TAG, "VHV");
    ESP_RETURN_ON_ERROR(wr(0x000B, 0x00), TAG, "VHV");
    ESP_RETURN_ON_ERROR(wr(REG_CLEAR, 0x01), TAG, "clear");
    ESP_RETURN_ON_ERROR(wr(REG_START, 0x40), TAG, "start");
    ESP_LOGI(TAG, "VL53L1X on port A: measuring");
    return ESP_OK;
}

void tr_tof_release(void)
{
    if (lock) {
        xSemaphoreTake(lock, portMAX_DELAY);
        release();
        retry_at = 0;
        xSemaphoreGive(lock);
    }
}

esp_err_t tr_tof_read(int *mm)
{
    if (!lock) {
        lock = xSemaphoreCreateMutex();
    }
    xSemaphoreTake(lock, portMAX_DELAY);
    esp_err_t err = ESP_OK;
    if (!ready) {
        const int64_t now = esp_timer_get_time();
        if (now < retry_at) {
            err = ESP_ERR_NOT_FOUND;
        } else {
            err = start();
            ready = err == ESP_OK;
            last_mm = -1;
            if (!ready) {
                release();
                retry_at = now + RETRY_US;
            }
        }
    }
    uint8_t b[2];
    if (err == ESP_OK) {
        err = rd(REG_READY, b, 1);
    }
    if (err == ESP_OK && (b[0] & 1)) { // a new measure
        uint8_t status = 0;
        err = rd(REG_STATUS, &status, 1);
        if (err == ESP_OK) {
            err = rd(REG_DISTANCE, b, 2);
        }
        if (err == ESP_OK) {
            last_mm = (status & 0x1F) == 9 ? b[0] << 8 | b[1] : -1;
            err = wr(REG_CLEAR, 0x01);
        }
    }
    if (err != ESP_OK && ready) { // the sensor went away (asleep: no 5 V)
        release();
        retry_at = esp_timer_get_time() + RETRY_US;
    }
    *mm = last_mm;
    xSemaphoreGive(lock);
    return err;
}
