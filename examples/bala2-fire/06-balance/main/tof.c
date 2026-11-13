// tof: see tof.h.
//
// The setup table, the registers and the start steps follow ST's VL53L1X
// Ultra Lite Driver (Copyright (c) 2018 STMicroelectronics, BSD 3-Clause),
// as in components/tr_board/tr_tof.c, which does the same for StackChan. We
// keep the sensor's defaults.
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tof.h"

#define ADDR          0x29
#define START_TIMEOUT 50 // ms, at boot
#define POLL_TIMEOUT  5  // ms, in the control loop: it has 5 ms in all

#define REG_VHV_BOUND 0x0008
#define REG_CONFIG    0x002D // the table goes from here to 0x0087
#define REG_READY     0x0031 // bit 0: a new measure waits
#define REG_ZONE_AT   0x007F // the zone's center
#define REG_ZONE_SIZE 0x0080 // (rows - 1) << 4 | (columns - 1)
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

static i2c_master_dev_handle_t dev;

static bool wr(uint16_t reg, uint8_t v, int timeout)
{
    const uint8_t d[3] = {reg >> 8, reg & 0xFF, v};
    return i2c_master_transmit(dev, d, 3, timeout) == ESP_OK;
}

static bool rd(uint16_t reg, uint8_t *buf, size_t n, int timeout)
{
    const uint8_t r[2] = {reg >> 8, reg & 0xFF};
    return i2c_master_transmit_receive(dev, r, 2, buf, n, timeout) == ESP_OK;
}

static bool start(void)
{
    uint8_t b = 0;
    if (!rd(REG_BOOTED, &b, 1, START_TIMEOUT) || !(b & 1) || !rd(REG_MODEL, &b, 1, START_TIMEOUT) ||
        b != 0xEA) {
        return false; // not a VL53L1X, or not awake yet
    }
    uint8_t table[2 + sizeof(config)] = {REG_CONFIG >> 8, REG_CONFIG & 0xFF};
    memcpy(table + 2, config, sizeof(config));
    // ST's start: one measure, then two settings that need it, then run.
    if (i2c_master_transmit(dev, table, sizeof(table), START_TIMEOUT) != ESP_OK ||
        !wr(REG_CLEAR, 0x01, START_TIMEOUT) || !wr(REG_START, 0x40, START_TIMEOUT)) {
        return false;
    }
    b = 0;
    for (int i = 0; i < 30 && !(b & 1); i++) {
        vTaskDelay(pdMS_TO_TICKS(10));
        if (!rd(REG_READY, &b, 1, START_TIMEOUT)) {
            b = 0;
        }
    }
    return (b & 1) && wr(REG_CLEAR, 0x01, START_TIMEOUT) && wr(REG_START, 0x00, START_TIMEOUT) &&
           wr(REG_VHV_BOUND, 0x09, START_TIMEOUT) && wr(0x000B, 0x00, START_TIMEOUT) &&
           wr(REG_CLEAR, 0x01, START_TIMEOUT) && wr(REG_START, 0x40, START_TIMEOUT);
}

bool tof_init(i2c_master_bus_handle_t bus)
{
    if (i2c_master_probe(bus, ADDR, START_TIMEOUT) != ESP_OK) {
        return false;
    }
    const i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = ADDR,
        .scl_speed_hz = 400000,
    };
    if (i2c_master_bus_add_device(bus, &cfg, &dev) != ESP_OK) {
        return false;
    }
    if (!start()) {
        i2c_master_bus_rm_device(dev);
        dev = NULL;
        return false;
    }
    return true;
}

int tof_poll(int *mm)
{
    uint8_t b[REG_DISTANCE + 2 - REG_STATUS];
    if (!dev || !rd(REG_READY, b, 1, POLL_TIMEOUT)) {
        return -1;
    }
    if (!(b[0] & 1)) {
        return 0;
    }
    // The status and the distance in one read, then let the next measure in.
    if (!rd(REG_STATUS, b, sizeof(b), POLL_TIMEOUT) || !wr(REG_CLEAR, 0x01, POLL_TIMEOUT)) {
        return -1;
    }
    const uint8_t *d = b + (REG_DISTANCE - REG_STATUS);
    *mm = (b[0] & 0x1F) == 9 ? d[0] << 8 | d[1] : -1;
    return 1;
}

bool tof_set_zone(int cols, int rows, int col, int row)
{
    if (!dev) {
        return false;
    }
    cols = cols < 4 ? 4 : (cols > 16 ? 16 : cols);
    rows = rows < 4 ? 4 : (rows > 16 ? 16 : rows);
    const int half_r = rows / 2, half_c = cols / 2;
    row = row < half_r - 1 ? half_r - 1 : (row > 15 - half_r ? 15 - half_r : row);
    col = col < half_c ? half_c : (col > 16 - half_c ? 16 - half_c : col);
    // ST numbers the detectors in two halves: the top 8 rows count up from
    // 128, column by column, the bottom 8 count down from 127.
    const int at = row < 8 ? 128 + 8 * col + row : 127 - 8 * col - (row - 8);
    // Stop, change, run again.
    return wr(REG_START, 0x00, POLL_TIMEOUT) && wr(REG_ZONE_AT, at, POLL_TIMEOUT) &&
           wr(REG_ZONE_SIZE, (rows - 1) << 4 | (cols - 1), POLL_TIMEOUT) &&
           wr(REG_CLEAR, 0x01, POLL_TIMEOUT) && wr(REG_START, 0x40, POLL_TIMEOUT);
}
