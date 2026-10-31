// tr_servo: StackChan's two head servos (FEETECH SCS0009 on UART1) and their
// power switch (VM_EN on the PY32 in the body). See include/tr_board.h.
//
// Protocol, IDs, zero positions and the angle scale follow M5Stack's
// StackChan-BSP (src/M5StackChan.cpp, FTServo_Arduino SCSCL, MIT).
#include <math.h>
#include <string.h>
#include "driver/uart.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "tr_board.h"

static const char *TAG = "tr_servo";

#define SERVO_UART UART_NUM_1
#define SERVO_TX   6
#define SERVO_RX   7
#define SERVO_BAUD 1000000
#define REPLY_MS   10 // a servo answers within this

// SCS registers. 16-bit values are sent high byte first.
#define INST_PING    0x01
#define INST_READ    0x02
#define INST_WRITE   0x03
#define REG_TORQUE   40
#define REG_GOAL     42 // goal position, time (ms), speed: 2 bytes each
#define REG_POSITION 56 // position, speed, load: 2 bytes each; voltage, temperature: 1 byte
#define REG_MOVING   66

// PY32 IO expander in the body: pin 0 is VM_EN, the servo power.
#define PY32_ADDR     0x6F
#define PY32_ADDR_ALT 0x71
#define PY32_VERSION  0x02
#define PY32_MODE_L   0x03 // 1 = output
#define PY32_OUT_L    0x05
#define PY32_PULLUP_L 0x09
#define PY32_PULLDN_L 0x0B
#define VM_EN         0x01

// 0.3125 degrees per step: raw = zero + degrees * 3.2 (as the BSP does).
#define STEPS_PER_DEG 3.2f

static const struct {
    const char *zero_key;
    int default_zero; // the BSP's default; our NVS value wins
    float min_deg, max_deg;
} axis[3] = {
    [TR_SERVO_YAW] = {"zero_pos_1", 460, -TR_YAW_LIMIT, TR_YAW_LIMIT},
    [TR_SERVO_PITCH] = {"zero_pos_2", 620, TR_PITCH_MIN, TR_PITCH_MAX},
};

static bool ready;
static esp_err_t leds_init(void);
static int zero[3];
static SemaphoreHandle_t bus_lock;
static i2c_master_dev_handle_t py32;

// The PY32 is a small MCU: it NACKs a read that follows a repeated start, so
// we send the register and read it in two transfers, at 100 kHz.
static esp_err_t py32_read(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *v)
{
    ESP_RETURN_ON_ERROR(i2c_master_transmit(dev, &reg, 1, 50), TAG, "PY32 reg");
    return i2c_master_receive(dev, v, 1, 50);
}

static esp_err_t py32_bit(uint8_t reg, uint8_t bit, bool on)
{
    uint8_t v = 0;
    ESP_RETURN_ON_ERROR(py32_read(py32, reg, &v), TAG, "PY32 read");
    const uint8_t now = on ? (v | bit) : (v & ~bit);
    if (now == v) {
        return ESP_OK;
    }
    const uint8_t data[2] = {reg, now};
    return i2c_master_transmit(py32, data, 2, 50);
}

esp_err_t tr_servo_power(bool on)
{
    ESP_RETURN_ON_FALSE(py32, ESP_ERR_INVALID_STATE, TAG, "not started");
    ESP_RETURN_ON_ERROR(py32_bit(PY32_OUT_L, VM_EN, on), TAG, "VM_EN");
    if (on) {
        vTaskDelay(pdMS_TO_TICKS(500)); // 200 ms (the BSP) was not enough on our unit
    }
    return ESP_OK;
}

// Send one packet: FF FF id len inst params... checksum.
static void send_packet(uint8_t id, uint8_t inst, const uint8_t *params, int n)
{
    uint8_t buf[16];
    buf[0] = 0xFF;
    buf[1] = 0xFF;
    buf[2] = id;
    buf[3] = n + 2;
    buf[4] = inst;
    uint8_t sum = id + n + 2 + inst;
    for (int i = 0; i < n; i++) {
        buf[5 + i] = params[i];
        sum += params[i];
    }
    buf[5 + n] = ~sum;
    uart_flush_input(SERVO_UART);
    uart_write_bytes(SERVO_UART, buf, 6 + n);
    uart_wait_tx_done(SERVO_UART, pdMS_TO_TICKS(REPLY_MS));
}

// Read the answer: FF FF id len error data... checksum. It skips our own
// packet if the bus echoes it back.
static esp_err_t read_reply(uint8_t id, uint8_t *data, int n)
{
    uint8_t buf[32];
    int got = 0;
    const int64_t end = esp_timer_get_time() + REPLY_MS * 1000;
    while (got < (int)sizeof(buf) && esp_timer_get_time() < end) {
        const int r = uart_read_bytes(SERVO_UART, buf + got, sizeof(buf) - got, pdMS_TO_TICKS(2));
        if (r > 0) {
            got += r;
        } else if (got >= 6 + n) {
            break;
        }
    }
    for (int i = 0; i + 6 + n <= got; i++) {
        if (buf[i] != 0xFF || buf[i + 1] != 0xFF || buf[i + 2] != id || buf[i + 3] != n + 2) {
            continue;
        }
        uint8_t sum = id + n + 2 + buf[i + 4];
        for (int k = 0; k < n; k++) {
            sum += buf[i + 5 + k];
        }
        if ((uint8_t)~sum != buf[i + 5 + n]) {
            continue; // not a valid answer: maybe our own echo, keep looking
        }
        memcpy(data, buf + i + 5, n);
        return ESP_OK;
    }
    return got ? ESP_ERR_INVALID_RESPONSE : ESP_ERR_TIMEOUT;
}

static esp_err_t scs_read(uint8_t id, uint8_t reg, uint8_t *data, int n)
{
    const uint8_t params[2] = {reg, (uint8_t)n};
    xSemaphoreTake(bus_lock, portMAX_DELAY);
    send_packet(id, INST_READ, params, 2);
    const esp_err_t err = read_reply(id, data, n);
    xSemaphoreGive(bus_lock);
    return err;
}

static esp_err_t scs_write(uint8_t id, uint8_t reg, const uint8_t *data, int n)
{
    uint8_t params[8];
    params[0] = reg;
    memcpy(params + 1, data, n);
    xSemaphoreTake(bus_lock, portMAX_DELAY);
    send_packet(id, INST_WRITE, params, n + 1);
    uint8_t none;
    const esp_err_t err = read_reply(id, &none, 0);
    xSemaphoreGive(bus_lock);
    return err;
}

static int load_zero(tr_servo_t id)
{
    nvs_handle_t nvs;
    int32_t v = axis[id].default_zero;
    if (nvs_open("servo", NVS_READONLY, &nvs) == ESP_OK) {
        nvs_get_i32(nvs, axis[id].zero_key, &v);
        nvs_close(nvs);
    }
    return v >= 0 && v <= 1023 ? (int)v : axis[id].default_zero;
}

esp_err_t tr_servo_init(void)
{
    if (ready) {
        return ESP_OK;
    }
    nvs_flash_init(); // for the zero positions; does nothing if it is already set up
    bus_lock = xSemaphoreCreateMutex();
    i2c_master_bus_handle_t bus;
    ESP_RETURN_ON_ERROR(tr_board_i2c(&bus), TAG, "I2C");
    // The PY32 starts slowly, so the BSP tries for 1.2 s. It answers at 0x6F,
    // or at 0x71 when its ADD_SEL pin is high.
    uint8_t version = 0;
    for (int tries = 0; tries < 8 && !py32; tries++) {
        for (int a = 0; a < 2 && !py32; a++) {
            const uint8_t addr = a ? PY32_ADDR_ALT : PY32_ADDR;
            if (i2c_master_probe(bus, addr, 50) != ESP_OK) {
                continue;
            }
            const i2c_device_config_t dev = {
                .dev_addr_length = I2C_ADDR_BIT_LEN_7,
                .device_address = addr,
                .scl_speed_hz = 100000,
            };
            ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(bus, &dev, &py32), TAG, "PY32");
            uint8_t reg = PY32_VERSION;
            const esp_err_t r = py32_read(py32, reg, &version);
            ESP_LOGI(TAG, "PY32 at 0x%02X: %s, version 0x%02X", addr, esp_err_to_name(r), version);
            if (r != ESP_OK || version == 0 || version == 0xFF) {
                i2c_master_bus_rm_device(py32);
                py32 = NULL;
            }
        }
        if (!py32) {
            vTaskDelay(pdMS_TO_TICKS(200));
        }
    }
    if (!py32) {
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "PY32 version 0x%02X", version);
    // VM_EN: output with pull-up, as the BSP sets it. Only bit 0 changes.
    ESP_RETURN_ON_ERROR(py32_bit(PY32_MODE_L, VM_EN, true), TAG, "VM_EN mode");
    ESP_RETURN_ON_ERROR(py32_bit(PY32_PULLDN_L, VM_EN, false), TAG, "VM_EN pull");
    ESP_RETURN_ON_ERROR(py32_bit(PY32_PULLUP_L, VM_EN, true), TAG, "VM_EN pull");

    const uart_config_t uart = {
        .baud_rate = SERVO_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_RETURN_ON_ERROR(uart_driver_install(SERVO_UART, 256, 0, 0, NULL, 0), TAG, "UART");
    ESP_RETURN_ON_ERROR(uart_param_config(SERVO_UART, &uart), TAG, "UART config");
    ESP_RETURN_ON_ERROR(uart_set_pin(SERVO_UART, SERVO_TX, SERVO_RX, -1, -1), TAG, "UART pins");

    for (int id = TR_SERVO_YAW; id <= TR_SERVO_PITCH; id++) {
        zero[id] = load_zero(id);
        ESP_LOGI(TAG, "servo %d zero %d", id, zero[id]);
    }
    if (leds_init() != ESP_OK) {
        ESP_LOGW(TAG, "side LEDs did not answer");
    }
    ready = true;
    return ESP_OK;
}

esp_err_t tr_servo_read(tr_servo_t id, tr_servo_state_t *s)
{
    ESP_RETURN_ON_FALSE(ready, ESP_ERR_INVALID_STATE, TAG, "not started");
    uint8_t d[8], t, m;
    ESP_RETURN_ON_ERROR(scs_read(id, REG_POSITION, d, 8), TAG, "read %d", id);
    ESP_RETURN_ON_ERROR(scs_read(id, REG_TORQUE, &t, 1), TAG, "torque %d", id);
    ESP_RETURN_ON_ERROR(scs_read(id, REG_MOVING, &m, 1), TAG, "moving %d", id);
    s->raw = d[0] << 8 | d[1];
    s->degrees = (s->raw - zero[id]) / STEPS_PER_DEG;
    const int load = d[4] << 8 | d[5];
    s->load = load & 0x400 ? -(load & 0x3FF) : load & 0x3FF; // bit 10 is the sign
    s->volts_x10 = d[6];
    s->celsius = d[7];
    s->torque = t;
    s->moving = m;
    s->zero = zero[id];
    return ESP_OK;
}

esp_err_t tr_servo_move(tr_servo_t id, float degrees, int ms)
{
    ESP_RETURN_ON_FALSE(ready, ESP_ERR_INVALID_STATE, TAG, "not started");
    ESP_RETURN_ON_FALSE(isfinite(degrees), ESP_ERR_INVALID_ARG, TAG, "angle");
    // The safe range. Pitch outside 5..85 degrees can stall and damage the servo.
    degrees = fminf(fmaxf(degrees, axis[id].min_deg), axis[id].max_deg);
    ms = ms < 100 ? 100 : ms > 5000 ? 5000 : ms;
    int raw = zero[id] + (int)lroundf(degrees * STEPS_PER_DEG);
    raw = raw < 0 ? 0 : raw > 1023 ? 1023 : raw;
    const uint8_t goal[6] = {raw >> 8, raw & 0xFF, ms >> 8, ms & 0xFF, 0, 0};
    return scs_write(id, REG_GOAL, goal, sizeof(goal));
}

esp_err_t tr_servo_torque(tr_servo_t id, bool on)
{
    ESP_RETURN_ON_FALSE(ready, ESP_ERR_INVALID_STATE, TAG, "not started");
    const uint8_t v = on;
    return scs_write(id, REG_TORQUE, &v, 1);
}

esp_err_t tr_servo_set_zero(tr_servo_t id, int raw)
{
    ESP_RETURN_ON_FALSE(raw >= 0 && raw <= 1023, ESP_ERR_INVALID_ARG, TAG, "raw");
    nvs_handle_t nvs;
    ESP_RETURN_ON_ERROR(nvs_open("servo", NVS_READWRITE, &nvs), TAG, "NVS");
    esp_err_t err = nvs_set_i32(nvs, axis[id].zero_key, raw);
    if (err == ESP_OK) {
        err = nvs_commit(nvs);
    }
    nvs_close(nvs);
    if (err == ESP_OK) {
        zero[id] = raw;
    }
    return err;
}

// Side LEDs: 12 WS2812 driven by the PY32 on its pin 13, set up as M5Stack's
// factory firmware does (hal_io_expander.cpp). Colors go to the PY32's LED
// memory (0x30, RGB565, low byte first); writing bit 6 of LED_CFG (0x24)
// shows them.
#define PY32_LED_CFG  0x24
#define PY32_LED_RAM  0x30
#define PY32_MODE_H   0x04
#define PY32_PULLUP_H 0x0A
#define PY32_PULLDN_H 0x0C
#define PY32_DRIVE_H  0x14 // 1 = open drain
#define LED_PIN_H     0x20 // pin 13 is bit 5 of the high registers
#define LED_SHOW      0x40

static bool leds_ready;
static uint8_t led_ram[1 + 2 * TR_LED_COUNT] = {PY32_LED_RAM}; // register, then the colors

// Called from tr_servo_init(), once: pin 13 as a push-pull output with a
// pull-up, 12 LEDs, then the 200 ms the PY32 needs before the first show.
static esp_err_t leds_init(void)
{
    ESP_RETURN_ON_ERROR(py32_bit(PY32_MODE_H, LED_PIN_H, true), TAG, "LED pin");
    ESP_RETURN_ON_ERROR(py32_bit(PY32_PULLDN_H, LED_PIN_H, false), TAG, "LED pull");
    ESP_RETURN_ON_ERROR(py32_bit(PY32_PULLUP_H, LED_PIN_H, true), TAG, "LED pull");
    ESP_RETURN_ON_ERROR(py32_bit(PY32_DRIVE_H, LED_PIN_H, false), TAG, "LED drive");
    const uint8_t cfg[2] = {PY32_LED_CFG, TR_LED_COUNT};
    ESP_RETURN_ON_ERROR(i2c_master_transmit(py32, cfg, 2, 50), TAG, "LED count");
    vTaskDelay(pdMS_TO_TICKS(200));
    leds_ready = true;
    return tr_led_show(); // all off, so the LEDs start in a known state
}

// Only stores the color: no I2C until tr_led_show().
esp_err_t tr_led_set(int index, uint8_t r, uint8_t g, uint8_t b)
{
    ESP_RETURN_ON_FALSE(index >= 0 && index < TR_LED_COUNT, ESP_ERR_INVALID_ARG, TAG, "index");
    const uint16_t c = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
    led_ram[1 + 2 * index] = c & 0xFF;
    led_ram[2 + 2 * index] = c >> 8;
    return ESP_OK;
}

// The LEDs need the 5 V that the CoreS3 gives the body: BUS_OUT_EN (AW9523
// P0_1), then BOOST_EN (P1_7), with P1_2 and P1_3 as inputs, as the factory
// firmware sets them. The boost draws from the battery: on a nearly empty one
// (3.39 V at 390 mA) it made the AXP2101 switch the robot off. So we turn it on
// only with USB power or 3.6 V or more, and off again when no LED is lit.
#define AW_P0_BUS_OUT 0x02
#define AW_P1_BOOST   0x80
#define MIN_LED_MV    3600

static bool five_volts;

static esp_err_t leds_power(bool on)
{
    if (on && five_volts) {
        return ESP_OK;
    }
    if (!on) { // always: after a software restart the AW9523 keeps its pins
        five_volts = false;
        return tr_board_aw9523_off(AW_P0_BUS_OUT, AW_P1_BOOST);
    }
    uint8_t status = 0;
    tr_battery_t b = {0};
    const bool usb = tr_board_reg_read(0x34, 0x00, &status) == ESP_OK && (status & 0x20);
    if (!usb && (tr_battery_read(&b) != ESP_OK || b.millivolts < MIN_LED_MV)) {
        ESP_LOGW(TAG, "no 5 V for the LEDs: battery %d mV, no USB", b.millivolts);
        return ESP_ERR_INVALID_STATE;
    }
    uint8_t cfg1;
    if (tr_board_reg_read(0x58, 0x05, &cfg1) == ESP_OK && (cfg1 & 0x0C) != 0x0C) {
        tr_board_reg_write(0x58, 0x05, cfg1 | 0x0C); // P1_2, P1_3 inputs
    }
    ESP_RETURN_ON_ERROR(tr_board_aw9523_on(AW_P0_BUS_OUT, 0), TAG, "BUS_OUT_EN");
    ESP_RETURN_ON_ERROR(tr_board_aw9523_on(0, AW_P1_BOOST), TAG, "BOOST_EN");
    five_volts = true;
    return ESP_OK;
}

// All 12 colors in one I2C transfer, then the show bit. The show bit is always
// written, even if it still reads as set.
esp_err_t tr_led_show(void)
{
    ESP_RETURN_ON_FALSE(leds_ready, ESP_ERR_INVALID_STATE, TAG, "no LEDs");
    bool any = false;
    for (size_t i = 1; i < sizeof(led_ram); i++) {
        any |= led_ram[i] != 0;
    }
    if (any) {
        ESP_RETURN_ON_ERROR(leds_power(true), TAG, "5 V");
    }
    xSemaphoreTake(bus_lock, portMAX_DELAY);
    uint8_t reg = PY32_LED_CFG, cfg = TR_LED_COUNT;
    esp_err_t err = i2c_master_transmit(py32, led_ram, sizeof(led_ram), 50);
    if (err == ESP_OK) {
        err = py32_read(py32, reg, &cfg);
    }
    if (err == ESP_OK) {
        const uint8_t show[2] = {PY32_LED_CFG, (uint8_t)(cfg | LED_SHOW)};
        err = i2c_master_transmit(py32, show, 2, 50);
    }
    xSemaphoreGive(bus_lock);
    if (err == ESP_OK && !any) {
        err = leds_power(false); // all off: no need for the 5 V
    }
    return err;
}
