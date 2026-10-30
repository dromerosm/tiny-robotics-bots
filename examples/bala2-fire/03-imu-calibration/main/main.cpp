// 03-imu-calibration: calibrate the gyroscope at every boot and show the tilt.
//
// A gyroscope at rest does not read exactly 0: each axis has a small offset
// (bias). Integrate that bias to get an angle and the angle drifts away. So at
// boot, with the robot still, we average 1000 readings (2 seconds) and take
// the mean as the offset. If the robot moves, the readings spread out, and we
// try again.
//
// Then the tilt angle comes from a complementary filter: the gyro is good for
// fast changes, the accelerometer knows where "down" is, and we mix both.
// BALA2 falls forward and backward around the IMU's X axis, so the tilt is the
// roll angle, the same one M5Stack's balancing app uses.
//
// The IMU is an MPU6886 at I2C 0x68, set up like M5Stack's library does it:
// +-2000 deg/s and +-8 g. In the emulator there is no IMU, so read_imu() makes
// up readings and the output says "(sim)".
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_random.h"
#include <M5GFX.h>

static const char *TAG = "imu";

#define I2C_SDA      21 // Fire internal bus
#define I2C_SCL      22
#define I2C_HZ       400000
#define I2C_TIMEOUT  50 // ms
#define MPU6886_ADDR 0x68
#define MPU6886_ID   0x19 // WHO_AM_I answer

// MPU6886 registers
#define REG_SMPLRT_DIV   0x19
#define REG_CONFIG       0x1A
#define REG_GYRO_CONFIG  0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_XOUT_H 0x3B // then temperature, then gyro: 14 bytes
#define REG_PWR_MGMT_1   0x6B
#define REG_WHO_AM_I     0x75

#define GYRO_LSB_PER_DPS 16.4f   // +-2000 deg/s
#define ACCEL_LSB_PER_G  4096.0f // +-8 g

#define CAL_SAMPLES    1000
#define CAL_PERIOD_MS  2     // 1000 samples x 2 ms = 2 seconds
#define CAL_MAX_STDDEV 20.0f // raw units (about 1.2 deg/s); more means it moved
#define CAL_TRIES      5

#define LOOP_MS      10    // 100 Hz filter update
#define SHOW_EVERY   20    // screen and log every 200 ms
#define GYRO_WEIGHT  0.98f // complementary filter
#define RAD_TO_DEG_F 57.29578f

// Offsets saved by the factory firmware on our unit (NVS "Bala2Cal"), to compare.
#define FACTORY_X (-167)
#define FACTORY_Y 0
#define FACTORY_Z 74

typedef struct {
    int16_t ax, ay, az; // accelerometer, raw
    int16_t gx, gy, gz; // gyroscope, raw
    int16_t temp;       // raw
} imu_raw_t;

typedef struct {
    float offset[3]; // gyro x, y, z, raw units
    float stddev[3];
    float tilt_deg; // tilt from the accelerometer at rest
    float temp_c;
    int tries;
} calibration_t;

static M5GFX display;
static i2c_master_dev_handle_t imu;
static bool sim;

// --- IMU ---------------------------------------------------------------

static esp_err_t write_reg(uint8_t reg, uint8_t val)
{
    const uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(imu, buf, 2, I2C_TIMEOUT);
}

static esp_err_t read_regs(uint8_t reg, uint8_t *buf, size_t len)
{
    return i2c_master_transmit_receive(imu, &reg, 1, buf, len, I2C_TIMEOUT);
}

// Returns false if there is no MPU6886 (as in the emulator).
static bool imu_init(void)
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

    if (i2c_master_probe(bus, MPU6886_ADDR, I2C_TIMEOUT) != ESP_OK) {
        return false;
    }
    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address = MPU6886_ADDR;
    dev_cfg.scl_speed_hz = I2C_HZ;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &dev_cfg, &imu));

    uint8_t id = 0;
    read_regs(REG_WHO_AM_I, &id, 1);
    if (id != MPU6886_ID) {
        ESP_LOGW(TAG, "WHO_AM_I is 0x%02x, expected 0x%02x (MPU6886)", id, MPU6886_ID);
    }

    // Same order and values as M5Stack's MPU6886::Init().
    write_reg(REG_PWR_MGMT_1, 0x80); // reset
    vTaskDelay(pdMS_TO_TICKS(10));
    write_reg(REG_PWR_MGMT_1, 0x01); // clock from the gyro PLL
    vTaskDelay(pdMS_TO_TICKS(10));
    write_reg(REG_ACCEL_CONFIG, 0x10); // +-8 g
    write_reg(REG_GYRO_CONFIG, 0x18);  // +-2000 deg/s
    write_reg(REG_CONFIG, 0x01);       // low-pass filter, 1 kHz rate
    write_reg(REG_SMPLRT_DIV, 0x01);   // 500 Hz output
    vTaskDelay(pdMS_TO_TICKS(50));     // let the first samples settle
    return true;
}

// Fake readings for the emulator: a still robot leaning 1.5 degrees, with a
// gyro bias close to our unit's and some noise.
static void simulate_imu(imu_raw_t *r)
{
    const float lean = -1.5f / RAD_TO_DEG_F;
    auto noise = [](int amp) { return (int)(esp_random() % (2 * amp + 1)) - amp; };
    r->ax = noise(20);
    r->ay = (int16_t)(sinf(lean) * ACCEL_LSB_PER_G) + noise(20);
    r->az = (int16_t)(cosf(lean) * ACCEL_LSB_PER_G) + noise(20);
    r->gx = -160 + noise(8);
    r->gy = 4 + noise(8);
    r->gz = 70 + noise(8);
    r->temp = 1600; // about 30 C
}

static bool read_imu(imu_raw_t *r)
{
    if (sim) {
        simulate_imu(r);
        return true;
    }
    uint8_t b[14];
    if (read_regs(REG_ACCEL_XOUT_H, b, sizeof(b)) != ESP_OK) {
        return false;
    }
    // Big-endian 16-bit values: accel x, y, z, temperature, gyro x, y, z.
    r->ax = (int16_t)(b[0] << 8 | b[1]);
    r->ay = (int16_t)(b[2] << 8 | b[3]);
    r->az = (int16_t)(b[4] << 8 | b[5]);
    r->temp = (int16_t)(b[6] << 8 | b[7]);
    r->gx = (int16_t)(b[8] << 8 | b[9]);
    r->gy = (int16_t)(b[10] << 8 | b[11]);
    r->gz = (int16_t)(b[12] << 8 | b[13]);
    return true;
}

// Tilt from the accelerometer alone: where gravity points (same formula as
// M5Stack's balancing app).
static float accel_tilt_deg(const imu_raw_t *r)
{
    return atan2f(r->ay, sqrtf((float)r->ax * r->ax + (float)r->az * r->az)) * RAD_TO_DEG_F;
}

static float temp_c(int16_t raw)
{
    return raw / 326.8f + 25.0f; // MPU6886 datasheet formula
}

// --- Screen ------------------------------------------------------------

static void screen_title(const char *text, uint32_t color)
{
    display.fillScreen(TFT_BLACK);
    display.setTextColor(color, TFT_BLACK);
    display.setFont(&fonts::DejaVu18);
    display.drawString(text, 8, 6);
    display.setFont(&fonts::DejaVu12);
}

static void screen_progress(float done)
{
    display.drawRect(8, 120, 304, 16, TFT_WHITE);
    display.fillRect(10, 122, (int)(300 * done), 12, TFT_GREEN);
}

// One line of text that overwrites the old one (no flicker).
static void screen_line(int row, uint32_t color, const char *fmt, ...)
{
    char buf[64];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    display.setTextColor(color, TFT_BLACK);
    display.setTextPadding(304);
    display.drawString(buf, 8, 34 + row * 16);
}

// --- Calibration ---------------------------------------------------------

// Average CAL_SAMPLES gyro readings. Returns false if the robot moved.
static bool calibrate_once(calibration_t *cal)
{
    double sum[3] = {}, sum_sq[3] = {}, tilt = 0, temp = 0;
    TickType_t last = xTaskGetTickCount();

    for (int i = 0; i < CAL_SAMPLES; i++) {
        imu_raw_t r;
        if (!read_imu(&r)) {
            ESP_LOGE(TAG, "IMU read failed");
            return false;
        }
        const int16_t g[3] = {r.gx, r.gy, r.gz};
        for (int a = 0; a < 3; a++) {
            sum[a] += g[a];
            sum_sq[a] += (double)g[a] * g[a];
        }
        tilt += accel_tilt_deg(&r);
        temp += temp_c(r.temp);
        if (i % 50 == 0) {
            screen_progress((float)i / CAL_SAMPLES);
        }
        vTaskDelayUntil(&last, pdMS_TO_TICKS(CAL_PERIOD_MS));
    }
    screen_progress(1.0f);

    bool still = true;
    for (int a = 0; a < 3; a++) {
        const double mean = sum[a] / CAL_SAMPLES;
        cal->offset[a] = (float)mean;
        cal->stddev[a] = (float)sqrt(sum_sq[a] / CAL_SAMPLES - mean * mean);
        still = still && cal->stddev[a] < CAL_MAX_STDDEV;
    }
    cal->tilt_deg = (float)(tilt / CAL_SAMPLES);
    cal->temp_c = (float)(temp / CAL_SAMPLES);
    return still;
}

static void calibrate(calibration_t *cal)
{
    for (cal->tries = 1; cal->tries <= CAL_TRIES; cal->tries++) {
        screen_title("Calibrating gyro", TFT_YELLOW);
        screen_line(1, TFT_WHITE, "Keep BALA2 still for 2 seconds.");
        screen_line(2, TFT_DARKGREY, "Try %d of %d", cal->tries, CAL_TRIES);
        printf("calibrating, keep still (try %d of %d)\n", cal->tries, CAL_TRIES);

        if (calibrate_once(cal)) {
            return;
        }
        ESP_LOGW(TAG, "it moved (noise %.1f %.1f %.1f), trying again", cal->stddev[0],
                 cal->stddev[1], cal->stddev[2]);
        screen_line(4, TFT_RED, "It moved. Trying again...");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    cal->tries = CAL_TRIES;
    ESP_LOGW(TAG, "could not get a still reading, using the last one");
}

static void show_calibration(const calibration_t *cal)
{
    const char *src = sim ? " (sim)" : "";
    printf("calibrated%s after %d tries: offset x=%.1f y=%.1f z=%.1f raw, noise %.1f %.1f %.1f, "
           "tilt %.2f deg, %.1f C\n",
           src, cal->tries, cal->offset[0], cal->offset[1], cal->offset[2], cal->stddev[0],
           cal->stddev[1], cal->stddev[2], cal->tilt_deg, cal->temp_c);

    screen_title(sim ? "Gyro calibrated (sim)" : "Gyro calibrated", TFT_GREEN);
    screen_line(0, TFT_DARKGREY, "            x          y          z");
    screen_line(1, TFT_WHITE, "offset  %7.1f   %7.1f   %7.1f  raw", cal->offset[0], cal->offset[1],
                cal->offset[2]);
    screen_line(2, TFT_WHITE, "        %7.2f   %7.2f   %7.2f  deg/s",
                cal->offset[0] / GYRO_LSB_PER_DPS, cal->offset[1] / GYRO_LSB_PER_DPS,
                cal->offset[2] / GYRO_LSB_PER_DPS);
    screen_line(3, TFT_WHITE, "noise   %7.1f   %7.1f   %7.1f  raw", cal->stddev[0], cal->stddev[1],
                cal->stddev[2]);
    screen_line(4, TFT_DARKGREY, "factory %7d   %7d   %7d  raw", FACTORY_X, FACTORY_Y, FACTORY_Z);
    screen_line(5, TFT_WHITE, "rest tilt %.2f deg   temp %.1f C   tries %d", cal->tilt_deg,
                cal->temp_c, cal->tries);
}

// --- Main ----------------------------------------------------------------

extern "C" void app_main(void)
{
    display.init();
    display.setRotation(1);

    sim = !imu_init();
    if (sim) {
        ESP_LOGW(TAG, "no MPU6886 at 0x%02x (emulator?), using simulated readings", MPU6886_ADDR);
    } else {
        ESP_LOGI(TAG, "MPU6886 found at 0x%02x", MPU6886_ADDR);
    }

    calibration_t cal = {};
    calibrate(&cal);
    show_calibration(&cal);

    // Live tilt with the offsets removed.
    imu_raw_t r;
    read_imu(&r);
    float tilt = accel_tilt_deg(&r);
    TickType_t last = xTaskGetTickCount();
    const float dt = LOOP_MS / 1000.0f;

    for (int n = 0;; n++) {
        if (read_imu(&r)) {
            const float rate = (r.gx - cal.offset[0]) / GYRO_LSB_PER_DPS; // deg/s around X
            const float acc = accel_tilt_deg(&r);
            tilt = GYRO_WEIGHT * (tilt + rate * dt) + (1.0f - GYRO_WEIGHT) * acc;

            if (n % SHOW_EVERY == 0) {
                screen_line(7, TFT_CYAN, "tilt  %+7.2f deg   (accel only %+7.2f)", tilt, acc);
                screen_line(8, TFT_CYAN, "rate  %+7.2f deg/s around X", rate);
                printf("tilt %+7.2f deg  accel %+7.2f deg  rate %+7.2f deg/s\n", tilt, acc, rate);
            }
        }
        vTaskDelayUntil(&last, pdMS_TO_TICKS(LOOP_MS));
    }
}
