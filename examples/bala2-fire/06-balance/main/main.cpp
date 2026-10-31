// 06-balance: keep BALA2 upright, moving as little as possible.
//
// Based on M5Stack's balancing app (M5Bala2), with what we measured on our
// robot (see examples 03, 04 and 05):
//
//   - tilt from a complementary filter, gyro offsets measured at every boot
//   - D term from the gyro rate itself (less delay than a filtered angle)
//   - the motors act as a SPEED command: the wheels reach the speed we ask
//     for almost at once (example 05). So the law gives the wheel speed:
//
//         v_cmd = kp*tilt + ki*SUM(tilt*dt) + kd*rate + kv*speed + kx*position
//         PWM   = v_cmd / 0.00118 + 31   (the line measured in example 05,
//                                         which also jumps the dead band)
//
//     A PID on the tilt, plus speed and position terms. M5Stack's law is the
//     same without the I term: in our first test a robot that leaned 5
//     degrees just rolled on at a fixed speed. The I term makes the wheels
//     speed up while the robot still leans, until they get under it.
//     tilt in rad (0 = upright, + = leaning forward), rate in rad/s,
//     position in m, speed in m/s (+ = forward).
//   - the gains come from a search on a simulated robot, checked over many
//     guesses for the center of mass, motor delay and lag (see the README)
//   - the balance point trims itself: if the robot keeps drifting one way,
//     its 0 is a little off, and kt slowly moves it (up to 5 degrees)
//   - all gains can be changed over Wi-Fi or the serial port while it runs
//
// Safety first. The robot should rather fall over than drive away:
//
//   - the asked speed is limited to 0.5 m/s (0.55 at most over Wi-Fi), and
//     the PWM to 500 (600 at most)
//   - it only starts when you hold it upright and still for 1 second, and
//     then the speed limit ramps up over 1 second
//   - it stops (and stays stopped) if the tilt goes past 25 degrees, if it
//     moves more than 0.3 m away, if it goes faster than 0.4 m/s, if an I2C
//     read fails, or if the loop runs late. Button B stops it too.
//   - after a stop, lay it down before it can start again
//   - the base keeps the last speed it got, so we send 0 at boot (right
//     after the screen starts, about 0.3 s), and a stuck loop resets the
//     chip (task watchdog)
//   - over Wi-Fi: if the computer stops sending, it stops (see link.cpp)
//
// Buttons: B stops. A and C move the balance point by -/+0.25 degrees.
//
// In the emulator there is no robot. A rough simulated pendulum on wheels
// stands in for it. It checks the start, the soft start and the safety stops,
// not the gains: its center of mass and motor response are guesses.
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include <M5GFX.h>
#include "link.h"
#include "m5law.h"
#include "rec.h"

static const char *TAG = "balance";

// --- Hardware ------------------------------------------------------------

#define I2C_SDA      21
#define I2C_SCL      22
#define I2C_TIMEOUT  5 // ms, short: the loop has 5 ms
#define MPU6886_ADDR 0x68
#define BASE_ADDR    0x3A
#define IP5306_ADDR  0x75 // power chip, as in example 02
#define BTN_A        GPIO_NUM_39
#define BTN_B        GPIO_NUM_38
#define BTN_C        GPIO_NUM_37

#define REG_SMPLRT_DIV   0x19
#define REG_CONFIG       0x1A
#define REG_GYRO_CONFIG  0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_XOUT_H 0x3B
#define REG_PWR_MGMT_1   0x6B
#define REG_SPEED        0x00
#define REG_ENCODER      0x10

#define GYRO_LSB_PER_DPS 16.4f      // +-2000 deg/s
#define M_PER_COUNT      0.0003366f // 45 mm wheel, 420 counts per turn
#define RAD_TO_DEG_F     57.29578f

// --- Control -------------------------------------------------------------

#define LOOP_HZ 200
#define DT      (1.0f / LOOP_HZ)

#define GYRO_WEIGHT 0.98f    // complementary filter
#define SPEED_LPF   0.2f     // speed filter, like M5Stack's
#define M_S_PER_PWM 0.00118f // example 05: speed = 0.00118 * (PWM - 31)
#define PWM_OFFSET  31.0f
#define MIN_SPEED   0.005f // m/s; below this, ask for 0
#define TRIM_LIMIT  5.0f   // degrees
#define KX_LIMIT    0.094f // m/s, like M5Stack's 80 PWM
#define OUTER_LIMIT 5.0f   // degrees, most the position loop may lean the robot
#define TRACK_M     0.077f // m between the wheel centers (measured)
#define TURN_LIMIT  0.1f   // m/s, most the heading loop may add to one wheel
#define HEADING_ENC 0.01f  // heading filter: share of the encoder heading per tick
#define HARD_MAX_V  0.55f  // m/s, never more

// Safety limits. MAX_PWM is the start value; "max" over serial can change it
// up to HARD_MAX_PWM and never more.
#define MAX_PWM          500
#define HARD_MAX_PWM     600
#define RAMP_S           1.0f
#define FALL_DEG         25.0f
#define MAX_DISTANCE_M   0.3f
#define MAX_SPEED_MS     0.4f
#define MAX_SPEED_TICKS  60 // 0.3 s
#define LATE_US          15000
#define UPRIGHT_DEG      3.0f
#define UPRIGHT_RATE_DPS 15.0f
#define UPRIGHT_TICKS    LOOP_HZ // 1 s
#define LYING_DEG        45.0f
#define MIN_BATTERY      25    // %, the IP5306 gives 0, 25, 50, 75 or 100
#define M5_POINT_DEG     -1.5f // factory balance point, as saved in our unit

#define CAL_SAMPLES 1000

typedef enum { ST_WAIT, ST_BALANCE, ST_STOPPED, ST_FALL } state_t;

// Written by the control task, read by the screen and log task.
typedef struct {
    volatile state_t state;
    volatile float tilt, rate, speed, position, u, setpoint;
    volatile float heading_enc, heading, yaw_rate, turn; // deg, deg, deg/s, m/s
    volatile int battery;                                // %, -1 = unknown
    volatile bool charging;
    volatile bool seen_lying;
    const char *volatile reason;
} status_t;

static M5GFX display;
static i2c_master_dev_handle_t imu, base, power;
static bool have_power;
static bool sim;
static status_t st = {ST_WAIT, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, false, false, "boot"};
static float gyro_offset, yaw_offset, gy_offset;
// Last raw IMU and encoder values, for the factory law (m5law.cpp).
static float last_ax, last_ay, last_az, last_gy;
static int32_t last_enc_sum;
// Gains, changed over Wi-Fi or serial while it runs (handle_command()).
// Start values: see the README.
static struct {
    volatile float kp = 3.5f;  // m/s per rad of tilt (about 52 PWM per degree)
    volatile float ki = 10.0f; // m/s per (rad*s)
    volatile float kd = 0.07f; // m/s per rad/s (0.10 shook more, test 9)
    // Low-pass filter on the rate used by the D term: share of the new sample
    // per tick (0.32 is about 12 Hz, 0.45 about 19 Hz). Without it, a jerk of
    // the wheels shakes the body, the gyro sees it, and the D term answers
    // with another jerk: our second test shook at 12 Hz or more. But it also
    // delays the D term, and the robot shakes at 6-8 Hz. 0.45 shook least in
    // the sweep of test 13; 0.2 shook most.
    volatile float rf = 0.45f;
    volatile float kv = 0.3f;        // m/s per m/s
    volatile float kx = 0.5f;        // m/s per m, at most KX_LIMIT
    volatile float kpos = 10.0f;     // lean back this many degrees per m ahead
    volatile float kvel = 3.0f;      // and this many per m/s forward
    volatile float kt = -0.05f;      // slow balance point trim, rad per (m*s); off
    volatile float off = PWM_OFFSET; // dead band jump, PWM
    volatile float ramp = 0.3f;      // m/s over which the jump grows from 0 (test 13)
    volatile float kh = 0.1f;        // heading hold, m/s per rad (test 11)
    volatile float khr = 0.0f;       // heading rate damping, m/s per rad/s
    volatile float ys = -1.0f;       // sign of the gyro Z axis (test 10: opposite to
                                     // the encoders)
    volatile float nf = 0.0f;        // notch filter frequency, Hz (0 = off; try 8.5)
    volatile float nq = 2.0f;        // notch filter Q (higher = narrower)
    volatile int law = 0;            // 0 = ours, 1 = factory (M5Stack), to compare
    volatile float vmax = 0.5f;      // m/s, asked speed limit
    volatile float max = MAX_PWM;
} gain;

static volatile bool stop_request;
static volatile bool fall_request; // next start is a fall test, not a balance
static volatile bool ready;        // set by the control task after calibration
static volatile float setpoint_trim;

// --- Simulated robot (emulator only) --------------------------------------

#define SIM_L   0.06f // m, height of the center of mass (a guess)
#define SIM_TAU 0.08f // s, how fast a wheel reaches the commanded speed (a guess)

static struct {
    float th, thd; // tilt (rad) and rate
    float v, x;    // wheel speed and position
    bool held;     // someone holds it upright
    int16_t u;
} sm = {0, 0, 0, 0, true, 0};

// Wheel speed for a command, from example 05: dead band, then a straight line.
static float sim_motor_speed(int16_t u)
{
    const int mag = abs(u);
    if (mag < 55) {
        return 0;
    }
    const float v = 0.00118f * (mag - 31);
    return u > 0 ? v : -v;
}

static void sim_step(void)
{
    for (int i = 0; i < 5; i++) {
        const float dt = DT / 5;
        const float a = (sim_motor_speed(sm.u) - sm.v) / SIM_TAU;
        sm.v += a * dt;
        sm.x += sm.v * dt;
        if (sm.held) {
            sm.th = 0;
            sm.thd = 0;
        } else {
            const float thdd = (9.81f * sinf(sm.th) - a * cosf(sm.th)) / SIM_L;
            sm.thd += thdd * dt;
            sm.th += sm.thd * dt;
        }
    }
}

// --- I2C -------------------------------------------------------------------

static esp_err_t imu_write(uint8_t reg, uint8_t val)
{
    const uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(imu, buf, 2, 50);
}

static bool hw_init(void)
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

    if (i2c_master_probe(bus, BASE_ADDR, 50) != ESP_OK ||
        i2c_master_probe(bus, MPU6886_ADDR, 50) != ESP_OK) {
        return false;
    }
    i2c_device_config_t dev = {};
    dev.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev.device_address = BASE_ADDR;
    dev.scl_speed_hz = 100000; // what UiFlow2 uses
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &dev, &base));
    dev.device_address = MPU6886_ADDR;
    dev.scl_speed_hz = 400000;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &dev, &imu));
    if (i2c_master_probe(bus, IP5306_ADDR, 50) == ESP_OK) {
        dev.device_address = IP5306_ADDR;
        dev.scl_speed_hz = 100000;
        ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &dev, &power));
        have_power = true;
    }

    // MPU6886, as in example 03: +-2000 deg/s, +-8 g, 500 Hz.
    imu_write(REG_PWR_MGMT_1, 0x80);
    vTaskDelay(pdMS_TO_TICKS(10));
    imu_write(REG_PWR_MGMT_1, 0x01);
    vTaskDelay(pdMS_TO_TICKS(10));
    imu_write(REG_ACCEL_CONFIG, 0x10);
    imu_write(REG_GYRO_CONFIG, 0x18);
    imu_write(REG_CONFIG, 0x01);
    imu_write(REG_SMPLRT_DIV, 0x01);
    vTaskDelay(pdMS_TO_TICKS(50));
    return true;
}

static bool set_wheels(int16_t left, int16_t right)
{
    if (sim) {
        sm.u = (left + right) / 2;
        return true;
    }
    const uint8_t buf[5] = {REG_SPEED, (uint8_t)(left >> 8), (uint8_t)left, (uint8_t)(right >> 8),
                            (uint8_t)right};
    return i2c_master_transmit(base, buf, sizeof(buf), I2C_TIMEOUT) == ESP_OK;
}

static bool set_speed(int16_t u)
{
    return set_wheels(u, u);
}

// Tilt from the accelerometer (deg), gyro X rate (raw, the tilt axis) and
// gyro Z rate (raw, the yaw axis when the robot stands).
static bool read_imu(float *acc_tilt, float *gyro_raw, float *yaw_raw)
{
    if (sim) {
        *acc_tilt = sm.th * RAD_TO_DEG_F + ((int)(esp_random() % 41) - 20) * 0.01f;
        *gyro_raw = sm.thd * RAD_TO_DEG_F * GYRO_LSB_PER_DPS - 150 + (int)(esp_random() % 17) - 8;
        *yaw_raw = 60 + (int)(esp_random() % 17) - 8;
        last_ax = 0;
        last_ay = sinf(sm.th) * 4096;
        last_az = cosf(sm.th) * 4096;
        last_gy = 0;
        return true;
    }
    const uint8_t reg = REG_ACCEL_XOUT_H;
    uint8_t b[14];
    if (i2c_master_transmit_receive(imu, &reg, 1, b, sizeof(b), I2C_TIMEOUT) != ESP_OK) {
        return false;
    }
    const float ax = (int16_t)(b[0] << 8 | b[1]);
    const float ay = (int16_t)(b[2] << 8 | b[3]);
    const float az = (int16_t)(b[4] << 8 | b[5]);
    *acc_tilt = atan2f(ay, sqrtf(ax * ax + az * az)) * RAD_TO_DEG_F;
    *gyro_raw = (int16_t)(b[8] << 8 | b[9]);
    *yaw_raw = (int16_t)(b[12] << 8 | b[13]);
    last_ax = ax;
    last_ay = ay;
    last_az = az;
    last_gy = (int16_t)(b[10] << 8 | b[11]);
    return true;
}

// Forward position in meters (the mean of both wheels), and how far the right
// wheel is ahead of the left one. Both encoders count down going forward.
static bool read_position(float *pos, float *right_ahead)
{
    if (sim) {
        *pos = sm.x;
        *right_ahead = 0;
        return true;
    }
    const uint8_t reg = REG_ENCODER;
    uint8_t b[8];
    if (i2c_master_transmit_receive(base, &reg, 1, b, sizeof(b), I2C_TIMEOUT) != ESP_OK) {
        return false;
    }
    const int32_t l = (int32_t)((uint32_t)b[0] << 24 | b[1] << 16 | b[2] << 8 | b[3]);
    const int32_t r = (int32_t)((uint32_t)b[4] << 24 | b[5] << 16 | b[6] << 8 | b[7]);
    *pos = -(l + r) * 0.5f * M_PER_COUNT;
    *right_ahead = (l - r) * M_PER_COUNT;
    last_enc_sum = l + r;
    return true;
}

// Battery level in 25% steps and charging state, from the IP5306 (registers
// from example 02). Under load the battery voltage drops, so the level reads
// lower while the motors work.
static void read_battery(void)
{
    if (sim) {
        st.battery = 100;
        return;
    }
    if (!have_power) {
        return;
    }
    uint8_t r0, r3;
    uint8_t reg = 0x70;
    if (i2c_master_transmit_receive(power, &reg, 1, &r0, 1, I2C_TIMEOUT) != ESP_OK) {
        return;
    }
    reg = 0x78;
    if (i2c_master_transmit_receive(power, &reg, 1, &r3, 1, I2C_TIMEOUT) != ESP_OK) {
        return;
    }
    st.charging = r0 & 0x08;
    switch (r3 & 0xF0) {
        case 0x00:
            st.battery = 100;
            break;
        case 0x80:
            st.battery = 75;
            break;
        case 0xC0:
            st.battery = 50;
            break;
        case 0xE0:
            st.battery = 25;
            break;
        default:
            st.battery = 0;
    }
}

// --- Gyro calibration (as in example 03) -----------------------------------

// Returns false if the IMU cannot be read.
static bool calibrate_gyro(void)
{
    for (int tries = 1;; tries++) {
        double sum = 0, sum_sq = 0, sum_yaw = 0, sum_gy = 0;
        for (int i = 0; i < CAL_SAMPLES; i++) {
            float acc, g, yaw;
            if (sim) {
                sim_step();
            }
            if (!read_imu(&acc, &g, &yaw)) {
                printf("calibration: IMU read failed\n");
                return false;
            }
            sum += g;
            sum_sq += (double)g * g;
            sum_yaw += yaw;
            sum_gy += last_gy;
            vTaskDelay(pdMS_TO_TICKS(2));
        }
        const double mean = sum / CAL_SAMPLES;
        const double sd = sqrt(sum_sq / CAL_SAMPLES - mean * mean);
        if (sd < 20 || tries == 5) {
            gyro_offset = (float)mean;
            yaw_offset = (float)(sum_yaw / CAL_SAMPLES);
            gy_offset = (float)(sum_gy / CAL_SAMPLES);
            printf("gyro offset %.1f raw (noise %.1f), yaw offset %.1f raw, try %d\n", mean, sd,
                   yaw_offset, tries);
            return true;
        }
        printf("it moved (noise %.1f), calibrating again\n", sd);
    }
}

// --- Control task ----------------------------------------------------------

// Notch filter: removes one frequency and lets the rest through. Test 11 showed
// that 75% of the shaking is at 6-10 Hz, with a peak at 8.5 Hz, while the
// balance itself works below about 3 Hz. Biquad, direct form II transposed.
typedef struct {
    float b0, b1, b2, a1, a2, z1, z2, f, q;
} notch_t;

static float notch_run(notch_t *n, float x, float f, float q)
{
    if (f <= 0) {
        return x;
    }
    if (f != n->f || q != n->q) { // new settings: new coefficients
        const float w = 2 * (float)M_PI * f / LOOP_HZ;
        const float al = sinf(w) / (2 * q), c = cosf(w), a0 = 1 + al;
        n->b0 = 1 / a0;
        n->b1 = -2 * c / a0;
        n->b2 = 1 / a0;
        n->a1 = -2 * c / a0;
        n->a2 = (1 - al) / a0;
        n->f = f;
        n->q = q;
    }
    const float y = n->b0 * x + n->z1;
    n->z1 = n->b1 * x - n->a1 * y + n->z2;
    n->z2 = n->b2 * x - n->a2 * y;
    return y;
}

// PWM for a wheel speed, from the line measured in example 05.
static float pwm_for(float v)
{
    if (fabsf(v) <= MIN_SPEED) {
        return 0;
    }
    // The dead band jump grows smoothly from 0. If it grows too fast, the
    // gain near 0 is twice as high and the robot shakes around its balance
    // point (test 6, ramp 0.03).
    const float u =
        fabsf(v) / M_S_PER_PWM + gain.off * fminf(1.0f, fabsf(v) / fmaxf(gain.ramp, 0.001f));
    return v > 0 ? u : -u;
}

static void stop(const char *reason)
{
    set_speed(0);
    if (st.state != ST_STOPPED) {
        ESP_LOGW(TAG, "stop: %s", reason);
    }
    st.state = ST_STOPPED;
    st.u = 0;
    st.reason = reason;
    st.seen_lying = false;
}

static float clampf(float v, float lim)
{
    return v > lim ? lim : (v < -lim ? -lim : v);
}

static void control_task(void *arg)
{
    // Motors off before anything else: the base keeps its last command. The
    // I2C bus is made here, on core 1, so its interrupt runs on this core and
    // Wi-Fi (on core 0) cannot delay it.
    sim = !hw_init();
    set_speed(0);
    if (sim) {
        st.seen_lying = true; // the emulator script starts with it held upright
    }
    const bool imu_ok = calibrate_gyro();
    if (!imu_ok) {
        // Never start without a working IMU.
        st.state = ST_STOPPED;
        st.reason = "IMU error, restart";
    }
    ready = true;
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

    float acc, g, yaw, pos = 0, ahead = 0;
    read_imu(&acc, &g, &yaw);
    float tilt = acc, speed = 0, last_pos = 0, start_pos = 0, integral = 0, trim = 0, rate_f = 0;
    float start_ahead = 0, heading = 0; // heading in rad, + = turned left
    notch_t notch = {};
    read_position(&last_pos, &ahead);
    int upright_ticks = 0, fast_ticks = 0, balance_ticks = 0, errors = 0, battery_ticks = 0;
    int64_t last_us = esp_timer_get_time();
    TickType_t wake = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(1000 / LOOP_HZ));
        esp_task_wdt_reset();
        const int64_t now = esp_timer_get_time();
        const bool late = now - last_us > LATE_US;
        last_us = now;
        if (sim) {
            sim_step();
        }

        // Sense. The battery once a second.
        if (++battery_ticks >= LOOP_HZ) {
            battery_ticks = 0;
            read_battery();
        }
        if (!read_imu(&acc, &g, &yaw) || !read_position(&pos, &ahead)) {
            if (++errors >= 3 && st.state == ST_BALANCE) {
                stop("I2C error");
            }
            continue;
        }
        errors = 0;
        const float rate = (g - gyro_offset) / GYRO_LSB_PER_DPS;
        // Factory law's angle filter runs all the time, so it has settled
        // when a start with "law 1" comes.
        m5_filter(last_ax, last_ay, last_az, rate, (last_gy - gy_offset) / GYRO_LSB_PER_DPS,
                  (yaw - yaw_offset) / GYRO_LSB_PER_DPS, DT);
        const float yaw_rate = gain.ys * (yaw - yaw_offset) / GYRO_LSB_PER_DPS; // deg/s
        rate_f += gain.rf * (rate - rate_f);
        tilt = GYRO_WEIGHT * (tilt + rate * DT) + (1 - GYRO_WEIGHT) * acc;
        speed = (1 - SPEED_LPF) * speed + SPEED_LPF * (pos - last_pos) / DT;
        last_pos = pos;
        const float setpoint = setpoint_trim + trim;
        const float err = tilt - setpoint;

        st.tilt = tilt;
        st.rate = rate;
        st.speed = speed;
        st.setpoint = setpoint;
        if (fabsf(tilt) > LYING_DEG) {
            st.seen_lying = true;
        }

        if (!imu_ok) {
            set_speed(0);
            continue;
        }
        switch (st.state) {
            case ST_STOPPED:
                set_speed(0); // the base keeps the last command, so repeat it
                if (st.seen_lying) {
                    st.state = ST_WAIT;
                    st.reason = "stand me up";
                }
                break;

            case ST_WAIT:
                set_speed(0);
                st.u = 0;
                // Do not start on a nearly empty battery (unless it charges): the
                // motors get weak, and the robot can switch off mid-balance (it
                // did after about 4 hours of tests).
                if (st.battery >= 0 && st.battery < MIN_BATTERY && !st.charging) {
                    st.reason = "battery low, charge me";
                    upright_ticks = 0;
                    break;
                }
                st.reason = "stand me up";
                // Start only after it was lying down, then held upright and still.
                if (st.seen_lying && fabsf(err) < UPRIGHT_DEG && fabsf(rate) < UPRIGHT_RATE_DPS) {
                    upright_ticks++;
                } else {
                    upright_ticks = 0;
                }
                if (upright_ticks >= UPRIGHT_TICKS) {
                    upright_ticks = 0;
                    balance_ticks = 0;
                    fast_ticks = 0;
                    start_pos = pos;
                    start_ahead = ahead;
                    heading = 0;
                    speed = 0;
                    integral = 0;
                    notch.z1 = notch.z2 = 0;
                    trim = 0;
                    stop_request = false;
                    if (fall_request) {
                        // Fall test: record 4 s at 200 Hz from now; let go.
                        fall_request = false;
                        rec_start(4 * LOOP_HZ);
                        st.state = ST_FALL;
                        st.reason = "fall test: let go";
                        ESP_LOGI(TAG, "fall test");
                        break;
                    }
                    m5_start(last_enc_sum);
                    st.state = ST_BALANCE;
                    st.reason = gain.law == 1 ? "balancing (factory law)" : "balancing";
                    ESP_LOGI(TAG, "start, law %d", gain.law);
                    if (sim) {
                        sm.held = false; // let go
                    }
                }
                break;

            case ST_BALANCE: {
                balance_ticks++;
                const float x = pos - start_pos;
                st.position = x;
                fast_ticks = fabsf(speed) > MAX_SPEED_MS ? fast_ticks + 1 : 0;

                if (stop_request) {
                    stop("button B");
                } else if (fabsf(tilt - setpoint_trim) > FALL_DEG) {
                    stop("fell over");
                } else if (fabsf(x) > MAX_DISTANCE_M) {
                    stop("moved too far");
                } else if (fast_ticks > MAX_SPEED_TICKS) {
                    stop("too fast");
                } else if (late && !sim) { // QEMU does not run in real time
                    stop("loop late");
                } else if (!link_alive()) {
                    stop("Wi-Fi link lost");
                }
                if (st.state != ST_BALANCE) {
                    break;
                }

                // The balance point follows a slow drift (kt), up to 5 degrees.
                trim = clampf(trim + gain.kt * x * DT * RAD_TO_DEG_F, TRIM_LIMIT);
                // Position loop: when the robot is ahead, or rolls forward, ask
                // it to lean back a little, so it comes back to where it started.
                const float outer = clampf(-gain.kpos * x - gain.kvel * speed, OUTER_LIMIT);
                st.setpoint = setpoint + outer;

                // Heading: the gyro is good over short times and does not care
                // if a wheel slips; the encoders do not drift. Mix both.
                const float heading_enc = (ahead - start_ahead) / TRACK_M;
                heading = (1 - HEADING_ENC) * (heading + yaw_rate / RAD_TO_DEG_F * DT) +
                          HEADING_ENC * heading_enc;
                // Heading hold: the two motors are not the same (the left one is
                // 3-5% faster, example 05), so the robot slowly turns. Give the
                // right wheel a little more when it has turned right, and less
                // when it has turned left.
                const float turn =
                    clampf(-(gain.kh * heading + gain.khr * yaw_rate / RAD_TO_DEG_F), TURN_LIMIT);
                st.heading_enc = heading_enc * RAD_TO_DEG_F;
                st.heading = heading * RAD_TO_DEG_F;
                st.yaw_rate = yaw_rate;
                st.turn = turn;

                // Soft start: the speed limit grows from 0 in RAMP_S.
                const float ramp = fminf(1.0f, balance_ticks * DT / RAMP_S);
                const float vlim = fminf(gain.vmax, HARD_MAX_V) * ramp;
                const float th = (err - outer) / RAD_TO_DEG_F;
                integral = clampf(integral + gain.ki * th * DT, vlim);
                const float v_pid =
                    clampf(gain.kp * th + integral + gain.kd * rate_f / RAD_TO_DEG_F +
                               gain.kv * speed + clampf(gain.kx * x, KX_LIMIT),
                           vlim);
                const float v_cmd =
                    clampf(notch_run(&notch, v_pid, gain.nf, fmaxf(gain.nq, 0.3f)), vlim);
                const float lim = fminf(gain.max, HARD_MAX_PWM);
                float ul = clampf(pwm_for(v_cmd - turn), lim);
                float ur = clampf(pwm_for(v_cmd + turn), lim);
                if (gain.law == 1) {
                    // Factory law instead of ours, with our limits. Same PWM on
                    // both wheels, as in the original.
                    const float m5 = m5_control(last_enc_sum, M5_POINT_DEG + setpoint_trim);
                    ul = ur = clampf(m5, lim * ramp);
                }
                st.u = (ul + ur) / 2;
                if (!set_wheels((int16_t)ul, (int16_t)ur) && ++errors >= 3) {
                    stop("I2C error");
                }
                break;
            }

            case ST_FALL: {
                // Fall test: the wheels hold their place (slowly, at most
                // 0.1 m/s), so the body turns around the axle like a plain
                // inverted pendulum. We record how fast the tilt grows.
                const float x = pos - start_pos;
                st.position = x;
                const float u = clampf(pwm_for(clampf(-3.0f * x, 0.1f)), 200);
                st.u = u;
                set_speed((int16_t)u);
                if (fabsf(tilt) > 40 || !rec_active() || stop_request) {
                    stop("fall test done");
                }
                break;
            }
        }

        // 200 Hz recorder (rec.h): every tick while it records.
        if (rec_active()) {
            const rec_sample_t s = {
                (uint32_t)(now / 1000),        tilt, rate, acc, st.position, speed, st.u,
                gain.law == 1 ? m5_angle() : 0};
            rec_add(&s);
        }
    }
}

// --- Screen, buttons and log -----------------------------------------------

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

static bool pressed(gpio_num_t pin, int *last)
{
    const int now = gpio_get_level(pin);
    const bool down = *last == 1 && now == 0;
    *last = now;
    return down;
}

// In the emulator: hold it upright, let go, push it, and check it stays up.
static void sim_script(int64_t ms)
{
    static int64_t start_ms = -1;
    static bool pushed, done;
    if (st.state == ST_BALANCE && start_ms < 0) {
        start_ms = ms;
    }
    if (start_ms >= 0 && !pushed && ms - start_ms > 3000) {
        pushed = true;
        sm.thd += 0.5f; // a push forward: 29 deg/s
        printf("sim: push\n");
    }
    if (pushed && !done && ms - start_ms > 6000) {
        done = true;
        printf("sim: done, %s\n", st.reason);
    }
}

// Print a line on the serial port and send it over Wi-Fi.
static void say(const char *fmt, ...)
{
    char line[256];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);
    fputs(line, stdout);
    link_send(line);
}

static void say_gains(void)
{
    say("gains kp %.2f ki %.1f kd %.3f rf %.2f kv %.2f kx %.2f kpos %.1f kvel %.1f kt %.3f "
        "off %.0f ramp %.3f kh %.2f khr %.2f ys %.0f nf %.1f nq %.1f law %d vmax %.2f max %.0f\n",
        gain.kp, gain.ki, gain.kd, gain.rf, gain.kv, gain.kx, gain.kpos, gain.kvel, gain.kt,
        gain.off, gain.ramp, gain.kh, gain.khr, gain.ys, gain.nf, gain.nq, gain.law, gain.vmax,
        gain.max);
}

// Change a gain while it runs, from the serial monitor or over Wi-Fi:
//     kp 3.5  ki 10  kd 0.07  rf 0.45  kv 0.3  kx 0.5  kpos 10  kvel 3  kt -0.05  off 31
//     ramp 0.3  kh 0.1  khr 0  ys -1  nf 8.5  nq 2
//     vmax 0.4  max 500  ?  stop
// "hb" (heartbeat) only keeps the Wi-Fi link alive. Each change is printed, so
// the log shows which values were used.
static void handle_command(const char *line)
{
    char name[8];
    float v;
    const int n = sscanf(line, "%7s %f", name, &v);
    if (n < 1 || !strcmp(name, "hb")) {
        return;
    }
    if (!strcmp(name, "stop")) {
        stop_request = true;
        return;
    }
    if (!strcmp(name, "fall")) {
        fall_request = true;
        say("fall test: lay it down, then hold it upright and still, and let go\n");
        return;
    }
    if (n == 2 && !strcmp(name, "rec")) {
        rec_start((int)(v * LOOP_HZ));
        say("recording %.1f s at %d Hz\n", v, LOOP_HZ);
        return;
    }
    if (n == 2 && !strcmp(name, "kp")) {
        gain.kp = v;
    } else if (n == 2 && !strcmp(name, "ki")) {
        gain.ki = v;
    } else if (n == 2 && !strcmp(name, "kd")) {
        gain.kd = v;
    } else if (n == 2 && !strcmp(name, "kx")) {
        gain.kx = v;
    } else if (n == 2 && !strcmp(name, "kv")) {
        gain.kv = v;
    } else if (n == 2 && !strcmp(name, "kt")) {
        gain.kt = v;
    } else if (n == 2 && !strcmp(name, "kpos")) {
        gain.kpos = v;
    } else if (n == 2 && !strcmp(name, "kvel")) {
        gain.kvel = v;
    } else if (n == 2 && !strcmp(name, "off")) {
        gain.off = fminf(fmaxf(v, 0), 60);
    } else if (n == 2 && !strcmp(name, "kh")) {
        gain.kh = v;
    } else if (n == 2 && !strcmp(name, "khr")) {
        gain.khr = v;
    } else if (n == 2 && !strcmp(name, "ys")) {
        gain.ys = v < 0 ? -1.0f : 1.0f;
    } else if (n == 2 && !strcmp(name, "nf")) {
        gain.nf = fminf(fmaxf(v, 0), 40);
    } else if (n == 2 && !strcmp(name, "law")) {
        gain.law = v >= 1 ? 1 : 0;
    } else if (n == 2 && !strcmp(name, "rf")) {
        gain.rf = fminf(fmaxf(v, 0.05f), 1);
    } else if (n == 2 && !strcmp(name, "nq")) {
        gain.nq = fminf(fmaxf(v, 0.3f), 10);
    } else if (n == 2 && !strcmp(name, "ramp")) {
        gain.ramp = fminf(fmaxf(v, 0.001f), 0.5f);
    } else if (n == 2 && !strcmp(name, "vmax")) {
        gain.vmax = fminf(fmaxf(v, 0), HARD_MAX_V);
    } else if (n == 2 && !strcmp(name, "max")) {
        gain.max = fminf(fmaxf(v, 0), HARD_MAX_PWM);
    } else if (strcmp(name, "?")) {
        say("unknown command: %s\n", line);
        return;
    }
    say_gains();
}

static void read_serial(void)
{
    static char line[32];
    static int len;
    uint8_t ch;
    while (uart_read_bytes(UART_NUM_0, &ch, 1, 0) == 1) {
        if (ch != '\n' && ch != '\r') {
            if (len < (int)sizeof(line) - 1) {
                line[len++] = (char)ch;
            }
            continue;
        }
        line[len] = 0;
        len = 0;
        handle_command(line);
    }
}

// Before an update: stop the motors and wait until the loop agrees.
static bool stop_for_update(void)
{
    stop_request = true;
    for (int i = 0; i < 20 && st.state == ST_BALANCE; i++) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return st.state != ST_BALANCE;
}

extern "C" void app_main(void)
{
    // The screen first: M5GFX looks at the I2C pins to find out which board
    // it is on, so it must finish before our I2C bus starts.
    display.init();
    // The control loop runs alone on core 1, so the screen and Wi-Fi cannot
    // slow it down. It sets up the I2C bus and stops the motors first.
    xTaskCreatePinnedToCore(control_task, "control", 4096, NULL, 20, NULL, 1);

    display.setRotation(1);
    display.fillScreen(TFT_BLACK);
    display.setFont(&fonts::DejaVu18);
    display.setTextColor(TFT_WHITE, TFT_BLACK);
    display.drawString("BALA2 balance", 8, 6);
    display.setFont(&fonts::DejaVu12);
    screen_line(40, TFT_YELLOW, "Calibrating. Keep it lying still...");
    while (!ready) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    if (sim) {
        ESP_LOGW(TAG, "no IMU or base (emulator?), using a simulated robot");
        display.drawString("BALA2 balance (sim)", 8, 6);
    } else {
        // No Wi-Fi in the emulator.
        const link_callbacks_t cb = {handle_command, stop_for_update};
        link_start(&cb);
        printf("Wi-Fi network %s, password %s, robot at %s, UDP port %d\n", link_ssid(),
               link_password(), LINK_IP, LINK_UDP_PORT);
    }

    gpio_config_t cfg = {};
    cfg.pin_bit_mask = (1ULL << BTN_A) | (1ULL << BTN_B) | (1ULL << BTN_C);
    cfg.mode = GPIO_MODE_INPUT;
    ESP_ERROR_CHECK(gpio_config(&cfg));

    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0));
    say_gains();
    say("t_ms,state,tilt,rate,speed,position,u,setpoint,heading_enc,heading,yaw_rate,turn,"
        "battery,charging\n");
    int last_a = 1, last_b = 1, last_c = 1;
    for (int n = 0;; n++) {
        const int64_t ms = esp_timer_get_time() / 1000;
        read_serial();
        if (sim) {
            sim_script(ms);
        } else {
            if (pressed(BTN_B, &last_b)) {
                stop_request = true;
            }
            if (pressed(BTN_A, &last_a)) {
                setpoint_trim -= 0.25f;
            }
            if (pressed(BTN_C, &last_c)) {
                setpoint_trim += 0.25f;
            }
        }
        // Send the 200 Hz recording, a few lines at a time.
        char rline[96];
        for (int k = 0; k < 8 && rec_next_line(rline, sizeof(rline)); k++) {
            say("%s", rline);
        }
        { // 50 Hz log
            say("%lld,%d,%.2f,%.1f,%.3f,%.3f,%.0f,%.2f,%.1f,%.1f,%.1f,%.3f,%d,%d\n", ms,
                (int)st.state, st.tilt, st.rate, st.speed, st.position, st.u, st.setpoint,
                st.heading_enc, st.heading, st.yaw_rate, st.turn, st.battery, (int)st.charging);
        }
        if (n % 10 == 0) { // 5 Hz screen
            const uint32_t color = st.state == ST_BALANCE ? TFT_GREEN
                                   : st.state == ST_WAIT  ? TFT_YELLOW
                                                          : TFT_RED;
            display.setFont(&fonts::DejaVu18);
            screen_line(40, color, "%s", st.reason);
            display.setFont(&fonts::DejaVu12);
            screen_line(80, TFT_WHITE, "tilt     %+7.2f deg", st.tilt);
            screen_line(98, TFT_WHITE, "position %+7.3f m", st.position);
            screen_line(116, TFT_WHITE, "motor    %+5.0f / %.0f", st.u, gain.max);
            screen_line(134, TFT_DARKGREY, "balance point %+.2f deg (A -, C +)", st.setpoint);
            screen_line(152, st.battery >= 0 && st.battery < MIN_BATTERY ? TFT_RED : TFT_DARKGREY,
                        "battery %d%%%s", st.battery, st.charging ? ", charging" : "");
            screen_line(170, TFT_DARKGREY, "B: stop");
            if (!sim) {
                screen_line(196, TFT_CYAN, "Wi-Fi %s  pass %s", link_ssid(), link_password());
                screen_line(214, TFT_CYAN, "%s  UDP %d  %s", LINK_IP, LINK_UDP_PORT,
                            link_alive() ? "" : "LINK LOST");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
