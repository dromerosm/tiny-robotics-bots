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

static const char *TAG = "balance";

// --- Hardware ------------------------------------------------------------

#define I2C_SDA      21
#define I2C_SCL      22
#define I2C_TIMEOUT  5 // ms, short: the loop has 5 ms
#define MPU6886_ADDR 0x68
#define BASE_ADDR    0x3A
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
// Low-pass filter on the rate used by the D term, at about 15 Hz. Without it,
// a jerk of the wheels shakes the body, the gyro sees it, and the D term
// answers with another jerk: our second test shook at 12 Hz or more.
#define RATE_LPF   0.32f
#define HARD_MAX_V 0.55f // m/s, never more

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

#define CAL_SAMPLES 1000

typedef enum { ST_WAIT, ST_BALANCE, ST_STOPPED } state_t;

// Written by the control task, read by the screen and log task.
typedef struct {
    volatile state_t state;
    volatile float tilt, rate, speed, position, u, setpoint;
    volatile bool seen_lying;
    const char *volatile reason;
} status_t;

static M5GFX display;
static i2c_master_dev_handle_t imu, base;
static bool sim;
static status_t st = {ST_WAIT, 0, 0, 0, 0, 0, 0, false, "boot"};
static float gyro_offset;
// Gains, changed over Wi-Fi or serial while it runs (handle_command()).
// Start values: see the README.
static struct {
    volatile float kp = 3.5f;        // m/s per rad of tilt (about 52 PWM per degree)
    volatile float ki = 10.0f;       // m/s per (rad*s)
    volatile float kd = 0.07f;       // m/s per rad/s (0.10 shook more, test 9)
    volatile float kv = 0.3f;        // m/s per m/s
    volatile float kx = 0.5f;        // m/s per m, at most KX_LIMIT
    volatile float kpos = 10.0f;     // lean back this many degrees per m ahead
    volatile float kvel = 3.0f;      // and this many per m/s forward
    volatile float kt = 0.0f;        // slow balance point trim, rad per (m*s); off
    volatile float off = PWM_OFFSET; // dead band jump, PWM
    volatile float ramp = 0.1f;      // m/s over which the jump grows from 0
    volatile float vmax = 0.5f;      // m/s, asked speed limit
    volatile float max = MAX_PWM;
} gain;

static volatile bool stop_request;
static volatile bool ready; // set by the control task after calibration
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

static bool set_speed(int16_t u)
{
    if (sim) {
        sm.u = u;
        return true;
    }
    const uint8_t buf[5] = {REG_SPEED, (uint8_t)(u >> 8), (uint8_t)u, (uint8_t)(u >> 8),
                            (uint8_t)u};
    return i2c_master_transmit(base, buf, sizeof(buf), I2C_TIMEOUT) == ESP_OK;
}

// Tilt from the accelerometer (deg) and gyro X rate (raw).
static bool read_imu(float *acc_tilt, float *gyro_raw)
{
    if (sim) {
        *acc_tilt = sm.th * RAD_TO_DEG_F + ((int)(esp_random() % 41) - 20) * 0.01f;
        *gyro_raw = sm.thd * RAD_TO_DEG_F * GYRO_LSB_PER_DPS - 150 + (int)(esp_random() % 17) - 8;
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
    return true;
}

// Forward position in meters: both encoders count down going forward.
static bool read_position(float *pos)
{
    if (sim) {
        *pos = sm.x;
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
    return true;
}

// --- Gyro calibration (as in example 03) -----------------------------------

// Returns false if the IMU cannot be read.
static bool calibrate_gyro(void)
{
    for (int tries = 1;; tries++) {
        double sum = 0, sum_sq = 0;
        for (int i = 0; i < CAL_SAMPLES; i++) {
            float acc, g;
            if (sim) {
                sim_step();
            }
            if (!read_imu(&acc, &g)) {
                printf("calibration: IMU read failed\n");
                return false;
            }
            sum += g;
            sum_sq += (double)g * g;
            vTaskDelay(pdMS_TO_TICKS(2));
        }
        const double mean = sum / CAL_SAMPLES;
        const double sd = sqrt(sum_sq / CAL_SAMPLES - mean * mean);
        if (sd < 20 || tries == 5) {
            gyro_offset = (float)mean;
            printf("gyro offset %.1f raw (noise %.1f), try %d\n", mean, sd, tries);
            return true;
        }
        printf("it moved (noise %.1f), calibrating again\n", sd);
    }
}

// --- Control task ----------------------------------------------------------

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

    float acc, g, pos = 0;
    read_imu(&acc, &g);
    float tilt = acc, speed = 0, last_pos = 0, start_pos = 0, integral = 0, trim = 0, rate_f = 0;
    read_position(&last_pos);
    int upright_ticks = 0, fast_ticks = 0, balance_ticks = 0, errors = 0;
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

        // Sense.
        if (!read_imu(&acc, &g) || !read_position(&pos)) {
            if (++errors >= 3 && st.state == ST_BALANCE) {
                stop("I2C error");
            }
            continue;
        }
        errors = 0;
        const float rate = (g - gyro_offset) / GYRO_LSB_PER_DPS;
        rate_f += RATE_LPF * (rate - rate_f);
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
                    speed = 0;
                    integral = 0;
                    trim = 0;
                    stop_request = false;
                    st.state = ST_BALANCE;
                    st.reason = "balancing";
                    ESP_LOGI(TAG, "start");
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

                // Soft start: the speed limit grows from 0 in RAMP_S.
                const float ramp = fminf(1.0f, balance_ticks * DT / RAMP_S);
                const float vlim = fminf(gain.vmax, HARD_MAX_V) * ramp;
                const float th = (err - outer) / RAD_TO_DEG_F;
                integral = clampf(integral + gain.ki * th * DT, vlim);
                const float v_cmd =
                    clampf(gain.kp * th + integral + gain.kd * rate_f / RAD_TO_DEG_F +
                               gain.kv * speed + clampf(gain.kx * x, KX_LIMIT),
                           vlim);
                float u = 0;
                if (fabsf(v_cmd) > MIN_SPEED) {
                    // The dead band jump grows smoothly from 0. If it grows too
                    // fast, the gain near 0 is twice as high and the robot
                    // shakes around its balance point (test 6, ramp 0.03).
                    u = fabsf(v_cmd) / M_S_PER_PWM +
                        gain.off * fminf(1.0f, fabsf(v_cmd) / fmaxf(gain.ramp, 0.001f));
                    u = v_cmd > 0 ? u : -u;
                }
                u = clampf(u, fminf(gain.max, HARD_MAX_PWM));
                st.u = u;
                if (!set_speed((int16_t)u) && ++errors >= 3) {
                    stop("I2C error");
                }
                break;
            }
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
    char line[160];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);
    fputs(line, stdout);
    link_send(line);
}

static void say_gains(void)
{
    say("gains kp %.2f ki %.1f kd %.3f kv %.2f kx %.2f kpos %.1f kvel %.1f kt %.3f off %.0f "
        "ramp %.3f vmax %.2f max %.0f\n",
        gain.kp, gain.ki, gain.kd, gain.kv, gain.kx, gain.kpos, gain.kvel, gain.kt, gain.off,
        gain.ramp, gain.vmax, gain.max);
}

// Change a gain while it runs, from the serial monitor or over Wi-Fi:
//     kp 3.5  ki 10  kd 0.1  kv 0.3  kx 0.5  kpos 10  kvel 3  kt 0  off 31  ramp 0.1
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
    say("t_ms,state,tilt,rate,speed,position,u,setpoint\n");
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
        { // 50 Hz log
            say("%lld,%d,%.2f,%.1f,%.3f,%.3f,%.0f,%.2f\n", ms, (int)st.state, st.tilt, st.rate,
                st.speed, st.position, st.u, st.setpoint);
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
