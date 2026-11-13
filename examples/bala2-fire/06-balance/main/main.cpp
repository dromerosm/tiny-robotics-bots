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
//         e     = tilt - setpoint,  setpoint = -(kpos*position + kvel*speed)
//         v_cmd = kp*e + ki*SUM(e*dt) + kd*rate
//         PWM   = v_cmd / 0.00118 + 31   (the line measured in example 05,
//                                         which also jumps the dead band)
//
//     A PID on the tilt. The position and the speed move its setpoint, so
//     the robot leans back when it is ahead or rolls forward. M5Stack's law
//     has no I term: in our first test a robot that leaned 5
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
// "drive <m/s> <deg/s> [seconds]" moves the robot while it balances: it moves
// the place and the heading the robot holds (see the README).
//
// A Servo Kit 360 servo on the base's S4 header takes "servo <pulse>" (see
// the README). It is not part of the balance.
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
#include "esp_app_desc.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_pm.h"
#include "esp_system.h"
#include "esp_random.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "nvs.h"
#include <M5GFX.h>
#include "bala2_link.h"
#include "bala2_mon.h"

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
#define REG_SERVO_US     0x30 // + 2 * (channel - 1): pulse in us, big-endian

// Servo Kit 360 on the base's S4 header: a speed servo. 1500 us is stop,
// 500 and 2500 are full speed, one each way. The base makes the 50 Hz signal.
#define SERVO_CH      4
#define SERVO_STOP_US 1500
#define SERVO_MIN_US  500
#define SERVO_MAX_US  2500

#define GYRO_LSB_PER_DPS 16.4f      // +-2000 deg/s
#define M_PER_COUNT      0.0003366f // 45 mm wheel, 420 counts per turn
#define RAD_TO_DEG_F     57.29578f

// --- Control -------------------------------------------------------------

#define LOOP_HZ 200
#define DT      (1.0f / LOOP_HZ)

#define SPEED_LPF   0.2f     // speed filter, like M5Stack's
#define M_S_PER_PWM 0.00118f // example 05: speed = 0.00118 * (PWM - 31)
#define PWM_OFFSET  31.0f
#define MIN_SPEED   0.005f // m/s; below this, ask for 0
#define TRIM_LIMIT  5.0f   // degrees
#define OUTER_LIMIT 5.0f   // degrees, most the position loop may lean the robot
#define TRACK_M     0.077f // m between the wheel centers (measured)
#define TURN_LIMIT  0.1f   // m/s, most the heading loop may add to one wheel
#define HEADING_ENC 0.01f  // heading filter: share of the encoder heading per tick
#define HARD_MAX_V  0.55f  // m/s, never more
#define KW_LIMIT    0.1f   // m/s, most the wheel position loop may add
#define YAW_SIGN    -1.0f  // gyro Z has the opposite sign to the encoders (test 10)
// Shake meter: the part of the rate between about 5 and 10 Hz (the gap of two
// low-pass filters), then its mean size over about 1 s. The gap passes a third
// of a 7 Hz wave, so SHAKE_GAIN brings it back to deg/s.
#define SHAKE_FAST  0.27f  // share per tick, about 10 Hz
#define SHAKE_SLOW  0.145f // share per tick, about 5 Hz
#define SHAKE_GAIN  3.0f
#define SHAKE_MEAN  0.005f // share per tick, about 1 s
#define ADAPT_RATE  0.2f   // 1/s, how fast `as` moves the kp
#define ADAPT_WAIT_S 2.0f  // no change this soon after the start
#define ADAPT_HOLD_S 2.0f  // nor this soon after the robot moved fast (av)

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
#define UPRIGHT_DEG      3.0f  // fall test only: this close to the balance point
// A balance starts from the angle the hand holds: a person feels where the
// robot stops pulling, and that is its balance point, to a degree or two.
// So the start needs no saved value and no buttons after the load changes.
#define START_DEG 25.0f // held this close to upright (15 was too little, test 33)
#define HOLD_DEG  1.5f  // and this still, for UPRIGHT_TICKS
#define UPRIGHT_RATE_DPS 15.0f
#define UPRIGHT_TICKS    LOOP_HZ // 1 s
#define LYING_DEG        45.0f
#define MIN_BATTERY      25    // %, the IP5306 gives 0, 25, 50, 75 or 100
// Balance point at the first boot: -1.5 for the bare robot. The robot then
// learns its own (see LEARN_ below) and keeps it in flash, because the point
// moves a lot with the load: +9.4 with the servo on the back, and -3.5 to
// +0.7 with it on top (2026-11-12). "bp" and buttons A and C move it too.
#define BALANCE_POINT_DEG -1.5f
// Learning the balance point: the mean tilt of a 10 s block, if the robot
// stayed slow in all of it. The first two blocks of a run do not count (the
// robot is still settling), and a block cut short by the end of the run
// is thrown away, so a push or a hand does not spoil the value.
#define LEARN_TICKS     (10 * LOOP_HZ)
#define LEARN_SKIP      2
#define LEARN_MAX_SPEED 0.15f // m/s
#define BP_LIMIT        25.0f // degrees

// Driving ("drive <m/s> <deg/s> [seconds]"). The order moves the place and
// the heading the robot holds, so every balance rule stays as it is. It ends
// by itself: a server must repeat it, or the robot stops and holds its place.
#define DRIVE_HARD_V  0.25f // m/s, never more (the "too fast" stop is at 0.4)
#define DRIVE_MAX_W   90.0f // deg/s
#define DRIVE_W_ACC   180.0f // deg/s^2
#define DRIVE_HOLD_S  0.5f   // an order lasts this long without a time
#define DRIVE_MAX_S   5.0f   // and at most this long with one
#define DRIVE_WAIT_S  2.0f   // no driving this soon after the start

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
    volatile float drive_v, drive_w; // m/s and deg/s the robot follows now
    // To watch the motor model, the shake meter and the kp in use (see `gain`).
    volatile float v_model, shake, kp_eff;
} status_t;

// The last raw readings, for the phone page only. The control task writes
// them on each tick; nothing in the control law reads them.
static struct {
    volatile int16_t ax, ay, az, temp, gx, gy, gz;
    volatile int32_t enc_l, enc_r;
    volatile uint32_t i2c_errors; // failed sensor reads since boot
} raw;
// Screen backlight in percent. 5 can still be read indoors, and the light is
// one of the larger loads when the motors rest. A command stores the value
// and the main task applies it: the display is not safe to use from two tasks.
#define BACKLIGHT_PCT 5
static volatile int backlight = BACKLIGHT_PCT;

// Charge mode: the robot lies down, waits and charges. The charger's current
// is shared with the Fire, so what the Fire does not use goes to the battery.
// After CHARGE_WAIT_S like this, the loop drops to CHARGE_HZ, Wi-Fi sleeps
// between beacons (the page and updates still work, a bit slower) and the
// screen dims, and the main task runs at 5 Hz and logs once a second. After
// SCREEN_OFF_S the screen goes off (the light and the panel), and the CPU
// drops to 80 MHz and sleeps while no task has work (light sleep). Standing
// the robot up, pulling the cable or an update ends it.
#define CHARGE_WAIT_S        10
#define SCREEN_OFF_S         20
#define CHARGE_HZ            10
#define CHARGE_BACKLIGHT_PCT 2
static volatile bool low_power;  // written by the control task
static volatile bool screen_off; // written by the main task, for /state

// The CPU at full speed and always awake (false), or slower and asleep while
// idle (true). Only for charge mode: asleep, a task can wake a little late.
static void cpu_rest(bool on)
{
    esp_pm_config_t pm = {};
    pm.max_freq_mhz = on ? 80 : CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ;
    pm.min_freq_mhz = pm.max_freq_mhz;
    pm.light_sleep_enable = on;
    const esp_err_t err = esp_pm_configure(&pm);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "cpu_rest(%d): %s", on, esp_err_to_name(err));
    }
}
static volatile float saved_bp = NAN; // the balance point in flash, NAN if none

static M5GFX display;
static i2c_master_dev_handle_t imu, base, power;
static bool have_power;
static bool sim;
static status_t st = {ST_WAIT, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, false, false, "boot", 0, 0, 0, 0, 0};
static float gyro_offset, yaw_offset;
// Gains, changed over Wi-Fi or serial while it runs (handle_command()).
// Start values: see the README.
static struct {
    // 3.5 was set on a carpet. On parquet it shakes at 8 Hz, and 3.0 cuts that
    // by a fifth for a little more sway on the carpet (test 33).
    volatile float kp = 3.0f;  // m/s per rad of tilt (about 45 PWM per degree)
    volatile float ki = 10.0f; // m/s per (rad*s)
    volatile float kd = 0.07f; // m/s per rad/s (0.10 shook more, test 9)
    // Low-pass filter on the rate used by the D term: share of the new sample
    // per tick (0.32 is about 12 Hz, 0.45 about 19 Hz). Without it, a jerk of
    // the wheels shakes the body, the gyro sees it, and the D term answers
    // with another jerk: our second test shook at 12 Hz or more. But it also
    // delays the D term, and the robot shakes at 6-8 Hz. 0.45 shook least in
    // the sweep of test 13; 0.2 shook most.
    volatile float rf = 0.45f;
    // Tilt prediction, s (0 = off). The P term uses the tilt the robot will
    // have this much later: tilt + rate * tp, with the rate not filtered. The
    // motors answer about 20 ms late, so 0.02 is the value to try. It is the
    // same as more D (kp * tp) on the raw rate, so it can also bring back the
    // 12 Hz shake that rf removes. It did: 0.02 and 0.03 shook more, at 9 to
    // 10 Hz (test 34). So it is off.
    volatile float tp = 0.0f;
    // Motor model. We measured that a wheel follows the command with a lag of
    // about 80 ms (test 11). v_model is the speed the wheels should have now,
    // by that lag (mt, s). mk (0 = off) asks for more while the model is
    // behind the command: v + mk * (v - v_model). The lag then acts as
    // mt / (1 + mk), so mk 1 halves it. It also makes the noise of the D term
    // (1 + mk) times larger. Every value shook more (test 34), so it is off.
    volatile float mt = 0.08f;
    volatile float mk = 0.0f;
    // Dead band with two steps (example 05): a still wheel starts at PWM 90 to
    // 100, and a turning one keeps going down to 50 to 60. ks is the least
    // PWM for a wheel that stands still, kr for one that turns (0 = off). A
    // wheel stands still when the encoders gave no count for kst seconds, so
    // a wheel that only changes direction gets no kick. ks alone changed
    // nothing; the two together shook a little less (test 34).
    volatile float ks = 90.0f;
    volatile float kr = 55.0f;
    volatile float kst = 0.04f;
    // kp by the floor. A hard floor wants a lower kp than a carpet (test 33).
    // While the shake meter reads more than `as` deg/s (0 = off), kp goes
    // down, to kp * amin at most. Under `as` it comes back. A push also
    // shakes the robot, so it moves slowly. With kp 3 the meter reads 8 to 9
    // on the carpet and 13 to 16 on the hard floor, so 11 is between them
    // (test 34). A push shakes the robot too, and took kp down to its least
    // value for 30 s (test 35). So kp does not change while the robot moves
    // faster than av (m/s), nor for ADAPT_HOLD_S after that.
    volatile float as = 11.0f;
    volatile float amin = 0.6f;
    volatile float av = 0.15f;
    // Position loop. It used to share the job with M5Stack's two terms on
    // v_cmd (kx 0.5, kv 0.3); 18 and 8 do the same without them (tests 22, 23).
    volatile float kpos = 18.0f; // lean back this many degrees per m ahead
    // 16 halves the travel on both floors (test 34); it was 8.
    volatile float kvel = 16.0f; // and this many per m/s forward
    volatile float kt = -0.05f;      // slow balance point trim, rad per (m*s); off
    volatile float off = PWM_OFFSET; // dead band jump, PWM
    volatile float ramp = 0.3f;      // m/s over which the jump grows from 0 (test 13)
    volatile float kh = 0.1f;        // heading hold, m/s per rad (test 11)
    volatile float vmax = 0.5f;      // m/s, asked speed limit
    volatile float max = MAX_PWM;
    // Share of the gyro in the tilt filter, per tick. 0.98 follows the
    // accelerometer in about 0.25 s, 0.995 in about 1 s. The accelerometer
    // also feels the wheels speed up, and that leaks into the tilt. With
    // 0.995 the robot shook less in tests 20 and 21, but not in test 22.
    volatile float gw = 0.995f;
    // Wheel position loop, 1/s (0 = off). The command is added up into the
    // place where the wheels should be. If they are behind (stuck in the
    // dead band), they get up to KW_LIMIT more speed. It cuts the slow sway
    // but adds to the fast shake (tests 20, 21), so it is off.
    volatile float kw = 0.0f;
    volatile float dv = 0.15f;  // m/s, most a drive order may ask for
    volatile float dacc = 0.2f; // m/s^2, how fast the asked speed changes
    // Share of the driving speed that goes straight to the wheels. With 0 the
    // robot only leans, and on our floors a lean of 3 degrees moved it at
    // 0.03 m/s when 0.10 was asked: the command stayed in the dead band.
    volatile float dff = 1.0f;
} gain;

static volatile bool stop_request;
static volatile bool fall_request; // next start is a fall test, not a balance
static volatile bool ready;        // set by the control task after calibration
static volatile float setpoint_trim = BALANCE_POINT_DEG;
// Balance point learned in this run, or NAN. The control task sets it; the
// main task saves it after the run, because a flash write stalls both cores.
static volatile float learned_bp = NAN;
// Servo pulse asked for by a command. Only the control task talks to the
// base, so it sends the value when it changes.
static volatile int servo_us = SERVO_STOP_US;
// The last drive order: speed (m/s, + = forward), turn rate (deg/s, + = left)
// and the time it ends (ms since boot). The control task follows it.
static volatile float drive_v, drive_w;
static volatile uint32_t drive_end_ms;

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

    // Look for the base and the IMU up to 5 times. A restart (an update, a
    // watchdog) can cut an I2C transfer in two. The chip that was answering
    // then holds the data line, and the first probe fails. The bus reset
    // clocks it free. Without this the robot once started as "sim", with no
    // Wi-Fi, after an update (2026-11-12).
    bool found = false;
    for (int i = 0; i < 5 && !found; i++) {
        if (i > 0) {
            i2c_master_bus_reset(bus);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        found = i2c_master_probe(bus, BASE_ADDR, 50) == ESP_OK &&
                i2c_master_probe(bus, MPU6886_ADDR, 50) == ESP_OK;
    }
    if (!found) {
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

static bool set_servo(int us)
{
    if (sim) {
        return true;
    }
    const uint8_t buf[3] = {REG_SERVO_US + 2 * (SERVO_CH - 1), (uint8_t)(us >> 8), (uint8_t)us};
    return i2c_master_transmit(base, buf, sizeof(buf), I2C_TIMEOUT) == ESP_OK;
}

// Tilt from the accelerometer (deg), gyro X rate (raw, the tilt axis) and
// gyro Z rate (raw, the yaw axis when the robot stands).
static bool read_imu(float *acc_tilt, float *gyro_raw, float *yaw_raw)
{
    if (sim) {
        *acc_tilt = sm.th * RAD_TO_DEG_F + ((int)(esp_random() % 41) - 20) * 0.01f;
        *gyro_raw = sm.thd * RAD_TO_DEG_F * GYRO_LSB_PER_DPS - 150 + (int)(esp_random() % 17) - 8;
        *yaw_raw = 60 + (int)(esp_random() % 17) - 8;
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
    raw.ax = ax;
    raw.ay = ay;
    raw.az = az;
    raw.temp = (int16_t)(b[6] << 8 | b[7]);
    raw.gx = *gyro_raw;
    raw.gy = (int16_t)(b[10] << 8 | b[11]);
    raw.gz = *yaw_raw;
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
    raw.enc_l = l;
    raw.enc_r = r;
    *pos = -(l + r) * 0.5f * M_PER_COUNT;
    *right_ahead = (l - r) * M_PER_COUNT;
    return true;
}

// Battery level in 25% steps and charging state, from the IP5306 (registers
// from example 02). Under load the battery voltage drops, so the level reads
// lower while the motors work.
// Raw IP5306 registers, to see how it charges: system (0x00 to 0x02), charger
// (0x20 to 0x24) and status (0x70 to 0x78). -1 = not read yet. In /state as "pwr".
static const uint8_t PWR_REGS[] = {0x00, 0x01, 0x02, 0x20, 0x21, 0x22, 0x23,
                                   0x24, 0x70, 0x71, 0x72, 0x77, 0x78};
static volatile int16_t pwr_raw[sizeof(PWR_REGS)] = {-1, -1, -1, -1, -1, -1, -1,
                                                     -1, -1, -1, -1, -1, -1};

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
    // One more register each second, in turn, so the 200 Hz loop loses no time.
    static size_t next;
    reg = PWR_REGS[next];
    uint8_t v;
    if (i2c_master_transmit_receive(power, &reg, 1, &v, 1, I2C_TIMEOUT) == ESP_OK) {
        pwr_raw[next] = v;
    }
    next = (next + 1) % sizeof(PWR_REGS);
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
        double sum = 0, sum_sq = 0, sum_yaw = 0;
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
            vTaskDelay(pdMS_TO_TICKS(2));
        }
        const double mean = sum / CAL_SAMPLES;
        const double sd = sqrt(sum_sq / CAL_SAMPLES - mean * mean);
        if (sd < 20 || tries == 5) {
            gyro_offset = (float)mean;
            yaw_offset = (float)(sum_yaw / CAL_SAMPLES);
            printf("gyro offset %.1f raw (noise %.1f), yaw offset %.1f raw, try %d\n", mean, sd,
                   yaw_offset, tries);
            return true;
        }
        printf("it moved (noise %.1f), calibrating again\n", sd);
    }
}

// --- Control task ----------------------------------------------------------

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

// The same PWM, but not smaller than `least`. A PWM of 0 stays 0.
static float at_least(float u, float least)
{
    if (u == 0 || fabsf(u) >= least) {
        return u;
    }
    return u > 0 ? least : -least;
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
    set_servo(SERVO_STOP_US); // the base keeps the last pulse too
    int servo_sent = SERVO_STOP_US;
    if (sim) {
        st.seen_lying = true; // the emulator script starts with it held upright
        setpoint_trim = 0;    // the simulated robot balances at 0
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
    float learn_sum = 0, hold_ref = 0, hold_sum = 0;
    int learn_n = 0, learn_blocks = 0;
    bool learn_calm = true;
    float start_ahead = 0, heading = 0; // heading in rad, + = turned left
    float x_ref = 0;                    // m, where the wheels should be (kw)
    float v_model = 0;                  // m/s, wheel speed by the motor model (mt)
    float shake_fast = 0, shake_slow = 0, shake = 0; // shake meter, deg/s
    float kp_scale = 1;                 // share of kp in use (as)
    int still_ticks = 0;                // ticks with no encoder count
    int fast_hold = 0;                  // ticks left before kp may change again (av)
    // Driving: the speed and turn rate followed now, and the place (m from
    // the start) and heading (rad) they have moved the robot's goal to.
    float v_ref = 0, w_ref = 0, x_goal = 0, h_goal = 0;
    read_position(&last_pos, &ahead);
    int upright_ticks = 0, fast_ticks = 0, balance_ticks = 0, errors = 0, write_errors = 0,
        battery_ticks = 0;
    int rest_ticks = 0; // ticks lying down, waiting and charging
    int64_t last_us = esp_timer_get_time();
    TickType_t wake = xTaskGetTickCount();

    for (;;) {
        const bool low = low_power;
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(1000 / (low ? CHARGE_HZ : LOOP_HZ)));
        esp_task_wdt_reset();
        const int64_t now = esp_timer_get_time();
        const bool late = now - last_us > LATE_US;
        last_us = now;
        if (sim) {
            sim_step();
        }

        // Sense. The battery once a second.
        if (++battery_ticks >= (low ? CHARGE_HZ : LOOP_HZ)) {
            battery_ticks = 0;
            read_battery();
        }
        if (!read_imu(&acc, &g, &yaw) || !read_position(&pos, &ahead)) {
            raw.i2c_errors = raw.i2c_errors + 1;
            if (++errors >= 3 && (st.state == ST_BALANCE || st.state == ST_FALL)) {
                stop("I2C error");
            }
            if (st.state == ST_WAIT || st.state == ST_STOPPED) {
                set_speed(0); // the base keeps the last command, so repeat it
            }
            continue;
        }
        errors = 0;
        if (st.state != ST_BALANCE) {
            trim = 0; // or the last run's trim would move the start window
            learn_sum = 0;
            learn_n = 0;
            learn_blocks = 0;
            learn_calm = true;
        }
        const float rate = (g - gyro_offset) / GYRO_LSB_PER_DPS;
        const float yaw_rate = YAW_SIGN * (yaw - yaw_offset) / GYRO_LSB_PER_DPS; // deg/s
        rate_f += gain.rf * (rate - rate_f);
        shake_fast += SHAKE_FAST * (rate - shake_fast);
        shake_slow += SHAKE_SLOW * (rate - shake_slow);
        shake += SHAKE_MEAN * (SHAKE_GAIN * fabsf(shake_fast - shake_slow) - shake);
        still_ticks = pos != last_pos ? 0 : (still_ticks < LOOP_HZ ? still_ticks + 1 : LOOP_HZ);
        const float gw = gain.gw;
        // The slow loop of charge mode does not fit the filter (DT), and the
        // robot is still: the accelerometer alone gives the angle.
        tilt = low ? acc : gw * (tilt + rate * DT) + (1 - gw) * acc;
        speed = (1 - SPEED_LPF) * speed + SPEED_LPF * (pos - last_pos) / DT;
        last_pos = pos;
        const float setpoint = setpoint_trim + trim;
        const float err = tilt - setpoint;

        st.tilt = tilt;
        st.rate = rate;
        st.speed = speed;
        st.setpoint = setpoint;
        st.shake = shake;
        if (st.state != ST_BALANCE) {
            v_model = 0;
            kp_scale = 1;
            st.v_model = 0;
            st.kp_eff = gain.kp;
        }
        if (fabsf(tilt) > LYING_DEG) {
            st.seen_lying = true;
        }
        if (st.state == ST_WAIT && st.charging && fabsf(tilt) > LYING_DEG && !link_updating()) {
            if (!low && ++rest_ticks >= CHARGE_WAIT_S * LOOP_HZ) {
                low_power = true;
            }
        } else {
            rest_ticks = 0;
            low_power = false;
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
                if (link_updating()) {
                    st.reason = "updating";
                    upright_ticks = 0;
                    break;
                }
                st.reason = low_power ? "charging, low power" : "stand me up";
                // Start only after it was lying down, then held upright and still:
                // within HOLD_DEG of where this hold began. A fall test still
                // needs the known balance point, so it keeps the old check.
                if (st.seen_lying && fabsf(rate) < UPRIGHT_RATE_DPS &&
                    (fall_request ? fabsf(err) < UPRIGHT_DEG
                                  : fabsf(tilt) < START_DEG &&
                                        (upright_ticks == 0 || fabsf(tilt - hold_ref) < HOLD_DEG))) {
                    if (upright_ticks++ == 0) {
                        hold_ref = tilt;
                        hold_sum = 0;
                    }
                    hold_sum += tilt;
                } else {
                    upright_ticks = 0;
                }
                if (upright_ticks >= UPRIGHT_TICKS) {
                    if (!fall_request) {
                        setpoint_trim = hold_sum / upright_ticks; // the hand's angle
                    }
                    upright_ticks = 0;
                    balance_ticks = 0;
                    fast_ticks = 0;
                    write_errors = 0;
                    start_pos = pos;
                    start_ahead = ahead;
                    heading = 0;
                    x_ref = 0;
                    v_ref = w_ref = x_goal = h_goal = 0;
                    drive_end_ms = 0; // an order from before the start does not count
                    speed = 0;
                    integral = 0;
                    trim = 0;
                    stop_request = false;
                    if (fall_request) {
                        // Fall test: record 4 s at 200 Hz from now; let go.
                        fall_request = false;
                        mon_rec_start(4 * LOOP_HZ);
                        st.state = ST_FALL;
                        st.reason = "fall test: let go";
                        ESP_LOGI(TAG, "fall test");
                        break;
                    }
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
                // Drive: follow the order while it lasts, then go back to 0.
                // The speed changes slowly, because the robot must lean first.
                const bool driving = balance_ticks > DRIVE_WAIT_S * LOOP_HZ &&
                                     (int32_t)(drive_end_ms - (uint32_t)(now / 1000)) > 0;
                const float want_v = driving ? clampf(drive_v, fminf(gain.dv, DRIVE_HARD_V)) : 0;
                const float want_w = driving ? clampf(drive_w, DRIVE_MAX_W) : 0;
                v_ref += clampf(want_v - v_ref, gain.dacc * DT);
                w_ref += clampf(want_w - w_ref, DRIVE_W_ACC * DT);
                x_goal += v_ref * DT;
                h_goal += w_ref / RAD_TO_DEG_F * DT;
                st.drive_v = v_ref;
                st.drive_w = w_ref;
                const bool moving = v_ref != 0 || w_ref != 0;

                const float x = pos - start_pos; // m from the start
                const float xe = x - x_goal;     // m ahead of where it should be
                st.position = x;
                fast_ticks = fabsf(speed) > MAX_SPEED_MS ? fast_ticks + 1 : 0;

                if (stop_request) {
                    stop("button B");
                } else if (fabsf(tilt - setpoint_trim) > FALL_DEG) {
                    stop("fell over");
                } else if (fabsf(xe) > MAX_DISTANCE_M) {
                    stop("moved too far");
                } else if (fast_ticks > MAX_SPEED_TICKS) {
                    stop("too fast");
                } else if (late && !sim) { // QEMU does not run in real time
                    stop("loop late");
                }
                if (st.state != ST_BALANCE) {
                    break;
                }
                // The computer went quiet: keep balancing. Stopping the motors
                // here made a healthy robot fall over. It holds its place by
                // itself (the position loop), the servo stops (below), and
                // every other stop rule still works. A drive order ends by
                // itself (above), so there is nothing to undo here.
                st.reason = link_alive() ? "balancing" : "balancing, link lost";

                // The balance point follows a slow drift (kt), up to 5 degrees.
                // Not while it drives: being behind the goal is normal then.
                if (!moving) {
                    trim = clampf(trim + gain.kt * xe * DT * RAD_TO_DEG_F, TRIM_LIMIT);
                }
                learn_sum += tilt;
                if (fabsf(speed) > LEARN_MAX_SPEED || moving) {
                    learn_calm = false;
                }
                if (++learn_n >= LEARN_TICKS) {
                    if (learn_calm && ++learn_blocks > LEARN_SKIP) {
                        learned_bp = learn_sum / learn_n;
                    }
                    learn_sum = 0;
                    learn_n = 0;
                    learn_calm = true;
                }
                // Position loop: when the robot is ahead, or rolls forward, ask
                // it to lean back a little, so it comes back to where it started.
                // A drive order moves that place, at the speed it asks for.
                const float outer =
                    clampf(-gain.kpos * xe - gain.kvel * (speed - v_ref), OUTER_LIMIT);
                st.setpoint = setpoint + outer;

                // Heading: the gyro is good over short times and does not care
                // if a wheel slips; the encoders do not drift. Mix both.
                const float heading_enc = (ahead - start_ahead) / TRACK_M;
                heading = (1 - HEADING_ENC) * (heading + yaw_rate / RAD_TO_DEG_F * DT) +
                          HEADING_ENC * heading_enc;
                // Heading hold: the two motors are not the same (the left one is
                // 3-5% faster, example 05), so the robot slowly turns. Give the
                // right wheel a little more when it has turned right, and less
                // when it has turned left. A drive order turns the heading it
                // holds, and the wheels get the speed difference of that turn
                // directly: kh alone is too soft to follow it.
                const float turn = clampf(-gain.kh * (heading - h_goal) +
                                              w_ref / RAD_TO_DEG_F * TRACK_M / 2,
                                          TURN_LIMIT);
                st.heading_enc = heading_enc * RAD_TO_DEG_F;
                st.heading = heading * RAD_TO_DEG_F;
                st.yaw_rate = yaw_rate;
                st.turn = turn;

                // Soft start: the speed limit grows from 0 in RAMP_S.
                const float ramp = fminf(1.0f, balance_ticks * DT / RAMP_S);
                const float vlim = fminf(gain.vmax, HARD_MAX_V) * ramp;
                const float th = (err - outer) / RAD_TO_DEG_F;
                integral = clampf(integral + gain.ki * th * DT, vlim);
                // kp by the floor: down while it shakes more than `as`, back
                // up while it shakes less.
                const float as = gain.as;
                if (fabsf(speed) > gain.av) {
                    fast_hold = (int)(ADAPT_HOLD_S * LOOP_HZ);
                } else if (fast_hold > 0) {
                    fast_hold--;
                }
                if (as <= 0) {
                    kp_scale = 1;
                } else if (balance_ticks > ADAPT_WAIT_S * LOOP_HZ && fast_hold == 0) {
                    kp_scale += ADAPT_RATE * clampf(1 - shake / as, 1) * DT;
                    kp_scale = fminf(1.0f, fmaxf(kp_scale, gain.amin));
                }
                const float kp = gain.kp * kp_scale;
                // The I term keeps the tilt of now: only P looks ahead (tp).
                const float th_p = th + gain.tp * rate / RAD_TO_DEG_F;
                const float v_cmd =
                    clampf(kp * th_p + integral + gain.kd * rate_f / RAD_TO_DEG_F, vlim);
                // Wheel position loop: push harder while the wheels are behind
                // the place the command asked for. The error is kept small,
                // so it cannot wind up.
                const float kw = gain.kw;
                x_ref = x + clampf(x_ref + v_cmd * DT - x, KW_LIMIT / fmaxf(kw, 0.01f));
                const float v_wheels =
                    clampf(v_cmd + kw * (x_ref - x) + gain.dff * v_ref, vlim);
                // Motor model: ask for more while the wheels are still on
                // their way to the command (mk), then move the model.
                const float v_out = clampf(v_wheels + gain.mk * (v_wheels - v_model), vlim);
                v_model += (v_out - v_model) * DT / fmaxf(gain.mt, DT);
                st.v_model = v_model;
                st.kp_eff = kp;
                const float lim = fminf(gain.max, HARD_MAX_PWM);
                // Dead band: a wheel that stands still needs more to start.
                const float least = still_ticks >= gain.kst * LOOP_HZ ? gain.ks : gain.kr;
                const float ul = clampf(at_least(pwm_for(v_out - turn), least), lim);
                const float ur = clampf(at_least(pwm_for(v_out + turn), least), lim);
                st.u = (ul + ur) / 2;
                // Its own counter: "errors" goes back to 0 on every good read.
                if (set_wheels((int16_t)ul, (int16_t)ur)) {
                    write_errors = 0;
                } else if (++write_errors >= 3) {
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
                if (fabsf(tilt) > 40 || !mon_rec_active() || stop_request) {
                    stop("fall test done");
                }
                break;
            }
        }

        // Servo: stop it if the computer went quiet. Send
        // the pulse only when it changes; a failed write is tried again.
        if (!link_alive()) {
            servo_us = SERVO_STOP_US;
        }
        const int want = servo_us;
        if (want != servo_sent && set_servo(want)) {
            servo_sent = want;
        }

        // Fast samples for bala2_mon. It takes them only while someone asked.
        const float sample[MON_REC_COLS] = {tilt,  rate,       acc,      st.position, speed,
                                            st.u,  st.v_model, shake,    st.kp_eff};
        mon_rec_add((uint32_t)(now / 1000), sample);
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

static void handle_command(const char *line);
static bool can_update(void);

// In the emulator: hold it upright, let go, and drive it forward as a server
// would (one order every tick). Then check it is still up, 2 s after the
// order ends. The drive takes the place of the push this script used to give:
// the simulated robot sways more and more after either, so it gets only one.
static void sim_script(int64_t ms)
{
    static int64_t start_ms = -1;
    static bool done;
    if (st.state == ST_BALANCE && start_ms < 0) {
        start_ms = ms;
    }
    if (start_ms < 0) {
        return;
    }
    const int64_t t = ms - start_ms;
    if (t > 2500 && t < 5500) {
        handle_command("drive 0.1 0");
    }
    if (!done && t > 7500) {
        done = true;
        printf("sim: drove %.2f m\n", st.position);
        printf("sim: done, %s\n", st.reason);
    }
}

// Commands, from the serial monitor or over Wi-Fi. The numbers the robot
// shows and takes ("kp 3.5", "bp 9", "servo 1700", "?" for the list) are in
// the table app_main() gives to bala2_mon. Each change is answered with the
// whole list, so the log shows which values were used. "hb" (heartbeat) only
// keeps the Wi-Fi link alive.
static void handle_command(const char *line)
{
    char name[12]; // "defaults" has 8 letters
    float v;
    const int n = sscanf(line, "%11s %f", name, &v);
    if (n < 1 || !strcmp(name, "hb")) {
        return;
    }
    if (!strcmp(name, "stop")) {
        stop_request = true;
        servo_us = SERVO_STOP_US;
        drive_end_ms = 0;
    } else if (!strcmp(name, "drive")) {
        // "drive 0.1 30 2": 0.1 m/s forward, turning left at 30 deg/s, for 2 s.
        // "drive 0 0", or "drive" alone, ends the order: the robot holds its place.
        float speed = 0, turn = 0, seconds = DRIVE_HOLD_S;
        sscanf(line, "%*s %f %f %f", &speed, &turn, &seconds);
        // Not NAN, and inside the limits (the control task has the last word).
        drive_v = isnan(speed) ? 0 : clampf(speed, DRIVE_HARD_V);
        drive_w = isnan(turn) ? 0 : clampf(turn, DRIVE_MAX_W);
        seconds = seconds > 0 ? fminf(seconds, DRIVE_MAX_S) : 0;
        drive_end_ms = (uint32_t)(esp_timer_get_time() / 1000) + (uint32_t)(seconds * 1000);
    } else if (!strcmp(name, "fall")) {
        fall_request = true;
        mon_say("fall test: lay it down, then hold it upright and still, and let go\n");
    } else if (n == 2 && !strcmp(name, "rec")) {
        // Fast samples, sent as they come. Up to 60 s: the link must keep up.
        const float seconds = fminf(fmaxf(v, 0), 60);
        mon_rec_start((int)(seconds * LOOP_HZ));
        mon_say("recording %.1f s at %d Hz\n", seconds, LOOP_HZ);
    } else if (!strcmp(name, "save") || !strcmp(name, "defaults")) {
        // To flash, or back to the firmware's values. A flash write stalls
        // both cores, and new gains in one step are a jolt: not while it balances.
        if (!can_update()) {
            mon_say("%s: stop the robot first\n", name);
        } else if (name[0] == 's' ? mon_save() : mon_defaults()) {
            mon_say("%s: %d values in flash\n", name, mon_stored());
            mon_list();
        } else {
            mon_say("%s: the flash did not take it\n", name);
        }
    } else if (!mon_command(line)) {
        mon_say("unknown command: %s\n", line);
    }
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

// An update is refused while the robot balances or runs a fall test.
static bool can_update(void)
{
    return st.state != ST_BALANCE && st.state != ST_FALL;
}

// For the phone page. The reasons are our own texts, with no quotes in them.
static void state_json(char *buf, size_t size)
{
    static const char *const names[] = {"waiting", "balancing", "stopped", "fall test"};
    const esp_app_desc_t *app = esp_app_get_description();
    const float saved = saved_bp;
    char saved_text[12] = "null";
    if (!isnan(saved)) {
        snprintf(saved_text, sizeof(saved_text), "%.2f", saved);
    }
    char net[33]; // the network name, without the two characters JSON escapes
    strlcpy(net, link_ssid(), sizeof(net));
    for (char *c = net; *c; c++) {
        if (*c == '"' || *c == '\\') {
            *c = '?';
        }
    }
    static char vars[640]; // one server task, so one call at a time
    mon_json(vars, sizeof(vars));
    // MPU6886: 4096 counts per g at +-8 g, and 326.8 counts per degree C.
    snprintf(buf, size,
             "{\"state\":\"%s\",\"reason\":\"%s\",\"tilt\":%.2f,\"speed\":%.3f,"
             "\"battery\":%d,\"charging\":%s,\"vars\":%s,"
             "\"rate\":%.1f,\"pos\":%.3f,\"u\":%.0f,\"setpoint\":%.2f,\"heading\":%.1f,"
             "\"yaw\":%.1f,\"turn\":%.3f,\"drive\":[%.3f,%.1f],\"saved_bp\":%s,\"stored\":%d,"
             "\"acc\":[%.2f,%.2f,%.2f],\"gyro\":[%.1f,%.1f,%.1f],\"temp\":%.1f,"
             "\"enc\":[%ld,%ld],\"btn\":[%d,%d,%d],\"i2c_err\":%lu,"
             "\"net\":\"%s\",\"home\":%s,\"ip\":\"%s\",\"rssi\":%d,"
             "\"up\":%lld,\"heap\":%lu,\"heap_min\":%lu,\"fw\":\"%s %s\",\"slot\":\"%s\","
             "\"restart\":\"%s\",\"pwr\":[%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d],\"screen\":%s}",
             names[st.state], st.reason, st.tilt, st.speed, st.battery,
             st.charging ? "true" : "false", vars, st.rate,
             st.position, st.u,
             st.setpoint, st.heading, st.yaw_rate, st.turn, st.drive_v, st.drive_w, saved_text, mon_stored(), raw.ax / 4096.0f,
             raw.ay / 4096.0f, raw.az / 4096.0f, raw.gx / GYRO_LSB_PER_DPS,
             raw.gy / GYRO_LSB_PER_DPS, raw.gz / GYRO_LSB_PER_DPS, raw.temp / 326.8f + 25,
             (long)raw.enc_l, (long)raw.enc_r, !gpio_get_level(BTN_A), !gpio_get_level(BTN_B),
             !gpio_get_level(BTN_C), (unsigned long)raw.i2c_errors, net,
             link_on_home() ? "true" : "false", link_ip(), link_rssi(),
             esp_timer_get_time() / 1000000, (unsigned long)esp_get_free_heap_size(),
             (unsigned long)esp_get_minimum_free_heap_size(), app->date, app->time,
             esp_ota_get_running_partition()->label, link_last_restart(), pwr_raw[0],
             pwr_raw[1], pwr_raw[2], pwr_raw[3], pwr_raw[4], pwr_raw[5], pwr_raw[6], pwr_raw[7],
             pwr_raw[8], pwr_raw[9], pwr_raw[10], pwr_raw[11], pwr_raw[12],
             screen_off ? "false" : "true");
}

// The learned balance point lives in flash (NVS), in hundredths of a degree.
static void save_balance_point(float deg)
{
    nvs_handle_t nvs;
    if (nvs_open("balance", NVS_READWRITE, &nvs) != ESP_OK) {
        return;
    }
    if (nvs_set_i32(nvs, "bp_cdeg", (int32_t)lroundf(deg * 100)) == ESP_OK) {
        nvs_commit(nvs);
        saved_bp = deg;
    }
    nvs_close(nvs);
}

static void load_balance_point(void)
{
    nvs_handle_t nvs;
    int32_t cdeg;
    if (nvs_open("balance", NVS_READONLY, &nvs) != ESP_OK) {
        return; // nothing saved yet
    }
    if (nvs_get_i32(nvs, "bp_cdeg", &cdeg) == ESP_OK) {
        setpoint_trim = clampf(cdeg / 100.0f, BP_LIMIT);
        saved_bp = setpoint_trim;
    }
    nvs_close(nvs);
}

extern "C" void app_main(void)
{
    // Before anything else: a new app that keeps failing gives way to the old one.
    link_guard();
    cpu_rest(false);
    // What a computer can see and change while the robot runs, with the range
    // each value is kept in. Start values and what they do: see `gain` above.
    mon_float("kp", &gain.kp, 0, 20);
    mon_float("ki", &gain.ki, 0, 100);
    mon_float("kd", &gain.kd, 0, 1);
    mon_float("rf", &gain.rf, 0.05f, 1);
    mon_float("tp", &gain.tp, 0, 0.05f);
    mon_float("mt", &gain.mt, 0.01f, 0.3f);
    mon_float("mk", &gain.mk, 0, 3);
    mon_float("ks", &gain.ks, 0, 120);
    mon_float("kr", &gain.kr, 0, 80);
    mon_float("kst", &gain.kst, 0.01f, 0.5f);
    mon_float("as", &gain.as, 0, 100);
    mon_float("amin", &gain.amin, 0.3f, 1);
    mon_float("av", &gain.av, 0.02f, 1);
    mon_float("kpos", &gain.kpos, 0, 100);
    mon_float("kvel", &gain.kvel, 0, 50);
    mon_float("kt", &gain.kt, -1, 1);
    mon_float("off", &gain.off, 0, 60);
    mon_float("ramp", &gain.ramp, 0.001f, 0.5f);
    mon_float("kh", &gain.kh, 0, 5);
    mon_float("vmax", &gain.vmax, 0, HARD_MAX_V);
    mon_float("max", &gain.max, 0, HARD_MAX_PWM);
    mon_float("gw", &gain.gw, 0.9f, 0.999f);
    mon_float("kw", &gain.kw, 0, 100);
    mon_float("dv", &gain.dv, 0, DRIVE_HARD_V);
    mon_float("dacc", &gain.dacc, 0.05f, 1);
    mon_float("dff", &gain.dff, 0, 1.5f);
    mon_float("bp", &setpoint_trim, -BP_LIMIT, BP_LIMIT); // balance point, degrees
    mon_int("servo", &servo_us, SERVO_MIN_US, SERVO_MAX_US);
    mon_int("bl", &backlight, 0, 100); // screen backlight, percent
    // "save" keeps all of them in flash but these two: the balance point has
    // its own place there, and a servo must not start by itself.
    mon_not_saved("bp");
    mon_not_saved("servo");
    mon_rec_columns("tilt,rate,acc_tilt,x,v,u,v_model,shake,kp_eff");
    // The screen first: M5GFX looks at the I2C pins to find out which board
    // it is on, so it must finish before our I2C bus starts.
    display.init();
    display.setBrightness(BACKLIGHT_PCT * 255 / 100);
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
        extern const char page_start[] asm("_binary_app_html_start");
        extern const char page_end[] asm("_binary_app_html_end");
        const link_callbacks_t cb = {handle_command, can_update, state_json, page_start,
                                     (size_t)(page_end - page_start - 1)}; // no final NUL
        if (link_start(&cb) != ESP_OK) {
            ESP_LOGE(TAG, "no Wi-Fi link: the robot works, but only the buttons control it");
        }
        load_balance_point(); // after link_start: it opens the flash storage
        mon_load();           // the same for the values of "save"
        printf("Wi-Fi %s %s, key %s, robot at %s (%s), UDP port %d\n",
               link_on_home() ? "home network" : "own network", link_ssid(), link_password(),
               link_ip(), LINK_HOST, LINK_UDP_PORT);
    }

    gpio_config_t cfg = {};
    cfg.pin_bit_mask = (1ULL << BTN_A) | (1ULL << BTN_B) | (1ULL << BTN_C);
    cfg.mode = GPIO_MODE_INPUT;
    ESP_ERROR_CHECK(gpio_config(&cfg));

    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0));
    mon_list();
    mon_say("t_ms,state,tilt,rate,speed,position,u,setpoint,heading_enc,heading,yaw_rate,turn,"
        "battery,charging,v_model,shake,kp_eff\n");
    int last_a = 1, last_b = 1, last_c = 1, backlight_set = BACKLIGHT_PCT;
    bool low_set = false;
    int64_t low_ms = 0; // when charge mode began
    for (int n = 0;; n++) {
        const int64_t ms = esp_timer_get_time() / 1000;
        read_serial();
        if (sim) {
            sim_script(ms);
        } else {
            if (pressed(BTN_B, &last_b)) {
                stop_request = true;
                servo_us = SERVO_STOP_US;
                drive_end_ms = 0;
            }
            if (pressed(BTN_A, &last_a)) {
                setpoint_trim -= 0.25f;
            }
            if (pressed(BTN_C, &last_c)) {
                setpoint_trim += 0.25f;
            }
            // After a run, start the next one from the point this one found.
            const float found = learned_bp;
            if (!isnan(found) && can_update()) {
                learned_bp = NAN;
                setpoint_trim = clampf(found, BP_LIMIT);
                save_balance_point(setpoint_trim);
                mon_say("balance point %.2f saved\n", setpoint_trim);
            }
        }
        mon_rec_send(); // the fast samples that are waiting, in one packet
        const bool slow = low_set; // charge mode: this loop at 5 Hz
        if (!slow || n % 5 == 0) { // 50 Hz log, or once a second
            mon_say("%lld,%d,%.2f,%.1f,%.3f,%.3f,%.0f,%.2f,%.1f,%.1f,%.1f,%.3f,%d,%d,%.3f,%.1f,%.2f\n",
                ms, (int)st.state, st.tilt, st.rate, st.speed, st.position, st.u, st.setpoint,
                st.heading_enc, st.heading, st.yaw_rate, st.turn, st.battery, (int)st.charging,
                st.v_model, st.shake, st.kp_eff);
        }
        if (slow || low_power || n % 10 == 0) { // 5 Hz screen
            const bool low = low_power;
            if (low != low_set) {
                low_set = low;
                low_ms = ms;
                link_power_save(low);
            }
            const bool off = low && ms - low_ms >= (SCREEN_OFF_S - CHARGE_WAIT_S) * 1000;
            if (off != screen_off) {
                screen_off = off;
                if (off) {
                    display.setBrightness(0);
                    display.sleep();
                    backlight_set = 0;
                    cpu_rest(true);
                } else {
                    cpu_rest(false);
                    display.wakeup(); // the light comes back just below
                }
            }
            if (off) {
                vTaskDelay(pdMS_TO_TICKS(200));
                continue; // nothing to draw
            }
            const int asked = backlight;
            const int light = low && asked > CHARGE_BACKLIGHT_PCT ? CHARGE_BACKLIGHT_PCT : asked;
            if (light != backlight_set) {
                backlight_set = light;
                display.setBrightness(light * 255 / 100);
            }
            const uint32_t color = st.state == ST_BALANCE ? TFT_GREEN
                                   : st.state == ST_WAIT  ? TFT_YELLOW
                                                          : TFT_RED;
            display.setFont(&fonts::DejaVu18);
            screen_line(40, color, "%s", st.reason);
            display.setFont(&fonts::DejaVu12);
            screen_line(80, TFT_WHITE, "tilt     %+7.2f deg", st.tilt);
            screen_line(98, TFT_WHITE, "position %+7.3f m", st.position);
            screen_line(116, TFT_WHITE, "motor    %+5.0f / %.0f", st.u, gain.max);
            screen_line(134, TFT_DARKGREY, "balance point %+.2f deg (auto)", st.setpoint);
            screen_line(152, st.battery >= 0 && st.battery < MIN_BATTERY ? TFT_RED : TFT_DARKGREY,
                        "battery %d%%%s", st.battery, st.charging ? ", charging" : "");
            screen_line(170, TFT_DARKGREY, "B: stop");
            if (!sim) {
                screen_line(196, TFT_CYAN, "Wi-Fi %s  key %s", link_ssid(), link_password());
                screen_line(214, TFT_CYAN, "%s  %s  %s", link_ip(), LINK_HOST,
                            link_alive() ? "" : "LINK LOST");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(low_set ? 200 : 20));
    }
}
