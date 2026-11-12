// M5Stack's balancing law, see m5law.h. The Madgwick filter is written here
// from the equations in Madgwick's 2010 report ("An efficient orientation
// filter for inertial and inertial/magnetic sensor arrays"), IMU version.
#include "m5law.h"

#include <math.h>

#define DEG_TO_RAD_F 0.01745329f
#define RAD_TO_DEG_F 57.29578f

// Butterworth low-pass, 50 Hz at 200 Hz (bilinear: K = tan(pi * 50 / 200) = 1).
#define BW_B0 0.29289322f
#define BW_B1 0.58578644f
#define BW_B2 0.29289322f
#define BW_A1 0.0f
#define BW_A2 0.17157288f

typedef struct {
    float x1, x2, y1, y2;
} bw_t;

static float q0 = 1, q1, q2, q3; // orientation quaternion
static float beta = 10;          // fast at first, as in the original
static bool settled;
static bw_t fx, fy, fz;
static float angle, angle_last, speed, integral;
static int32_t last_enc;

static float bw_run(bw_t *f, float x)
{
    const float y = BW_B0 * x + BW_B1 * f->x1 + BW_B2 * f->x2 - BW_A1 * f->y1 - BW_A2 * f->y2;
    f->x2 = f->x1;
    f->x1 = x;
    f->y2 = f->y1;
    f->y1 = y;
    return y;
}

// One Madgwick step: gyro in rad/s, accel in any unit.
static void madgwick(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
    // Rate of change from the gyro: 0.5 * q x (0, gx, gy, gz).
    float d0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    float d1 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    float d2 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    float d3 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    const float n = sqrtf(ax * ax + ay * ay + az * az);
    if (n > 0) {
        ax /= n;
        ay /= n;
        az /= n;
        // Error between the gravity the quaternion expects and the one we see.
        const float f1 = 2 * (q1 * q3 - q0 * q2) - ax;
        const float f2 = 2 * (q0 * q1 + q2 * q3) - ay;
        const float f3 = 2 * (0.5f - q1 * q1 - q2 * q2) - az;
        // Gradient (Jacobian transposed times the error).
        float s0 = -2 * q2 * f1 + 2 * q1 * f2;
        float s1 = 2 * q3 * f1 + 2 * q0 * f2 - 4 * q1 * f3;
        float s2 = -2 * q0 * f1 + 2 * q3 * f2 - 4 * q2 * f3;
        float s3 = 2 * q1 * f1 + 2 * q2 * f2;
        const float sn = sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        if (sn > 0) {
            d0 -= beta * s0 / sn;
            d1 -= beta * s1 / sn;
            d2 -= beta * s2 / sn;
            d3 -= beta * s3 / sn;
        }
    }
    q0 += d0 * dt;
    q1 += d1 * dt;
    q2 += d2 * dt;
    q3 += d3 * dt;
    const float qn = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 /= qn;
    q1 /= qn;
    q2 /= qn;
    q3 /= qn;
}

float m5_angle(void)
{
    return angle;
}

void m5_filter(float ax, float ay, float az, float gx, float gy, float gz, float dt)
{
    const float fax = bw_run(&fx, ax), fay = bw_run(&fy, ay), faz = bw_run(&fz, az);
    madgwick(gx * DEG_TO_RAD_F, gy * DEG_TO_RAD_F, gz * DEG_TO_RAD_F, fax, fay, faz, dt);
    const float roll = atan2f(2 * (q2 * q3 + q0 * q1), 1 - 2 * (q1 * q1 + q2 * q2)) * RAD_TO_DEG_F;
    // As the original: once it agrees with the accelerometer, slow down.
    if (!settled) {
        const float acc_roll = atan2f(ay, sqrtf(az * az + ax * ax)) * RAD_TO_DEG_F;
        if (fabsf(acc_roll - roll) < 1) {
            settled = true;
            beta = 0.1f;
        }
    }
    angle = roll;
}

void m5_start(int32_t enc_sum)
{
    angle_last = angle;
    speed = integral = 0;
    last_enc = enc_sum;
}

float m5_control(int32_t enc_sum, float point_deg)
{
    // PD on the angle, per 5 ms step.
    const float pwm_angle = 24 * (angle - point_deg) + 90 * (angle - angle_last);
    angle_last = angle;

    // PI on the speed: change of the encoder sum per step, filtered.
    speed = 0.8f * speed + 0.2f * (enc_sum - last_enc);
    last_enc = enc_sum;
    integral += 0.075f * (0 - speed);
    integral = integral > 40 ? 40 : (integral < -40 ? -40 : integral);
    const float pwm_speed = 15 * (0 - speed) + integral;

    const float out = pwm_angle + pwm_speed;
    return out > 1023 ? 1023 : (out < -1023 ? -1023 : out);
}
