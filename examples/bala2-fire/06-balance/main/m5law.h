// M5Stack's balancing law, as in the factory firmware of our unit (the
// M5Bala2 app; we found its constants in our flash backup). Used to compare
// it with our own law, with the same telemetry and the same safety limits.
//
// Same as the original:
//   - Madgwick filter (beta 10 until it agrees with the accelerometer, then
//     0.1), the robot's angle is its roll
//   - a 2nd order Butterworth low-pass on the accelerometer, at 50 Hz
//   - PD on the angle: 24 * (angle - point) + 90 * (change in 5 ms)
//   - PI on the speed: -15 * speed + I (0.075 per tick, at most +-40), with
//     speed = filtered change of (left + right encoder counts) per 5 ms
//   - balance point -1.5 degrees, as saved in our unit
//
// Not the same:
//   - it runs at our 200 Hz on single IMU reads. The original read the
//     IMU FIFO at 500 Hz, so the Butterworth is recomputed for 200 Hz.
//   - our safety limits stay in place (PWM limit, stop rules). The original
//     allowed +-1023 and only stopped past 70 degrees.
#pragma once

#include <stdint.h>

// The angle filter runs all the time, from boot, as in the original, so it
// has settled before the robot starts. Call every 5 ms. Accel in any unit,
// gyro in deg/s.
void m5_filter(float ax, float ay, float az, float gx, float gy, float gz, float dt);

// Clear the control (D term memory, speed filter, integral) at the start.
void m5_start(int32_t enc_sum);

// One 5 ms control step. enc_sum = left + right encoder counts, point_deg =
// balance point. Returns PWM (before our limits).
float m5_control(int32_t enc_sum, float point_deg);

// Last angle from the Madgwick filter, in degrees.
float m5_angle(void);
