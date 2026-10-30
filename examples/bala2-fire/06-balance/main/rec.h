// 200 Hz recorder: keeps every control tick for a few seconds in RAM, then
// hands the samples out one line at a time. The normal telemetry runs at
// 50 Hz and its UDP packets arrive in bursts, which is too coarse to see a
// 8.5 Hz shake or to measure how fast the robot falls.
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define REC_MAX 1000 // samples, 5 s at 200 Hz

typedef struct {
    uint32_t t_ms;
    float tilt;     // deg, complementary filter
    float rate;     // deg/s, gyro X
    float acc_tilt; // deg, accelerometer only
    float x;        // m, from the start point
    float v;        // m/s
    float u;        // PWM sent
    float m5_angle; // deg, factory law angle (Madgwick), 0 if not used
} rec_sample_t;

// Start a recording of n samples (at most REC_MAX). Safe from any task.
void rec_start(int n);

// True while recording.
bool rec_active(void);

// Add one sample; does nothing if not recording. Control task only.
void rec_add(const rec_sample_t *s);

// Next line to send ("rec,..."), or false if there is nothing to send.
bool rec_next_line(char *buf, size_t len);
