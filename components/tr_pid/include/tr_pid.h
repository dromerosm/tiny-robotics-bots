// tr_pid: a small, portable PID controller.
//
// - Derivative on measurement, so setpoint jumps do not cause output kicks.
// - Anti-windup: the integral only grows while the output is not saturated.
// - Plain C, no dynamic memory, no ESP-IDF dependency (it builds on a PC too).
#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float kp, ki, kd;       // gains
    float out_min, out_max; // output limits
    float integral;         // accumulated ki * error * dt
    float prev_measurement; // for the derivative term
    bool has_prev;          // false until the first update
} tr_pid_t;

// Set gains and output limits, and clear the state.
void tr_pid_init(tr_pid_t *pid, float kp, float ki, float kd, float out_min, float out_max);

// Clear the integral and the derivative history (keep gains and limits).
void tr_pid_reset(tr_pid_t *pid);

// Compute the next output. dt is the time since the last update, in seconds (> 0).
float tr_pid_update(tr_pid_t *pid, float setpoint, float measurement, float dt);

#ifdef __cplusplus
}
#endif
