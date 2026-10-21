#include "tr_pid.h"

static float clamp(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

void tr_pid_init(tr_pid_t *pid, float kp, float ki, float kd, float out_min, float out_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->out_min = out_min;
    pid->out_max = out_max;
    tr_pid_reset(pid);
}

void tr_pid_reset(tr_pid_t *pid)
{
    pid->integral = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->has_prev = false;
}

float tr_pid_update(tr_pid_t *pid, float setpoint, float measurement, float dt)
{
    if (dt <= 0.0f) {
        dt = 1e-6f;
    }
    const float error = setpoint - measurement;

    // Derivative on measurement: -d(measurement)/dt.
    float derivative = 0.0f;
    if (pid->has_prev) {
        derivative = -(measurement - pid->prev_measurement) / dt;
    }
    pid->prev_measurement = measurement;
    pid->has_prev = true;

    const float p_term = pid->kp * error;
    const float d_term = pid->kd * derivative;

    // Conditional integration (anti-windup): only accept the new integral
    // if it does not push an already saturated output further out.
    const float candidate = pid->integral + pid->ki * error * dt;
    const float unclamped = p_term + candidate + d_term;
    const bool saturated_high = unclamped > pid->out_max && error > 0.0f;
    const bool saturated_low = unclamped < pid->out_min && error < 0.0f;
    if (!saturated_high && !saturated_low) {
        pid->integral = candidate;
    }

    return clamp(p_term + pid->integral + d_term, pid->out_min, pid->out_max);
}
