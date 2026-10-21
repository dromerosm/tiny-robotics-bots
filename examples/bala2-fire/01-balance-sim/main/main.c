// 01-balance-sim: balance a simulated two-wheeled robot with a PID loop.
//
// The robot is modeled as an inverted pendulum on wheels (like BALA2):
//
//     theta'' = (g * sin(theta) - a * cos(theta)) / L
//
// theta is the tilt (0 = upright), L the height of the center of mass and a the
// wheel acceleration the controller asks for. The PID reads theta and sets a.
// Every few seconds the robot gets a push, and the loop has to catch it.
//
// It runs the physics in software, so it needs no motors or IMU. Swap
// simulate() for the real IMU and motor calls to run it on the robot.
#include <math.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "tr_pid.h"

static const char *TAG = "balance";

#define G              9.81f // m/s^2
#define COM_HEIGHT_M   0.04f // center of mass height (BALA2 is about 65 mm tall)
#define CONTROL_HZ     200   // control loop rate
#define PHYSICS_STEPS  5     // physics substeps per control step (1 kHz)
#define MAX_ACCEL      20.0f // m/s^2, what the motors can give
#define START_TILT_DEG 10.0f
#define PUSH_EVERY_S   5.0f
#define PUSH_RATE      1.5f // rad/s added to the tilt rate by each push

#define DEG(rad) ((rad) * 57.29578f)

typedef struct {
    float theta;     // tilt, rad
    float theta_dot; // tilt rate, rad/s
} robot_t;

// Advance the pendulum by dt seconds with wheel acceleration a.
static void simulate(robot_t *r, float a, float dt)
{
    const float theta_ddot = (G * sinf(r->theta) - a * cosf(r->theta)) / COM_HEIGHT_M;
    r->theta_dot += theta_ddot * dt;
    r->theta += r->theta_dot * dt;
}

static void balance_task(void *arg)
{
    // Gains: kp must beat gravity (kp > g), kd damps the motion, ki removes
    // any steady offset. tr_pid returns kp*(0 - theta) - kd*theta_dot + ...,
    // and the wheels must accelerate toward the fall, so a = -output.
    tr_pid_t pid;
    tr_pid_init(&pid, 30.0f, 5.0f, 1.5f, -MAX_ACCEL, MAX_ACCEL);

    robot_t robot = {.theta = START_TILT_DEG / 57.29578f};
    const float dt = 1.0f / CONTROL_HZ;
    float t = 0.0f, upright_for = 0.0f, next_push = PUSH_EVERY_S;
    bool balanced = false;
    int push_sign = 1;

    printf("\nBALA2 balance simulation: start tilt %.1f deg\n", START_TILT_DEG);
    TickType_t last_wake = xTaskGetTickCount();
    for (int step = 0;; step++) {
        const float accel = -tr_pid_update(&pid, 0.0f, robot.theta, dt);
        for (int i = 0; i < PHYSICS_STEPS; i++) {
            simulate(&robot, accel, dt / PHYSICS_STEPS);
        }
        t += dt;

        if (fabsf(DEG(robot.theta)) > 45.0f) {
            ESP_LOGE(TAG, "fell over at t=%.2f s, restarting", t);
            robot = (robot_t){.theta = START_TILT_DEG / 57.29578f};
            tr_pid_reset(&pid);
            balanced = false;
            upright_for = 0.0f;
        }

        // "Balanced" = within 0.5 degrees for one second.
        upright_for = fabsf(DEG(robot.theta)) < 0.5f ? upright_for + dt : 0.0f;
        if (!balanced && upright_for >= 1.0f) {
            balanced = true;
            ESP_LOGI(TAG, "balanced at t=%.2f s", t);
        }

        if (t >= next_push) {
            robot.theta_dot += push_sign * PUSH_RATE;
            ESP_LOGW(TAG, "push %s!", push_sign > 0 ? "forward" : "backward");
            push_sign = -push_sign;
            next_push += PUSH_EVERY_S;
            balanced = false;
            upright_for = 0.0f;
        }

        if (step % (CONTROL_HZ / 10) == 0) { // 10 lines per second
            printf("t=%6.2f s  tilt=%6.2f deg  rate=%7.2f deg/s  wheel accel=%6.2f m/s^2\n", t,
                   DEG(robot.theta), DEG(robot.theta_dot), accel);
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1000 / CONTROL_HZ));
    }
}

void app_main(void)
{
    xTaskCreate(balance_task, "balance", 4096, NULL, 5, NULL);
}
