// Host unit tests for tr_pid. Run with scripts/test-host.sh.
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "tr_pid.h"

static int failures = 0;

#define CHECK(cond, msg)                                                                           \
    do {                                                                                           \
        if (!(cond)) {                                                                             \
            printf("  FAIL: %s (line %d)\n", msg, __LINE__);                                       \
            failures++;                                                                            \
        }                                                                                          \
    } while (0)

static void test_proportional_only(void)
{
    tr_pid_t pid;
    tr_pid_init(&pid, 2.0f, 0.0f, 0.0f, -100.0f, 100.0f);
    float out = tr_pid_update(&pid, 10.0f, 4.0f, 0.01f);
    CHECK(fabsf(out - 12.0f) < 1e-4f, "P output is kp * error");
}

static void test_output_is_clamped(void)
{
    tr_pid_t pid;
    tr_pid_init(&pid, 10.0f, 0.0f, 0.0f, -1.0f, 1.0f);
    CHECK(tr_pid_update(&pid, 100.0f, 0.0f, 0.01f) == 1.0f, "clamped to out_max");
    CHECK(tr_pid_update(&pid, -100.0f, 0.0f, 0.01f) == -1.0f, "clamped to out_min");
}

static void test_no_integral_windup(void)
{
    tr_pid_t pid;
    tr_pid_init(&pid, 0.0f, 1.0f, 0.0f, -1.0f, 1.0f);
    // Large error for a long time: the output saturates at 1.0.
    for (int i = 0; i < 1000; i++) {
        tr_pid_update(&pid, 100.0f, 0.0f, 0.01f);
    }
    CHECK(pid.integral <= 1.0f + 1e-4f, "integral stops growing while saturated");
    // When the error flips, the output must leave saturation right away.
    float out = tr_pid_update(&pid, -1.0f, 0.0f, 0.01f);
    CHECK(out < 1.0f, "output recovers immediately after the error flips");
}

static void test_no_derivative_kick_on_setpoint_jump(void)
{
    tr_pid_t pid;
    tr_pid_init(&pid, 0.0f, 0.0f, 1.0f, -1000.0f, 1000.0f);
    tr_pid_update(&pid, 0.0f, 5.0f, 0.01f);
    float out = tr_pid_update(&pid, 50.0f, 5.0f, 0.01f);
    CHECK(fabsf(out) < 1e-4f, "a setpoint jump alone gives no derivative output");
}

static void test_closed_loop_settles(void)
{
    // First-order plant: x' = u - x. The PID should drive x to 1.0.
    tr_pid_t pid;
    tr_pid_init(&pid, 2.0f, 1.0f, 0.05f, -10.0f, 10.0f);
    float x = 0.0f;
    const float dt = 0.01f;
    for (int i = 0; i < 2000; i++) {
        float u = tr_pid_update(&pid, 1.0f, x, dt);
        x += (u - x) * dt;
    }
    CHECK(fabsf(x - 1.0f) < 0.01f, "closed loop reaches the setpoint");
}

int main(void)
{
    printf("tr_pid host tests\n");
    test_proportional_only();
    test_output_is_clamped();
    test_no_integral_windup();
    test_no_derivative_kick_on_setpoint_jump();
    test_closed_loop_settles();
    if (failures) {
        printf("%d check(s) failed\n", failures);
        return EXIT_FAILURE;
    }
    printf("all checks passed\n");
    return EXIT_SUCCESS;
}
