# tr_pid

A small PID controller in plain C. It has no ESP-IDF dependency, so the same code runs on the robot and in the host unit tests.

- **Derivative on measurement:** a sudden setpoint change does not cause an output spike.
- **Anti-windup:** the integral stops growing while the output is saturated, so the loop recovers quickly.
- **No dynamic memory.**

## Use it

Add the components folder to your project's `CMakeLists.txt`, before `project()`:

```cmake
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../../components")
```

Then require it in `main/CMakeLists.txt` with `PRIV_REQUIRES tr_pid`, and:

```c
#include "tr_pid.h"

tr_pid_t pid;
tr_pid_init(&pid, 30.0f, 5.0f, 1.5f, -20.0f, 20.0f);  // kp, ki, kd, out_min, out_max

float out = tr_pid_update(&pid, setpoint, measurement, dt_seconds);
```

Call `tr_pid_reset()` when the loop restarts (for example after the robot falls).

## Tests

```bash
scripts/test-host.sh
```

They check the proportional term, output limits, anti-windup, the absence of derivative kick and that a closed loop settles.
