// tr_web: the robot's web page (http://stackchan.local/app) and the head
// endpoints, on tr_ota's server. See tr_web.c for the list.
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Add the endpoints to tr_ota and look for the servos. Nothing moves.
// Call it before or after tr_ota_start().
esp_err_t tr_web_start(void);

// True if tr_web_start() found the servos (not in the emulator).
bool tr_web_servos(void);

// True once servo power is on (the servos only answer then).
bool tr_web_head_powered(void);

// Turn on servo power, once. The page and the example share this state.
esp_err_t tr_web_head_power(void);
// Torque and servo power off (as POST /head/off). The next
// tr_web_head_power() waits until the servos answer again.
void tr_web_head_off(void);

// esp_timer time of the last move from the page or curl, or -1. An example
// that moves the head on its own waits a while after this.
int64_t tr_web_last_move_us(void);

#ifdef __cplusplus
}
#endif
