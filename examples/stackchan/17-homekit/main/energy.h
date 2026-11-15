// energy: the energy each device uses, added up from its power readings. See energy.c.
#pragma once

#include <stdbool.h>
#include "json.h"

// Set the clock from the network (SNTP, time zone CONFIG_HOME_TZ) and load the
// saved history from NVS. After tr_ota_start() and plugs_start().
void energy_start(void);
// A device was read (plugs_start's callback, in the worker task).
void energy_fresh(int dev);
// The clock has been set from the network.
bool energy_clock_ok(void);
// The local time now as "2026-11-15T17:42:10", or "" before the clock is set.
void energy_clock_text(char *out, size_t size);
// The history as one JSON object: totals, the last 48 hours and the last 62 days.
void energy_json(jw_t *w);
