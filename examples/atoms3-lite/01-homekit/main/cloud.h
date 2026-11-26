// cloud: sends the energy history to the home-energy Worker once an hour. See cloud.c.
#pragma once

#include "esp_err.h"
#include "json.h"

// After energy_start(). Does nothing when CONFIG_HOME_ENERGY_URL is empty.
esp_err_t cloud_start(void);
// The last sends, as one JSON object, for GET /api/status.
void cloud_json(jw_t *w);
