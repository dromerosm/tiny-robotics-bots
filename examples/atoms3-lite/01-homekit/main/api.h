// api: JSON endpoints and a page on the bridge's web server (port 80). See api.c.
//
//   GET  /api/status                 the bridge, HomeKit's clients, the Apple TVs, the bridge
//   GET  /api/plugs                  every device: outlets, power, voltage, current
//   POST /api/plugs?dev=&sw=&on=0|1  switch an outlet
//   GET  /api/energy                 Wh and euros per device: total, last 48 hours, last 62 days
//   GET  /panel                      a page that shows all of it
#pragma once

#include "esp_err.h"

// Add the endpoints to tr_ota's web server. After energy_start() and info_init().
esp_err_t api_start(void);
