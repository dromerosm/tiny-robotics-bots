// bridge: the plugs as a HomeKit bridge, for Apple's Home app. See bridge.c.
#pragma once

#include <stdbool.h>
#include "esp_err.h"

// Start HomeKit, after tr_ota_start() and plugs_start().
esp_err_t bridge_start(void);
// A device was read (plugs_start's callback): tell Home what changed.
void bridge_fresh(int dev);

// The code to type in the Home app ("123-45-678"), made on the first boot.
const char *bridge_code(void);
// The same code as the text of a QR code for the Home app ("X-HM://...").
const char *bridge_qr_text(void);
// How many iPhones or home hubs are paired. 0: waiting for the first one.
int bridge_paired(void);
// Forget every pairing. The robot restarts.
void bridge_unpair(void);
