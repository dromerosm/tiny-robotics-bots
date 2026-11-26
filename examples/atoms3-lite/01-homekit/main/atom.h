// atom: the AtomS3 Lite's own parts. The RGB LED (G35) shows how the bridge
// is, and the button (G41) opens the pairing page for a short time. See atom.c.
#pragma once

#include <stdbool.h>
#include "esp_err.h"

#define ATOM_PAIR_S 120 // the pairing page stays open this long after a press

// Start the LED and the button. After bridge_start() and plugs_start().
esp_err_t atom_start(void);

// True for ATOM_PAIR_S seconds after a press on the button.
bool atom_pair_open(void);
