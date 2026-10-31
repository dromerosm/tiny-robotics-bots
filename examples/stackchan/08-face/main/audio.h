// The audio analyzer screen (audio.c).
#pragma once

#include "esp_err.h"

// Start the microphones (once) and draw the empty screen with its title bar.
esp_err_t audio_open(void);
// Read 32 ms of sound and draw the bars. Takes about 60 ms.
esp_err_t audio_frame(void);
