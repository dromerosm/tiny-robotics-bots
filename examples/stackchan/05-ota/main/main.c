// 05-ota: update StackChan over Wi-Fi, with rollback.
//
// All the work is in the tr_ota component (components/tr_ota): home Wi-Fi,
// http://stackchan.local/, POST /update with a token, and rollback. Every
// example that runs on the robot calls tr_ota_start() the same way, so it can
// always take the next update.
#include "esp_err.h"
#include "tr_ota.h"

void app_main(void)
{
    ESP_ERROR_CHECK(tr_ota_start());
}
