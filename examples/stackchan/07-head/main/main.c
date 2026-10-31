// 07-head: read and move StackChan's head over Wi-Fi, from a page the robot
// serves at http://stackchan.local/app or with curl.
//
// The endpoints and the page are in tr_web (components/tr_web), so other
// examples can use them too. tr_ota keeps the robot open for updates.
#include "esp_err.h"
#include "tr_board.h"
#include "tr_ota.h"
#include "tr_web.h"

void app_main(void)
{
    ESP_ERROR_CHECK(tr_ota_add_status(tr_board_status));
    ESP_ERROR_CHECK(tr_ota_start());
    ESP_ERROR_CHECK(tr_web_start());
}
