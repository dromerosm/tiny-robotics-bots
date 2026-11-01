// 06-status: the robot's status on its own screen, one section per page.
//
// The status screen is tr_screen's (components/tr_web), the same one that
// 08-face and 09-wifi open on the mouth: battery, robot, network (with the
// update token), touch and motion, and the room. PREV and NEXT change the
// section, and the numbers change every second. A tap on the top bar goes
// back to the first section. tr_web also serves the page at
// http://stackchan.local/app, and tr_ota keeps the robot open for updates.
//
// After a minute with no touch, on battery, the robot sleeps (tr_board): the
// screen goes off and this loop waits. A touch on the screen or the head
// wakes it.
#include <stdio.h>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tr_board.h"
#include "tr_ota.h"
#include "tr_screen.h"
#include "tr_web.h"

#define BRIGHTNESS 20 // screen backlight, in %: less light, less power
#define LOOP_MS    66 // read the touch about 15 times a second

void app_main(void)
{
    ESP_ERROR_CHECK(tr_ota_add_status(tr_board_status));
    ESP_ERROR_CHECK(tr_ota_start());
    ESP_ERROR_CHECK(tr_web_start());

    esp_err_t err = tr_lcd_init();
    if (err != ESP_OK) {
        printf("no screen (%s), status only on http://stackchan.local/\n", esp_err_to_name(err));
        return;
    }
    tr_lcd_brightness(BRIGHTNESS);
    tr_status_open();
    printf("screen ready\n");

    bool was_down = false;
    int tap_x = 0, tap_y = 0;
    int64_t next_update = esp_timer_get_time() + 1000000;
    for (;;) {
        if (tr_sleep_asleep()) { // nothing to draw: wait, then start again
            tr_sleep_wait_awake();
            tr_status_open();
            was_down = false;
            continue;
        }
        // A tap is a touch that ends: act on where the finger was last.
        int x, y;
        const bool down = tr_touch_read(&x, &y);
        if (down) {
            tap_x = x;
            tap_y = y;
        }
        if (was_down && !down) {
            if (tr_screen_on_bar(tap_y)) {
                tr_status_open();
            } else {
                tr_status_tap(tap_x, tap_y);
            }
        }
        was_down = down;
        const int64_t now = esp_timer_get_time();
        if (now >= next_update) {
            tr_status_update();
            next_update = now + 1000000;
        }
        vTaskDelay(pdMS_TO_TICKS(LOOP_MS));
    }
}
