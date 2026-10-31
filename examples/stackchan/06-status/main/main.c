// 06-status: the robot's status on its own screen, new every second.
//
// The same facts as http://stackchan.local/ (tr_ota): firmware, slot, uptime,
// last restart, Wi-Fi, battery and free memory. tr_ota also keeps the robot
// open for the next update over Wi-Fi.
#include <stdarg.h>
#include <stdio.h>
#include "esp_app_desc.h"
#include "esp_netif.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tr_board.h"
#include "tr_ota.h"

#define COLS     26 // characters per line at scale 2 (320 / 12)
#define TOP      40 // first status line, below the title
#define LINE_H   20 // 16 pixels of text and 4 of space
#define GREY     TR_LCD_RGB(150, 150, 150)
#define GREEN    TR_LCD_RGB(80, 220, 120)
#define YELLOW   TR_LCD_RGB(255, 210, 60)
#define RED      TR_LCD_RGB(255, 80, 80)
#define TITLE_BG TR_LCD_RGB(20, 60, 120)

// Print one line, padded with spaces so it covers the old text.
static void line(int row, uint16_t color, const char *fmt, ...)
{
    char text[COLS + 1];
    va_list args;
    va_start(args, fmt);
    int n = vsnprintf(text, sizeof(text), fmt, args);
    va_end(args);
    for (n = n < 0 ? 0 : n > COLS ? COLS : n; n < COLS; n++) {
        text[n] = ' ';
    }
    text[COLS] = 0;
    tr_lcd_text(4, TOP + row * LINE_H, 2, color, TR_LCD_BLACK, text);
}

static void draw_status(void)
{
    const esp_app_desc_t *app = esp_app_get_description();
    const char *state = tr_ota_state();
    line(0, GREY, "fw   %s", app->version);
    line(1, state[0] == 'v' ? GREEN : YELLOW, "slot %s %s", esp_ota_get_running_partition()->label,
         state);

    const int64_t s = esp_timer_get_time() / 1000000;
    line(2, TR_LCD_WHITE, "up   %lld:%02lld:%02lld", s / 3600, s / 60 % 60, s % 60);
    line(3, GREY, "boot %s", tr_ota_last_restart());

    wifi_ap_record_t ap;
    esp_netif_ip_info_t ip = {0};
    if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
        esp_netif_get_ip_info(esp_netif_get_handle_from_ifkey("WIFI_STA_DEF"), &ip);
        line(4, GREEN, "wifi %s", (const char *)ap.ssid);
        line(5, ap.rssi > -70 ? GREEN : YELLOW, "     %d dBm, channel %d", ap.rssi, ap.primary);
        line(6, TR_LCD_WHITE, "ip   " IPSTR, IP2STR(&ip.ip));
    } else {
        line(4, YELLOW, "wifi not connected");
        line(5, GREY, "");
        line(6, GREY, "");
    }

    tr_battery_t b;
    if (tr_battery_read(&b) == ESP_OK) {
        line(7, b.millivolts < 3400 ? RED : TR_LCD_WHITE, "bat  %d.%02d V %d mA %s",
             b.millivolts / 1000, b.millivolts % 1000 / 10,
             b.milliamps < 0 ? -b.milliamps : b.milliamps,
             b.milliamps > 0   ? "out"
             : b.milliamps < 0 ? "in"
                               : "");
    } else {
        line(7, GREY, "bat  no reading");
    }
    line(8, GREY, "mem  %lu KB free", (unsigned long)esp_get_free_heap_size() / 1024);
}

void app_main(void)
{
    ESP_ERROR_CHECK(tr_ota_add_status(tr_board_status));
    ESP_ERROR_CHECK(tr_ota_start());

    esp_err_t err = tr_lcd_init();
    if (err != ESP_OK) {
        printf("no screen (%s), status only on http://stackchan.local/\n", esp_err_to_name(err));
        return;
    }
    tr_lcd_fill_rect(0, 0, TR_LCD_W, 30, TITLE_BG);
    tr_lcd_text(8, 7, 2, TR_LCD_WHITE, TITLE_BG, "StackChan status");
    printf("screen ready\n");
    for (;;) {
        draw_status();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
