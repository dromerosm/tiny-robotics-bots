// atom: see atom.h.
//
// The LED, from the worst state to the best:
//   red          no Wi-Fi
//   blue         Wi-Fi, but not paired with Home
//   blue, blink  the pairing page is open (after a press on the button)
//   amber        paired, but a plug does not answer
//   dim green    all good
// A press on the button flashes it white once.
//
// The LED is a WS2812: 24 bits (green, red, blue) in pulses of 0.3 and 0.9 us.
// The RMT peripheral sends them, with the bytes encoder of ESP-IDF, so there is
// no extra library. The LED is sent a color only when the color changes.
//
// One small task looks at the button and the state every 50 ms. A bridge is on
// all day, so it does nothing else.
#include "atom.h"

#include "bridge.h"
#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "plugs.h"

static const char *TAG = "atom";

#define LED_GPIO    35
#define BUTTON_GPIO 41 // low while pressed
#define TICK_MS     50
#define BLINK_MS    500
#define FLASH_MS    150

static rmt_channel_handle_t led_chan;
static rmt_encoder_handle_t led_enc;
static volatile int64_t pair_until; // esp_timer time, in us

bool atom_pair_open(void)
{
    return esp_timer_get_time() < pair_until;
}

static esp_err_t led_init(void)
{
    const rmt_tx_channel_config_t chan = {
        .gpio_num = LED_GPIO,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000, // 0.1 us per tick
        .mem_block_symbols = 48,
        .trans_queue_depth = 1,
    };
    const rmt_bytes_encoder_config_t bytes = {
        .bit0 = {.level0 = 1, .duration0 = 3, .level1 = 0, .duration1 = 9},
        .bit1 = {.level0 = 1, .duration0 = 9, .level1 = 0, .duration1 = 3},
        .flags.msb_first = 1,
    };
    esp_err_t err = rmt_new_tx_channel(&chan, &led_chan);
    if (err == ESP_OK) {
        err = rmt_new_bytes_encoder(&bytes, &led_enc);
    }
    if (err == ESP_OK) {
        err = rmt_enable(led_chan);
    }
    return err;
}

// rgb is 0xRRGGBB.
static void led_set(uint32_t rgb)
{
    static uint32_t shown = 0xFFFFFFFF;
    if (!led_chan || rgb == shown) {
        return;
    }
    const uint8_t grb[3] = {(uint8_t)(rgb >> 8), (uint8_t)(rgb >> 16), (uint8_t)rgb};
    const rmt_transmit_config_t once = {0};
    if (rmt_transmit(led_chan, led_enc, grb, sizeof(grb), &once) == ESP_OK &&
        rmt_tx_wait_all_done(led_chan, 100) == ESP_OK) {
        shown = rgb;
    }
}

static bool wifi_up(void)
{
    esp_netif_t *sta = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    esp_netif_ip_info_t ip;
    return sta && esp_netif_get_ip_info(sta, &ip) == ESP_OK && ip.ip.addr != 0;
}

static bool plugs_ok(void)
{
    for (int dev = 0; dev < plugs_count(); dev++) {
        if (plugs_switches(dev) > 0 && plugs_state(dev, 0) < 0) {
            return false;
        }
    }
    return true;
}

// Dim colors: the LED is very bright, and it is on all day.
#define RED   0x100000
#define BLUE  0x000014
#define AMBER 0x0C0500
#define GREEN 0x000500
#define WHITE 0x101010
#define OFF   0x000000

static uint32_t state_color(int64_t now)
{
    if (!wifi_up()) {
        return RED;
    }
    if (atom_pair_open() && !bridge_paired()) {
        return now / 1000 / BLINK_MS % 2 ? BLUE : OFF;
    }
    if (!bridge_paired()) {
        return BLUE;
    }
    return plugs_ok() ? GREEN : AMBER;
}

static void atom_task(void *arg)
{
    bool was_down = false;
    int64_t flash_until = 0;
    for (;;) {
        const int64_t now = esp_timer_get_time();
        const bool down = gpio_get_level(BUTTON_GPIO) == 0;
        if (down && !was_down) {
            pair_until = now + ATOM_PAIR_S * 1000000LL;
            flash_until = now + FLASH_MS * 1000LL;
            ESP_LOGI(TAG, "button: pairing page open for %d s", ATOM_PAIR_S);
        }
        was_down = down;
        led_set(now < flash_until ? WHITE : state_color(now));
        vTaskDelay(pdMS_TO_TICKS(TICK_MS));
    }
}

esp_err_t atom_start(void)
{
    const gpio_config_t button = {
        .pin_bit_mask = 1ULL << BUTTON_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    esp_err_t err = gpio_config(&button);
    if (err != ESP_OK) {
        return err;
    }
    if (led_init() != ESP_OK) {
        ESP_LOGW(TAG, "no LED");
        led_chan = NULL;
    }
    return xTaskCreate(atom_task, "atom", 2560, NULL, 3, NULL) == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}
