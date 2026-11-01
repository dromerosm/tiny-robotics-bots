// tr_sleep: turn off what nobody uses when nobody uses the robot, and wake on
// a touch. See include/tr_board.h.
//
// One small task does it all: every 100 ms it checks for a touch (when no
// example did), once a second it reads the battery and the USB power, and it
// puts the robot to sleep or wakes it. Wi-Fi stays on the whole time.
#include <stdio.h>
#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_pm.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "tr_board.h"
#include "tr_board_priv.h"

static const char *TAG = "tr_sleep";

#define TICK_MS     100
#define POLL_US     300000 // poll the touch ourselves if no example did for this long
#define BATTERY_MAH 550    // StackChan's battery (M5Stack's docs)
#define MAX_HOOKS   4
#define AWAKE_BIT   BIT0

static TaskHandle_t task;
static EventGroupHandle_t events;
static tr_sleep_fn hooks[MAX_HOOKS];
static volatile bool asleep, wake_asked, sleep_asked, usb;
static volatile int64_t last_activity, last_poll[2], asleep_since;
static volatile int after_s = CONFIG_TR_SLEEP_AFTER_S;
static volatile uint32_t steps_asked = TR_SLEEP_ALL;
static uint32_t steps_done;
static bool screen_given_light = true;
static float avg_ma; // above 0 while the battery discharges
static volatile int minutes_left = -1;
#if CONFIG_PM_ENABLE
static esp_pm_lock_handle_t no_light_sleep, cpu_max;
static bool pm_on;
#endif

void tr_sleep_activity(void)
{
    last_activity = esp_timer_get_time();
}

bool tr_sleep_asleep(void)
{
    return asleep;
}

void tr_sleep_wait_awake(void)
{
    if (events) {
        xEventGroupWaitBits(events, AWAKE_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
    }
}

esp_err_t tr_sleep_wake(void)
{
    tr_sleep_activity();
    if (!task || !asleep) {
        return ESP_OK;
    }
    wake_asked = true;
    xTaskNotifyGive(task);
    const EventBits_t bits =
        xEventGroupWaitBits(events, AWAKE_BIT, pdFALSE, pdTRUE, pdMS_TO_TICKS(3000));
    return bits & AWAKE_BIT ? ESP_OK : ESP_ERR_TIMEOUT;
}

esp_err_t tr_sleep_now(uint32_t steps)
{
    ESP_RETURN_ON_FALSE(task, ESP_ERR_INVALID_STATE, TAG, "not started");
    steps_asked = steps & TR_SLEEP_ALL;
    sleep_asked = true;
    xTaskNotifyGive(task);
    return ESP_OK;
}

void tr_sleep_set_after(int seconds)
{
    after_s = seconds < 0 ? 0 : seconds;
    tr_sleep_activity();
}

esp_err_t tr_sleep_add_hook(tr_sleep_fn fn)
{
    for (int i = 0; i < MAX_HOOKS; i++) {
        if (!hooks[i] || hooks[i] == fn) {
            hooks[i] = fn;
            return ESP_OK;
        }
    }
    return ESP_ERR_NO_MEM;
}

// A touch on the screen or the head. While asleep it wakes the robot, and the
// example does not see it until the finger lets go.
bool tr_sleep_touch(int source, bool down)
{
    static bool swallow[2];
    last_poll[source] = esp_timer_get_time();
    if (!down) {
        swallow[source] = false;
        return false;
    }
    tr_sleep_activity();
    if (asleep) {
        swallow[source] = true;
        wake_asked = true;
        if (task) {
            xTaskNotifyGive(task);
        }
    }
    return !swallow[source];
}

float tr_battery_average_ma(void)
{
    return avg_ma > 0 ? avg_ma : 0;
}

int tr_battery_minutes_left(void)
{
    return minutes_left;
}

// Once a second: the INA226 current, averaged over about 20 s, and the time
// left: 550 mAh times the power chip's charge, over that average. A guess:
// it assumes the robot keeps using what it uses now.
static void battery_tick(void)
{
    tr_battery_t b;
    if (tr_battery_read(&b) == ESP_OK) {
        if (b.milliamps <= 0 || avg_ma <= 0) {
            avg_ma = b.milliamps; // charging, on USB, or a new start
        } else {
            avg_ma += (b.milliamps - avg_ma) * 0.05f;
        }
    }
    tr_power_t p;
    if (tr_power_read(&p) == ESP_OK) {
        usb = p.usb;
        minutes_left = avg_ma >= 5 ? (int)(BATTERY_MAH * p.percent / 100.0f / avg_ma * 60) : -1;
    }
}

static void run_hooks(bool going_to_sleep)
{
    for (int i = 0; i < MAX_HOOKS && hooks[i]; i++) {
        hooks[i](going_to_sleep);
    }
}

static void go_to_sleep(uint32_t steps)
{
    const int64_t t0 = esp_timer_get_time();
    asleep = true;
    asleep_since = t0;
    xEventGroupClearBits(events, AWAKE_BIT);
    steps_done = steps;
    if (steps & TR_SLEEP_HOOKS) {
        run_hooks(true); // servos and LEDs (tr_web), the example's own
    }
    if (steps & TR_SLEEP_SCREEN) {
        tr_lcd_power(false);
    }
    if (steps & (TR_SLEEP_CAMERA | TR_SLEEP_MOTION)) {
        tr_sensors_sleep(true, steps & TR_SLEEP_CAMERA, steps & TR_SLEEP_MOTION);
    }
#if CONFIG_PM_ENABLE
    if (pm_on && (steps & TR_SLEEP_CPU)) {
        esp_pm_lock_release(cpu_max);
        esp_pm_lock_release(no_light_sleep);
    }
#endif
    ESP_LOGI(TAG, "asleep (steps 0x%02lX) in %lld ms, task stack left %u bytes",
             (unsigned long)steps, (esp_timer_get_time() - t0) / 1000,
             (unsigned)uxTaskGetStackHighWaterMark(NULL));
}

static void wake_up(void)
{
    const int64_t t0 = esp_timer_get_time();
    const uint32_t steps = steps_done;
#if CONFIG_PM_ENABLE
    if (pm_on && (steps & TR_SLEEP_CPU)) {
        esp_pm_lock_acquire(no_light_sleep);
        esp_pm_lock_acquire(cpu_max);
    }
#endif
    if (steps & (TR_SLEEP_CAMERA | TR_SLEEP_MOTION)) {
        tr_sensors_sleep(false, steps & TR_SLEEP_CAMERA, steps & TR_SLEEP_MOTION);
    }
    if (steps & TR_SLEEP_SCREEN) {
        tr_lcd_power(true);
    }
    asleep = false;
    if (steps & TR_SLEEP_HOOKS) {
        run_hooks(false);
    }
    xEventGroupSetBits(events, AWAKE_BIT);
    tr_sleep_activity();
    ESP_LOGI(TAG, "awake in %lld ms", (esp_timer_get_time() - t0) / 1000);
}

static void sleep_task(void *arg)
{
    int64_t next_second = 0;
    const int64_t started = esp_timer_get_time();
    for (;;) {
        const int64_t now = esp_timer_get_time();
        if (now >= next_second) {
            battery_tick();
            next_second = now + 1000000;
        }
        // A screen nobody set up gets no light (05-ota, 07-head). The screen
        // examples set theirs up in their first second.
        if (screen_given_light && now - started > 3000000 && !tr_lcd_ready()) {
            screen_given_light = false;
            tr_lcd_power(false);
        }
        // Check the touch ourselves when no example does: always while
        // asleep (the examples wait), and in examples without a screen.
        int x, y;
        if (now - last_poll[0] >= POLL_US) {
            tr_touch_read(&x, &y);
        }
        if (now - last_poll[1] >= POLL_US) {
            tr_head_touch();
        }
        if (asleep) {
            sleep_asked = false; // already asleep
            if (wake_asked || usb) {
                wake_asked = false;
                wake_up();
            }
        } else {
            wake_asked = false;
            if (sleep_asked) {
                sleep_asked = false;
                go_to_sleep(steps_asked);
            } else if (!usb && after_s > 0 && now - last_activity >= after_s * 1000000LL) {
                go_to_sleep(TR_SLEEP_ALL);
            }
        }
        // Wait for the next tick, or less if someone asks to wake or sleep.
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(TICK_MS));
    }
}

esp_err_t tr_sleep_start(void)
{
    if (task) {
        return ESP_OK;
    }
    events = xEventGroupCreate();
    ESP_RETURN_ON_FALSE(events, ESP_ERR_NO_MEM, TAG, "events");
    xEventGroupSetBits(events, AWAKE_BIT);
    tr_sleep_activity();
#if CONFIG_PM_ENABLE
    // Light sleep only on the robot: without the power chip (as in the
    // emulator) the robot stays at full speed.
    tr_power_t p;
    if (tr_power_read(&p) == ESP_OK) {
        ESP_RETURN_ON_ERROR(esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0, "awake", &no_light_sleep),
                            TAG, "lock");
        ESP_RETURN_ON_ERROR(esp_pm_lock_create(ESP_PM_CPU_FREQ_MAX, 0, "awake", &cpu_max), TAG,
                            "lock");
        esp_pm_lock_acquire(no_light_sleep);
        esp_pm_lock_acquire(cpu_max);
        const esp_pm_config_t pm = {
            .max_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
            .min_freq_mhz = 80,
            .light_sleep_enable = true,
        };
        // In light sleep the ESP32-S3 turns its pins off (a workaround for a
        // reset on wake, ESP_SLEEP_GPIO_RESET_WORKAROUND). Keep the ones we use
        // as they are: internal I2C, screen, servo UART. Without this the green
        // LED at the bottom blinked with every tick.
        static const gpio_num_t keep[] = {0, 11, 12, 3, 35, 36, 37, 6, 7};
        for (size_t i = 0; i < sizeof(keep) / sizeof(keep[0]); i++) {
            gpio_sleep_sel_dis(keep[i]);
        }
        const esp_err_t err = esp_pm_configure(&pm);
        pm_on = err == ESP_OK;
        if (!pm_on) {
            ESP_LOGW(TAG, "no light sleep: %s", esp_err_to_name(err));
        }
    }
#endif
    ESP_RETURN_ON_FALSE(xTaskCreate(sleep_task, "sleep", 3072, NULL, 4, &task) == pdPASS,
                        ESP_ERR_NO_MEM, TAG, "task");
    ESP_LOGI(TAG, "sleeps after %d s with no touch, on battery", after_s);
    return ESP_OK;
}

void tr_sleep_status(char *text, size_t size)
{
    const int64_t now = esp_timer_get_time();
    if (!task) {
        snprintf(text, size, "sleep: not started\n");
    } else if (asleep) {
        const int64_t s = (now - asleep_since) / 1000000;
        snprintf(text, size, "sleep: asleep for %lld min %02lld s\n", s / 60, s % 60);
    } else if (usb) {
        snprintf(text, size, "sleep: awake, never on USB power\n");
    } else if (after_s == 0) {
        snprintf(text, size, "sleep: awake, sleep is off\n");
    } else {
        const int64_t left = after_s - (now - last_activity) / 1000000;
        snprintf(text, size, "sleep: awake, sleeps in %lld s\n", left > 0 ? left : 0);
    }
}
