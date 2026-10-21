// 03-tasks: three FreeRTOS tasks that work together, the way a robot reads a
// sensor, filters it and reacts. The "sensor" is a simulated distance sensor
// (like the Unit ToF4M), so it runs without hardware.
//
//   sensor task --(queue)--> filter task --(event group)--> alert task
#include <math.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_random.h"

static const char *TAG = "tasks";

#define SAMPLE_PERIOD_MS 50  // 20 readings per second
#define AVERAGE_WINDOW   8   // moving average over the last 8 readings
#define OBSTACLE_MM      300 // closer than this counts as an obstacle
#define OBSTACLE_BIT     BIT0

typedef struct {
    uint32_t time_ms;
    uint16_t distance_mm;
} reading_t;

static QueueHandle_t readings;
static EventGroupHandle_t events;

// Simulated distance: a slow wave between 150 and 1200 mm plus some noise,
// as if something moved toward the robot and away again every 6 seconds.
static uint16_t simulated_distance(uint32_t time_ms)
{
    const float wave = sinf(2.0f * (float)M_PI * (float)time_ms / 6000.0f);
    const float noise = (float)(esp_random() % 41) - 20.0f; // -20..+20 mm
    return (uint16_t)(675.0f + 525.0f * wave + noise);
}

static void sensor_task(void *arg)
{
    TickType_t last_wake = xTaskGetTickCount();
    for (;;) {
        reading_t r = {.time_ms = pdTICKS_TO_MS(xTaskGetTickCount())};
        r.distance_mm = simulated_distance(r.time_ms);
        if (xQueueSend(readings, &r, 0) != pdTRUE) {
            ESP_LOGW(TAG, "queue full, reading dropped");
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }
}

static void filter_task(void *arg)
{
    uint16_t window[AVERAGE_WINDOW] = {0};
    uint32_t count = 0, sum = 0;
    for (;;) {
        reading_t r;
        xQueueReceive(readings, &r, portMAX_DELAY);

        // Moving average with a ring buffer.
        sum -= window[count % AVERAGE_WINDOW];
        window[count % AVERAGE_WINDOW] = r.distance_mm;
        sum += r.distance_mm;
        count++;
        const uint32_t n = count < AVERAGE_WINDOW ? count : AVERAGE_WINDOW;
        const uint32_t average = sum / n;

        if (average < OBSTACLE_MM) {
            xEventGroupSetBits(events, OBSTACLE_BIT);
        } else {
            xEventGroupClearBits(events, OBSTACLE_BIT);
        }
        if (count % 20 == 0) { // once per second
            ESP_LOGI(TAG, "distance avg=%lu mm (last %u mm)", (unsigned long)average,
                     r.distance_mm);
        }
    }
}

static void alert_task(void *arg)
{
    for (;;) {
        xEventGroupWaitBits(events, OBSTACLE_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
        ESP_LOGW(TAG, "obstacle closer than %d mm!", OBSTACLE_MM);
        while (xEventGroupGetBits(events) & OBSTACLE_BIT) {
            vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
        }
        ESP_LOGI(TAG, "path clear");
    }
}

void app_main(void)
{
    printf("\nStackChan tasks demo: sensor -> filter -> alert\n");
    readings = xQueueCreate(16, sizeof(reading_t));
    events = xEventGroupCreate();
    configASSERT(readings && events);

    // Higher number = higher priority. The sensor must never miss its period.
    xTaskCreate(sensor_task, "sensor", 3072, NULL, 5, NULL);
    xTaskCreate(filter_task, "filter", 3072, NULL, 4, NULL);
    xTaskCreate(alert_task, "alert", 3072, NULL, 3, NULL);
}
