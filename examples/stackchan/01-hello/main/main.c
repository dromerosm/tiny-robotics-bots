// hello: first StackChan program. Prints a heartbeat and answers simple
// commands on UART0, so it can be tried in the browser emulator.
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"

static const char *TAG = "hello";

#define UART_PORT UART_NUM_0
#define CMD_MAX   64

static void print_info(void)
{
    esp_chip_info_t chip;
    uint32_t flash = 0;
    esp_chip_info(&chip);
    esp_flash_get_size(NULL, &flash);
    printf("chip: %s, %d core(s), revision %d.%d, %lu MB flash\n", CONFIG_IDF_TARGET, chip.cores,
           chip.revision / 100, chip.revision % 100, (unsigned long)(flash / (1024 * 1024)));
    printf("free heap: %lu bytes, uptime: %lld ms\n", (unsigned long)esp_get_free_heap_size(),
           esp_timer_get_time() / 1000);
}

static void handle(char *line)
{
    for (char *p = line; *p; p++)
        *p = (char)tolower((unsigned char)*p);
    if (strcmp(line, "ping") == 0) {
        printf("pong\n");
    } else if (strcmp(line, "info") == 0) {
        print_info();
    } else if (strcmp(line, "help") == 0) {
        printf("commands: ping, info, help\n");
    } else if (line[0] != '\0') {
        printf("unknown command: %s (try help)\n", line);
    }
}

static void heartbeat(void *arg)
{
    for (int n = 1;; n++) {
        ESP_LOGI(TAG, "heartbeat %d", n);
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void app_main(void)
{
    uart_driver_install(UART_PORT, 256, 0, 0, NULL, 0);

    printf("\nHello from StackChan!\n");
    print_info();
    printf("Type a command and press Enter (try: help)\n");

    xTaskCreate(heartbeat, "heartbeat", 2048, NULL, 1, NULL);

    char line[CMD_MAX];
    size_t len = 0;
    uint8_t c;
    for (;;) {
        if (uart_read_bytes(UART_PORT, &c, 1, portMAX_DELAY) != 1)
            continue;
        if (c == '\r')
            continue;
        if (c == '\n') {
            line[len] = '\0';
            handle(line);
            len = 0;
        } else if (len < CMD_MAX - 1) {
            line[len++] = (char)c;
        }
    }
}
