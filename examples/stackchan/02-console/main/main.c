// 02-console: an interactive command line on UART0 with ESP-IDF's console
// component. Type "help" to list commands.
#include <stdio.h>
#include "esp_chip_info.h"
#include "esp_console.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_timer.h"

static int cmd_info(int argc, char **argv)
{
    esp_chip_info_t chip;
    uint32_t flash = 0;
    esp_chip_info(&chip);
    esp_flash_get_size(NULL, &flash);
    printf("chip: %s, %d core(s), revision %d.%d, %lu MB flash\n", CONFIG_IDF_TARGET, chip.cores,
           chip.revision / 100, chip.revision % 100, (unsigned long)(flash / (1024 * 1024)));
    return 0;
}

static int cmd_heap(int argc, char **argv)
{
    printf("free heap: %lu bytes (minimum since boot: %lu)\n",
           (unsigned long)esp_get_free_heap_size(),
           (unsigned long)esp_get_minimum_free_heap_size());
    return 0;
}

static int cmd_uptime(int argc, char **argv)
{
    int64_t ms = esp_timer_get_time() / 1000;
    printf("uptime: %lld.%03lld s\n", ms / 1000, ms % 1000);
    return 0;
}

static int cmd_echo(int argc, char **argv)
{
    for (int i = 1; i < argc; i++) {
        printf("%s%s", argv[i], i + 1 < argc ? " " : "");
    }
    printf("\n");
    return 0;
}

static int cmd_restart(int argc, char **argv)
{
    printf("restarting...\n");
    esp_restart();
    return 0;
}

static void register_commands(void)
{
    const esp_console_cmd_t commands[] = {
        {.command = "info", .help = "Show chip and flash details", .func = cmd_info},
        {.command = "heap", .help = "Show free heap memory", .func = cmd_heap},
        {.command = "uptime", .help = "Show time since boot", .func = cmd_uptime},
        {.command = "echo", .help = "Print the arguments back", .func = cmd_echo},
        {.command = "restart", .help = "Restart the chip", .func = cmd_restart},
    };
    for (size_t i = 0; i < sizeof(commands) / sizeof(commands[0]); i++) {
        ESP_ERROR_CHECK(esp_console_cmd_register(&commands[i]));
    }
}

void app_main(void)
{
    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_config.prompt = "stackchan>";
    repl_config.max_cmdline_length = 128;

    esp_console_register_help_command();
    register_commands();

    esp_console_dev_uart_config_t uart_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&uart_config, &repl_config, &repl));

    printf("\nStackChan console ready.\n");
    ESP_ERROR_CHECK(esp_console_start_repl(repl));
}
