// 16-servos: StackChan's head servos from the console on UART0, with the real
// driver in tr_board (SCS packets on UART1, power through the PY32).
//
// On the robot it moves the head. In the browser emulator a virtual part
// plays the body (the PY32 and both servos), so the same image moves the head
// on the page. Type "help" to list commands.
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_console.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tr_board.h"
#include "tr_ota.h"

#define MOVE_MS 1000 // default time for a move

static bool s_body;    // the PY32 answered at boot
static bool s_powered; // servo power is on

static bool need_body(void)
{
    if (!s_body) {
        printf("no servos: the body (PY32) did not answer at boot\n");
    }
    return s_body;
}

static esp_err_t power(bool on)
{
    const esp_err_t err = tr_servo_power(on);
    s_powered = on && err == ESP_OK;
    printf("power %s: %s\n", on ? "on" : "off", esp_err_to_name(err));
    return err;
}

// Move both servos and wait until the move is over.
static esp_err_t move(float yaw, float pitch, int ms)
{
    if (!s_powered && power(true) != ESP_OK) {
        return ESP_FAIL;
    }
    esp_err_t err = tr_servo_move(TR_SERVO_YAW, yaw, ms);
    if (err == ESP_OK) {
        err = tr_servo_move(TR_SERVO_PITCH, pitch, ms);
    }
    if (err != ESP_OK) {
        printf("move: %s\n", esp_err_to_name(err));
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(ms + 50));
    return ESP_OK;
}

static void print_servo(const char *name, tr_servo_t id)
{
    tr_servo_state_t s;
    const esp_err_t err = tr_servo_read(id, &s);
    if (err != ESP_OK) {
        printf("%s: %s\n", name, esp_err_to_name(err));
        return;
    }
    printf("%s: %.1f deg (raw %d, zero %d), torque %s, %s, %d.%d V, %d C\n", name, s.degrees, s.raw,
           s.zero, s.torque ? "on" : "off", s.moving ? "moving" : "still", s.volts_x10 / 10,
           s.volts_x10 % 10, s.celsius);
}

static int cmd_read(int argc, char **argv)
{
    if (!need_body()) {
        return 1;
    }
    if (!s_powered) {
        printf("power is off: type \"power on\" or move first\n");
        return 1;
    }
    print_servo("yaw", TR_SERVO_YAW);
    print_servo("pitch", TR_SERVO_PITCH);
    return 0;
}

static int cmd_move(int argc, char **argv)
{
    if (argc < 3) {
        printf("usage: move <yaw> <pitch> [ms]  (yaw -90..90, pitch 5..85, ms 100..5000)\n");
        return 1;
    }
    if (!need_body()) {
        return 1;
    }
    const int ms = argc > 3 ? atoi(argv[3]) : MOVE_MS;
    if (move(strtof(argv[1], NULL), strtof(argv[2], NULL), ms) != ESP_OK) {
        return 1;
    }
    return cmd_read(0, NULL);
}

static int cmd_center(int argc, char **argv)
{
    if (!need_body()) {
        return 1;
    }
    return move(0, 45, MOVE_MS) == ESP_OK ? cmd_read(0, NULL) : 1;
}

// Say yes, then no.
static int cmd_nod(int argc, char **argv)
{
    if (!need_body()) {
        return 1;
    }
    const float steps[][2] = {{0, 25}, {0, 65}, {0, 25}, {0, 45}, {-40, 45}, {40, 45}, {0, 45}};
    for (size_t i = 0; i < sizeof(steps) / sizeof(steps[0]); i++) {
        if (move(steps[i][0], steps[i][1], 400) != ESP_OK) {
            return 1;
        }
    }
    printf("nod done\n");
    return 0;
}

static int cmd_power(int argc, char **argv)
{
    if (argc < 2 || (strcmp(argv[1], "on") && strcmp(argv[1], "off"))) {
        printf("usage: power on|off\n");
        return 1;
    }
    if (!need_body()) {
        return 1;
    }
    return power(!strcmp(argv[1], "on")) == ESP_OK ? 0 : 1;
}

static int cmd_torque(int argc, char **argv)
{
    if (argc < 2 || (strcmp(argv[1], "on") && strcmp(argv[1], "off"))) {
        printf("usage: torque on|off\n");
        return 1;
    }
    if (!need_body() || (!s_powered && power(true) != ESP_OK)) {
        return 1;
    }
    const bool on = !strcmp(argv[1], "on");
    esp_err_t err = tr_servo_torque(TR_SERVO_YAW, on);
    if (err == ESP_OK) {
        err = tr_servo_torque(TR_SERVO_PITCH, on);
    }
    printf("torque %s: %s\n", argv[1], esp_err_to_name(err));
    return err == ESP_OK ? 0 : 1;
}

static void register_commands(void)
{
    const esp_console_cmd_t commands[] = {
        {.command = "move", .help = "Move the head: move <yaw> <pitch> [ms]", .func = cmd_move},
        {.command = "center", .help = "Look straight ahead", .func = cmd_center},
        {.command = "nod", .help = "Say yes, then no", .func = cmd_nod},
        {.command = "read", .help = "Read both servos", .func = cmd_read},
        {.command = "power", .help = "Servo power: power on|off", .func = cmd_power},
        {.command = "torque", .help = "Hold or free the head: torque on|off", .func = cmd_torque},
    };
    for (size_t i = 0; i < sizeof(commands) / sizeof(commands[0]); i++) {
        ESP_ERROR_CHECK(esp_console_cmd_register(&commands[i]));
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(tr_ota_start());

    const esp_err_t err = tr_servo_init();
    s_body = err == ESP_OK;
    printf("\n%s\n", s_body ? "servos: body found (PY32), power off until the first move"
                            : "no servos: the body (PY32) did not answer");

    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_config.prompt = "servos>";
    repl_config.max_cmdline_length = 64;
    esp_console_register_help_command();
    register_commands();
    esp_console_dev_uart_config_t uart_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&uart_config, &repl_config, &repl));
    ESP_ERROR_CHECK(esp_console_start_repl(repl));
}
