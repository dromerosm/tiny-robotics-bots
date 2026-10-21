// 04-face: StackChan's face on the screen and head moves on the servos, with
// commands on UART0. In the browser emulator you see both next to the console.
// Type "help" to list commands.
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_console.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tr_emu.h"

#define FRAME_MS      66 // about 15 frames per second
#define IDLE_AFTER_US (8 * 1000 * 1000)
#define BG            TR_RGB(0, 0, 0)
#define FG            TR_RGB(255, 255, 255)

static volatile bool s_idle = true;     // look around on its own
static volatile bool s_blink_now;       // set by the "blink" command
static volatile int64_t s_last_command; // esp_timer time of the last "look"

static void look(float pan, float tilt)
{
    tr_servo_set(pan, tilt);
    float p, t;
    tr_servo_get(&p, &t);
    printf("pan=%.0f tilt=%.0f\n", p, t);
}

// The face follows the head: the eyes move a little toward where it looks.
static void draw_face(float pan, float tilt, bool eyes_closed)
{
    int dx = (int)(pan / 180.0f * 24.0f);
    int dy = (int)(-(tilt - 45.0f) / 45.0f * 12.0f);
    tr_screen_fill(BG);
    for (int i = 0; i < 2; i++) {
        int cx = (i ? 230 : 90) + dx;
        int cy = 93 + dy;
        if (eyes_closed) {
            tr_screen_fill_rect(cx - 14, cy - 2, 28, 5, FG);
        } else {
            tr_screen_fill_circle(cx, cy, 12, FG);
        }
    }
    tr_screen_fill_rect(120 + dx, 148 + dy, 80, 4, FG);
    tr_screen_present();
}

static void face_task(void *arg)
{
    int64_t next_blink = esp_timer_get_time() + 2000000;
    int64_t blink_end = 0;
    int64_t next_glance = 0;
    float drawn_pan = 1e9f, drawn_tilt = 1e9f;
    bool drawn_closed = false;
    TickType_t wake = xTaskGetTickCount();

    for (;;) {
        int64_t now = esp_timer_get_time();

        if (s_blink_now || now >= next_blink) {
            s_blink_now = false;
            blink_end = now + 150000;
            next_blink = now + 2500000 + esp_random() % 3000000;
        }
        if (s_idle && now - s_last_command > IDLE_AFTER_US && now >= next_glance) {
            float pan = (float)((int)(esp_random() % 121) - 60);
            float tilt = (float)(30 + esp_random() % 31);
            tr_servo_set(pan, tilt);
            next_glance = now + 1500000 + esp_random() % 2500000;
        }

        float pan, tilt;
        tr_servo_get(&pan, &tilt);
        bool closed = now < blink_end;
        // Only draw when something changed, so the emulator stays fast.
        if (pan != drawn_pan || tilt != drawn_tilt || closed != drawn_closed) {
            draw_face(pan, tilt, closed);
            drawn_pan = pan;
            drawn_tilt = tilt;
            drawn_closed = closed;
        }
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(FRAME_MS));
    }
}

static int cmd_look(int argc, char **argv)
{
    static const struct {
        const char *name;
        float pan, tilt;
    } presets[] = {
        {"left", -60, 45}, {"right", 60, 45}, {"up", 0, 80}, {"down", 0, 10}, {"center", 0, 45},
    };
    s_last_command = esp_timer_get_time();
    if (argc == 2) {
        for (size_t i = 0; i < sizeof(presets) / sizeof(presets[0]); i++) {
            if (strcmp(argv[1], presets[i].name) == 0) {
                look(presets[i].pan, presets[i].tilt);
                return 0;
            }
        }
    } else if (argc == 3) {
        look(strtof(argv[1], NULL), strtof(argv[2], NULL));
        return 0;
    }
    printf("usage: look <pan> <tilt>  (pan -180..180, tilt 5..85)\n"
           "       look left|right|up|down|center\n");
    return 1;
}

static int cmd_center(int argc, char **argv)
{
    s_last_command = esp_timer_get_time();
    look(0, 45);
    return 0;
}

static int cmd_idle(int argc, char **argv)
{
    if (argc == 2 && (strcmp(argv[1], "on") == 0 || strcmp(argv[1], "off") == 0)) {
        s_idle = strcmp(argv[1], "on") == 0;
    } else if (argc != 1) {
        printf("usage: idle [on|off]\n");
        return 1;
    }
    printf("idle %s\n", s_idle ? "on" : "off");
    return 0;
}

static int cmd_blink(int argc, char **argv)
{
    s_blink_now = true;
    return 0;
}

static void register_commands(void)
{
    const esp_console_cmd_t commands[] = {
        {.command = "look",
         .help = "Move the head: look <pan> <tilt>, or look left",
         .func = cmd_look},
        {.command = "center", .help = "Look straight ahead", .func = cmd_center},
        {.command = "idle", .help = "Look around on its own: idle on|off", .func = cmd_idle},
        {.command = "blink", .help = "Blink once", .func = cmd_blink},
    };
    for (size_t i = 0; i < sizeof(commands) / sizeof(commands[0]); i++) {
        ESP_ERROR_CHECK(esp_console_cmd_register(&commands[i]));
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(tr_emu_init());
    xTaskCreate(face_task, "face", 4096, NULL, 5, NULL);

    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_config.prompt = "face>";
    repl_config.max_cmdline_length = 128;

    esp_console_register_help_command();
    register_commands();

    esp_console_dev_uart_config_t uart_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&uart_config, &repl_config, &repl));

    printf("\nStackChan face ready. Try: look left, look 30 70, idle off\n");
    ESP_ERROR_CHECK(esp_console_start_repl(repl));
}
