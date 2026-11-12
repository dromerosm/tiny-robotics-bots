// tr_web: the robot's web page and head endpoints, on tr_ota's server.
//
//   GET  /head                          both servos: angle, raw position, voltage, temperature
//   POST /head?yaw=20&pitch=45&ms=1000  move (token needed); &hold=1 keeps the torque on
//   POST /head/off                      torque and servo power off (token needed)
//   POST /head/zero?yaw=436&pitch=585   save new zero positions, raw 0..1023 (token needed)
//   GET  /i2c                           the chips on the internal I2C bus and the Grove ports
//   GET  /sensors                       touch, motion, clock, power chip, temperature
//   POST /tof?zone=8                    how wide the distance sensor looks, 4 to 16 (token needed)
//   GET  /leds, POST /leds?color=ff8800&side=left|right|all   side LEDs (token needed)
//   GET  /camera.jpg                    one camera frame, JPEG (CONFIG_TR_WEB_CAMERA)
//   GET  /sleep                         awake or asleep, and the battery use
//   POST /sleep?now=1|wake=1|after=60   sleep now, wake now, or the seconds before sleep (token
//   needed) GET  /app                           a page with all of the above
//
// Nothing moves at boot. After a move the torque goes off again, unless hold=1,
// so the servos do not fight a hand that turns the head. Pitch stays between
// 5 and 85 degrees, yaw between -90 and 90 (tr_board clamps the angles).
//
// It also starts tr_board's sleep: when the robot sleeps, the servos and the
// side LEDs go off, and they come back when it wakes. A request that moves the
// head or lights the LEDs wakes the robot first.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_check.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "img_converters.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tr_board.h"
#include "esp_timer.h"
#include "tr_ota.h"
#include "tr_web.h"

static bool servos_found;
static esp_err_t servo_err = ESP_ERR_INVALID_STATE;
static bool powered, powered_before_sleep;
static char head_before_sleep[2][160]; // the servos' last reading, shown while asleep
static volatile int64_t last_move_us = -1;

static bool query_float(httpd_req_t *req, const char *key, float *value)
{
    char query[96], text[16];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
        httpd_query_key_value(query, key, text, sizeof(text)) != ESP_OK) {
        return false;
    }
    char *end;
    *value = strtof(text, &end);
    return end != text;
}

// An error answer, with a line in /log that says which request and why.
static esp_err_t send_error(httpd_req_t *req, const char *status, const char *text)
{
    ESP_LOGW("tr_web", "%s: %.3s, %.*s", req->uri, status, (int)strcspn(text, "\n"), text);
    tr_ota_send_error(req, status, text);
    return ESP_OK;
}

static void describe(tr_servo_t id, const char *name, char *line, size_t size)
{
    tr_servo_state_t s;
    const esp_err_t err = tr_servo_read(id, &s);
    if (err != ESP_OK) {
        snprintf(line, size, "%s: no answer (%s)\n", name, esp_err_to_name(err));
        return;
    }
    snprintf(line, size, "%s: %.1f deg (raw %d, zero %d), torque %s, %s, load %d, %d.%d V, %d C\n",
             name, s.degrees, s.raw, s.zero, s.torque ? "on" : "off", s.moving ? "moving" : "still",
             s.load, s.volts_x10 / 10, s.volts_x10 % 10, s.celsius);
}

// GET /head
static esp_err_t head_get(httpd_req_t *req)
{
    char line[160];
    if (!servos_found) {
        snprintf(line, sizeof(line), "no servos (%s)\n", esp_err_to_name(servo_err));
        return httpd_resp_sendstr(req, line);
    }
    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    if (!powered && tr_sleep_asleep() && head_before_sleep[0][0]) {
        httpd_resp_sendstr_chunk(req, "servo power: off while the robot sleeps (last reading)\n");
        httpd_resp_sendstr_chunk(req, head_before_sleep[0]);
        httpd_resp_sendstr_chunk(req, head_before_sleep[1]);
        return httpd_resp_sendstr_chunk(req, NULL);
    }
    snprintf(line, sizeof(line), "servo power: %s\n", powered ? "on" : "off");
    httpd_resp_sendstr_chunk(req, line);
    if (powered) {
        describe(TR_SERVO_YAW, "yaw  ", line, sizeof(line));
        httpd_resp_sendstr_chunk(req, line);
        describe(TR_SERVO_PITCH, "pitch", line, sizeof(line));
        httpd_resp_sendstr_chunk(req, line);
    } else {
        httpd_resp_sendstr_chunk(req, "(the servos answer only with power on: POST /head)\n");
    }
    return httpd_resp_sendstr_chunk(req, NULL);
}

esp_err_t tr_web_head_power(void)
{
    if (tr_sleep_asleep()) {
        return ESP_ERR_INVALID_STATE; // the servos stay off while the robot sleeps
    }
    if (!powered) {
        esp_err_t err = tr_servo_power(true);
        if (err != ESP_OK) {
            return err;
        }
        powered = true;
    }
    return ESP_OK;
}

void tr_web_head_off(void)
{
    if (powered) {
        tr_servo_torque(TR_SERVO_YAW, false);
        tr_servo_torque(TR_SERVO_PITCH, false);
        tr_servo_power(false);
        powered = false;
    }
}

// POST /head?yaw=..&pitch=..&ms=..&hold=1
static esp_err_t head_post(httpd_req_t *req)
{
    if (!tr_ota_token_ok(req)) {
        return ESP_OK;
    }
    if (!servos_found) {
        return send_error(req, "409 Conflict", "no servos\n");
    }
    float yaw, pitch, ms = 1000, hold = 0;
    const bool has_yaw = query_float(req, "yaw", &yaw);
    const bool has_pitch = query_float(req, "pitch", &pitch);
    query_float(req, "ms", &ms);
    query_float(req, "hold", &hold);
    last_move_us = esp_timer_get_time();
    ESP_LOGI("tr_web", "head move from the page or curl: %s", req->uri); // to find it in /log
    tr_sleep_wake();
    if (tr_web_head_power() != ESP_OK) {
        return send_error(req, "500 Internal Server Error", "servo power failed\n");
    }
    esp_err_t err = ESP_OK;
    if (has_yaw) {
        err = tr_servo_move(TR_SERVO_YAW, yaw, (int)ms);
    }
    if (err == ESP_OK && has_pitch) {
        err = tr_servo_move(TR_SERVO_PITCH, pitch, (int)ms);
    }
    if (err != ESP_OK) {
        return send_error(req, "500 Internal Server Error", "servo did not answer\n");
    }
    if (has_yaw || has_pitch) {
        vTaskDelay(pdMS_TO_TICKS((int)ms + 300)); // let the move finish
        if (hold == 0) {
            tr_servo_torque(TR_SERVO_YAW, false);
            tr_servo_torque(TR_SERVO_PITCH, false);
        }
    }
    return head_get(req);
}

// POST /head/off
static esp_err_t head_off(httpd_req_t *req)
{
    if (!tr_ota_token_ok(req)) {
        return ESP_OK;
    }
    tr_web_head_off();
    return httpd_resp_sendstr(req, "servo power: off\n");
}

// POST /head/zero?yaw=436&pitch=585
static esp_err_t head_zero(httpd_req_t *req)
{
    if (!tr_ota_token_ok(req)) {
        return ESP_OK;
    }
    float v;
    esp_err_t err = ESP_OK;
    if (query_float(req, "yaw", &v)) {
        err = tr_servo_set_zero(TR_SERVO_YAW, (int)v);
    }
    if (err == ESP_OK && query_float(req, "pitch", &v)) {
        err = tr_servo_set_zero(TR_SERVO_PITCH, (int)v);
    }
    if (err != ESP_OK) {
        return send_error(req, "400 Bad Request", "zero must be raw 0..1023\n");
    }
    return head_get(req);
}

// GET /i2c: which chips answer on the internal bus and on the Grove ports.
static esp_err_t i2c_get(httpd_req_t *req)
{
    static const struct {
        uint8_t addr;
        const char *name;
    } known[] = {
        {0x10, "BMM150 magnetometer"}, {0x21, "GC0308 camera"},   {0x23, "LTR553 light"},
        {0x34, "AXP2101 power"},       {0x36, "AW88298 speaker"}, {0x38, "FT6336 touch"},
        {0x40, "ES7210 microphone"},   {0x41, "INA226 battery"},  {0x50, "ST25R3916 NFC"},
        {0x51, "BM8563 clock"},        {0x58, "AW9523 IO"},       {0x68, "Si12T head touch"},
        {0x69, "BMI270 motion"},       {0x6F, "PY32 IO (body)"},  {0x71, "PY32 IO (body, alt)"},
    };
    uint8_t found[32];
    const int n = tr_board_scan(found, 32);
    char line[64];
    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    for (int i = 0; i < n; i++) {
        const char *name = "?";
        for (size_t k = 0; k < sizeof(known) / sizeof(known[0]); k++) {
            if (known[k].addr == found[i]) {
                name = known[k].name;
            }
        }
        snprintf(line, sizeof(line), "0x%02X %s\n", found[i], name);
        httpd_resp_sendstr_chunk(req, line);
    }
    // The three Grove ports of the body: what is plugged in, if anything.
    static const struct {
        char name;
        int sda, scl;
    } ports[] = {{'A', 2, 1}, {'B', 9, 8}, {'C', 17, 18}};
    for (size_t p = 0; p < sizeof(ports) / sizeof(ports[0]); p++) {
        const int m = tr_board_scan_port(ports[p].sda, ports[p].scl, found, 32);
        if (m < 0) {
            snprintf(line, sizeof(line),
                     "---- port %c: a line is held low (a unit with no power?)\n", ports[p].name);
            httpd_resp_sendstr_chunk(req, line);
        }
        for (int i = 0; i < m; i++) {
            snprintf(line, sizeof(line), "0x%02X %s, port %c\n", found[i],
                     found[i] == 0x29 ? "VL53L1X distance" : "?", ports[p].name);
            httpd_resp_sendstr_chunk(req, line);
        }
    }
    return httpd_resp_sendstr_chunk(req, NULL);
}

// Side LEDs: 0 to 5 on the left, 6 to 11 on the right (as the BSP numbers them).
static uint32_t led_color[TR_LED_COUNT];

static esp_err_t leds_get(httpd_req_t *req)
{
    char text[96];
    snprintf(text, sizeof(text), "left: #%06lx\nright: #%06lx\n", (unsigned long)led_color[0],
             (unsigned long)led_color[TR_LED_COUNT - 1]);
    return httpd_resp_sendstr(req, text);
}

// POST /leds?color=ff8800&side=left|right|all (token needed). color=000000 is off.
static esp_err_t leds_post(httpd_req_t *req)
{
    if (!tr_ota_token_ok(req)) {
        return ESP_OK;
    }
    if (!servos_found) {
        return send_error(req, "409 Conflict", "no PY32, so no LEDs\n");
    }
    char query[64], value[16] = "", side[8] = "all";
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        httpd_query_key_value(query, "color", value, sizeof(value));
        httpd_query_key_value(query, "side", side, sizeof(side));
    }
    char *end;
    const unsigned long rgb = strtoul(value[0] == '#' ? value + 1 : value, &end, 16);
    if (end == value || rgb > 0xFFFFFF) {
        return send_error(req, "400 Bad Request", "color must be RRGGBB, like ff8800\n");
    }
    tr_sleep_wake();
    const int from = strcmp(side, "right") == 0 ? TR_LED_COUNT / 2 : 0;
    const int to = strcmp(side, "left") == 0 ? TR_LED_COUNT / 2 : TR_LED_COUNT;
    esp_err_t err = ESP_OK;
    for (int i = from; i < to && err == ESP_OK; i++) {
        led_color[i] = rgb;
        err = tr_led_set(i, rgb >> 16, rgb >> 8 & 0xFF, rgb & 0xFF);
    }
    if (err == ESP_OK) {
        err = tr_led_show();
    }
    if (err == ESP_ERR_INVALID_STATE) {
        return send_error(req, "409 Conflict", "battery too low for the LEDs: plug in USB\n");
    }
    if (err != ESP_OK) {
        return send_error(req, "500 Internal Server Error", "LEDs did not answer\n");
    }
    return leds_get(req);
}

#if CONFIG_TR_WEB_CAMERA
// GET /camera.jpg: one frame from the camera, as JPEG. Starts the camera the first time.
static esp_err_t camera_get(httpd_req_t *req)
{
    if (tr_cam_start() != ESP_OK) {
        return send_error(req, "503 Service Unavailable", "no camera\n");
    }
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        return send_error(req, "503 Service Unavailable", "no frame\n");
    }
    uint8_t *jpg = NULL;
    size_t len = 0;
    const bool ok = frame2jpg(fb, 80, &jpg, &len);
    esp_camera_fb_return(fb);
    if (!ok) {
        return send_error(req, "500 Internal Server Error", "JPEG failed\n");
    }
    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    const esp_err_t err = httpd_resp_send(req, (const char *)jpg, len);
    free(jpg);
    return err;
}
#endif

// HEAD /camera.jpg: 200 if this app has the camera, 404 if not, with no body
// (an answer to HEAD must not have one). The page asks it to show its camera
// card. Without this, the server's own 404 sent a body, and browsers then
// closed the connection with a reset.
static esp_err_t camera_head(httpd_req_t *req)
{
#if CONFIG_TR_WEB_CAMERA
    httpd_resp_set_type(req, "image/jpeg");
#else
    httpd_resp_set_status(req, "404 Not Found");
#endif
    return httpd_resp_send(req, NULL, 0);
}

// Power and IO registers, for debugging the board's power setup.
static const struct {
    const char *name;
    uint8_t addr;
    uint8_t regs[16];
    int n;
} reg_chips[] = {
    {"axp",
     0x34,
     {0x00, 0x01, 0x27, 0x30, 0x69, 0x80, 0x90, 0x91, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x99},
     15},
    {"aw", 0x58, {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x11, 0x12, 0x13}, 9},
    {"py32",
     0x6F,
     {0x02, 0x03, 0x04, 0x05, 0x06, 0x09, 0x0A, 0x0B, 0x0C, 0x13, 0x14, 0x24, 0x30, 0x31},
     14},
};

// GET /regs: the registers above, as "chip reg=value".
static esp_err_t regs_get(httpd_req_t *req)
{
    char line[48];
    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    for (size_t c = 0; c < sizeof(reg_chips) / sizeof(reg_chips[0]); c++) {
        for (int i = 0; i < reg_chips[c].n; i++) {
            uint8_t v = 0;
            const esp_err_t err = tr_board_reg_read(reg_chips[c].addr, reg_chips[c].regs[i], &v);
            if (err == ESP_OK) {
                snprintf(line, sizeof(line), "%s 0x%02X=0x%02X\n", reg_chips[c].name,
                         reg_chips[c].regs[i], v);
            } else {
                snprintf(line, sizeof(line), "%s 0x%02X=%s\n", reg_chips[c].name,
                         reg_chips[c].regs[i], esp_err_to_name(err));
            }
            httpd_resp_sendstr_chunk(req, line);
        }
    }
    return httpd_resp_sendstr_chunk(req, NULL);
}

// POST /reg?chip=aw&reg=0x03&val=0x8F: write one register (token needed).
static esp_err_t reg_post(httpd_req_t *req)
{
    if (!tr_ota_token_ok(req)) {
        return ESP_OK;
    }
    char query[64], chip[8] = "", reg[8] = "", val[8] = "";
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
        httpd_query_key_value(query, "chip", chip, sizeof(chip)) != ESP_OK ||
        httpd_query_key_value(query, "reg", reg, sizeof(reg)) != ESP_OK ||
        httpd_query_key_value(query, "val", val, sizeof(val)) != ESP_OK) {
        return send_error(req, "400 Bad Request", "need chip, reg and val\n");
    }
    for (size_t c = 0; c < sizeof(reg_chips) / sizeof(reg_chips[0]); c++) {
        if (strcmp(chip, reg_chips[c].name) == 0) {
            const uint8_t r = strtoul(reg, NULL, 0), v = strtoul(val, NULL, 0);
            ESP_LOGW("tr_web", "write %s 0x%02X=0x%02X", chip, r, v); // in /log, in case it resets
            if (tr_board_reg_write(reg_chips[c].addr, r, v) != ESP_OK) {
                return send_error(req, "500 Internal Server Error", "write failed\n");
            }
            return httpd_resp_sendstr(req, "ok\n");
        }
    }
    return send_error(req, "400 Bad Request", "chip is axp, aw or py32\n");
}

// GET /sensors: all the other sensors, as text.
static esp_err_t sensors_get(httpd_req_t *req)
{
    static char text[1024];
    tr_sensors_status(text, sizeof(text));
    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    return httpd_resp_sendstr(req, text);
}

// POST /tof?zone=8 (token needed): how wide the distance sensor looks, 4
// (narrow) to 16 (wide). Kept until the next restart.
static esp_err_t tof_post(httpd_req_t *req)
{
    if (!tr_ota_token_ok(req)) {
        return ESP_OK;
    }
    float v;
    if (!query_float(req, "zone", &v) || tr_tof_set_zone((int)v) == ESP_ERR_INVALID_ARG) {
        return send_error(req, "400 Bad Request", "zone must be 4 to 16\n");
    }
    char text[48];
    snprintf(text, sizeof(text), "distance zone: %d x %d of 16 x 16\n", tr_tof_zone(),
             tr_tof_zone());
    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    return httpd_resp_sendstr(req, text);
}

// GET /sleep: the sleep state and the battery use, as on GET /.
static esp_err_t sleep_get(httpd_req_t *req)
{
    char text[192];
    tr_board_status(text, sizeof(text));
    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    return httpd_resp_sendstr(req, text);
}

// POST /sleep?now=1 (&steps=0x1F to measure single steps), wake=1, after=60.
static esp_err_t sleep_post(httpd_req_t *req)
{
    if (!tr_ota_token_ok(req)) {
        return ESP_OK;
    }
    char query[64], value[12];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) {
        return send_error(req, "400 Bad Request", "use now=1, wake=1 or after=<seconds>\n");
    }
    if (httpd_query_key_value(query, "after", value, sizeof(value)) == ESP_OK) {
        tr_sleep_set_after(atoi(value));
    }
    if (httpd_query_key_value(query, "wake", value, sizeof(value)) == ESP_OK &&
        tr_sleep_wake() != ESP_OK) {
        return send_error(req, "500 Internal Server Error", "did not wake\n");
    }
    if (httpd_query_key_value(query, "now", value, sizeof(value)) == ESP_OK) {
        uint32_t steps = TR_SLEEP_ALL;
        if (httpd_query_key_value(query, "steps", value, sizeof(value)) == ESP_OK) {
            steps = strtoul(value, NULL, 0);
        }
        tr_sleep_now(steps);
        vTaskDelay(pdMS_TO_TICKS(300)); // let it go to sleep, so the answer shows it
    }
    return sleep_get(req);
}

// Before the robot sleeps: torque and servo power off, LEDs off (the colors
// stay in led_color). After it wakes: both back as they were.

static void show_leds(bool on)
{
    bool any = false;
    for (int i = 0; i < TR_LED_COUNT; i++) {
        any |= led_color[i] != 0;
        const uint32_t c = on ? led_color[i] : 0;
        tr_led_set(i, c >> 16, c >> 8 & 0xFF, c & 0xFF);
    }
    if (any) {
        tr_led_show();
    }
}

static void on_sleep(bool going_to_sleep)
{
    if (!servos_found) {
        return;
    }
    if (going_to_sleep) {
        powered_before_sleep = powered;
        head_before_sleep[0][0] = 0;
        if (powered) {
            tr_servo_torque(TR_SERVO_YAW, false);
            tr_servo_torque(TR_SERVO_PITCH, false);
            describe(TR_SERVO_YAW, "yaw  ", head_before_sleep[0], sizeof(head_before_sleep[0]));
            describe(TR_SERVO_PITCH, "pitch", head_before_sleep[1], sizeof(head_before_sleep[1]));
            tr_servo_power(false);
            powered = false;
        }
        show_leds(false);
    } else {
        if (powered_before_sleep) {
            tr_web_head_power(); // so the example can read where the head is
        }
        show_leds(true);
    }
}

// GET /app: a small page with the status and buttons to move the head.
extern const char app_html[] asm("_binary_app_html_start");

static esp_err_t app_get(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    return httpd_resp_sendstr(req, app_html);
}

bool tr_web_head_powered(void)
{
    return powered;
}

bool tr_web_servos(void)
{
    return servos_found;
}

int64_t tr_web_last_move_us(void)
{
    return last_move_us;
}

esp_err_t tr_web_start(void)
{
    const httpd_uri_t uris[] = {
        {.uri = "/head", .method = HTTP_GET, .handler = head_get},
        {.uri = "/head", .method = HTTP_POST, .handler = head_post},
        {.uri = "/head/off", .method = HTTP_POST, .handler = head_off},
        {.uri = "/head/zero", .method = HTTP_POST, .handler = head_zero},
        {.uri = "/i2c", .method = HTTP_GET, .handler = i2c_get},
        {.uri = "/app", .method = HTTP_GET, .handler = app_get},
        {.uri = "/sensors", .method = HTTP_GET, .handler = sensors_get},
        {.uri = "/tof", .method = HTTP_POST, .handler = tof_post},
        {.uri = "/leds", .method = HTTP_GET, .handler = leds_get},
        {.uri = "/leds", .method = HTTP_POST, .handler = leds_post},
#if CONFIG_TR_WEB_CAMERA
        {.uri = "/camera.jpg", .method = HTTP_GET, .handler = camera_get},
#endif
        {.uri = "/camera.jpg", .method = HTTP_HEAD, .handler = camera_head},
        {.uri = "/sleep", .method = HTTP_GET, .handler = sleep_get},
        {.uri = "/sleep", .method = HTTP_POST, .handler = sleep_post},
        {.uri = "/regs", .method = HTTP_GET, .handler = regs_get},
        {.uri = "/reg", .method = HTTP_POST, .handler = reg_post},
    };
    for (size_t i = 0; i < sizeof(uris) / sizeof(uris[0]); i++) {
        ESP_RETURN_ON_ERROR(tr_ota_add_uri(&uris[i]), "tr_web", "add %s", uris[i].uri);
    }
    tr_sensors_init();
    servo_err = tr_servo_init();
    servos_found = servo_err == ESP_OK;
    if (servos_found) {
        printf("servos ready, power off until the first move\n");
    } else {
        printf("no servos (%s)\n", esp_err_to_name(servo_err));
    }
    tr_sleep_add_hook(on_sleep);
    return tr_sleep_start();
}
