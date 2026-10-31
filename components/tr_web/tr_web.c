// tr_web: the robot's web page and head endpoints, on tr_ota's server.
//
//   GET  /head                          both servos: angle, raw position, voltage, temperature
//   POST /head?yaw=20&pitch=45&ms=1000  move (token needed); &hold=1 keeps the torque on
//   POST /head/off                      torque and servo power off (token needed)
//   POST /head/zero?yaw=436&pitch=585   save new zero positions, raw 0..1023 (token needed)
//   GET  /i2c                           the chips on the internal I2C bus
//   GET  /sensors                       touch, motion, clock, power chip, temperature
//   GET  /leds, POST /leds?color=ff8800&side=left|right|all   side LEDs (token needed)
//   GET  /camera.jpg                    one camera frame, JPEG
//   GET  /app                           a page with all of the above
//
// Nothing moves at boot. After a move the torque goes off again, unless hold=1,
// so the servos do not fight a hand that turns the head. Pitch stays between
// 5 and 85 degrees, yaw between -90 and 90 (tr_board clamps the angles).
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_check.h"
#include "esp_http_server.h"
#include "img_converters.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tr_board.h"
#include "esp_timer.h"
#include "tr_ota.h"
#include "tr_web.h"

static bool servos_found;
static esp_err_t servo_err = ESP_ERR_INVALID_STATE;
static bool powered;
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

static esp_err_t send_error(httpd_req_t *req, const char *status, const char *text)
{
    httpd_resp_set_status(req, status);
    httpd_resp_sendstr(req, text);
    return ESP_FAIL;
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
    if (!powered) {
        esp_err_t err = tr_servo_power(true);
        if (err != ESP_OK) {
            return err;
        }
        powered = true;
    }
    return ESP_OK;
}

// POST /head?yaw=..&pitch=..&ms=..&hold=1
static esp_err_t head_post(httpd_req_t *req)
{
    if (!tr_ota_token_ok(req)) {
        return ESP_FAIL;
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
        return ESP_FAIL;
    }
    if (powered) {
        tr_servo_torque(TR_SERVO_YAW, false);
        tr_servo_torque(TR_SERVO_PITCH, false);
        tr_servo_power(false);
        powered = false;
    }
    return httpd_resp_sendstr(req, "servo power: off\n");
}

// POST /head/zero?yaw=436&pitch=585
static esp_err_t head_zero(httpd_req_t *req)
{
    if (!tr_ota_token_ok(req)) {
        return ESP_FAIL;
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

// GET /i2c: which chips answer on the internal bus.
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
        return ESP_FAIL;
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
    if (err != ESP_OK) {
        return send_error(req, "500 Internal Server Error", "LEDs did not answer\n");
    }
    return leds_get(req);
}

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

// GET /sensors: all the other sensors, as text.
static esp_err_t sensors_get(httpd_req_t *req)
{
    static char text[1024];
    tr_sensors_status(text, sizeof(text));
    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    return httpd_resp_sendstr(req, text);
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
        {.uri = "/leds", .method = HTTP_GET, .handler = leds_get},
        {.uri = "/leds", .method = HTTP_POST, .handler = leds_post},
        {.uri = "/camera.jpg", .method = HTTP_GET, .handler = camera_get},
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
    return ESP_OK;
}
