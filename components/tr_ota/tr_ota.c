// tr_ota: home Wi-Fi, updates over Wi-Fi and rollback. See include/tr_ota.h.
#include "tr_ota.h"

#include <stdio.h>
#include <string.h>
#include "esp_app_desc.h"
#include "esp_check.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_ota_ops.h"
#include "esp_random.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "mdns.h"
#include "nvs.h"
#include "nvs_flash.h"

static const char *TAG = "tr_ota";

#define HOSTNAME     "stackchan"
#define TRIAL_WIFI_S 60 // an app on trial with no Wi-Fi after this goes back to the old app
#define RETRY_MS     5000

typedef struct {
    const char *ssid;
    const char *password;
} network_t;

// Empty SSIDs (no wifi.env at build time) fall back to esp-emu's built-in
// network, so the examples also run in the emulator.
static const network_t home[] = {
    {CONFIG_TR_WIFI_SSID_01, CONFIG_TR_WIFI_PASSWORD_01},
    {CONFIG_TR_WIFI_SSID_02, CONFIG_TR_WIFI_PASSWORD_02},
};
static const network_t emulator = {"myssid", "mypassword"};

#define GOT_IP BIT0
#define LOST   BIT1
static EventGroupHandle_t wifi_events;
static char token[17];
static esp_netif_t *sta;

#define MAX_STATUS 4
static tr_ota_status_fn status_fns[MAX_STATUS];

static const char *state_name(esp_ota_img_states_t state)
{
    switch (state) {
        case ESP_OTA_IMG_NEW:
            return "new";
        case ESP_OTA_IMG_PENDING_VERIFY:
            return "on trial";
        case ESP_OTA_IMG_VALID:
            return "valid";
        case ESP_OTA_IMG_INVALID:
            return "invalid";
        case ESP_OTA_IMG_ABORTED:
            return "aborted";
        default:
            return "no state";
    }
}

static esp_ota_img_states_t running_state(void)
{
    esp_ota_img_states_t state = ESP_OTA_IMG_UNDEFINED;
    esp_ota_get_state_partition(esp_ota_get_running_partition(), &state);
    return state;
}

// One line about the running app, for the serial port and for GET /.
static void describe_app(char *text, size_t size)
{
    const esp_app_desc_t *app = esp_app_get_description();
    snprintf(text, size, "%s %s, built %s %s, running from %s (%s)\n", app->project_name,
             app->version, app->date, app->time, esp_ota_get_running_partition()->label,
             state_name(running_state()));
}

// Read the update token from NVS, or make a new one on the first boot. It is
// shown only on the USB port, and flashing over USB erases it.
static esp_err_t load_token(void)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open("ota", NVS_READWRITE, &nvs);
    if (err != ESP_OK) {
        return err;
    }
    size_t len = sizeof(token);
    if (nvs_get_str(nvs, "token", token, &len) != ESP_OK) {
        const char *chars = "abcdefghijkmnpqrstuvwxyz23456789";
        for (int i = 0; i < 16; i++) {
            token[i] = chars[esp_random() % strlen(chars)];
        }
        token[16] = 0;
        err = nvs_set_str(nvs, "token", token);
        if (err == ESP_OK) {
            err = nvs_commit(nvs);
        }
    }
    nvs_close(nvs);
    return err;
}

static void on_wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupClearBits(wifi_events, GOT_IP);
        xEventGroupSetBits(wifi_events, LOST);
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        const ip_event_got_ip_t *got = data;
        printf("Wi-Fi: robot at " IPSTR ", http://" HOSTNAME ".local/\n", IP2STR(&got->ip_info.ip));
        xEventGroupSetBits(wifi_events, GOT_IP);
    }
}

// Scan, and pick the known network with the strongest signal. NULL if none is in range.
static const network_t *strongest_network(int *rssi)
{
    const network_t *list = home;
    size_t count = sizeof(home) / sizeof(home[0]);
    if (!home[0].ssid[0] && !home[1].ssid[0]) {
        list = &emulator;
        count = 1;
    }

    if (esp_wifi_scan_start(NULL, true) != ESP_OK) {
        return NULL;
    }
    uint16_t found = 20;
    static wifi_ap_record_t aps[20];
    esp_wifi_scan_get_ap_records(&found, aps);

    const network_t *best = NULL;
    for (uint16_t i = 0; i < found; i++) {
        for (size_t n = 0; n < count; n++) {
            if (list[n].ssid[0] && strcmp((const char *)aps[i].ssid, list[n].ssid) == 0 &&
                (!best || aps[i].rssi > *rssi)) {
                best = &list[n];
                *rssi = aps[i].rssi;
            }
        }
    }
    return best;
}

// Join the strongest network, and join again whenever the link drops.
static void wifi_task(void *arg)
{
    for (;;) {
        int rssi = 0;
        const network_t *net = strongest_network(&rssi);
        if (!net) {
            printf("Wi-Fi: no known network in range, scanning again\n");
            vTaskDelay(pdMS_TO_TICKS(RETRY_MS));
            continue;
        }
        printf("Wi-Fi: joining %s (%d dBm)\n", net->ssid, rssi);
        wifi_config_t cfg = {0};
        strlcpy((char *)cfg.sta.ssid, net->ssid, sizeof(cfg.sta.ssid));
        strlcpy((char *)cfg.sta.password, net->password, sizeof(cfg.sta.password));
        xEventGroupClearBits(wifi_events, LOST);
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &cfg));
        esp_wifi_connect();
        xEventGroupWaitBits(wifi_events, LOST, pdFALSE, pdFALSE, portMAX_DELAY);
        printf("Wi-Fi: link lost\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ESP_ERR_INVALID_STATE means the example already did it.
static esp_err_t ok_if_done(esp_err_t err)
{
    return err == ESP_ERR_INVALID_STATE ? ESP_OK : err;
}

static esp_err_t start_wifi(void)
{
    wifi_events = xEventGroupCreate();
    ESP_RETURN_ON_ERROR(ok_if_done(esp_netif_init()), TAG, "netif");
    ESP_RETURN_ON_ERROR(ok_if_done(esp_event_loop_create_default()), TAG, "event loop");
    sta = esp_netif_create_default_wifi_sta();
    esp_netif_set_hostname(sta, HOSTNAME);
    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&init), TAG, "wifi init");
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, on_wifi_event, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, on_wifi_event, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    xTaskCreate(wifi_task, "wifi", 4096, NULL, 5, NULL);
    return ESP_OK;
}

// POST /update: write the new app to the other slot, then restart.
static esp_err_t update_handler(httpd_req_t *req)
{
    // The server is on the home network, so only someone who saw the USB port may update.
    char given[sizeof(token)] = "";
    if (httpd_req_get_hdr_value_str(req, "X-Token", given, sizeof(given)) != ESP_OK ||
        strcmp(given, token) != 0) {
        httpd_resp_set_status(req, "403 Forbidden");
        httpd_resp_sendstr(req, "wrong or missing X-Token\n");
        return ESP_FAIL;
    }
    const esp_partition_t *slot = esp_ota_get_next_update_partition(NULL);
    esp_ota_handle_t ota;
    if (esp_ota_begin(slot, OTA_WITH_SEQUENTIAL_WRITES, &ota) != ESP_OK) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    printf("update: receiving %d bytes into %s\n", req->content_len, slot->label);
    static char buf[4096];
    int left = req->content_len;
    while (left > 0) {
        const int n = httpd_req_recv(req, buf, left < (int)sizeof(buf) ? left : sizeof(buf));
        if (n == HTTPD_SOCK_ERR_TIMEOUT) {
            continue;
        }
        if (n <= 0 || esp_ota_write(ota, buf, n) != ESP_OK) {
            esp_ota_abort(ota);
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }
        left -= n;
    }
    // esp_ota_end() checks the image (chip, size, checksum) before we boot from it.
    if (esp_ota_end(ota) != ESP_OK || esp_ota_set_boot_partition(slot) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad image");
        return ESP_FAIL;
    }
    httpd_resp_sendstr(req, "ok, restarting\n");
    printf("update: done, restarting\n");
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
    return ESP_OK;
}

static const char *reset_name(esp_reset_reason_t reason)
{
    switch (reason) {
        case ESP_RST_POWERON:
            return "power on";
        case ESP_RST_SW:
            return "restart from software (an update?)";
        case ESP_RST_PANIC:
            return "crash";
        case ESP_RST_INT_WDT:
        case ESP_RST_TASK_WDT:
        case ESP_RST_WDT:
            return "watchdog";
        case ESP_RST_BROWNOUT:
            return "brownout (low power)";
        case ESP_RST_USB:
            return "USB reset";
        case ESP_RST_DEEPSLEEP:
            return "wake from deep sleep";
        default:
            return "other";
    }
}

static void describe_wifi(char *text, size_t size)
{
    wifi_ap_record_t ap;
    esp_netif_ip_info_t ip = {0};
    if (esp_wifi_sta_get_ap_info(&ap) != ESP_OK) {
        snprintf(text, size, "Wi-Fi: not connected\n");
        return;
    }
    esp_netif_get_ip_info(sta, &ip);
    snprintf(text, size, "Wi-Fi: %s, %d dBm, channel %d, IP " IPSTR "\n", (const char *)ap.ssid,
             ap.rssi, ap.primary, IP2STR(&ip.ip));
}

// GET /: the robot's status as plain text, one item per line.
static esp_err_t info_handler(httpd_req_t *req)
{
    char line[192];
    describe_app(line, sizeof(line));
    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    httpd_resp_sendstr_chunk(req, line);

    const int64_t s = esp_timer_get_time() / 1000000;
    snprintf(line, sizeof(line), "uptime: %lld h %02lld min %02lld s\n", s / 3600, s / 60 % 60,
             s % 60);
    httpd_resp_sendstr_chunk(req, line);
    snprintf(line, sizeof(line), "last restart: %s\n", reset_name(esp_reset_reason()));
    httpd_resp_sendstr_chunk(req, line);
    snprintf(line, sizeof(line), "free memory: %lu KB (lowest since boot %lu KB)\n",
             (unsigned long)esp_get_free_heap_size() / 1024,
             (unsigned long)esp_get_minimum_free_heap_size() / 1024);
    httpd_resp_sendstr_chunk(req, line);
    describe_wifi(line, sizeof(line));
    httpd_resp_sendstr_chunk(req, line);

    for (int i = 0; i < MAX_STATUS && status_fns[i]; i++) {
        line[0] = 0;
        status_fns[i](line, sizeof(line));
        httpd_resp_sendstr_chunk(req, line);
    }
    return httpd_resp_sendstr_chunk(req, NULL);
}

const char *tr_ota_state(void)
{
    return state_name(running_state());
}

const char *tr_ota_last_restart(void)
{
    return reset_name(esp_reset_reason());
}

esp_err_t tr_ota_add_status(tr_ota_status_fn fn)
{
    for (int i = 0; i < MAX_STATUS; i++) {
        if (!status_fns[i]) {
            status_fns[i] = fn;
            return ESP_OK;
        }
    }
    return ESP_ERR_NO_MEM;
}

static esp_err_t start_http(void)
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.recv_wait_timeout = 10;
    httpd_handle_t server;
    ESP_RETURN_ON_ERROR(httpd_start(&server, &cfg), TAG, "http");
    const httpd_uri_t update = {.uri = "/update", .method = HTTP_POST, .handler = update_handler};
    const httpd_uri_t info = {.uri = "/", .method = HTTP_GET, .handler = info_handler};
    httpd_register_uri_handler(server, &update);
    httpd_register_uri_handler(server, &info);
    return ESP_OK;
}

// stackchan.local, so we need not look up the address the router gave.
static void start_mdns(void)
{
    if (mdns_init() == ESP_OK) {
        mdns_hostname_set(HOSTNAME);
        mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);
    }
}

// An app on trial: valid once it has Wi-Fi (the web server already runs),
// otherwise back to the old app.
static void trial_task(void *arg)
{
    if (xEventGroupWaitBits(wifi_events, GOT_IP, pdFALSE, pdFALSE,
                            pdMS_TO_TICKS(TRIAL_WIFI_S * 1000)) &
        GOT_IP) {
        ESP_ERROR_CHECK(esp_ota_mark_app_valid_cancel_rollback());
        printf("new app marked valid\n");
    } else {
        printf("no Wi-Fi within %d s, going back to the old app\n", TRIAL_WIFI_S);
        esp_ota_mark_app_invalid_rollback_and_reboot();
    }
    vTaskDelete(NULL);
}

esp_err_t tr_ota_start(void)
{
    char text[160];
    describe_app(text, sizeof(text));
    printf("\n%s", text);

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_RETURN_ON_ERROR(nvs_flash_erase(), TAG, "nvs erase");
        err = nvs_flash_init();
    }
    ESP_RETURN_ON_ERROR(err, TAG, "nvs");
    ESP_RETURN_ON_ERROR(load_token(), TAG, "token");
    printf("update token: %s\n", token);

    err = start_wifi();
    if (err != ESP_OK) {
        // An app on trial is not marked valid here, so the next restart goes
        // back to the old app.
        printf("no Wi-Fi (%s), so no updates\n", esp_err_to_name(err));
        return err;
    }
    ESP_RETURN_ON_ERROR(start_http(), TAG, "http");
    start_mdns();
    printf("update with: curl -H 'X-Token: <token>' --data-binary @build/<app>.bin "
           "http://" HOSTNAME ".local/update\n");

    if (running_state() == ESP_OTA_IMG_PENDING_VERIFY) {
        xTaskCreate(trial_task, "ota_trial", 3072, NULL, 5, NULL);
    }
    ESP_LOGI(TAG, "ready");
    return ESP_OK;
}
