// 05-ota: update StackChan over Wi-Fi, with rollback.
//
// The robot joins the home Wi-Fi. Two networks can be set at build time (see
// README); at boot it scans and joins the one with the strongest signal. It is
// then at http://stackchan.local/ (mDNS) and runs a small web server:
//
//   GET  /        which firmware runs, and from which slot
//   POST /update  the app image (build/ota.bin, not the merged image), with the
//                 header "X-Token: <token>". The robot writes it to the other
//                 slot and restarts.
//
// Rollback: a new app boots once "on trial". It marks itself valid only when
// Wi-Fi and the web server work, so it can always take the next update. If it
// crashes, or gets no Wi-Fi within TRIAL_WIFI_S, the bootloader goes back to
// the old app.
#include <stdio.h>
#include <string.h>
#include "esp_app_desc.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_ota_ops.h"
#include "esp_random.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "mdns.h"
#include "nvs.h"
#include "nvs_flash.h"

static const char *TAG = "ota";

#define HOSTNAME     "stackchan"
#define TRIAL_WIFI_S 60 // an app on trial with no Wi-Fi after this goes back to the old app
#define RETRY_MS     5000

typedef struct {
    const char *ssid;
    const char *password;
} network_t;

// Empty SSIDs (no wifi.env at build time) fall back to esp-emu's built-in
// network, so the example also runs in the emulator.
static const network_t home[] = {
    {CONFIG_TR_WIFI_SSID_01, CONFIG_TR_WIFI_PASSWORD_01},
    {CONFIG_TR_WIFI_SSID_02, CONFIG_TR_WIFI_PASSWORD_02},
};
static const network_t emulator = {"myssid", "mypassword"};

#define GOT_IP BIT0
#define LOST   BIT1
static EventGroupHandle_t wifi_events;
static char token[17];

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
static void load_token(void)
{
    nvs_handle_t nvs;
    ESP_ERROR_CHECK(nvs_open("ota", NVS_READWRITE, &nvs));
    size_t len = sizeof(token);
    if (nvs_get_str(nvs, "token", token, &len) != ESP_OK) {
        const char *chars = "abcdefghijkmnpqrstuvwxyz23456789";
        for (int i = 0; i < 16; i++) {
            token[i] = chars[esp_random() % strlen(chars)];
        }
        token[16] = 0;
        ESP_ERROR_CHECK(nvs_set_str(nvs, "token", token));
        ESP_ERROR_CHECK(nvs_commit(nvs));
    }
    nvs_close(nvs);
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

static esp_err_t start_wifi(void)
{
    wifi_events = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *sta = esp_netif_create_default_wifi_sta();
    esp_netif_set_hostname(sta, HOSTNAME);
    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    esp_err_t err = esp_wifi_init(&init);
    if (err != ESP_OK) {
        return err;
    }
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

// GET /: which firmware runs.
static esp_err_t info_handler(httpd_req_t *req)
{
    char text[160];
    describe_app(text, sizeof(text));
    return httpd_resp_sendstr(req, text);
}

static esp_err_t start_http(void)
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.recv_wait_timeout = 10;
    httpd_handle_t server;
    esp_err_t err = httpd_start(&server, &cfg);
    if (err != ESP_OK) {
        return err;
    }
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

void app_main(void)
{
    char text[160];
    describe_app(text, sizeof(text));
    printf("\n%s", text);

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    load_token();
    printf("update token: %s\n", token);

    err = start_wifi();
    if (err != ESP_OK) {
        // An app on trial is not marked valid here, so the next restart goes
        // back to the old app.
        printf("no Wi-Fi (%s), so no updates\n", esp_err_to_name(err));
        return;
    }

    const bool on_trial = running_state() == ESP_OTA_IMG_PENDING_VERIFY;
    // On trial we wait a limited time. Otherwise the Wi-Fi task keeps trying.
    const TickType_t wait = on_trial ? pdMS_TO_TICKS(TRIAL_WIFI_S * 1000) : portMAX_DELAY;
    if (!(xEventGroupWaitBits(wifi_events, GOT_IP, pdFALSE, pdFALSE, wait) & GOT_IP)) {
        printf("no Wi-Fi within %d s, going back to the old app\n", TRIAL_WIFI_S);
        esp_ota_mark_app_invalid_rollback_and_reboot();
    }
    ESP_ERROR_CHECK(start_http());
    start_mdns();
    printf("update with: curl -H 'X-Token: <token>' --data-binary @build/ota.bin "
           "http://" HOSTNAME ".local/update\n");

    // Wi-Fi and the web server work, so this app can take the next update.
    if (on_trial) {
        ESP_ERROR_CHECK(esp_ota_mark_app_valid_cancel_rollback());
        printf("new app marked valid\n");
    }
    ESP_LOGI(TAG, "ready");
}
