// Wireless link for 06-balance.
//
//   Wi-Fi   At boot the robot scans for the home networks set at build time
//           (WIFI_ENV, see README.md) and joins the strongest one. If none is
//           in range, or it can't join, it makes its own network instead:
//           "BALA2-xxxx" (end of the MAC), WPA2, robot at 192.168.4.1.
//   Key     Made at random on the first boot, saved in NVS, and shown on the
//           screen and the serial port. Never in the source code. It is the
//           password of the robot's own network, and the update token.
//   mDNS    The robot is bala2.local on both kinds of network.
//   UDP     Port 4210. Send any line to the robot (a command like "kp 30", or
//           "hb" as a heartbeat), and it sends its telemetry back to you.
//   OTA     POST the app image (build/balance.bin, not the merged image) to
//           http://bala2.local/update, with the header "X-Token: <key>".
//           Refused while the robot balances.
//
// Safety: once a computer has talked to the robot, link_alive() turns false
// if it is quiet for more than 1 second, and the control loop stops.
#include "link.h"

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_app_desc.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_ota_ops.h"
#include "esp_random.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "lwip/sockets.h"
#include "mdns.h"
#include "nvs.h"
#include "nvs_flash.h"

static const char *TAG = "link";

#define QUIET_US 1000000 // 1 s without a packet = link lost

static link_callbacks_t cbs;
#define HOSTNAME "bala2"

typedef struct {
    const char *ssid;
    const char *password;
} home_net_t;

static const home_net_t home_nets[] = {
    {CONFIG_BALA2_WIFI_SSID_01, CONFIG_BALA2_WIFI_PASSWORD_01},
    {CONFIG_BALA2_WIFI_SSID_02, CONFIG_BALA2_WIFI_PASSWORD_02},
};

static char ssid[33];
static char password[16];
static char ip[16] = LINK_IP;
static volatile bool on_home; // joined a home network (station)
static volatile bool got_ip;
static int sock = -1;
static struct sockaddr_in peer;
static volatile bool have_peer;
static volatile int64_t last_rx_us;
static volatile bool updating;

const char *link_ssid(void)
{
    return ssid;
}
const char *link_password(void)
{
    return password;
}
const char *link_ip(void)
{
    return ip;
}
bool link_on_home(void)
{
    return on_home;
}

bool link_updating(void)
{
    return updating;
}

bool link_alive(void)
{
    return !have_peer || esp_timer_get_time() - last_rx_us < QUIET_US;
}

void link_send(const char *line)
{
    if (sock >= 0 && have_peer) {
        sendto(sock, line, strlen(line), 0, (struct sockaddr *)&peer, sizeof(peer));
    }
}

// Read the password from NVS, or make a new one on the first boot.
static void load_password(void)
{
    nvs_handle_t nvs;
    ESP_ERROR_CHECK(nvs_open("link", NVS_READWRITE, &nvs));
    size_t len = sizeof(password);
    if (nvs_get_str(nvs, "ap_pass", password, &len) != ESP_OK) {
        // No 0/o, 1/l: easy to read from the screen.
        const char *chars = "abcdefghijkmnpqrstuvwxyz23456789";
        for (int i = 0; i < 12; i++) {
            password[i] = chars[esp_random() % strlen(chars)];
        }
        password[12] = 0;
        ESP_ERROR_CHECK(nvs_set_str(nvs, "ap_pass", password));
        ESP_ERROR_CHECK(nvs_commit(nvs));
    }
    nvs_close(nvs);
}

static void on_wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED && on_home) {
        // The home link dropped: join again. If the robot was balancing, its
        // heartbeat stops and the control loop stops it.
        got_ip = false;
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        const ip_event_got_ip_t *ev = (const ip_event_got_ip_t *)data;
        snprintf(ip, sizeof(ip), IPSTR, IP2STR(&ev->ip_info.ip));
        got_ip = true;
    }
}

// The strongest home network in range, or NULL.
static const home_net_t *strongest_home(int *rssi)
{
    if (esp_wifi_scan_start(NULL, true) != ESP_OK) {
        return NULL;
    }
    static wifi_ap_record_t aps[20];
    uint16_t found = sizeof(aps) / sizeof(aps[0]);
    esp_wifi_scan_get_ap_records(&found, aps);
    const home_net_t *best = NULL;
    for (int i = 0; i < found; i++) {
        for (const home_net_t &net : home_nets) {
            if (net.ssid[0] && !strcmp(net.ssid, (const char *)aps[i].ssid) &&
                (!best || aps[i].rssi > *rssi)) {
                best = &net;
                *rssi = aps[i].rssi;
            }
        }
    }
    return best;
}

// Try the home networks. True once the robot has an address there.
static bool join_home(void)
{
    int rssi = 0;
    const home_net_t *net = strongest_home(&rssi);
    if (!net) {
        ESP_LOGI(TAG, "no home network in range");
        return false;
    }
    ESP_LOGI(TAG, "joining %s (%d dBm)", net->ssid, rssi);
    wifi_config_t cfg = {};
    strlcpy((char *)cfg.sta.ssid, net->ssid, sizeof(cfg.sta.ssid));
    strlcpy((char *)cfg.sta.password, net->password, sizeof(cfg.sta.password));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &cfg));
    on_home = true;
    esp_wifi_connect();
    for (int i = 0; i < 100 && !got_ip; i++) { // 10 s
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    if (!got_ip) {
        ESP_LOGW(TAG, "could not join %s", net->ssid);
        on_home = false;
        esp_wifi_disconnect();
        return false;
    }
    strlcpy(ssid, net->ssid, sizeof(ssid));
    return true;
}

// The robot's own network, at 192.168.4.1.
static void start_ap(void)
{
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
    snprintf(ssid, sizeof(ssid), "BALA2-%02X%02X", mac[4], mac[5]);
    wifi_config_t cfg = {};
    strcpy((char *)cfg.ap.ssid, ssid);
    cfg.ap.ssid_len = strlen(ssid);
    strcpy((char *)cfg.ap.password, password);
    cfg.ap.channel = 6;
    cfg.ap.max_connection = 2;
    cfg.ap.authmode = WIFI_AUTH_WPA2_PSK;
    ESP_ERROR_CHECK(esp_wifi_stop());
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &cfg));
    ESP_ERROR_CHECK(esp_wifi_start());
    strlcpy(ip, LINK_IP, sizeof(ip));
}

static void start_wifi(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    esp_netif_create_default_wifi_ap();
    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, on_wifi_event, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, on_wifi_event, NULL));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    // No modem sleep: with it, the robot answers late and drops packets, and
    // a quiet second stops the balance.
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    if (!join_home()) {
        start_ap();
    }
    ESP_LOGI(TAG, "Wi-Fi %s %s, robot at %s (%s.local)", on_home ? "home network" : "own network",
             ssid, ip, HOSTNAME);

    if (mdns_init() == ESP_OK) {
        mdns_hostname_set(HOSTNAME);
        mdns_instance_name_set("BALA2 Fire");
        mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);
    }
}

static void udp_task(void *arg)
{
    sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    struct sockaddr_in addr = {};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(LINK_UDP_PORT);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    bind(sock, (struct sockaddr *)&addr, sizeof(addr));

    char buf[64];
    for (;;) {
        struct sockaddr_in from;
        socklen_t from_len = sizeof(from);
        const int n = recvfrom(sock, buf, sizeof(buf) - 1, 0, (struct sockaddr *)&from, &from_len);
        if (n <= 0) {
            continue;
        }
        buf[n] = 0;
        peer = from; // answer whoever talked last
        last_rx_us = esp_timer_get_time();
        have_peer = true;
        // A packet can hold several lines.
        char *save = NULL;
        for (char *line = strtok_r(buf, "\r\n", &save); line;
             line = strtok_r(NULL, "\r\n", &save)) {
            cbs.on_command(line);
        }
    }
}

// POST /update: write the new app to the other OTA slot, then restart.
static esp_err_t update_handler(httpd_req_t *req)
{
    char token[sizeof(password)] = "";
    if (httpd_req_get_hdr_value_str(req, "X-Token", token, sizeof(token)) != ESP_OK ||
        strcmp(token, password) != 0) {
        httpd_resp_set_status(req, "401 Unauthorized");
        httpd_resp_sendstr(req, "wrong or missing X-Token\n");
        return ESP_FAIL;
    }
    // Never stop a balancing robot for an update: it would fall over.
    if (!cbs.can_update()) {
        httpd_resp_set_status(req, "409 Conflict");
        httpd_resp_sendstr(req, "robot is balancing, stop it first\n");
        return ESP_FAIL;
    }
    updating = true; // from here on the robot does not start
    const esp_partition_t *slot = esp_ota_get_next_update_partition(NULL);
    esp_ota_handle_t ota;
    // Erase each flash sector just before it is written. Erasing all 3 MB at
    // once stalls the control loop for over 1 s, and the watchdog resets.
    if (esp_ota_begin(slot, OTA_WITH_SEQUENTIAL_WRITES, &ota) != ESP_OK) {
        updating = false;
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    printf("update: receiving %d bytes into %s\n", req->content_len, slot->label);
    // Erasing and writing the flash keeps core 0 busy for the whole update
    // (about 15 s), so its idle task does not run for over 1 s and the task
    // watchdog resets the robot (seen twice on 2026-10-31: "IDLE0", with the
    // backtrace in esp_ota_write). Short pauses did not help. The motors are
    // stopped, so we stop watching core 0's idle task until the update ends.
    // The control task stays watched, and a stalled upload ends after 3
    // receive timeouts in a row (30 s).
    TaskHandle_t idle0 = xTaskGetIdleTaskHandleForCore(0);
    esp_task_wdt_delete(idle0);
    static char buf[4096];
    int left = req->content_len, timeouts = 0;
    while (left > 0) {
        const int n = httpd_req_recv(req, buf, left < (int)sizeof(buf) ? left : sizeof(buf));
        if (n == HTTPD_SOCK_ERR_TIMEOUT && ++timeouts < 3) {
            continue;
        }
        if (n > 0) {
            timeouts = 0;
        }
        if (n <= 0 || esp_ota_write(ota, buf, n) != ESP_OK) {
            esp_ota_abort(ota);
            esp_task_wdt_add(idle0);
            updating = false;
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }
        left -= n;
    }
    // esp_ota_end() checks the image before we boot from it.
    if (esp_ota_end(ota) != ESP_OK || esp_ota_set_boot_partition(slot) != ESP_OK) {
        esp_task_wdt_add(idle0);
        updating = false;
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad image");
        return ESP_FAIL;
    }
    httpd_resp_sendstr(req, "ok, restarting\n");
    printf("update: done, restarting\n");
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
    return ESP_OK;
}

// GET /: which firmware is running.
static esp_err_t info_handler(httpd_req_t *req)
{
    const esp_app_desc_t *app = esp_app_get_description();
    char text[128];
    snprintf(text, sizeof(text), "%s, built %s %s, running from %s\n", app->project_name, app->date,
             app->time, esp_ota_get_running_partition()->label);
    return httpd_resp_sendstr(req, text);
}

static void start_http(void)
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.core_id = 0; // keep it away from the control loop on core 1
    cfg.recv_wait_timeout = 10;
    httpd_handle_t server;
    ESP_ERROR_CHECK(httpd_start(&server, &cfg));
    httpd_uri_t update = {};
    update.uri = "/update";
    update.method = HTTP_POST;
    update.handler = update_handler;
    httpd_register_uri_handler(server, &update);
    httpd_uri_t info = {};
    info.uri = "/";
    info.method = HTTP_GET;
    info.handler = info_handler;
    httpd_register_uri_handler(server, &info);
}

void link_start(const link_callbacks_t *cb)
{
    cbs = *cb;
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    load_password();
    start_wifi();
    start_http();
    xTaskCreatePinnedToCore(udp_task, "udp", 4096, NULL, 5, NULL, 0);
}
