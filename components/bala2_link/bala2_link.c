// bala2_link: Wi-Fi, updates over Wi-Fi and a small web server for BALA2 Fire.
// See include/bala2_link.h and README.md.
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
//   OTA     POST the app image (build/<app>.bin, not the merged image) to
//           http://bala2.local/update, with the header "X-Token: <key>".
//
// Everything here runs on core 0. A control loop on core 1 only calls
// link_alive() and link_updating(), which read one word each.
#include "bala2_link.h"

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_app_desc.h"
#include "esp_check.h"
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

#define HOSTNAME     "bala2"
#define QUIET_MS     1000  // without a UDP packet for this long = link lost
#define JOIN_MS      10000 // time to get an address on a home network
#define TRIAL_STARTS 3     // a new app that fails this many starts goes back
#define TRIAL_OK_MS  15000 // link up for this long: the new app stays
#define NVS_SPACE    "link"

typedef struct {
    const char *ssid;
    const char *password;
} home_net_t;

static const home_net_t home_nets[] = {
    {CONFIG_BALA2_WIFI_SSID_01, CONFIG_BALA2_WIFI_PASSWORD_01},
    {CONFIG_BALA2_WIFI_SSID_02, CONFIG_BALA2_WIFI_PASSWORD_02},
};

static link_callbacks_t cbs;
static char ssid[33];
static char password[16]; // 12 characters; the rest stays 0
static char ip[16] = LINK_IP;
static volatile bool on_home; // joined a home network (station)
static volatile bool got_ip;
static volatile bool updating;
static volatile bool on_trial; // a new app that has not shown a working link yet
static int sock = -1;
// The UDP peer. The lock keeps a reader from seeing half of a new address.
static portMUX_TYPE peer_lock = portMUX_INITIALIZER_UNLOCKED;
static struct sockaddr_in peer;
static volatile bool have_peer;
// Time of the last UDP packet, in ms. 32 bits on purpose: the control loop on
// the other core reads it, and one word cannot be seen half written.
static volatile uint32_t last_rx_ms;

static uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

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

int link_rssi(void)
{
    wifi_ap_record_t ap;
    return on_home && esp_wifi_sta_get_ap_info(&ap) == ESP_OK ? ap.rssi : 0;
}

const char *link_last_restart(void)
{
    switch (esp_reset_reason()) {
        case ESP_RST_POWERON:
            return "power on";
        case ESP_RST_BROWNOUT:
            return "brownout";
        case ESP_RST_PANIC:
            return "crash";
        case ESP_RST_INT_WDT:
        case ESP_RST_TASK_WDT:
        case ESP_RST_WDT:
            return "watchdog";
        case ESP_RST_SW:
            return "software";
        default:
            return "other";
    }
}

bool link_updating(void)
{
    return updating;
}

bool link_alive(void)
{
    return !have_peer || now_ms() - last_rx_ms < QUIET_MS;
}

void link_send(const char *line)
{
    if (sock < 0 || !have_peer) {
        return;
    }
    struct sockaddr_in to;
    portENTER_CRITICAL(&peer_lock);
    to = peer;
    portEXIT_CRITICAL(&peer_lock);
    sendto(sock, line, strlen(line), 0, (struct sockaddr *)&to, sizeof(to));
}

static esp_err_t start_nvs(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_RETURN_ON_ERROR(nvs_flash_erase(), TAG, "nvs erase");
        err = nvs_flash_init();
    }
    return err;
}

// ---- A new app is on trial ----
//
// The Fire's bootloader has no rollback, and a new bootloader can only go in
// over USB. So the app does it: the update writes "trial" = 1 before it
// restarts, each start of the new app adds 1, and a link that stays up for
// TRIAL_OK_MS sets it back to 0. A new app that reaches TRIAL_STARTS first
// (it crashes, hangs or has no link) sets the old app to boot and restarts.
// It cannot help with an app that crashes before link_guard() runs.

static void set_trial(uint8_t starts)
{
    nvs_handle_t nvs;
    if (nvs_open(NVS_SPACE, NVS_READWRITE, &nvs) == ESP_OK) {
        if (nvs_set_u8(nvs, "trial", starts) == ESP_OK) {
            nvs_commit(nvs);
        }
        nvs_close(nvs);
    }
}

void link_guard(void)
{
    nvs_handle_t nvs;
    uint8_t starts = 0;
    if (start_nvs() != ESP_OK || nvs_open(NVS_SPACE, NVS_READONLY, &nvs) != ESP_OK) {
        return; // no storage yet: a first boot, not a trial
    }
    nvs_get_u8(nvs, "trial", &starts);
    nvs_close(nvs);
    if (starts == 0) {
        return;
    }
    if (starts < TRIAL_STARTS) {
        on_trial = true;
        set_trial(starts + 1);
        ESP_LOGW(TAG, "new app on trial, start %d of %d", starts, TRIAL_STARTS);
        return;
    }
    // With two app slots, the "next update" slot is the one with the old app.
    // esp_ota_set_boot_partition() checks that image before it accepts it.
    set_trial(0);
    const esp_partition_t *old = esp_ota_get_next_update_partition(NULL);
    if (old && esp_ota_set_boot_partition(old) == ESP_OK) {
        ESP_LOGE(TAG, "new app failed %d starts, back to the app in %s", TRIAL_STARTS, old->label);
        esp_restart();
    }
    ESP_LOGE(TAG, "new app failed %d starts, and there is no old app to go back to", TRIAL_STARTS);
}

// Read the key from NVS, or make a new one on the first boot.
static esp_err_t load_password(void)
{
    nvs_handle_t nvs;
    ESP_RETURN_ON_ERROR(nvs_open(NVS_SPACE, NVS_READWRITE, &nvs), TAG, "nvs open");
    size_t len = sizeof(password);
    esp_err_t err = ESP_OK;
    if (nvs_get_str(nvs, "ap_pass", password, &len) != ESP_OK) {
        // No 0/o, 1/l: easy to read from the screen.
        static const char chars[] = "abcdefghijkmnpqrstuvwxyz23456789";
        memset(password, 0, sizeof(password));
        for (int i = 0; i < 12; i++) {
            password[i] = chars[esp_random() % (sizeof(chars) - 1)];
        }
        err = nvs_set_str(nvs, "ap_pass", password);
        if (err == ESP_OK) {
            err = nvs_commit(nvs);
        }
    }
    nvs_close(nvs);
    return err;
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
    if (!home_nets[0].ssid[0] && !home_nets[1].ssid[0]) {
        return NULL; // none set at build time: no need to scan
    }
    if (esp_wifi_scan_start(NULL, true) != ESP_OK) {
        return NULL;
    }
    static wifi_ap_record_t aps[20]; // the scan gives the strongest ones first
    uint16_t found = sizeof(aps) / sizeof(aps[0]);
    if (esp_wifi_scan_get_ap_records(&found, aps) != ESP_OK) {
        return NULL;
    }
    const home_net_t *best = NULL;
    for (int i = 0; i < found; i++) {
        for (size_t k = 0; k < sizeof(home_nets) / sizeof(home_nets[0]); k++) {
            const home_net_t *net = &home_nets[k];
            if (net->ssid[0] && !strcmp(net->ssid, (const char *)aps[i].ssid) &&
                (!best || aps[i].rssi > *rssi)) {
                best = net;
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
    wifi_config_t cfg = {0};
    strlcpy((char *)cfg.sta.ssid, net->ssid, sizeof(cfg.sta.ssid));
    strlcpy((char *)cfg.sta.password, net->password, sizeof(cfg.sta.password));
    // With a password, WPA2 at least: never an open network with the same name.
    cfg.sta.threshold.authmode = net->password[0] ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;
    if (esp_wifi_set_config(WIFI_IF_STA, &cfg) != ESP_OK) {
        return false;
    }
    on_home = true;
    esp_wifi_connect();
    for (int i = 0; i < JOIN_MS / 100 && !got_ip; i++) {
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
static esp_err_t start_ap(void)
{
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
    snprintf(ssid, sizeof(ssid), "BALA2-%02X%02X", mac[4], mac[5]);
    wifi_config_t cfg = {0};
    strlcpy((char *)cfg.ap.ssid, ssid, sizeof(cfg.ap.ssid));
    cfg.ap.ssid_len = strlen(ssid);
    strlcpy((char *)cfg.ap.password, password, sizeof(cfg.ap.password));
    cfg.ap.channel = 6;
    cfg.ap.max_connection = 2;
    cfg.ap.authmode = WIFI_AUTH_WPA2_PSK;
    // The access point's network interface is only made when it is needed.
    ESP_RETURN_ON_FALSE(esp_netif_create_default_wifi_ap(), ESP_FAIL, TAG, "ap netif");
    ESP_RETURN_ON_ERROR(esp_wifi_stop(), TAG, "wifi stop");
    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_AP), TAG, "ap mode");
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_AP, &cfg), TAG, "ap config");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "ap start");
    strlcpy(ip, LINK_IP, sizeof(ip));
    return ESP_OK;
}

static esp_err_t start_wifi(void)
{
    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "netif");
    esp_err_t err = esp_event_loop_create_default();
    ESP_RETURN_ON_FALSE(err == ESP_OK || err == ESP_ERR_INVALID_STATE, err, TAG, "event loop");
    esp_netif_t *sta = esp_netif_create_default_wifi_sta();
    ESP_RETURN_ON_FALSE(sta, ESP_FAIL, TAG, "sta netif");
    esp_netif_set_hostname(sta, HOSTNAME);
    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&init), TAG, "wifi init");
    // The networks come from the build, so the driver need not save them: no
    // flash writes from Wi-Fi.
    ESP_RETURN_ON_ERROR(esp_wifi_set_storage(WIFI_STORAGE_RAM), TAG, "wifi storage");
    ESP_RETURN_ON_ERROR(
        esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, on_wifi_event, NULL), TAG,
        "wifi events");
    ESP_RETURN_ON_ERROR(
        esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, on_wifi_event, NULL), TAG,
        "ip events");
    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), TAG, "sta mode");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "wifi start");
    // No modem sleep: with it, the robot answers late and drops packets, and
    // a quiet second stops the balance.
    esp_wifi_set_ps(WIFI_PS_NONE);
    if (!join_home()) {
        ESP_RETURN_ON_ERROR(start_ap(), TAG, "own network");
    }
    ESP_LOGI(TAG, "Wi-Fi %s %s, robot at %s (%s.local)", on_home ? "home network" : "own network",
             ssid, ip, HOSTNAME);

    if (mdns_init() == ESP_OK) {
        mdns_hostname_set(HOSTNAME);
        mdns_instance_name_set("BALA2 Fire");
        mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);
    }
    return ESP_OK;
}

static void udp_task(void *arg)
{
    const int64_t up_since = esp_timer_get_time();
    char buf[64];
    for (;;) {
        // The link has been up for a while: the new app stays.
        if (on_trial && esp_timer_get_time() - up_since > TRIAL_OK_MS * 1000LL) {
            on_trial = false;
            set_trial(0);
            ESP_LOGI(TAG, "new app accepted");
        }
        if (sock < 0) {
            const int s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
            struct sockaddr_in addr = {0};
            addr.sin_family = AF_INET;
            addr.sin_port = htons(LINK_UDP_PORT);
            addr.sin_addr.s_addr = htonl(INADDR_ANY);
            // Wake once a second with no packet, for the trial check above.
            const struct timeval wait = {.tv_sec = 1};
            if (s < 0 || bind(s, (struct sockaddr *)&addr, sizeof(addr)) != 0 ||
                setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &wait, sizeof(wait)) != 0) {
                if (s >= 0) {
                    close(s);
                }
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }
            sock = s;
        }
        struct sockaddr_in from;
        socklen_t from_len = sizeof(from);
        const int n = recvfrom(sock, buf, sizeof(buf) - 1, 0, (struct sockaddr *)&from, &from_len);
        if (n <= 0) {
            if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
                vTaskDelay(pdMS_TO_TICKS(100)); // a socket error: do not spin
            }
            continue;
        }
        buf[n] = 0;
        portENTER_CRITICAL(&peer_lock);
        peer = from; // answer whoever talked last
        portEXIT_CRITICAL(&peer_lock);
        last_rx_ms = now_ms();
        have_peer = true;
        if (!cbs.on_command) {
            continue;
        }
        // A packet can hold several lines.
        char *save = NULL;
        for (char *line = strtok_r(buf, "\r\n", &save); line;
             line = strtok_r(NULL, "\r\n", &save)) {
            cbs.on_command(line);
        }
    }
}

// Send an error answer. If the request has a body we did not read (an update
// of 1 MB), return ESP_FAIL: the server then closes the connection at once.
// Left open, the server reads the body as new requests, and that kept core 0
// so busy that the task watchdog restarted the robot (seen on 2026-11-12).
static esp_err_t refuse(httpd_req_t *req, const char *status, const char *text)
{
    httpd_resp_set_status(req, status);
    httpd_resp_sendstr(req, text);
    return req->content_len > 0 ? ESP_FAIL : ESP_OK;
}

// The key check takes the same time for a right and a wrong key.
static bool token_ok(httpd_req_t *req)
{
    char token[sizeof(password)] = "";
    if (httpd_req_get_hdr_value_str(req, "X-Token", token, sizeof(token)) != ESP_OK) {
        return false;
    }
    uint8_t diff = 0;
    for (size_t i = 0; i < sizeof(password); i++) {
        diff |= token[i] ^ password[i];
    }
    return diff == 0 && password[0];
}

// POST /update: write the new app to the other OTA slot, then restart.
static esp_err_t update_handler(httpd_req_t *req)
{
    if (!token_ok(req)) {
        return refuse(req, "401 Unauthorized", "wrong or missing X-Token\n");
    }
    const esp_partition_t *slot = esp_ota_get_next_update_partition(NULL);
    if (!slot || req->content_len == 0 || req->content_len > slot->size) {
        return refuse(req, "400 Bad Request", "no app image, or it is too large\n");
    }
    // Say "updating" first, then ask: so the robot cannot start in between.
    updating = true;
    // Never stop a balancing robot for an update: it would fall over.
    if (cbs.can_update && !cbs.can_update()) {
        updating = false;
        return refuse(req, "409 Conflict", "robot is balancing, stop it first\n");
    }
    esp_ota_handle_t ota;
    // Erase each flash sector just before it is written. Erasing all 3 MB at
    // once stalls the control loop for over 1 s, and the watchdog resets.
    if (esp_ota_begin(slot, OTA_WITH_SEQUENTIAL_WRITES, &ota) != ESP_OK) {
        updating = false;
        return refuse(req, "500 Internal Server Error", "could not start the update\n");
    }
    printf("update: receiving %d bytes into %s\n", (int)req->content_len, slot->label);
    // Erasing and writing the flash keeps core 0 busy for the whole update
    // (about 15 s), so its idle task does not run for over 1 s and the task
    // watchdog resets the robot (seen twice on 2026-10-31: "IDLE0", with the
    // backtrace in esp_ota_write). Short pauses did not help. The motors are
    // stopped, so we stop watching core 0's idle task until the update ends.
    // A control task stays watched, and a stalled upload ends after 3 receive
    // timeouts in a row (30 s).
    TaskHandle_t idle0 = xTaskGetIdleTaskHandleForCore(0);
    const bool watched = esp_task_wdt_delete(idle0) == ESP_OK;
    static char buf[4096];
    int left = req->content_len, timeouts = 0;
    const char *problem = NULL;
    while (left > 0 && !problem) {
        const int n = httpd_req_recv(req, buf, left < (int)sizeof(buf) ? left : (int)sizeof(buf));
        if (n == HTTPD_SOCK_ERR_TIMEOUT && ++timeouts < 3) {
            continue;
        }
        if (n <= 0) {
            problem = "the upload stopped\n";
        } else if (esp_ota_write(ota, buf, n) != ESP_OK) {
            problem = "could not write the flash\n";
        } else {
            timeouts = 0;
            left -= n;
        }
    }
    if (problem) {
        esp_ota_abort(ota);
        // esp_ota_end() checks the image before we boot from it.
    } else if (esp_ota_end(ota) != ESP_OK || esp_ota_set_boot_partition(slot) != ESP_OK) {
        problem = "bad image\n";
    }
    if (watched) {
        esp_task_wdt_add(idle0);
    }
    if (problem) {
        updating = false;
        printf("update: failed, %s", problem);
        return refuse(req, "400 Bad Request", problem);
    }
    set_trial(1); // the new app must show a working link, or the robot comes back here
    httpd_resp_sendstr(req, "ok, restarting\n");
    printf("update: done, restarting\n");
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
    return ESP_OK;
}

// GET /info: which firmware is running.
static esp_err_t info_handler(httpd_req_t *req)
{
    const esp_app_desc_t *app = esp_app_get_description();
    char text[160];
    snprintf(text, sizeof(text), "%s, built %s %s, running from %s%s, last restart: %s\n",
             app->project_name, app->date, app->time, esp_ota_get_running_partition()->label,
             on_trial ? ", on trial" : "", link_last_restart());
    return httpd_resp_sendstr(req, text);
}

// GET /: the firmware's page, or the info line if it has none.
static esp_err_t page_handler(httpd_req_t *req)
{
    if (!cbs.page) {
        return info_handler(req);
    }
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    return httpd_resp_send(req, cbs.page, cbs.page_len);
}

// GET /favicon.ico: the robot icon of robotics.diegoromero.es (an SVG, 0.5
// KB), the same file StackChan's page serves.
static esp_err_t favicon_handler(httpd_req_t *req)
{
    extern const char icon_start[] asm("_binary_bala2_icon_svg_start");
    extern const char icon_end[] asm("_binary_bala2_icon_svg_end");
    httpd_resp_set_type(req, "image/svg+xml");
    httpd_resp_set_hdr(req, "Cache-Control", "max-age=86400");
    return httpd_resp_send(req, icon_start, icon_end - icon_start - 1); // no final NUL
}

// GET /state: the robot's state as JSON. A page asks for it twice a second.
static esp_err_t state_handler(httpd_req_t *req)
{
    static char json[1536]; // one server task, so one request at a time
    cbs.state_json(json, sizeof(json));
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_sendstr(req, json);
}

// POST /cmd: one command line, the same ones as over UDP. It needs the
// X-Bala2 header: a browser does not let a page from another site add it, so
// a web page you happen to visit cannot drive the robot.
static esp_err_t cmd_handler(httpd_req_t *req)
{
    char line[64];
    if (httpd_req_get_hdr_value_len(req, "X-Bala2") == 0 || req->content_len == 0 ||
        req->content_len >= sizeof(line)) {
        return refuse(req, "400 Bad Request", "bad command\n");
    }
    const int n = httpd_req_recv(req, line, req->content_len);
    if (n <= 0) {
        return ESP_FAIL; // the phone went away: close the connection
    }
    line[n] = 0;
    line[strcspn(line, "\r\n")] = 0;
    cbs.on_command(line);
    return httpd_resp_sendstr(req, "ok\n");
}

static esp_err_t start_http(void)
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.core_id = 0; // keep it away from the control loop on core 1
    cfg.recv_wait_timeout = 10;
    // A phone leaves connections open. When all are taken, drop the oldest
    // one, so the page and an update always get in.
    cfg.lru_purge_enable = true;
    httpd_handle_t server;
    ESP_RETURN_ON_ERROR(httpd_start(&server, &cfg), TAG, "http");
    const httpd_uri_t uris[] = {
        {.uri = "/update", .method = HTTP_POST, .handler = update_handler},
        {.uri = "/info", .method = HTTP_GET, .handler = info_handler},
        {.uri = "/", .method = HTTP_GET, .handler = page_handler},
        {.uri = "/favicon.ico", .method = HTTP_GET, .handler = favicon_handler},
        {.uri = "/state", .method = HTTP_GET, .handler = cbs.state_json ? state_handler : NULL},
        {.uri = "/cmd", .method = HTTP_POST, .handler = cbs.on_command ? cmd_handler : NULL},
    };
    for (size_t i = 0; i < sizeof(uris) / sizeof(uris[0]); i++) {
        if (uris[i].handler) {
            ESP_RETURN_ON_ERROR(httpd_register_uri_handler(server, &uris[i]), TAG, "%s",
                                uris[i].uri);
        }
    }
    return ESP_OK;
}

esp_err_t link_start(const link_callbacks_t *cb)
{
    if (cb) {
        cbs = *cb;
    }
    ESP_RETURN_ON_ERROR(start_nvs(), TAG, "nvs");
    ESP_RETURN_ON_ERROR(load_password(), TAG, "key");
    ESP_RETURN_ON_ERROR(start_wifi(), TAG, "wifi");
    ESP_RETURN_ON_ERROR(start_http(), TAG, "web server");
    ESP_RETURN_ON_FALSE(xTaskCreatePinnedToCore(udp_task, "udp", 4096, NULL, 5, NULL, 0) == pdPASS,
                        ESP_ERR_NO_MEM, TAG, "udp task");
    return ESP_OK;
}
