// plugs: see plugs.h.
//
// The devices come from CONFIG_HOME_DEVICES, set at build time from
// TUYA_DEVICES in the env file (never in git), as in 13-home. One device per
// ";", fields by ",":
//
//   name,version,ip,id,key-hex,switch,switch,...
//   switch = dp:label:long_name[:ap]
//
// The IP is only where to look first. Each device says where it is in a UDP
// broadcast every few seconds, so the find task follows it when the router
// gives it a new address.
//
// ":ap" marks the outlet that feeds the Wi-Fi access point. Off for that
// outlet is refused while the robot is on CONFIG_HOME_AP_SSID: the robot
// would lose its network and nobody could switch it on again from Home.
//
// The worker task does all the switching and reading, so nobody else waits on
// the network. It reads every device again each POLL_MS, because someone may
// press the button on a plug or use the phone app.
#include "plugs.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "sdkconfig.h"
#include "tr_tuya.h"

static const char *TAG = "plugs";

#define POLL_MS  15000
#define RETRY_MS 3000

typedef struct {
    int dp;
    char name[24];
    bool ap;
    int on; // 1, 0, or -1 while not known
} sw_t;

typedef struct {
    char name[16];
    char id[24];
    char ip[16];
    tr_tuya_dev_t dev;
    sw_t sw[PLUGS_MAX_SW];
    int nsw;
    int watts10; // dp 19, in tenths of a watt (-1: not known)
    int err;     // the last tr_tuya result, 0 when fine
} plug_t;

typedef enum { JOB_READ, JOB_SET, JOB_MOVED } job_kind_t;

typedef struct {
    job_kind_t kind;
    int dev;
    int sw;
    bool on;
    char ip[16];
} job_t;

static plug_t devs[PLUGS_MAX_DEV];
static int ndev;
static SemaphoreHandle_t mu; // devs[] between the worker and the readers
static QueueHandle_t jobs;
static void (*on_fresh)(int dev);
static volatile unsigned reads;

// --- Devices -----------------------------------------------------------------

static void copy(char *out, size_t size, const char *in)
{
    snprintf(out, size, "%s", in ? in : "");
}

static bool parse_switch(plug_t *d, char *spec)
{
    char *save = NULL;
    const char *dp = strtok_r(spec, ":", &save), *label = strtok_r(NULL, ":", &save);
    const char *name = strtok_r(NULL, ":", &save), *flag = strtok_r(NULL, ":", &save);
    if (!dp || !label || !name || d->nsw == PLUGS_MAX_SW) {
        return false;
    }
    sw_t *s = &d->sw[d->nsw++];
    s->dp = atoi(dp);
    copy(s->name, sizeof(s->name), name);
    for (char *p = s->name; *p; p++) {
        *p = *p == '_' ? ' ' : *p;
    }
    s->ap = flag && strcmp(flag, "ap") == 0;
    s->on = -1;
    return s->dp > 0;
}

static void parse_devices(void)
{
    static char text[sizeof(CONFIG_HOME_DEVICES)];
    copy(text, sizeof(text), CONFIG_HOME_DEVICES);
    char *save_dev = NULL;
    for (char *entry = strtok_r(text, ";", &save_dev); entry && ndev < PLUGS_MAX_DEV;
         entry = strtok_r(NULL, ";", &save_dev)) {
        plug_t *d = &devs[ndev];
        memset(d, 0, sizeof(*d));
        char *save = NULL;
        const char *name = strtok_r(entry, ",", &save), *version = strtok_r(NULL, ",", &save);
        const char *ip = strtok_r(NULL, ",", &save), *id = strtok_r(NULL, ",", &save);
        const char *key = strtok_r(NULL, ",", &save);
        if (!name || !version || !ip || !id || !tr_tuya_key_from_hex(key, d->dev.key)) {
            ESP_LOGW(TAG, "skipped a device: bad entry in TUYA_DEVICES");
            continue;
        }
        copy(d->name, sizeof(d->name), name);
        copy(d->ip, sizeof(d->ip), ip);
        copy(d->id, sizeof(d->id), id);
        d->dev.id = d->id;
        d->dev.ip = d->ip;
        d->dev.version = atoi(version);
        d->watts10 = -1;
        d->err = TR_TUYA_ERR_CONNECT; // not read yet
        for (char *spec = strtok_r(NULL, ",", &save); spec; spec = strtok_r(NULL, ",", &save)) {
            if (!parse_switch(d, spec)) {
                ESP_LOGW(TAG, "%s: skipped a bad switch", d->name);
            }
        }
        ndev++;
    }
}

// The robot is on the network that the ":ap" outlet feeds.
static bool on_guarded_wifi(void)
{
    wifi_ap_record_t ap;
    return CONFIG_HOME_AP_SSID[0] && esp_wifi_sta_get_ap_info(&ap) == ESP_OK &&
           strcmp((const char *)ap.ssid, CONFIG_HOME_AP_SSID) == 0;
}

// Read one device (tr_tuya_status) and keep what it says.
static void refresh(int i)
{
    tr_tuya_dp_t dps[24];
    const int n = tr_tuya_status(&devs[i].dev, dps, 24);
    xSemaphoreTake(mu, portMAX_DELAY);
    plug_t *d = &devs[i];
    const int was = d->err;
    d->err = n < 0 ? n : 0;
    for (int k = 0; k < d->nsw; k++) {
        d->sw[k].on = n < 0 ? -1 : tr_tuya_value(dps, n, d->sw[k].dp);
    }
    d->watts10 = n < 0 ? -1 : tr_tuya_value(dps, n, 19);
    xSemaphoreGive(mu);
    if (n < 0 && d->err != was) { // say it once, not on every read
        ESP_LOGW(TAG, "%s at %s: %s", d->name, d->ip, tr_tuya_err_text(n));
    } else if (n >= 0 && was) {
        ESP_LOGI(TAG, "%s at %s answers", d->name, d->ip);
    }
    reads = reads + 1;
    if (on_fresh) {
        on_fresh(i);
    }
}

// True once the robot has an address on the home network.
static bool have_ip(void)
{
    esp_netif_ip_info_t ip;
    esp_netif_t *sta = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    return sta && esp_netif_get_ip_info(sta, &ip) == ESP_OK && ip.ip.addr != 0;
}

static void worker_task(void *arg)
{
    // The first read waits for Wi-Fi: before it, no device answers.
    while (!have_ip()) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    for (int i = 0; i < ndev; i++) {
        refresh(i);
    }
    static char text[512];
    plugs_text(text, sizeof(text));
    ESP_LOGI(TAG, "first read:\n%s", text);
    job_t job;
    for (;;) {
        // A device that did not answer gets its next try sooner: a power
        // strip that takes one connection at a time is often just busy.
        bool all_fine = true;
        for (int i = 0; i < ndev; i++) {
            all_fine = all_fine && devs[i].err == 0;
        }
        if (xQueueReceive(jobs, &job, pdMS_TO_TICKS(all_fine ? POLL_MS : RETRY_MS)) != pdTRUE) {
            for (int i = 0; i < ndev; i++) {
                refresh(i);
            }
            continue;
        }
        plug_t *d = &devs[job.dev];
        if (job.kind == JOB_SET) {
            const int r = tr_tuya_set(&d->dev, d->sw[job.sw].dp, job.on);
            ESP_LOGI(TAG, "%s %s %s: %s", d->name, d->sw[job.sw].name, job.on ? "on" : "off",
                     tr_tuya_err_text(r));
        } else if (job.kind == JOB_MOVED) {
            ESP_LOGI(TAG, "%s moved from %s to %s", d->name, d->ip, job.ip);
            xSemaphoreTake(mu, portMAX_DELAY);
            copy(d->ip, sizeof(d->ip), job.ip);
            xSemaphoreGive(mu);
        }
        refresh(job.dev);
    }
}

// Listen to the devices' broadcasts. When one of ours shows a new address,
// tell the worker: only the worker changes devs[].
static void find_task(void *arg)
{
    static uint8_t in[TR_TUYA_UDP_MAX];
    for (;;) {
        const int s = socket(AF_INET, SOCK_DGRAM, 0);
        const struct sockaddr_in any = {.sin_family = AF_INET, .sin_port = htons(TR_TUYA_UDP_PORT)};
        if (s < 0 || bind(s, (const struct sockaddr *)&any, sizeof(any)) != 0) {
            ESP_LOGW(TAG, "no UDP socket for the broadcasts, again in 10 s");
            if (s >= 0) {
                close(s);
            }
            vTaskDelay(pdMS_TO_TICKS(10000));
            continue;
        }
        for (;;) {
            const int n = recv(s, in, sizeof(in), 0);
            if (n < 0) {
                break; // the network went away: open the socket again
            }
            job_t job = {.kind = JOB_MOVED};
            char id[24];
            if (tr_tuya_broadcast(in, (size_t)n, id, sizeof(id), job.ip, sizeof(job.ip)) != 0) {
                continue;
            }
            xSemaphoreTake(mu, portMAX_DELAY);
            job.dev = -1;
            for (int i = 0; i < ndev; i++) {
                if (strcmp(devs[i].id, id) == 0 && strcmp(devs[i].ip, job.ip) != 0) {
                    job.dev = i;
                }
            }
            xSemaphoreGive(mu);
            if (job.dev >= 0) {
                xQueueSend(jobs, &job, 0);
                // The same broadcast comes again in a few seconds: give the
                // worker time to take this one.
                vTaskDelay(pdMS_TO_TICKS(2000));
            }
        }
        close(s);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

esp_err_t plugs_start(void (*fresh)(int dev))
{
    parse_devices();
    ESP_LOGI(TAG, "%d devices", ndev);
    if (ndev == 0) {
        return ESP_ERR_NOT_FOUND;
    }
    on_fresh = fresh;
    mu = xSemaphoreCreateMutex();
    jobs = xQueueCreate(8, sizeof(job_t));
    if (!mu || !jobs || xTaskCreate(worker_task, "plugs", 6144, NULL, 3, NULL) != pdPASS ||
        xTaskCreate(find_task, "plugs_find", 4096, NULL, 2, NULL) != pdPASS) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

// --- For the bridge, the screen and the web --------------------------------------

int plugs_count(void)
{
    return ndev;
}

int plugs_switches(int dev)
{
    return devs[dev].nsw;
}

const char *plugs_dev_name(int dev)
{
    return devs[dev].name;
}

const char *plugs_dev_id(int dev)
{
    return devs[dev].id;
}

const char *plugs_sw_name(int dev, int sw)
{
    return devs[dev].sw[sw].name;
}

int plugs_sw_dp(int dev, int sw)
{
    return devs[dev].sw[sw].dp;
}

int plugs_state(int dev, int sw)
{
    xSemaphoreTake(mu, portMAX_DELAY);
    const int on = devs[dev].err ? -1 : devs[dev].sw[sw].on;
    xSemaphoreGive(mu);
    return on;
}

bool plugs_set(int dev, int sw, bool on)
{
    if (!on && devs[dev].sw[sw].ap && on_guarded_wifi()) {
        ESP_LOGW(TAG, "%s %s stays on: it feeds the robot's own Wi-Fi", devs[dev].name,
                 devs[dev].sw[sw].name);
        return false;
    }
    const job_t job = {.kind = JOB_SET, .dev = dev, .sw = sw, .on = on};
    return xQueueSend(jobs, &job, 0) == pdTRUE;
}

unsigned plugs_reads(void)
{
    return reads;
}

void plugs_info(int dev, char *out, size_t size)
{
    xSemaphoreTake(mu, portMAX_DELAY);
    const plug_t *d = &devs[dev];
    if (d->err) {
        snprintf(out, size, "%s", tr_tuya_err_text(d->err));
    } else if (d->watts10 < 0) {
        snprintf(out, size, "%s", d->ip);
    } else {
        snprintf(out, size, "%s  %d.%d W", d->ip, d->watts10 / 10, d->watts10 % 10);
    }
    xSemaphoreGive(mu);
}

void plugs_text(char *out, size_t size)
{
    size_t n = 0;
    out[0] = 0;
    xSemaphoreTake(mu, portMAX_DELAY);
    for (int i = 0; i < ndev && n < size; i++) {
        const plug_t *d = &devs[i];
        n += snprintf(out + n, size - n, "%s (%s):", d->name, d->ip);
        if (d->err) {
            n += snprintf(out + n, size - n, " %s\n", tr_tuya_err_text(d->err));
            continue;
        }
        for (int k = 0; k < d->nsw && n < size; k++) {
            n += snprintf(out + n, size - n, " %s %s,", d->sw[k].name,
                          d->sw[k].on == 1   ? "on"
                          : d->sw[k].on == 0 ? "off"
                                             : "?");
        }
        if (n < size && d->watts10 >= 0) {
            n += snprintf(out + n, size - n, " %d.%d W", d->watts10 / 10, d->watts10 % 10);
        }
        if (n < size) {
            n += snprintf(out + n, size - n, "\n");
        }
    }
    xSemaphoreGive(mu);
}
