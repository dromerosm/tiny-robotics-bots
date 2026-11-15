// energy: see energy.h.
//
// The plugs give their power now (dp 19), not a running total that we can
// trust: the SP112's dp 17 is small and goes back to zero when Tuya's cloud
// collects it, and the strip has no dp 17 at all. So the bridge adds up the
// energy itself. After each read it adds the last power times the time since
// the read before it. A plug's power only changes when the load does, so the
// last reading is the best guess for the time in between.
//
// A read that fails adds nothing and does not move the start of the interval.
// When the device answers again, the whole gap counts at the old power, up to
// MAX_GAP_MS. A longer outage counts only that long: the device may have been
// unplugged.
//
// The totals go into one slot per hour (the last 48) and one per day (the last
// 62), by the local clock, plus a total since the first boot. Energy measured
// before the clock is set waits and goes into the first hour with a clock.
// All of it is one blob in NVS, saved at most every SAVE_MS, so a power cut
// loses up to that much. Each column is kept with its device's id, so a
// device list that changes in a new build does not mix two devices.
#include "energy.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include "esp_log.h"
#include "esp_netif_sntp.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "nvs.h"
#include "plugs.h"
#include "sdkconfig.h"

static const char *TAG = "energy";

#define HOURS      48
#define DAYS       62
#define MAX_GAP_MS 60000
#define SAVE_MS    (15 * 60 * 1000)
#define UNIT       3600000ULL // tenths of a watt times ms in 0.1 Wh (360 J)
#define MAGIC      0x454e5231 // "ENR1": change it when store_t changes

typedef struct {
    uint32_t magic;
    uint32_t hour[HOURS];                    // the local hour number in each slot (0: empty)
    uint32_t day[DAYS];                      // the local day number in each slot
    uint32_t hour_dwh[HOURS][PLUGS_MAX_DEV]; // in 0.1 Wh
    uint32_t day_dwh[DAYS][PLUGS_MAX_DEV];
    uint64_t total_dwh[PLUGS_MAX_DEV];
    char id[PLUGS_MAX_DEV][24]; // the device in each column
} store_t;

typedef struct {
    bool known;     // at_ms and w10 hold the last good read
    int64_t at_ms;  // when
    int w10;        // its power, in tenths of a watt
    uint64_t rest;  // what is left under 0.1 Wh, in tenths of a watt times ms
    uint32_t early; // 0.1 Wh measured before the clock was set
} meter_t;

static store_t st;
static meter_t meter[PLUGS_MAX_DEV];
static SemaphoreHandle_t mu;
static int64_t saved_ms;
static bool dirty;

// --- The calendar ---------------------------------------------------------------

// Days since 1970-01-01 for a date, and back (Howard Hinnant's algorithms).
static int32_t days_from_civil(int y, unsigned m, unsigned d)
{
    y -= m <= 2;
    const int era = (y >= 0 ? y : y - 399) / 400;
    const unsigned yoe = (unsigned)(y - era * 400);
    const unsigned doy = (153 * (m > 2 ? m - 3 : m + 9) + 2) / 5 + d - 1;
    const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return era * 146097 + (int32_t)doe - 719468;
}

static void civil_from_days(int32_t z, int *y, unsigned *m, unsigned *d)
{
    z += 719468;
    const int era = (z >= 0 ? z : z - 146096) / 146097;
    const unsigned doe = (unsigned)(z - era * 146097);
    const unsigned yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    const unsigned doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    const unsigned mp = (5 * doy + 2) / 153;
    *d = doy - (153 * mp + 2) / 5 + 1;
    *m = mp < 10 ? mp + 3 : mp - 9;
    *y = (int)yoe + era * 400 + (*m <= 2);
}

bool energy_clock_ok(void)
{
    return time(NULL) > 1735689600; // after 2025-01-01: set from the network
}

// The local day and hour numbers now. False before the clock is set.
static bool local_now(uint32_t *hour, uint32_t *day)
{
    if (!energy_clock_ok()) {
        return false;
    }
    const time_t t = time(NULL);
    struct tm tm;
    localtime_r(&t, &tm);
    *day =
        (uint32_t)days_from_civil(tm.tm_year + 1900, (unsigned)tm.tm_mon + 1, (unsigned)tm.tm_mday);
    *hour = *day * 24 + (uint32_t)tm.tm_hour;
    return true;
}

void energy_clock_text(char *out, size_t size)
{
    out[0] = 0;
    if (energy_clock_ok()) {
        const time_t t = time(NULL);
        struct tm tm;
        localtime_r(&t, &tm);
        strftime(out, size, "%Y-%m-%dT%H:%M:%S", &tm);
    }
}

// --- NVS ----------------------------------------------------------------------

static void save(void)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open("energy", NVS_READWRITE, &nvs);
    if (err == ESP_OK) {
        err = nvs_set_blob(nvs, "store", &st, sizeof(st));
        if (err == ESP_OK) {
            err = nvs_commit(nvs);
        }
        nvs_close(nvs);
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "not saved: %s", esp_err_to_name(err));
    }
}

// Load the saved history, column by column, for the devices of this build.
static void load(void)
{
    store_t *old = calloc(1, sizeof(*old));
    size_t size = sizeof(*old);
    nvs_handle_t nvs;
    bool found = false;
    if (old && nvs_open("energy", NVS_READONLY, &nvs) == ESP_OK) {
        found = nvs_get_blob(nvs, "store", old, &size) == ESP_OK && size == sizeof(*old) &&
                old->magic == MAGIC;
        nvs_close(nvs);
    }
    memset(&st, 0, sizeof(st));
    st.magic = MAGIC;
    for (int i = 0; i < plugs_count(); i++) {
        snprintf(st.id[i], sizeof(st.id[i]), "%s", plugs_dev_id(i));
    }
    if (found) {
        memcpy(st.hour, old->hour, sizeof(st.hour));
        memcpy(st.day, old->day, sizeof(st.day));
        for (int i = 0; i < plugs_count(); i++) {
            for (int j = 0; j < PLUGS_MAX_DEV; j++) {
                if (strcmp(old->id[j], st.id[i]) != 0) {
                    continue;
                }
                for (int k = 0; k < HOURS; k++) {
                    st.hour_dwh[k][i] = old->hour_dwh[k][j];
                }
                for (int k = 0; k < DAYS; k++) {
                    st.day_dwh[k][i] = old->day_dwh[k][j];
                }
                st.total_dwh[i] = old->total_dwh[j];
            }
        }
    }
    ESP_LOGI(TAG, "%s", found ? "history loaded" : "no history yet");
    free(old);
}

// --- Adding up ----------------------------------------------------------------

// Put dwh (0.1 Wh) of device dev in the slots of now. Called with mu taken.
static void add(int dev, uint32_t dwh)
{
    uint32_t hour, day;
    st.total_dwh[dev] += dwh;
    dirty = true;
    if (!local_now(&hour, &day)) {
        meter[dev].early += dwh;
        return;
    }
    dwh += meter[dev].early;
    meter[dev].early = 0;
    const int h = (int)(hour % HOURS), d = (int)(day % DAYS);
    if (st.hour[h] != hour) { // a slot from 48 hours ago: start it again
        st.hour[h] = hour;
        memset(st.hour_dwh[h], 0, sizeof(st.hour_dwh[h]));
    }
    if (st.day[d] != day) {
        st.day[d] = day;
        memset(st.day_dwh[d], 0, sizeof(st.day_dwh[d]));
    }
    st.hour_dwh[h][dev] += dwh;
    st.day_dwh[d][dev] += dwh;
}

void energy_fresh(int dev)
{
    if (!mu || dev >= PLUGS_MAX_DEV) {
        return;
    }
    int w10, v10, ma;
    const bool ok = plugs_power(dev, &w10, &v10, &ma) && w10 >= 0;
    const int64_t now = esp_timer_get_time() / 1000;
    xSemaphoreTake(mu, portMAX_DELAY);
    meter_t *m = &meter[dev];
    if (ok) {
        if (m->known) {
            int64_t dt = now - m->at_ms;
            dt = dt > MAX_GAP_MS ? MAX_GAP_MS : dt;
            m->rest += (uint64_t)m->w10 * (uint64_t)dt;
            if (m->rest >= UNIT) {
                add(dev, (uint32_t)(m->rest / UNIT));
                m->rest %= UNIT;
            }
        }
        m->known = true;
        m->at_ms = now;
        m->w10 = w10;
    }
    const bool due = dirty && now - saved_ms >= SAVE_MS;
    if (due) {
        save();
        saved_ms = now;
        dirty = false;
    }
    xSemaphoreGive(mu);
}

void energy_start(void)
{
    setenv("TZ", CONFIG_HOME_TZ, 1);
    tzset();
    esp_sntp_config_t cfg = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
    if (esp_netif_sntp_init(&cfg) != ESP_OK) {
        ESP_LOGW(TAG, "no SNTP: the energy waits for a clock");
    }
    load();
    saved_ms = esp_timer_get_time() / 1000;
    mu = xSemaphoreCreateMutex();
}

// --- JSON ---------------------------------------------------------------------

static void dwh_list(jw_t *w, const uint32_t *dwh)
{
    jw_raw(w, "[");
    for (int i = 0; i < plugs_count(); i++) {
        jw_raw(w, i ? "," : "");
        jw_tenths(w, (long)dwh[i]);
    }
    jw_raw(w, "]");
}

void energy_json(jw_t *w)
{
    static store_t copy; // the web server's task only: one answer at a time
    char clock[24];
    uint32_t hour = 0, day = 0;
    const bool clock_ok = local_now(&hour, &day);
    if (!mu) {
        jw_raw(w, "null");
        return;
    }
    xSemaphoreTake(mu, portMAX_DELAY);
    copy = st;
    xSemaphoreGive(mu);
    energy_clock_text(clock, sizeof(clock));
    jw_raw(w, "{\"clock\":");
    jw_str(w, clock_ok ? clock : NULL);
    jw_raw(w, ",\"devices\":[");
    for (int i = 0; i < plugs_count(); i++) {
        jw_raw(w, i ? "," : "");
        jw_raw(w, "{\"name\":");
        jw_str(w, plugs_dev_name(i));
        jw_raw(w, ",\"total_wh\":");
        jw_fmt(w, "%llu.%llu", copy.total_dwh[i] / 10, copy.total_dwh[i] % 10);
        jw_raw(w, "}");
    }
    // Only the slots of the last 48 hours and 62 days, oldest first. An hour
    // with no slot had no energy, or the bridge was off.
    jw_raw(w, "],\"hours\":[");
    bool first = true;
    for (uint32_t k = clock_ok ? hour - (HOURS - 1) : 1; clock_ok && k <= hour; k++) {
        if (copy.hour[k % HOURS] != k) {
            continue;
        }
        int y;
        unsigned m, d;
        civil_from_days((int32_t)(k / 24), &y, &m, &d);
        jw_fmt(w, "%s{\"t\":\"%04d-%02u-%02uT%02u\",\"wh\":", first ? "" : ",", y, m, d,
               (unsigned)(k % 24));
        dwh_list(w, copy.hour_dwh[k % HOURS]);
        jw_raw(w, "}");
        first = false;
    }
    jw_raw(w, "],\"days\":[");
    first = true;
    for (uint32_t k = clock_ok ? day - (DAYS - 1) : 1; clock_ok && k <= day; k++) {
        if (copy.day[k % DAYS] != k) {
            continue;
        }
        int y;
        unsigned m, d;
        civil_from_days((int32_t)k, &y, &m, &d);
        jw_fmt(w, "%s{\"d\":\"%04d-%02u-%02u\",\"wh\":", first ? "" : ",", y, m, d);
        dwh_list(w, copy.day_dwh[k % DAYS]);
        jw_raw(w, "}");
        first = false;
    }
    jw_raw(w, "]}");
}
