// info: see info.h.
//
// Two things here come from the network:
//
// - The HomeKit clients: the addresses on the other end of the connections
//   that HomeKit's web server has open now. An iPhone connects while the Home
//   app is in use. A home hub keeps its connection open all the time.
// - The Apple TVs: HomeKit does not say which client is the home hub, so we
//   look for them ourselves. An Apple TV announces AirPlay on mDNS
//   (_airplay._tcp) with a "model" that starts with "AppleTV". One that also
//   has a HomeKit connection to the bridge is marked "hub": it is the home hub
//   in use, or at least it is talking to the bridge.
#include "info.h"

#include <stdio.h>
#include <string.h>
#include "bridge.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "hap_platform_httpd.h"
#include "lwip/sockets.h"
#include "mdns.h"

#define SEARCH_EVERY_US (60 * 1000000LL)
#define SEARCH_MS       2500
#define MAX_TV          2
#define MAX_CLIENTS     8

typedef struct {
    char name[20];
    char ip[16];
} tv_t;

static tv_t tvs[MAX_TV];
static int ntv;
static mdns_search_once_t *search;
static int64_t next_search;
static unsigned version;
static char last_clients[MAX_CLIENTS * 16];
static int nclients;
static SemaphoreHandle_t mu; // the screen task and the web server both call in

// The IPv4 addresses of HomeKit's open connections, separated by spaces.
static int clients(char *out, size_t size)
{
    int fds[MAX_CLIENTS], count = 0;
    size_t n = MAX_CLIENTS, used = 0;
    out[0] = 0;
    httpd_handle_t *server = hap_platform_httpd_get_handle();
    if (!server || !*server || httpd_get_client_list(*server, &n, fds) != ESP_OK) {
        return 0;
    }
    for (size_t i = 0; i < n; i++) {
        struct sockaddr_in6 peer;
        socklen_t len = sizeof(peer);
        if (getpeername(fds[i], (struct sockaddr *)&peer, &len) != 0) {
            continue;
        }
        // The server listens on IPv6 too, so an IPv4 client shows up as an
        // IPv4-mapped address: its last 4 bytes are the IPv4 address.
        const uint8_t *b = peer.sin6_family == AF_INET6
                               ? &peer.sin6_addr.s6_addr[12]
                               : (const uint8_t *)&((struct sockaddr_in *)&peer)->sin_addr;
        used += snprintf(out + used, used < size ? size - used : 0, "%s%d.%d.%d.%d",
                         count ? " " : "", b[0], b[1], b[2], b[3]);
        count++;
    }
    return count;
}

// A name as the screen can show it. The screen's font has only ASCII, and a
// name comes in UTF-8 ("Salón"), where a letter with an accent is two bytes.
// Those letters lose the accent ("Salon"), a curly apostrophe becomes a plain
// one, and any other letter the font does not have becomes one '?'.
static void ascii_name(char *out, size_t size, const char *name)
{
    // U+00C0 to U+00FF, in order: the byte after 0xC3 picks one.
    static const char plain[] = "AAAAAAACEEEEIIIIDNOOOOOxOUUUUYTs"
                                "aaaaaaaceeeeiiiidnooooo/ouuuuyty";
    const unsigned char *in = (const unsigned char *)name;
    size_t n = 0;
    while (*in && n + 1 < size) {
        if (*in < 0x80) {
            out[n++] = (char)*in++;
        } else if (in[0] == 0xC3 && in[1] >= 0x80 && in[1] <= 0xBF) {
            out[n++] = plain[in[1] - 0x80];
            in += 2;
        } else {
            out[n++] = in[0] == 0xE2 && in[1] == 0x80 && in[2] == 0x99 ? '\'' : '?';
            // Skip the rest of this letter: its bytes all start with bits 10.
            for (in++; (*in & 0xC0) == 0x80; in++) {
            }
        }
    }
    out[n] = 0;
}

static void keep_results(mdns_result_t *results)
{
    tv_t found[MAX_TV];
    int n = 0;
    for (mdns_result_t *r = results; r && n < MAX_TV; r = r->next) {
        bool apple_tv = false;
        for (size_t i = 0; i < r->txt_count; i++) {
            apple_tv =
                apple_tv || (strcmp(r->txt[i].key, "model") == 0 && r->txt_value_len[i] >= 7 &&
                             memcmp(r->txt[i].value, "AppleTV", 7) == 0);
        }
        for (mdns_ip_addr_t *a = r->addr; apple_tv && a; a = a->next) {
            if (a->addr.type == ESP_IPADDR_TYPE_V4) {
                ascii_name(found[n].name, sizeof(found[n].name),
                           r->instance_name ? r->instance_name : "Apple TV");
                snprintf(found[n].ip, sizeof(found[n].ip), IPSTR, IP2STR(&a->addr.u_addr.ip4));
                // The same TV can answer twice (once per interface or
                // address): keep it once.
                bool seen = false;
                for (int k = 0; k < n; k++) {
                    seen = seen || strcmp(found[k].ip, found[n].ip) == 0;
                }
                n += !seen;
                break;
            }
        }
    }
    if (n != ntv || memcmp(found, tvs, (size_t)n * sizeof(tv_t)) != 0) {
        memset(tvs, 0, sizeof(tvs));
        memcpy(tvs, found, (size_t)n * sizeof(tv_t));
        ntv = n;
        version++;
        for (int i = 0; i < n; i++) {
            ESP_LOGI("info", "Apple TV \"%s\" found", tvs[i].name);
        }
    }
}

void info_init(void)
{
    mu = xSemaphoreCreateMutex();
}

void info_poll(void)
{
    xSemaphoreTake(mu, portMAX_DELAY);
    const int64_t now = esp_timer_get_time();
    if (search) {
        mdns_result_t *results = NULL;
        uint8_t count = 0;
        if (mdns_query_async_get_results(search, 0, &results, &count)) {
            keep_results(results);
            mdns_query_results_free(results);
            mdns_query_async_delete(search);
            search = NULL;
        }
    } else if (now >= next_search) {
        next_search = now + SEARCH_EVERY_US;
        search = mdns_query_async_new(NULL, "_airplay", "_tcp", MDNS_TYPE_PTR, SEARCH_MS, 8, NULL);
    }
    char now_clients[sizeof(last_clients)];
    nclients = clients(now_clients, sizeof(now_clients));
    if (strcmp(now_clients, last_clients) != 0) {
        memcpy(last_clients, now_clients, sizeof(last_clients));
        version++;
    }
    xSemaphoreGive(mu);
}

unsigned info_version(void)
{
    return version;
}

// An Apple TV is "hub" when it has a HomeKit connection to the bridge now.
static bool is_hub(const tv_t *tv)
{
    char spaced[20], all[sizeof(last_clients) + 2];
    snprintf(spaced, sizeof(spaced), " %s ", tv->ip);
    snprintf(all, sizeof(all), " %s ", last_clients);
    return strstr(all, spaced) != NULL;
}

void info_json(jw_t *w)
{
    char list[sizeof(last_clients)];
    xSemaphoreTake(mu, portMAX_DELAY);
    memcpy(list, last_clients, sizeof(list));
    jw_raw(w, "{\"clients\":[");
    bool first = true;
    for (char *save = NULL, *one = strtok_r(list, " ", &save); one;
         one = strtok_r(NULL, " ", &save)) {
        jw_raw(w, first ? "" : ",");
        jw_str(w, one);
        first = false;
    }
    jw_fmt(w, "],\"looking\":%s,\"apple_tvs\":[", search ? "true" : "false");
    for (int i = 0; i < ntv; i++) {
        jw_raw(w, i ? ",{\"name\":" : "{\"name\":");
        jw_str(w, tvs[i].name);
        jw_raw(w, ",\"ip\":");
        jw_str(w, tvs[i].ip);
        jw_fmt(w, ",\"hub\":%s}", is_hub(&tvs[i]) ? "true" : "false");
    }
    jw_raw(w, "]}");
    xSemaphoreGive(mu);
}

void info_text(char *out, size_t size)
{
    char list[sizeof(last_clients)];
    size_t n = 0;
    xSemaphoreTake(mu, portMAX_DELAY);
    esp_netif_ip_info_t ip = {0};
    esp_netif_t *sta = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (sta) {
        esp_netif_get_ip_info(sta, &ip);
    }
    n += snprintf(out + n, size - n, "Bridge " IPSTR "\n", IP2STR(&ip.ip));
    n += snprintf(out + n, n < size ? size - n : 0, "HomeKit: %d paired\n", bridge_paired());
    // The clients as info_poll() last saw them.
    memcpy(list, last_clients, sizeof(list));
    n += snprintf(out + n, n < size ? size - n : 0, "Clients now: %d\n", nclients);
    for (char *save = NULL, *one = strtok_r(list, " ", &save); one;
         one = strtok_r(NULL, " ", &save)) {
        n += snprintf(out + n, n < size ? size - n : 0, " %s\n", one);
    }
    if (ntv == 0) {
        n += snprintf(out + n, n < size ? size - n : 0, "Apple TV: %s\n",
                      search ? "looking..." : "none found");
    }
    for (int i = 0; i < ntv; i++) {
        n += snprintf(out + n, n < size ? size - n : 0, "TV %s\n %s%s\n", tvs[i].name, tvs[i].ip,
                      is_hub(&tvs[i]) ? " hub" : "");
    }
    xSemaphoreGive(mu);
}
