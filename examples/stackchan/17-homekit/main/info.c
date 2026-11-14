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
//   has a HomeKit connection to the robot is marked "hub": it is the home hub
//   in use, or at least it is talking to the bridge.
#include "info.h"

#include <stdio.h>
#include <string.h>
#include "bridge.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
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
                snprintf(found[n].name, sizeof(found[n].name), "%s",
                         r->instance_name ? r->instance_name : "Apple TV");
                snprintf(found[n].ip, sizeof(found[n].ip), IPSTR, IP2STR(&a->addr.u_addr.ip4));
                n++;
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
            ESP_LOGI("info", "Apple TV \"%s\" at %s", tvs[i].name, tvs[i].ip);
        }
    }
}

void info_poll(void)
{
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
}

unsigned info_version(void)
{
    return version;
}

void info_text(char *out, size_t size)
{
    char list[sizeof(last_clients)];
    size_t n = 0;
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
        // "hub": this Apple TV has a HomeKit connection to the robot now.
        char spaced[20];
        snprintf(spaced, sizeof(spaced), " %s ", tvs[i].ip);
        char all[sizeof(last_clients) + 2];
        snprintf(all, sizeof(all), " %s ", last_clients);
        n += snprintf(out + n, n < size ? size - n : 0, "TV %s\n %s%s\n", tvs[i].name, tvs[i].ip,
                      strstr(all, spaced) ? " hub" : "");
    }
}
