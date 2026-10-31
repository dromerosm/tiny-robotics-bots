// tr_ota: home Wi-Fi, updates over Wi-Fi and rollback for StackChan examples.
//
// Every example that runs on the robot needs this. An app sent over Wi-Fi
// boots "on trial" and must mark itself valid, or the robot goes back to the
// old app on the next restart. An app without this code also can't take the
// next update.
//
// After tr_ota_start() the robot:
// - joins the strongest of the two networks set at build time (see README.md),
//   and joins again when the link drops
// - is at http://stackchan.local/ (mDNS)
// - serves GET / (status: firmware, uptime, memory, Wi-Fi, and lines the
//   example adds with tr_ota_add_status()) and POST /update (a new app image, with
//   the header "X-Token: <token>"; the token is shown only on the USB port)
// - marks an app on trial valid once it has Wi-Fi and the web server runs, or
//   goes back to the old app if there is no Wi-Fi within 60 seconds
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"
#include "esp_http_server.h"

#ifdef __cplusplus
extern "C" {
#endif

// Start all of the above. It returns at once and does not wait for Wi-Fi, so
// the example can go on without a network. It sets up NVS, esp_netif and the
// default event loop if the example has not done it yet. Call it once.
esp_err_t tr_ota_start(void);

// Add a line (or a few) to the status on GET /. fn writes text ending in "\n"
// into text. It runs in the web server's task each time someone asks. Up to
// 4 functions; call before or after tr_ota_start().
typedef void (*tr_ota_status_fn)(char *text, size_t size);
esp_err_t tr_ota_add_status(tr_ota_status_fn fn);

// Add an endpoint of the example to the web server (up to 8), before or after
// tr_ota_start(). For anything that changes the robot, check the token first:
//   if (!tr_ota_token_ok(req)) return ESP_FAIL;   // it has sent the 403 answer
esp_err_t tr_ota_add_uri(const httpd_uri_t *uri);
bool tr_ota_token_ok(httpd_req_t *req);

// The running app's state ("valid", "on trial", ...) and why the robot last
// restarted ("power on", "crash", ...), as GET / shows them.
const char *tr_ota_state(void);
const char *tr_ota_last_restart(void);

#ifdef __cplusplus
}
#endif
