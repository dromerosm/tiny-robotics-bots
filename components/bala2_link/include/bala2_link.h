// bala2_link: Wi-Fi, updates over Wi-Fi and a small web server for BALA2 Fire
// firmware. See README.md.
//
// After link_start() the robot:
// - is on the strongest home network set at build time, or on its own
//   network (BALA2-xxxx, at 192.168.4.1) if none is in range
// - is bala2.local (mDNS)
// - takes commands and sends telemetry over UDP, port 4210
// - takes a new app with POST /update (header "X-Token: <key>")
// - serves the firmware's page on GET /, its state on GET /state and its
//   commands on POST /cmd, if the firmware gives them
//
// Safety: once a computer has talked to the robot over UDP, link_alive()
// turns false if it is quiet for more than 1 second.
//
// A new app is on trial. If it does not get the link up within 3 starts, the
// robot goes back to the app it had before (see link_guard()).
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LINK_UDP_PORT 4210
#define LINK_IP       "192.168.4.1" // on the robot's own network
#define LINK_HOST     "bala2.local"

// What the firmware gives the link. Every field can be NULL (or 0).
typedef struct {
    // A command line came in, over UDP or POST /cmd (for example "kp 30").
    void (*on_command)(const char *line);
    // Asked before an update. Return false to refuse it (the robot balances).
    bool (*can_update)(void);
    // Write the robot's state as JSON, for GET /state.
    void (*state_json)(char *buf, size_t size);
    // The page for GET / (HTML). Without one, GET / gives the same as /info.
    const char *page;
    size_t page_len;
} link_callbacks_t;

// Call this first in app_main(). It counts the starts of a new app. On the
// third start without a working link it sets the old app to boot again and
// restarts. It does nothing when the running app is not on trial.
void link_guard(void);

// Join a home network or start the robot's own, then the UDP link and the web
// server. Takes up to about 15 s. On an error it returns it and the robot goes
// on without a link: it never stops the firmware.
esp_err_t link_start(const link_callbacks_t *cb);

// For the screen: the network name, the key (the own network's password and
// the update token), the robot's address, and if it is on a home network.
const char *link_ssid(void);
const char *link_password(void);
const char *link_ip(void);
bool link_on_home(void);
// Signal of the home network in dBm, or 0 on the robot's own network.
int link_rssi(void);

// Send one line to the computer that last talked to us over UDP (if any).
void link_send(const char *line);

// False if a computer was talking to us and has been quiet for 1 second.
// Cheap, and safe to call from a fast control loop.
bool link_alive(void);

// True while an update is being written. A robot must not start moving then:
// the flash writes stall both cores for a few milliseconds at a time.
bool link_updating(void);

#ifdef __cplusplus
}
#endif
