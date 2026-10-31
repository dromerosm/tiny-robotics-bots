// Wireless link: the robot joins the strongest home Wi-Fi network, or makes
// its own (access point). It sends telemetry over UDP, takes commands over
// UDP, and takes new firmware over HTTP (OTA). It is bala2.local (mDNS). See
// link.cpp.
#pragma once

#include <stdbool.h>

#define LINK_UDP_PORT 4210
#define LINK_IP       "192.168.4.1" // on the robot's own network
#define LINK_HOST     "bala2.local"

typedef struct {
    // A line came in over UDP (for example "kp 30").
    void (*on_command)(const char *line);
    // Asked before an update. Return true once the motors are stopped.
    bool (*stop_for_update)(void);
} link_callbacks_t;

// Join a home network or start the access point, then the UDP link and the
// update server. Takes up to about 15 s.
void link_start(const link_callbacks_t *cb);

// For the screen: the network name, the key (own network's password and
// update token), the robot's address, and if it is on a home network.
const char *link_ssid(void);
const char *link_password(void);
const char *link_ip(void);
bool link_on_home(void);

// Send one line to the computer that last talked to us (if any).
void link_send(const char *line);

// False if a computer was talking to us and has been quiet for 1 second.
bool link_alive(void);
