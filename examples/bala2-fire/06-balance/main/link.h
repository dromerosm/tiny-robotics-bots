// Wireless link: the robot makes its own Wi-Fi network (access point), sends
// telemetry over UDP, takes commands over UDP, and takes new firmware over
// HTTP (OTA). See link.cpp.
#pragma once

#include <stdbool.h>

#define LINK_UDP_PORT 4210
#define LINK_IP       "192.168.4.1"

typedef struct {
    // A line came in over UDP (for example "kp 30").
    void (*on_command)(const char *line);
    // Asked before an update. Return true once the motors are stopped.
    bool (*stop_for_update)(void);
} link_callbacks_t;

// Start the access point, the UDP link and the update server.
void link_start(const link_callbacks_t *cb);

// The network name and password, for the screen.
const char *link_ssid(void);
const char *link_password(void);

// Send one line to the computer that last talked to us (if any).
void link_send(const char *line);

// False if a computer was talking to us and has been quiet for 1 second.
bool link_alive(void);
