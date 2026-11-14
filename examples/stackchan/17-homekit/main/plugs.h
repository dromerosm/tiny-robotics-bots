// plugs: the Tuya plugs on the home network (tr_tuya). See plugs.c.
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

#define PLUGS_MAX_DEV 4
#define PLUGS_MAX_SW  5

// Read the devices from CONFIG_HOME_DEVICES and start the two tasks. fresh(dev)
// runs in the worker task each time a device has been read.
esp_err_t plugs_start(void (*fresh)(int dev));

int plugs_count(void);
int plugs_switches(int dev);
const char *plugs_dev_name(int dev);
const char *plugs_dev_id(int dev);
const char *plugs_sw_name(int dev, int sw);
int plugs_sw_dp(int dev, int sw);

// 1 on, 0 off, -1 when the device does not answer.
int plugs_state(int dev, int sw);
// Switch an outlet. It only puts the order in a queue: the worker task does
// it, then reads the device again. False when it will not do it: a full
// queue, or off for the access point outlet while the robot is on that Wi-Fi.
bool plugs_set(int dev, int sw, bool on);

// One device in a few words: its IP and its power, or why it does not answer.
void plugs_info(int dev, char *out, size_t size);
// Every device as short text, one per line, and a number that grows with each read.
void plugs_text(char *out, size_t size);
unsigned plugs_reads(void);
