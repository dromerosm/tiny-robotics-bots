// bala2_mon: watch and tune a BALA2 Fire firmware from a computer, over the
// link of bala2_link. See README.md.
//
// - Variables: the firmware names the numbers it wants to show and change.
//   "kp 3.5" over UDP (or POST /cmd) sets one, "?" lists them all.
// - Fast samples: the control loop hands in one sample per tick, and they go
//   out over UDP as they come, about 20 ms later ("rec 3" for 3 seconds).
//
// The control loop pays almost nothing: a variable is a plain number it reads
// as before, and a sample is a copy of a few floats into a ring. Everything
// else runs in the tasks of core 0.
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MON_VARS     40 // at most this many variables
#define MON_REC_COLS 10 // at most this many numbers in a sample

// Name a number, with the range a command may set it to. The name has up to
// 7 characters. Call these at boot, before link_start().
void mon_float(const char *name, volatile float *value, float min, float max);
void mon_int(const char *name, volatile int *value, int min, int max);

// ---- Values that outlive a restart ----
//
// The values can go to flash (NVS), so the robot starts with them and nobody
// has to build a new firmware to keep a good set. A flash write stops both
// cores for a few ms: call mon_save() and mon_defaults() only while the
// control loop can take that (the robot does not balance).

// Leave a variable out of the saved set (a servo speed, for example). Call
// it after the variable is named.
void mon_not_saved(const char *name);

// Read the saved values, each kept inside its range. Call it once at boot,
// after link_start() (it opens the flash storage). Returns how many it read.
int mon_load(void);

// Write the values of now to flash. False if the flash did not take them.
bool mon_save(void);

// Delete the saved values and go back to the ones the firmware started with.
bool mon_defaults(void);

// How many values are in flash now (0 = none: the firmware's values rule).
int mon_stored(void);

// Take a command line. "name value" sets a variable (kept inside its range)
// and "?" lists them; both answer with the list. Returns false if the line is
// not one of these, so the firmware can try its own commands.
bool mon_command(const char *line);

// Send the list: "gains kp 3.5 ki 10 ...".
void mon_list(void);

// Write the variables as a JSON object, {"kp":3.5,"ki":10}, for GET /state.
void mon_json(char *buf, size_t size);

// Print a line on the serial port and send it over the link (printf style).
void mon_say(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

// ---- Fast samples ----

// The names of the numbers in a sample, like "tilt,rate,u". Call at boot.
void mon_rec_columns(const char *names);

// Send the next n samples (0 stops). Safe from any task.
void mon_rec_start(int n);

// True while samples are being taken.
bool mon_rec_active(void);

// Hand in one sample: the time and MON_REC_COLS numbers (or fewer, as named).
// The control task calls it on every tick, also when nothing is being taken:
// a take starts and stops here. It never waits. With nothing asked for it
// returns at once, and if the ring is full the sample is counted as lost.
void mon_rec_add(uint32_t t_ms, const float *values);

// Send the samples that are waiting, in one UDP packet. Call it from a task
// on core 0, 50 times a second or more.
void mon_rec_send(void);

#ifdef __cplusplus
}
#endif
