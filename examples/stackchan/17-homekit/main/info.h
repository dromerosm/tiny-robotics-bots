// info: what the Info tab shows that is not a plug. See info.c.
#pragma once

#include <stddef.h>

// Call it once a second while the Info tab is open. It never waits: it starts a
// search for Apple TVs on the network now and then, and picks up the answers.
void info_poll(void);
// The bridge, its HomeKit clients and the Apple TVs, one item per line.
void info_text(char *out, size_t size);
// A number that changes when info_text() would say something new.
unsigned info_version(void);
