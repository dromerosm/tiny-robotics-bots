// bala2_mon: variables and fast samples over the link. See include/bala2_mon.h.
#include "bala2_mon.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "bala2_link.h"

// ---- Variables ----

typedef struct {
    const char *name;
    volatile float *f; // one of the two is set
    volatile int *i;
    float min, max;
} var_t;

static var_t vars[MON_VARS];
static int var_count;

static void add_var(const char *name, volatile float *f, volatile int *i, float min, float max)
{
    if (var_count < MON_VARS) {
        vars[var_count++] = (var_t){name, f, i, min, max};
    }
}

void mon_float(const char *name, volatile float *value, float min, float max)
{
    add_var(name, value, NULL, min, max);
}

void mon_int(const char *name, volatile int *value, int min, int max)
{
    add_var(name, NULL, value, min, max);
}

static float var_get(const var_t *v)
{
    return v->f ? *v->f : (float)*v->i;
}

void mon_say(const char *fmt, ...)
{
    char line[256];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);
    fputs(line, stdout);
    link_send(line);
}

// Write every variable as <open><name><mid><value><sep>...<close>.
static void write_vars(char *buf, size_t size, const char *open, const char *quote, const char *mid,
                       const char *sep, const char *close)
{
    size_t at = snprintf(buf, size, "%s", open);
    for (int k = 0; k < var_count && at < size; k++) {
        at += snprintf(buf + at, size - at, "%s%s%s%s%s%.5g", k ? sep : "", quote, vars[k].name,
                       quote, mid, var_get(&vars[k]));
    }
    if (at < size) {
        snprintf(buf + at, size - at, "%s", close);
    }
}

void mon_list(void)
{
    char line[512];
    write_vars(line, sizeof(line), "gains ", "", " ", " ", "\n");
    fputs(line, stdout);
    link_send(line);
}

void mon_json(char *buf, size_t size)
{
    write_vars(buf, size, "{", "\"", ":", ",", "}");
}

bool mon_command(const char *line)
{
    char name[8];
    float v;
    const int n = sscanf(line, "%7s %f", name, &v);
    if (n == 1 && !strcmp(name, "?")) {
        mon_list();
        return true;
    }
    if (n != 2 || v != v) { // v != v: not a number
        return false;
    }
    for (int k = 0; k < var_count; k++) {
        if (!strcmp(name, vars[k].name)) {
            v = v < vars[k].min ? vars[k].min : v > vars[k].max ? vars[k].max : v;
            if (vars[k].f) {
                *vars[k].f = v;
            } else {
                *vars[k].i = (int)v;
            }
            mon_list();
            return true;
        }
    }
    return false;
}

// ---- Fast samples ----
//
// A ring with one writer (the control task, on core 1) and one reader (the
// task that calls mon_rec_send(), on core 0). Each side writes only its own
// words, and a word cannot be seen half written, so no lock is needed. One
// slot stays empty to tell a full ring from an empty one.

#define RING 1024 // samples: 5 s at 200 Hz if nobody reads; a power of two

typedef struct {
    uint32_t t_ms;
    float v[MON_REC_COLS];
} sample_t;

static sample_t ring[RING];
// Written by the control task only (in mon_rec_add()):
static volatile uint32_t head;  // next slot to write
static volatile int to_take;    // samples still to take
static volatile int taking;     // size of the take in progress, for the header
static volatile uint32_t takes; // counts the takes, so the sender sees a new one
static volatile uint32_t lost;  // samples dropped because the ring was full
// Written by the sender only (in mon_rec_send()):
static volatile uint32_t tail; // next slot to send
// Written by mon_rec_start(), from any task: a take to begin (-1 = stop).
static volatile int asked;
static const char *columns = "";
static int cols;

void mon_rec_columns(const char *names)
{
    columns = names;
    cols = 1;
    for (const char *c = names; *c; c++) {
        cols += *c == ',';
    }
    if (cols > MON_REC_COLS) {
        cols = MON_REC_COLS;
    }
}

void mon_rec_start(int n)
{
    asked = n > 0 ? n : -1;
}

bool mon_rec_active(void)
{
    return asked > 0 || to_take > 0;
}

void mon_rec_add(uint32_t t_ms, const float *values)
{
    const int want = asked;
    if (want != 0) { // a new take, or a stop
        asked = 0;
        to_take = want > 0 ? want : 0;
        if (want > 0) {
            taking = want;
            lost = 0;
            takes = takes + 1;
        }
    }
    const int left = to_take;
    if (left <= 0) {
        return;
    }
    const uint32_t h = head, next = (h + 1) & (RING - 1);
    if (next == tail) {
        lost = lost + 1;
    } else {
        ring[h].t_ms = t_ms;
        memcpy(ring[h].v, values, cols * sizeof(float));
        head = next;
    }
    to_take = left - 1;
}

void mon_rec_send(void)
{
    static uint32_t seen; // takes the header was sent for
    static bool open;     // a footer is still owed
    char out[1100];       // one packet, under the usual 1500 byte limit
    size_t at = 0;
    if (seen != takes) {
        seen = takes;
        open = true;
        at =
            snprintf(out, sizeof(out), "rec_start %d samples\nrec,t_ms,%s\n", (int)taking, columns);
    }
    // Up to 16 samples per call: at 50 calls a second that is 800 a second,
    // four times what a 200 Hz loop makes.
    for (int k = 0; k < 16 && tail != head && at < sizeof(out) - 180; k++) {
        const sample_t *s = &ring[tail];
        at += snprintf(out + at, sizeof(out) - at, "rec,%lu", (unsigned long)s->t_ms);
        for (int c = 0; c < cols; c++) {
            at += snprintf(out + at, sizeof(out) - at, ",%.5g", s->v[c]);
        }
        out[at++] = '\n';
        tail = (tail + 1) & (RING - 1);
    }
    if (open && !mon_rec_active() && tail == head && at < sizeof(out) - 40) {
        open = false;
        at += snprintf(out + at, sizeof(out) - at, "rec_end lost %lu\n", (unsigned long)lost);
    }
    if (at > 0) {
        out[at] = 0;
        link_send(out); // not to the serial port: it is too slow for this
    }
}
