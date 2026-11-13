// bala2_mon: variables and fast samples over the link. See include/bala2_mon.h.
#include "bala2_mon.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "bala2_link.h"
#include "nvs.h"

// ---- Variables ----

typedef struct {
    const char *name;
    volatile float *f; // one of the two is set
    volatile int *i;
    float min, max;
    float start; // the firmware's value, for mon_defaults()
    bool keep;   // false: mon_save() leaves it out
} var_t;

static var_t vars[MON_VARS];
static int var_count;
static int stored; // values in flash now

static float var_get(const var_t *v)
{
    return v->f ? *v->f : (float)*v->i;
}

// Keep the value inside the variable's range, then write it.
static void var_set(const var_t *v, float value)
{
    value = value < v->min ? v->min : value > v->max ? v->max : value;
    if (v->f) {
        *v->f = value;
    } else {
        *v->i = (int)value;
    }
}

static void add_var(const char *name, volatile float *f, volatile int *i, float min, float max)
{
    if (var_count < MON_VARS) {
        vars[var_count] = (var_t){name, f, i, min, max, 0, true};
        vars[var_count].start = var_get(&vars[var_count]);
        var_count++;
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

void mon_not_saved(const char *name)
{
    for (int k = 0; k < var_count; k++) {
        if (!strcmp(name, vars[k].name)) {
            vars[k].keep = false;
        }
    }
}

// ---- Values in flash ----
//
// One NVS entry per variable, under its name, so a firmware with more or
// fewer variables still reads the ones it knows. NVS has no float type: the
// entry holds the float's 32 bits.

#define MON_SPACE "mon"

int mon_load(void)
{
    nvs_handle_t nvs;
    stored = 0;
    if (nvs_open(MON_SPACE, NVS_READONLY, &nvs) != ESP_OK) {
        return 0; // nothing saved yet
    }
    for (int k = 0; k < var_count; k++) {
        uint32_t bits;
        float v;
        if (vars[k].keep && nvs_get_u32(nvs, vars[k].name, &bits) == ESP_OK) {
            memcpy(&v, &bits, sizeof(v));
            if (v == v) { // not NAN
                var_set(&vars[k], v);
                stored++;
            }
        }
    }
    nvs_close(nvs);
    return stored;
}

bool mon_save(void)
{
    nvs_handle_t nvs;
    if (nvs_open(MON_SPACE, NVS_READWRITE, &nvs) != ESP_OK) {
        return false;
    }
    int count = 0;
    bool ok = true;
    for (int k = 0; k < var_count; k++) {
        if (vars[k].keep) {
            const float v = var_get(&vars[k]);
            uint32_t bits;
            memcpy(&bits, &v, sizeof(bits));
            ok = nvs_set_u32(nvs, vars[k].name, bits) == ESP_OK && ok;
            count++;
        }
    }
    ok = nvs_commit(nvs) == ESP_OK && ok;
    nvs_close(nvs);
    if (ok) {
        stored = count;
    }
    return ok;
}

bool mon_defaults(void)
{
    nvs_handle_t nvs;
    if (nvs_open(MON_SPACE, NVS_READWRITE, &nvs) != ESP_OK) {
        return false;
    }
    const bool ok = nvs_erase_all(nvs) == ESP_OK && nvs_commit(nvs) == ESP_OK;
    nvs_close(nvs);
    if (ok) {
        stored = 0;
        for (int k = 0; k < var_count; k++) {
            if (vars[k].keep) {
                var_set(&vars[k], vars[k].start);
            }
        }
    }
    return ok;
}

int mon_stored(void)
{
    return stored;
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
            var_set(&vars[k], v);
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
