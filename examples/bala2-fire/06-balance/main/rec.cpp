// 200 Hz recorder, see rec.h.
#include "rec.h"

#include <stdio.h>

static rec_sample_t buf[REC_MAX];
static volatile int want; // samples asked for
static volatile int have; // samples recorded
static int sent = -1;     // -1: header not sent yet
static volatile bool recording;

void rec_start(int n)
{
    if (n < 1) {
        n = 1;
    }
    want = n > REC_MAX ? REC_MAX : n;
    have = 0;
    sent = -1;
    recording = true;
}

bool rec_active(void)
{
    return recording;
}

void rec_add(const rec_sample_t *s)
{
    if (!recording) {
        return;
    }
    buf[have] = *s;
    have = have + 1;
    if (have >= want) {
        recording = false; // full: ready to send
    }
}

bool rec_next_line(char *out, size_t len)
{
    if (recording || have == 0 || sent > have) {
        return false;
    }
    if (sent < 0) {
        snprintf(out, len, "rec_start %d samples\nrec,t_ms,tilt,rate,acc_tilt,x,v,u\n",
                 have);
    } else if (sent == have) {
        snprintf(out, len, "rec_end\n");
    } else {
        const rec_sample_t *s = &buf[sent];
        snprintf(out, len, "rec,%lu,%.3f,%.2f,%.3f,%.4f,%.4f,%.0f\n", (unsigned long)s->t_ms,
                 s->tilt, s->rate, s->acc_tilt, s->x, s->v, s->u);
    }
    sent = sent + 1;
    return true;
}
