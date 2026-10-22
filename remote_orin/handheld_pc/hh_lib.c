/* ctypes wrapper: exposes common/rc_handheld.c to handheld_lib.py. Built by tools.load_library(). */
#include "rc_handheld.h"
#include <stdlib.h>

#ifdef _WIN32
#define EXPORT __declspec(dllexport)
#else
#define EXPORT
#endif

typedef struct {
    int link_up, indicator, state;
    unsigned seq, replies;
    unsigned counters[RC_COUNTER_COUNT];
} HhInfo;

static int8_t axis(int value) {
    return (int8_t)(value > RC_AXIS_MAX ? RC_AXIS_MAX : value < -RC_AXIS_MAX ? -RC_AXIS_MAX : value);
}

EXPORT void *hh_new(unsigned now_ms) {
    RcHandheld *h = malloc(sizeof *h);
    if (h) rc_hh_init(h, now_ms);
    return h;
}

EXPORT void hh_free(void *h) { free(h); }

EXPORT unsigned hh_sample(void *h, int connected, int drive, int turn, unsigned buttons) {
    RcPad pad = {connected != 0, axis(drive), axis(turn), (uint16_t)buttons};
    return rc_hh_sample(h, &pad);
}

EXPORT unsigned hh_take_line(void *h, unsigned now_ms, char *out, unsigned cap) {
    return (unsigned)rc_hh_take_line(h, now_ms, out, cap);
}

EXPORT void hh_feed(void *h, const char *data, unsigned len, unsigned now_ms) {
    rc_hh_feed(h, (const uint8_t *)data, len, now_ms);
}

EXPORT unsigned hh_poll(void *h, unsigned now_ms) { return rc_hh_poll(h, now_ms); }

EXPORT int hh_rumble(void *h, unsigned now_ms) { return rc_hh_rumble(h, now_ms); }

EXPORT void hh_info(void *p, HhInfo *out) {
    const RcHandheld *h = p;
    out->link_up = h->link_up;
    out->indicator = h->indicator;
    out->state = h->state;
    out->seq = h->input.seq;
    out->replies = h->replies;
    for (unsigned i = 0; i < RC_COUNTER_COUNT; i++) out->counters[i] = h->input.counters[i];
}
