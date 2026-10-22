/* ctypes wrapper: exposes common/joy_api.c to joy_lib.py. Built by tools.load_library(). */
#include "joy_api.h"
#include <stdlib.h>

#ifdef _WIN32
#define EXPORT __declspec(dllexport)
#else
#define EXPORT
#endif

typedef struct {
    int link_ok, sticks_neutral;
    float drive, turn;
    unsigned events;
    int rssi, snr;
} JoyFlat;

EXPORT void *joy_new(unsigned peer_address, unsigned timeout_ms, unsigned deadband) {
    JoyConfig config = {(uint16_t)peer_address, timeout_ms, (uint8_t)deadband, 1, 1};
    JoyApi *j = malloc(sizeof *j);
    if (j && !joy_api_init(j, &config)) {
        free(j);
        return NULL;
    }
    return j;
}

EXPORT void joy_free(void *j) { free(j); }

EXPORT void joy_feed(void *j, const char *data, unsigned len, unsigned now_ms) {
    joy_api_feed(j, (const uint8_t *)data, len, now_ms);
}

EXPORT unsigned joy_take_reply(void *j, char *out, unsigned cap) {
    return (unsigned)joy_api_take_reply(j, out, cap);
}

EXPORT void joy_set_status(void *j, int indicator, int state) {
    joy_api_set_status(j, (JoyIndicator)indicator, (uint8_t)state);
}

EXPORT void joy_poll(void *j, unsigned now_ms, JoyFlat *out) {
    JoyOutput o;
    joy_api_poll(j, now_ms, &o);
    out->link_ok = o.link_ok;
    out->sticks_neutral = o.sticks_neutral;
    out->drive = o.drive;
    out->turn = o.turn;
    out->events = o.events;
    out->rssi = o.rssi;
    out->snr = o.snr;
}
