#include "rc_handheld.h"
#include <string.h>

#define PULSE_ON_MS 500u /* 0.2 s pulses were too faint to notice on the F710 */
#define PULSE_OFF_MS 300u
#define LONG_MS 1000u
#define SLOT_MASK ((uint16_t)((1u << RC_COUNTER_COUNT) - 1u))

static int8_t clamp_axis(int8_t value) {
    if (value > RC_AXIS_MAX) return (int8_t)RC_AXIS_MAX;
    if (value < -RC_AXIS_MAX) return (int8_t)-RC_AXIS_MAX;
    return value;
}

static void play(RcHandheld *h, uint32_t now, uint8_t pulses, uint16_t on_ms, uint16_t off_ms) {
    h->rumble_ms = now;
    h->pulses = pulses;
    h->on_ms = on_ms;
    h->off_ms = off_ms;
}

/* RC_LORA_PROTOCOL.md 4.3: ORIN three pulses, JOY one, waiting two, fault one long rumble. */
static void play_indicator(RcHandheld *h, uint32_t now) {
    static const uint8_t pulses[] = {3, 1, 2};
    if (h->indicator < sizeof pulses)
        play(h, now, pulses[h->indicator], PULSE_ON_MS, PULSE_OFF_MS);
    else
        play(h, now, 1, LONG_MS, 0);
}

static size_t put_uint(char *out, uint32_t x) {
    char digits[10];
    size_t n = 0;
    do {
        digits[n++] = (char)('0' + x % 10u);
        x /= 10u;
    } while (x);
    for (size_t i = 0; i < n; i++) out[i] = digits[n - 1 - i];
    return n;
}

/* Strict decimal: at least one digit, no overflow past max. */
static bool parse_uint(const char **p, const char *end, uint32_t max, uint32_t *out) {
    const char *s = *p;
    uint32_t x = 0;
    if (s == end || *s < '0' || *s > '9') return false;
    while (s < end && *s >= '0' && *s <= '9') {
        x = x * 10u + (uint32_t)(*s - '0');
        if (x > max) return false;
        s++;
    }
    *p = s;
    *out = x;
    return true;
}

static bool expect(const char **p, const char *end, char c) {
    if (*p == end || **p != c) return false;
    (*p)++;
    return true;
}

/* joy_api's reply passed through the transparent module: "AT+SEND=<handheld address>,16,<hex>". */
static void handle_line(RcHandheld *h, const char *line, size_t len, uint32_t now) {
    const char *p = line, *end = line + len;
    uint32_t address, hex_len;
    uint8_t frame[RC_STATUS_SIZE];
    RcStatus status;
    if (len < 8 || memcmp(p, "AT+SEND=", 8) != 0) return;
    p += 8;
    if (!parse_uint(&p, end, 0xffffu, &address) || address != RC_HH_ADDRESS || !expect(&p, end, ',') ||
        !parse_uint(&p, end, 2 * RC_STATUS_SIZE, &hex_len) || hex_len != 2 * RC_STATUS_SIZE ||
        !expect(&p, end, ',') || (size_t)(end - p) != hex_len || !rc_hex_decode(p, hex_len, frame) ||
        !rc_status_unpack(frame, &status))
        return;

    bool was_up = h->link_up;
    uint8_t before = h->indicator;
    h->link_up = true;
    h->indicator = status.indicator;
    h->state = status.state;
    h->last_reply_ms = now;
    h->replies++;
    if (!was_up) {
        h->events |= (uint16_t)RC_HH_EV_LINK_UP;
        play_indicator(h, now); /* tell the operator the current state on (re)connect */
    } else if (status.indicator != before) {
        h->events |= (uint16_t)RC_HH_EV_INDICATOR;
        play_indicator(h, now);
    }
}

void rc_hh_init(RcHandheld *h, uint32_t now_ms) {
    memset(h, 0, sizeof *h);
    h->next_send_ms = now_ms;
    h->last_reply_ms = now_ms;
    h->alarm_ms = now_ms + RC_HH_LINK_TIMEOUT_MS;
}

uint16_t rc_hh_sample(RcHandheld *h, const RcPad *pad) {
    if (!pad->connected) {
        /* Buttons are unknown, so the held set stays: a button still down on reconnect is no new press. */
        h->input.flags = 0;
        h->input.drive = h->input.turn = 0;
        return 0;
    }
    uint16_t buttons = (uint16_t)(pad->buttons & SLOT_MASK);
    uint16_t pressed = (uint16_t)(buttons & ~h->held);
    for (unsigned i = 0; i < RC_COUNTER_COUNT; i++)
        if (pressed & (1u << i)) rc_input_count_press(&h->input, i);
    h->held = buttons;
    h->input.flags = RC_FLAG_GAMEPAD_OK;
    h->input.drive = clamp_axis(pad->drive);
    h->input.turn = clamp_axis(pad->turn);
    return pressed;
}

size_t rc_hh_take_line(RcHandheld *h, uint32_t now_ms, char *out, size_t cap) {
    uint8_t frame[RC_INPUT_SIZE];
    size_t pos = 0;
    if ((int32_t)(now_ms - h->next_send_ms) < 0 || cap < RC_HH_SEND_MAX) return 0;
    h->input.seq++;
    rc_input_pack(&h->input, frame);

    /* "+RCV=<address>,32,<hex>,0,0\r\n": RSSI and SNR are unknown to the sender, so 0. */
    memcpy(out, "+RCV=", 5);
    pos = 5;
    pos += put_uint(out + pos, RC_HH_ADDRESS);
    out[pos++] = ',';
    pos += put_uint(out + pos, 2 * RC_INPUT_SIZE);
    out[pos++] = ',';
    rc_hex_encode(frame, sizeof frame, out + pos);
    pos += 2 * RC_INPUT_SIZE;
    memcpy(out + pos, ",0,0\r\n", 6);
    pos += 6;

    h->next_send_ms += RC_HH_SEND_PERIOD_MS;
    if ((int32_t)(now_ms - h->next_send_ms) >= 0) h->next_send_ms = now_ms + RC_HH_SEND_PERIOD_MS; /* no burst */
    return pos;
}

void rc_hh_feed(RcHandheld *h, const uint8_t *data, size_t len, uint32_t now_ms) {
    for (size_t i = 0; i < len; i++) {
        char c = (char)data[i];
        if (c == '\n') {
            size_t n = h->line_len;
            if (!h->discarding) {
                if (n && h->line[n - 1] == '\r') n--;
                handle_line(h, h->line, n, now_ms);
            }
            h->line_len = 0;
            h->discarding = false;
        } else if (h->discarding) {
            continue;
        } else if (h->line_len < RC_HH_LINE_MAX) {
            h->line[h->line_len++] = c;
        } else {
            h->discarding = true;
        }
    }
}

uint16_t rc_hh_poll(RcHandheld *h, uint32_t now_ms) {
    if ((int32_t)(now_ms - h->last_reply_ms) >= (int32_t)RC_HH_LINK_TIMEOUT_MS) {
        if (h->link_up) {
            h->link_up = false;
            h->events |= (uint16_t)RC_HH_EV_LINK_LOST;
            h->alarm_ms = now_ms;
        }
        if ((int32_t)(now_ms - h->alarm_ms) >= 0) {
            play(h, now_ms, 1, LONG_MS, 0);
            h->events |= (uint16_t)RC_HH_EV_ALARM;
            h->alarm_ms = now_ms + RC_HH_ALARM_REPEAT_MS;
        }
    }
    uint16_t events = h->events;
    h->events = 0;
    return events;
}

bool rc_hh_rumble(const RcHandheld *h, uint32_t now_ms) {
    uint32_t elapsed = now_ms - h->rumble_ms, period = (uint32_t)h->on_ms + h->off_ms;
    if (!h->pulses || !period || elapsed / period >= h->pulses) return false;
    return elapsed % period < h->on_ms;
}
