#include "joy_api.h"
#include <string.h>

#define BLADE_PAIR ((uint16_t)(JOY_EV_BLADE_UP | JOY_EV_BLADE_DOWN))

static bool seq_newer(uint16_t candidate, uint16_t previous) {
    uint16_t d = (uint16_t)(candidate - previous);
    return d != 0 && d < 0x8000u;
}

/* An event still unread from the previous poll means presses were merged. */
static void raise_events(JoyApi *j, uint16_t events) {
    if (j->events & events) j->events |= JOY_EV_OVERFLOW;
    j->events |= events;
}

static bool raw_neutral(const JoyApi *j) {
    int drive = j->drive < 0 ? -j->drive : j->drive;
    int turn = j->turn < 0 ? -j->turn : j->turn;
    return drive <= j->cfg.deadband && turn <= j->cfg.deadband;
}

static float shape(int value, uint8_t deadband) {
    int magnitude = value < 0 ? -value : value;
    if (magnitude <= deadband) return 0.0f;
    float x = (float)(magnitude - deadband) / (float)(RC_AXIS_MAX - deadband);
    return value < 0 ? -x : x;
}

static void lose_link(JoyApi *j) {
    if (j->link_ok) raise_events(j, JOY_EV_LINK_LOST);
    j->link_ok = false;
    j->armed = false;
    j->drive = j->turn = 0;
}

static void handle_line(JoyApi *j, const char *line, size_t len, uint32_t now) {
    RcAtRcv rx;
    RcInput in;
    if (!rc_at_parse_rcv(line, len, &rx) || rx.address != j->cfg.peer_address ||
        rx.frame_size != RC_INPUT_SIZE || !rc_input_unpack(rx.frame, &in))
        return;
    /* While the link is down any sequence is accepted: the handheld may have rebooted. */
    if (j->link_ok && j->have_seq && !seq_newer(in.seq, j->last_seq)) return;
    j->have_seq = true;
    j->last_seq = in.seq;
    j->rssi = rx.rssi;
    j->snr = rx.snr;
    j->reply_due = true;
    j->reply_seq = in.seq;

    if (!(in.flags & RC_FLAG_GAMEPAD_OK)) {
        lose_link(j);
        return;
    }
    j->last_good_ms = now;
    j->drive = in.drive;
    j->turn = in.turn;
    if (!j->link_ok) {
        /* First packet after boot or loss is a baseline: earlier presses never fire. */
        j->link_ok = true;
        j->armed = false;
        memcpy(j->counters, in.counters, sizeof j->counters);
        raise_events(j, JOY_EV_LINK_RESTORED);
    } else {
        uint16_t events = 0;
        for (unsigned i = 0; i < RC_COUNTER_COUNT; i++) {
            uint8_t d = (uint8_t)((in.counters[i] - j->counters[i]) & 0x0fu);
            if (d >= 1 && d <= 7) events |= (uint16_t)(1u << i);
            j->counters[i] = in.counters[i];
        }
        /* Up and down together: neither moves the blade. */
        if ((events & BLADE_PAIR) == BLADE_PAIR) events &= (uint16_t)~BLADE_PAIR;
        if (events) raise_events(j, events);
    }
    /* Sticks must pass through neutral before a restored link may drive. */
    if (raw_neutral(j)) j->armed = true;
}

bool joy_api_init(JoyApi *j, const JoyConfig *cfg) {
    memset(j, 0, sizeof *j);
    j->indicator = JOY_IND_FAULT;
    if (!cfg || cfg->link_timeout_ms < 50 || cfg->link_timeout_ms > 5000 || cfg->deadband > 50 ||
        (cfg->drive_sign != 1 && cfg->drive_sign != -1) || (cfg->turn_sign != 1 && cfg->turn_sign != -1))
        return false;
    j->cfg = *cfg;
    j->ready = true;
    return true;
}

void joy_api_feed(JoyApi *j, const uint8_t *data, size_t len, uint32_t now_ms) {
    if (!j->ready) return;
    for (size_t i = 0; i < len; i++) {
        char c = (char)data[i];
        if (c == '\n') {
            size_t n = j->line_len;
            if (!j->discarding) {
                if (n && j->line[n - 1] == '\r') n--;
                handle_line(j, j->line, n, now_ms);
            }
            j->line_len = 0;
            j->discarding = false;
        } else if (j->discarding) {
            continue;
        } else if (j->line_len < JOY_LINE_MAX) {
            j->line[j->line_len++] = c;
        } else {
            j->discarding = true;
            raise_events(j, JOY_EV_OVERFLOW);
        }
    }
}

void joy_api_report_overflow(JoyApi *j) {
    raise_events(j, JOY_EV_OVERFLOW);
}

void joy_api_poll(JoyApi *j, uint32_t now_ms, JoyOutput *out) {
    memset(out, 0, sizeof *out);
    if (!j->ready) return;
    /* Signed difference: a feed stamped slightly after this poll is not a timeout. */
    if (j->link_ok && (int32_t)(now_ms - j->last_good_ms) >= (int32_t)j->cfg.link_timeout_ms)
        lose_link(j);
    out->link_ok = j->link_ok;
    out->sticks_neutral = j->link_ok && raw_neutral(j);
    if (j->link_ok && j->armed) {
        out->drive = shape(j->cfg.drive_sign * j->drive, j->cfg.deadband);
        out->turn = shape(j->cfg.turn_sign * j->turn, j->cfg.deadband);
    }
    out->events = j->events;
    j->events = 0;
    out->rssi = j->rssi;
    out->snr = j->snr;
}

void joy_api_set_status(JoyApi *j, JoyIndicator indicator, uint8_t state) {
    j->indicator = (uint8_t)(indicator <= JOY_IND_FAULT ? indicator : JOY_IND_FAULT);
    j->state = state;
}

size_t joy_api_take_reply(JoyApi *j, char *out, size_t cap) {
    if (!j->ready || !j->reply_due) return 0;
    RcStatus status = {j->reply_seq, j->indicator, j->state};
    uint8_t frame[RC_STATUS_SIZE];
    rc_status_pack(&status, frame);
    size_t n = rc_at_send(j->cfg.peer_address, frame, sizeof frame, out, cap);
    if (n) j->reply_due = false;
    return n;
}
