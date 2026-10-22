/* Host tests for rc_handheld, plus a round trip through the receiver's joy_api. Build: run_tests.py. */
#include "joy_api.h"
#include "rc_handheld.h"
#include <stdio.h>
#include <string.h>

static int checks, failures;
#define CHECK(x) do { checks++; if (!(x)) { failures++; printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #x); } } while (0)

static const RcPad NEUTRAL = {.connected = true};

/* Decodes a handheld line back into the input frame it carries. */
static bool decode_line(const char *line, size_t n, RcInput *in) {
    RcAtRcv rx;
    if (n < 2 || line[n - 2] != '\r' || line[n - 1] != '\n') return false;
    return rc_at_parse_rcv(line, n - 2, &rx) && rx.address == RC_HH_ADDRESS &&
           rx.frame_size == RC_INPUT_SIZE && rc_input_unpack(rx.frame, in);
}

/* The receiver's reply line for an indicator and state, as joy_api would send it. */
static size_t reply_line(uint16_t address, uint8_t indicator, uint8_t state, char *out) {
    RcStatus status = {1, indicator, state};
    uint8_t frame[RC_STATUS_SIZE];
    rc_status_pack(&status, frame);
    return rc_at_send(address, frame, sizeof frame, out, 64);
}

static void feed_reply(RcHandheld *h, uint8_t indicator, uint32_t now) {
    char line[64];
    size_t n = reply_line(RC_HH_ADDRESS, indicator, 3, line);
    rc_hh_feed(h, (const uint8_t *)line, n, now);
}

static void test_frames(void) {
    RcHandheld h;
    char line[RC_HH_SEND_MAX];
    RcInput in;
    rc_hh_init(&h, 1000);

    size_t n = rc_hh_take_line(&h, 1000, line, sizeof line);
    CHECK(n == 48 && memcmp(line, "+RCV=1,32,", 10) == 0 && memcmp(line + 42, ",0,0\r\n", 6) == 0);
    CHECK(decode_line(line, n, &in) && in.seq == 1 && in.flags == 0); /* no gamepad sample yet */
    CHECK(rc_hh_take_line(&h, 1099, line, sizeof line) == 0);
    CHECK(rc_hh_take_line(&h, 1100, line, 10) == 0); /* too small: not consumed */
    n = rc_hh_take_line(&h, 1100, line, sizeof line);
    CHECK(decode_line(line, n, &in) && in.seq == 2);
    CHECK(rc_hh_take_line(&h, 1500, line, sizeof line) > 0); /* late: one line, no burst */
    CHECK(rc_hh_take_line(&h, 1550, line, sizeof line) == 0);
    CHECK(rc_hh_take_line(&h, 1600, line, sizeof line) > 0);

    /* Same bytes as the Python packer in rc_pc_test.py: seq 1, gamepad ok, all neutral. */
    rc_hh_init(&h, 0);
    rc_hh_sample(&h, &NEUTRAL);
    n = rc_hh_take_line(&h, 0, line, sizeof line);
    CHECK(n == 48 && memcmp(line, "+RCV=1,32,01110100000001000000000000007AC5,0,0\r\n", 48) == 0);
}

static void test_sampling(void) {
    RcHandheld h;
    char line[RC_HH_SEND_MAX];
    RcInput in;
    rc_hh_init(&h, 0);

    RcPad pad = {.connected = true, .drive = 120, .turn = -128, .buttons = 1u << RC_BTN_START};
    CHECK(rc_hh_sample(&h, &pad) == 1u << RC_BTN_START);
    CHECK(rc_hh_sample(&h, &pad) == 0); /* held: no new press */
    pad.buttons = 0;
    rc_hh_sample(&h, &pad);
    pad.buttons = (1u << RC_BTN_START) | (1u << RC_BTN_RB) | 0x8000u; /* bit 15 is no slot */
    CHECK(rc_hh_sample(&h, &pad) == ((1u << RC_BTN_START) | (1u << RC_BTN_RB)));
    size_t n = rc_hh_take_line(&h, 0, line, sizeof line);
    CHECK(decode_line(line, n, &in));
    CHECK(in.drive == 100 && in.turn == -100 && in.flags == RC_FLAG_GAMEPAD_OK); /* clamped */
    CHECK(in.counters[RC_BTN_START] == 2 && in.counters[RC_BTN_RB] == 1 && in.counters[RC_BTN_B] == 0);

    /* Gamepad gone: flags and axes cleared; a button still held on return is no new press. */
    RcPad gone = {.connected = false, .drive = 50, .buttons = 1u << RC_BTN_B};
    CHECK(rc_hh_sample(&h, &gone) == 0);
    n = rc_hh_take_line(&h, 100, line, sizeof line);
    CHECK(decode_line(line, n, &in) && in.flags == 0 && in.drive == 0 && in.counters[RC_BTN_START] == 2);
    CHECK(rc_hh_sample(&h, &pad) == 0);
    pad.buttons = 1u << RC_BTN_B;
    CHECK(rc_hh_sample(&h, &pad) == 1u << RC_BTN_B);
}

static void test_replies_and_rumble(void) {
    RcHandheld h;
    rc_hh_init(&h, 0);
    CHECK(!rc_hh_rumble(&h, 0));

    feed_reply(&h, JOY_IND_JOY, 100);
    CHECK(rc_hh_poll(&h, 100) == RC_HH_EV_LINK_UP && h.link_up && h.indicator == 1 && h.state == 3);
    CHECK(rc_hh_rumble(&h, 100) && rc_hh_rumble(&h, 599) && !rc_hh_rumble(&h, 600)); /* one pulse */
    CHECK(!rc_hh_rumble(&h, 900));

    feed_reply(&h, JOY_IND_JOY, 200);
    CHECK(rc_hh_poll(&h, 200) == 0); /* unchanged: no event */

    feed_reply(&h, JOY_IND_ORIN, 1000);
    CHECK(rc_hh_poll(&h, 1000) == RC_HH_EV_INDICATOR);
    CHECK(rc_hh_rumble(&h, 1000) && !rc_hh_rumble(&h, 1500) && rc_hh_rumble(&h, 1800)); /* three pulses */
    CHECK(rc_hh_rumble(&h, 2600) && !rc_hh_rumble(&h, 3100) && !rc_hh_rumble(&h, 3400));

    feed_reply(&h, JOY_IND_FAULT, 3500);
    CHECK(rc_hh_poll(&h, 3500) == RC_HH_EV_INDICATOR);
    CHECK(rc_hh_rumble(&h, 4499) && !rc_hh_rumble(&h, 4500)); /* one long */

    /* Ignored: wrong address, bad CRC, other module output, an over-long line. */
    char line[64];
    size_t n = reply_line(2, JOY_IND_JOY, 3, line);
    rc_hh_feed(&h, (const uint8_t *)line, n, 3600);
    n = reply_line(RC_HH_ADDRESS, JOY_IND_JOY, 3, line);
    line[20] = line[20] == '0' ? '1' : '0';
    rc_hh_feed(&h, (const uint8_t *)line, n, 3600);
    rc_hh_feed(&h, (const uint8_t *)"+OK\r\n+READY\r\n", 13, 3600);
    char junk[RC_HH_LINE_MAX + 10];
    memset(junk, 'Z', sizeof junk);
    rc_hh_feed(&h, (const uint8_t *)junk, sizeof junk, 3600);
    rc_hh_feed(&h, (const uint8_t *)"\n", 1, 3600);
    CHECK(rc_hh_poll(&h, 3600) == 0 && h.indicator == JOY_IND_FAULT && h.replies == 4);

    /* Byte by byte, after the junk: the parser recovered. */
    n = reply_line(RC_HH_ADDRESS, JOY_IND_RELEASED, 1, line);
    for (size_t i = 0; i < n; i++) rc_hh_feed(&h, (const uint8_t *)line + i, 1, 3700);
    CHECK(rc_hh_poll(&h, 3700) == RC_HH_EV_INDICATOR && h.indicator == JOY_IND_RELEASED && h.state == 1);
}

static void test_link_loss(void) {
    RcHandheld h;
    rc_hh_init(&h, 0);
    CHECK(rc_hh_poll(&h, 299) == 0);
    CHECK(rc_hh_poll(&h, 300) == RC_HH_EV_ALARM); /* nothing heard since boot */
    CHECK(rc_hh_rumble(&h, 1299) && !rc_hh_rumble(&h, 1300));
    CHECK(rc_hh_poll(&h, 3299) == 0 && rc_hh_poll(&h, 3300) == RC_HH_EV_ALARM);

    feed_reply(&h, JOY_IND_JOY, 4000);
    CHECK(rc_hh_poll(&h, 4000) == RC_HH_EV_LINK_UP);
    CHECK(rc_hh_poll(&h, 4299) == 0);
    CHECK(rc_hh_poll(&h, 4300) == (RC_HH_EV_LINK_LOST | RC_HH_EV_ALARM) && !h.link_up);
    CHECK(rc_hh_rumble(&h, 4300) && !rc_hh_rumble(&h, 5300));
    CHECK(rc_hh_poll(&h, 7299) == 0 && rc_hh_poll(&h, 7300) == RC_HH_EV_ALARM);

    feed_reply(&h, JOY_IND_JOY, 7400); /* same indicator as before the loss: still reported */
    CHECK(rc_hh_poll(&h, 7400) == RC_HH_EV_LINK_UP && rc_hh_rumble(&h, 7400));
}

/* Handheld line -> joy_api (the Orin receiver) -> reply -> handheld. */
static void test_round_trip(void) {
    static const JoyConfig config = {.peer_address = RC_HH_ADDRESS, .link_timeout_ms = 300, .deadband = 8,
                                     .drive_sign = 1, .turn_sign = 1};
    RcHandheld h;
    JoyApi j;
    JoyOutput out;
    char line[RC_HH_SEND_MAX], reply[JOY_REPLY_MAX];
    rc_hh_init(&h, 0);
    joy_api_init(&j, &config);
    joy_api_set_status(&j, JOY_IND_JOY, 3);

    uint32_t now = 0;
    RcPad pad = NEUTRAL;
    for (int i = 0; i < 4; i++, now += 100) {
        if (i == 2) pad.buttons = 1u << RC_BTN_RB;
        if (i == 3) pad.drive = 100;
        rc_hh_sample(&h, &pad);
        size_t n = rc_hh_take_line(&h, now, line, sizeof line);
        joy_api_feed(&j, (const uint8_t *)line, n, now);
        size_t m = joy_api_take_reply(&j, reply, sizeof reply);
        CHECK(m > 0);
        rc_hh_feed(&h, (const uint8_t *)reply, m, now);
        joy_api_poll(&j, now, &out);
        if (i == 0) CHECK(out.link_ok && out.events == JOY_EV_LINK_RESTORED);
        if (i == 2) CHECK(out.events == JOY_EV_CONTROL_TOGGLE);
        if (i == 3) CHECK(out.drive == 1.0f && out.events == 0);
    }
    CHECK(rc_hh_poll(&h, now) == RC_HH_EV_LINK_UP && h.indicator == JOY_IND_JOY && h.replies == 4);
}

int main(void) {
    test_frames();
    test_sampling();
    test_replies_and_rumble();
    test_link_loss();
    test_round_trip();
    printf("%d checks, %d failed\n", checks, failures);
    return failures ? 1 : 0;
}
