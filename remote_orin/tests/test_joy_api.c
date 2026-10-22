/* Host tests for rc_proto and joy_api. Build: see README.md. */
#include "joy_api.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

static int checks, failures;
#define CHECK(x) do { checks++; if (!(x)) { failures++; printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #x); } } while (0)

static const JoyConfig CONFIG = {.peer_address = 1, .link_timeout_ms = 300, .deadband = 8,
                                 .drive_sign = 1, .turn_sign = 1};

static RcInput input(uint16_t seq) {
    RcInput in;
    memset(&in, 0, sizeof in);
    in.seq = seq;
    in.flags = RC_FLAG_GAMEPAD_OK;
    return in;
}

/* Formats an RYLR998 receive line for an input frame, as the module would print it. */
static size_t rcv_line(const RcInput *in, uint16_t address, char *out) {
    uint8_t frame[RC_INPUT_SIZE];
    char hex[2 * RC_INPUT_SIZE + 1];
    rc_input_pack(in, frame);
    rc_hex_encode(frame, sizeof frame, hex);
    hex[2 * RC_INPUT_SIZE] = '\0';
    return (size_t)sprintf(out, "+RCV=%u,32,%s,-47,9\r\n", address, hex);
}

static void send_input(JoyApi *j, const RcInput *in, uint32_t now) {
    char line[96];
    size_t n = rcv_line(in, CONFIG.peer_address, line);
    joy_api_feed(j, (const uint8_t *)line, n, now);
}

static JoyOutput poll(JoyApi *j, uint32_t now) {
    JoyOutput out;
    joy_api_poll(j, now, &out);
    return out;
}

static bool near(float a, float b) { return fabsf(a - b) < 1e-4f; }

static void test_crc_and_frames(void) {
    const uint8_t check[] = "123456789";
    CHECK(rc_crc16(check, 9) == 0x29b1);

    RcInput in = input(0xbeef), back;
    in.drive = -100;
    in.turn = 37;
    for (unsigned i = 0; i < RC_COUNTER_COUNT; i++) in.counters[i] = (uint8_t)(15 - i);
    uint8_t frame[RC_INPUT_SIZE];
    rc_input_pack(&in, frame);
    CHECK(frame[0] == 1 && frame[1] == 0x11 && frame[2] == 0xef && frame[3] == 0xbe);
    CHECK(frame[7] == 0xef); /* START=15 low nibble, BACK=14 high nibble */
    CHECK(rc_input_unpack(frame, &back));
    CHECK(back.seq == 0xbeef && back.drive == -100 && back.turn == 37 && back.flags == 1);
    CHECK(memcmp(back.counters, in.counters, RC_COUNTER_COUNT) == 0);

    uint8_t bad[RC_INPUT_SIZE];
    memcpy(bad, frame, sizeof bad);
    bad[4] ^= 1;
    CHECK(!rc_input_unpack(bad, &back)); /* CRC */
    in.drive = 101;
    rc_input_pack(&in, bad);
    CHECK(!rc_input_unpack(bad, &back)); /* axis range */
    in.drive = 0;
    in.flags = 0x02;
    rc_input_pack(&in, bad);
    CHECK(!rc_input_unpack(bad, &back)); /* reserved flag */
    memcpy(bad, frame, sizeof bad);
    bad[1] = RC_STATUS_TYPE;
    CHECK(!rc_input_unpack(bad, &back)); /* type */

    RcStatus status = {7, 2, 3}, status_back;
    uint8_t sframe[RC_STATUS_SIZE];
    rc_status_pack(&status, sframe);
    CHECK(rc_status_unpack(sframe, &status_back));
    CHECK(status_back.ack_seq == 7 && status_back.indicator == 2 && status_back.state == 3);
    status.indicator = 4;
    rc_status_pack(&status, sframe);
    CHECK(!rc_status_unpack(sframe, &status_back));

    RcInput counted = input(0);
    counted.counters[RC_BTN_RB] = 15;
    rc_input_count_press(&counted, RC_BTN_RB);
    CHECK(counted.counters[RC_BTN_RB] == 0);
}

static void test_hex_and_at(void) {
    uint8_t bytes[2];
    CHECK(rc_hex_decode("a5Ff", 4, bytes) && bytes[0] == 0xa5 && bytes[1] == 0xff);
    CHECK(!rc_hex_decode("A5G0", 4, bytes));
    CHECK(!rc_hex_decode("A5F", 3, bytes));

    char line[JOY_REPLY_MAX];
    const uint8_t frame[2] = {0x01, 0xab};
    size_t n = rc_at_send(65535, frame, 2, line, sizeof line);
    CHECK(n == 22 && memcmp(line, "AT+SEND=65535,4,01AB\r\n", 22) == 0);
    CHECK(rc_at_send(1, frame, 2, line, 15) == 0);

    RcAtRcv rx;
    const char ok[] = "+RCV=12,4,01AB,-120,-5";
    CHECK(rc_at_parse_rcv(ok, strlen(ok), &rx));
    CHECK(rx.address == 12 && rx.frame_size == 2 && rx.frame[1] == 0xab && rx.rssi == -120 && rx.snr == -5);
    const char *bad[] = {"+OK", "+RCV=12,4,01AB,-120", "+RCV=12,3,01A,-1,1", "+RCV=12,4,01A,-1,1",
                         "+RCV=12,4,01AB,-1,1x", "+RCV=70000,4,01AB,-1,1", "+RCV=1,34,"
                         "0000000000000000000000000000000000,-1,1", "+RCV=1,4,01AB,-1,200"};
    for (unsigned i = 0; i < sizeof bad / sizeof bad[0]; i++) CHECK(!rc_at_parse_rcv(bad[i], strlen(bad[i]), &rx));
}

static void test_config(void) {
    JoyApi j;
    JoyConfig c = CONFIG;
    CHECK(joy_api_init(&j, &c));
    c.turn_sign = 0;
    CHECK(!joy_api_init(&j, &c));
    send_input(&j, &(RcInput){.seq = 1, .flags = RC_FLAG_GAMEPAD_OK}, 0);
    CHECK(!poll(&j, 0).link_ok); /* an invalid config never reports a link */
    c = CONFIG;
    c.link_timeout_ms = 10;
    CHECK(!joy_api_init(&j, &c));
}

static void test_link_and_neutral_interlock(void) {
    JoyApi j;
    joy_api_init(&j, &CONFIG);
    JoyOutput out = poll(&j, 0);
    CHECK(!out.link_ok && !out.sticks_neutral && out.events == 0); /* no LINK_LOST at boot */

    RcInput in = input(10);
    in.drive = 60; /* stick held while the link comes up */
    in.counters[RC_BTN_START] = 3;
    send_input(&j, &in, 100);
    out = poll(&j, 100);
    CHECK(out.link_ok && out.events == JOY_EV_LINK_RESTORED); /* baseline: START not fired */
    CHECK(out.drive == 0.0f && !out.sticks_neutral);
    CHECK(out.rssi == -47 && out.snr == 9);

    in.seq = 11;
    in.drive = 0;
    send_input(&j, &in, 200);
    out = poll(&j, 200);
    CHECK(out.sticks_neutral && out.drive == 0.0f);
    in.seq = 12;
    in.drive = 100;
    in.turn = -54;
    send_input(&j, &in, 300);
    out = poll(&j, 300);
    CHECK(near(out.drive, 1.0f) && near(out.turn, -0.5f) && !out.sticks_neutral);

    out = poll(&j, 599); /* 299 ms since the last packet */
    CHECK(out.link_ok);
    out = poll(&j, 600);
    CHECK(!out.link_ok && out.events == JOY_EV_LINK_LOST && out.drive == 0.0f);
    CHECK(poll(&j, 700).events == 0);

    /* Handheld rebooted: an older sequence is accepted while the link is down. */
    in = input(0);
    in.drive = 0;
    in.counters[RC_BTN_B] = 9;
    send_input(&j, &in, 800);
    out = poll(&j, 800);
    CHECK(out.link_ok && out.events == JOY_EV_LINK_RESTORED);

    /* A feed stamped after the poll tick is not a timeout. */
    in.seq = 1;
    send_input(&j, &in, 900);
    CHECK(poll(&j, 899).link_ok);
}

static void test_button_counters(void) {
    JoyApi j;
    joy_api_init(&j, &CONFIG);
    RcInput in = input(100);
    send_input(&j, &in, 0);
    poll(&j, 0);

    in.seq = 101;
    rc_input_count_press(&in, RC_BTN_START);
    rc_input_count_press(&in, RC_BTN_RB);
    send_input(&j, &in, 10);
    CHECK(poll(&j, 10).events == (JOY_EV_ENGINE_ON | JOY_EV_CONTROL_TOGGLE));

    send_input(&j, &in, 20); /* duplicate seq */
    CHECK(poll(&j, 20).events == 0);
    in.seq = 100;
    rc_input_count_press(&in, RC_BTN_Y);
    send_input(&j, &in, 30); /* stale seq */
    CHECK(poll(&j, 30).events == 0);

    in.seq = 102; /* the Y press above, plus a lost packet with another Y press */
    rc_input_count_press(&in, RC_BTN_Y);
    send_input(&j, &in, 40);
    CHECK(poll(&j, 40).events == JOY_EV_GEAR_FAN);

    in.seq = 103;
    for (int i = 0; i < 9; i++) rc_input_count_press(&in, RC_BTN_X); /* jump of 9: resync only */
    send_input(&j, &in, 50);
    CHECK(poll(&j, 50).events == 0);
    in.seq = 104;
    rc_input_count_press(&in, RC_BTN_X);
    send_input(&j, &in, 60);
    CHECK(poll(&j, 60).events == JOY_EV_GEAR_BLADE);

    in.seq = 105;
    rc_input_count_press(&in, RC_BTN_DPAD_UP);
    rc_input_count_press(&in, RC_BTN_DPAD_DOWN);
    send_input(&j, &in, 70);
    CHECK(poll(&j, 70).events == 0); /* up and down together do nothing */

    in.seq = 106;
    rc_input_count_press(&in, RC_BTN_DPAD_DOWN);
    send_input(&j, &in, 80);
    in.seq = 107;
    rc_input_count_press(&in, RC_BTN_DPAD_DOWN);
    send_input(&j, &in, 90); /* second press before the main loop polled */
    CHECK(poll(&j, 90).events == (JOY_EV_BLADE_DOWN | JOY_EV_OVERFLOW));

    in.seq = 0xffff; /* 107 -> 0xffff is backwards in serial-number order */
    send_input(&j, &in, 95);
    CHECK(poll(&j, 95).events == 0);

    joy_api_init(&j, &CONFIG);
    in = input(0xfffe);
    send_input(&j, &in, 0);
    poll(&j, 0);
    in.seq = 0xffff;
    send_input(&j, &in, 10);
    in.seq = 0; /* sequence wrap */
    rc_input_count_press(&in, RC_BTN_A);
    send_input(&j, &in, 20);
    CHECK(poll(&j, 20).events == JOY_EV_GEAR_NEUTRAL);
}

static void test_gamepad_lost(void) {
    JoyApi j;
    joy_api_init(&j, &CONFIG);
    RcInput in = input(1);
    send_input(&j, &in, 0);
    poll(&j, 0);
    char reply[JOY_REPLY_MAX];
    joy_api_take_reply(&j, reply, sizeof reply);

    in.seq = 2;
    in.flags = 0;
    in.drive = 100;
    send_input(&j, &in, 10);
    JoyOutput out = poll(&j, 10);
    CHECK(!out.link_ok && out.events == JOY_EV_LINK_LOST && out.drive == 0.0f);
    CHECK(joy_api_take_reply(&j, reply, sizeof reply) > 0); /* the status reply still goes out */

    in.seq = 3;
    in.flags = RC_FLAG_GAMEPAD_OK;
    in.drive = 0;
    send_input(&j, &in, 20);
    CHECK(poll(&j, 20).events == JOY_EV_LINK_RESTORED);
}

static void test_reply(void) {
    JoyApi j;
    joy_api_init(&j, &CONFIG);
    char reply[JOY_REPLY_MAX];
    CHECK(joy_api_take_reply(&j, reply, sizeof reply) == 0);

    send_input(&j, &(RcInput){.seq = 0x1234, .flags = RC_FLAG_GAMEPAD_OK}, 0);
    joy_api_set_status(&j, JOY_IND_RELEASED, 2);
    CHECK(joy_api_take_reply(&j, reply, 10) == 0); /* too small: stays due */
    size_t n = joy_api_take_reply(&j, reply, sizeof reply);
    CHECK(n == 31 && memcmp(reply, "AT+SEND=1,16,", 13) == 0 && memcmp(reply + n - 2, "\r\n", 2) == 0);
    uint8_t frame[RC_STATUS_SIZE];
    RcStatus status;
    CHECK(rc_hex_decode(reply + 13, 16, frame) && rc_status_unpack(frame, &status));
    CHECK(status.ack_seq == 0x1234 && status.indicator == JOY_IND_RELEASED && status.state == 2);
    CHECK(joy_api_take_reply(&j, reply, sizeof reply) == 0); /* one reply per packet */

    joy_api_set_status(&j, (JoyIndicator)9, 0);
    send_input(&j, &(RcInput){.seq = 0x1235, .flags = RC_FLAG_GAMEPAD_OK}, 10);
    n = joy_api_take_reply(&j, reply, sizeof reply);
    CHECK(rc_hex_decode(reply + 13, 16, frame) && rc_status_unpack(frame, &status));
    CHECK(status.indicator == JOY_IND_FAULT); /* out of range clamps to fault */
}

static void test_line_handling(void) {
    JoyApi j;
    joy_api_init(&j, &CONFIG);
    char line[96];
    RcInput in = input(5);
    size_t n = rcv_line(&in, CONFIG.peer_address, line);

    const char noise[] = "+OK\r\n+READY\r\n+ERR=4\r\n";
    joy_api_feed(&j, (const uint8_t *)noise, strlen(noise), 0);
    for (size_t i = 0; i < n; i++) joy_api_feed(&j, (const uint8_t *)line + i, 1, 0); /* byte by byte */
    CHECK(poll(&j, 0).events == JOY_EV_LINK_RESTORED);

    char other[96];
    in.seq = 6;
    rc_input_count_press(&in, RC_BTN_START);
    n = rcv_line(&in, 7, other); /* another handheld address */
    joy_api_feed(&j, (const uint8_t *)other, n, 10);
    CHECK(poll(&j, 10).events == 0);

    char junk[JOY_LINE_MAX + 20];
    memset(junk, 'Z', sizeof junk);
    joy_api_feed(&j, (const uint8_t *)junk, sizeof junk, 20);
    joy_api_feed(&j, (const uint8_t *)"\r\n", 2, 20);
    CHECK(poll(&j, 20).events == JOY_EV_OVERFLOW);
    n = rcv_line(&in, CONFIG.peer_address, line);
    joy_api_feed(&j, (const uint8_t *)line, n, 30);
    CHECK(poll(&j, 30).events == JOY_EV_ENGINE_ON); /* parser recovered */

    joy_api_report_overflow(&j);
    CHECK(poll(&j, 40).events == JOY_EV_OVERFLOW);
}

static void test_deadband_and_signs(void) {
    JoyApi j;
    JoyConfig c = CONFIG;
    c.turn_sign = -1;
    joy_api_init(&j, &c);
    RcInput in = input(1);
    send_input(&j, &in, 0);
    in.seq = 2;
    in.drive = 8;
    in.turn = 54;
    send_input(&j, &in, 10);
    JoyOutput out = poll(&j, 10);
    CHECK(out.drive == 0.0f && near(out.turn, -0.5f));
    in.seq = 3;
    in.drive = -9;
    send_input(&j, &in, 20);
    CHECK(near(poll(&j, 20).drive, -1.0f / 92.0f));
}

int main(void) {
    test_crc_and_frames();
    test_hex_and_at();
    test_config();
    test_link_and_neutral_interlock();
    test_button_counters();
    test_gamepad_lost();
    test_reply();
    test_line_handling();
    test_deadband_and_signs();
    printf("%d checks, %d failed\n", checks, failures);
    return failures ? 1 : 0;
}
