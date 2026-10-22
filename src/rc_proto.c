#include "rc_proto.h"
#include <string.h>

static void put16(uint8_t *p, uint16_t x) { p[0] = (uint8_t)x; p[1] = (uint8_t)(x >> 8); }
static uint16_t get16(const uint8_t *p) { return (uint16_t)(p[0] | ((uint16_t)p[1] << 8)); }

uint16_t rc_crc16(const uint8_t *p, size_t n) {
    uint16_t crc = 0xffff;
    for (size_t i = 0; i < n; i++) {
        crc ^= (uint16_t)p[i] << 8;
        for (unsigned b = 0; b < 8; b++) crc = (uint16_t)((crc << 1) ^ ((crc & 0x8000) ? 0x1021 : 0));
    }
    return crc;
}

void rc_input_pack(const RcInput *in, uint8_t out[RC_INPUT_SIZE]) {
    memset(out, 0, RC_INPUT_SIZE);
    out[0] = RC_PROTO_VERSION;
    out[1] = RC_INPUT_TYPE;
    put16(out + 2, in->seq);
    out[4] = (uint8_t)in->drive;
    out[5] = (uint8_t)in->turn;
    out[6] = in->flags;
    for (unsigned i = 0; i < RC_COUNTER_COUNT; i++)
        out[7 + i / 2] |= (uint8_t)((in->counters[i] & 0x0fu) << (i % 2 ? 4 : 0));
    put16(out + 14, rc_crc16(out, 14));
}

bool rc_input_unpack(const uint8_t in[RC_INPUT_SIZE], RcInput *out) {
    int8_t drive = (int8_t)in[4], turn = (int8_t)in[5];
    if (in[0] != RC_PROTO_VERSION || in[1] != RC_INPUT_TYPE || get16(in + 14) != rc_crc16(in, 14))
        return false;
    if (drive < -RC_AXIS_MAX || drive > RC_AXIS_MAX || turn < -RC_AXIS_MAX || turn > RC_AXIS_MAX ||
        (in[6] & ~RC_FLAG_GAMEPAD_OK) || in[12] || in[13])
        return false;
    out->seq = get16(in + 2);
    out->drive = drive;
    out->turn = turn;
    out->flags = in[6];
    for (unsigned i = 0; i < RC_COUNTER_COUNT; i++)
        out->counters[i] = (uint8_t)((in[7 + i / 2] >> (i % 2 ? 4 : 0)) & 0x0fu);
    return true;
}

void rc_status_pack(const RcStatus *in, uint8_t out[RC_STATUS_SIZE]) {
    out[0] = RC_PROTO_VERSION;
    out[1] = RC_STATUS_TYPE;
    put16(out + 2, in->ack_seq);
    out[4] = in->indicator;
    out[5] = in->state;
    put16(out + 6, rc_crc16(out, 6));
}

bool rc_status_unpack(const uint8_t in[RC_STATUS_SIZE], RcStatus *out) {
    if (in[0] != RC_PROTO_VERSION || in[1] != RC_STATUS_TYPE || get16(in + 6) != rc_crc16(in, 6) ||
        in[4] > 3)
        return false;
    out->ack_seq = get16(in + 2);
    out->indicator = in[4];
    out->state = in[5];
    return true;
}

void rc_input_count_press(RcInput *in, unsigned slot) {
    if (slot < RC_COUNTER_COUNT) in->counters[slot] = (uint8_t)((in->counters[slot] + 1u) & 0x0fu);
}

void rc_hex_encode(const uint8_t *d, size_t n, char *out) {
    static const char digits[] = "0123456789ABCDEF";
    for (size_t i = 0; i < n; i++) {
        out[2 * i] = digits[d[i] >> 4];
        out[2 * i + 1] = digits[d[i] & 0x0f];
    }
}

static int hex_value(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

bool rc_hex_decode(const char *hex, size_t len, uint8_t *out) {
    if (len % 2) return false;
    for (size_t i = 0; i < len; i += 2) {
        int hi = hex_value(hex[i]), lo = hex_value(hex[i + 1]);
        if (hi < 0 || lo < 0) return false;
        out[i / 2] = (uint8_t)((hi << 4) | lo);
    }
    return true;
}

static size_t digit_count(uint32_t x) {
    size_t n = 1;
    while (x >= 10) { x /= 10; n++; }
    return n;
}

static size_t put_uint(char *out, uint32_t x) {
    size_t n = digit_count(x);
    for (size_t i = n; i > 0; i--) { out[i - 1] = (char)('0' + x % 10); x /= 10; }
    return n;
}

size_t rc_at_send(uint16_t address, const uint8_t *frame, size_t size, char *out, size_t cap) {
    if (size > RC_FRAME_MAX) return 0;
    size_t need = 8 + digit_count(address) + 1 + digit_count((uint32_t)(2 * size)) + 1 + 2 * size + 2;
    if (need > cap) return 0;
    size_t pos = 8;
    memcpy(out, "AT+SEND=", 8);
    pos += put_uint(out + pos, address);
    out[pos++] = ',';
    pos += put_uint(out + pos, (uint32_t)(2 * size));
    out[pos++] = ',';
    rc_hex_encode(frame, size, out + pos);
    pos += 2 * size;
    out[pos++] = '\r';
    out[pos++] = '\n';
    return pos;
}

/* Strict decimal: at least one digit, no overflow past max. */
static bool parse_uint(const char **p, const char *end, uint32_t max, uint32_t *out) {
    const char *s = *p;
    uint32_t x = 0;
    if (s == end || *s < '0' || *s > '9') return false;
    while (s < end && *s >= '0' && *s <= '9') {
        x = x * 10 + (uint32_t)(*s - '0');
        if (x > max) return false;
        s++;
    }
    *p = s;
    *out = x;
    return true;
}

static bool parse_int(const char **p, const char *end, int32_t lo, int32_t hi, int32_t *out) {
    bool negative = *p < end && **p == '-';
    uint32_t magnitude;
    if (negative) (*p)++;
    if (!parse_uint(p, end, 0x7fffu, &magnitude)) return false;
    int32_t x = negative ? -(int32_t)magnitude : (int32_t)magnitude;
    if (x < lo || x > hi) return false;
    *out = x;
    return true;
}

static bool expect(const char **p, const char *end, char c) {
    if (*p == end || **p != c) return false;
    (*p)++;
    return true;
}

bool rc_at_parse_rcv(const char *line, size_t len, RcAtRcv *out) {
    const char *p = line, *end = line + len;
    uint32_t address, hex_len;
    int32_t rssi, snr;
    if (len < 5 || memcmp(p, "+RCV=", 5) != 0) return false;
    p += 5;
    if (!parse_uint(&p, end, 0xffffu, &address) || !expect(&p, end, ',')) return false;
    if (!parse_uint(&p, end, 2 * RC_FRAME_MAX, &hex_len) || hex_len == 0 || hex_len % 2 ||
        !expect(&p, end, ','))
        return false;
    if ((size_t)(end - p) < hex_len || !rc_hex_decode(p, hex_len, out->frame)) return false;
    p += hex_len;
    if (!expect(&p, end, ',') || !parse_int(&p, end, -255, 255, &rssi) || !expect(&p, end, ',') ||
        !parse_int(&p, end, -128, 127, &snr) || p != end)
        return false;
    out->address = (uint16_t)address;
    out->frame_size = hex_len / 2;
    out->rssi = (int16_t)rssi;
    out->snr = (int8_t)snr;
    return true;
}
