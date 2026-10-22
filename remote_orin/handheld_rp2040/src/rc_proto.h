/* RC LoRa link frames, shared by the handheld MCU and the STM32 joy_api.
 * Wire format: RC_LORA_PROTOCOL.md section 4. No HAL or OS dependency. */
#ifndef RC_PROTO_H
#define RC_PROTO_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define RC_PROTO_VERSION 0x01u
#define RC_INPUT_TYPE 0x11u
#define RC_STATUS_TYPE 0x91u
#define RC_INPUT_SIZE 16u
#define RC_STATUS_SIZE 8u
#define RC_FRAME_MAX RC_INPUT_SIZE
#define RC_COUNTER_COUNT 10u
#define RC_AXIS_MAX 100
#define RC_FLAG_GAMEPAD_OK 0x01u

/* Press-counter slots in wire order (byte 7 low nibble first). */
enum {
    RC_BTN_START = 0,  /* engine power on */
    RC_BTN_BACK,       /* engine power off */
    RC_BTN_X,          /* blade gear */
    RC_BTN_Y,          /* fan gear */
    RC_BTN_A,          /* clutch neutral */
    RC_BTN_B,          /* park + neutral */
    RC_BTN_RB,         /* JOY/ORIN toggle request */
    RC_BTN_LB,         /* unlock stop */
    RC_BTN_DPAD_UP,    /* blade up */
    RC_BTN_DPAD_DOWN   /* blade down */
};

typedef struct {
    uint16_t seq;
    int8_t drive;  /* -100..100, forward is positive */
    int8_t turn;   /* -100..100, right is positive */
    uint8_t flags; /* RC_FLAG_* */
    uint8_t counters[RC_COUNTER_COUNT]; /* 0..15, +1 per press */
} RcInput;

typedef struct {
    uint16_t ack_seq;
    uint8_t indicator; /* 0 ORIN, 1 JOY, 2 released, 3 fault; the handheld rumbles on change */
    uint8_t state;     /* STM32 state, same values as Orin 0x81 byte 20 */
} RcStatus;

/* One "+RCV=addr,len,data,rssi,snr" line from an RYLR998. */
typedef struct {
    uint16_t address;
    uint8_t frame[RC_FRAME_MAX];
    size_t frame_size;
    int16_t rssi;
    int8_t snr;
} RcAtRcv;

/* CRC-16/CCITT-FALSE, the same CRC as the Orin <-> STM32 protocol. */
uint16_t rc_crc16(const uint8_t *data, size_t size);

void rc_input_pack(const RcInput *in, uint8_t out[RC_INPUT_SIZE]);
bool rc_input_unpack(const uint8_t in[RC_INPUT_SIZE], RcInput *out);
void rc_status_pack(const RcStatus *in, uint8_t out[RC_STATUS_SIZE]);
bool rc_status_unpack(const uint8_t in[RC_STATUS_SIZE], RcStatus *out);

/* Handheld side: record one fresh press of a button slot. */
void rc_input_count_press(RcInput *in, unsigned slot);

/* Uppercase hex, 2*size chars, no terminator. */
void rc_hex_encode(const uint8_t *data, size_t size, char *out);
/* Accepts upper or lower case; hex_len must be even. out is undefined on failure. */
bool rc_hex_decode(const char *hex, size_t hex_len, uint8_t *out);

/* Writes "AT+SEND=<address>,<2*size>,<hex>\r\n". Returns its length, or 0 when
 * cap is too small or size exceeds RC_FRAME_MAX. No terminator is written. */
size_t rc_at_send(uint16_t address, const uint8_t *frame, size_t size, char *out, size_t cap);
/* Parses one line without its "\r\n". Returns false for any other line, such as "+OK". */
bool rc_at_parse_rcv(const char *line, size_t len, RcAtRcv *out);

#endif
