/* Handheld side of the RC LoRa link: gamepad samples in, RYLR998 lines out, rumble out.
 *
 * Shared by the laptop stand-in (handheld_pc.py, through ctypes) and the Feather RP2040
 * sketch, so a laptop test exercises the same logic the MCU will run. Wire format:
 * RC_LORA_PROTOCOL.md v0.3. The RYLR998 must be in transparent mode: the handheld writes
 * the exact "+RCV=..." line the receiver's joy_api expects and reads joy_api's
 * "AT+SEND=..." reply (RC_LORA_PROTOCOL.md section 2.4).
 *
 * Not thread-safe: call everything from one task. No HAL, no heap. */
#ifndef RC_HANDHELD_H
#define RC_HANDHELD_H
#include "rc_proto.h"

#define RC_HH_ADDRESS 1u            /* "+RCV=1,..." label; must match joy_api peer_address */
#define RC_HH_SEND_PERIOD_MS 100u   /* 10 Hz */
#define RC_HH_LINK_TIMEOUT_MS 300u  /* no valid reply for this long means the link is lost */
#define RC_HH_ALARM_REPEAT_MS 3000u /* the lost-link rumble repeats this often */
#define RC_HH_LINE_MAX 64u          /* longest reply line kept; longer lines are dropped */
#define RC_HH_SEND_MAX 64u          /* buffer size for rc_hh_take_line() */

/* rc_hh_poll() events */
enum {
    RC_HH_EV_LINK_UP = 1u << 0,   /* first valid reply after boot or after a loss */
    RC_HH_EV_LINK_LOST = 1u << 1, /* replies stopped */
    RC_HH_EV_INDICATOR = 1u << 2, /* indicator changed while linked */
    RC_HH_EV_ALARM = 1u << 3      /* lost-link rumble started, first time or repeat */
};

typedef struct {
    bool connected;   /* the gamepad is delivering reports */
    int8_t drive;     /* left stick Y, -100..100, forward positive */
    int8_t turn;      /* right stick X, -100..100, right positive */
    uint16_t buttons; /* bit (1 << RC_BTN_*) set while that button is held */
} RcPad;

/* Allocate statically. The link fields may be read; do not write any field. */
typedef struct {
    RcInput input; /* next frame: sequence, axes, flags, press counters */
    uint16_t held;
    uint32_t next_send_ms;
    char line[RC_HH_LINE_MAX];
    size_t line_len;
    bool discarding;
    bool link_up;
    uint8_t indicator; /* last reply: 0 ORIN, 1 JOY, 2 waiting, 3 fault */
    uint8_t state;     /* last reply: STM32 state, same values as Orin 0x81 byte 20 */
    uint32_t last_reply_ms, alarm_ms;
    uint32_t replies;
    uint16_t events;
    /* Rumble: `pulses` pulses of on_ms, each followed by off_ms, starting at rumble_ms. */
    uint32_t rumble_ms;
    uint16_t on_ms, off_ms;
    uint8_t pulses;
} RcHandheld;

void rc_hh_init(RcHandheld *h, uint32_t now_ms);
/* Latest gamepad state. Call about every 10 ms so short presses are counted.
 * Returns the RC_BTN bits newly pressed since the previous sample. */
uint16_t rc_hh_sample(RcHandheld *h, const RcPad *pad);
/* Writes the line due for the RYLR998 and returns its length, or 0 when none is due
 * or cap < RC_HH_SEND_MAX. No terminator is written. */
size_t rc_hh_take_line(RcHandheld *h, uint32_t now_ms, char *out, size_t cap);
/* Raw bytes from the RYLR998 UART, any split. */
void rc_hh_feed(RcHandheld *h, const uint8_t *data, size_t len, uint32_t now_ms);
/* Checks the reply timeout and returns the RC_HH_EV_* raised since the previous call. */
uint16_t rc_hh_poll(RcHandheld *h, uint32_t now_ms);
/* Whether the rumble motors should be on now. */
bool rc_hh_rumble(const RcHandheld *h, uint32_t now_ms);

#endif
