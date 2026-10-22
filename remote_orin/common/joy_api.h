/* STM32 side of the handheld LoRa link: RYLR998 UART lines in, function requests out.
 *
 * Not thread-safe. Call every function from the same task: a UART ISR or DMA
 * callback should only queue received bytes, and the control task drains them
 * into joy_api_feed(). See README.md for the integration loop. */
#ifndef JOY_API_H
#define JOY_API_H
#include "rc_proto.h"

#define JOY_LINE_MAX 80u  /* longest RYLR998 line kept; longer lines raise JOY_EV_OVERFLOW */
#define JOY_REPLY_MAX 40u /* buffer size for joy_api_take_reply() */

/* One-shot events. Button bits match the RC_BTN_* slot numbers. */
enum {
    JOY_EV_ENGINE_ON      = 1u << RC_BTN_START,
    JOY_EV_ENGINE_OFF     = 1u << RC_BTN_BACK,
    JOY_EV_GEAR_BLADE     = 1u << RC_BTN_X,
    JOY_EV_GEAR_FAN       = 1u << RC_BTN_Y,
    JOY_EV_GEAR_NEUTRAL   = 1u << RC_BTN_A,
    JOY_EV_PARK           = 1u << RC_BTN_B,
    JOY_EV_CONTROL_TOGGLE = 1u << RC_BTN_RB,
    JOY_EV_UNLOCK         = 1u << RC_BTN_LB,
    JOY_EV_BLADE_UP       = 1u << RC_BTN_DPAD_UP,
    JOY_EV_BLADE_DOWN     = 1u << RC_BTN_DPAD_DOWN,
    JOY_EV_LINK_LOST      = 1u << 10, /* link was up and is now down */
    JOY_EV_LINK_RESTORED  = 1u << 11, /* link came up, including the first time after boot */
    JOY_EV_OVERFLOW       = 1u << 12  /* data may have been lost; see joy_api_report_overflow() */
};

typedef enum {
    JOY_IND_ORIN = 0,     /* ORIN holds control */
    JOY_IND_JOY = 1,      /* JOY holds control */
    JOY_IND_RELEASED = 2, /* handheld released, waiting for a new request */
    JOY_IND_FAULT = 3     /* stop lock or communication fault */
} JoyIndicator;

typedef struct {
    uint16_t peer_address;    /* RYLR998 ADDRESS of the handheld */
    uint32_t link_timeout_ms; /* 50..5000; no valid input for this long means link lost */
    uint8_t deadband;         /* 0..50, in stick percent */
    int8_t drive_sign;        /* +1 or -1: axis direction calibration */
    int8_t turn_sign;         /* +1 or -1 */
} JoyConfig;

typedef struct {
    bool link_ok;
    bool sticks_neutral; /* false while the link is down */
    float drive;         /* -1..1 after deadband, forward positive */
    float turn;          /* -1..1 after deadband, right positive */
    uint16_t events;     /* JOY_EV_* raised since the previous joy_api_poll() */
    int16_t rssi;        /* last accepted packet, for diagnostics */
    int8_t snr;
} JoyOutput;

/* Internal state. Allocate statically; do not touch the fields. */
typedef struct {
    JoyConfig cfg;
    bool ready;
    char line[JOY_LINE_MAX];
    size_t line_len;
    bool discarding;
    bool link_ok, armed;
    uint32_t last_good_ms;
    bool have_seq;
    uint16_t last_seq;
    uint8_t counters[RC_COUNTER_COUNT];
    int8_t drive, turn;
    uint16_t events;
    int16_t rssi;
    int8_t snr;
    bool reply_due;
    uint16_t reply_seq;
    uint8_t indicator, state;
} JoyApi;

/* Returns false for an invalid config; the instance then reports no link forever. */
bool joy_api_init(JoyApi *j, const JoyConfig *cfg);
/* Raw bytes from the RYLR998 UART, any split. now_ms: a monotonic millisecond tick. */
void joy_api_feed(JoyApi *j, const uint8_t *data, size_t len, uint32_t now_ms);
/* Call when the UART ring buffer dropped bytes. */
void joy_api_report_overflow(JoyApi *j);
/* Once per control step: checks the link timeout and hands over pending events. */
void joy_api_poll(JoyApi *j, uint32_t now_ms, JoyOutput *out);
/* Main control sets the indicator and state; each reply carries the latest values.
 * The handheld rumbles when the indicator changes. */
void joy_api_set_status(JoyApi *j, JoyIndicator indicator, uint8_t state);
/* The pending "AT+SEND=..." reply line for the RYLR998, or 0 if none is due.
 * Call right after joy_api_feed(); one reply per accepted packet. */
size_t joy_api_take_reply(JoyApi *j, char *out, size_t cap);

#endif
