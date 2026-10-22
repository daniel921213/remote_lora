/* Standalone joy_api test for a NUCLEO-L476RG with no motors attached.
 *
 * UART4 (PA0 = TX, PA1 = RX): the RYLR998, or a 3.3 V USB-UART adapter from the laptop.
 * USART2 (ST-LINK virtual COM port): readable log for a PC terminal at 115200.
 * LD2 (green LED) is lit while the handheld link is up.
 * B1 (blue button) stands in for "Orin takes over" while the handheld is released.
 *
 * The indicator rules here are only a stand-in so the handheld rumble can be tried;
 * the real main control decides them (spec section 3.1). */
#include "rc_test_app.h"
#include "main.h"
#include "joy_api.h"
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart4;

#define RING_SIZE 256u
#define STEP_MS 10u

static volatile uint8_t ring[RING_SIZE];
static volatile uint16_t ring_head, ring_tail;
static volatile bool ring_overflow;
static uint8_t rx_byte;

static JoyApi joy;
static const JoyConfig joy_config = {.peer_address = 1, .link_timeout_ms = 300, .deadband = 8,
                                     .drive_sign = 1, .turn_sign = 1};
static JoyIndicator indicator = JOY_IND_JOY;
static uint32_t last_step;
static bool shown, shown_link;
static int shown_drive, shown_turn;

static const char *const EVENT_NAMES[] = {
    "ENGINE_ON (START)", "ENGINE_OFF (BACK)", "GEAR_BLADE (X)", "GEAR_FAN (Y)",
    "GEAR_NEUTRAL (A)", "PARK (B)", "CONTROL_TOGGLE (RB)", "UNLOCK (LB)",
    "BLADE_UP (DPAD UP)", "BLADE_DOWN (DPAD DOWN)", "LINK_LOST", "LINK_RESTORED", "OVERFLOW"};
static const char *const INDICATOR_NAMES[] = {"ORIN", "JOY", "RELEASED", "FAULT"};
static const uint8_t STATE_FOR_INDICATOR[] = {3, 2, 2, 4}; /* Orin 0x81 state values */

static void log_text(const char *text) {
    HAL_UART_Transmit(&huart2, (uint8_t *)text, (uint16_t)strlen(text), 100);
}

static void start_rx(void) {
    (void)HAL_UART_Receive_IT(&huart4, &rx_byte, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart != &huart4) return;
    uint16_t next = (uint16_t)((ring_head + 1u) % RING_SIZE);
    if (next == ring_tail) {
        ring_overflow = true;
    } else {
        ring[ring_head] = rx_byte;
        ring_head = next;
    }
    start_rx();
}

/* An overrun or noise error stops interrupt reception; restart it so the link can recover. */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart4) start_rx();
}

static size_t ring_read(uint8_t *out, size_t cap) {
    size_t n = 0;
    while (n < cap && ring_tail != ring_head) {
        out[n++] = ring[ring_tail];
        ring_tail = (uint16_t)((ring_tail + 1u) % RING_SIZE);
    }
    return n;
}

static void set_indicator(JoyIndicator next) {
    char line[40];
    if (next == indicator) return;
    indicator = next;
    snprintf(line, sizeof line, "indicator -> %s\r\n", INDICATOR_NAMES[indicator]);
    log_text(line);
}

/* Stand-in main control: RB toggles JOY/RELEASED, link loss is a fault, LB clears it. */
static void stand_in_main_control(uint16_t events) {
    if (events & JOY_EV_LINK_LOST) {
        set_indicator(JOY_IND_FAULT);
    } else if (indicator == JOY_IND_FAULT) {
        if (events & JOY_EV_UNLOCK) set_indicator(JOY_IND_JOY);
    } else if (events & JOY_EV_CONTROL_TOGGLE) {
        set_indicator(indicator == JOY_IND_JOY ? JOY_IND_RELEASED : JOY_IND_JOY);
    } else if (indicator == JOY_IND_RELEASED && HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET) {
        set_indicator(JOY_IND_ORIN);
    }
    joy_api_set_status(&joy, indicator, STATE_FOR_INDICATOR[indicator]);
}

static int percent(float x) {
    return (int)(x * 100.0f + (x < 0 ? -0.5f : 0.5f));
}

void rc_test_init(void) {
    if (!joy_api_init(&joy, &joy_config)) log_text("joy_api config invalid\r\n");
    joy_api_set_status(&joy, indicator, STATE_FOR_INDICATOR[indicator]);
    log_text("\r\nrc_lora STM32 test: waiting for handheld packets on UART4 (PA0 TX, PA1 RX)\r\n");
    start_rx();
}

void rc_test_loop(void) {
    uint32_t now = HAL_GetTick();
    if (now - last_step < STEP_MS) return;
    last_step = now;

    uint8_t rx[64];
    size_t n;
    do {
        n = ring_read(rx, sizeof rx);
        joy_api_feed(&joy, rx, n, now);
    } while (n == sizeof rx);
    if (ring_overflow) {
        ring_overflow = false;
        joy_api_report_overflow(&joy);
    }

    char reply[JOY_REPLY_MAX];
    size_t m = joy_api_take_reply(&joy, reply, sizeof reply);
    if (m) HAL_UART_Transmit(&huart4, (uint8_t *)reply, (uint16_t)m, 20);

    JoyOutput rc;
    joy_api_poll(&joy, now, &rc);
    for (unsigned i = 0; i < sizeof EVENT_NAMES / sizeof EVENT_NAMES[0]; i++) {
        if (rc.events & (1u << i)) {
            char line[48];
            snprintf(line, sizeof line, "event: %s\r\n", EVENT_NAMES[i]);
            log_text(line);
        }
    }
    stand_in_main_control(rc.events);

    int drive = percent(rc.drive), turn = percent(rc.turn);
    if (!shown || rc.link_ok != shown_link || drive != shown_drive || turn != shown_turn) {
        char line[64];
        snprintf(line, sizeof line, "link %s  drive %+d%%  turn %+d%%  rssi %d\r\n",
                 rc.link_ok ? "OK" : "LOST", drive, turn, rc.rssi);
        log_text(line);
        shown = true;
        shown_link = rc.link_ok;
        shown_drive = drive;
        shown_turn = turn;
    }
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, rc.link_ok ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
