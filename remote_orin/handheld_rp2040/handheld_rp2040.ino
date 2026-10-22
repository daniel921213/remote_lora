// Handheld MCU: F710 receiver on the USB-A host port -> RYLR998 on TX/RX -> Orin.
//
// Same handheld logic as the laptop stand-in (common/rc_handheld.c, copied into src/), so the
// laptop test already covered framing, press counting, link timeout and rumble timing. This
// file is only the hardware glue.
//
// Board:   Adafruit Feather RP2040 USB Host, arduino-pico core (Earle Philhower)
// Tools:   CPU Speed 120 MHz, USB Stack "Adafruit TinyUSB"
// Library: "Pico PIO USB" (Library Manager)
// Wiring:  RYLR998 TXD -> RX (GPIO1), RXD <- TX (GPIO0), 3V3, GND; module in transparent mode, 115200
// F710:    switch on the back set to D, MODE light off
//
// NOT YET RUN ON HARDWARE. The F710 report layout and rumble command come from the Logitech
// DirectInput layout and the Linux hid-lg2ff driver: check them first with DEBUG_REPORTS 1.

#include "pio_usb.h"
#include "Adafruit_TinyUSB.h"
#include <pico/mutex.h>

extern "C" {
#include "src/rc_handheld.h"
}

#ifndef PIN_5V_EN_STATE
#define PIN_5V_EN_STATE 1  // USB-A power switch is active high (same default as Adafruit's usbh_helper.h)
#endif

#define DEBUG_REPORTS 0  // 1: print every raw F710 report on the USB serial port
#define LORA Serial1     // TX = GPIO0, RX = GPIO1
#define SAMPLE_MS 10

static const uint16_t F710_VID = 0x046d;
static const uint16_t F710_PID_D_MODE = 0xc219;

Adafruit_USBH_Host USBHost;

// Shared between core 1 (USB host) and core 0 (protocol), guarded by pad_mutex.
auto_init_mutex(pad_mutex);
static RcPad shared_pad;            // latest F710 state
static volatile bool want_rumble;   // written by core 0, applied by core 1

// Core 1 only.
static bool pad_mounted;
static uint8_t pad_addr, pad_instance;

// Core 0 only.
static RcHandheld hh;

static void set_pad(const RcPad &pad) {
  mutex_enter_blocking(&pad_mutex);
  shared_pad = pad;
  mutex_exit(&pad_mutex);
}

// 0..255 with ~128 at rest -> -100..100.
static int8_t axis(uint8_t raw, bool invert) {
  int v = invert ? 128 - (int)raw : (int)raw - 128;
  v = v * 100 / 128;
  return (int8_t)(v > 100 ? 100 : v < -100 ? -100 : v);
}

// Logitech DirectInput layout (F310/F710 "D" mode), report bytes:
// [0] left X  [1] left Y  [2] right X  [3] right Y   (Y: up = 0, down = 255)
// [4] low nibble: hat 0 up, 1 up-right, 2 right, 3 down-right, 4 down, 5 down-left, 6 left,
//     7 up-left, 8 released; high nibble: 0x10 X, 0x20 A, 0x40 B, 0x80 Y
// [5] 0x01 LB, 0x02 RB, 0x04 LT, 0x08 RT, 0x10 BACK, 0x20 START, 0x40 L3, 0x80 R3
static bool parse_f710(const uint8_t *r, uint16_t len, RcPad *pad) {
  if (len < 6) return false;
  uint8_t hat = r[4] & 0x0f;
  uint16_t b = 0;
  if (r[5] & 0x20) b |= 1u << RC_BTN_START;
  if (r[5] & 0x10) b |= 1u << RC_BTN_BACK;
  if (r[4] & 0x10) b |= 1u << RC_BTN_X;
  if (r[4] & 0x80) b |= 1u << RC_BTN_Y;
  if (r[4] & 0x20) b |= 1u << RC_BTN_A;
  if (r[4] & 0x40) b |= 1u << RC_BTN_B;
  if (r[5] & 0x02) b |= 1u << RC_BTN_RB;
  if (r[5] & 0x01) b |= 1u << RC_BTN_LB;
  if (hat == 7 || hat == 0 || hat == 1) b |= 1u << RC_BTN_DPAD_UP;
  if (hat == 3 || hat == 4 || hat == 5) b |= 1u << RC_BTN_DPAD_DOWN;
  pad->connected = true;
  pad->drive = axis(r[1], true);   // left stick up = forward
  pad->turn = axis(r[2], false);   // right stick right = positive
  pad->buttons = b;
  return true;
}

// hid-lg2ff: output report 0x51 starts the motors (weak at [2], strong at [4]), 0xf3 stops them.
static bool send_rumble(bool on) {
  uint8_t report[7] = {(uint8_t)(on ? 0x51 : 0xf3), 0, (uint8_t)(on ? 0xff : 0), 0, (uint8_t)(on ? 0xff : 0), 0, 0};
  return tuh_hid_send_report(pad_addr, pad_instance, 0, report, sizeof report);
}

//------------- Core 0: protocol -------------//

void setup() {
  Serial.begin(115200);
  LORA.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  rc_hh_init(&hh, millis());
  Serial.println("handheld_rp2040: F710 -> RYLR998 (transparent, 115200)");
}

void loop() {
  static uint32_t last_sample;
  uint32_t now = millis();

  uint8_t rx[64];
  size_t n = 0;
  while (n < sizeof rx && LORA.available()) rx[n++] = (uint8_t)LORA.read();
  if (n) rc_hh_feed(&hh, rx, n, now);

  if (now - last_sample >= SAMPLE_MS) {
    last_sample = now;
    RcPad pad;
    mutex_enter_blocking(&pad_mutex);
    pad = shared_pad;
    mutex_exit(&pad_mutex);
    uint16_t pressed = rc_hh_sample(&hh, &pad);
    if (pressed) Serial.printf("pressed 0x%03x\r\n", pressed);
  }

  char line[RC_HH_SEND_MAX];
  size_t m = rc_hh_take_line(&hh, now, line, sizeof line);
  if (m) LORA.write((const uint8_t *)line, m);

  uint16_t events = rc_hh_poll(&hh, now);
  if (events & RC_HH_EV_LINK_UP) Serial.printf("link up: indicator %u, STM32 state %u\r\n", hh.indicator, hh.state);
  if (events & RC_HH_EV_INDICATOR) Serial.printf("indicator -> %u\r\n", hh.indicator);
  if (events & RC_HH_EV_LINK_LOST) Serial.println("link lost: no reply for 300 ms");

  want_rumble = rc_hh_rumble(&hh, now);
  digitalWrite(LED_BUILTIN, hh.link_up ? HIGH : LOW);
}

//------------- Core 1: USB host -------------//

void setup1() {
  uint32_t cpu_hz = clock_get_hz(clk_sys);
  if (cpu_hz % 12000000UL) {  // PIO USB needs a multiple of 12 MHz
    while (true) {
      Serial.println("Tools > CPU Speed must be 120 MHz");
      delay(1000);
    }
  }
#ifdef PIN_5V_EN
  pinMode(PIN_5V_EN, OUTPUT);
  digitalWrite(PIN_5V_EN, PIN_5V_EN_STATE);  // power the USB-A port
#endif
  pio_usb_configuration_t pio_cfg = PIO_USB_DEFAULT_CONFIG;
  pio_cfg.pin_dp = PIN_USB_HOST_DP;
  USBHost.configure_pio_usb(1, &pio_cfg);
  USBHost.begin(1);
}

void loop1() {
  static bool applied;
  USBHost.task();
  bool on = want_rumble;
  if (pad_mounted && on != applied && send_rumble(on)) applied = on;
}

extern "C" {

void tuh_hid_mount_cb(uint8_t dev_addr, uint8_t instance, uint8_t const *desc_report, uint16_t desc_len) {
  (void)desc_report;
  (void)desc_len;
  uint16_t vid, pid;
  tuh_vid_pid_get(dev_addr, &vid, &pid);
  Serial.printf("USB HID %04x:%04x mounted\r\n", vid, pid);
  if (vid != F710_VID || pid != F710_PID_D_MODE) {
    Serial.println("not an F710 in D mode: ignored (switch on the back to D?)");
    return;
  }
  pad_mounted = true;
  pad_addr = dev_addr;
  pad_instance = instance;
  tuh_hid_receive_report(dev_addr, instance);
}

void tuh_hid_umount_cb(uint8_t dev_addr, uint8_t instance) {
  if (!pad_mounted || dev_addr != pad_addr || instance != pad_instance) return;
  pad_mounted = false;
  RcPad gone = {};
  set_pad(gone);  // connected = false: the next frames tell the Orin the gamepad is gone
  Serial.println("F710 receiver removed");
}

void tuh_hid_report_received_cb(uint8_t dev_addr, uint8_t instance, uint8_t const *report, uint16_t len) {
  if (pad_mounted && dev_addr == pad_addr && instance == pad_instance) {
#if DEBUG_REPORTS
    for (uint16_t i = 0; i < len; i++) Serial.printf("%02X ", report[i]);
    Serial.println();
#endif
    RcPad pad;
    if (parse_f710(report, len, &pad)) set_pad(pad);
  }
  tuh_hid_receive_report(dev_addr, instance);
}

}  // extern "C"
