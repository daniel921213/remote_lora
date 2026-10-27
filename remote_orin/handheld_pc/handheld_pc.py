"""Laptop stand-in for the handheld MCU: F710 -> RYLR998 (transparent) -> Orin.

  python handheld_pc.py --port COM7           F710 on this laptop (switch on the back set to X)
  python handheld_pc.py --port COM7 --demo    scripted presses, no F710 needed

Runs the same C handheld logic as the Feather RP2040 (common/rc_handheld.c), so a test here
also tests the future MCU program. The F710 rumbles when the Orin reports a new state.
"""

import argparse
import ctypes
import sys
import time
from ctypes import wintypes

from handheld_lib import (EV_ALARM, EV_INDICATOR, EV_LINK_LOST, EV_LINK_UP, INDICATOR_NAMES, RUMBLE_TEXT,
                          SLOT, SLOTS, STATE_NAMES, Handheld, tools)

SAMPLE_PERIOD = 0.01  # gamepad sampled faster than the 10 Hz send rate, so short presses count

# XInput button bit for each slot name.
XINPUT_BITS = {'START': 0x0010, 'BACK': 0x0020, 'X': 0x4000, 'Y': 0x8000, 'A': 0x1000, 'B': 0x2000,
               'RB': 0x0200, 'LB': 0x0100, '十字鍵上': 0x0001, '十字鍵下': 0x0002}


def axis(value):
    return max(-100, min(100, round(value * 100 / 32767)))


class XInputGamepad(ctypes.Structure):
    _fields_ = [('wButtons', wintypes.WORD), ('bLeftTrigger', ctypes.c_ubyte),
                ('bRightTrigger', ctypes.c_ubyte), ('sThumbLX', ctypes.c_short),
                ('sThumbLY', ctypes.c_short), ('sThumbRX', ctypes.c_short), ('sThumbRY', ctypes.c_short)]


class XInputState(ctypes.Structure):
    _fields_ = [('dwPacketNumber', wintypes.DWORD), ('Gamepad', XInputGamepad)]


class XInputVibration(ctypes.Structure):
    _fields_ = [('wLeftMotorSpeed', wintypes.WORD), ('wRightMotorSpeed', wintypes.WORD)]


class F710:
    """read() -> (connected, drive, turn, buttons) in handheld terms."""

    def __init__(self, index):
        self.lib = ctypes.windll.xinput1_4
        self.index = index

    def read(self):
        state = XInputState()
        if self.lib.XInputGetState(self.index, ctypes.byref(state)) != 0:
            return False, 0, 0, 0
        pad = state.Gamepad
        buttons = sum(1 << SLOT[name] for name, bit in XINPUT_BITS.items() if pad.wButtons & bit)
        return True, axis(pad.sThumbLY), axis(pad.sThumbRX), buttons  # XInput: up and right positive

    def vibrate(self, on):
        speed = 65535 if on else 0  # full strength
        self.lib.XInputSetState(self.index, ctypes.byref(XInputVibration(speed, speed)))


class DemoGamepad:
    """Scripted operator for --demo. Pairs with: orin_bridge.py --fake-stm32 --fake-auto."""

    PRESSES = [(4.0, 'RB'), (6.0, 'RB'), (7.2, 'X'), (8.0, 'B'), (11.0, 'BACK')]
    STICKS = [(1.5, 2.5, 60, 0), (2.5, 3.0, 60, 50), (6.5, 8.5, -50, 0), (9.0, 9.5, 40, 0)]
    GONE = (10.0, 10.6)  # F710 switched off
    END = 13.0

    def __init__(self):
        self.start = None

    def elapsed(self):
        if self.start is None:
            self.start = time.monotonic()  # the timeline starts with the main loop
        return time.monotonic() - self.start

    def read(self):
        t = self.elapsed()
        if self.GONE[0] <= t < self.GONE[1]:
            return False, 0, 0, 0
        buttons = sum(1 << SLOT[name] for at, name in self.PRESSES if at <= t < at + 0.15)
        drive = turn = 0
        for begin, end, d, r in self.STICKS:
            if begin <= t < end:
                drive, turn = d, r
        return True, drive, turn, buttons

    def vibrate(self, on):
        pass

    def key(self):
        return 'q' if self.elapsed() >= self.END else None


class Session:
    """The handheld loop shared by the console (run) and the panel (handheld_gui.py).
    Call step() about every 10 ms; log receives ready-to-show lines."""

    def __init__(self, gamepad, port, log):
        self.gamepad, self.port, self.log = gamepad, port, log
        self.start = time.monotonic()
        self.handheld = Handheld(0)
        self.reading = (False, 0, 0, 0)  # (connected, drive, turn, buttons) from the last step
        self.info = self.handheld.info()
        self.presses = [0] * len(SLOTS)  # per slot since start; the wire counters wrap at 16
        self.sent = 0
        self.last_reply_at = None
        self.rumbling = False
        self.shown_send = self.shown_state = None

    def emit(self, text):
        self.log(f'[{time.monotonic() - self.start:5.1f}s] {text}')

    def step(self):
        now = time.monotonic()
        now_ms = int((now - self.start) * 1000)

        reading = self.reading = self.gamepad.read()
        pressed = self.handheld.sample(*reading)
        for slot, (name, function) in enumerate(SLOTS):
            if pressed & (1 << slot):
                self.presses[slot] += 1
                self.emit(f'按下 {name}（{function}）')

        line = self.handheld.take_line(now_ms)
        if line:
            self.port.write(line)
            self.sent += 1
            connected, drive, turn, _ = reading
            if (connected, drive, turn) != self.shown_send:
                self.emit(f'送出 前後 {drive:+d}、轉向 {turn:+d}' if connected else '送出 手把斷線（gamepad_ok=0）')
                self.shown_send = (connected, drive, turn)

        self.handheld.feed(self.port.read(self.port.in_waiting or 1), now_ms)
        events = self.handheld.poll(now_ms)
        replies_before = self.info.replies
        info = self.info = self.handheld.info()
        if info.replies != replies_before:
            self.last_reply_at = now
        if events & EV_LINK_UP:
            self.emit(f'連上 Orin：{INDICATOR_NAMES[info.indicator]}，STM32 {STATE_NAMES.get(info.state, info.state)}'
                      f' → {RUMBLE_TEXT[info.indicator]}')
        elif events & EV_INDICATOR:
            self.emit(f'狀態 → {INDICATOR_NAMES[info.indicator]} → {RUMBLE_TEXT[info.indicator]}')
        if events & EV_LINK_LOST:
            self.emit('0.3 秒沒收到 Orin 回覆：失聯')
        if events & EV_ALARM:
            self.emit('→ 長震 1 秒（沒連上，每 3 秒重複一次）')
        if info.link_up and info.state != self.shown_state:
            if self.shown_state is not None:
                self.emit(f'STM32 狀態 → {STATE_NAMES.get(info.state, info.state)}')
            self.shown_state = info.state

        on = self.handheld.rumble(now_ms)
        if on != self.rumbling:
            self.gamepad.vibrate(on)
            self.rumbling = on

    def close(self):
        self.gamepad.vibrate(False)
        self.handheld.close()


def run(gamepad, key, port):
    session = Session(gamepad, port, lambda line: print(line, flush=True))
    try:
        while True:
            began = time.monotonic()
            session.step()
            if key() == 'q':
                break
            time.sleep(max(0.0, SAMPLE_PERIOD - (time.monotonic() - began)))
    except KeyboardInterrupt:
        pass
    finally:
        session.close()


def main():
    parser = argparse.ArgumentParser(description='筆電假裝手把 MCU：F710 → LoRa → Orin')
    parser.add_argument('--port', required=True, help='手把那顆 LoRa 模組的 COM，例如 COM7')
    parser.add_argument('--demo', action='store_true', help='不接 F710，自動跑一段示範操作')
    parser.add_argument('--pad', type=int, default=0, help='XInput 手把編號 0～3')
    args = parser.parse_args()
    if sys.platform != 'win32':
        sys.exit('這支程式只支援 Windows（用 XInput 讀 F710）。')
    tools.utf8_console()

    port = tools.open_serial(args.port)
    print(f'手把端：{args.port}，每 0.1 秒送一筆。按 q 結束。')
    try:
        if args.demo:
            demo = DemoGamepad()
            run(demo, demo.key, port)
        else:
            gamepad = F710(args.pad)
            if not gamepad.read()[0]:
                print('⚠ 找不到 F710：背面開關要切到 X，並插上接收器。程式會繼續執行。')
            keys = tools.Keys()
            try:
                run(gamepad, keys.get, port)
            finally:
                keys.close()
    finally:
        port.close()


if __name__ == '__main__':
    main()
