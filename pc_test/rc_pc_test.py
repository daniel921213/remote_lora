"""Test the RC LoRa link on a Windows PC.

The handheld side is this script (what the handheld MCU will do): it reads the F710
through XInput (X mode), sends packets and rumbles on the car's replies.

Three ways to connect it:
  python rc_pc_test.py                              radio and car both simulated
  python rc_pc_test.py --port COM7 --car-port COM8  real transparent LoRa; car simulated
                                                    on COM8 with the real joy_api code
  python rc_pc_test.py --port COM7                  real LoRa or wire to a real STM32

Add --demo to replace the F710 with scripted presses, and --loss 0.3 (simulated radio
only) to drop packets.
"""

import argparse
import ctypes
import random
import subprocess
import sys
import time
from ctypes import wintypes
from pathlib import Path

HERE = Path(__file__).resolve().parent
SRC = HERE.parent / 'src'
DLL_PATH = HERE / 'joy_sim.dll'
DLL_SOURCES = [SRC / 'rc_proto.c', SRC / 'joy_api.c', HERE / 'joy_sim.c']
DLL_HEADERS = [SRC / 'rc_proto.h', SRC / 'joy_api.h']

HANDHELD_ADDRESS = 1
CAR_ADDRESS = 2
BAUD = 115200
SEND_PERIOD = 0.10    # handheld sends at 10 Hz
SAMPLE_PERIOD = 0.01  # gamepad sampled faster, so short presses are not missed
LINK_TIMEOUT = 0.30   # both ends
DEADBAND = 8

# Counter slots in wire order (RC_BTN_* in rc_proto.h): name, XInput bit, function.
SLOTS = [
    ('START', 0x0010, '引擎通電'),
    ('BACK', 0x0020, '引擎斷電'),
    ('X', 0x4000, '刀具檔'),
    ('Y', 0x8000, '風機檔'),
    ('A', 0x1000, '離合空檔'),
    ('B', 0x2000, '停車＋空檔'),
    ('RB', 0x0200, 'JOY／ORIN 切換'),
    ('LB', 0x0100, '解除停止鎖定'),
    ('十字鍵上', 0x0001, '刀頭上升'),
    ('十字鍵下', 0x0002, '刀頭下降'),
]
EV_RB, EV_LB, EV_LINK_LOST = 1 << 6, 1 << 7, 1 << 10
LINK_EVENTS = [(1 << 10, '失聯 LINK_LOST'), (1 << 11, '連線恢復 LINK_RESTORED'), (1 << 12, '可能漏資料 OVERFLOW')]

IND_ORIN, IND_JOY, IND_RELEASED, IND_FAULT = 0, 1, 2, 3
IND_NAMES = {IND_ORIN: 'ORIN（Orin 控制）', IND_JOY: 'JOY（手把控制）',
             IND_RELEASED: '釋放等待', IND_FAULT: '異常／停止鎖定'}
STATE_FOR_IND = {IND_JOY: 2, IND_RELEASED: 2, IND_ORIN: 3, IND_FAULT: 4}

PULSE = [(0.5, True), (0.3, False)]  # 0.2 s pulses were too faint to notice on the F710
RUMBLE = {IND_JOY: ('震 1 下', PULSE), IND_RELEASED: ('震 2 下', PULSE * 2),
          IND_ORIN: ('震 3 下', PULSE * 3), IND_FAULT: ('長震 1 秒', [(1.0, True)])}
LOST_RUMBLE = ('長震 1 秒（失聯，每 3 秒重複）', [(1.0, True)])


# ---- Protocol, written independently of the C code so the two cross-check ----

def crc16(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021 if crc & 0x8000 else crc << 1) & 0xFFFF
    return crc


def pack_input(seq, drive, turn, gamepad_ok, counters):
    frame = bytearray(16)
    frame[0], frame[1] = 0x01, 0x11
    frame[2:4] = seq.to_bytes(2, 'little')
    frame[4], frame[5] = drive & 0xFF, turn & 0xFF
    frame[6] = 1 if gamepad_ok else 0
    for i, count in enumerate(counters):
        frame[7 + i // 2] |= (count & 0x0F) << (4 if i % 2 else 0)
    frame[14:16] = crc16(frame[:14]).to_bytes(2, 'little')
    return bytes(frame)


def at_send(address, frame):
    return f'AT+SEND={address},{2 * len(frame)},{frame.hex().upper()}'


def parse_reply_line(line):
    """The car's status reply -> dict, or None.

    Accepts '+RCV=2,16,<hex>,rssi,snr' (printed by a module in AT mode) and
    'AT+SEND=1,16,<hex>' (the STM32's own line, passed through a transparent module).
    """
    line = line.strip()
    if line.startswith('+RCV='):
        parts, sender = line[5:].split(','), CAR_ADDRESS
        if len(parts) != 5:
            return None
    elif line.startswith('AT+SEND='):
        parts, sender = line[8:].split(','), HANDHELD_ADDRESS  # addressed to the handheld
        if len(parts) != 3:
            return None
    else:
        return None
    try:
        address, length, payload = int(parts[0]), int(parts[1]), parts[2]
        frame = bytes.fromhex(payload)
    except ValueError:
        return None
    if (address != sender or length != len(payload) or len(frame) != 8 or frame[:2] != b'\x01\x91'
            or int.from_bytes(frame[6:8], 'little') != crc16(frame[:6]) or frame[4] > 3):
        return None
    return {'ack_seq': int.from_bytes(frame[2:4], 'little'), 'indicator': frame[4], 'state': frame[5]}


def axis(value):
    return max(-100, min(100, round(value * 100 / 32767)))


def open_serial(name):
    try:
        import serial
    except ImportError:
        sys.exit('需要 pyserial：pip install pyserial')
    port = serial.Serial()
    port.port, port.baudrate, port.timeout = name, BAUD, 0
    port.dtr = port.rts = False  # do not drive DTR/RTS; they can hold a module in reset
    try:
        port.open()
    except serial.SerialException as error:
        sys.exit(f'無法開啟 {name}：{error}\n（終端機或其他程式是否正在使用這個 COM？）')
    port.reset_input_buffer()
    return port


# ---- Gamepad ----

class XInputGamepad(ctypes.Structure):
    _fields_ = [('wButtons', wintypes.WORD), ('bLeftTrigger', ctypes.c_ubyte),
                ('bRightTrigger', ctypes.c_ubyte), ('sThumbLX', ctypes.c_short),
                ('sThumbLY', ctypes.c_short), ('sThumbRX', ctypes.c_short), ('sThumbRY', ctypes.c_short)]


class XInputState(ctypes.Structure):
    _fields_ = [('dwPacketNumber', wintypes.DWORD), ('Gamepad', XInputGamepad)]


class XInputVibration(ctypes.Structure):
    _fields_ = [('wLeftMotorSpeed', wintypes.WORD), ('wRightMotorSpeed', wintypes.WORD)]


class F710:
    """Returns (buttons, left_y, right_x) or None when no gamepad is connected."""

    def __init__(self, index):
        self.lib = ctypes.windll.xinput1_4
        self.index = index

    def read(self):
        state = XInputState()
        if self.lib.XInputGetState(self.index, ctypes.byref(state)) != 0:
            return None
        pad = state.Gamepad
        return pad.wButtons, pad.sThumbLY, pad.sThumbRX

    def vibrate(self, on):
        speed = 65535 if on else 0  # full strength
        self.lib.XInputSetState(self.index, ctypes.byref(XInputVibration(speed, speed)))


class DemoGamepad:
    """Scripted presses for --demo. Keys are simulated keyboard input."""

    def __init__(self):
        self.bit = {name: b for name, b, _ in SLOTS}
        self.presses = [(0.5, 'START'), (3.0, 'RB'), (4.0, 'RB'), (4.6, 'Y'), (4.9, 'X'),
                        (5.2, 'A'), (5.5, 'B'), (5.8, '十字鍵上'), (6.1, '十字鍵下'), (8.6, 'LB')]
        self.keys = [(3.5, 'o'), (7.0, 'l'), (8.0, 'l'), (10.5, 'q')]
        self.start = None

    def elapsed(self):
        if self.start is None:
            self.start = time.monotonic()  # the timeline starts with the main loop
        return time.monotonic() - self.start

    def read(self):
        t = self.elapsed()
        if 9.3 <= t < 9.8:
            return None  # F710 switched off
        buttons = sum(self.bit[name] for at, name in self.presses if at <= t < at + 0.15)
        left_y = 32767 if 1.0 <= t < 2.0 else 16000 if 2.0 <= t < 2.5 else 0
        right_x = -32767 if 1.5 <= t < 2.0 else 0
        return buttons, left_y, right_x

    def vibrate(self, on):
        pass

    def key(self):
        t = self.elapsed()
        if self.keys and self.keys[0][0] <= t:
            return self.keys.pop(0)[1]
        return None


def console_key():
    import msvcrt
    return msvcrt.getwch().lower() if msvcrt.kbhit() else None


# ---- Handheld side (what the MCU program will do) ----

class Handheld:
    def __init__(self, now):
        self.seq = 0
        self.counters = [0] * len(SLOTS)
        self.held = 0
        self.drive = self.turn = 0
        self.gamepad_ok = False
        self.link_up = False
        self.indicator = None
        self.last_reply = now
        self.alarm_at = now + LINK_TIMEOUT

    def sample(self, reading):
        """Counts fresh presses; returns the slot numbers pressed since the last sample."""
        if reading is None:
            self.gamepad_ok, self.held, self.drive, self.turn = False, 0, 0, 0
            return []
        buttons, left_y, right_x = reading
        pressed = []
        for i, (_, bit, _) in enumerate(SLOTS):
            if buttons & bit and not self.held & bit:
                self.counters[i] = (self.counters[i] + 1) & 0x0F
                pressed.append(i)
        self.held = buttons
        self.gamepad_ok = True
        self.drive = axis(left_y)   # XInput: up is positive
        self.turn = axis(right_x)   # right is positive
        return pressed

    def next_frame(self):
        self.seq = (self.seq + 1) & 0xFFFF
        return pack_input(self.seq, self.drive, self.turn, self.gamepad_ok, self.counters)


class Rumble:
    def __init__(self, gamepad):
        self.gamepad = gamepad
        self.steps = []
        self.on = False

    def play(self, pattern, now):
        self.steps = []
        for seconds, on in pattern:
            now += seconds
            self.steps.append((now, on))

    def update(self, now):
        while self.steps and now >= self.steps[0][0]:
            self.steps.pop(0)
        on = bool(self.steps) and self.steps[0][1]
        if on != self.on:
            self.gamepad.vibrate(on)
            self.on = on


# ---- Car side: the real joy_api plus a stand-in main control ----

class SimOutput(ctypes.Structure):
    _fields_ = [('link_ok', ctypes.c_int), ('sticks_neutral', ctypes.c_int), ('drive', ctypes.c_float),
                ('turn', ctypes.c_float), ('events', ctypes.c_uint), ('rssi', ctypes.c_int), ('snr', ctypes.c_int)]


def build_dll():
    newest = max(p.stat().st_mtime for p in DLL_SOURCES + DLL_HEADERS)
    if DLL_PATH.exists() and DLL_PATH.stat().st_mtime >= newest:
        return
    command = ['gcc', '-shared', '-std=c99', '-O2', '-Wall', '-Wextra', '-Werror', '-static-libgcc',
               f'-I{SRC}', *map(str, DLL_SOURCES), '-o', str(DLL_PATH)]
    print('編譯 joy_api …')
    try:
        subprocess.run(command, check=True)
    except FileNotFoundError:
        sys.exit('找不到 gcc。請安裝 MinGW-w64，或使用資料夾內已編譯好的 joy_sim.dll。')


class Car:
    """The real joy_api C code, loaded from joy_sim.dll."""

    def __init__(self):
        build_dll()
        dll = ctypes.CDLL(str(DLL_PATH))
        dll.sim_new.argtypes = [ctypes.c_uint] * 3
        dll.sim_new.restype = ctypes.c_void_p
        dll.sim_feed.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_uint, ctypes.c_uint]
        dll.sim_take_reply.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_char), ctypes.c_uint]
        dll.sim_take_reply.restype = ctypes.c_uint
        dll.sim_set_status.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int]
        dll.sim_poll.argtypes = [ctypes.c_void_p, ctypes.c_uint, ctypes.POINTER(SimOutput)]
        self.dll = dll
        self.handle = dll.sim_new(HANDHELD_ADDRESS, int(LINK_TIMEOUT * 1000), DEADBAND)

    def receive(self, data, now_ms):
        self.dll.sim_feed(self.handle, data, len(data), now_ms)

    def take_reply(self):
        buffer = ctypes.create_string_buffer(64)
        n = self.dll.sim_take_reply(self.handle, buffer, len(buffer))
        return buffer.raw[:n] if n else None

    def poll(self, now_ms):
        out = SimOutput()
        self.dll.sim_poll(self.handle, now_ms, ctypes.byref(out))
        return out

    def set_status(self, indicator, state):
        self.dll.sim_set_status(self.handle, indicator, state)


class FakeMainControl:
    """Stand-in for the STM32 main control, only so the rumble can be tried.
    The real JOY/ORIN rules (spec section 3.1) are still undecided."""

    def __init__(self):
        self.indicator = IND_JOY

    def on_events(self, events):
        if events & EV_LINK_LOST:
            self.indicator = IND_FAULT  # spec 3.1: RC loss -> ESTOP
        elif self.indicator == IND_FAULT:
            if events & EV_LB:
                self.indicator = IND_JOY
        elif events & EV_RB:
            self.indicator = IND_RELEASED if self.indicator == IND_JOY else IND_JOY


class SerialCar:
    """Feeds the simulated car from a real port, as the STM32 UART would be."""

    def __init__(self, car, port_name):
        self.car = car
        self.port = open_serial(port_name)

    def pump(self, now_ms):
        data = self.port.read(self.port.in_waiting or 1)
        if data:
            self.car.receive(data, now_ms)
        reply = self.car.take_reply()
        if reply:
            self.port.write(reply)


# ---- Links seen from the handheld ----

class Radio:
    """Simulated radio: turns the sender's AT+SEND into the receiver's +RCV, or loses it."""

    def __init__(self, loss):
        self.loss = loss
        self.up = True

    def carry(self, at_line, from_address):
        if not self.up or random.random() < self.loss:
            return None
        _, length, payload = at_line.strip().split('=', 1)[1].split(',')
        return f'+RCV={from_address},{length},{payload},-45,10\r\n'


class SimLink:
    """Radio and car both inside this process."""

    def __init__(self, car, loss):
        self.car = car
        self.radio = Radio(loss)
        self.replies = []

    def send(self, frame, now_ms):
        line = at_send(CAR_ADDRESS, frame)
        received = self.radio.carry(line, HANDHELD_ADDRESS)
        if received:
            self.car.receive(received.encode('ascii'), now_ms)
        reply = self.car.take_reply()
        if reply:
            back = self.radio.carry(reply.decode('ascii'), CAR_ADDRESS)
            if back:
                self.replies.append(back)
        return line

    def receive(self):
        statuses = [parse_reply_line(line) for line in self.replies]
        self.replies = []
        return [s for s in statuses if s]


class SerialLink:
    """Handheld on a real port: a transparent LoRa module, or a wire to the STM32 UART.
    Sends the exact line joy_api expects (RC_LORA_PROTOCOL.md section 2.4)."""

    def __init__(self, port_name):
        self.port = open_serial(port_name)
        self.buffer = b''

    def send(self, frame, now_ms):
        line = f'+RCV={HANDHELD_ADDRESS},{2 * len(frame)},{frame.hex().upper()},0,0'
        self.port.write((line + '\r\n').encode('ascii'))
        return line

    def receive(self):
        self.buffer += self.port.read(self.port.in_waiting or 1)
        statuses = []
        while b'\n' in self.buffer:
            raw, self.buffer = self.buffer.split(b'\n', 1)
            status = parse_reply_line(raw.decode('ascii', 'replace'))
            if status:
                statuses.append(status)
        if len(self.buffer) > 256:
            self.buffer = b''  # noise without a line end
        return statuses


# ---- Main loop ----

def run(gamepad, key_source, link, car, serial_car):
    """car is None when a real STM32 answers at the far end."""
    main = FakeMainControl() if car else None
    if car:
        # A real STM32 main control sets this every cycle from boot, before any packet arrives.
        car.set_status(main.indicator, STATE_FOR_IND[main.indicator])
    rumble = Rumble(gamepad)
    start = time.monotonic()
    handheld = Handheld(start)
    next_send = start
    shown_packet = shown_car = None

    def log(text):
        print(f'[{time.monotonic() - start:5.1f}s] {text}', flush=True)

    keys = ['q = 結束']
    if isinstance(link, SimLink):
        keys.insert(0, 'l = 切斷／恢復 LoRa')
    if car:
        keys[:0] = ['o = 模擬 Orin 接手', 'f = 模擬故障']
    print('按鍵：' + '、'.join(keys))
    if isinstance(gamepad, F710) and gamepad.read() is None:
        print('⚠ 找不到手把：F710 要切到 X 模式，並插上接收器。程式會繼續執行。')
    try:
        while True:
            now = time.monotonic()
            now_ms = int((now - start) * 1000) & 0xFFFFFFFF

            for slot in handheld.sample(gamepad.read()):
                name, _, function = SLOTS[slot]
                log(f'按下 {name}（{function}），計數 → {handheld.counters[slot]}')

            key = key_source()
            if key == 'q':
                break
            if key == 'l':
                if isinstance(link, SimLink):
                    link.radio.up = not link.radio.up
                    log('LoRa 恢復' if link.radio.up else 'LoRa 切斷（模擬斷訊）')
                else:
                    log('真實 LoRa：要測斷訊，請拔掉一顆模組的電源')
            elif key == 'o' and main and main.indicator == IND_RELEASED:
                main.indicator = IND_ORIN
                log('模擬 Orin 接手')
            elif key == 'f' and main:
                main.indicator = IND_FAULT
                log('模擬故障')

            if now >= next_send:
                next_send = max(next_send + SEND_PERIOD, now)
                line = link.send(handheld.next_frame(), now_ms)
                content = (handheld.drive, handheld.turn, handheld.gamepad_ok, tuple(handheld.counters))
                if content != shown_packet:
                    log(f'手把送出 {line}（前後 {handheld.drive:+d}、轉向 {handheld.turn:+d}'
                        f'{"" if handheld.gamepad_ok else "、手把斷線"}）')
                    shown_packet = content

            if serial_car:
                serial_car.pump(now_ms)
            if car:
                out = car.poll(now_ms)
                for slot, (name, _, function) in enumerate(SLOTS):
                    if out.events & (1 << slot):
                        log(f'車子事件 {function}（{name}）')
                for bit, text in LINK_EVENTS:
                    if out.events & bit:
                        log(f'車子事件 {text}')
                if out.events:
                    before = main.indicator
                    main.on_events(out.events)
                    if main.indicator != before:
                        log(f'主控（模擬）狀態 → {IND_NAMES[main.indicator]}')
                car.set_status(main.indicator, STATE_FOR_IND[main.indicator])
                seen = (bool(out.link_ok), round(out.drive, 2), round(out.turn, 2))
                if seen != shown_car:
                    log(f'車子收到 連線 {"正常" if out.link_ok else "失聯"}、'
                        f'前後 {out.drive:+.2f}、轉向 {out.turn:+.2f}')
                    shown_car = seen

            statuses = link.receive()
            if statuses:
                indicator = statuses[-1]['indicator']
                if not handheld.link_up or indicator != handheld.indicator:
                    text, pattern = RUMBLE[indicator]
                    log(f'手把收到 狀態 {IND_NAMES[indicator]} → 震動：{text}')
                    rumble.play(pattern, now)
                handheld.link_up, handheld.indicator, handheld.last_reply = True, indicator, now
            elif now - handheld.last_reply > LINK_TIMEOUT:
                if handheld.link_up:
                    handheld.link_up = False
                    handheld.alarm_at = now
                if now >= handheld.alarm_at:
                    log(f'手把 {LINK_TIMEOUT:.1f} 秒沒收到回覆 → 震動：{LOST_RUMBLE[0]}')
                    rumble.play(LOST_RUMBLE[1], now)
                    handheld.alarm_at = now + 3.0
            rumble.update(now)

            time.sleep(max(0.0, SAMPLE_PERIOD - (time.monotonic() - now)))
    except KeyboardInterrupt:
        pass
    finally:
        gamepad.vibrate(False)


def main():
    parser = argparse.ArgumentParser(description='遙控器 LoRa 鏈路電腦測試')
    parser.add_argument('--port', help='手把端用的 COM，例如 COM7（透傳 LoRa，或直接接線到 STM32）')
    parser.add_argument('--car-port', help='在電腦上模擬車子，用這個 COM 收發，例如 COM8')
    parser.add_argument('--demo', action='store_true', help='不接手把，自動跑一段示範')
    parser.add_argument('--loss', type=float, default=0.0, help='模擬無線的掉包率，0～0.9（只用在模擬模式）')
    parser.add_argument('--pad', type=int, default=0, help='XInput 手把編號 0～3')
    args = parser.parse_args()
    if sys.platform != 'win32':
        sys.exit('這支程式只支援 Windows（使用 XInput 讀 F710）。')
    if not 0.0 <= args.loss <= 0.9:
        parser.error('--loss 要在 0～0.9 之間')
    if args.car_port and not args.port:
        parser.error('--car-port 要搭配 --port 一起使用')
    if args.port and args.loss:
        parser.error('--loss 只能用在模擬模式；真實 LoRa 的掉包要實測')
    if hasattr(sys.stdout, 'reconfigure'):
        sys.stdout.reconfigure(encoding='utf-8')

    # The Python packer must match the C one byte for byte (value from rc_proto.c).
    assert pack_input(1, 0, 0, True, [0] * 10).hex().upper() == '01110100000001000000000000007AC5'

    if args.port:
        car = Car() if args.car_port else None
        link = SerialLink(args.port)
        serial_car = SerialCar(car, args.car_port) if args.car_port else None
        where = f'模擬的車子在 {args.car_port}' if args.car_port else '對面是真的 STM32'
        print(f'手把端：{args.port}；{where}')
    else:
        car = Car()
        link = SimLink(car, args.loss)
        serial_car = None
        print('全部模擬：無線和車子都在電腦裡')

    if args.demo:
        demo = DemoGamepad()
        run(demo, demo.key, link, car, serial_car)
    else:
        run(F710(args.pad), console_key, link, car, serial_car)


if __name__ == '__main__':
    main()
