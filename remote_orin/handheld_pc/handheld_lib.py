"""The C handheld logic (common/rc_handheld.c) for Python, through ctypes.

The laptop stand-in and the tests both use this, so they run the code the RP2040 will run.
"""

import ctypes
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / 'common'))
import tools  # noqa: E402

# Button slots in wire order (RC_BTN_* in rc_proto.h): name, function.
SLOTS = [
    ('START', '引擎通電'), ('BACK', '引擎斷電'), ('X', '刀具檔'), ('Y', '風機檔'), ('A', '離合空檔'),
    ('B', '停車'), ('RB', 'JOY／ORIN 切換'), ('LB', '解除停止鎖定'), ('十字鍵上', '刀頭上升'),
    ('十字鍵下', '刀頭下降'),
]
SLOT = {name: i for i, (name, _) in enumerate(SLOTS)}

EV_LINK_UP, EV_LINK_LOST, EV_INDICATOR, EV_ALARM = 1, 2, 4, 8

INDICATOR_NAMES = {0: 'ORIN（自動）', 1: 'JOY（手動）', 2: '等待 STM32 進 AUTO（按 R12 的 A）', 3: '異常／停止鎖定'}
RUMBLE_TEXT = {0: '震 3 下', 1: '震 1 下', 2: '震 2 下', 3: '長震 1 秒'}
STATE_NAMES = {0: '未設定', 1: '停止', 2: 'MANUAL', 3: 'AUTO', 4: 'ESTOP', 5: 'SBUS 測試'}


class HhInfo(ctypes.Structure):
    _fields_ = [('link_up', ctypes.c_int), ('indicator', ctypes.c_int), ('state', ctypes.c_int),
                ('seq', ctypes.c_uint), ('replies', ctypes.c_uint), ('counters', ctypes.c_uint * len(SLOTS))]


def _load():
    common = tools.COMMON
    dll = tools.load_library('handheld', [common / 'rc_proto.c', common / 'rc_handheld.c', HERE / 'hh_lib.c'],
                             HERE / 'build')
    dll.hh_new.argtypes = [ctypes.c_uint]
    dll.hh_new.restype = ctypes.c_void_p
    dll.hh_free.argtypes = [ctypes.c_void_p]
    dll.hh_sample.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_uint]
    dll.hh_sample.restype = ctypes.c_uint
    dll.hh_take_line.argtypes = [ctypes.c_void_p, ctypes.c_uint, ctypes.c_char_p, ctypes.c_uint]
    dll.hh_take_line.restype = ctypes.c_uint
    dll.hh_feed.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_uint, ctypes.c_uint]
    dll.hh_poll.argtypes = [ctypes.c_void_p, ctypes.c_uint]
    dll.hh_poll.restype = ctypes.c_uint
    dll.hh_rumble.argtypes = [ctypes.c_void_p, ctypes.c_uint]
    dll.hh_rumble.restype = ctypes.c_int
    dll.hh_info.argtypes = [ctypes.c_void_p, ctypes.POINTER(HhInfo)]
    return dll


class Handheld:
    """One handheld instance. Times are uint32 milliseconds from any monotonic origin."""

    _dll = None

    def __init__(self, now_ms):
        if Handheld._dll is None:
            Handheld._dll = _load()
        self.dll = Handheld._dll
        self.handle = self.dll.hh_new(now_ms & 0xFFFFFFFF)
        if not self.handle:
            raise MemoryError('hh_new failed')

    def sample(self, connected, drive, turn, buttons):
        """drive/turn -100..100; buttons: bit SLOT[name] set while held. Returns newly pressed bits."""
        return self.dll.hh_sample(self.handle, int(connected), int(drive), int(turn), buttons)

    def take_line(self, now_ms):
        buffer = ctypes.create_string_buffer(64)
        n = self.dll.hh_take_line(self.handle, now_ms & 0xFFFFFFFF, buffer, len(buffer))
        return buffer.raw[:n] if n else None

    def feed(self, data, now_ms):
        if data:
            self.dll.hh_feed(self.handle, data, len(data), now_ms & 0xFFFFFFFF)

    def poll(self, now_ms):
        return self.dll.hh_poll(self.handle, now_ms & 0xFFFFFFFF)

    def rumble(self, now_ms):
        return bool(self.dll.hh_rumble(self.handle, now_ms & 0xFFFFFFFF))

    def info(self):
        out = HhInfo()
        self.dll.hh_info(self.handle, ctypes.byref(out))
        return out

    def close(self):
        if self.handle:
            self.dll.hh_free(self.handle)
            self.handle = None
