"""The receiver C code (common/joy_api.c) for Python, through ctypes.

Same code that was written for the STM32 and tested over real LoRa; on the Orin it decodes
the F710 packets, tracks the link and builds the status reply that makes the F710 rumble.
"""

import ctypes
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / 'common'))
import tools  # noqa: E402

HANDHELD_ADDRESS = 1  # must match RC_HH_ADDRESS in rc_handheld.h
LINK_TIMEOUT_MS = 300
DEADBAND = 8          # stick percent


class JoyOutput(ctypes.Structure):
    _fields_ = [('link_ok', ctypes.c_int), ('sticks_neutral', ctypes.c_int), ('drive', ctypes.c_float),
                ('turn', ctypes.c_float), ('events', ctypes.c_uint), ('rssi', ctypes.c_int), ('snr', ctypes.c_int)]


def _load():
    common = tools.COMMON
    dll = tools.load_library('joy', [common / 'rc_proto.c', common / 'joy_api.c', HERE / 'joy_lib.c'],
                             HERE / 'build')
    dll.joy_new.argtypes = [ctypes.c_uint] * 3
    dll.joy_new.restype = ctypes.c_void_p
    dll.joy_free.argtypes = [ctypes.c_void_p]
    dll.joy_feed.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_uint, ctypes.c_uint]
    dll.joy_take_reply.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_uint]
    dll.joy_take_reply.restype = ctypes.c_uint
    dll.joy_set_status.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int]
    dll.joy_poll.argtypes = [ctypes.c_void_p, ctypes.c_uint, ctypes.POINTER(JoyOutput)]
    return dll


class Joy:
    _dll = None

    def __init__(self):
        if Joy._dll is None:
            Joy._dll = _load()
        self.dll = Joy._dll
        self.handle = self.dll.joy_new(HANDHELD_ADDRESS, LINK_TIMEOUT_MS, DEADBAND)
        if not self.handle:
            raise RuntimeError('joy_api config rejected')

    def feed(self, data, now_ms):
        if data:
            self.dll.joy_feed(self.handle, data, len(data), now_ms & 0xFFFFFFFF)

    def take_reply(self):
        buffer = ctypes.create_string_buffer(64)
        n = self.dll.joy_take_reply(self.handle, buffer, len(buffer))
        return buffer.raw[:n] if n else None

    def poll(self, now_ms):
        out = JoyOutput()
        self.dll.joy_poll(self.handle, now_ms & 0xFFFFFFFF, ctypes.byref(out))
        return out

    def set_status(self, indicator, state):
        self.dll.joy_set_status(self.handle, indicator, state)

    def close(self):
        if self.handle:
            self.dll.joy_free(self.handle)
            self.handle = None
