"""Helpers shared by the laptop handheld, the Orin bridge and the tests.

Python 3.8 or newer (the Orin may run Ubuntu 20.04). No third-party packages except pyserial.
"""

import ctypes
import os
import subprocess
import sys
from pathlib import Path

COMMON = Path(__file__).resolve().parent
BAUD = 115200


def load_library(name, sources, out_dir):
    """Compiles the C sources into a shared library when it is missing or older than any
    source or common header, then loads it. Needs gcc (MinGW-w64 on Windows)."""
    out_dir = Path(out_dir)
    out_dir.mkdir(exist_ok=True)
    path = out_dir / (name + ('.dll' if os.name == 'nt' else '.so'))
    inputs = [Path(s) for s in sources] + list(COMMON.glob('*.h'))
    if not path.exists() or path.stat().st_mtime < max(p.stat().st_mtime for p in inputs):
        command = ['gcc', '-shared', '-std=c99', '-O2', '-Wall', '-Wextra', '-Werror', f'-I{COMMON}',
                   '-static-libgcc' if os.name == 'nt' else '-fPIC', *map(str, sources), '-o', str(path)]
        print(f'編譯 {path.name} …', flush=True)
        try:
            subprocess.run(command, check=True)
        except FileNotFoundError:
            sys.exit('找不到 gcc。Windows 請安裝 MinGW-w64；Orin 請執行 sudo apt install build-essential。')
    return ctypes.CDLL(str(path))


def open_serial(name):
    """Opens a serial port at 115200 8N1 without touching DTR/RTS, non-blocking reads."""
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
        sys.exit(f'無法開啟 {name}：{error}\n（有沒有其他程式或終端機正在使用這個埠？）')
    port.reset_input_buffer()
    return port


def utf8_console():
    if hasattr(sys.stdout, 'reconfigure'):
        sys.stdout.reconfigure(encoding='utf-8')


class Keys:
    """Single key presses without Enter. Returns None when no key (or no terminal) is available."""

    def __init__(self):
        self.restore = None
        if os.name == 'nt':
            import msvcrt
            self.msvcrt = msvcrt
        elif sys.stdin.isatty():
            import termios
            import tty
            fd = sys.stdin.fileno()
            saved = termios.tcgetattr(fd)
            tty.setcbreak(fd)
            self.restore = lambda: termios.tcsetattr(fd, termios.TCSADRAIN, saved)

    def get(self):
        if os.name == 'nt':
            return self.msvcrt.getwch().lower() if self.msvcrt.kbhit() else None
        if self.restore is None:
            return None
        import select
        if select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.read(1).lower()
        return None

    def close(self):
        if self.restore:
            self.restore()
            self.restore = None
