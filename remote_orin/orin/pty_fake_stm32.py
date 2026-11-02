#!/usr/bin/env python3
"""把 fake_stm32.py 包成假串口（PTY），給組員的 main_control 當 STM32 用。只能在 Linux 跑。

  python3 pty_fake_stm32.py --auto
  cd ~/fucking_mower && python3 -m main_control --stm-port /tmp/fake-stm32 --no-lora --no-auto-map --tcp-port 8790

收 main_control 送來的 32 bytes 命令 → 交給 FakeStm32；每 0.1 秒把 FakeStm32 的狀態包成 32 bytes 回報。
封包格式跟真的 STM32（10/28 版、有 ARM）一樣，R12、編碼器當作正常。
鍵盤：a＝R12 的 A（進 AUTO）、b＝B、c＝C、r＝從 ESTOP 恢復、q＝結束。
"""

import argparse
import binascii
import errno
import os
import pty
import select
import struct
import time
import tty
from pathlib import Path

from fake_stm32 import FakeStm32
from joy_lib import tools
from remote_logic import AUTO

FLAG_NAMES = ((1, 'ESTOP'), (2, 'ENGINE_KILL'), (4, 'CLEAR_ESTOP'), (8, 'ARM'))
HEALTH_RC_ENCODERS = 0x02 | 0x08 | 0x10  # R12、左右編碼器都當作正常
HEALTH_JETSON = 0x04


def crc16(data):
    return binascii.crc_hqx(bytes(data), 0xFFFF)


def status_packet(status, health):
    p = bytearray(32)
    p[:4] = b'\xA5\x5A\x01\x81'
    struct.pack_into('<IIhhHH', p, 4, status['sequence'], status['lease'],
                     round(status['rpm'][0] * 100), round(status['rpm'][1] * 100), *status['pwm'])
    p[20] = status['state']
    p[21] = health
    struct.pack_into('<H', p, 22, status['faults'])
    struct.pack_into('<H', p, 30, crc16(p[:30]))
    return bytes(p)


def decode_command(p):
    if p[:4] != b'\xA5\x5A\x01\x01' or crc16(p[:30]) != struct.unpack_from('<H', p, 30)[0]:
        return None
    _, _, left, right, hydraulic, flags = struct.unpack_from('<IIhhbB', p, 4)
    return left / 100.0, right / 100.0, hydraulic, flags


def main():
    parser = argparse.ArgumentParser(description='假 STM32 串口（PTY），給 main_control 用')
    parser.add_argument('--link', default='/tmp/fake-stm32', help='給 main_control --stm-port 用的路徑')
    parser.add_argument('--auto', action='store_true',
                        help='main_control 連上（收到第一筆命令）1 秒後自己進 AUTO，像 10/28 韌體開機就在 AUTO')
    args = parser.parse_args()
    tools.utf8_console()
    start = time.monotonic()

    def log(text):
        print(f'[{time.monotonic() - start:6.1f}s] {text}', flush=True)

    master, slave = pty.openpty()
    tty.setraw(slave)  # 二進位封包，不能讓終端機改字元
    slave_path = os.ttyname(slave)
    link = Path(args.link)
    if link.is_symlink():
        link.unlink()
    elif link.exists():
        parser.error(f'{link} 已存在而且不是捷徑，換一個 --link')
    link.symlink_to(slave_path)
    os.close(slave)
    os.set_blocking(master, False)

    fake = FakeStm32(log=log)
    keys = tools.Keys()
    log(f'假 STM32：{link} → {slave_path}')
    log('鍵盤：a＝R12 的 A（進 AUTO）、b、c、r＝從 ESTOP 恢復、q＝結束')
    buffer = bytearray()
    sent_at = None
    last_shown = None
    auto_pending, auto_at = args.auto, None
    try:
        while True:
            if auto_at is not None and fake.clock() >= auto_at:
                auto_at = None
                fake.key('a')
            fake.poll()
            # FakeStm32 每 0.1 秒更新一次狀態，更新了就送一包回報
            if fake.status is not None and fake.status_at != sent_at:
                health = fake.status['health'] | HEALTH_RC_ENCODERS
                if fake.state == AUTO and fake.clock() - fake.last_command <= 0.2:
                    health |= HEALTH_JETSON
                try:
                    os.write(master, status_packet(fake.status, health))
                except OSError:
                    pass  # main_control 還沒開這個埠
                sent_at = fake.status_at

            if select.select([master], [], [], 0.01)[0]:
                try:
                    buffer.extend(os.read(master, 4096))
                except BlockingIOError:
                    pass
                except OSError as error:
                    if error.errno != errno.EIO:  # 沒人開這個埠時會一直 EIO，正常
                        raise
            while len(buffer) >= 32:
                command = decode_command(buffer[:32])
                if command is None:
                    del buffer[0]
                    continue
                del buffer[:32]
                if auto_pending:
                    auto_pending, auto_at = False, fake.clock() + 1.0
                left, right, hydraulic, flags = command
                try:
                    fake.command(left, right, hydraulic, estop=bool(flags & 1), engine_kill=bool(flags & 2),
                                 clear_estop=bool(flags & 4), arm_auto=bool(flags & 8))
                except (ConnectionError, ValueError) as error:
                    log(f'命令被擋：{error}')
                if (left, right, hydraulic, flags) != last_shown:
                    names = '+'.join(name for bit, name in FLAG_NAMES if flags & bit) or '無'
                    log(f'收到命令：左 {left:+.2f} 右 {right:+.2f} RPM｜液壓 {hydraulic}｜旗標 {names}'
                        f'｜目標 {fake.target}')
                    last_shown = (left, right, hydraulic, flags)

            key = keys.get()
            if key == 'q':
                break
            if key:
                fake.key(key)
    except KeyboardInterrupt:
        pass
    finally:
        keys.close()
        os.close(master)
        if link.is_symlink() and os.path.realpath(link) == slave_path:
            link.unlink()


if __name__ == '__main__':
    main()
