"""Orin side: F710 packets from the LoRa module -> JOY/ORIN choice -> STM32 RPM commands.

  python3 orin_bridge.py --lora-port /dev/ttyUSB0 --stm32-port /dev/ttyACM0
  python orin_bridge.py --lora-port COM8 --fake-stm32               laptop test without an STM32
  python orin_bridge.py --lora-port COM8 --fake-stm32 --fake-auto   ... entering AUTO by itself

STM32 commands go through the teammate's MowerLink (jetson/mower_link.py), which must be the
only program on the STM32 port: do not run monitor at the same time. Commands go out every
50 ms, and at once on a stop. On exit the bridge sends a latched ESTOP, as the teammate's
guide asks for an abnormal exit; recovering needs the R12.
"""

import argparse
import sys
import time
from pathlib import Path

from fake_stm32 import FakeStm32
from joy_lib import Joy, tools
from remote_logic import STATE_NAMES_ZH, RemoteLogic

HERE = Path(__file__).resolve().parent
STEP = 0.01            # loop period
COMMAND_PERIOD = 0.05  # 20 Hz as the STM32 guide recommends; AUTO stops after 200 ms without one
FRESH = 0.15           # older telemetry counts as no STM32 state (MowerLink uses the same limit)
DISPLAY_PERIOD = 0.2
MOWER_LINK_DIRS = [Path('/home/user/MowerL476_project/MowerL476/jetson'), HERE]  # path from the guide
INDICATOR_TEXT = {0: 'ORIN', 1: 'JOY', 2: '等待 AUTO', 3: '異常'}


class Bridge:
    """One pass of step() reads the LoRa link, answers the handheld and commands the STM32."""

    def __init__(self, lora, stm32, logic, joy, log, clock=time.monotonic, start=None):
        self.lora, self.stm32, self.logic, self.joy, self.log, self.clock = lora, stm32, logic, joy, log, clock
        self.start = clock() if start is None else start
        self.last_command = None
        self.command_error = False
        self.shown, self.shown_at = None, None

    def stm32_state(self, now):
        status = self.stm32.status
        if status is None or now - self.stm32.status_at > FRESH:
            return None
        return status['state']

    def step(self):
        now = self.clock()
        now_ms = int((now - self.start) * 1000)
        self.joy.feed(self.lora.read(self.lora.in_waiting or 1), now_ms)
        rc = self.joy.poll(now_ms)
        self.stm32.poll()
        state = self.stm32_state(now)

        command = self.logic.step(rc, state)
        for message in command.messages:
            self.log(message)
        # Reply after deciding, so it carries the state this very packet led to.
        self.joy.set_status(command.indicator, 0 if state is None else state)
        reply = self.joy.take_reply()
        if reply:
            self.lora.write(reply)
        if command.urgent or self.last_command is None or now - self.last_command >= COMMAND_PERIOD:
            self.send(command, now)
        self.display(rc, state, command, now)
        return command

    def send(self, command, now):
        self.last_command = now
        try:
            self.stm32.command(left=command.left, right=command.right, engine_kill=command.engine_kill)
        except ConnectionError:
            if not self.command_error:
                self.log('STM32 沒有新的遙測：暫停送行走命令（STM32 自己 200 ms 後會停）')
                self.command_error = True
            return
        if self.command_error:
            self.log('STM32 遙測恢復')
            self.command_error = False
        self.logic.sent()

    def display(self, rc, state, command, now):
        status = self.stm32.status
        key = (self.logic.mode, bool(rc.link_ok), command.left, command.right, state,
               status['faults'] if status else None)
        if key == self.shown or (self.shown_at is not None and now - self.shown_at < DISPLAY_PERIOD):
            return
        stm32 = '沒回應' if state is None else f'{STATE_NAMES_ZH.get(state, state)} faults={status["faults"]}'
        measured = ''
        if state is not None:
            left, right = (round(rpm, 1) + 0.0 for rpm in status['rpm'])  # + 0.0 turns -0.0 into 0.0
            measured = f'｜實測 左 {left:+.1f} 右 {right:+.1f}'
        self.log(f'模式 {self.logic.mode}｜遙控 {"正常" if rc.link_ok else "失聯"}｜'
                 f'命令 左 {command.left:+.1f} 右 {command.right:+.1f}｜STM32 {stm32}{measured}｜'
                 f'回報 {INDICATOR_TEXT[command.indicator]}')
        self.shown, self.shown_at = key, now

    def shutdown(self):
        for _ in range(3):
            try:
                self.stm32.command(estop=True)
            except Exception:
                pass
            time.sleep(0.05)


def load_mower_link(directory):
    for folder in [Path(directory)] if directory else MOWER_LINK_DIRS:
        if (folder / 'mower_link.py').exists():
            sys.path.insert(0, str(folder))
            from mower_link import MowerLink
            return MowerLink, folder
    sys.exit('找不到組員的 mower_link.py：用 --mower-link-dir 指定它所在的資料夾（MowerL476/jetson）。')


def main():
    parser = argparse.ArgumentParser(description='Orin 端：收 F710 的 LoRa 封包，判斷 JOY／ORIN，送轉速給 STM32')
    parser.add_argument('--lora-port', required=True, help='LoRa 模組的串口，例如 /dev/ttyUSB0（筆電測試用 COM8）')
    target = parser.add_mutually_exclusive_group(required=True)
    target.add_argument('--stm32-port', help='STM32 的串口，例如 /dev/ttyACM0')
    target.add_argument('--fake-stm32', action='store_true', help='不接 STM32，用程式模擬；鍵盤 a/b/c/r 模擬 R12')
    parser.add_argument('--fake-auto', action='store_true', help='搭配 --fake-stm32：1 秒後自己進 AUTO')
    parser.add_argument('--max-rpm', type=float, default=5.0, help='JOY 搖桿推到底的轉速，預設 5，最大 40')
    parser.add_argument('--auto-rpm', type=float, default=5.0,
                        help='ORIN 模式的測試轉速（左右相同、往前），預設 5；0 表示原地不動')
    parser.add_argument('--mower-link-dir', help='組員 mower_link.py 所在的資料夾')
    args = parser.parse_args()
    if args.fake_auto and not args.fake_stm32:
        parser.error('--fake-auto 只能搭配 --fake-stm32')
    try:
        logic = RemoteLogic(args.max_rpm, args.auto_rpm)
    except ValueError as error:
        parser.error(str(error))
    tools.utf8_console()

    start = time.monotonic()

    def log(text):
        print(f'[{time.monotonic() - start:6.1f}s] {text}', flush=True)

    lora = tools.open_serial(args.lora_port)
    if args.fake_stm32:
        stm32 = FakeStm32(log=log, auto_after=1.0 if args.fake_auto else None)
        where = '假 STM32（鍵盤 a＝R12 的 A 進 AUTO、b＝B、c＝C、r＝從 ESTOP 恢復）'
    else:
        mower_link, folder = load_mower_link(args.mower_link_dir)
        try:
            stm32 = mower_link(args.stm32_port)
        except Exception as error:
            sys.exit(f'無法開啟 STM32 串口 {args.stm32_port}：{error}')
        where = f'STM32 {args.stm32_port}（mower_link.py 在 {folder}）'
    joy = Joy()
    keys = tools.Keys()
    print(f'Orin 端：LoRa {args.lora_port}；{where}')
    print(f'JOY 搖桿推到底 {args.max_rpm:g} RPM；ORIN 測試轉速 {args.auto_rpm:g} RPM。'
          '按 q 結束（結束時會送 ESTOP，要用 R12 恢復）。')

    bridge = Bridge(lora, stm32, logic, joy, log, start=start)
    try:
        if not args.fake_stm32:
            deadline = time.monotonic() + 2.0
            while stm32.status is None and time.monotonic() < deadline:
                stm32.poll()
                time.sleep(0.01)
            if stm32.status is None:
                log('⚠ 2 秒內沒收到 STM32 遙測：確認 USB 線和串口名稱，以及沒有其他程式開著這個埠')
        while True:
            began = time.monotonic()
            bridge.step()
            key = keys.get()
            if key == 'q':
                break
            if key and args.fake_stm32:
                stm32.key(key)
            time.sleep(max(0.0, STEP - (time.monotonic() - began)))
    except KeyboardInterrupt:
        pass
    finally:
        log('結束：送 ESTOP 給 STM32')
        bridge.shutdown()
        keys.close()
        joy.close()
        stm32.close()
        lora.close()


if __name__ == '__main__':
    main()
