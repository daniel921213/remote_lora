"""Orin 主程式：LoRa 收 F710 → 決定聽搖桿（JOY）還是導航（ORIN）→ 送轉速給 STM32。

  python3 orin_bridge.py --lora-port /dev/lora_gs --stm32-port /dev/ttyACM0
  python orin_bridge.py --lora-port COM8 --fake-stm32               沒 STM32，筆電上測
  python orin_bridge.py --lora-port COM8 --fake-stm32 --fake-auto   假 STM32 會自己進 AUTO

STM32 的串口一次只能一支程式開，跑這支時 monitor 要先關。結束時會送 ESTOP 把車鎖住。
"""

import argparse
import importlib.util
import os
import subprocess
import sys
import time
from pathlib import Path

from fake_stm32 import FakeStm32
from joy_lib import Joy, tools
from remote_logic import AUTO, ESTOP, EV_ENGINE_ON, EV_UNLOCK, STATE_NAMES_ZH, RemoteLogic
from motion_authority import MotionAuthority

HERE = Path(__file__).resolve().parent
STEP = 0.01            # 主迴圈 10 ms 一圈
COMMAND_PERIOD = 0.05  # 50 ms 送一次命令；STM32 超過 200 ms 沒收到會自己停
FRESH = 0.15           # STM32 回報超過這個時間沒更新，就當它沒回應
DISPLAY_PERIOD = 0.2
CLEAR_CONFIRM = 1.0    # 送出解除 ESTOP 後，這段時間內要看到 STM32 回到 AUTO
MOWER_LINK_DIRS = [HERE]
INDICATOR_TEXT = {0: 'ORIN', 1: 'JOY', 2: '等待 AUTO', 3: '異常'}


class Bridge:
    """每叫一次 step()：收 LoRa、回手把、送命令給 STM32。"""

    def __init__(self, lora, stm32, logic, joy, log, clock=time.monotonic, start=None):
        self.lora, self.stm32, self.logic, self.joy, self.log, self.clock = lora, stm32, logic, joy, log, clock
        self.start = clock() if start is None else start
        self.last_command = None
        self.command_error = False
        self.shown, self.shown_at = None, None
        self.authority = MotionAuthority(log)
        self.key_buffer = ''
        self.last_applied = (0.0, 0.0)
        self.clear_sent = None

    def key(self, key):
        # 鍵盤打 ARM 再按 Enter，跟按 LB 一樣（備用）
        if key in ('\r', '\n'):
            if self.key_buffer == 'ARM':
                self.authority.request()
            elif self.key_buffer:
                self.log('輸入 ARM 並按 Enter 才會申請行走許可')
            self.key_buffer = ''
        elif key in ('\b', '\x7f'):
            self.key_buffer = self.key_buffer[:-1]
        elif key.isprintable():
            self.key_buffer = (self.key_buffer + key)[-16:]

    def stm32_state(self, now):
        status = self.stm32.status
        if status is None or now - self.stm32.status_at > FRESH:
            return None
        return status['state']

    def clear_allowed(self, rc, state):
        # F710 START 解除 ESTOP：STM32 自己說可以（clear_ready）才送，而且只送一次
        if state != ESTOP:
            self.log('START：STM32 沒回應，不能解除' if state is None else 'START：STM32 沒有在 ESTOP，不用解除')
        elif not rc.sticks_neutral:
            self.log('START 拒絕：先放開搖桿')
        elif not self.stm32.status.get('clear_ready', False):
            self.log('START 拒絕：STM32 還不能解除（故障沒排除、R12 搖桿沒回中、引擎開關不在 HOLD 或編碼器異常）')
        else:
            return True
        return False

    def step(self):
        now = self.clock()
        now_ms = int((now - self.start) * 1000)
        # 收遙控器、收 STM32
        self.joy.feed(self.lora.read(self.lora.in_waiting or 1), now_ms)
        rc = self.joy.poll(now_ms)
        self.stm32.poll()
        state = self.stm32_state(now)

        # 照搖桿算轉速；按 LB 就申請解鎖
        command = self.logic.step(rc, state)
        for message in command.messages:
            self.log(message)
        if rc.events & EV_UNLOCK:
            self.authority.request()
        clear = bool(rc.events & EV_ENGINE_ON) and self.clear_allowed(rc, state)
        status = self.stm32.status
        if self.clear_sent is not None:
            if state == AUTO and not status['faults']:
                self.log('ESTOP 已解除：STM32 回到 AUTO 待命，放開搖桿按 LB 做 ARM')
                self.clear_sent = None
            elif now - self.clear_sent > CLEAR_CONFIRM:
                self.log('ESTOP 解除沒有確認：不自動重送，再按一次 START 或用 R12')
                self.clear_sent = None
        authority_status = dict(status, _received_at=self.stm32.status_at) if status else None
        was_active = self.authority.active
        arm = self.authority.update(authority_status, state is not None, rc, self.logic.mode,
                                    command.engine_kill, now)
        applied = command
        # 沒解鎖就一律送 0
        if not self.authority.active:
            applied = command._replace(left=0.0, right=0.0, urgent=command.urgent or was_active,
                                       indicator=command.indicator if command.indicator == 3 else 2)
        # 回手把，再送 STM32（平常 50 ms 一次，要停車就馬上送）
        self.joy.set_status(applied.indicator, 0 if state is None else state)
        reply = self.joy.take_reply()
        if reply:
            self.lora.write(reply)
        if arm or clear or applied.urgent or self.last_command is None or now - self.last_command >= COMMAND_PERIOD:
            self.send(applied, now, arm=arm, clear=clear)
        self.display(rc, state, command, now)
        return applied

    def send(self, command, now, arm=False, clear=False):
        self.last_command = now
        try:
            if clear:
                self.stm32.command(clear_estop=True)
                self.clear_sent = now
            elif arm:
                self.stm32.command(arm_auto=True)
                # command() 送完序號會 +1，減回去才是 ARM 那包的序號
                self.authority.arm_sent((self.stm32.sequence - 1) & 0xffffffff, now)
            else:
                self.stm32.command(left=command.left, right=command.right, engine_kill=command.engine_kill)
            self.last_applied = (command.left, command.right)
        except ConnectionError:
            # STM32 沒回報或寫不進去：先鎖起來，STM32 自己 200 ms 後會停
            self.authority.revoke('串口寫入失敗或沒有新遙測')
            if not self.command_error:
                self.log('STM32 沒有新的遙測：暫停送行走命令（STM32 自己 200 ms 後會停）')
                self.command_error = True
            return
        if self.command_error:
            self.log('STM32 遙測恢復')
            self.command_error = False
        self.logic.sent()

    def display(self, rc, state, command, now):
        # Orin 畫面那一行：要求＝搖桿想要的、送出＝真的送的、實測＝STM32 量到的
        status = self.stm32.status
        key = (self.logic.mode, bool(rc.link_ok), command.left, command.right, state,
               repr(status and {k: status.get(k) for k in ('faults', 'rpm', 'pwm', 'health')}),
               self.authority.phase, self.last_applied)
        if key == self.shown or (self.shown_at is not None and now - self.shown_at < DISPLAY_PERIOD):
            return
        stm32 = '沒回應' if state is None else f'{STATE_NAMES_ZH.get(state, state)} faults={status["faults"]}'
        measured = ''
        if state is not None:
            left, right = (round(rpm, 1) + 0.0 for rpm in status['rpm'])
            measured = f'｜實測 左 {left:+.1f} 右 {right:+.1f}'
            measured += (f'｜PWM {status.get("pwm")}｜ARM {self.authority.phase}'
                         f' ready={int(status.get("arm_ready", False))}'
                         f' run={int(status.get("run_request", False))}'
                         f' health=0x{status.get("health", 0):02X}')
        self.log(f'模式 {self.logic.mode}｜遙控 {"正常" if rc.link_ok else "失聯"}｜'
                 f'要求 左 {command.left:+.1f} 右 {command.right:+.1f}｜'
                 f'送出 左 {self.last_applied[0]:+.1f} 右 {self.last_applied[1]:+.1f}｜STM32 {stm32}{measured}｜'
                 f'回報 {INDICATOR_TEXT[command.indicator if self.authority.active or command.indicator == 3 else 2]}')
        self.shown, self.shown_at = key, now

    def shutdown(self):
        # 結束前送 3 次 ESTOP 把車鎖住
        for _ in range(3):
            try:
                self.stm32.command(estop=True)
            except Exception:
                pass
            time.sleep(0.05)


def load_mower_link(directory):
    # 只用同資料夾的 new_mower_link.py，不會用組員舊版的
    for folder in [Path(directory)] if directory else MOWER_LINK_DIRS:
        source = folder / 'new_mower_link.py'
        if source.exists():
            spec = importlib.util.spec_from_file_location('new_mower_link', source)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            return module.MowerLink, folder
    sys.exit('找不到 new_mower_link.py；移除舊的 --mower-link-dir，使用 bridge 同資料夾的新版。')


def check_port_free(port):
    # 串口被別的程式占用就停下來
    if os.name == 'posix':
        result = subprocess.run(['fuser', os.path.realpath(port)], capture_output=True, text=True)
        if result.returncode == 0:
            sys.exit(f'串口 {port} 已被使用（PID {result.stdout.strip()}）；請先確認占用程式，不會自動關閉它。')

def main():
    parser = argparse.ArgumentParser(description='Orin 端：收 F710 的 LoRa 封包，判斷 JOY／ORIN，送轉速給 STM32')
    parser.add_argument('--lora-port', required=True, help='LoRa 模組的串口，例如 /dev/ttyUSB0（筆電測試用 COM8）')
    target = parser.add_mutually_exclusive_group(required=True)
    target.add_argument('--stm32-port', help='STM32 的串口，例如 /dev/ttyACM0')
    target.add_argument('--fake-stm32', action='store_true', help='不接 STM32，用程式模擬；鍵盤 a/b/c/r 模擬 R12')
    parser.add_argument('--fake-auto', action='store_true', help='搭配 --fake-stm32：1 秒後自己進 AUTO')
    parser.add_argument('--max-rpm', type=float, default=10.0, help='JOY 搖桿推到底的轉速，預設 10，最大 40')
    parser.add_argument('--auto-rpm', type=float, default=5.0,
                        help='ORIN 模式的測試轉速（左右相同、往前），預設 5；0 表示原地不動')
    parser.add_argument('--mower-link-dir', help='獨立 new_mower_link.py 所在的資料夾；預設 bridge 同資料夾')
    parser.add_argument('--log', help='記錄真實 STM32 收送封包的 JSONL 檔案（附加寫入）')
    args = parser.parse_args()
    if args.log and args.fake_stm32:
        parser.error('--log 用於真實 STM32，請搭配 --stm32-port')
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

    stm_log = None
    if args.log:
        try:
            stm_log = open(args.log, 'a', encoding='utf-8')
        except OSError as error:
            parser.error(f'無法開啟紀錄檔 {args.log}：{error}')
        log(f'STM32 收送紀錄：{args.log}')

    check_port_free(args.lora_port)
    if not args.fake_stm32:
        check_port_free(args.stm32_port)
    lora = tools.open_serial(args.lora_port)
    if args.fake_stm32:
        stm32 = FakeStm32(log=log, auto_after=1.0 if args.fake_auto else None)
        where = '假 STM32（鍵盤 a＝R12 的 A 進 AUTO、b＝B、c＝C、r＝從 ESTOP 恢復）'
    else:
        mower_link, folder = load_mower_link(args.mower_link_dir)
        try:
            stm32 = mower_link(args.stm32_port, log=stm_log)
        except Exception as error:
            sys.exit(f'無法開啟 STM32 串口 {args.stm32_port}：{error}')
        where = f'STM32 {args.stm32_port}（new_mower_link.py 在 {folder}）'
    joy = Joy()
    keys = tools.Keys(preserve_case=True)
    print(f'Orin 端：LoRa {args.lora_port}；{where}')
    print(f'JOY 搖桿推到底 {args.max_rpm:g} RPM；ORIN 測試轉速 {args.auto_rpm:g} RPM。'
          '按 q 結束（結束時會送 ESTOP，用 R12 或 F710 的 START 恢復）。')
    print('行走預設鎖定。R12 進 AUTO、F710 置中後，按 F710 的 LB（或在這裡輸入 ARM 並按 Enter）；失聯後需重新確認。')
    print('ESTOP 時放開搖桿按 F710 的 START 解除（R12 照樣可以），解除後還要再按 LB 才能開。')

    bridge = Bridge(lora, stm32, logic, joy, log, start=start)
    try:
        if not args.fake_stm32:
            # 先等 STM32 第一筆回報，最多 2 秒
            deadline = time.monotonic() + 2.0
            while stm32.status is None and time.monotonic() < deadline:
                stm32.poll()
                time.sleep(0.01)
            if stm32.status is None:
                log('⚠ 2 秒內沒收到 STM32 遙測：確認 USB 線和串口名稱，以及沒有其他程式開著這個埠')
        # 主迴圈
        while True:
            began = time.monotonic()
            bridge.step()
            key = keys.get()
            if key == 'q':
                break
            # 假 STM32 時 a/b/c/r 是假裝按 R12
            if key and args.fake_stm32 and key in 'abcr' and not bridge.key_buffer:
                stm32.key(key)
            elif key:
                bridge.key(key)
            time.sleep(max(0.0, STEP - (time.monotonic() - began)))
    except KeyboardInterrupt:
        pass
    finally:
        # 不管怎麼結束都送 ESTOP
        log('結束：送 ESTOP 給 STM32')
        bridge.shutdown()
        keys.close()
        joy.close()
        stm32.close()
        if stm_log is not None:
            stm_log.close()
        lora.close()


if __name__ == '__main__':
    main()
