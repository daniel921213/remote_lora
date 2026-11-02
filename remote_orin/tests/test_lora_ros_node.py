"""lora_ros_node 離線測試：真的手把 C 程式 → 記憶體裡的無線電 → 真的 joy_api → Remote。

ROS 那一層換成 FakeOut，記下要發的 joint_command、/remote/mode 和要叫的 service。
"""

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path[:0] = [str(ROOT / 'orin'), str(ROOT / 'handheld_pc'), str(ROOT / 'common'), str(ROOT / 'tests')]

import remote_logic as rl  # noqa: E402
from handheld_lib import SLOT, Handheld  # noqa: E402
from joy_lib import Joy  # noqa: E402
from lora_ros_node import Remote  # noqa: E402
from test_system import Clock, Radio  # noqa: E402


class FakeOut:
    def __init__(self):
        self.logs, self.wheel_log, self.samples, self.modes, self.calls = [], [], [], [], []

    def log(self, text):
        self.logs.append(text)

    def wheels(self, left, right):
        self.wheel_log.append((left, right))

    def sample(self, sample):
        self.samples.append(sample)

    def mode(self, mode):
        self.modes.append(mode)

    def estop(self):
        self.calls.append('/estop')

    def release(self):
        self.calls.append('/release_state')


class TestLoraRosNode(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.radio = Radio()
        self.out = FakeOut()
        self.remote = Remote(self.radio.orin, Joy(), rl.RemoteLogic(5, 3), self.out, clock=self.clock)
        self.handheld = Handheld(0)
        self.start = self.clock()

    def tearDown(self):
        self.handheld.close()
        self.remote.joy.close()

    def run_for(self, seconds, connected=True, drive=0, turn=0, buttons=()):
        mask = sum(1 << SLOT[name] for name in buttons)
        for _ in range(round(seconds / 0.01)):
            now_ms = int((self.clock() - self.start) * 1000)
            self.handheld.sample(connected, drive, turn, mask)
            line = self.handheld.take_line(now_ms)
            if line:
                self.radio.handheld.write(line)
            self.handheld.feed(self.radio.handheld.read(1024), now_ms)
            self.handheld.poll(now_ms)
            self.remote.step()
            self.clock.t += 0.01

    @property
    def wheels(self):
        return self.out.wheel_log[-1]

    def drive_ready(self):
        self.remote.state = rl.AUTO  # 等於 stm_bridge 回報 STM32 在 AUTO
        self.run_for(0.3)

    def test_nothing_moves_without_stm_bridge(self):
        self.run_for(0.5, drive=100)
        self.assertEqual(self.wheels, (0.0, 0.0))
        self.assertEqual(self.handheld.info().indicator, rl.IND_FAULT)
        self.assertEqual(self.out.modes, [rl.JOY])
        self.run_for(0.15, buttons=['START'])
        self.run_for(0.15, buttons=['LB'])
        self.run_for(0.2)
        self.assertIn('START：STM32 沒回應，不能解除', self.out.logs)
        self.assertIn('LB：STM32 沒回應，不能 ARM', self.out.logs)
        self.assertEqual(self.out.calls, [])

    def test_joy_then_rb_orin_at_20hz(self):
        self.drive_ready()
        sent = len(self.out.wheel_log)
        self.run_for(1.0, drive=100)
        self.assertEqual(self.wheels, (5.0, 5.0))
        self.assertAlmostEqual(len(self.out.wheel_log) - sent, 20, delta=3)
        self.assertEqual(self.handheld.info().indicator, rl.IND_JOY)

        self.run_for(0.15, drive=100, buttons=['RB'])
        self.run_for(0.3, drive=100)  # ORIN 不看搖桿
        self.assertEqual(self.wheels, (3.0, 3.0))
        self.assertEqual(self.out.modes, [rl.JOY, rl.ORIN])
        self.assertEqual(self.handheld.info().indicator, rl.IND_ORIN)

        self.run_for(0.15, buttons=['RB'])
        self.run_for(0.2)
        self.assertEqual(self.wheels, (0.0, 0.0))
        self.assertEqual(self.out.modes[-1], rl.JOY)

    def test_back_calls_estop_then_start_clears(self):
        self.drive_ready()
        self.run_for(0.3, drive=100)
        self.assertEqual(self.wheels, (5.0, 5.0))

        self.run_for(0.15, drive=100, buttons=['BACK'])
        self.run_for(0.1, drive=100)
        self.assertEqual(self.out.calls, ['/estop'])
        self.assertEqual(self.wheels, (0.0, 0.0))
        self.assertIn('BACK：停車，呼叫 /estop 鎖住（這條路不會熄火；用 START 解除）', self.out.logs)
        self.assertFalse(any('熄火並鎖定' in text for text in self.out.logs))
        self.remote.state = rl.ESTOP  # /estop 成功
        self.run_for(0.2)
        self.assertEqual(self.handheld.info().indicator, rl.IND_FAULT)

        self.run_for(0.15, buttons=['LB'])  # ESTOP 時按 LB：不送
        self.run_for(0.2)
        self.assertIn('LB：STM32 不在 AUTO，不能 ARM', self.out.logs)
        self.run_for(0.15, drive=100, buttons=['START'])  # 搖桿推著：不送
        self.run_for(0.2, drive=100)
        self.assertIn('START 拒絕：先放開搖桿', self.out.logs)
        self.assertEqual(self.out.calls, ['/estop'])

        self.run_for(0.15, buttons=['START'])
        self.run_for(0.2)
        self.assertEqual(self.out.calls, ['/estop', '/release_state'])
        self.remote.state = rl.AUTO  # main_control 解除 ESTOP，再自己 ARM
        self.run_for(0.2)
        self.run_for(0.3, drive=100)
        self.assertEqual(self.wheels, (5.0, 5.0))

    def test_back_lock_holds_even_if_estop_failed(self):
        self.drive_ready()
        self.run_for(0.15, buttons=['BACK'])
        self.run_for(0.3, drive=100)  # /estop 沒成功、STM32 還在 AUTO：這邊照樣送 0
        self.assertEqual(self.wheels, (0.0, 0.0))
        self.assertEqual(self.handheld.info().indicator, rl.IND_FAULT)
        self.run_for(0.15, buttons=['LB'])
        self.run_for(0.2)
        self.assertIn('LB 拒絕：BACK 鎖住中，先按 START', self.out.logs)

        self.run_for(0.15, buttons=['START'])
        self.run_for(0.2)
        self.assertIn('START：解除 BACK 的停車鎖', self.out.logs)
        self.assertIn('START：STM32 沒有在 ESTOP，不用解除', self.out.logs)
        self.assertEqual(self.out.calls, ['/estop'])
        self.run_for(0.3, drive=100)
        self.assertEqual(self.wheels, (5.0, 5.0))

    def test_estop_state_stops_and_needs_neutral_after(self):
        self.drive_ready()
        self.run_for(0.3, drive=100)
        self.remote.state = rl.ESTOP  # STM32 自己進 ESTOP（R12、逾時…）
        self.run_for(0.2, drive=100)
        self.assertEqual(self.wheels, (0.0, 0.0))
        self.remote.state = rl.AUTO
        self.run_for(0.3, drive=100)  # 回 AUTO 時搖桿還推著：先回中才會動
        self.assertEqual(self.wheels, (0.0, 0.0))
        self.run_for(0.2)
        self.run_for(0.3, drive=100)
        self.assertEqual(self.wheels, (5.0, 5.0))

    def test_lb_calls_release_and_link_loss_stops(self):
        self.drive_ready()
        self.run_for(0.15, buttons=['START'])  # AUTO 時按 START：不送
        self.run_for(0.2)
        self.assertIn('START：STM32 沒有在 ESTOP，不用解除', self.out.logs)
        self.assertEqual(self.out.calls, [])
        self.run_for(0.15, buttons=['LB'])
        self.run_for(0.2)
        self.assertEqual(self.out.calls, ['/release_state'])
        self.run_for(0.3, drive=100)
        self.assertEqual(self.wheels, (5.0, 5.0))

        self.radio.up = False  # LoRa 斷了
        self.run_for(0.5, drive=100)
        self.assertEqual(self.wheels, (0.0, 0.0))
        self.assertFalse(self.out.samples[-1].connected)
        self.assertIn('遙控器失聯：停車，回到 JOY', self.out.logs)

    def test_close_publishes_zero(self):
        self.drive_ready()
        self.run_for(0.3, drive=100)
        self.radio.orin.close = lambda: None  # 假無線電沒有 close
        self.remote.close()
        self.assertEqual(self.wheels, (0.0, 0.0))
        self.assertFalse(self.out.samples[-1].connected)


if __name__ == '__main__':
    unittest.main()
