"""Python-side tests: Orin JOY/ORIN rules, the fake STM32, and an end-to-end run in one process.

The end-to-end test wires the real C handheld logic (what the laptop and the RP2040 run) to
the real C receiver (joy_api, what the Orin runs) through an in-memory radio, then through the
bridge into the fake STM32, all on a simulated clock.
"""

import binascii
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path[:0] = [str(ROOT / 'orin'), str(ROOT / 'handheld_pc'), str(ROOT / 'common')]

import remote_logic as rl  # noqa: E402
from fake_stm32 import FakeStm32  # noqa: E402
from handheld_lib import EV_INDICATOR, EV_LINK_LOST, EV_LINK_UP, SLOT, Handheld  # noqa: E402
from joy_lib import Joy  # noqa: E402
from orin_bridge import Bridge  # noqa: E402


def pack_input(seq, drive, turn, gamepad_ok, counters):
    """RC_INPUT written from RC_LORA_PROTOCOL.md 4.1, independently of the C packer."""
    frame = bytearray(16)
    frame[0], frame[1] = 0x01, 0x11
    frame[2:4] = seq.to_bytes(2, 'little')
    frame[4], frame[5] = drive & 0xFF, turn & 0xFF
    frame[6] = 1 if gamepad_ok else 0
    for i, count in enumerate(counters):
        frame[7 + i // 2] |= (count & 0x0F) << (4 if i % 2 else 0)
    frame[14:16] = binascii.crc_hqx(bytes(frame[:14]), 0xFFFF).to_bytes(2, 'little')
    return bytes(frame)


class Rc:
    """Stand-in for a joy_api output."""

    def __init__(self, link_ok=True, neutral=True, drive=0.0, turn=0.0, events=0):
        self.link_ok, self.sticks_neutral, self.drive, self.turn, self.events = link_ok, neutral, drive, turn, events


class Clock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


class Radio:
    """Two transparent LoRa modules: what one end writes comes out at the other, unless cut."""

    class End:
        def __init__(self, radio, inbox, outbox):
            self.radio, self.inbox, self.outbox = radio, inbox, outbox

        @property
        def in_waiting(self):
            return len(self.inbox)

        def read(self, n):
            data = bytes(self.inbox[:n])
            del self.inbox[:n]
            return data

        def write(self, data):
            if self.radio.up:
                self.outbox.extend(data)
            return len(data)

    def __init__(self):
        to_handheld, to_orin = bytearray(), bytearray()
        self.up = True
        self.handheld = Radio.End(self, to_handheld, to_orin)
        self.orin = Radio.End(self, to_orin, to_handheld)


class TestRemoteLogic(unittest.TestCase):
    def setUp(self):
        self.logic = rl.RemoteLogic(max_rpm=5, auto_rpm=3)

    def drive_ready(self):
        """AUTO, JOY, sticks already centred once."""
        self.logic.step(Rc(), rl.AUTO)

    def test_limits(self):
        for max_rpm, auto_rpm in ((0, 0), (41, 0), (5, 6), (5, -1)):
            with self.assertRaises(ValueError):
                rl.RemoteLogic(max_rpm, auto_rpm)

    def test_nothing_moves_outside_auto(self):
        out = self.logic.step(Rc(neutral=False, drive=1.0, events=rl.EV_CONTROL_TOGGLE), rl.STOPPED)
        self.assertEqual((out.left, out.right, out.indicator), (0.0, 0.0, rl.IND_WAITING))
        self.assertEqual(self.logic.mode, rl.JOY)
        self.assertIn('不在 AUTO', out.messages[0])
        self.assertEqual(self.logic.step(Rc(), None).indicator, rl.IND_FAULT)
        self.assertEqual(self.logic.step(Rc(), rl.ESTOP).indicator, rl.IND_FAULT)

    def test_joy_waits_for_neutral_then_mixes(self):
        out = self.logic.step(Rc(neutral=False, drive=1.0), rl.AUTO)
        self.assertEqual((out.left, out.right, out.indicator), (0.0, 0.0, rl.IND_JOY))
        self.drive_ready()
        cases = [((1.0, 0.0), (5.0, 5.0)), ((0.0, 1.0), (5.0, -5.0)), ((1.0, 1.0), (5.0, 0.0)),
                 ((-1.0, 0.5), (-1.67, -5.0)), ((0.5, 0.0), (2.5, 2.5))]
        for (drive, turn), expected in cases:
            out = self.logic.step(Rc(neutral=False, drive=drive, turn=turn), rl.AUTO)
            self.assertEqual((out.left, out.right), expected, (drive, turn))

    def test_rb_toggles_orin_test_speed(self):
        self.drive_ready()
        out = self.logic.step(Rc(neutral=False, drive=1.0, events=rl.EV_CONTROL_TOGGLE), rl.AUTO)
        self.assertEqual((self.logic.mode, out.left, out.right, out.indicator), (rl.ORIN, 3.0, 3.0, rl.IND_ORIN))
        out = self.logic.step(Rc(neutral=False, drive=1.0, events=rl.EV_CONTROL_TOGGLE), rl.AUTO)
        self.assertEqual((self.logic.mode, out.left, out.right), (rl.JOY, 0.0, 0.0))  # stick still pushed
        self.assertTrue(out.urgent)

    def test_b_stops_and_wins_over_rb(self):
        self.drive_ready()
        self.logic.step(Rc(events=rl.EV_CONTROL_TOGGLE), rl.AUTO)
        out = self.logic.step(Rc(events=rl.EV_PARK | rl.EV_CONTROL_TOGGLE), rl.AUTO)
        self.assertEqual((self.logic.mode, out.left, out.right, out.urgent), (rl.JOY, 0.0, 0.0, True))

    def test_back_sends_engine_kill_three_times(self):
        self.drive_ready()
        self.logic.step(Rc(neutral=False, drive=1.0), rl.AUTO)
        out = self.logic.step(Rc(neutral=False, drive=1.0, events=rl.EV_ENGINE_OFF), rl.AUTO)
        self.assertTrue(out.engine_kill and out.urgent)
        self.assertEqual((out.left, out.right), (0.0, 0.0))
        for _ in range(3):
            self.assertTrue(self.logic.step(Rc(), rl.AUTO).engine_kill)
            self.logic.sent()
        self.assertFalse(self.logic.step(Rc(), rl.AUTO).engine_kill)

    def test_link_loss_or_leaving_auto_falls_back_to_joy(self):
        self.drive_ready()
        self.logic.step(Rc(events=rl.EV_CONTROL_TOGGLE), rl.AUTO)
        out = self.logic.step(Rc(link_ok=False), rl.AUTO)
        self.assertEqual((self.logic.mode, out.left, out.urgent), (rl.JOY, 0.0, True))
        self.logic.step(Rc(), rl.AUTO)
        self.logic.step(Rc(events=rl.EV_CONTROL_TOGGLE), rl.AUTO)
        self.logic.step(Rc(), rl.STOPPED)
        self.assertEqual(self.logic.mode, rl.JOY)
        out = self.logic.step(Rc(neutral=False, drive=1.0), rl.AUTO)  # back in AUTO: neutral first
        self.assertEqual(out.left, 0.0)

    def test_messages_for_unsupported_buttons(self):
        events = rl.EV_ENGINE_ON | rl.EV_GEAR_BLADE | rl.EV_UNLOCK | rl.EV_BLADE_DOWN
        messages = self.logic.step(Rc(events=events), rl.AUTO).messages
        self.assertEqual(len(messages), 4)
        self.assertIn('LB：申請 ARM（解開行走鎖）', messages)


class TestFakeStm32(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.stm32 = FakeStm32(clock=self.clock, log=lambda text: None)
        self.stm32.poll()

    def test_follows_only_in_auto(self):
        self.stm32.command(5, 5)
        self.assertEqual(self.stm32.target, [0.0, 0.0])
        self.stm32.key('a')
        self.stm32.command(5, -5)
        self.assertEqual(self.stm32.target, [0.0, 0.0])  # AUTO alone is not ARM
        for _ in range(12):
            self.clock.t += 0.05
            self.stm32.command()
        self.stm32.command(arm_auto=True)
        self.stm32.command(5, -5)
        self.assertEqual(self.stm32.target, [5.0, -5.0])

    def test_auto_stops_when_commands_pause(self):
        self.stm32.key('a')
        self.clock.t += 0.19
        self.stm32.poll()
        self.assertEqual(self.stm32.state, rl.AUTO)
        self.clock.t += 0.02
        self.stm32.poll()
        self.assertEqual((self.stm32.state, self.stm32.faults), (rl.ESTOP, 16))

    def test_estop_latches_until_r12_recovery(self):
        self.stm32.key('a')
        self.stm32.command(engine_kill=True)
        self.assertEqual((self.stm32.state, self.stm32.faults), (rl.ESTOP, 512))
        self.stm32.command(5, 5)
        self.stm32.key('a')
        self.assertEqual(self.stm32.state, rl.ESTOP)
        self.stm32.key('r')
        self.assertEqual((self.stm32.state, self.stm32.faults), (rl.STOPPED, 0))

    def test_clear_estop_needs_one_second_of_zero(self):
        self.stm32.key('a')
        self.stm32.command(estop=True)
        self.stm32.command(clear_estop=True)
        self.assertEqual(self.stm32.state, rl.ESTOP)  # too early: ignored
        for _ in range(21):
            self.clock.t += 0.05
            self.stm32.command()
        self.assertTrue(self.stm32.status['clear_ready'])
        self.stm32.command(clear_estop=True)
        self.assertEqual((self.stm32.state, self.stm32.faults, self.stm32.run_request), (rl.AUTO, 0, False))

    def test_over_limit_stops(self):
        self.stm32.key('a')
        self.stm32.command(41, 0)
        self.assertEqual(self.stm32.state, rl.ESTOP)


class TestEndToEnd(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.radio = Radio()
        self.logs = []
        self.stm32 = FakeStm32(clock=self.clock, log=self.logs.append)
        self.bridge = Bridge(self.radio.orin, self.stm32, rl.RemoteLogic(5, 3), Joy(), self.logs.append,
                             clock=self.clock)
        self.handheld = Handheld(0)
        self.start = self.clock()
        self.events = 0

    def tearDown(self):
        self.handheld.close()
        self.bridge.joy.close()

    def run_for(self, seconds, connected=True, drive=0, turn=0, buttons=()):
        """Both ends at 10 ms steps, as handheld_pc.py and orin_bridge.py loop."""
        mask = sum(1 << SLOT[name] for name in buttons)
        for _ in range(round(seconds / 0.01)):
            now_ms = int((self.clock() - self.start) * 1000)
            self.handheld.sample(connected, drive, turn, mask)
            line = self.handheld.take_line(now_ms)
            if line:
                self.radio.handheld.write(line)
            self.handheld.feed(self.radio.handheld.read(1024), now_ms)
            self.events |= self.handheld.poll(now_ms)
            self.bridge.step()
            self.clock.t += 0.01

    def take_events(self):
        events, self.events = self.events, 0
        return events

    def test_field_test_sequence(self):
        self.run_for(0.5)
        self.assertEqual(self.take_events(), EV_LINK_UP)  # first reply already says "waiting": one rumble
        self.assertEqual(self.handheld.info().indicator, rl.IND_WAITING)  # R12 A not pressed yet
        self.assertEqual(self.stm32.state, rl.STOPPED)

        self.stm32.key('a')  # operator presses A on the R12
        self.run_for(0.3)
        self.assertEqual(self.handheld.info().indicator, rl.IND_WAITING)
        self.run_for(0.3, drive=100)
        self.assertEqual(self.stm32.target, [0.0, 0.0])  # motion is blocked before manual ARM
        self.run_for(0.15, drive=100, buttons=['LB'])  # LB with the stick pushed is refused
        self.run_for(0.2)
        self.assertIn('ARM 拒絕：先回 JOY 並放開搖桿', self.logs)
        self.assertEqual(self.bridge.authority.phase, 'LOCKED')
        self.run_for(0.15, buttons=['LB'])  # F710 LB does what typing ARM on the Orin does
        self.run_for(0.8)
        self.assertTrue(self.take_events() & EV_INDICATOR)
        self.assertEqual(self.handheld.info().indicator, rl.IND_JOY)

        self.run_for(0.3, drive=100)
        self.assertEqual(self.stm32.target, [5.0, 5.0])
        self.run_for(0.12)  # stick released: zero within one packet period plus one step
        self.assertEqual(self.stm32.target, [0.0, 0.0])

        self.run_for(0.15, drive=100, buttons=['RB'])
        self.run_for(0.3, drive=100)  # ORIN ignores the stick
        self.assertEqual(self.stm32.target, [3.0, 3.0])
        self.assertEqual(self.handheld.info().indicator, rl.IND_ORIN)

        self.run_for(0.15, buttons=['RB'])
        self.run_for(0.3)
        self.assertEqual((self.bridge.logic.mode, self.stm32.target), (rl.JOY, [0.0, 0.0]))

        self.run_for(0.3, drive=-100, turn=100)
        self.assertEqual(self.stm32.target, [0.0, -5.0])
        self.run_for(0.3, connected=False)  # F710 switched off: stop at once
        self.assertEqual(self.stm32.target, [0.0, 0.0])
        self.assertIn('遙控器失聯：停車，回到 JOY', self.logs)
        self.run_for(0.3)
        self.assertEqual(self.stm32.state, rl.AUTO)
        self.run_for(0.3, drive=100)
        self.assertEqual(self.stm32.target, [0.0, 0.0])  # reconnection cannot resume motion
        self.run_for(0.2)
        for key in 'ARM\n':  # the keyboard still works as a backup
            self.bridge.key(key)
        self.run_for(0.3)
        self.run_for(0.3, drive=100)
        self.assertEqual(self.stm32.target, [5.0, 5.0])
        self.run_for(0.2)

        self.take_events()
        self.radio.up = False  # LoRa cut: both ends notice within 0.3 s and the tracks stay stopped
        self.run_for(0.5)
        self.assertTrue(self.take_events() & EV_LINK_LOST)
        self.assertEqual(self.stm32.target, [0.0, 0.0])
        self.radio.up = True
        self.run_for(0.3)
        self.assertTrue(self.take_events() & EV_LINK_UP)

        self.run_for(0.15, buttons=['BACK'])
        self.run_for(0.3)
        self.assertEqual((self.stm32.state, self.stm32.faults), (rl.ESTOP, 512))
        self.assertEqual(self.handheld.info().indicator, rl.IND_FAULT)

        self.run_for(0.15, buttons=['START'])  # F710 START right away: the STM32 is not ready to clear
        self.run_for(0.2)
        self.assertIn('START 拒絕：STM32 還不能解除（故障沒排除、R12 搖桿沒回中、引擎開關不在 HOLD 或編碼器異常）',
                      self.logs)
        self.run_for(1.0)
        self.run_for(0.15, drive=100, buttons=['START'])  # stick pushed: refused
        self.run_for(0.2)
        self.assertIn('START 拒絕：先放開搖桿', self.logs)
        self.assertEqual(self.stm32.state, rl.ESTOP)
        self.run_for(0.15, buttons=['START'])
        self.run_for(0.3)
        self.assertIn('ESTOP 已解除：STM32 回到 AUTO 待命，放開搖桿按 LB 做 ARM', self.logs)
        self.assertEqual(self.stm32.state, rl.AUTO)
        self.assertEqual(self.handheld.info().indicator, rl.IND_WAITING)
        self.run_for(0.3, drive=100)
        self.assertEqual(self.stm32.target, [0.0, 0.0])  # cleared is not armed
        self.run_for(0.15, buttons=['LB'])
        self.run_for(0.8)
        self.run_for(0.3, drive=100)
        self.assertEqual(self.stm32.target, [5.0, 5.0])

    def test_frame_matches_independent_packer(self):
        handheld = Handheld(0)
        handheld.sample(True, 0, 0, 1 << SLOT['START'])
        handheld.sample(True, 0, 0, 0)
        handheld.sample(True, -37, 55, (1 << SLOT['START']) | (1 << SLOT['RB']))
        counters = [0] * 10
        counters[SLOT['START']], counters[SLOT['RB']] = 2, 1
        expected = pack_input(1, -37, 55, True, counters).hex().upper().encode()
        self.assertEqual(handheld.take_line(0), b'+RCV=1,32,' + expected + b',0,0\r\n')
        handheld.close()


if __name__ == '__main__':
    unittest.main()
