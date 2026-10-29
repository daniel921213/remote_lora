"""ARM acceptance, serial wire compatibility, and permission loss regressions."""
import binascii
import struct
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'orin'))
from motion_authority import MotionAuthority
from new_mower_link import PacketParser, command_packet, decode_packet
from test_system import Rc


def status_frame(sequence=23, health=222):
    packet = bytearray(32)
    struct.pack_into('<BBBBIIhhHHBBH', packet, 0, 165, 90, 1, 129,
                     sequence, 42, 123, -234, 1530, 1460, 3, health, 0)
    struct.pack_into('<HH', packet, 25, 1100, 1900)
    packet[29] = 3
    struct.pack_into('<H', packet, 30, binascii.crc_hqx(packet[:30], 0xffff))
    return bytes(packet)


class TestProtocol(unittest.TestCase):
    def test_raw_status_layout_and_health(self):
        result = decode_packet(status_frame())
        self.assertEqual(result['rpm'], [1.23, -2.34])
        self.assertEqual(result['pwm'], [1530, 1460])
        self.assertEqual(result['fc_pwm'], [1100, 1900])
        self.assertTrue(result['explicit_arm_supported'] and result['arm_ready'])
        self.assertFalse(result['run_request'])

    def test_fragmentation_noise_and_corrupt_crc(self):
        parser = PacketParser()
        corrupt = bytearray(status_frame()); corrupt[12] ^= 1
        self.assertEqual(parser.feed(b'noise' + corrupt + status_frame()[:13]), [])
        self.assertEqual(len(parser.feed(status_frame()[13:])), 1)

    def test_arm_packet_is_separate_zero_command(self):
        packet = command_packet(7, 42, flags=8)
        self.assertEqual(packet[12:18], b'\0\0\0\0\0\x08')
        self.assertEqual(struct.unpack_from('<H', packet, 30)[0], binascii.crc_hqx(packet[:30], 0xffff))
        for flags, left in ((8, 1), (12, 0), (16, 0)):
            with self.assertRaises(ValueError):
                command_packet(7, 42, flags=flags, left=left)


class TestAuthority(unittest.TestCase):
    def setUp(self):
        self.logs = []
        self.auth = MotionAuthority(self.logs.append)
        self.status = decode_packet(status_frame())
        self.status['_received_at'] = 1.0

    def update(self, now=1.0, fresh=True, rc=None, mode='JOY', kill=False):
        return self.auth.update(self.status, fresh, rc or Rc(), mode, kill, now)

    def activate(self):
        self.auth.request(); self.assertTrue(self.update())
        self.auth.arm_sent(0xffffffff, 1.0)
        self.status.update(sequence=0, run_request=True, _received_at=1.1)
        self.assertFalse(self.update(1.1))
        self.assertTrue(self.auth.active)

    def test_no_implicit_arm_and_ack_wrap(self):
        self.assertFalse(self.update())
        self.assertFalse(self.auth.active)
        self.activate()

    def test_interlock_refusals(self):
        for kwargs in ({'fresh': False}, {'rc': Rc(link_ok=False)},
                       {'rc': Rc(neutral=False)}, {'mode': 'ORIN'}, {'kill': True}):
            self.auth.request(); self.assertFalse(self.update(**kwargs))
            self.assertEqual(self.auth.phase, 'LOCKED')
        for field, value in (('faults', 1), ('state', 4), ('explicit_arm_supported', False)):
            old = self.status[field]; self.status[field] = value
            self.auth.request(); self.assertFalse(self.update())
            self.assertEqual(self.auth.phase, 'LOCKED'); self.status[field] = old

    def test_no_arm_retry_after_timeout(self):
        self.auth.request(); self.assertTrue(self.update())
        self.auth.arm_sent(23, 1.0)
        self.assertFalse(self.update(1.2))
        self.assertFalse(self.update(2.1))
        self.assertEqual(self.auth.phase, 'LOCKED')
        self.assertFalse(self.update(3.0))

    def test_ack_must_be_new_and_not_old_sequence(self):
        self.auth.request(); self.update(); self.auth.arm_sent(24, 1.0)
        self.status.update(run_request=True, _received_at=1.1)
        self.update(1.1); self.assertFalse(self.auth.active)
        self.status.update(sequence=24, _received_at=1.0)
        self.update(1.2); self.assertFalse(self.auth.active)
        self.status['_received_at'] = 1.2
        self.update(1.2); self.assertTrue(self.auth.active)

    def test_disconnect_and_mcu_run_revocation_require_manual_permission(self):
        self.activate()
        self.update(1.2, rc=Rc(link_ok=False)); self.assertFalse(self.auth.active)
        self.update(1.3); self.assertFalse(self.auth.active)
        self.auth.request(); self.update(1.4); self.assertTrue(self.auth.active)
        self.status['run_request'] = False
        self.update(1.5); self.assertFalse(self.auth.active)

    def test_rolling_lease_during_arm_and_active_does_not_revoke(self):
        self.status['arm_ready'] = False
        self.auth.request(); self.assertFalse(self.update())
        self.status.update(lease=43, arm_ready=True, _received_at=1.1)
        self.assertTrue(self.update(1.1))
        self.auth.arm_sent(24, 1.1)
        self.status.update(lease=44, sequence=24, run_request=True, _received_at=1.2)
        self.update(1.2); self.assertTrue(self.auth.active)
        for i, lease in enumerate((45, 46, 0xffffffff, 0, 1)):
            self.status.update(lease=lease, sequence=25+i, _received_at=1.3+i*.1)
            self.update(1.3+i*.1, rc=Rc(neutral=False, drive=1.0))
            self.assertTrue(self.auth.active)

    def test_active_stale_fault_and_mode_change_revoke(self):
        for change in ({'state': 1}, {'faults': 4}, {'explicit_arm_supported': False}):
            self.setUp(); self.activate(); self.status.update(change)
            self.update(1.2); self.assertFalse(self.auth.active)
        self.setUp(); self.activate()
        self.update(1.2, fresh=False); self.assertFalse(self.auth.active)


if __name__ == '__main__':
    unittest.main()
