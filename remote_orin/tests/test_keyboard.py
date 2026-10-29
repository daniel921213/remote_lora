"""Exercise the actual keyboard reader, including pasted multi-key input."""
import os
import sys
import unittest
from unittest.mock import Mock, patch
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'common'))
from tools import Keys


class TestKeys(unittest.TestCase):
    def test_windows_preserves_arm_and_default_still_lowercases(self):
        keyboard = Mock()
        keyboard.kbhit.return_value = True
        keyboard.getwch.return_value = 'A'
        with patch('tools.os.name', 'nt'), patch.dict(sys.modules, msvcrt=keyboard):
            self.assertEqual(Keys(preserve_case=True).get(), 'A')
            self.assertEqual(Keys().get(), 'a')

    @unittest.skipUnless(os.name == 'posix', 'requires a POSIX terminal')
    def test_pasted_arm_reaches_bridge_without_an_extra_key(self):
        import pty
        import time
        import termios
        from orin_bridge import Bridge
        master, slave = pty.openpty()
        stream = os.fdopen(os.dup(slave), 'r')
        saved = termios.tcgetattr(slave)
        keys = None
        bridge = Bridge.__new__(Bridge)
        bridge.key_buffer = ''
        bridge.authority = Mock()
        bridge.log = Mock()
        try:
            with patch('tools.sys.stdin', stream):
                keys = Keys(preserve_case=True)
                os.write(master, b'ARM\n')
                received = ''
                deadline = time.monotonic() + .5
                while len(received) < 4 and time.monotonic() < deadline:
                    key = keys.get()
                    if key:
                        received += key
                        bridge.key(key)
                    else:
                        time.sleep(.001)
                self.assertEqual(received, 'ARM\n')
                bridge.authority.request.assert_called_once_with()
                keys.close()
                keys = None
            self.assertEqual(termios.tcgetattr(slave), saved)
        finally:
            if keys:
                keys.close()
            stream.close()
            os.close(master)
            os.close(slave)


if __name__ == '__main__':
    unittest.main()
