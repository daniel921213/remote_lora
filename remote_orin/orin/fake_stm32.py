"""Stand-in for the teammate's MowerLink when no STM32 is connected.

Same methods and status fields the bridge uses, with the documented rules that matter to it
(Jetson_STM32介面使用說明 sections 1, 7, 10): only AUTO follows RPM commands, AUTO stops when
commands pause for 200 ms, ESTOP/ENGINE_KILL latch, targets beyond +/-40 RPM stop the chassis,
and motion commands need telemetry younger than 150 ms. It is not a model of the motors.

Keys while orin_bridge.py --fake-stm32 runs: a = R12 A (AUTO), b = R12 B (MANUAL),
c = R12 C (stop tracks), r = recover from ESTOP the way the R12 procedure would.
"""

import math
import time

from remote_logic import AUTO, ESTOP, MANUAL, STATE_NAMES_ZH, STM32_LIMIT_RPM, STOPPED

TELEMETRY_PERIOD = 0.1
COMMAND_TIMEOUT = 0.2
FRESH = 0.15
SLEW_RPM_PER_S = 10.0     # the STM32's target slew limit
COAST_TIME_CONSTANT = 0.2  # crude spin-down after a zero target
FAULT_JETSON, FAULT_JETSON_STOP = 16, 512


class FakeStm32:
    def __init__(self, clock=time.monotonic, log=print, auto_after=None):
        self.clock = clock
        self.log = log
        now = clock()
        self.state = STOPPED
        self.faults = 0
        self.sequence = 0
        self.target = [0.0, 0.0]
        self.rpm = [0.0, 0.0]
        self.last_command = now
        self.last_update = now
        self.next_telemetry = now
        self.auto_at = None if auto_after is None else now + auto_after
        self.status = None
        self.status_at = 0.0

    def _set(self, state, reason):
        if state != self.state:
            self.state = state
            self.log(f'［假 STM32］狀態 → {STATE_NAMES_ZH[state]}（{reason}）')
        if state == AUTO:
            self.last_command = self.clock()  # the timeout runs from entering AUTO

    def _fault(self, bit, reason):
        self.faults |= bit
        self._set(ESTOP, reason)

    def key(self, key):
        if key == 'a':
            if self.state in (STOPPED, MANUAL):
                self._set(AUTO, '模擬按 R12 的 A')
            else:
                self.log(f'［假 STM32］目前是 {STATE_NAMES_ZH[self.state]}，不能進 AUTO（ESTOP 先按 r 恢復）')
        elif key == 'b' and self.state in (STOPPED, AUTO):
            self._set(MANUAL, '模擬按 R12 的 B')
        elif key == 'c' and self.state in (MANUAL, AUTO):
            self._set(STOPPED, '模擬按 R12 的 C')
        elif key == 'r' and self.state == ESTOP:
            self.faults = 0
            self._set(STOPPED, '模擬用 R12 現場恢復')

    def _update(self):
        now = self.clock()
        dt, self.last_update = now - self.last_update, now
        if self.auto_at is not None and now >= self.auto_at:
            self.auto_at = None
            self.key('a')
        if self.state == AUTO and now - self.last_command > COMMAND_TIMEOUT:
            self._fault(FAULT_JETSON, '超過 200 ms 沒收到命令')
        if self.state != AUTO:
            self.target = [0.0, 0.0]
        for i in range(2):
            if self.target[i] == 0:
                self.rpm[i] *= math.exp(-dt / COAST_TIME_CONSTANT)
            else:
                step = SLEW_RPM_PER_S * dt
                self.rpm[i] += max(-step, min(step, self.target[i] - self.rpm[i]))
        if now >= self.next_telemetry:
            self.next_telemetry = now + TELEMETRY_PERIOD
            self.status_at = now
            self.status = {'type': 'status', 'sequence': self.sequence, 'lease': 1, 'state': self.state,
                           'faults': self.faults, 'rpm': [round(r, 2) for r in self.rpm], 'health': 0}

    def poll(self):
        self._update()
        return []

    def command(self, left=0, right=0, hydraulic=0, estop=False, engine_kill=False):
        self._update()
        if estop or engine_kill:
            if self.state != ESTOP:
                self._fault(FAULT_JETSON_STOP, 'Orin 送 ENGINE_KILL' if engine_kill else 'Orin 送 ESTOP')
            return []
        if self.status is None or self.clock() - self.status_at > FRESH:
            raise ConnectionError('No fresh MCU telemetry; motion command not sent.')
        self.sequence += 1
        if abs(left) > STM32_LIMIT_RPM or abs(right) > STM32_LIMIT_RPM:
            self._fault(FAULT_JETSON, '轉速超過 ±40')
        elif self.state == AUTO:
            self.target = [float(left), float(right)]
            self.last_command = self.clock()
        return []

    def close(self):
        pass
