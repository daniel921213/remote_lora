"""JOY/ORIN rules for the first field test: F710 requests -> left/right track RPM.

The STM32 stays in AUTO (entered with the R12 A button) and only follows the Orin's RPM
commands. This decides whether those commands come from the F710 sticks (JOY) or from the
autonomous side (ORIN). There is no navigation program yet, so ORIN drives a fixed test
speed. The RB "release and wait for the ground station" flow from the spec comes later.
"""

from collections import namedtuple

# STM32 states (0x81 telemetry byte 20)
UNCONFIGURED, STOPPED, MANUAL, AUTO, ESTOP, SBUS_TEST = range(6)
STATE_NAMES_ZH = {UNCONFIGURED: '未設定', STOPPED: '停止', MANUAL: 'MANUAL', AUTO: 'AUTO', ESTOP: 'ESTOP',
                  SBUS_TEST: 'SBUS 測試'}

# joy_api.h events
EV_ENGINE_ON, EV_ENGINE_OFF, EV_GEAR_BLADE, EV_GEAR_FAN = 1 << 0, 1 << 1, 1 << 2, 1 << 3
EV_GEAR_NEUTRAL, EV_PARK, EV_CONTROL_TOGGLE, EV_UNLOCK = 1 << 4, 1 << 5, 1 << 6, 1 << 7
EV_BLADE_UP, EV_BLADE_DOWN, EV_LINK_LOST, EV_LINK_RESTORED, EV_OVERFLOW = 1 << 8, 1 << 9, 1 << 10, 1 << 11, 1 << 12

# Reply indicator: the F710 rumbles when it changes (RC_LORA_PROTOCOL.md 4.3).
IND_ORIN, IND_JOY, IND_WAITING, IND_FAULT = 0, 1, 2, 3

JOY, ORIN = 'JOY', 'ORIN'
STM32_LIMIT_RPM = 40.0  # the STM32 treats any target beyond its configured +/-40 as invalid and stops
KILL_REPEATS = 3        # ENGINE_KILL goes out in this many consecutive commands

UNSUPPORTED = [(EV_ENGINE_ON, 'START 引擎通電'), (EV_GEAR_BLADE, 'X 刀具檔'), (EV_GEAR_FAN, 'Y 風機檔'),
               (EV_GEAR_NEUTRAL, 'A 離合空檔'), (EV_BLADE_UP, '十字鍵上 刀頭上升'),
               (EV_BLADE_DOWN, '十字鍵下 刀頭下降')]

Command = namedtuple('Command', 'left right engine_kill indicator urgent messages')


class RemoteLogic:
    def __init__(self, max_rpm, auto_rpm):
        if not 0 < max_rpm <= STM32_LIMIT_RPM:
            raise ValueError(f'max_rpm 要在 0～{STM32_LIMIT_RPM:g} 之間（不含 0）')
        if not 0 <= auto_rpm <= max_rpm:
            raise ValueError('auto_rpm 要在 0～max_rpm 之間')
        self.max_rpm = max_rpm
        self.auto_rpm = auto_rpm
        self.mode = JOY
        self.need_neutral = True  # sticks must be centred before JOY drives
        self.kill_left = 0
        self.last = (0.0, 0.0)

    def _to_joy(self):
        self.mode = JOY
        self.need_neutral = True

    def step(self, rc, state):
        """rc: joy_api output (link_ok, sticks_neutral, drive, turn, events).
        state: STM32 state from fresh telemetry, or None when there is none."""
        events, messages = rc.events, []
        auto = state == AUTO

        if events & EV_LINK_RESTORED:
            messages.append('遙控器連上')
        if events & EV_LINK_LOST:
            messages.append('遙控器失聯：停車，回到 JOY')
        if events & EV_OVERFLOW:
            messages.append('警告：遙控資料可能漏掉（OVERFLOW）')
        if events & EV_ENGINE_OFF:
            self.kill_left = KILL_REPEATS
            messages.append('BACK：熄火並鎖定停車（要用 R12 才能恢復）')
        if events & EV_PARK:
            self._to_joy()
            messages.append('B：停車，回到 JOY（搖桿先回中間才會動）')
        elif events & EV_CONTROL_TOGGLE:
            if not auto:
                messages.append('RB：STM32 不在 AUTO，不切換（先按 R12 的 A）')
            elif self.mode == JOY:
                self.mode = ORIN
                messages.append(f'RB：切到 ORIN（自動），測試速度 {self.auto_rpm:g} RPM')
            else:
                self._to_joy()
                messages.append('RB：切到 JOY（手動），搖桿先回中間才會動')
        if events & EV_UNLOCK:
            messages.append('LB：解除鎖定要用 R12，Orin 沒有解鎖權')
        for bit, name in UNSUPPORTED:
            if events & bit:
                messages.append(f'{name}：這次測試還不支援，忽略')

        if not rc.link_ok or not auto:
            if self.mode == ORIN:
                messages.append('遙控器失聯或 STM32 離開 AUTO：回到 JOY')
            self._to_joy()
            left = right = 0.0
        elif self.mode == ORIN:
            left = right = float(self.auto_rpm)
        else:
            if self.need_neutral and rc.sticks_neutral:
                self.need_neutral = False
            left, right = (0.0, 0.0) if self.need_neutral else self.mix(rc.drive, rc.turn)

        engine_kill = self.kill_left > 0
        if engine_kill:
            left = right = 0.0
        urgent = engine_kill or (self.last != (0.0, 0.0) and (left, right) == (0.0, 0.0))
        self.last = (left, right)
        return Command(left, right, engine_kill, self.indicator(state), urgent, messages)

    def mix(self, drive, turn):
        """drive/turn -1..1 (forward, right positive) -> left/right RPM, steering ratio kept."""
        left, right = drive + turn, drive - turn
        scale = max(1.0, abs(left), abs(right))
        return round(left / scale * self.max_rpm, 2), round(right / scale * self.max_rpm, 2)

    def indicator(self, state):
        if state is None or state in (UNCONFIGURED, ESTOP, SBUS_TEST):
            return IND_FAULT
        if state != AUTO:
            return IND_WAITING
        return IND_ORIN if self.mode == ORIN else IND_JOY

    def sent(self):
        """Call after each command actually written to the STM32."""
        if self.kill_left:
            self.kill_left -= 1
