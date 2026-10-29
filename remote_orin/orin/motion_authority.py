"""Nonblocking, explicit operator permission for the STM32 ARM protocol."""


class MotionAuthority:
    def __init__(self, log):
        self.log = log
        self.phase = 'LOCKED'
        self.requested = False
        self.arm_sequence = None
        self.deadline = 0.0
        self.sent_at = 0.0

    @property
    def active(self):
        return self.phase == 'ACTIVE'

    def request(self):
        if not self.active:
            self.requested = True

    def revoke(self, reason):
        if self.phase != 'LOCKED':
            self.log('ARM 已取消：' + reason + '；需重新按 LB 或輸入 ARM')
        self.phase = 'LOCKED'
        self.arm_sequence = None

    def update(self, status, fresh, rc, mode, engine_kill, now):
        """Return True only when one zero-valued ARM frame should be sent."""
        requested, self.requested = self.requested, False
        safe = (fresh and status is not None and status['state'] == 3
                and not status['faults'] and status.get('explicit_arm_supported', False)
                and bool(rc.link_ok) and not engine_kill)
        if not safe:
            self.revoke('遙測過期、失聯、非 AUTO、故障或韌體不支援 ARM')
            if requested:
                self.log('ARM 拒絕：需遙控正常、STM32 AUTO、無故障及新版 ARM 韌體')
            return False
        # The MCU renews lease on each telemetry frame; it is a rolling command token,
        # not a boot/session identifier. MowerLink echoes the latest token per command.
        if self.active:
            if not status.get('run_request', False):
                self.revoke('STM32 已取消行走許可')
            return False
        if requested and self.phase == 'LOCKED':
            if mode != 'JOY' or not rc.sticks_neutral:
                self.log('ARM 拒絕：先回 JOY 並放開搖桿')
                return False
            # A surviving MCU run bit still needs a new human permission after link loss.
            if status.get('run_request', False):
                self.phase = 'ACTIVE'
                self.log('ARM 已確認：STM32 行走許可有效，搖桿可控制')
                return False
            self.phase = 'PREPARING'
            self.deadline = now + 10.0
            self.log('ARM 準備：維持零速，等待 STM32 arm_ready')
        if self.phase in ('PREPARING', 'WAIT_ACK'):
            if mode != 'JOY' or not rc.sticks_neutral:
                self.revoke('握手期間搖桿需保持置中及 JOY')
                return False
            if now >= self.deadline:
                self.revoke('握手逾時，不自動重送 ARM')
                return False
        if self.phase == 'PREPARING' and status.get('arm_ready', False):
            return True
        if self.phase == 'WAIT_ACK':
            distance = (status['sequence'] - self.arm_sequence) & 0xffffffff
            if (status.get('run_request', False) and status['_received_at'] > self.sent_at
                    and distance < 0x80000000):
                self.phase = 'ACTIVE'
                self.log('ARM 成功：STM32 已回覆行走許可，搖桿可控制')
        return False

    def arm_sent(self, sequence, now):
        self.arm_sequence = sequence
        self.sent_at = now
        self.phase = 'WAIT_ACK'
        self.deadline = now + 1.0
