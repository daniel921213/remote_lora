#!/usr/bin/env python3
"""LoRa 收 F710 → 照 orin_bridge 的按鍵規則 → 發 ROS 2 topic 給組員的 stm_bridge。這支不開 STM32 串口。

先跑組員的 main_control 和 stm_bridge，再在 source 過 ROS 2 的終端機執行：
    python3 lora_ros_node.py --lora-port socket://127.0.0.1:8789   跟地面站共用 LoRa（先開 lora_mux.py orin）
    python3 lora_ros_node.py --lora-port /dev/lora_gs              自己獨占 LoRa

按鍵跟 orin_bridge.py 一樣（共用 remote_logic.py）：
    搖桿        JOY 模式開車 → joint_command（左右輪 rad/s）
    RB          切換 JOY / ORIN（ORIN 先用固定測試轉速）→ /remote/mode
    B           停車，回到 JOY
    BACK        呼叫 /estop（這條路只會 ESTOP，不會熄火），之後一直送 0 直到按 START
    START       STM32 在 ESTOP 時呼叫 /release_state 解除；也會解開 BACK 的鎖
    LB          STM32 在 AUTO 時呼叫 /release_state 申請 ARM（main_control 進 AUTO 也會自己 ARM）
STM32 狀態從 /stm/state、/stm/connected 收回來。另外照舊發 /remote/joy、/remote/connected、
/remote/sticks_neutral；/remote/joy 的按鍵是「按一下只出現一筆」，不是按住的狀態。
"""

import argparse
import math
import os
import subprocess
import sys
import time
from dataclasses import dataclass

from joy_lib import Joy
from remote_logic import (AUTO, ESTOP, EV_ENGINE_OFF, EV_ENGINE_ON, EV_UNLOCK, IND_FAULT, STATE_NAMES_ZH,
                          RemoteLogic)

BUTTON_NAMES = ('START', 'BACK', 'X', 'Y', 'A', 'B', 'RB', 'LB', 'UP', 'DOWN')
LEFT_JOINT, RIGHT_JOINT = 'left_wheel_joint', 'right_wheel_joint'  # stm_bridge 預設的輪子名稱
RPM_TO_RAD_S = 2.0 * math.pi / 60.0  # joint_command 用 rad/s，stm_bridge 會再換回 RPM
DISPLAY_PERIOD = 0.2


@dataclass(frozen=True)
class Sample:
    connected: bool
    neutral: bool
    drive: float
    turn: float
    buttons: tuple
    events: int = 0


class Remote:
    """每叫一次 step()：收 LoRa、算 JOY/ORIN、回手把，要發的東西交給 out（ROS 節點，測試時換成假的）。"""

    def __init__(self, port, joy, logic, out, publish_hz=20.0, clock=time.monotonic):
        if not math.isfinite(publish_hz) or not 5 <= publish_hz <= 100:
            raise ValueError('publish_hz must be within 5..100')
        self.port, self.joy, self.logic, self.out, self.clock = port, joy, logic, out, clock
        self.period = 1.0 / publish_hz
        self.start = clock()
        self.state = None   # STM32 狀態；None＝stm_bridge 沒在跑或它沒連上 STM32
        self.held = False   # 按過 BACK：START/LB 之前一律送 0，就算 /estop 沒成功也一樣
        self.published_at = None
        self.last_connected = None
        self.last_mode = None
        self.shown, self.shown_at = None, None
        self.closed = False

    def step(self):
        if self.closed:
            return None
        now = self.clock()
        now_ms = int((now - self.start) * 1000)
        # timeout=0: whatever has arrived. A socket:// port reports in_waiting only as 0 or 1.
        self.joy.feed(self.port.read(4096), now_ms)
        rc = self.joy.poll(now_ms)
        events = int(rc.events)
        command = self.logic.step(rc, self.state)
        for message in command.messages:
            if not message.startswith('BACK'):  # BACK 在這條路只有 ESTOP，下面自己印
                self.out.log(message)

        if events & EV_ENGINE_OFF:
            self.held = True
            self.out.log('BACK：停車，呼叫 /estop 鎖住（這條路不會熄火；用 START 解除）')
            self.out.estop()
        # START 和 LB 都是叫 /release_state，先看 STM32 狀態，按錯鍵就不送
        if events & EV_ENGINE_ON:
            if not rc.sticks_neutral:
                self.out.log('START 拒絕：先放開搖桿')
            else:
                if self.held:
                    self.held = False
                    self.out.log('START：解除 BACK 的停車鎖')
                if self.state == ESTOP:
                    self.out.log('START：呼叫 /release_state 解除 ESTOP')
                    self.out.release()
                else:
                    self.out.log('START：STM32 沒回應，不能解除' if self.state is None
                                 else 'START：STM32 沒有在 ESTOP，不用解除')
        if events & EV_UNLOCK:
            if not rc.sticks_neutral:
                self.out.log('LB 拒絕：先放開搖桿')
            elif self.state != AUTO:
                self.out.log('LB：STM32 沒回應，不能 ARM' if self.state is None
                             else 'LB：STM32 不在 AUTO，不能 ARM')
            elif self.held:
                self.out.log('LB 拒絕：BACK 鎖住中，先按 START')
            else:
                self.out.log('LB：呼叫 /release_state 申請 ARM')
                self.out.release()

        left, right, indicator = command.left, command.right, command.indicator
        if self.held:
            left = right = 0.0
            indicator = IND_FAULT
        # 回手把（F710 會依狀態震動）
        self.joy.set_status(indicator, 0 if self.state is None else self.state)
        reply = self.joy.take_reply()
        if reply and self.port.write(reply) != len(reply):
            raise ConnectionError('Incomplete LoRa acknowledgement write')

        # 平常 20 Hz 發一次；要停車、有按鍵、連線狀態變了就馬上發
        connected = bool(rc.link_ok)
        if (command.urgent or events or connected != self.last_connected
                or self.published_at is None or now - self.published_at >= self.period):
            self.out.wheels(left, right)
            self.out.sample(Sample(connected, bool(rc.sticks_neutral) if connected else False,
                                   float(rc.drive) if connected else 0.0,
                                   float(rc.turn) if connected else 0.0,
                                   tuple(int(connected and bool(events & (1 << i)))
                                         for i in range(len(BUTTON_NAMES))), events))
            self.logic.sent()
            self.published_at = now
        self.last_connected = connected
        if self.logic.mode != self.last_mode:
            self.out.mode(self.logic.mode)
            self.last_mode = self.logic.mode
        self.display(rc, left, right, now)
        return command

    def display(self, rc, left, right, now):
        key = (self.logic.mode, bool(rc.link_ok), left, right, self.state, self.held)
        if key == self.shown or (self.shown_at is not None and now - self.shown_at < DISPLAY_PERIOD):
            return
        stm32 = '沒回應' if self.state is None else STATE_NAMES_ZH.get(self.state, self.state)
        self.out.log(f'模式 {self.logic.mode}｜遙控 {"正常" if rc.link_ok else "失聯"}｜'
                     f'送出 左 {left:+.1f} 右 {right:+.1f} RPM｜STM32 {stm32}'
                     + ('｜BACK 鎖住中，按 START 解除' if self.held else ''))
        self.shown, self.shown_at = key, now

    def close(self, publish=True):
        if self.closed:
            return
        self.closed = True
        try:
            if publish:
                self.out.wheels(0.0, 0.0)
                self.out.sample(Sample(False, False, 0.0, 0.0, (0,) * len(BUTTON_NAMES)))
        finally:
            try:
                self.joy.close()
            finally:
                self.port.close()


def open_lora(name):
    """Only the requested LoRa port is opened. Never import or instantiate MowerLink.
    A socket:// name is lora_mux.py's remote port: the LoRa shared with the ground station."""
    import serial
    if '://' in name:
        return serial.serial_for_url(name, timeout=0, write_timeout=0.1)
    if os.name == 'posix':
        owner = subprocess.run(['fuser', os.path.realpath(name)], capture_output=True, text=True)
        if owner.returncode == 0:
            raise RuntimeError('LoRa port is busy (PID %s). Stop its previous owner first.'
                               % owner.stdout.strip())
    port = serial.Serial(None, 115200, timeout=0, write_timeout=0.1)
    port.dtr = port.rts = False
    if os.name == 'posix':
        port.exclusive = True
    port.port = name
    try:
        port.open()
        port.reset_input_buffer()
    except BaseException:
        port.close()
        raise
    return port


def main(argv=None):
    parser = argparse.ArgumentParser(description='LoRa 收 F710，發 joint_command 和 /estop、/release_state 給 stm_bridge')
    parser.add_argument('--lora-port', required=True,
                        help='LoRa 模組的串口（例如 /dev/lora_gs），或分流程式 socket://127.0.0.1:8789')
    parser.add_argument('--publish-hz', type=float, default=20.0, help='joint_command 發送頻率，預設 20')
    parser.add_argument('--max-rpm', type=float, default=10.0, help='JOY 搖桿推到底的轉速，預設 10，最大 40')
    parser.add_argument('--auto-rpm', type=float, default=5.0,
                        help='ORIN 模式的測試轉速（左右相同、往前），預設 5；0 表示原地不動')
    args, ros_args = parser.parse_known_args(argv)
    if not math.isfinite(args.publish_hz) or not 5 <= args.publish_hz <= 100:
        parser.error('--publish-hz must be within 5..100')
    try:
        logic = RemoteLogic(args.max_rpm, args.auto_rpm)
    except ValueError as error:
        parser.error(str(error))
    try:
        import rclpy
        from rclpy.clock import Clock, ClockType
        from rclpy.executors import ExternalShutdownException
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
        from sensor_msgs.msg import Joy as JoyMessage, JointState
        from std_msgs.msg import Bool, String, UInt8
        from std_srvs.srv import Trigger
    except ImportError as error:
        parser.exit(1, 'ROS 2 environment is missing: %s\nSource ROS 2 setup.bash first.\n' % error)

    class LoraRosNode(Node):
        def __init__(self, port):
            super().__init__('lora_remote')
            qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.VOLATILE)
            # 跟 stm_bridge 的 /stm/* 一樣：晚開的程式也拿得到最後一筆
            latched = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                                 reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.wheels_pub = self.create_publisher(JointState, 'joint_command', 10)
            self.joy_pub = self.create_publisher(JoyMessage, '/remote/joy', qos)
            self.connected_pub = self.create_publisher(Bool, '/remote/connected', qos)
            self.neutral_pub = self.create_publisher(Bool, '/remote/sticks_neutral', qos)
            self.mode_pub = self.create_publisher(String, '/remote/mode', latched)
            self.estop_client = self.create_client(Trigger, '/estop')
            self.release_client = self.create_client(Trigger, '/release_state')
            self.stm_state = None
            self.stm_connected = False
            self.stm_alive = None
            self.other_wheel_publishers = 0
            self.failure = None
            self.remote = Remote(port, Joy(), logic, self, args.publish_hz)
            self.create_subscription(UInt8, '/stm/state', self.on_stm_state, latched)
            self.create_subscription(Bool, '/stm/connected', self.on_stm_connected, latched)
            # 計時用系統時鐘，ROS 的 /clock 停了也照跑
            steady = Clock(clock_type=ClockType.STEADY_TIME)
            self.timer = self.create_timer(0.01, self.tick, clock=steady)
            self.peer_timer = self.create_timer(1.0, self.check_peers, clock=steady)
            self.log(f'LoRa {args.lora_port} → joint_command；STM32 串口不在這支開')
            self.log(f'JOY 搖桿推到底 {args.max_rpm:g} RPM；ORIN 測試轉速 {args.auto_rpm:g} RPM')
            self.log('R12 進 AUTO 後 main_control 會自動 ARM（也可按 LB）；BACK 鎖住後按 START 解除')
            self.check_peers()

        # ---- Remote 叫的這幾個：真的發 topic、叫 service ----
        def log(self, text):
            self.get_logger().info(text)

        def wheels(self, left, right):
            message = JointState()
            message.header.stamp = self.get_clock().now().to_msg()
            message.name = [LEFT_JOINT, RIGHT_JOINT]
            message.velocity = [left * RPM_TO_RAD_S, right * RPM_TO_RAD_S]
            self.wheels_pub.publish(message)

        def sample(self, sample):
            self.connected_pub.publish(Bool(data=sample.connected))
            self.neutral_pub.publish(Bool(data=sample.neutral))
            message = JoyMessage()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = 'f710'
            message.axes = [sample.drive, sample.turn]
            message.buttons = list(sample.buttons)
            self.joy_pub.publish(message)

        def mode(self, mode):
            self.mode_pub.publish(String(data=mode))

        def estop(self):
            self.call(self.estop_client, '/estop')

        def release(self):
            self.call(self.release_client, '/release_state')

        def call(self, client, name):
            if not client.service_is_ready():
                self.get_logger().error(f'{name} 叫不到：stm_bridge 沒在跑？')
                return
            future = client.call_async(Trigger.Request())
            future.add_done_callback(lambda done: self.on_reply(name, done))

        def on_reply(self, name, done):
            try:
                result = done.result()
            except Exception as error:
                self.get_logger().error(f'{name} 失敗：{error}')
                return
            if result.success:
                self.get_logger().info(f'{name} 成功：{result.message}')
            else:
                self.get_logger().warning(f'{name} 被拒絕：{result.message}')

        # ---- STM32 狀態 ----
        def on_stm_state(self, message):
            self.stm_state = int(message.data)
            self.update_state()

        def on_stm_connected(self, message):
            self.stm_connected = bool(message.data)
            self.update_state()

        def update_state(self):
            self.remote.state = self.stm_state if self.stm_alive and self.stm_connected else None

        def check_peers(self):
            # 每秒看一次：stm_bridge 還在不在、有沒有別的程式也在發 joint_command
            alive = self.count_publishers('/stm/state') > 0
            if alive != self.stm_alive:
                if alive:
                    self.get_logger().info('stm_bridge 在線')
                else:
                    self.get_logger().warning('找不到 stm_bridge（沒人發 /stm/state）：車子不會動')
                self.stm_alive = alive
                self.update_state()
            others = self.count_publishers('joint_command') - 1
            if others != self.other_wheel_publishers:
                if others > 0:
                    self.get_logger().warning(f'還有 {others} 個程式也在發 joint_command（例如 diff_controller），'
                                              '它們送速度時會跟遙控器打架')
                self.other_wheel_publishers = others

        def tick(self):
            try:
                self.remote.step()
            except Exception as error:
                self.failure = error
                self.timer.cancel()
                hint = '（分流程式關掉了？）' if '://' in args.lora_port else ''
                self.get_logger().error(f'LoRa 讀寫失敗{hint}：{error}')
                self.remote.close(publish=rclpy.ok())

    rclpy.init(args=ros_args)
    node, port = None, None
    exit_code = 0
    try:
        port = open_lora(args.lora_port)
        node = LoraRosNode(port)
        while rclpy.ok() and node.failure is None:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.failure is not None:
            exit_code = 1
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as error:
        print('lora_ros_node: %s' % error, file=sys.stderr)
        exit_code = 1
    finally:
        if node is not None:
            try:
                node.timer.cancel()
                node.remote.close(publish=rclpy.ok())
            finally:
                node.destroy_node()
        elif port is not None:
            port.close()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
