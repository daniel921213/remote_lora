#!/usr/bin/env python3
"""Receive the existing F710 LoRa protocol and publish ROS 2 topics. No STM32 access.

Run inside a sourced ROS 2 environment:
    python3 lora_ros_node.py --lora-port /dev/ttyUSB4

Joy axes: [forward, right], normalized -1..1 after the existing deadband.
Joy buttons are ONE-MESSAGE PRESS EVENTS, not held states: the wire only has counters.
Slots: START BACK X Y A B RB LB UP DOWN. This node never executes those actions.
"""

import argparse
import math
import os
import subprocess
import sys
import time
from dataclasses import dataclass

from joy_lib import Joy

BUTTON_NAMES = ('START', 'BACK', 'X', 'Y', 'A', 'B', 'RB', 'LB', 'UP', 'DOWN')
LINK_LOST, LINK_RESTORED, OVERFLOW = 1 << 10, 1 << 11, 1 << 12


def velocity(sample, max_linear, max_angular):
    """ROS Twist: forward m/s, left-positive yaw rad/s (wire turn is right-positive)."""
    if not sample.connected:
        return 0.0, 0.0
    return sample.drive * max_linear, -sample.turn * max_angular


@dataclass(frozen=True)
class Sample:
    connected: bool
    neutral: bool
    drive: float
    turn: float
    buttons: tuple
    events: int = 0


class Receiver:
    """Hardware protocol pump; separate from ROS so real C decoding can be tested offline."""

    def __init__(self, port, joy, publish, publish_hz=20.0, clock=time.monotonic):
        if not math.isfinite(publish_hz) or not 5 <= publish_hz <= 100:
            raise ValueError('publish_hz must be within 5..100')
        self.port, self.joy, self.publish, self.clock = port, joy, publish, clock
        self.period = 1.0 / publish_hz
        self.start = clock()
        self.published_at = None
        self.last_connected = None
        self.closed = False

    def step(self):
        if self.closed:
            return
        now = self.clock()
        now_ms = int((now - self.start) * 1000)
        self.joy.feed(self.port.read(min(self.port.in_waiting or 1, 4096)), now_ms)
        rc = self.joy.poll(now_ms)
        connected = bool(rc.link_ok)
        # Acknowledge LoRa reception, without inventing STM32 state or a driving grant.
        # 2 = released/waiting, 3 = fault; state 0 = unknown/unconfigured.
        self.joy.set_status(2 if connected else 3, 0)
        reply = self.joy.take_reply()
        if reply and self.port.write(reply) != len(reply):
            raise ConnectionError('Incomplete LoRa acknowledgement write')
        events = int(rc.events)
        sample = Sample(connected, bool(rc.sticks_neutral) if connected else False,
                        float(rc.drive) if connected else 0.0,
                        float(rc.turn) if connected else 0.0,
                        tuple(int(connected and bool(events & (1 << i)))
                              for i in range(len(BUTTON_NAMES))), events)
        # Button presses and disconnection are published immediately, not lost between timers.
        if (self.published_at is None or now - self.published_at >= self.period
                or connected != self.last_connected or events):
            self.publish(sample)
            self.published_at = now
        self.last_connected = connected
        return sample

    def close(self, publish=True):
        if self.closed:
            return
        self.closed = True
        try:
            if publish:
                self.publish(Sample(False, False, 0.0, 0.0, (0,) * len(BUTTON_NAMES)))
        finally:
            try:
                self.joy.close()
            finally:
                self.port.close()


def open_lora(name):
    """Only the requested LoRa port is opened. Never import or instantiate MowerLink."""
    import serial
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
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--lora-port', required=True)
    parser.add_argument('--publish-hz', type=float, default=20.0)
    parser.add_argument('--max-linear', type=float, default=0.3, help='Full-forward speed in m/s (default 0.3)')
    parser.add_argument('--max-angular', type=float, default=1.0, help='Full-turn speed in rad/s (default 1.0)')
    args, ros_args = parser.parse_known_args(argv)
    if not math.isfinite(args.publish_hz) or not 5 <= args.publish_hz <= 100:
        parser.error('--publish-hz must be within 5..100')
    if not all(math.isfinite(value) and value > 0 for value in (args.max_linear, args.max_angular)):
        parser.error('--max-linear and --max-angular must be finite and positive')
    try:
        import rclpy
        from rclpy.clock import Clock, ClockType
        from rclpy.executors import ExternalShutdownException
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
        from sensor_msgs.msg import Joy as JoyMessage
        from geometry_msgs.msg import Twist
        from std_msgs.msg import Bool
    except ImportError as error:
        parser.exit(1, 'ROS 2 environment is missing: %s\nSource ROS 2 setup.bash first.\n' % error)

    class LoraRosNode(Node):
        def __init__(self, port):
            super().__init__('lora_remote')
            qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.VOLATILE)
            self.joy_pub = self.create_publisher(JoyMessage, '/remote/joy', qos)
            self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', qos)
            self.connected_pub = self.create_publisher(Bool, '/remote/connected', qos)
            self.neutral_pub = self.create_publisher(Bool, '/remote/sticks_neutral', qos)
            self.previous_connected = None
            self.failure = None
            self.receiver = Receiver(port, Joy(), self.publish_sample, args.publish_hz)
            # Link timeouts must keep working even if /clock or simulated time stops.
            self.timer = self.create_timer(0.01, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
            self.get_logger().info('LoRa %s -> /remote/joy; STM32 serial is NOT opened' % args.lora_port)
            self.get_logger().info('/cmd_vel enabled: max linear %.3f m/s, max angular %.3f rad/s'
                                   % (args.max_linear, args.max_angular))
            self.get_logger().info('axes=[forward,right]; buttons=[%s] are press pulses, not holds'
                                   % ','.join(BUTTON_NAMES))

        def publish_sample(self, sample):
            self.connected_pub.publish(Bool(data=sample.connected))
            self.neutral_pub.publish(Bool(data=sample.neutral))
            message = JoyMessage()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = 'f710'
            message.axes = [sample.drive, sample.turn]
            message.buttons = list(sample.buttons)
            self.joy_pub.publish(message)
            twist = Twist()
            twist.linear.x, twist.angular.z = velocity(sample, args.max_linear, args.max_angular)
            self.cmd_vel_pub.publish(twist)
            if sample.connected != self.previous_connected:
                self.get_logger().info('Remote connected' if sample.connected else 'Remote lost: publishing zeros')
                self.previous_connected = sample.connected
            pressed = [name for name, value in zip(BUTTON_NAMES, sample.buttons) if value]
            if pressed:
                self.get_logger().info('Press: ' + ', '.join(pressed))
            if sample.events & OVERFLOW:
                self.get_logger().warning('LoRa input overflow: some button events may have been merged')

        def tick(self):
            try:
                self.receiver.step()
            except Exception as error:
                self.failure = error
                self.timer.cancel()
                self.get_logger().error('LoRa read/write failed: %s' % error)
                self.receiver.close(publish=rclpy.ok())

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
                node.receiver.close(publish=rclpy.ok())
            finally:
                node.destroy_node()
        elif port is not None:
            port.close()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
