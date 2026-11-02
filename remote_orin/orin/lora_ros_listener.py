#!/usr/bin/env python3
"""Teammate's read-only subscriber example. Prints input; never opens any serial port."""
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

BUTTON_NAMES = ('START', 'BACK', 'X', 'Y', 'A', 'B', 'RB', 'LB', 'UP', 'DOWN')


class Listener(Node):
    def __init__(self):
        super().__init__('remote_joy_example')
        self.connected = False
        self.last_joy = None
        self.warned = False
        self.create_subscription(Joy, '/remote/joy', self.on_joy, 1)
        self.create_subscription(Bool, '/remote/connected', self.on_connected, 1)
        self.create_timer(0.05, self.watchdog, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_connected(self, message):
        self.connected = message.data
        if not self.connected:
            self.stop_request()

    def on_joy(self, message):
        if len(message.axes) != 2 or len(message.buttons) != 10:
            self.get_logger().error('Unexpected /remote/joy layout')
            self.stop_request()
            return
        self.last_joy = time.monotonic()
        self.warned = False
        forward, right = message.axes
        pressed = [name for name, value in zip(BUTTON_NAMES, message.buttons) if value]
        # This example only prints. The teammate integrates with their existing control owner.
        # Check connected AND local message timeout before accepting motion; handle each press once.
        self.get_logger().info('forward=%+.2f right=%+.2f presses=%s' % (forward, right, pressed))

    def watchdog(self):
        if not self.connected or self.last_joy is None or time.monotonic() - self.last_joy > 0.3:
            self.stop_request()

    def stop_request(self):
        if not self.warned:
            self.get_logger().warning('No fresh remote input: request zero speed in your controller')
            self.warned = True


def main():
    rclpy.init()
    node = Listener()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
