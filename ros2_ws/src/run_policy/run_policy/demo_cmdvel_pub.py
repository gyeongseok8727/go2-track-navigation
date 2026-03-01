#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import tty
import termios
import select
from contextlib import contextmanager

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32


@contextmanager
def raw_stdin_if_tty():
    """stdin이 TTY일 때만 raw 모드로 전환 (TTY가 아니면 그대로 진행)"""
    if sys.stdin.isatty():
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            new = termios.tcgetattr(fd)
            new[3] = new[3] & ~(termios.ECHO)
            termios.tcsetattr(fd, termios.TCSANOW, new)
            yield
        finally:
            termios.tcsetattr(fd, termios.TCSANOW, old)
    else:
        yield


def read_key_nonblocking(timeout_s=0.0):
    """키 입력 비동기 읽기 (TTY가 아니면 '')"""
    if not sys.stdin.isatty():
        return ''
    r, _, _ = select.select([sys.stdin], [], [], timeout_s)
    if not r:
        return ''
    ch = sys.stdin.read(1)
    if ch == '\x1b':  # arrow keys
        if select.select([sys.stdin], [], [], 0.0005)[0]:
            ch2 = sys.stdin.read(1)
            if ch2 == '[' and select.select([sys.stdin], [], [], 0.0005)[0]:
                ch3 = sys.stdin.read(1)
                return {'A': 'up', 'B': 'down', 'C': 'right', 'D': 'left'}.get(ch3, '')
        return ''
    return ch.lower()


class KeyboardCmdVelPublisher(Node):
    """
    키보드로 /cmd_vel(연속 발행) + /mode_cmd(숫자키) 발행 데모
    - w/s or ↑/↓ : +x / -x
    - a/d or ←/→ : +y / -y
    - q/e        : +yaw / -yaw
    - space/c    : 정지
    - z/x        : 속도 스케일 fast/slow
    - 1..9,0     : /mode_cmd(Int32) 발행
    """

    def __init__(self):
        super().__init__('keyboard_cmdvel_pub')

        # Params
        self.declare_parameter('rate', 20.0)
        self.declare_parameter('vx_max', 1.0)
        self.declare_parameter('vy_max', 0.6)
        self.declare_parameter('wz_max', 1.5)
        self.declare_parameter('verbose', True)

        self.rate_hz = float(self.get_parameter('rate').value)
        self.vx_max = float(self.get_parameter('vx_max').value)
        self.vy_max = float(self.get_parameter('vy_max').value)
        self.wz_max = float(self.get_parameter('wz_max').value)
        self.verbose = bool(self.get_parameter('verbose').value)

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_mode = self.create_publisher(Int32, '/mode_cmd', 10)

        # State
        self.scale_fast = 1.0
        self.scale_slow = 0.3
        self.scale_default = 0.6
        self.scale = self.scale_default

        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self.last_mode = 0

        # Timers
        self.timer = self.create_timer(1.0 / self.rate_hz, self.on_timer)
        self.mode_heartbeat_hz = 2.0
        self.mode_timer = self.create_timer(1.0 / self.mode_heartbeat_hz, self.on_mode_heartbeat)

        self.get_logger().info("Keyboard teleop started. Press Ctrl+C to quit.")
        self.get_logger().info("Mode keys: 1..9,0 -> /mode_cmd publish")
        if not sys.stdin.isatty():
            self.get_logger().warn("stdin is NOT a TTY. Key input disabled, but /cmd_vel will still publish.")

        # Ensure /mode_cmd shows up immediately
        self.pub_mode.publish(Int32(data=self.last_mode))
        if self.verbose:
            self.get_logger().info("[mode] initial publish: 0")

    def on_mode_heartbeat(self):
        # 주기적으로 마지막 모드 재발행(토픽 가시성/디버깅용)
        self.pub_mode.publish(Int32(data=self.last_mode))

    def on_timer(self):
        key = read_key_nonblocking(timeout_s=0.0)

        if key:
            # Mode keys
            if key in tuple('1234567890'):
                n = 0 if key == '0' else int(key)
                self.last_mode = n
                self.pub_mode.publish(Int32(data=n))
                self.get_logger().info(f"[mode] publish: {n}")

            # Motion keys
            elif key in ('w', 'up'):
                self.vx = +self.vx_max * self.scale; self.vy = 0.0; self.wz = 0.0
            elif key in ('s', 'down'):
                self.vx = -self.vx_max * self.scale; self.vy = 0.0; self.wz = 0.0
            elif key in ('a', 'left'):
                self.vx = 0.0; self.vy = +self.vy_max * self.scale; self.wz = 0.0
            elif key in ('d', 'right'):
                self.vx = 0.0; self.vy = -self.vy_max * self.scale; self.wz = 0.0
            elif key == 'q':
                self.vx = 0.0; self.vy = 0.0; self.wz = +self.wz_max * self.scale
            elif key == 'e':
                self.vx = 0.0; self.vy = 0.0; self.wz = -self.wz_max * self.scale
            elif key in (' ', 'c'):
                self.vx = self.vy = self.wz = 0.0
            elif key == 'z':
                self.scale = self.scale_fast
                self.get_logger().info("Speed scale = FAST (100%)")
            elif key == 'x':
                self.scale = self.scale_slow
                self.get_logger().info("Speed scale = SLOW (30%)")

        # Publish /cmd_vel continuously
        msg = Twist()
        msg.linear.x = self.vx
        msg.linear.y = self.vy
        msg.angular.z = self.wz
        self.pub.publish(msg)
        if self.verbose:
            self.get_logger().info(f"/cmd_vel publish: vx={self.vx:+.3f} vy={self.vy:+.3f} wz={self.wz:+.3f}")


def main():
    rclpy.init()
    node = KeyboardCmdVelPublisher()
    try:
        with raw_stdin_if_tty():
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
