#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import socket
import sys
from typing import Tuple

# [ADD]
import os, signal  # ← 모드 4 수신시 자기 자신에게 SIGINT 보내기 위해 추가

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class UdpBridgeNode(Node):
    """
    /cmd_vel(geometry_msgs/Twist) → "vx vy wz\n" UDP로 고정주기 송신
    /mode_cmd(std_msgs/Int32)     → "N\n" 또는 "CMD N\n" 즉시 송신
    - 파라미터:
        host (str)         : 기본 '127.0.0.1'
        port (int)         : 기본 9000
        rate (float)       : 기본 50.0 Hz
        deadzone (float)   : 기본 0.0
        lpf_tau (float)    : 기본 0.12 (0 => 비활성)
        vx_max/vy_max/wz_max (float)
        verbose (bool)     : 기본 True
        mode_prefix (str)  : 기본 '' (예: 'CMD ' 가능)
    """

    def __init__(self):
        super().__init__('udp_bridge')

        # ---- Parameters ----
        self.declare_parameter('host', '127.0.0.1')
        self.declare_parameter('port', 9000)
        self.declare_parameter('rate', 50.0)
        self.declare_parameter('deadzone', 0.0)
        self.declare_parameter('lpf_tau', 0.12)
        self.declare_parameter('vx_max', 1.0)
        self.declare_parameter('vy_max', 0.6)
        self.declare_parameter('wz_max', 1.5)
        self.declare_parameter('verbose', True)
        self.declare_parameter('mode_prefix', '')

        self.host: str = self.get_parameter('host').value
        self.port: int = int(self.get_parameter('port').value)
        self.rate_hz: float = float(self.get_parameter('rate').value)
        self.deadzone: float = float(self.get_parameter('deadzone').value)
        self.lpf_tau: float = float(self.get_parameter('lpf_tau').value)
        self.vx_max: float = float(self.get_parameter('vx_max').value)
        self.vy_max: float = float(self.get_parameter('vy_max').value)
        self.wz_max: float = float(self.get_parameter('wz_max').value)
        self.verbose: bool = bool(self.get_parameter('verbose').value)
        self.mode_prefix: str = str(self.get_parameter('mode_prefix').value or '')

        self.dt = 1.0 / max(1e-6, self.rate_hz)

        # ---- UDP ----
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.addr: Tuple[str, int] = (self.host, self.port)
        except Exception as e:
            self.get_logger().error(f'Failed to create UDP socket: {e}')
            sys.exit(1)

        # ---- State ----
        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_wz = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0

        # ---- Subscribers ----
        self.sub_cmd = self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        self.sub_mode = self.create_subscription(Int32, '/mode_cmd', self.mode_cb, 10)

        # ---- Timer ----
        self.timer = self.create_timer(self.dt, self.on_timer)

        self.get_logger().info(
            f'UDP Bridge: /cmd_vel & /mode_cmd -> {self.host}:{self.port} @ {self.rate_hz:.1f} Hz'
        )
        if self.mode_prefix:
            self.get_logger().info(f'Mode payload prefix: "{self.mode_prefix}"')

    def destroy_node(self):
        try:
            self.sock.close()
        except Exception:
            pass
        return super().destroy_node()

    def apply_deadzone(self, v: float) -> float:
        return 0.0 if abs(v) < self.deadzone else v

    # ---- Callbacks ----
    def cmd_cb(self, msg: Twist):
        self.target_vx = clamp(msg.linear.x,  -self.vx_max, self.vx_max)
        self.target_vy = clamp(msg.linear.y,  -self.vy_max, self.vy_max)
        self.target_wz = clamp(msg.angular.z, -self.wz_max, self.wz_max)
        if self.verbose:
            self.get_logger().info(
                f"[recv /cmd_vel] target vx={self.target_vx:+.3f} vy={self.target_vy:+.3f} wz={self.target_wz:+.3f}"
            )

    def mode_cb(self, msg: Int32):
        n = int(msg.data)
        payload = f"{self.mode_prefix}{n}\n".encode('utf-8')
        try:
            self.sock.sendto(payload, self.addr)
        except Exception as e:
            self.get_logger().warn(f'[udp] send mode error: {e}')
        else:
            self.get_logger().info(f"[mode] sent: {self.mode_prefix}{n}")

        # [ADD] 모드 4 수신 시 자기 자신 종료 (SIGINT) → spin 종료 → finally에서 정리
        if n == 4:
            self.get_logger().info("Mode 4 received → shutting down (SIGINT).")
            os.kill(os.getpid(), signal.SIGINT)

    def on_timer(self):
        # 1st order LPF
        if self.lpf_tau > 1e-6:
            alpha = clamp(self.dt / (self.lpf_tau + 1e-6), 0.0, 1.0)
            self.vx += alpha * (self.target_vx - self.vx)
            self.vy += alpha * (self.target_vy - self.vy)
            self.wz += alpha * (self.target_wz - self.wz)
        else:
            self.vx, self.vy, self.wz = self.target_vx, self.target_vy, self.target_wz

        # deadzone & clamp
        self.vx = clamp(self.apply_deadzone(self.vx), -self.vx_max, self.vx_max)
        self.vy = clamp(self.apply_deadzone(self.vy), -self.vy_max, self.vy_max)
        self.wz = clamp(self.apply_deadzone(self.wz), -self.wz_max, self.wz_max)

        payload = f"{self.vx:.6f} {self.vy:.6f} {self.wz:.6f}\n".encode('utf-8')
        try:
            self.sock.sendto(payload, self.addr)
        except Exception as e:
            self.get_logger().warn(f'[udp] send error: {e}')
        else:
            if self.verbose:
                self.get_logger().info(
                    f"[udp] sent vel: vx={self.vx:+.3f} vy={self.vy:+.3f} wz={self.wz:+.3f}"
                )


def main():
    rclpy.init()
    node = UdpBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
