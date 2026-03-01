#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from rclpy.qos import qos_profile_sensor_data

import math, time, os, signal, threading

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.sport.sport_client import SportClient

def _safe_client_close(sc):
    for m in ("Close", "close", "Deinit", "DeInitialize", "Disconnect", "Release"):
        fn = getattr(sc, m, None)
        if callable(fn):
            try:
                fn()
            except Exception:
                pass
            break

def _safe_sdk_finalize():
    try:
        from unitree_sdk2py.core import channel as _ch
    except Exception:
        return
    for name in ("ChannelFactoryFinalize", "ChannelFactoryDeinitialize", "ChannelFactoryUninit",
                 "ChannelFactoryShutdown", "ChannelFactoryFini"):
        fn = getattr(_ch, name, None)
        if callable(fn):
            try:
                fn()
            except Exception:
                pass
            break

class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower')
        self.get_logger().info('Initializing Unitree SportClient...')
        ChannelFactoryInitialize(0)

        self.sc = SportClient(enableLease=False)
        self.sc.SetTimeout(5.0)
        self.sc.Init()
        self.sc.BalanceStand()

        self.kp = 6.16
        self.error = 0.0

        self.declare_parameter('auto_exit_sec', 3.0)
        auto_exit_sec = float(self.get_parameter('auto_exit_sec').value)
        self._exit_timer = None
        if auto_exit_sec > 0.0:
            self._exit_timer = self.create_timer(auto_exit_sec, self._on_exit_timer)

        self.sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_cb,
            qos_profile_sensor_data,  # ← 요걸로!
        )
        # self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def _destroy_timers_and_subs(self):
        # timer
        try:
            if self._exit_timer:
                self._exit_timer.cancel()
                self.destroy_timer(self._exit_timer)
                self._exit_timer = None
        except Exception:
            pass
        # subscription
        try:
            if self.subscription:
                self.destroy_subscription(self.subscription)
                self.subscription = None
        except Exception:
            pass

    def _cleanup(self):
        # 1) ROS 엔티티 해제
        self._destroy_timers_and_subs()
        # 2) 로봇 정지 + 클라이언트 종료
        try:
            self.sc.StopMove()
        except Exception:
            pass
        _safe_client_close(self.sc)
        # 3) 채널 팩토리 종료
        _safe_sdk_finalize()

    def _on_exit_timer(self):
        self.get_logger().info('Auto-exit timer triggered. Stopping and shutting down...')
        self._cleanup()
        # 노드 파괴는 main()에서 일괄 처리
        rclpy.shutdown()

    def lidar_cb(self, msg: LaserScan):
        vx = float(1.0); wz = float(0.0)
        # 왼쪽(150~180도), 오른쪽(0~30도) 영역 평균
        left_ranges = [r for i,r in enumerate(msg.ranges[150:180]) if r > 0.1]
        right_ranges = [r for i,r in enumerate(msg.ranges[0:30]) if r > 0.1]

        if not left_ranges or not right_ranges:
            return

        left_dist = sum(left_ranges)/len(left_ranges)
        right_dist = sum(right_ranges)/len(right_ranges)
        self.get_logger().info(f'left: {left_dist}')
        self.get_logger().info(f'right: {right_dist}')

        if left_dist <= 0.85 and right_dist <= 0.85:
            self.error = left_dist - right_dist
        else:
            pass
        vy = self.kp * self.error

        if abs(vy) >= 4.0:
            vy = math.copysign(4.0, vy)
        try:
            self.sc.Move(float(vx), float(vy), float(wz))
        except Exception as e:
            self.get_logger().warn(f"Move failed: {e}")
        # self.pub.publish(cmd)
    
    def destroy_node(self):
        # 중복 안전
        self._cleanup()
        super().destroy_node()

def _hard_kill_if_needed(grace_sec=0.3):
    # 남은 non-daemon 스레드가 있으면 마지막으로 확실히 종료
    time.sleep(grace_sec)
    alive = [t for t in threading.enumerate() if t is not threading.main_thread()]
    if alive:
        # print 남기고 강종하고 싶으면 주석 해제
        # print("Non-daemon threads still alive:", alive)
        os._exit(0)

def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # 정리 순서: SDK/타이머 해제 → 노드 파괴 → executor 종료 → rclpy 종료
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            executor.shutdown(timeout_sec=0.2)
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass

    # 혹시 남은 스레드가 있으면 강제 종료
    _hard_kill_if_needed(grace_sec=0.3)

if __name__ == '__main__':
    main()

