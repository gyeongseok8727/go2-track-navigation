#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from unitree_go.msg import LowState
import math, time, os, signal, threading
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.sport.sport_client import SportClient
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient

from process_executor import BaseProcessExecutor, ExecConfig


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

class Stair(Node):
    def __init__(self):
        super().__init__('Stair')
        self.get_logger().info('Initializing Unitree SportClient...')
        ChannelFactoryInitialize(0)

        self.sc = SportClient(enableLease=False)
        self.sc.SetTimeout(10.0)
        self.sc.Init()
        #self.sc.BalanceStand()

        self.kp = 1.5
        self.error = 0.0

        self.kp2 = 5.0
        self.yaw = 0.0
        self.yaw_target = 0.3329617381095886

        self.vx = 1.5
        self.wz = 0.0
        self.vy = 0.0

        self.declare_parameter('auto_exit_sec', 18.5)
        auto_exit_sec = float(self.get_parameter('auto_exit_sec').value)
        self._exit_timer = None
        if auto_exit_sec > 0.0:
            self._exit_timer = self.create_timer(auto_exit_sec, self._on_exit_timer)

        self.subscription = self.create_subscription(LowState, '/lowstate', self.stair_callback, 3)

        self.sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_cb,
            qos_profile_sensor_data,  # ← 요걸로!
        )

        cfg = ExecConfig(
            cmd=["bash", "-i", "-c", "./go2_ctrl --network eno1"],
            cwd="/home/mr/unitree_rl_lab/deploy/robots/go2/build",
        )
        self.ex = BaseProcessExecutor(cfg)

        self.msc = MotionSwitcherClient()
        self.msc.SetTimeout(5.0)
        self.msc.Init()

        self.pub_mode = self.create_publisher(Int32, '/mode_cmd', 10)
        self.mode = None
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        print("standdown")
        self.sc.StandDown()
        time.sleep(3)
        print("release")
        self.msc.ReleaseMode()
        time.sleep(3)

        print("start")
        self.ex.start()
        time.sleep(4)
        self.send_mode(1)
        time.sleep(2)

        self.send_mode(2)
        time.sleep(1)

    def send_mode(self, mode_value: int):
        msg = Int32()
        msg.data = mode_value
        self.pub_mode.publish(msg)
        self.get_logger().info(f'📤 Sent mode command: {mode_value}')

    def send_cmd(self, vx, vy, wz):
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = wz
        self.pub.publish(msg)

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
            self.send_cmd(0.0, 0.0, 0.0)
            time.sleep(1)
            self.send_mode(3)
            time.sleep(1)
            self.ex.stop()
            time.sleep(4)
            self.send_mode(4)
            time.sleep(1)
            self.msc.SelectMode("mcf".strip())
            time.sleep(1)
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

    def stair_callback(self, msg: LowState):
        
        if len(msg.imu_state.rpy) >= 3:
            self.yaw = float(msg.imu_state.rpy[2])
            self.yaw = (self.yaw + math.pi) % (2.0 * math.pi) - math.pi
        else:
            self.get_logger().warn("imu_state.rpy 값이 올바르지 않습니다.")
            return

        yaw_error = float(self.yaw_target) - float(self.yaw)
        if abs(yaw_error) < 0.05:
            yaw_error = 0.0
        self.wz = float(self.kp2) * yaw_error
        if abs(self.wz) >= 4.0:
            self.wz = math.copysign(4.0, self.wz)
        # try:
        #     #self.sc.Move(float(vx), float(vy), float(wz))
        #     self.send_cmd(float(vx), float(vy), float(wz))

        # except Exception as e:
        #     self.get_logger().warn(f"Move failed: {e}")

    def lidar_cb(self, msg: LaserScan):
        # 왼쪽(150~180도), 오른쪽(0~30도) 영역 평균
        left_ranges = [r for i,r in enumerate(msg.ranges[150:180]) if r > 0.1]
        right_ranges = [r for i,r in enumerate(msg.ranges[0:30]) if r > 0.1]

        if not left_ranges or not right_ranges:
            return

        left_dist = sum(left_ranges)/len(left_ranges)
        right_dist = sum(right_ranges)/len(right_ranges)
        self.get_logger().info(f'left: {left_dist}')
        self.get_logger().info(f'right: {right_dist}')

        if left_dist + right_dist <= 1.2:
        # if left_dist <= 5.0 and right_dist <= 5.0:
            self.error = left_dist - right_dist
        self.vy = self.kp * self.error

        self.get_logger().warn(f"vy: {self.vy}")

        if abs(self.vy) >= 3.7:
            self.vy = math.copysign(4.0, self.vy)
        try:
            self.send_cmd(float(self.vx), float(self.vy), float(self.wz))
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
    node = Stair()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 정리 순서: SDK/타이머 해제 → 노드 파괴 → executor 종료 → rclpy 종료
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass

    # 혹시 남은 스레드가 있으면 강제 종료
    _hard_kill_if_needed(grace_sec=0.3)

if __name__ == '__main__':
    main()
