#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

# from unitree_go.msg import LowState
from geometry_msgs.msg import PoseWithCovarianceStamped
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
# target_yaw atan2( 2wz, 1 - 2*z² )
def target_yaw(z, w):
    return math.atan2(2*w*z, 1 - 2*z**2)

class Stair(Node):
    def __init__(self):
        super().__init__('Stair')
        self.get_logger().info('Initializing Unitree SportClient...')
        ChannelFactoryInitialize(0)

        self.sc = SportClient(enableLease=False)
        self.sc.SetTimeout(10.0)
        self.sc.Init()
        self.sc.BalanceStand()

        self.kp = 5.0
        self.yaw = 0.0
        self.yaw_target = target_yaw(-0.3126348728838737, 0.9449064534569476)

        self.declare_parameter('auto_exit_sec', 10.0)
        auto_exit_sec = float(self.get_parameter('auto_exit_sec').value)
        self._exit_timer = None
        if auto_exit_sec > 0.0:
            self._exit_timer = self.create_timer(auto_exit_sec, self._on_exit_timer)

        # self.subscription = self.create_subscription(LowState, '/lowstate', self.stair_callback, 3)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.stair_callback, 3)

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

    # def stair_callback(self, msg: LowState):
    #     vx = float(1.0); vy = float(0.0)
    #     if len(msg.imu_state.rpy) >= 3:
    #         self.yaw = float(msg.imu_state.rpy[2])
    #         self.yaw = (self.yaw + math.pi) % (2.0 * math.pi) - math.pi
    #     else:
    #         self.get_logger().warn("imu_state.rpy 값이 올바르지 않습니다.")
    #         return

    #     yaw_error = float(self.yaw_target) - float(self.yaw)
    #     if abs(yaw_error) < 0.05:
    #         yaw_error = 0.0
    #     wz = float(self.kp) * yaw_error
    #     if abs(wz) >= 4.0:
    #         wz = math.copysign(4.0, wz)
    #     try:
    #         self.sc.Move(float(vx), float(vy), float(wz))
    #     except Exception as e:
    #         self.get_logger().warn(f"Move failed: {e}")

    def stair_callback(self, msg):
        vx = float(0.0); vy = float(0.0)

        q = msg.pose.pose.orientation
        self.yaw = float(target_yaw(q.z, q.w))
        self.yaw = (self.yaw + math.pi) % (2.0 * math.pi) - math.pi

        yaw_error = float(self.yaw_target) - float(self.yaw)
        if abs(yaw_error) < 0.05:
            yaw_error = 0.0
        wz = float(self.kp) * yaw_error
        if abs(wz) >= 4.0:
            wz = math.copysign(4.0, wz)
        try:
            self.sc.Move(float(vx), float(vy), float(wz))
        except Exception as e:
            self.get_logger().warn(f"Move failed: {e}")

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
