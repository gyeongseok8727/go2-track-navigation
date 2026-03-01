#!/usr/bin/env python3
import math, time
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
# from unitree_go.msg import LowState
from geometry_msgs.msg import PoseWithCovarianceStamped

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.sport.sport_client import SportClient

import subprocess

# ====== Unitree SDK: 전역으로 1회 초기화 ======
ChannelFactoryInitialize(0)
sc = SportClient(enableLease=False)
sc.SetTimeout(5.0)
sc.Init()

# target_yaw atan2( 2wz, 1 - 2*z² )
def target_yaw(z, w):
    return math.atan2(2*w*z, 1 - 2*z**2)

def shortest_ang_err(t, c):
    return math.atan2(math.sin(t - c), math.cos(t - c))

YAW_TARGET = target_yaw(-0.008334371881327926, 0.999965268519534)

# ---------- 공통 유틸 ----------
def _safe_client_close(client):
    for m in ("Close", "close", "Deinit", "DeInitialize", "Disconnect", "Release"):
        fn = getattr(client, m, None)
        if callable(fn):
            try:
                fn()
            except Exception:
                pass
            break

def _safe_channel_finalize():
    try:
        from unitree_sdk2py.core.channel import ChannelFactoryFinalize
        ChannelFactoryFinalize()
    except Exception:
        pass

# ---------- 1) 중앙 정렬 주행 ----------
class WallFollower(Node):
    def __init__(self, vx, yaw_target, duration_s=2.5, use_one_side_lidar = True, forward_target = False):
        super().__init__('wall_follower')
        sc.BalanceStand()

        self.vx = vx; self.vy = 0.0; self.wz = 0.0
        self.kp = 1.5; self.kp2 = 5.0; self.kp3 = -1.5
        self.error = 0.0
        self.yaw = 0.0
        self.yaw_target = yaw_target
        self.use_one_side_lidar = use_one_side_lidar
        self.forward_target = forward_target

        self.done = False
        self._t0 = time.time()
        self._duration = duration_s

        self.create_subscription(LaserScan, '/scan', self.lidar_cb, qos_profile_sensor_data)
        # self.create_subscription(LowState, '/lowstate', self.yaw_cb, 3)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.yaw_cb, 3)
    
    """
    def yaw_cb(self, msg: LowState):
        if len(msg.imu_state.rpy) < 3:
            return
        self.yaw = float(msg.imu_state.rpy[2])
        self.yaw = (self.yaw + math.pi) % (2.0 * math.pi) - math.pi
        yaw_error = self.yaw_target - self.yaw
        if abs(yaw_error) < 0.05:
            yaw_error = 0.0
        self.wz = max(-4.0, min(4.0, self.kp2 * yaw_error))
    """

    def yaw_cb(self, msg):
        q = msg.pose.pose.orientation
        self.yaw = float(target_yaw(q.z, q.w))
        self.yaw = (self.yaw + math.pi) % (2.0 * math.pi) - math.pi
        self.yaw_target = (self.yaw_target + math.pi) % (2*math.pi) - math.pi
        yaw_error = shortest_ang_err(self.yaw_target, self.yaw)
        if abs(yaw_error) < 0.05:
            yaw_error = 0.0
        self.wz = max(-4.0, min(4.0, self.kp2 * yaw_error))

    def lidar_cb(self, msg: LaserScan):
        # 좌(150~180), 우(0~30) 평균으로 중앙 유지
        left = [r for r in msg.ranges[150:180] if r > 0.1]
        right = [r for r in msg.ranges[0:30] if r > 0.1]
        if not left or not right:
            return
        left_dist = sum(left)/len(left)
        right_dist = sum(right)/len(right)

        if left_dist + right_dist <= 1.2:
            self.error = left_dist - right_dist
        elif left_dist > 0.85 and right_dist > 0.85:
            self.error = 0.0
        elif self.use_one_side_lidar and left_dist <= 0.85 and right_dist > 0.85:
            self.error =  (left_dist - 0.5) * 2 # 중앙값 기준 0.5
            self.get_logger().info(f"left_dist: {left_dist}")
        elif self.use_one_side_lidar and left_dist > 0.85 and right_dist <= 0.85:
            self.error = (0.5 - right_dist) * 2 # 중앙값 기준 0.5
            self.get_logger().info(f"right_dist: {right_dist}")
        self.vy = max(-3.7, min(3.7, self.kp * self.error))

        # 일정 시간 지나면 단계 종료
        if time.time() - self._t0 >= self._duration:
            self.done = True
            return
        if self.forward_target is not False:
            fwd = [r for r in msg.ranges[75:105] if r > 0.1]
            if not fwd:
                return
            forward_dist = sum(fwd)/len(fwd)
            err = self.forward_target - forward_dist
            if abs(err) < 0.08:
                try: sc.Move(0.0, 0.0, 0.0)
                except Exception: pass
                self.done = True
                return
            self.vx = max(-self.vx, min(self.vx, self.kp3 * err))
        sc.Move(self.vx, self.vy, self.wz)


# ---------- 2) 좌회전 ----------
def normalize_angle(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi

def ang_error_shortest(target, current):
    # signed shortest angular distance in [-π, π]
    return math.atan2(math.sin(target - current), math.cos(target - current))

class Turn_Left(Node):
    def __init__(self, yaw_target, duration_s=1.2, kp=3.1, wz_max=2.5, min_wz=0.2):
        super().__init__('turn_left')
        sc.BalanceStand()
        self.vx = 0.0; self.vy = 0.0; self.wz = 0.0
        self.kp = kp; self.wz_max = wz_max; self.min_wz = min_wz
        self.yaw = 0.0
        self.yaw_target = yaw_target   # 첫 pose 수신 때 현재+90°
        self._t0 = time.time(); self._duration = duration_s
        self.done = False
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.turn_cb, 3)
        sc.Move(0,0,1)
        time.sleep(0.3)

    def turn_cb(self, msg):
        q = msg.pose.pose.orientation
        self.yaw = normalize_angle(target_yaw(q.z, q.w))

        # 첫 수신 시 목표를 "왼쪽 90°"로 설정
        if self.yaw_target is None:
            self.yaw_target = normalize_angle(self.yaw + math.pi/2)
            # 초기 과회전 방지: 정지 상태에서 시작
            try: sc.Move(0.0, 0.0, 0.0)
            except Exception: pass

        # 최단각 오차
        e = ang_error_shortest(self.yaw_target, self.yaw)

        # 종료 조건
        if abs(e) < 0.05:
            try: sc.Move(0.0, 0.0, 0.0)
            except Exception: pass
            self.done = True
            return
        if time.time() - self._t0 >= self._duration:
            self.done = True
            return

        # P 제어 + 포화 + 최소 회전속도(정지 마찰 극복)
        wz_cmd = max(-self.wz_max, min(self.wz_max, self.kp * e))
        if abs(wz_cmd) < self.min_wz:  # deadband 탈출
            wz_cmd = self.min_wz * (1.0 if wz_cmd >= 0.0 else -1.0)

        self.wz = wz_cmd
        self.get_logger().info(f"yaw={self.yaw:.3f}, tgt={self.yaw_target:.3f}, e={e:.3f}, wz={self.wz:.2f}")
        sc.Move(self.vx, self.vy, self.wz)

    # def turn_cb(self, msg: LowState):
    #     if len(msg.imu_state.rpy) < 3:
    #         return
    #     self.yaw = float(msg.imu_state.rpy[2])
    #     self.yaw = (self.yaw + math.pi) % (2.0 * math.pi) - math.pi
    #     yaw_error = self.yaw_target - self.yaw
    #     if abs(yaw_error) < 0.05:
    #         # 목표 각도 도달 → 정지 후 완료
    #         try: sc.Move(0.0, 0.0, 0.0)
    #         except Exception: pass
    #         self.done = True
    #         return
    #     self.wz = max(-4.0, min(4.0, self.kp2 * yaw_error))
    #     sc.Move(self.vx, self.vy, self.wz)


# ---------- 메인: 단계별 실행 오케스트레이션 ----------
def run_stage(executor, node, timeout_s=None):
    start = time.time()
    while rclpy.ok() and not node.done:
        executor.spin_once(timeout_sec=0.05)
        if timeout_s and (time.time() - start) > timeout_s:
            break
    # 단계 종료시 정지 & 정리
    try: sc.Move(0.0, 0.0, 0.0)
    except Exception: pass
    executor.remove_node(node)
    node.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    executor = SingleThreadedExecutor()

    try:
        n1 = WallFollower(vx = 1.0, yaw_target=YAW_TARGET, duration_s=1.0, use_one_side_lidar = False, forward_target = 0.5)
        executor.add_node(n1)
        run_stage(executor, n1)

        time.sleep(0.5)
        YAW_TARGET2 = target_yaw(0.6838540407856182, 0.7296188394642658)
        n2 = Turn_Left(yaw_target = YAW_TARGET2, duration_s = 1.2)
        executor.add_node(n2)
        run_stage(executor, n2)  # 안전 타임아웃
        
        n3 = WallFollower(vx = 1.2, yaw_target=YAW_TARGET2, duration_s=5.2, use_one_side_lidar = False, forward_target = False)
        executor.add_node(n3)
        run_stage(executor, n3)

        n4 = WallFollower(vx = 0.6, yaw_target=YAW_TARGET2, duration_s=5.0, use_one_side_lidar = False, forward_target = 0.5)
        executor.add_node(n4)
        run_stage(executor, n4)

        time.sleep(5.0)
        ROS_SETUP = "source /opt/ros/humble/setup.bash && source ~/nav2_ws/install/setup.bash"
        new_localization_cmd = f"""{ROS_SETUP} && ros2 topic pub -1 /initialpose geometry_msgs/PoseWithCovarianceStamped "{{
                            header: {{
                                stamp: {{sec: 0, nanosec: 0}},
                                frame_id: map
                            }},
                            pose: {{
                                pose: {{
                                position: {{x: 6.690623713134998, y: 7.933888863966541, z: 0.0}},
                                orientation: {{x: 0.0, y: 0.0, z: 0.6676954505828355, w: 0.7444345406219303}}
                                }},
                                covariance: [
                                0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891909122467
                                ]
                            }}
                        }}" """
        # or: --global --wait-amcl
        print("[init] auto_localize:", new_localization_cmd)
        subprocess.call(["bash", "-lc", new_localization_cmd])
        time.sleep(0.5)
        YAW_TARGET3 = target_yaw(0.9999976873100451, 0.002150668398693065)
        n5 = Turn_Left(yaw_target = YAW_TARGET3, duration_s = 1.2)
        executor.add_node(n5)
        run_stage(executor, n5)



    finally:
        try: executor.shutdown()
        except Exception: pass
        try: sc.Move(0.0, 0.0, 0.0)
        except Exception: pass
        _safe_client_close(sc)
        _safe_channel_finalize()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
