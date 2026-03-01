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

# ====== Unitree SDK: 전역으로 1회 초기화 ======
ChannelFactoryInitialize(0)
sc = SportClient(enableLease=False)
sc.SetTimeout(5.0)
sc.Init()
sc.BalanceStand()

# target_yaw atan2( 2wz, 1 - 2*z² )
def target_yaw(z, w):
    return math.atan2(2*w*z, 1 - 2*z**2)

def shortest_ang_err(t, c):
    return math.atan2(math.sin(t - c), math.cos(t - c))

YAW_TARGET = target_yaw(0.9562548154860727, 0.2925350027940196)

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

def cosd(deg: float) -> float:
    return math.cos(math.radians(deg))

def sector_threshold(theta_deg: int):
    """
    문제에서 주신 각도 구간별 임계 r을 반환.
    해당 구간이 아니면 None 반환.
    분모가 0에 가까우면 None로 취급하여 제외.
    """
    eps = 1e-3

    if 30 < theta_deg < 60:
        c = cosd(theta_deg)
        if abs(c) < eps: return None
        return 0.4 / c

    elif 60 < theta_deg < 90:
        c = cosd(90 - theta_deg)
        if abs(c) < eps: return None
        return 0.8 / c

    elif 90 < theta_deg < 120:
        c = cosd(theta_deg - 90)
        if abs(c) < eps: return None
        return 0.8 / c

    elif 120 < theta_deg < 150:
        c = cosd(180 - theta_deg)
        if abs(c) < eps: return None
        return 0.4 / c

    return None  # 그 외 각도는 기록하지 않음

def sector_threshold_forward(theta_deg: int):
    """
    문제에서 주신 각도 구간별 임계 r을 반환.
    해당 구간이 아니면 None 반환.
    분모가 0에 가까우면 None로 취급하여 제외.
    """
    eps = 1e-3

    if 75 < theta_deg < 90:
        c = cosd(90 - theta_deg)
        if abs(c) < eps: return None
        return 0.6 / c

    elif 90 < theta_deg < 105:
        c = cosd(theta_deg - 90)
        if abs(c) < eps: return None
        return 0.6 / c


    return None  # 그 외 각도는 기록하지 않음

# ---------- 1) 중앙 정렬 주행 ----------
class Obstacle_lidar(Node):
    def __init__(self, vx, yaw_target, use_one_side_lidar = True):
        super().__init__('wall_follower')
        sc.BalanceStand()

        self.vx = vx; self.vy = 0.0; self.wz = 0.0
        self.kp = 1.5; self.kp2 = 5.0; self.kp3 = -1.5
        self.error = 0.0
        self.yaw = 0.0
        self.yaw_target = yaw_target
        self.use_one_side_lidar = use_one_side_lidar
        self.forward_target = 0.5
        self.close = False
            
        self.done = False

        self.create_subscription(LaserScan, '/scan', self.lidar_cb, qos_profile_sensor_data)
        # self.create_subscription(LowState, '/lowstate', self.yaw_cb, 3)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.yaw_cb, 3)
    
    def select_points_and_average(self, msg):
        """
        LaserScan에서 문제 조건을 만족하는 점들만 선별하고,
        그 점들의 평균 거리를 반환합니다.
        반환값: (avg_dist or None, selected_points)
        - selected_points: [(theta, r), ...]
        """
        selected = []

        # 스캔 길이 보호
        n = len(msg.ranges)
        # 우리가 쓸 각도 범위만 검사 (31~149)
        for theta in range(31, min(150, n)):
            r = msg.ranges[theta]
            # 유효성 체크
            if not math.isfinite(r) or r <= 0.1:
                continue

            thr = sector_threshold(theta)
            dist = 0
            c = 1
            if thr is None:
                continue

            if r < thr:
                eps = 1e-3

                if 30 < theta < 90:
                    c = cosd(90 - theta)
                    if abs(c) < eps: 
                        pass

                elif 90 < theta < 150:
                    c = cosd(theta - 90)
                    if abs(c) < eps:
                        pass
                
                dist = r * c
                selected.append((theta, dist))

        if not selected:
            return None, []

        avg = sum(dist for _, dist in selected) / len(selected)
        return avg, selected

    def select_points_and_average_forward(self, msg):
        """
        LaserScan에서 문제 조건을 만족하는 점들만 선별하고,
        그 점들의 평균 거리를 반환합니다.
        반환값: (avg_dist or None, selected_points)
        - selected_points: [(theta, r), ...]
        """
        selected = []

        # 스캔 길이 보호
        n = len(msg.ranges)
        # 우리가 쓸 각도 범위만 검사 (76~104)
        for theta in range(76, min(104, n)):
            r = msg.ranges[theta]
            # 유효성 체크
            if not math.isfinite(r) or r <= 0.1:
                continue

            thr = sector_threshold_forward(theta)
            dist = 0
            c = 1
            if thr is None:
                continue

            if r < thr:
                eps = 1e-3

                if 75 < theta < 90:
                    c = cosd(90 - theta)
                    if abs(c) < eps: 
                        pass

                elif 90 < theta < 105:
                    c = cosd(theta - 90)
                    if abs(c) < eps:
                        pass
                
                dist = r * c
                selected.append((theta, dist))

        if not selected:
            return None, []

        avg = sum(dist for _, dist in selected) / len(selected)
        return avg, selected

    def yaw_cb(self, msg):
        q = msg.pose.pose.orientation
        self.yaw = float(target_yaw(q.z, q.w))
        self.yaw = (self.yaw + math.pi) % (2.0 * math.pi) - math.pi
        yaw_error = self.yaw_target - self.yaw
        if abs(yaw_error) < 0.05:
            yaw_error = 0.0
        self.wz = max(-4.0, min(4.0, self.kp2 * yaw_error))

    def lidar_cb(self, msg: LaserScan):
        # theta_deg: 0~180 기준 각도(90 전방). 조건 만족 시 r_max 반환, 아니면 None.
        # 30<θ<60:    r < 0.5 / cos(θ)
        # 60<θ<90:    r < 1.0 / cos(90-θ)
        # 90<θ<120:   r < 1.0 / cos(θ-90)
        # 120<θ<150:  r < 0.5 / cos(180-θ)
        if self.close is False:
            
            left = [r for r in msg.ranges[150:180] if r > 0.1]
            right = [r for r in msg.ranges[0:30] if r > 0.1]
            if not left or not right:
                return
            left_dist = sum(left)/len(left)
            right_dist = sum(right)/len(right)
            avg_dist, pts = self.select_points_and_average(msg)

            if left_dist + right_dist <= 1.2 and avg_dist is not None:
                self.error = left_dist - right_dist
            elif left_dist > 0.85 and right_dist > 0.85 and avg_dist is not None:
                self.error = 0.0
            elif self.use_one_side_lidar and left_dist <= 0.85 and right_dist > 0.85 and avg_dist is not None:
                self.error =  (left_dist - 0.5) * 2 # 중앙값 기준 0.5
                self.get_logger().info(f"left_dist: {left_dist}")
            elif self.use_one_side_lidar and left_dist > 0.85 and right_dist <= 0.85 and avg_dist is not None:
                self.error = (0.5 - right_dist) * 2 # 중앙값 기준 0.5
                self.get_logger().info(f"right_dist: {right_dist}")
            self.vy = max(-3.7, min(3.7, self.kp * self.error))

            if avg_dist is None:
                # 조건 만족 점이 없으니 천천히 전진
                # 예: self.vx = 0.2 (또는 현재 속도 유지/감속)
                self.get_logger().info("No target points → slow move")
                self.vx = 0.5
            else:
                self.get_logger().info(f"target_dist: {avg_dist}")
                err = self.forward_target - avg_dist
                if abs(err) < 0.08:
                    self.close = True
                    self.vx = 0.0    
                self.vx = max(-self.vx, min(self.vx, self.kp3 * err))
            sc.Move(float(self.vx), float(self.vy), float(self.wz))
              
        else:
            avg_dist, pts = self.select_points_and_average_forward(msg)
            self.get_logger().info(f"forward: {avg_dist}")
            
            if avg_dist is None:
                self.done = True
                return

            elif avg_dist > 1.2:
                self.done = True
                return
            else:
                sc.Move(0,0,0)

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

        try:
            sc.Move(float(self.vx), float(self.vy), float(self.wz))
        except Exception as e:
            self.get_logger().warn(f"Move failed: {e}")

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
        # time.sleep(5)
        print("aline")
        n0 = WallFollower(vx = 0.2, yaw_target=YAW_TARGET, duration_s=4.0, use_one_side_lidar = False, forward_target = False)
        executor.add_node(n0)
        run_stage(executor, n0)
        print("aline finish") 

        n1 = Obstacle_lidar(vx = 1.0, yaw_target=YAW_TARGET, use_one_side_lidar = True)
        executor.add_node(n1)
        run_stage(executor, n1)

        time.sleep(1.0) # wait obstacle
        n2 = WallFollower(vx = 3.7, yaw_target=YAW_TARGET, duration_s=1.2, use_one_side_lidar = True, forward_target = False)
        executor.add_node(n2)
        run_stage(executor, n2)  # 안전 타임아웃
        

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
