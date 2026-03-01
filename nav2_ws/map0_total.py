#!/usr/bin/env python3
import math, time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.executors import SingleThreadedExecutor
# from unitree_go.msg import LowState
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import qos_profile_sensor_data

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.sport.sport_client import SportClient

# --- SDK 전역 1회 초기화 ---
ChannelFactoryInitialize(0)
sc = SportClient(enableLease=False)
sc.SetTimeout(5.0)
sc.Init()

# target_yaw atan2( 2wz, 1 - 2*z² )
def target_yaw(z, w):
    return math.atan2(2*w*z, 1 - 2*z**2)

def shortest_ang_err(t, c):
    return math.atan2(math.sin(t - c), math.cos(t - c))

YAW_TARGET = target_yaw(0.9562548154860727, 0.2925350027940196)

def _safe_client_close(client):
    for m in ("Close","close","Deinit","DeInitialize","Disconnect","Release"):
        fn = getattr(client, m, None)
        if callable(fn):
            try: fn()
            except Exception: pass
            break

def _safe_channel_finalize():
    try:
        from unitree_sdk2py.core.channel import ChannelFactoryFinalize
        ChannelFactoryFinalize()
    except Exception:
        pass

class WallFollower(Node):
    def __init__(self, vx, yaw_target, duration_s=2.5, use_one_side_lidar = True, forward_target = False):
        super().__init__('wall_follower')

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

    def destroy_node(self):
        try:
            sc.Move(0.0, 0.0, 0.0)
        except Exception:
            pass
        super().destroy_node()
        
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
        ChannelFactoryInitialize(0)
        sc = SportClient(enableLease=False)
        sc.SetTimeout(5.0)
        sc.Init()
        sc.BalanceStand()

        n1 = WallFollower(vx = 0.5, yaw_target=YAW_TARGET, duration_s=4.7, use_one_side_lidar = False, forward_target = False) # 2.5
        executor.add_node(n1)
        run_stage(executor, n1)

        time.sleep(0.8)            # ← 이 유예 없으면 점프가 씹힐 수 있음
        sc.BalanceStand()          # (선택) 자세 안정화
        sc.FrontJump()
        time.sleep(1.2)            # 점프 동작 완료 대기
        
        n2 = WallFollower(vx = 2.5, yaw_target=YAW_TARGET, duration_s=0.8, use_one_side_lidar = False, forward_target = False)
        executor.add_node(n2)
        run_stage(executor, n2)

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

# def main(args=None):
#     rclpy.init(args=args)
#     node = WallFollower()

#     try:
#         # 1) LiDAR 기반 정렬 1.4초
#         t0 = time.time()
#         while rclpy.ok() and time.time() - t0 < 2.5:
#             rclpy.spin_once(node)

#         # 2) 점프 시퀀스: 스트리밍 일시정지 → 대기 → FrontJump → 대기
#         node.pause_streaming()
#         time.sleep(0.8)            # ← 이 유예 없으면 점프가 씹힐 수 있음
#         sc.BalanceStand()          # (선택) 자세 안정화
#         sc.FrontJump()
#         time.sleep(1.2)            # 점프 동작 완료 대기
#         node.resume_streaming()

#         # 3) 후속 스핀 1초
#         t1 = time.time()
#         while rclpy.ok() and time.time() - t1 < 2.0:
#             rclpy.spin_once(node)

#     finally:
#         # 정리: 노드 → SDK → 채널 → rclpy
#         try: node.destroy_node()
#         except Exception: pass
#         try: sc.Move(0.0, 0.0, 0.0)
#         except Exception: pass
#         _safe_client_close(sc)
#         _safe_channel_finalize()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()
