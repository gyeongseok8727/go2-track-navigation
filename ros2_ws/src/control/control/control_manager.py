import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import time
import math
from enum import Enum

from std_msgs.msg import Int32
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist

from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
from control.process_executor import BaseProcessExecutor, ExecConfig

#sdk class definition
msc = MotionSwitcherClient()

class Policy(Enum):
    WALK = 0
    JUMP = 1
    DYNAMIC_OBSTACLE_WAIT=2
    DYNAMIC_OBSTACLE_GO=3
    CREEP_START=4
    CREEP_MID=5
    CREEP_END=6
    TRAFFIC_WAIT=7
    TRAFFIC_GO=8
    FUCKING_RUN=9
    TERMINATE=10

class SCid(Enum):
    DAMP = 0
    STAND_UP = 1
    STAND_DOWN = 2
    MOVE_FORWARD = 3
    MOVE_LATERAL = 4
    MOVE_ROTATE = 5
    STOP_MOVE = 6
    HAND_STAND = 7
    BALANCED_STAND = 9
    RECOVERY = 10
    LEFT_FLIP = 11
    BACK_FLIP = 12
    FREE_WALK = 13
    FREE_BOUND = 14
    FREE_AVOID = 15
    WALK_UPRIGHT = 17
    CROSS_STEP = 18
    FREE_JUMP = 19

class YoloCmdSubscriber(Node):
    def __init__(self):
        super().__init__('yolo_cmd_subscriber')
        # 구독자 생성
        self.subscription = self.create_subscription(
            Int32,          # 메시지 타입
            'yolo_cmd',     # 토픽 이름
            self.listener_callback,  # 콜백 함수
            10              # QoS (queue size)
        )
        self.subscription  # prevent unused variable warning
        self.get_logger().info("Subscribed to topic: yolo_cmd")
        self.value = False

    def listener_callback(self, msg: Int32):
        self.get_logger().info(f"Received YOLO cmd: {msg.data}")
        if msg.data == 0:
            self.value = False
        elif msg.data == 1:
            self.value = True

class MoveCmdPublisher(Node):
    def __init__(self, x = 0.0, y = 0.0, z = 0.0):
        super().__init__('move_cmd_publisher')
        self.move_publisher_ = self.create_publisher(Vector3, '/go2/move_cmd', 10)
        self.stop_publisher_ = self.create_publisher(Int32, '/go2/sport_cmd', 10)

    def move_publish(self, x, y, z):
        msg = Vector3()
        msg.x, msg.y, msg.z = float(x), float(y), float(z)
        self.move_publisher_.publish(msg)
        self.get_logger().info('Publishing x: "%f"' % msg.x)
        self.get_logger().info('Publishing y: "%f"' % msg.y)
        self.get_logger().info('Publishing z: "%f"' % msg.z)
    
    def stop_publish(self):
        num = Int32()
        num.data = 6
        self.stop_publisher_.publish(num.data) # stop move

class MoveFollowLine:
    def __init__(self, move_node: MoveCmdPublisher):
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0            # 월드 기준 yaw [rad]
        self.robot_vx_world = None      # 선택: 월드 기준 속도(m/s) 있으면 PD-D 사용
        self.robot_vy_world = None

        # 내부 메모리(미분 추정용)
        self._prev_r = 0.0
        self._prev_t = time.time()

        # 제어 이득
        self.Kp_lat = 1.5               # 횡방향(라인 법선) P
        self.Kd_lat = 0.8               # 횡방향 D
        self.Kp_yaw = 2.0               # 진행 방향(yaw) P
        self.Kd_yaw = 0.2               # 진행 방향 D

        # 전진 피드포워드 속도(라인 접선 방향)
        self.v_ff = 0.6                 # m/s
        # 포화 한계
        self.vx_lim = 1.0               # m/s
        self.vy_lim = 1.0               # m/s
        self.yawrate_lim = 1.2          # rad/s
        self.move_pub = move_node

    def robot_pose_update(self, pos_x, pos_y, ori_x, ori_y, ori_z, ori_w):
        self.robot_x = pos_x
        self.robot_y = pos_y
        
        siny_cosp = 2.0 * (ori_w * ori_z + ori_x * ori_y)
        cosy_cosp = 1.0 - 2.0 * (ori_y * ori_y + ori_z * ori_z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def move_cmd(self, vx_body, vy_body, yaw_rate):
        self.move_pub.move_publish(vx_body, vy_body, yaw_rate)

    def stop_cmd(self):
        self.move_pub.stop_publisher_()
        
    def move_follow_line(self, start_x, start_y, target_x, target_y,
                         stop_at_target=True, stop_dist=0.1):
        """
        월드 좌표계의 두 점을 잇는 직선을 따라가도록 PD 제어하여 move() 호출.
        - stop_at_target=True이면 타겟 점 부근에서 정지
        - stop_dist: 타겟까지 거리가 이 값 이하이면 정지
        """
        # 1) 라인 접선/법선 벡터 계산
        dx = target_x - start_x
        dy = target_y - start_y
        L = math.hypot(dx, dy)
        if L < 1e-6:
            # 퇴화: 점 두 개가 같음 → 정지
            self.move_cmd(0.0, 0.0, 0.0)
            return

        t_hat = (dx / L, dy / L)            # 접선 단위벡터(전진 방향)
        n_hat = ( dy / L, -dx / L)          # 법선 단위벡터(왼쪽이 +)

        # 2) 현재 위치와 라인까지의 부호 있는 횡거리 r (법선방향 투영)
        #    라인 방정식: a x + b y + c = 0, a=dy, b=-(dx)
        a, b, c = dy, -dx, target_x*start_y - target_y*start_x
        r = (a*self.robot_x + b*self.robot_y + c) / math.hypot(a, b)  # == n_hat·(p - p_on_line)

        # 3) r_dot (횡오차 변화율) 추정: 월드속도 있으면 n_hat·v_world, 없으면 차분
        now = time.time()
        dt = max(1e-3, now - self._prev_t)
        if self.robot_vx_world is not None and self.robot_vy_world is not None:
            r_dot = n_hat[0]*self.robot_vx_world + n_hat[1]*self.robot_vy_world
        else:
            r_dot = (r - self._prev_r) / dt

        # 4) 진행 방향(라인 각도)과 요 오차
        theta_line = math.atan2(dy, dx)
        # 로봇 요 속도가 있다면 넣어주세요. 없을 경우 0으로 둡니다.
        robot_yawrate_meas = 0.0
        yaw_err = (theta_line - self.robot_yaw + math.pi) % (2 * math.pi) - math.pi

        # 5) 월드 기준 원하는 속도: 접선 전진 + 법선 보정(PD)
        v_tangent = self.v_ff
        v_normal = -(self.Kp_lat * r + self.Kd_lat * r_dot)

        vx_world_des = v_tangent * t_hat[0] + v_normal * n_hat[0]
        vy_world_des = v_tangent * t_hat[1] + v_normal * n_hat[1]

        # 6) 월드 → 바디 변환 (바디가 원하는 속도를 move로 보냄)
        cy = math.cos(self.robot_yaw)
        sy = math.sin(self.robot_yaw)
        # v_body = R(-yaw) v_world
        vx_body_des =  cy*vx_world_des + sy*vy_world_des
        vy_body_des = -sy*vx_world_des + cy*vy_world_des

        # 7) 요 레이트 명령 (PD)
        yaw_rate_cmd = self.Kp_yaw * yaw_err - self.Kd_yaw * robot_yawrate_meas

        # 8) 포화
        vx_body_des = max(-self.vx_lim, min(self.vx_lim, vx_body_des))
        vy_body_des = max(-self.vy_lim, min(self.vy_lim, vy_body_des))
        yaw_rate_cmd = max(-self.yawrate_lim, min(self.yawrate_lim, yaw_rate_cmd))

        # 9) 세그먼트 종료 조건(선분 기준으로 멈추고 싶을 때)
        if stop_at_target:
            dist_to_target = math.hypot(self.robot_x - target_x, self.robot_y - target_y)
            if dist_to_target <= stop_dist:
                self.stop_cmd()
                # 내부 상태 업데이트 후 리턴
                self._prev_r = r
                self._prev_t = now
                return

        # 10) 명령 전송
        self.move_cmd(vx_body_des, vy_body_des, yaw_rate_cmd)

        # 11) 내부 상태 업데이트
        self._prev_r = r
        self._prev_t = now

class Go2SportPublisher(Node):
    def __init__(self):
        super().__init__('go2_sport_publisher')
        self.pub = self.create_publisher(Int32, '/go2/sport_cmd', 10)
        self.get_logger().info("Publisher ready on /go2/sport_cmd")

    def send(self, cmd_id: int):
        msg = Int32()
        msg.data = int(cmd_id)
        self.pub.publish(msg)
        self.get_logger().info(f"Published id={cmd_id}")

class LowCmdPublisher(Node):
    """
    단순 퍼블리셔:
    - /cmd_vel : Twist를 rate Hz로 연속 발행
    - /mode_cmd : Int32를 heartbeat_hz로 연속 발행(가시성/디버깅용)
    파라미터:
      rate (float)         : /cmd_vel 발행 주기(Hz)
      vx (float)           : linear.x
      vy (float)           : linear.y
      wz (float)           : angular.z
      mode (int)           : /mode_cmd로 발행할 모드 값
      heartbeat_hz (float) : /mode_cmd 발행 주기(Hz)
      verbose (bool)       : 로그 출력 여부
    """

    def __init__(self):
        super().__init__('simple_cmdvel_pub')

        # Params
        self.declare_parameter('rate', 20.0)
        self.declare_parameter('vx', 0.0)
        self.declare_parameter('vy', 0.0)
        self.declare_parameter('wz', 0.0)
        self.declare_parameter('mode', 0)
        self.declare_parameter('heartbeat_hz', 2.0)
        self.declare_parameter('verbose', True)

        self.rate_hz     = float(self.get_parameter('rate').value)
        self.vx          = float(self.get_parameter('vx').value)
        self.vy          = float(self.get_parameter('vy').value)
        self.wz          = float(self.get_parameter('wz').value)
        self.mode        = int(self.get_parameter('mode').value)
        self.heartbeat_hz= float(self.get_parameter('heartbeat_hz').value)
        self.verbose     = bool(self.get_parameter('verbose').value)

        # Publishers
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel_low', 10)
        self.pub_mode = self.create_publisher(Int32, '/mode_cmd', 10)

        # Initial publish
        self.pub_mode.publish(Int32(data=self.mode))
        if self.verbose:
            self.get_logger().info(
                f"Started simple cmd_vel_low publisher: rate={self.rate_hz}Hz, "
                f"vx={self.vx}, vy={self.vy}, wz={self.wz}, mode={self.mode}"
            )

    def send_cmd(self):
        msg = Twist()
        msg.linear.x = self.vx
        msg.linear.y = self.vy
        msg.angular.z = self.wz
        self.pub_cmd.publish(msg)
        if self.verbose:
            self.get_logger().info(f"/cmd_vel_low publish: vx={self.vx:+.3f} vy={self.vy:+.3f} wz={self.wz:+.3f}")

    def send_mode(self):
        self.pub_mode.publish(Int32(data=self.mode))

class GetPoseSubscriber(Node):
    def __init__(self, mover: MoveCmdPublisher, follower: MoveFollowLine, sc: Go2SportPublisher, yolo: YoloCmdSubscriber, low_cmd: LowCmdPublisher):
        super().__init__('get_pose_subscriber')
        self.subscription = self.create_subscription(
            Pose,
            'robot_pose_topic',
            self.get_pose_callback,
            10)
        self.subscription  # prevent unused variable warning

        # define variable
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_z = 0.0

        self.robot_quaternion_x = 0.0
        self.robot_quaternion_y = 0.0
        self.robot_quaternion_z = 0.0
        self.robot_quaternion_w = 1.0

        self.mover = mover
        self.follower = follower
        self.sc = sc
        self.yolo = yolo
        self.low_cmd = low_cmd

        cfg = ExecConfig(
            cmd=["./go2_ctrl", "--network", "eno1"],
            cwd="/home/mr/unitree_rl_lab/deploy/robots/go2/build",
            # env=DEFAULT_MINIMAL_ENV.copy(),  # 필요시 명시
        )
        self.ex = BaseProcessExecutor(cfg)
        
        self.policy = Policy.WALK

    def get_pose_callback(self, msg):
        self.get_logger().info('robot_x: "%f"' % msg.position.x)
        self.get_logger().info('robot_y: "%f"' % msg.position.y)
        self.get_logger().info('robot_z: "%f"' % msg.position.z)
        
        self.get_logger().info('robot_quaternion_x: "%f"' % msg.orientation.x)
        self.get_logger().info('robot_quaternion_y: "%f"' % msg.orientation.y)
        self.get_logger().info('robot_quaternion_z: "%f"' % msg.orientation.z)
        self.get_logger().info('robot_quaternion_w: "%f"' % msg.orientation.w)

        # update robot pose
        self.robot_x = msg.position.x
        self.robot_y = msg.position.y
        self.robot_z = msg.position.z

        self.robot_quaternion_x = msg.orientation.x
        self.robot_quaternion_y = msg.orientation.y
        self.robot_quaternion_z = msg.orientation.z
        self.robot_quaternion_w = msg.orientation.w

        self.policy = self.execute_policy()
        self.get_logger().info(f"Current policy: {policy.name}")  # Enum 이름 출력

    def check_robot_position(self, target_x, target_y, target_z, x_bound = 0.2, y_bound = 0.2, z_bound = 1):
        if abs(target_x - self.robot_x) < x_bound and abs(target_y - self.robot_y) < y_bound and abs(target_z - self.robot_z) < z_bound:
            return True
        else:
            return False

    def execute_policy(self) -> Policy:
        match self.policy:
            case Policy.WALK:
                self.follower.robot_pose_update(self.robot_x, self.robot_y, self.robot_quaternion_x, self.robot_quaternion_y, self.robot_quaternion_z, self.robot_quaternion_w)
                self.follower.move_follow_line(0,0,0,1,1,1) # 필요에 따라 좌표 수정

                if self.check_robot_position(1, 1, 1): # 필요에 따라 수정
                    return Policy.JUMP
                else:
                    return Policy.WALK

            case Policy.JUMP:
                self.sc.send(SCid.FREE_JUMP)
                return Policy.WALK
            
            case Policy.DYNAMIC_OBSTACLE_WAIT:
                if self.yolo.value:
                    return Policy.DYNAMIC_OBSTACLE_GO
                else:
                    return Policy.DYNAMIC_OBSTACLE_WAIT
                
            case Policy.DYNAMIC_OBSTACLE_GO:
                return Policy.FUCKING_RUN
            
            case Policy.CREEP_START:
                self.sc.send(SCid.STAND_DOWN)
                msc.ReleaseMode()

                self.ex.start()
                print("started pid:", self.ex.pid)

                self.low_cmd.mode = 1
                self.low_cmd.send_mode()

                return Policy.CREEP_MID
            
            case Policy.CREEP_MID:
                self.low_cmd.mode = 2
                self.low_cmd.send_mode()

                self.low_cmd.vx = 0.6
                self.low_cmd.vy = 0
                self.low_cmd.wz = 0
                self.low_cmd.send_cmd()

                if WALK_END:
                    return Policy.CREEP_END
                else:
                    return Policy.CREEP_MID
            
            case Policy.CREEP_END:
                self.low_cmd.mode = 3
                self.low_cmd.send_mode()
                
                self.ex.stop()

                msc.SelectMode("mcf")
                return Policy.WALK
            
            case Policy.TRAFFIC_WAIT:
                return Policy.WALK
            
            case Policy.TRAFFIC_GO:
                return Policy.WALK
            
            case Policy.FUCKING_RUN:
                return Policy.WALK
            
            case Policy.TERMINATE:
                return Policy.TERMINATE

def main(args=None):
    rclpy.init(args=args)

    move_node = MoveCmdPublisher()
    follower = MoveFollowLine(move_node)
    sc_node = Go2SportPublisher()
    yolo_node = YoloCmdSubscriber()
    low_cmd_node = LowCmdPublisher()
    pose_node = GetPoseSubscriber(move_node, follower, sc_node, yolo_node, low_cmd_node)
    

    executor = MultiThreadedExecutor()
    executor.add_node(move_node)
    executor.add_node(pose_node)
    executor.add_node(sc_node)
    executor.add_node(yolo_node)
    executor.add_node(low_cmd_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        move_node.destroy_node()
        pose_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
