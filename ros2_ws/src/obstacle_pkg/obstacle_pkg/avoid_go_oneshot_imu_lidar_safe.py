#!/usr/bin/env python3
# oneshot, high move command, with IMU, close automatically
# no need to send stop cmd command

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import LaserScan

from unitree_go.msg import LowState
from rclpy.qos import qos_profile_sensor_data

import math, time, os, signal, threading, queue, sys
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.sport.sport_client import SportClient
from unitree_sdk2py.go2.video.video_client import VideoClient

import numpy as np
import cv2
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory


# ---- SDK 채널 초기화(전역 한 번) ----
try:
    ChannelFactoryInitialize(0)
except Exception as e:
    print(f'ChannelFactoryInitialize failed: {e}')
    raise


# ------------------------- safe helpers -------------------------

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


# ------------------------- utility -------------------------

def iou_xyxy(a, b):
    ax1, ay1, ax2, ay2 = a; bx1, by1, bx2, by2 = b
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    iw, ih = max(0, ix2 - ix1), max(0, iy2 - iy1)
    inter = iw * ih
    area_a = max(0, ax2 - ax1) * max(0, ay2 - ay1)
    area_b = max(0, bx2 - bx1) * max(0, by2 - by1)
    union = area_a + area_b - inter
    return inter / union if union > 0 else 0.0

def is_safe_spot(xc, yc):
    # 간단 규칙: 좌/우 가장자리면 안전
    return (xc < 0.2) or (xc > 0.8)


# ------------------------- ROS 2 Node -------------------------

class ObstacleDecisionNode(Node):
    def __init__(self):
        super().__init__('obstacle_decision_node')

        # ---- Parameters ----
        self.declare_parameter(
            "model_path",
            "/home/mr/ros2_ws/src/obstacle_pkg/models/best.pt"
            
            #os.path.join(
             #   get_package_share_directory("obstacle_pkg"),
              #  "models",
               # "bestnew.pt"
            #)
        )
        
        # self.declare_parameter('unitree_arg', 'eno1')
        self.declare_parameter('auto_exit_sec', 2.0)

        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        # self.unitree_arg = self.get_parameter('unitree_arg').get_parameter_value().string_value
        self.auto_exit_sec = float(self.get_parameter('auto_exit_sec').value)

        # ---- Queues & Events ----
        self.raw_q = queue.Queue(maxsize=2)
        self.stop_ev = threading.Event()
        self.started = threading.Event()     # 원샷 시작 여부
        self.want_start = False              # 워커→메인 스레드 신호
        self.stopping = False

        # ---- State / Ctrl gains ----
        self.vx = 1.0
        self.vy = 0.0
        self.wz = 0.0

        self.kp = 1.5      # Lidar → vy
        self.kp2 = 5.0     # IMU → wz
        self.error = 0.0
        self.yaw = 0.0
        self.yaw_target = 0.691239595413208  # 약 39.6도

        # ---- YOLO ----
        try:
            self.model = YOLO(model_path)
            self.get_logger().info(f'YOLO loaded: {model_path} (device={self.model.device})')
        except Exception as e:
            self.get_logger().error(f'YOLO load failed: {e}')
            raise

        # ---- Unitree SportClient ----
        self.sc = SportClient(enableLease=False)
        self.sc.SetTimeout(10.0)
        self.sc.Init()
        self.get_logger().info("Initializing Unitree SportClient...")
        try:
            self.sc.BalanceStand()
        except Exception as e:
            self.get_logger().warn(f'BalanceStand failed: {e}')

        # 필요 환경일 때 API 원격 제어 허용
        try:
            self.sc.UseRemoteCommandFromApi(True)
        except Exception:
            pass

        # ---- Subscriptions (미리 생성, 콜백 가드로 제어) ----
        self.sub_scan = self.create_subscription(
            LaserScan, '/scan', self.lidar_cb, qos_profile_sensor_data
        )
        self.sub_lowstate = self.create_subscription(
            LowState, '/lowstate', self.yaw_callback, 3
        )

        # ---- Timers ----
        self.exit_timer = None                 # 시작 후 자동 종료
        self.arm_timer = self.create_timer(0.02, self._maybe_start)  # 20ms 폴링
        self.hb_timer = self.create_timer(2.0, self._heartbeat)

        # ---- Threads ----
        self.t_vid = threading.Thread(target=self.video_loop, daemon=True)
        self.t_det = threading.Thread(target=self.yolo_worker, daemon=True)
        self.t_vid.start(); time.sleep(0.05)
        self.t_det.start()

        # ---- Signal handlers ----
        signal.signal(signal.SIGINT, self._sig_handler)
        signal.signal(signal.SIGTERM, self._sig_handler)

    # --------------------- callbacks / timers ---------------------

    def _maybe_start(self):
        """메인 스레드: 워커가 세운 want_start를 감지해 즉시 시작/타이머 생성."""
        if self.stopping or self.started.is_set() or not self.want_start:
            return
        self.want_start = False
        self.started.set()
        self.get_logger().info('START triggered by YOLO (oneshot).')

        # 자동 종료 타이머
        if self.auto_exit_sec > 0.0 and self.exit_timer is None:
            self.exit_timer = self.create_timer(self.auto_exit_sec, self._on_exit_timer)

    def _on_exit_timer(self):
        self.get_logger().info('Auto-exit timer: stopping and shutting down...')
        self._cleanup()
        # 노드 파괴는 main()에서 일괄 처리
        try:
            rclpy.shutdown()
        except Exception:
            pass

    def yaw_callback(self, msg: LowState):
        if not self.started.is_set() or self.stopping:
            return
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
        if abs(self.wz) > 4.0:
            self.wz = math.copysign(4.0, self.wz)

    def lidar_cb(self, msg: LaserScan):
        if not self.started.is_set() or self.stopping:
            return
        # 0~30도 / 150~180도 가정 (센서 해상도 다르면 보정 필요)
        left_ranges = [r for r in msg.ranges[150:180] if r > 0.1]
        right_ranges = [r for r in msg.ranges[0:30] if r > 0.1]
        if not left_ranges or not right_ranges:
            return

        left_dist = sum(left_ranges)/len(left_ranges)
        right_dist = sum(right_ranges)/len(right_ranges)

        # 양쪽 다 충분히 가까울 때만 벽 따라가기 오차 갱신
        if left_dist <= 0.85 and right_dist <= 0.85:
            self.error = left_dist - right_dist
        self.vy = self.kp * self.error

        # 속도 포화(임계=포화 3.7 통일)
        if abs(self.vy) > 3.7:
            self.vy = math.copysign(3.7, self.vy)

        try:
            self.sc.Move(float(self.vx), float(self.vy), float(self.wz))
        except Exception as e:
            self.get_logger().warn(f"Move failed: {e}")

    def _sig_handler(self, signum, frame):
        self.get_logger().info(f'Signal {signum} received → stopping...')
        self.stopping = True
        self.stop_ev.set()
        try:
            rclpy.shutdown()
        except Exception:
            pass

    def _heartbeat(self):
        self.get_logger().debug(f'q(raw={self.raw_q.qsize()})')

    # --------------------- workers ---------------------

    def yolo_worker(self):
        """프레임에서 안전 지점 처음 감지되면 want_start만 True로 설정(ROS 엔티티 생성 X)"""
        while not self.stop_ev.is_set():
            try:
                frame = self.raw_q.get(timeout=0.1)
                h, w = frame.shape[:2]
            except queue.Empty:
                continue
            except Exception:
                continue

            try:
                res = self.model.predict(source=frame, conf=0.1, verbose=False)[0]
                boxes = res.boxes
                chosen = None

                if boxes is not None and len(boxes) > 0:
                    xyxy = boxes.xyxy.cpu().numpy()
                    confs = boxes.conf.cpu().numpy()
                    for (x1, y1, x2, y2), p in zip(xyxy, confs):
                        x1, y1, x2, y2 = map(int, [x1, y1, x2, y2])
                        if x2 <= x1 or y2 <= y1:
                            continue
                        xc = ((x1 + x2) / 2) / w
                        yc = ((y1 + y2) / 2) / h
                        cand = {"xc": xc, "yc": yc, "conf": float(p)}
                        if chosen is None or cand["conf"] > chosen["conf"]:
                            chosen = cand
                print("chosen is: ", chosen)

                if chosen is not None and not self.started.is_set():
                    safe = is_safe_spot(chosen["xc"], chosen["yc"])
                    self.get_logger().info(f"is_safe={safe}, xc={chosen['xc']:.3f}, yc={chosen['yc']:.3f}")
                    
                    if safe:
                        # 즉시 시작 신호
                        self.want_start = True
                        # 반응 지연을 최소화하려면 첫 Move 한번 즉시 쏘기(선택)
                        try:
                            self.sc.Move(float(self.vx), float(self.vy), float(self.wz))
                        except Exception:
                            pass
                            

            except Exception as e:
                self.get_logger().warning(f'[YOLO] error: {e}')
                continue

    def video_loop(self):
        client = VideoClient()
        client.SetTimeout(3.0)
        client.Init()

        while not self.stop_ev.is_set():
            code, data = client.GetImageSample()
            if code != 0:
                self.get_logger().error(f'VideoClient error code: {code}')
                self.stop_ev.set()
                break
            try:
                image_data = np.frombuffer(bytes(data), dtype=np.uint8)
                frame = cv2.imdecode(image_data, cv2.IMREAD_COLOR)
                if frame is None:
                    continue
                if not self.raw_q.empty():
                    try:
                        self.raw_q.get_nowait()
                    except queue.Empty:
                        pass
                self.raw_q.put_nowait(frame)
            except Exception:
                continue

    # --------------------- shutdown ---------------------

    def _destroy_timers_and_subs(self):
        # timers
        try:
            if self.exit_timer:
                self.exit_timer.cancel()
                self.destroy_timer(self.exit_timer)
                self.exit_timer = None
        except Exception:
            pass
        try:
            if self.arm_timer:
                self.arm_timer.cancel()
                self.destroy_timer(self.arm_timer)
                self.arm_timer = None
        except Exception:
            pass

        # subscriptions
        try:
            if self.sub_scan:
                self.destroy_subscription(self.sub_scan)
                self.sub_scan = None
        except Exception:
            pass
        try:
            if self.sub_lowstate:
                self.destroy_subscription(self.sub_lowstate)
                self.sub_lowstate = None
        except Exception:
            pass

    def stop(self):
        self.stop_ev.set()
        # 영상 스레드 조인
        try:
            if self.t_vid.is_alive():
                self.t_vid.join(timeout=2.0)
        except Exception:
            pass

    def _cleanup(self):
        self.stopping = True
        # 1) 동작 정지
        try:
            self.sc.StopMove()
        except Exception:
            pass
        try:
            self.sc.UseRemoteCommandFromApi(False)
        except Exception:
            pass

        # 2) ROS 엔티티 해제
        self._destroy_timers_and_subs()

        # 3) 스레드 정지
        self.stop()

        # 4) SDK 클라이언트/팩토리 정리
        _safe_client_close(self.sc)
        _safe_sdk_finalize()

    def destroy_node(self):
        # 중복 안전
        self._cleanup()
        super().destroy_node()


def _hard_kill_if_needed(grace_sec=0.3):
    time.sleep(grace_sec)
    alive = [t for t in threading.enumerate() if t is not threading.main_thread()]
    if alive:
        os._exit(0)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDecisionNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
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

    _hard_kill_if_needed(grace_sec=0.3)


if __name__ == '__main__':
    main()
