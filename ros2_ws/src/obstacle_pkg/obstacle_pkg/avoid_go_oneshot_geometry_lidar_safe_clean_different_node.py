#!/usr/bin/env python3
# 두 노드(탐지→주행) + stage-style run
# - DetectionNode: YOLO로 안전지점 탐지 → finish() (로봇 제어 없음)
# - RunningNode: BalanceStand 후 Move 제어만 수행(중앙정렬). TrotRun() 호출 없음.
# - 종료는 주행 타임아웃(timeout_s) 또는 시그널만 사용(결승선/완주 감지 없음)

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped

import math, time, signal, threading, queue
import numpy as np
import cv2

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.sport.sport_client import SportClient
from unitree_sdk2py.go2.video.video_client import VideoClient
from ultralytics import YOLO


# ---- SDK 채널 초기화(전역 한 번) ----
try:
    ChannelFactoryInitialize(0)
except Exception as e:
    print(f'ChannelFactoryInitialize failed: {e}')
    raise


def target_yaw(z, w):
    return math.atan2(2*w*z, 1 - 2*z**2)


# 필요 시 변경
#YAW_TARGET = target_yaw(-0.3126348728838737, 0.9449064534569476)
YAW_TARGET = target_yaw(-0.13178851904133532, 0.9912778552196612)


# ---- 안전 종료 헬퍼 ----
def _safe_client_close(sc):
    for m in ("Close", "close", "Deinit", "DeInitialize", "Disconnect", "Release"):
        fn = getattr(sc, m, None)
        if callable(fn):
            try: fn()
            except Exception: pass
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
            try: fn()
            except Exception: pass
            break


# ---- 유틸 ----
def is_safe_spot(xc, yc):
    # 간단 규칙: 좌/우 가장자리면 안전
    return (xc < 0.2) or (xc > 0.8)


# ===================== 1) DetectionNode =====================
class DetectionNode(Node):
    """YOLO + VideoClient로 안전지점 감지되면 finish(). 로봇 제어 없음."""
    def __init__(self):
        super().__init__('detection_node')

        self.declare_parameter("model_path", "/home/mr/ros2_ws/src/obstacle_pkg/models/best.pt")
        model_path = self.get_parameter('model_path').get_parameter_value().string_value

        self.done = False
        self.stop_ev = threading.Event()
        self.raw_q = queue.Queue(maxsize=2)

        # YOLO
        try:
            self.model = YOLO(model_path)
            self.get_logger().info(f"YOLO loaded: {model_path} (device={self.model.device})")
        except Exception as e:
            self.get_logger().error(f"YOLO load failed: {e}")
            raise

        # 타이머(선택)
        self.hb_timer = self.create_timer(4.0, self._heartbeat)

        # 스레드
        self.t_vid = threading.Thread(target=self.video_loop, daemon=True)
        self.t_det = threading.Thread(target=self.yolo_worker, daemon=True)
        self.t_vid.start(); time.sleep(0.05)
        self.t_det.start()

        # 시그널
        signal.signal(signal.SIGINT, self._sig_handler)
        signal.signal(signal.SIGTERM, self._sig_handler)

    def finish(self, reason=""):
        if self.done: return
        self.get_logger().info(f"[Detection FINISH] {reason}")
        self.done = True
        self.stop_ev.set()
        self._cleanup()

    def _sig_handler(self, signum, frame):
        self.finish(f"signal {signum}")

    def _destroy_timers(self):
        try:
            if hasattr(self, "hb_timer") and self.hb_timer:
                self.hb_timer.cancel()
                self.destroy_timer(self.hb_timer)
                self.hb_timer = None
        except Exception: pass

    def _cleanup(self):
        self._destroy_timers()

    def destroy_node(self):
        try: self._cleanup()
        except Exception: pass
        super().destroy_node()

    # ---- 워커 ----
    def yolo_worker(self):
        while not self.stop_ev.is_set() and not self.done:
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
                best = None
                if boxes is not None and len(boxes) > 0:
                    xyxy = boxes.xyxy.cpu().numpy()
                    confs = boxes.conf.cpu().numpy()
                    for (x1, y1, x2, y2), p in zip(xyxy, confs):
                        x1, y1, x2, y2 = map(int, [x1, y1, x2, y2])
                        if x2 <= x1 or y2 <= y1: continue
                        xc = ((x1 + x2) / 2) / w
                        yc = ((y1 + y2) / 2) / h
                        cand = {"xc": xc, "yc": yc, "conf": float(p)}
                        if best is None or cand["conf"] > best["conf"]:
                            best = cand
                if best is not None:
                    print(best)

                if best is not None and is_safe_spot(best["xc"], best["yc"]):
                    self.get_logger().info(f"Safe spot detected: xc={best['xc']:.3f}")
                    self.finish("safe detected")
            except Exception as e:
                self.get_logger().warning(f"[YOLO] error: {e}")
                continue

    def video_loop(self):
        client = VideoClient()
        client.SetTimeout(3.0)
        client.Init()
        while not self.stop_ev.is_set() and not self.done:
            code, data = client.GetImageSample()
            if code != 0:
                self.get_logger().error('VideoClient error')
                self.finish("video error"); break
            try:
                frame = cv2.imdecode(np.frombuffer(bytes(data), dtype=np.uint8), cv2.IMREAD_COLOR)
                if frame is None: continue
                if not self.raw_q.empty():
                    try: self.raw_q.get_nowait()
                    except queue.Empty: pass
                self.raw_q.put_nowait(frame)
            except Exception:
                continue

    def _heartbeat(self):
        self.get_logger().info(f"[DETECT] raw_q={self.raw_q.qsize()}, done={self.done}")


# ===================== 2) RunningNode =====================
class RunningNode(Node):
    """
    - BalanceStand 후 Move 제어만 수행(중앙정렬).
    - 결승선/완주 조건으로 finish() 호출하지 않음.
    - 종료는 오직 run_stage(timeout_s=...) 또는 시그널.
    """
    def __init__(self, yaw_target):
        super().__init__('running_node')

        self.done = False

        # 제어 게인
        self.vx = 3.7
        self.vy = 0.0
        self.wz = 0.0
        self.kp = 1.5          # LiDAR → vy
        self.kp2 = 5.0         # pose → wz
        self.error = 0.0
        self.yaw = 0.0
        self.yaw_target = float(yaw_target)

        # Unitree
        self.sc = SportClient(enableLease=False)
        self.sc.SetTimeout(10.0)
        self.sc.Init()
        try:
            self.sc.BalanceStand()
        except Exception:
            pass
        # 주행 모드는 사용하지 않음 (TrotRun() 호출 금지)

        # 구독
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.lidar_cb, qos_profile_sensor_data)
        self.sub_pose = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.yaw_cb, 3)

        # 시그널
        signal.signal(signal.SIGINT, self._sig_handler)
        signal.signal(signal.SIGTERM, self._sig_handler)

    # 종료 경로
    def finish(self, reason=""):
        if self.done: return
        self.get_logger().info(f"[Running FINISH] {reason}")
        self.done = True
        try: self.sc.Move(0, 0, 0)
        except Exception: pass
        self._cleanup()

    def _sig_handler(self, signum, frame):
        self.finish(f"signal {signum}")

    def _destroy_subs(self):
        try:
            if self.sub_scan:
                self.destroy_subscription(self.sub_scan); self.sub_scan = None
        except Exception: pass
        try:
            if self.sub_pose:
                self.destroy_subscription(self.sub_pose); self.sub_pose = None
        except Exception: pass

    def _cleanup(self):
        self._destroy_subs()
        try: self.sc.StopMove()
        except Exception: pass
        _safe_client_close(self.sc)
        _safe_sdk_finalize()

    def destroy_node(self):
        try: self._cleanup()
        except Exception: pass
        super().destroy_node()

    # 콜백
    def yaw_cb(self, msg: PoseWithCovarianceStamped):
        if self.done: return
        q = msg.pose.pose.orientation
        self.yaw = float(target_yaw(q.z, q.w))
        self.yaw = (self.yaw + math.pi) % (2.0 * math.pi) - math.pi
        yaw_error = self.yaw_target - self.yaw
        if abs(yaw_error) < 0.05: yaw_error = 0.0
        self.wz = max(-4.0, min(4.0, self.kp2 * yaw_error))

    def lidar_cb(self, msg: LaserScan):
        if self.done: return
        left = [r for r in msg.ranges[150:180] if r > 0.1]
        right = [r for r in msg.ranges[0:30] if r > 0.1]
        if not left or not right: return

        left_dist = sum(left)/len(left)
        right_dist = sum(right)/len(right)

        # 중앙 정렬만 수행(완주/종료 판정 없음)
        if left_dist + right_dist <= 1.2:
            self.error = left_dist - right_dist

        self.vy = self.kp * self.error
        if abs(self.vy) > 3.7:
            self.vy = math.copysign(3.7, self.vy)

        try:
            self.sc.Move(float(self.vx), float(self.vy), float(self.wz))
        except Exception as e:
            self.get_logger().warning(f"Move failed: {e}")


# ===================== 3) Stage runner =====================
def run_stage(executor, node, timeout_s=None):
    start = time.time()
    while rclpy.ok() and not node.done:
        executor.spin_once(timeout_sec=0.05)
        if timeout_s and (time.time() - start) > timeout_s:
            node.finish("stage timeout")
            break

    # 스테이지 종료 시: 안전 정지 & 정리
    try:
        if hasattr(node, "sc") and node.sc is not None:
            node.sc.Move(0.0, 0.0, 0.0)
    except Exception:
        pass
    try: executor.remove_node(node)
    except Exception: pass
    try: node.destroy_node()
    except Exception: pass


# ===================== 4) main =====================
def main(args=None):
    rclpy.init(args=args)
    executor = SingleThreadedExecutor()

    try:
        # 1) 탐지 단계(안전지점 나올 때까지 대기)
        det = DetectionNode()
        executor.add_node(det)
        run_stage(executor, det)  # 타임아웃 없음

        # 2) 주행 단계(시간 고정: timeout_s만으로 종료)
        runn = RunningNode(yaw_target=YAW_TARGET)
        executor.add_node(runn)
        run_stage(executor, runn, timeout_s=4.0)  # ← 여기 시간만 바꿔서 주행 시간 조절

    finally:
        try: executor.shutdown()
        except Exception: pass
        try: rclpy.shutdown()
        except Exception: pass


if __name__ == "__main__":
    main()
