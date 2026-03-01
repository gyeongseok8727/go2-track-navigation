#!/usr/bin/env python3
# Refactored: stage-style closing with a single `done` flag.
# - No `_maybe_start`, no `arm_timer`, no `want_start`
# - decision_worker에서 바로 `self.started.set()` 호출
# - 종료는 `finish()` 한곳으로 수렴, rclpy.shutdown()은 메인에서만

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import qos_profile_sensor_data

import math, time, signal, threading, queue, os
import numpy as np
import cv2

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.video.video_client import VideoClient
from unitree_sdk2py.go2.sport.sport_client import SportClient

from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory


# ---- SDK 채널 초기화(전역 한 번) ----
try:
    ChannelFactoryInitialize(0)
except Exception as e:
    print(f'ChannelFactoryInitialize failed: {e}')
    raise


# target_yaw atan2( 2wz, 1 - 2*z² )
def target_yaw(z, w):
    return math.atan2(2*w*z, 1 - 2*z**2)

#yaw_target needs edit
def shortest_ang_err(t, c):
    return math.atan2(math.sin(t - c), math.cos(t - c))

YAW_TARGET = target_yaw(0.9999976873100451, 0.002150668398693065)


#------------------------safe helpers------------
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


def detect_color(
    roi_bgr,
    v_min=170, s_min=100,                 # 밝기/채도 하한(0~255)
    use_color_hint=True,                  # H(색상) 힌트 가산
    center_band_sigma_ratio=0.18,         # 중앙 가우시안 가중 폭 (W*ratio)
    gap_ratio=0.02,                       # 삼등분 경계 divide (H*ratio)
    sep_ratio=1.08,                       # 단일등 판단용 1등/2등 분리 비율
    abs_v_floor=100,                      # 단일/쌍 모두에 적용할 절대 밝기 하한
    allow_red_yellow=True,                # 빨+노 동시 상태 허용 여부
    pair_balance_ratio=0.80,              # red↔yellow 균형도 임계
    pair_domination_ratio=1.12            # (red, yellow) 각각이 green 대비 우세해야 할 비율
):
    if roi_bgr is None or roi_bgr.size == 0:
        return {"state": "unknown", "reason": "empty_roi"}

    hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)
    v_eq = v  # CLAHE 미사용

    # 밝고 + 채도x 픽셀만 마스크
    bright_mask = ((v_eq >= v_min) & (s <= s_min)).astype(np.uint8) * 255
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    bright_mask = cv2.morphologyEx(bright_mask, cv2.MORPH_OPEN, k, iterations=1)
    bright_mask = cv2.dilate(bright_mask, k, iterations=1)

    H, W = roi_bgr.shape[:2]
    if H < 9 or W < 9 or np.count_nonzero(bright_mask) < 20:
        return {"state": "unknown", "reason": "too_dark_or_small"}

    # 중앙 가우시안 가중(가로) → 세로 프로파일
    x = np.arange(W, dtype=np.float32)
    sigma = max(1.0, W * center_band_sigma_ratio)
    wx = np.exp(-0.5 * ((x - W / 2) / sigma) ** 2)
    wx = wx / (wx.sum() + 1e-6)

    v_f = v_eq.astype(np.float32)
    m_f = (bright_mask > 0).astype(np.float32)
    num = (v_f * m_f) * wx[None, :]
    den = (m_f) * wx[None, :]

    thirds = np.linspace(0, H, 4).astype(int)
    gap = int(H * gap_ratio)
    y0t, y1t = thirds[0], max(thirds[1] - gap, thirds[0] + 1)
    y0m, y1m = min(thirds[1] + gap, thirds[2] - gap), max(thirds[2] - gap, thirds[1] + gap + 1)
    y0b, y1b = min(thirds[2] + gap, H - 1), thirds[3]

    def seg_score(y0, y1, min_cov_ratio=0.05):
        seg_num = float(num[y0:y1, :].sum())
        seg_den = float(den[y0:y1, :].sum())
        min_den = max(1e-6, (y1 - y0) * min_cov_ratio)
        if seg_den < min_den:
            return 0.0, seg_den
        return seg_num / (seg_den + 1e-6), seg_den

    top, den_t = seg_score(y0t, y1t)
    mid, den_m = seg_score(y0m, y1m)
    bot, den_b = seg_score(y0b, y1b)

    if use_color_hint:
        def color_boost(y0, y1):
            reg_mask = (bright_mask[y0:y1] > 0)
            if int(reg_mask.sum()) < 50:
                return 0.0, 0.0, 0.0
            Hreg = h[y0:y1][reg_mask]
            red_ratio    = ((Hreg <= 10) | (Hreg >= 170)).mean()
            yellow_ratio = ((Hreg >= 15) & (Hreg <= 35)).mean()
            green_ratio  = ((Hreg >= 45) & (Hreg <= 85)).mean()
            return float(red_ratio), float(yellow_ratio), float(green_ratio)

        r_t, y_t, g_t = color_boost(y0t, y1t) if den_t > 0 else (0, 0, 0)
        r_m, y_m, g_m = color_boost(y0m, y1m) if den_m > 0 else (0, 0, 0)
        r_b, y_b, g_b = color_boost(y0b, y1b) if den_b > 0 else (0, 0, 0)

        top += 0.08 * (1.0 * r_t + 0.6 * y_t + 0.4 * g_t)
        mid += 0.08 * (0.8 * r_m + 1.0 * y_m + 0.6 * g_m)
        bot += 0.08 * (0.5 * r_b + 0.7 * y_b + 1.0 * g_b)

    scores = {"top": top, "mid": mid, "bot": bot}

    # (A) 빨+노 동시
    if allow_red_yellow:
        red_on = top >= abs_v_floor
        yel_on = mid >= abs_v_floor
        grn_on = bot >= abs_v_floor

        if red_on and yel_on:
            balance = (min(top, mid) + 1e-6) / (max(top, mid) + 1e-6)
            dom_top = (top + 1e-6) / (bot + 1e-6)
            dom_mid = (mid + 1e-6) / (bot + 1e-6)
            domination_ok = (dom_top >= pair_domination_ratio) and (dom_mid >= pair_domination_ratio)
            if (balance >= pair_balance_ratio) and domination_ok:
                conf_balance = min(balance / pair_balance_ratio, 1.0)
                conf_dom = min(min(dom_top, dom_mid) / pair_domination_ratio, 1.0)
                confidence = float(0.5 * conf_balance + 0.5 * conf_dom)
                return {"state": "ry", "scores": scores, "confidence": confidence, "height_width": (int(H), int(W))}

    # (B) 단일등
    best_key, best_val = max(scores.items(), key=lambda kv: kv[1])
    second_val = sorted(scores.values(), reverse=True)[1]
    ratio = (best_val + 1e-6) / (second_val + 1e-6)
    abs_ok = best_val >= abs_v_floor
    sep_ok = ratio >= sep_ratio

    if abs_ok and sep_ok:
        state_map = {"top": "r", "mid": "y", "bot": "g"}
        state = state_map[best_key]
        confidence = float(min(ratio / 1.5, 1.0))
        return {"state": state, "scores": scores, "confidence": confidence, "height_width": (int(H), int(W))}

    # (C) 불확실
    return {"state": "unknown", "scores": scores, "confidence": 0.0, "height_width": (int(H), int(W))}


# ------------------------- ROS2 Node -------------------------

class DetectionNode(Node):
    def __init__(self):
        super().__init__("detection_node")

        # ---- Params----
        # pkg_share = get_package_share_directory("traffic_pkg")
        # default_model_path = os.path.join(pkg_share, "models", "besttraffic.pt")
        default_model_path = "/home/mr/ros2_ws/src/traffic_pkg/models/besttraffic.pt"

        self.declare_parameter("model_path", default_model_path)
        self.declare_parameter("unitree_arg", "eno1")
        #self.declare_parameter('auto_exit_sec', 0.0)  # 사용 안 함(남겨두어도 finish로만 종료)

        model_path = self.get_parameter("model_path").get_parameter_value().string_value
        self.unitree_arg = self.get_parameter("unitree_arg").get_parameter_value().string_value
        #self.auto_exit_sec = float(self.get_parameter('auto_exit_sec').value)

        # ---- Queues & Event ----
        self.raw_q = queue.Queue(maxsize=2)
        self.sig_q = queue.Queue(maxsize=5)
        self.stop_ev = threading.Event()

        # ---- Lifecycle flags ----
        self.done = False  # ✅ 단일 종료 플래그

        

        # ---- YOLO ----
        try:
            self.model = YOLO(model_path)
            self.get_logger().info(f'YOLO loaded: {model_path} (device={self.model.device})')
        except Exception as e:
            self.get_logger().error(f"YOLO load failed: {e}")
            raise


        # ---- Timers ----
        self.hb_timer = self.create_timer(4.0, self._heartbeat)

        # ---- Threads ----
        self.t_vid = threading.Thread(target=self.video_loop, daemon=True)
        self.t_det = threading.Thread(target=self.yolo_worker, daemon=True)
        self.t_dec = threading.Thread(target=self.decision_worker, daemon=True)

        self.t_vid.start()
        time.sleep(0.05)
        self.t_det.start()
        self.t_dec.start()

        # ---- Signal handlers → finish()만 호출 ----
        signal.signal(signal.SIGINT, self._sig_handler)
        signal.signal(signal.SIGTERM, self._sig_handler)

    # ======= Stage-style Closing: Single exit point =======

    def finish(self, reason=""):
        """모든 종료 트리거가 들어오는 단일 함수. 여기서는 rclpy.shutdown()을 호출하지 않음."""
        if self.done:
            return
        self.get_logger().info(f"[FINISH] {reason}")
        self.done = True

        # 워커에게 종료 신호
        self.stop_ev.set()

        # 내부 정리
        self._cleanup()

    def _sig_handler(self, signum, frame):
        self.finish(f"signal {signum}")

    # ======= Cleanup helpers =======

    def _destroy_timers(self):
        # timers
        try:
            if hasattr(self, "hb_timer") and self.hb_timer:
                self.hb_timer.cancel(); self.destroy_timer(self.hb_timer); self.hb_timer = None
        except Exception: pass


    def _cleanup(self):
        # 1) ROS 엔티티 해제
        self._destroy_timers()


    def destroy_node(self):
        # 이중 호출 대비 안전
        try:
            self._cleanup()
        except Exception:
            pass
        super().destroy_node()


    def yolo_worker(self):
        while not self.stop_ev.is_set():
            try:
                frame = self.raw_q.get(timeout=0.1)
            except queue.Empty:
                continue

            try:
                res = self.model.predict(source=frame, conf=0.25, verbose=False)[0]
                chosen = None
                boxes = res.boxes

                if boxes is not None and len(boxes) > 0:
                    xyxy = boxes.xyxy.cpu().numpy()
                    confs = boxes.conf.cpu().numpy()

                    for (x1, y1, x2, y2), p in zip(xyxy, confs):
                        x1, y1, x2, y2 = map(int, [x1, y1, x2, y2])
                        if x2 <= x1 or y2 <= y1:
                            continue

                        roi = frame[max(0, y1):max(0, y2), max(0, x1):max(0, x2)]
                        color = detect_color(roi)["state"]

                        cand = {"color": color, "conf": float(p), "bbox": (int(x1), int(y1), int(x2), int(y2))}
                        if chosen is None or cand["conf"] > chosen["conf"]:
                            chosen = cand

                # send compact signal to decision worker
                if chosen is not None:
                    sig = (chosen["color"], chosen["conf"], chosen["bbox"], time.time())
                    if not self.sig_q.empty():
                        try:
                            self.sig_q.get_nowait()
                        except queue.Empty:
                            pass
                    self.sig_q.put_nowait(sig)

            except Exception as e:
                self.get_logger().warning(f"[YOLO] error: {e}")
                continue

    def decision_worker(self):
        K_G, K_R = 2, 2
        IOU_MIN, C_MIN = 0.5, 0.5

        state = "INIT"         # INIT, INIT_G, INIT_R, WAIT_R, WAIT_G, CROSSING
        g_cnt = r_cnt = 0
        last_bbox = None

        def same_lamp(prev, cur):
            return (prev is None) or (iou_xyxy(prev, cur) >= IOU_MIN)

        while not self.stop_ev.is_set():
            try:
                color, conf, bbox, ts = self.sig_q.get(timeout=0.1)
            except queue.Empty:
                continue

            # 동일등 체크
            if not same_lamp(last_bbox, bbox):
                last_bbox=bbox
                continue
            last_bbox=bbox

            if conf < C_MIN:
                continue

            # 카운트 갱신
            if color == "g":
                g_cnt += 1; r_cnt = 0
            elif color == "r":
                r_cnt += 1; g_cnt = 0
            else:
                pass

            # 상태 전이
            if state == "INIT":
                if color == "g":
                    state = "INIT_G"; g_cnt = 1; r_cnt = 0
                elif color == "r":
                    state = "INIT_R"; r_cnt = 1; g_cnt = 0

            elif state == "INIT_G":
                if r_cnt >= K_R:
                    state = "WAIT_G"; g_cnt = 0   # 이제 초록 3회 기다리면 GO

            elif state == "INIT_R":
                if g_cnt >= K_G:
                    # GO!

                    # ✅ 바로 시작 (타이머 없이)
                    self.finish("go detected")

                    self.get_logger().info("GO (Gx3 after INIT_R)")
                    state = "CROSSING"

            elif state == "WAIT_G":
                if g_cnt >= K_G:

                    # 바로 시작 (타이머 없이)
                    self.finish("go detected")
                    self.get_logger().info("GO (Gx3 after R-phase)")
                    state = "CROSSING"

            elif state == "CROSSING":
                # 이동 중엔 비전 무시(중도 정지 없음).
                # (선택) 완주 조건 달성 시 finish() 호출 가능
                pass

            # debug
            print('state: ', state, 'g_cnt: ', g_cnt, 'r_cnt: ', r_cnt)

    def video_loop(self):
        client = VideoClient()
        client.SetTimeout(3.0)
        client.Init()

        while not self.stop_ev.is_set():
            code, data = client.GetImageSample()
            if code != 0:
                self.get_logger().error(f"VideoClient error code: {code}")
                self.finish("video client error")
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

    def _heartbeat(self):
        # Simple health log (optional)
        self.get_logger().info(
            f"q(raw={self.raw_q.qsize()}, sig={self.sig_q.qsize()}), done={self.done}"
        )

class RunningNode(Node):
    def __init__(self, yaw_target, use_one_side_lidar = True, use_side_lidar = True):
        super().__init__("running_node")

    
    # ---- State / Ctrl gains ----
        self.vx = 3.0
        self.vy = 0.0
        self.wz = 0.0

        self.kp = 1.5      # Lidar → vy
        self.kp2 = 5.0     # pose → wz
        self.error = 0.0
        self.yaw = 0.0
        self.yaw_target = float(yaw_target)
        self.use_one_side_lidar = use_one_side_lidar
        self.use_side_lidar = True

        self.done = False  # 단일 종료 플래그
        
        # -----unitree sport client----------
        self.sc = SportClient(enableLease=False)
        self.sc.SetTimeout(10.0)
        self.sc.Init()
        self.get_logger().info("Initializing Unitree SportClient...")
        try:
            self.sc.BalanceStand()
        except Exception as e:
            self.get_logger().warning(f'BalanceStand failed: {e}')

        # ---- Subscriptions ----
        self.sub_scan = self.create_subscription(
            LaserScan, '/scan', self.lidar_cb, qos_profile_sensor_data
        )
        self.sub_pose = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.yaw_callback, 3
        )

        self.sc.TrotRun()
        self.lidartime = time.time()

    # ======= Stage-style Closing: Single exit point =======

    def finish(self, reason=""):
        """모든 종료 트리거가 들어오는 단일 함수. 여기서는 rclpy.shutdown()을 호출하지 않음."""
        if self.done:
            return
        self.get_logger().info(f"[FINISH] {reason}")
        self.done = True

        # 즉시 로봇 정지
        try:
            self.sc.Move(0, 0, 0)
        except Exception:
            pass

        # 내부 정리
        self._cleanup()

    def _sig_handler(self, signum, frame):
        self.finish(f"signal {signum}")

    # ======= Cleanup helpers =======

    def _destroy_timers_and_subs(self):
        # timers
        try:
            if hasattr(self, "hb_timer") and self.hb_timer:
                self.hb_timer.cancel(); self.destroy_timer(self.hb_timer); self.hb_timer = None
        except Exception: pass
        # subs
        try:
            if self.sub_scan:
                self.destroy_subscription(self.sub_scan); self.sub_scan = None
        except Exception: pass
        try:
            if self.sub_pose:
                self.destroy_subscription(self.sub_pose); self.sub_pose = None
        except Exception: pass

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

    def destroy_node(self):
        # 이중 호출 대비 안전
        try:
            self._cleanup()
        except Exception:
            pass
        super().destroy_node()

    # ======= Control callbacks/workers =======

    def yaw_callback(self, msg):
        if self.done:
            return
        q = msg.pose.pose.orientation
        self.yaw = float(target_yaw(q.z, q.w))
        self.yaw = (self.yaw + math.pi) % (2.0 * math.pi) - math.pi

        self.yaw_target = (self.yaw_target + math.pi) % (2*math.pi) - math.pi
        yaw_error = shortest_ang_err(self.yaw_target, self.yaw)
        if abs(yaw_error) < 0.05:
            yaw_error = 0.0
        self.wz = max(-4.0, min(4.0, self.kp2 * yaw_error))

    def lidar_cb(self, msg: LaserScan):
        if self.done:
            return

        # 0~30도 / 150~180도 가정 (센서 해상도 다르면 보정 필요)
        left_ranges = [r for r in msg.ranges[150:180] if r > 0.1]
        right_ranges = [r for r in msg.ranges[0:30] if r > 0.1]
        if not left_ranges or not right_ranges:
            return

        left_dist = sum(left_ranges)/len(left_ranges)
        right_dist = sum(right_ranges)/len(right_ranges)

        # 중앙 보정
        if left_dist + right_dist <= 1.2:
            self.error = left_dist - right_dist
        elif (time.time() - self.lidartime > 6) and left_dist > 1.2 and right_dist > 1.2:
            # (선택) 완주 조건을 lidar로 확인하려면 여기서 finish() 호출 가능
            # self.finish("finish detected by lidar")
            self.sc.Move(0,0,0)
            self.error = 0.0
            
            self.vx = 0.0
            self.vy=0.0
            self.wz=0.0
            self.get_logger().info("lidar finish")
            self.finish()
            return
        elif self.use_one_side_lidar and left_dist <= 0.85 and right_dist > 0.85:
            self.error =  (left_dist - 0.5) * 2 # 중앙값 기준 0.5
            self.get_logger().info(f"left_dist: {left_dist}")
        elif self.use_one_side_lidar and left_dist > 0.85 and right_dist <= 0.85:
            self.error = (0.5 - right_dist) * 2 # 중앙값 기준 0.5
            self.get_logger().info(f"right_dist: {right_dist}")

        self.vy = max(-2.5, min(2.5, self.kp * self.error))

        try:
            self.get_logger().warning(f"move command sending from lidar cb")

            self.sc.Move(float(self.vx), float(self.vy), float(self.wz))
        except Exception as e:
            self.get_logger().warning(f"Move failed: {e}")


# --------------------- stage-style run loop ---------------------

def run_stage(executor, node, timeout_s=None):
    start = time.time()
    while rclpy.ok() and not node.done:
        executor.spin_once(timeout_sec=0.05)
        if timeout_s and (time.time() - start) > timeout_s:
            node.finish("stage timeout")
            break

    # 스테이지 종료 시: 정지 & ROS 엔티티 해제
    try:
        if hasattr(node, "sc") and node.sc is not None:
            node.sc.Move(0.0, 0.0, 0.0)
    except Exception:
        pass
    try:
        executor.remove_node(node)
    except Exception:
        pass
    try:
        node.destroy_node()
    except Exception:
        pass


def main(args=None):
    rclpy.init(args=args)
    executor = SingleThreadedExecutor()
    node = None

    node1 = DetectionNode()
    executor.add_node(node1)
    # 기존 spin() 대신 — 두 번째 코드 스타일
    run_stage(executor, node1)  # 필요 시 제한시간 부여

    node2 = RunningNode(yaw_target = YAW_TARGET, use_one_side_lidar = True)
    executor.add_node(node2)
    # 기존 spin() 대신 — 두 번째 코드 스타일
    run_stage(executor, node2, timeout_s=10)  # 필요 시 제한시간 부여
    
    # node3 = RunningNode(yaw_target = YAW_TARGET, use_one_side_lidar = True, use_side_lidar = True)
    # executor.add_node(node3)
    # # 기존 spin() 대신 — 두 번째 코드 스타일
    # run_stage(executor, node3, timeout_s=2)


    try:
        executor.shutdown()
    except Exception:
        pass
    # 혹시 finish를 못 탔다면 안전 정리 1회 더
    for node in [node1, node2]:            
        if node is not None and not node.done:
            try:
                node.finish("finalize fallback")
            except Exception:
                pass
    try:
        rclpy.shutdown()
    except Exception:
        pass


if __name__ == "__main__":
    main()
