#!/usr/bin/env python3
import os, sys

# 1) CUDA 강제 비활성화 (PyTorch가 아예 CUDA를 보지 못하게)
os.environ["CUDA_VISIBLE_DEVICES"] = "-1"     # 빈 문자열 "" 대신 "-1"이 더 확실하게 차단됩니다.
os.environ["NVIDIA_VISIBLE_DEVICES"] = "none" # 도커/컨테이너 환경 대비

# 2) 시끄러운 로그 끄기
os.environ["TORCH_CPP_LOG_LEVEL"] = "ERROR"   # PyTorch C++ 로그
os.environ["LOGURU_LEVEL"] = "ERROR"          # Ultralytics(Loguru)
os.environ["OV_LOG_LEVEL"] = "ERROR"          # OpenVINO
os.environ["RCUTILS_LOGGING_SEVERITY"] = "ERROR"  # ROS2 전역 로그 레벨

# (선택) loguru 미설치 환경에서도 안전하게
try:
    from loguru import logger
    logger.remove()
    logger.add(sys.stderr, level="ERROR")
except Exception:
    pass

# (선택) OpenCV 스레드/로그 제한
import cv2
cv2.setNumThreads(1)
try:
    cv2.utils.logging.setLogLevel(cv2.utils.logging.LOG_LEVEL_ERROR)
except Exception:
    pass
# ---------------------------------------

import signal
import time
import threading
import queue
import collections
import numpy as np

cv2.setNumThreads(1)





from ultralytics import YOLO
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.video.video_client import VideoClient
# from unitree_sdk2py.go2.sport.sport_client import SportClient  # (not used here)

try:
    from loguru import logger
    logger.remove()                        # 기존 핸들러 제거
    logger.add(sys.stderr, level="ERROR")  # ERROR 이상만
except Exception:
    pass




import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

from ament_index_python.packages import get_package_share_directory
import os

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

    # 삼등분 + 완충
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

class CrossingNode(Node):
    def __init__(self):
        super().__init__("crossing_node")

        # ---- Params ----
        pkg_share = get_package_share_directory("traffic_pkg")
        default_model_path = os.path.join(pkg_share, "models", "best_openvino_model")

        

        self.declare_parameter("model_path", default_model_path)
        self.get_logger().info(f"Loading OV model from: {default_model_path}")
        self.declare_parameter("unitree_arg", "eno1")           # optional ip/token if needed by ChannelFactoryInitialize
        self.declare_parameter("cmd_topic", "/go2/sport_cmd") # Int32 command topic (3=GO, 9=STOP)
        self.declare_parameter("cmd_topic_low", "/go2/low_level_cmd") # Int32 command topic (1=GO, 0=STOP)
        
        model_path = self.get_parameter("model_path").get_parameter_value().string_value
        unitree_arg = self.get_parameter("unitree_arg").get_parameter_value().string_value
        self.cmd_topic = self.get_parameter("cmd_topic").get_parameter_value().string_value
        self.cmd_topic_low = self.get_parameter("cmd_topic_low").get_parameter_value().string_value
        

        # ---- Queues & Event ----
        self.raw_q = queue.Queue(maxsize=2)
        self.vis_q = queue.Queue(maxsize=2)
        self.sig_q = queue.Queue(maxsize=5)
        self.stop_ev = threading.Event()

        # ---- Publisher ----
        self.cmd_pub = self.create_publisher(Int32, self.cmd_topic, 10)
        self.cmd_low_pub = self.create_publisher(Int32, self.cmd_topic_low, 10)
        

        # ---- YOLO ----
        try:
            self.model = YOLO(model_path, task="detect")
            self.model.overrides["device"] = "cpu"   # 기본값 고정
            self.get_logger().info(f'YOLO loaded: {model_path} (device={self.model.device})')
        except Exception as e:
            self.get_logger().error(f"YOLO load failed: {e}")
            raise

        # ---- Unitree Channel init ----
        try:
            if unitree_arg:
                ChannelFactoryInitialize(0, unitree_arg)
            else:
                ChannelFactoryInitialize(0)
        except Exception as e:
            self.get_logger().error(f"ChannelFactoryInitialize failed: {e}")
            raise

        # ---- Threads ----
        self.t_vid = threading.Thread(target=self.video_loop, daemon=False)
        self.t_det = threading.Thread(target=self.yolo_worker, daemon=True)
        self.t_dec = threading.Thread(target=self.decision_worker, daemon=True)

        self.t_vid.start()
        time.sleep(0.3)
        self.t_det.start()
        self.t_dec.start()

        # Optional timer to log health
        self.create_timer(2.0, self._heartbeat)

        # Signal handler
        signal.signal(signal.SIGINT, self._sig_handler)
        signal.signal(signal.SIGTERM, self._sig_handler)

    # --------------------- workers ---------------------

    def _sig_handler(self, signum, frame):
        self.get_logger().info(f"Signal {signum} received → stopping...")
        self.stop_ev.set()
        # request rclpy shutdown from timer-safe thread
        def _later_shutdown():
            try:
                rclpy.shutdown()
            except Exception:
                pass
        threading.Thread(target=_later_shutdown, daemon=True).start()

    def publish_cmd(self, code: int):
        msg = Int32(data=int(code))
        self.cmd_pub.publish(msg)
        
        low = 1 if code == 3 else 0
        self.cmd_low_pub.publish(Int32(data=low))

    def yolo_worker(self):
        while not self.stop_ev.is_set():
            try:
                frame = self.raw_q.get(timeout=0.1)
            except queue.Empty:
                continue

            try:
                res = self.model.predict(source=frame, conf=0.65, device='cpu', imgsz=640, verbose=False, task="detect")[0]

                chosen = None
                boxes = res.boxes

                annotated = frame.copy()  # if you want, copy: frame.copy()
                if boxes is not None and len(boxes) > 0:
                    xyxy = boxes.xyxy.cpu().numpy()
                    confs = boxes.conf.cpu().numpy()

                    for (x1, y1, x2, y2), p in zip(xyxy, confs):
                        x1, y1, x2, y2 = map(int, [x1, y1, x2, y2])
                        if x2 <= x1 or y2 <= y1:
                            continue

                        roi = frame[max(0, y1):max(0, y2), max(0, x1):max(0, x2)]
                        color = detect_color(roi)["state"]

                        # (Optional) annotate for debug
                        # cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        # cv2.putText(annotated, f"{p:.2f}|{color}", (x1, max(0, y1 - 6)),
                        #             cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2, cv2.LINE_AA)

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

                # (Optional) visualize queue, don't need for now, but just leave it here for debug
                # if not self.vis_q.empty():
                #     try:
                #         self.vis_q.get_nowait()
                #     except queue.Empty:
                #         pass
                # self.vis_q.put_nowait(annotated)

            except Exception as e:
                self.get_logger().warning(f"[YOLO] error: {e}")
                continue

    def decision_worker(self):


        
        N = 10                # sliding window size
        K_GREEN = 7           # need >=K green in last N
        HOLD_GREEN_S = 0.8    # min dwell before go
        DROP_OUT_MS = 400     # allowed green loss during crossing
        MIN_IOU = 0.5         # same lamp consistency

        win = collections.deque(maxlen=N)
        state = "IDLE"
        last_bbox = None
        green_since = None
        last_green_ts = None

        while not self.stop_ev.is_set():
            try:
                color, conf, bbox, ts = self.sig_q.get(timeout=0.1)
            except queue.Empty:
                #should add logics here!!
                #
                #
                #
                #
                continue

            if last_bbox is not None and iou_xyxy(last_bbox, bbox) < MIN_IOU:
                win.clear()
                green_since = None

            last_bbox = bbox
            win.append((color, ts))

            greens = sum(1 for c, _ in win if c == "g")
            # reds   = sum(1 for c, _ in win if c == "r")

            if state in ("IDLE", "WAIT_GREEN"):
                state = "WAIT_GREEN"
                if color == "g":
                    if green_since is None:
                        green_since = ts
                    last_green_ts = ts
                else:
                    green_since = None

                if greens >= K_GREEN and green_since and (ts - green_since) >= HOLD_GREEN_S:
                    # Go!
                    self.publish_cmd(3)  # (3=GO)
                    state = "CROSSING"
                    last_green_ts = ts
                    self.get_logger().info("CROSSING: GO command sent")

            elif state == "CROSSING":
                if color == "g":
                    last_green_ts = ts

                if last_green_ts and (ts - last_green_ts) * 1000.0 > DROP_OUT_MS:
                    self.publish_cmd(9)  # (9=STOP/ABORT)
                    state = "ABORT"
                    self.get_logger().warn("ABORT: STOP command sent (green lost)")

            elif state == "ABORT":
                state = "WAIT_GREEN"
                win.clear()
                green_since = None
                last_bbox = None

    def video_loop(self):
        client = VideoClient()
        client.SetTimeout(3.0)
        client.Init()

        while not self.stop_ev.is_set():
            code, data = client.GetImageSample()
            if code != 0:
                self.get_logger().error(f"VideoClient error code: {code}")
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

    def _heartbeat(self):
        # Simple health log (optional)
        self.get_logger().debug(
            f"q(raw={self.raw_q.qsize()}, vis={self.vis_q.qsize()}, sig={self.sig_q.qsize()})"
        )

    # --------------------- shutdown ---------------------

    def stop(self):
        self.stop_ev.set()
        try:
            if self.t_vid.is_alive():
                self.t_vid.join(timeout=2.0)
        except Exception:
            pass
        # Detectors/decision are daemon threads; they’ll exit when stop_ev is set


def main():
    rclpy.init()
    node = None
    try:
        node = CrossingNode()
        rclpy.spin(node)
    except Exception as e:
        if node:
            node.get_logger().error(f"Exception in node: {e}")
    finally:
        if node:
            node.stop()
            node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
