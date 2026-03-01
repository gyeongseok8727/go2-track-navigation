#!/usr/bin/env python3

#oneshot, no need to stop cmd pub
#no IMU, but pose
#use lidar cb also for Move command


import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import LaserScan

#from unitree_go.msg import LowState
from geometry_msgs.msg import PoseWithCovarianceStamped

from rclpy.qos import qos_profile_sensor_data

import math, time, os, signal, threading, queue, sys
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.video.video_client import VideoClient
from unitree_sdk2py.go2.sport.sport_client import SportClient


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


# target_yaw atan2( 2wz, 1 - 2*z² )
def target_yaw(z, w):
    return math.atan2(2*w*z, 1 - 2*z**2)

YAW_TARGET = target_yaw(-0.957516130819927, 0.28837971360627346)


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
    def __init__(self, yaw_target):
        super().__init__("crossing_node")


        # ---- Params----
        pkg_share = get_package_share_directory("traffic_pkg")
        default_model_path = os.path.join(pkg_share, "models", "besttraffic.pt")
        default_model_path = "/home/mr/ros2_ws/src/traffic_pkg/models/besttraffic.pt"

        self.declare_parameter("model_path", default_model_path)
        self.declare_parameter("unitree_arg", "eno1")           # optional ip/token if needed by ChannelFactoryInitialize
        self.declare_parameter('auto_exit_sec', 3.0)


        model_path = self.get_parameter("model_path").get_parameter_value().string_value
        self.unitree_arg = self.get_parameter("unitree_arg").get_parameter_value().string_value
        self.auto_exit_sec = float(self.get_parameter('auto_exit_sec').value)

        
        # ---- Queues & Event ----
        self.raw_q = queue.Queue(maxsize=2)
        self.stop_ev = threading.Event()
        
        #different with obstacle
        self.sig_q = queue.Queue(maxsize=5)

        self.started = threading.Event()
        self.want_start = False              # 워커→메인 스레드 신호
        self.stopping = False


        # ---- State / Ctrl gains ----
        self.vx = 3.7
        self.vy = 0.0
        self.wz = 0.0

        self.kp = 1.5      # Lidar → vy
        self.kp2 = 5.0     # IMU → wz
        self.error = 0.0
        self.yaw = 0.0
        self.yaw_target = float(yaw_target)

        # ---- YOLO ----
        try:
            self.model = YOLO(model_path)
            self.get_logger().info(f'YOLO loaded: {model_path} (device={self.model.device})')
        except Exception as e:
            self.get_logger().error(f"YOLO load failed: {e}")
            raise


        # -----unitree sport client----------
        self.sc = SportClient(enableLease=False)
        self.sc.SetTimeout(10.0)
        self.sc.Init()
        self.get_logger().info("Initializing Unitree SportClient...")
        try:
            self.sc.BalanceStand()
        except Exception as e:
            self.get_logger().warn(f'BalanceStand failed: {e}')


        # ---- Subscriptions (미리 생성, 콜백 가드로 제어) ----
        self.sub_scan = self.create_subscription(
            LaserScan, '/scan', self.lidar_cb, qos_profile_sensor_data
        )
        self.sub_lowstate = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.yaw_callback, 3
        )



        self._exit_timer = None
        self.arm_timer = self.create_timer(0.02, self._maybe_start)  # 20ms 폴링
        # Optional timer to log health
        self.hb_timer = self.create_timer(4.0, self._heartbeat)



        # ---- Threads ----
        self.t_vid = threading.Thread(target=self.video_loop, daemon=True)
        self.t_det = threading.Thread(target=self.yolo_worker, daemon=True)
        self.t_dec = threading.Thread(target=self.decision_worker, daemon=True)

        self.t_vid.start()
        time.sleep(0.05)
        self.t_det.start()
        self.t_dec.start()


        # Signal handler
        signal.signal(signal.SIGINT, self._sig_handler)
        signal.signal(signal.SIGTERM, self._sig_handler)

    def _maybe_start(self):
        """메인 스레드: 워커가 세운 want_start를 감지해 즉시 시작/타이머 생성."""
        if self.stopping or self.started.is_set() or not self.want_start:
            return
        self.want_start = False
        self.started.set()
        self.get_logger().info('START triggered by YOLO (oneshot).')

        if self._exit_timer is None and self.auto_exit_sec > 0:
            self._exit_timer = self.create_timer(self.auto_exit_sec, self._on_exit_timer)


    def _on_exit_timer(self):
        self.get_logger().info('Auto-exit timer triggered. Stopping and shutting down...')
        self._cleanup()
        # 노드 파괴는 main()에서 일괄 처리
        try:
            rclpy.shutdown()
        except Exception:
            pass

    def yaw_callback(self, msg):
        if not self.started.is_set() or self.stopping:
            return
        #if msg is None:

        q = msg.pose.pose.orientation
        self.yaw = float(target_yaw(q.z, q.w))
        self.yaw = (self.yaw + math.pi) % (2.0 * math.pi) - math.pi
        yaw_error = self.yaw_target - self.yaw
        if abs(yaw_error) < 0.05:
            yaw_error = 0.0
        self.wz = max(-4.0, min(4.0, self.kp2 * yaw_error))

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
        # if left_dist <= 0.85 and right_dist <= 0.85:
        if left_dist + right_dist <= 1.2:
            self.error = left_dist - right_dist
        self.vy = self.kp * self.error

        # 속도 포화(임계=포화 3.7 통일)
        if abs(self.vy) > 3.7:
            self.vy = math.copysign(3.7, self.vy)

        try:
            self.sc.Move(float(self.vx), float(self.vy), float(self.wz))
        except Exception as e:
            self.get_logger().warn(f"Move failed: {e}")

    def _destroy_timers_and_subs(self):
        # timers
        try:
            if self._exit_timer:
                self._exit_timer.cancel()
                self.destroy_timer(self._exit_timer)
                self._exit_timer = None
        except Exception:
            pass
        try:
            if self.arm_timer:
                self.arm_timer.cancel()
                self.destroy_timer(self.arm_timer)
                self.arm_timer = None
        except Exception:
            pass
        try:
            if hasattr(self, "hb_timer") and self.hb_timer:
                self.hb_timer.cancel()
                self.destroy_timer(self.hb_timer)
                self.hb_timer = None
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
        # 중복 안전
        self._cleanup()
        super().destroy_node()



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
                #print(boxes)

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

                        #(Optional) annotate for debug
                        cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        cv2.putText(annotated, f"{p:.2f}|{color}", (x1, max(0, y1 - 6)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2, cv2.LINE_AA)
                        

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
                    #print("sending sig!!")
                    self.sig_q.put_nowait(sig)


            except Exception as e:
                self.get_logger().warning(f"[YOLO] error: {e}")
                continue

    

    def decision_worker(self):
        K_G, K_R = 3, 2
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
                g_cnt = r_cnt = 0
            last_bbox = bbox

            if conf < C_MIN:
                continue

            # 카운트 갱신
            if color == "g":
                g_cnt += 1; r_cnt = 0
            elif color == "r":
                r_cnt += 1; g_cnt = 0
            else:
                # unknown/y/ry → 카운트 유지(증가/리셋 없음)
                pass

            # 상태 전이
            if state == "INIT":
                # 최초 유효 1회로 즉시 초기 상태 확정
                if color == "g":
                    state = "INIT_G"; g_cnt = 1; r_cnt = 0
                elif color == "r":
                    state = "INIT_R"; r_cnt = 1; g_cnt = 0

            elif state == "INIT_G":
                # 초록으로 시작 → 빨강 연속 K_R 나오면 R 위상 확인됨
                if r_cnt >= K_R:
                    state = "WAIT_G"; g_cnt = 0   # 이제 초록 3회 기다리면 GO
            elif state == "INIT_R":
                # 빨강으로 시작 → 바로 초록 3회면 GO
                if g_cnt >= K_G:
                    #self.publish_cmd(0)
                    self.sc.TrotRun()
                    self.want_start = True
                    try:
                        self.sc.Move(float(self.vx), float(self.vy), float(self.wz))
                    except Exception:
                        pass
                            
                    self.get_logger().info("GO (Gx3 after INIT_R)")
                    state = "CROSSING"

            elif state == "WAIT_R":
                # (쓰지 않아도 됨: INIT_G에서 곧장 r_cnt로 WAIT_G 전이하므로)
                pass

            elif state == "WAIT_G":
                if g_cnt >= K_G:

                    self.sc.TrotRun()
                    self.want_start = True
                    try:
                        self.sc.Move(float(self.vx), float(self.vy), float(self.wz))
                    except Exception:
                        pass
                    self.get_logger().info("GO (Gx3 after R-phase)")
                    state = "CROSSING"

            elif state == "CROSSING":
                # 이동 중엔 비전 무시(중도 정지 없음)
                pass

            print('state: ', state, 'g_cnt: ', g_cnt, 'r_cnt: ', r_cnt)


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
        self.get_logger().info(
            f"q(raw={self.raw_q.qsize()}, sig={self.sig_q.qsize()})"
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
    node = CrossingNode(yaw_target=YAW_TARGET)
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

if __name__ == "__main__":
    main()

