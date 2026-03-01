#!/usr/bin/env python3
# oneshot, high move command, no IMU, but pose, and lidar close automatically
# no need to send stop cmd command
# Refactored: no maybe_start, no want_start, no arm_timer

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import qos_profile_sensor_data

import math, time, os, signal, threading, queue, sys
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.sport.sport_client import SportClient
from unitree_sdk2py.go2.video.video_client import VideoClient

import numpy as np
import cv2
from ultralytics import YOLO


# ---- SDK 채널 초기화(전역 한 번) ----
try:
    ChannelFactoryInitialize(0)
except Exception as e:
    print(f'ChannelFactoryInitialize failed: {e}')
    raise

def target_yaw(z, w):
    return math.atan2(2*w*z, 1 - 2*z**2)

YAW_TARGET = target_yaw(-0.3126348728838737, 0.9449064534569476)


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
    except Exception: return
    for name in ("ChannelFactoryFinalize", "ChannelFactoryDeinitialize", "ChannelFactoryUninit",
                 "ChannelFactoryShutdown", "ChannelFactoryFini"):
        fn = getattr(_ch, name, None)
        if callable(fn):
            try: fn()
            except Exception: pass
            break


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
    return (xc < 0.2) or (xc > 0.8)


class ObstacleDecisionNode(Node):
    def __init__(self, yaw_target):
        super().__init__('obstacle_decision_node')

        self.declare_parameter("model_path", "/home/mr/ros2_ws/src/obstacle_pkg/models/best.pt")
        self.declare_parameter("auto_exit_sec", 0.0)  # 자동 종료 없음

        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.auto_exit_sec = float(self.get_parameter('auto_exit_sec').value)

        # ---- Events ----
        self.raw_q = queue.Queue(maxsize=2)
        self.stop_ev = threading.Event()
        self.started = threading.Event()
        self.done = False

        # ---- Gains ----
        self.vx = 3.7; self.vy = 0.0; self.wz = 0.0
        self.kp = 1.5; self.kp2 = 5.0
        self.error = 0.0
        self.yaw = 0.0
        self.yaw_target = float(yaw_target)

        # ---- YOLO ----
        try:
            self.model = YOLO(model_path)
            self.get_logger().info(f'YOLO loaded: {model_path} (device={self.model.device})')
        except Exception as e:
            self.get_logger().error(f'YOLO load failed: {e}')
            raise

        # ---- Unitree ----
        self.sc = SportClient(enableLease=False)
        self.sc.SetTimeout(10.0)
        self.sc.Init()
        try: self.sc.BalanceStand()
        except Exception: pass

        # ---- Subs ----
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.lidar_cb, qos_profile_sensor_data)
        self.sub_pose = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.yaw_callback, 3)

        # ---- Timer ----
        self.hb_timer = self.create_timer(4.0, self._heartbeat)

        # ---- Threads ----
        self.t_vid = threading.Thread(target=self.video_loop, daemon=True)
        self.t_det = threading.Thread(target=self.yolo_worker, daemon=True)
        self.t_vid.start(); time.sleep(0.05)
        self.t_det.start()

        # ---- Signals ----
        signal.signal(signal.SIGINT, self._sig_handler)
        signal.signal(signal.SIGTERM, self._sig_handler)

    def finish(self, reason=""):
        if self.done: return
        self.get_logger().info(f"[FINISH] {reason}")
        self.done = True
        self.stop_ev.set()
        try: self.sc.Move(0,0,0)
        except Exception: pass
        self._cleanup()

    def _sig_handler(self, signum, frame):
        self.finish(f"signal {signum}")

    def yaw_callback(self, msg):
        if not self.started.is_set() or self.done: return
        q = msg.pose.pose.orientation
        self.yaw = float(target_yaw(q.z, q.w))
        self.yaw = (self.yaw + math.pi) % (2.0*math.pi) - math.pi
        yaw_error = self.yaw_target - self.yaw
        if abs(yaw_error) < 0.05: yaw_error = 0.0
        self.wz = max(-4.0, min(4.0, self.kp2*yaw_error))

    def lidar_cb(self, msg: LaserScan):
        if not self.started.is_set() or self.done: return
        left = [r for r in msg.ranges[150:180] if r > 0.1]
        right = [r for r in msg.ranges[0:30] if r > 0.1]
        if not left or not right: return
        left_dist = sum(left)/len(left); right_dist = sum(right)/len(right)

        if left_dist + right_dist <= 1.2:
            self.error = left_dist - right_dist
        self.vy = self.kp * self.error
        if abs(self.vy) > 3.7:
            self.vy = math.copysign(3.7, self.vy)

        try: self.sc.Move(float(self.vx), float(self.vy), float(self.wz))
        except Exception as e: self.get_logger().warn(f"Move failed: {e}")

    def yolo_worker(self):
        while not self.stop_ev.is_set():
            try:
                frame = self.raw_q.get(timeout=0.1)
                h, w = frame.shape[:2]
            except queue.Empty: continue
            except Exception: continue

            try:
                res = self.model.predict(source=frame, conf=0.1, verbose=False)[0]
                boxes = res.boxes; chosen = None
                if boxes is not None and len(boxes) > 0:
                    xyxy = boxes.xyxy.cpu().numpy()
                    confs = boxes.conf.cpu().numpy()
                    for (x1,y1,x2,y2),p in zip(xyxy,confs):
                        x1,y1,x2,y2 = map(int,[x1,y1,x2,y2])
                        if x2<=x1 or y2<=y1: continue
                        xc = ((x1+x2)/2)/w; yc = ((y1+y2)/2)/h
                        cand = {"xc":xc,"yc":yc,"conf":float(p)}
                        if chosen is None or cand["conf"]>chosen["conf"]:
                            chosen = cand
                if chosen is not None and not self.started.is_set():
                    safe = is_safe_spot(chosen["xc"], chosen["yc"])
                    if safe:
                        self.get_logger().info(f"Safe spot! xc={chosen['xc']:.3f}")
                        self.started.set()
                        try: self.sc.Move(float(self.vx), float(self.vy), float(self.wz))
                        except Exception: pass
            except Exception as e:
                self.get_logger().warning(f"[YOLO] error: {e}")
                continue

    def video_loop(self):
        client = VideoClient(); client.SetTimeout(3.0); client.Init()
        while not self.stop_ev.is_set():
            code,data = client.GetImageSample()
            if code!=0: self.finish("video error"); break
            try:
                frame = cv2.imdecode(np.frombuffer(bytes(data),dtype=np.uint8), cv2.IMREAD_COLOR)
                if frame is None: continue
                if not self.raw_q.empty():
                    try: self.raw_q.get_nowait()
                    except queue.Empty: pass
                self.raw_q.put_nowait(frame)
            except Exception: continue

    def _heartbeat(self):
        self.get_logger().info(f"raw_q={self.raw_q.qsize()}, started={self.started.is_set()}, done={self.done}")

    def _cleanup(self):
        try: self.sc.StopMove()
        except Exception: pass
        try: self.sub_scan; self.destroy_subscription(self.sub_scan)
        except Exception: pass
        try: self.sub_pose; self.destroy_subscription(self.sub_pose)
        except Exception: pass
        try: self.hb_timer; self.destroy_timer(self.hb_timer)
        except Exception: pass
        _safe_client_close(self.sc)
        _safe_sdk_finalize()

    def destroy_node(self):
        self._cleanup()
        super().destroy_node()


def run_stage(executor, node, timeout_s=None):
    start=time.time()
    while rclpy.ok() and not node.done:
        executor.spin_once(timeout_sec=0.05)
        if timeout_s and (time.time()-start)>timeout_s:
            node.finish("timeout")
            break
    try: node.sc.Move(0,0,0)
    except Exception: pass
    try: executor.remove_node(node)
    except Exception: pass
    try: node.destroy_node()
    except Exception: pass


def main(args=None):
    rclpy.init(args=args)
    executor=SingleThreadedExecutor()
    node=ObstacleDecisionNode(yaw_target=YAW_TARGET)
    executor.add_node(node)
    try:
        run_stage(executor,node,timeout_s=5.0)
    finally:
        try: executor.shutdown()
        except Exception: pass
        if node and not node.done:
            try: node.finish("finalize fallback")
            except Exception: pass
        try: rclpy.shutdown()
        except Exception: pass


if __name__=="__main__":
    main()
