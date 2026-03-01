#!/usr/bin/env python3
import signal
import time
import threading
import queue
import os
import sys

import numpy as np
import cv2
from ultralytics import YOLO

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.video.video_client import VideoClient

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from ament_index_python.packages import get_package_share_directory

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
    # 화면 중앙/하단에 장애물이 없으면 안전하다고 보는 간단 규칙
    return (xc < 0.2) or (xc > 0.8)

# ------------------------- ROS 2 Node -------------------------

class ObstacleDecisionNode(Node):
    def __init__(self):
        super().__init__('obstacle_decision_node')

        # ---- Parameters ----
        self.declare_parameter(
            "model_path",
            os.path.join(
                get_package_share_directory("obstacle_pkg"),
                "models",
                "best.pt"
            )
        )
        self.declare_parameter('unitree_arg', 'eno1')
        self.declare_parameter('cmd_topic', 'stop_cmd')  # Int32: 0=GO, 1=STOP

        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        unitree_arg = self.get_parameter('unitree_arg').get_parameter_value().string_value
        self.cmd_topic = self.get_parameter('cmd_topic').get_parameter_value().string_value

        # ---- Queues & Event ----
        self.raw_q = queue.Queue(maxsize=2)
        self.stop_ev = threading.Event()
        self.started = threading.Event()                   # 원샷 GO 발행 여부

        # ---- Publisher ----
        self.cmd_pub = self.create_publisher(Int32, self.cmd_topic, 10)

        # ---- YOLO ----
        try:
            self.model = YOLO(model_path)
            self.get_logger().info(f'YOLO loaded: {model_path} (device={self.model.device})')
        except Exception as e:
            self.get_logger().error(f'YOLO load failed: {e}')
            raise

        # ---- Unitree Channel init ----
        try:
            if unitree_arg:
                ChannelFactoryInitialize(0, unitree_arg)
            else:
                ChannelFactoryInitialize(0)
        except Exception as e:
            self.get_logger().error(f'ChannelFactoryInitialize failed: {e}')
            raise

        # ---- Threads ----
        self.t_vid = threading.Thread(target=self.video_loop, daemon=False)
        self.t_det = threading.Thread(target=self.yolo_worker, daemon=True)

        self.t_vid.start(); time.sleep(0.2)
        self.t_det.start()

        # Heartbeat (디버깅용)
        self.create_timer(2.0, self._heartbeat)

        # Signal handlers
        signal.signal(signal.SIGINT, self._sig_handler)
        signal.signal(signal.SIGTERM, self._sig_handler)

    # --------------------- workers ---------------------

    def _sig_handler(self, signum, frame):
        self.get_logger().info(f'Signal {signum} received → stopping...')
        self.stop_ev.set()
        def _later_shutdown():
            try:
                rclpy.shutdown()
            except Exception:
                pass
        threading.Thread(target=_later_shutdown, daemon=True).start()

    def publish_cmd(self, code: int):
        # code: 0 = GO (CROSS_START), no need to use stop for this code
        self.cmd_pub.publish(Int32(data=int(code)))

    def yolo_worker(self):
        while not self.stop_ev.is_set():
            try:
                frame = self.raw_q.get(timeout=0.1)
                h, w = frame.shape[:2]
            except queue.Empty:
                continue
            except Exception:
                continue

            try:
                # 한 프레임 추론
                res = self.model.predict(source=frame, conf=0.1, verbose=False)[0]

                chosen = None
                boxes = res.boxes

                if boxes is not None and len(boxes) > 0:
                    xyxy = boxes.xyxy.cpu().numpy()
                    confs = boxes.conf.cpu().numpy()

                    for (x1, y1, x2, y2), p in zip(xyxy, confs):
                        x1, y1, x2, y2 = map(int, [x1, y1, x2, y2])
                        if x2 <= x1 or y2 <= y1:
                            continue

                        xc = ((x1 + x2) / 2) / w   # normalized center x
                        yc = ((y1 + y2) / 2) / h   # normalized center y
                        cand = {"xc": xc, "yc": yc, "conf": float(p), "bbox": (x1, y1, x2, y2)}
                        if chosen is None or cand["conf"] > chosen["conf"]:
                            chosen = cand
                else:
                    print("no box detected")

                # 원샷 트리거: "안전 지점"이 처음 감지되면 GO 한 번만 발행
                if chosen is not None and not self.started.is_set():
                    
                    #for debug
                    print("is safe: ", is_safe_spot(chosen["xc"], chosen["yc"]), "xc: ", chosen["xc"], "yc: ", chosen["yc"])
                    if is_safe_spot(chosen["xc"], chosen["yc"]):
                        
                        #comment down two lines to debug
                        self.publish_cmd(0)  # GO
                        self.started.set()
                        self.get_logger().info(
                            f'GO published (one-shot by first SAFE spot: xc={chosen["xc"]:.3f}, yc={chosen["yc"]:.3f})'
                        )
                elif chosen is not None and self.started.is_set():
                    print("finished")


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

    def _heartbeat(self):
        # 간단 상태 로그(디버깅용) - sig_q 제거 반영
        self.get_logger().debug(f'q(raw={self.raw_q.qsize()})')

    # --------------------- shutdown ---------------------

    def stop(self):
        self.stop_ev.set()
        try:
            if self.t_vid.is_alive():
                self.t_vid.join(timeout=2.0)
        except Exception:
            pass
        # 나머지 데몬 스레드는 stop_ev로 종료

def main():
    rclpy.init()
    node = None
    try:
        node = ObstacleDecisionNode()
        rclpy.spin(node)
    except Exception as e:
        if node:
            node.get_logger().error(f'Exception in node: {e}')
    finally:
        if node:
            node.stop()
            node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()
