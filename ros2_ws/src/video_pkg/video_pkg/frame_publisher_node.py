#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import cv2

from sensor_msgs.msg import CompressedImage
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.video.video_client import VideoClient


class FramePublisher(Node):
    def __init__(self):
        super().__init__('frame_publisher')
        self.pub = self.create_publisher(CompressedImage, '/camera/image_raw/compressed', 10)

        fps = float(self.declare_parameter('fps', 30.0).value)
        nic = str(self.declare_parameter('unitree_nic', 'eno1').value)
        quality = int(self.declare_parameter('jpeg_quality', 80).value)  # 1~100 (higher = larger)

        try:
            ChannelFactoryInitialize(0, nic)
            self.get_logger().info(f'Channel init OK (0)')
        except Exception as e:
        
            self.get_logger().error(f'Channel init failed: {e}')
            raise

        self.vc = VideoClient(); self.vc.Init()
        self.enc_param = [int(cv2.IMWRITE_JPEG_QUALITY), max(1, min(100, quality))]
        self.timer = self.create_timer(1.0 / fps, self.tick)

    def tick(self):
        code, data = self.vc.GetImageSample()
        if code != 0:
            self.get_logger().error(f'VideoClient error: {code}')
            return

        buf = np.frombuffer(bytes(data), dtype=np.uint8)
        frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warn('Empty frame'); return

        ok, enc = cv2.imencode('.jpg', frame, self.enc_param)
        if not ok:
            self.get_logger().error('JPEG encode failed'); return

        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = 'jpeg'
        msg.data = enc.tobytes()
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = FramePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
