#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
import numpy as np, cv2
from sensor_msgs.msg import CompressedImage

WIN_NAME = 'Live Stream'

class ViewerNode(Node):
    def __init__(self):
        super().__init__('viewer_node')
        cv2.namedWindow(WIN_NAME)
        self.sub = self.create_subscription(
            CompressedImage, '/camera/image_raw/compressed', self.on_image, 10
        )
        self.key_timer = self.create_timer(0.01, self.poll_key)
        self.get_logger().info('Viewer started (/camera/image_raw/compressed)')

    def on_image(self, msg: CompressedImage):
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warn('Failed to decode JPEG frame')
            return
        cv2.imshow(WIN_NAME, frame)

    def poll_key(self):
        k = cv2.waitKey(1)
        if k == 27:  # ESC
            self.get_logger().info('ESC pressed. Closing...')
            cv2.destroyAllWindows()
            self.destroy_node()
            # 타이머 콜백에서 shutdown 요청
            rclpy.shutdown()

def main():
    rclpy.init()
    node = ViewerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # 안전하게 한 번 더 정리 (중복 shutdown 방지)
        cv2.destroyAllWindows()
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():  # 아직 살아있으면만 종료
            rclpy.shutdown()

if __name__ == '__main__':
    main()
