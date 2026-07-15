"""Publish webcam frames as ROS Image messages for perception smoke tests."""

import sys

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class WebcamImagePublisherNode(Node):
    """Small camera source node for feeding yolo_detector_node's /image_raw."""

    def __init__(self):
        super().__init__('webcam_image_publisher_node')

        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('camera_device', '')
        self.declare_parameter('frame_id', 'webcam')
        self.declare_parameter('fps', 15.0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)

        self.image_topic = str(self.get_parameter('image_topic').value)
        self.camera_index = int(self.get_parameter('camera_index').value)
        self.camera_device = str(self.get_parameter('camera_device').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.fps = max(float(self.get_parameter('fps').value), 1.0)
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)

        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, self.image_topic, 10)
        self.frame_count = 0
        self.failed_read_count = 0

        self.cap = self._open_capture()
        timer_period = 1.0 / self.fps
        self.timer = self.create_timer(timer_period, self._publish_frame)

        source = self.camera_device if self.camera_device else self.camera_index
        self.get_logger().info(
            f'웹캠 이미지 발행 시작: source={source}, '
            f'{self.width}x{self.height}@{self.fps:.1f}Hz -> {self.image_topic}')

    def _open_capture(self):
        source = self.camera_device if self.camera_device else self.camera_index
        cap = cv2.VideoCapture(source, cv2.CAP_V4L2)

        if self.width > 0:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        if self.height > 0:
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, self.fps)

        if not cap.isOpened():
            raise RuntimeError(
                f'웹캠을 열 수 없습니다: source={source}. '
                '/dev/video* 노출 여부와 다른 프로그램 점유 여부를 확인하세요.')

        return cap

    def _publish_frame(self):
        ok, frame = self.cap.read()
        if not ok or frame is None:
            self.failed_read_count += 1
            if self.failed_read_count == 1 or self.failed_read_count % 30 == 0:
                self.get_logger().warning(
                    f'웹캠 프레임을 읽지 못했습니다. 실패 횟수={self.failed_read_count}')
            return

        self.failed_read_count = 0
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        self.publisher.publish(msg)

        self.frame_count += 1
        if self.frame_count == 1 or self.frame_count % int(self.fps * 10) == 0:
            height, width = frame.shape[:2]
            self.get_logger().info(
                f'프레임 발행 중: count={self.frame_count}, size={width}x{height}')

    def destroy_node(self):
        if hasattr(self, 'cap') and self.cap is not None:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = WebcamImagePublisherNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as exc:
        print(f'[webcam_image_publisher_node] {exc}', file=sys.stderr)
        raise SystemExit(1) from exc
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
