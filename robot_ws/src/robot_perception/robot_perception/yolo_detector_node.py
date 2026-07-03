"""YOLO11 detector node.

원본 vision_ros2_node.py(팀원 작성)의 로직을 그대로 이식했습니다:
  - YOLO11(Ultralytics) 추론
  - MediaPipe Pose로 양손 끝(검지)/입 중앙 픽셀 좌표 추출
  - risk_db.py 기반 위험도(level)/우선순위(dynamic_priority) 판단 및
    입-손 근접(삼킴 위험) 경고 로직
  - cv2 시각화(bbox, 우선순위 텍스트, 위험 시 입-물체 연결선) + vision/detected_objects
    JSON(String) 발행 (원본과 동일 토픽명, 호환용으로 유지)

여기에 더해, 이 노드는 SLAM+Nav2 경로생성 파이프라인이 쓸 수 있도록 관심 클래스의
픽셀 bbox를 robot_perception_msgs/DetectionArray로 /detections에도 함께 발행합니다.
좌표계 변환(픽셀 -> 로봇/지면 좌표)은 이 노드에서 하지 않고 ground_projection_node가
전담합니다.

추가 위험도 로직:
  - battery/coin/lego는 risk_db.py의 기존 위험도 그대로 처리합니다.
  - risk_db.py에 없는 클래스라도 bbox가 battery/coin/lego 기준 bbox보다 작으면
    unknown_small_object로 간주해 Level 2 위험도로 처리합니다.

주의: 이 로봇의 웹캠은 로봇 최상단에서 지면을 향해 아래로 틸트되어 장착됩니다
(robot_slam/README.md, config/camera_extrinsics.yaml 참고). 원본 코드는 사람 얼굴이
정면으로 보이는 카메라 각도를 전제로 하므로, 실제 장착각에서 포즈/입-손 랜드마크가
안정적으로 검출되는지는 실측 후 확인이 필요합니다.
"""
import json
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

from robot_perception_msgs.msg import Detection, DetectionArray
from robot_perception.risk_db import RISK_DATABASE

try:
    from ultralytics import YOLO
except ImportError:  # pragma: no cover - exercised only on machines without ultralytics installed
    YOLO = None

try:
    import mediapipe as mp
except ImportError:  # pragma: no cover - pose/mouth-hand scoring is optional
    mp = None

try:
    import cv2
except ImportError:  # pragma: no cover - visualization / color conversion is optional
    cv2 = None


class YoloDetectorNode(Node):

    def __init__(self):
        super().__init__('yolo_detector_node')

        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('detections_topic', '/detections')
        self.declare_parameter('legacy_json_topic', 'vision/detected_objects')
        self.declare_parameter('model_path', 'best.pt')
        self.declare_parameter('target_classes', ['battery', 'coin', 'lego'])
        self.declare_parameter('confidence_threshold', 0.3)
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('inference_rate_hz', 15.0)
        self.declare_parameter('mouth_threshold_px', 120)
        self.declare_parameter('enable_pose_risk_scoring', True)
        self.declare_parameter('enable_visualization', True)
        self.declare_parameter('enable_unknown_small_object_risk', True)
        self.declare_parameter('unknown_small_object_class_name', 'unknown_small_object')
        self.declare_parameter('small_object_reference_classes', ['battery', 'coin', 'lego'])
        self.declare_parameter('small_object_reference_area_scale', 1.0)
        self.declare_parameter('small_object_fallback_max_area_ratio', 0.035)

        image_topic = self.get_parameter('image_topic').value
        detections_topic = self.get_parameter('detections_topic').value
        legacy_json_topic = self.get_parameter('legacy_json_topic').value
        model_path = self.get_parameter('model_path').value
        self.target_classes = {c.lower() for c in self.get_parameter('target_classes').value}
        self.confidence_threshold = float(self.get_parameter('confidence_threshold').value)
        self.device = self.get_parameter('device').value
        inference_rate_hz = float(self.get_parameter('inference_rate_hz').value)
        self._min_infer_period = 1.0 / inference_rate_hz if inference_rate_hz > 0.0 else 0.0
        self._last_infer_time = 0.0
        self.mouth_threshold_px = int(self.get_parameter('mouth_threshold_px').value)
        self.enable_pose_risk_scoring = bool(self.get_parameter('enable_pose_risk_scoring').value)
        self.enable_visualization = bool(self.get_parameter('enable_visualization').value)
        self.enable_unknown_small_object_risk = bool(
            self.get_parameter('enable_unknown_small_object_risk').value)
        self.unknown_small_object_class_name = (
            self.get_parameter('unknown_small_object_class_name').value.lower())
        self.small_object_reference_classes = {
            c.lower() for c in self.get_parameter('small_object_reference_classes').value
        }
        self.small_object_reference_area_scale = float(
            self.get_parameter('small_object_reference_area_scale').value)
        self.small_object_fallback_max_area_ratio = float(
            self.get_parameter('small_object_fallback_max_area_ratio').value)

        if YOLO is None:
            raise RuntimeError(
                "ultralytics가 설치되어 있지 않습니다. 'pip install ultralytics'를 먼저 실행하세요.")

        self.get_logger().info(f'YOLO11 모델 로드 중... ("{model_path}", device={self.device})')
        self.model = YOLO(model_path)
        self.get_logger().info(
            f'모델 로드 완료. class filter={sorted(self.target_classes)}, '
            f'conf_threshold={self.confidence_threshold}')

        self.mp_pose = None
        self.pose = None
        self.mp_drawing = None
        if self.enable_pose_risk_scoring:
            if mp is None:
                self.get_logger().warn(
                    'mediapipe가 설치되어 있지 않아 입-손 근접 판단(포즈 추정)을 건너뜁니다. '
                    "'pip install mediapipe'로 설치하면 활성화됩니다.")
            else:
                self.mp_pose = mp.solutions.pose
                self.pose = self.mp_pose.Pose(
                    min_detection_confidence=0.5, min_tracking_confidence=0.5)
                self.mp_drawing = mp.solutions.drawing_utils

        self.bridge = CvBridge()
        self.detections_pub = self.create_publisher(DetectionArray, detections_topic, 10)
        self.legacy_json_pub = self.create_publisher(String, legacy_json_topic, 10)
        self.image_sub = self.create_subscription(Image, image_topic, self._image_callback, 10)

        self.get_logger().info(
            f'yolo_detector_node 활성화 완료: {image_topic} -> {detections_topic}, {legacy_json_topic}')

    @staticmethod
    def _calculate_distance(p1, p2):
        """두 픽셀 좌표점 (x, y) 사이의 유클리드 거리를 계산합니다. (원본과 동일 로직)"""
        if not p1 or not p2:
            return float('inf')
        return math.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)

    @staticmethod
    def _bbox_area(x1, y1, x2, y2):
        """bbox의 픽셀 면적을 계산합니다."""
        return max(0, x2 - x1) * max(0, y2 - y1)

    def _is_unknown_small_object(self, bbox_area, frame_area, reference_areas):
        """미확인 객체 bbox가 삼킴 가능 크기 기준에 들어오는지 판단합니다."""
        if not self.enable_unknown_small_object_risk:
            return False
        if bbox_area <= 0 or frame_area <= 0:
            return False

        if reference_areas:
            max_reference_area = max(reference_areas) * self.small_object_reference_area_scale
            return bbox_area <= max_reference_area

        area_ratio = bbox_area / frame_area
        return area_ratio <= self.small_object_fallback_max_area_ratio

    def _image_callback(self, msg: Image):
        now = time.monotonic()
        if self._min_infer_period and (now - self._last_infer_time) < self._min_infer_period:
            return
        self._last_infer_time = now

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w, _ = frame.shape
        frame_area = w * h

        # ================================================================
        # [1. 인체 랜드마크 추출 (MediaPipe)] - 원본 vision_ros2_node.py 그대로 이식
        # 아이의 양손 끝(검지)과 입 중앙의 픽셀 좌표를 실시간으로 추출
        # ================================================================
        hands = []
        mouth = None
        if self.pose is not None and cv2 is not None:
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            pose_results = self.pose.process(rgb_frame)
            if pose_results.pose_landmarks:
                landmarks = pose_results.pose_landmarks.landmark
                l_index = landmarks[self.mp_pose.PoseLandmark.LEFT_INDEX]
                r_index = landmarks[self.mp_pose.PoseLandmark.RIGHT_INDEX]
                hands.append((int(l_index.x * w), int(l_index.y * h)))
                hands.append((int(r_index.x * w), int(r_index.y * h)))

                m_left = landmarks[self.mp_pose.PoseLandmark.MOUTH_LEFT]
                m_right = landmarks[self.mp_pose.PoseLandmark.MOUTH_RIGHT]
                mouth = (int(((m_left.x + m_right.x) / 2) * w),
                         int(((m_left.y + m_right.y) / 2) * h))

                if self.enable_visualization:
                    self.mp_drawing.draw_landmarks(
                        frame, pose_results.pose_landmarks, self.mp_pose.POSE_CONNECTIONS)

        # ================================================================
        # [2. YOLO 추론 + 위험도 판단] - 원본 vision_ros2_node.py 그대로 이식
        # ================================================================
        results = self.model.predict(
            source=frame, conf=self.confidence_threshold, device=self.device, verbose=False)

        pipeline_detections = []   # -> DetectionArray (ground_projection_node로 이어지는 입력)
        legacy_objects = []        # -> vision/detected_objects (원본 호환 JSON payload)
        parsed_detections = []
        reference_areas = []

        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                class_name = self.model.names[cls_id].lower()
                bbox_area = self._bbox_area(x1, y1, x2, y2)

                parsed_detections.append({
                    "x1": x1,
                    "y1": y1,
                    "x2": x2,
                    "y2": y2,
                    "bbox_area": bbox_area,
                    "cls_id": cls_id,
                    "conf": conf,
                    "class_name": class_name,
                })

                if class_name in self.small_object_reference_classes:
                    reference_areas.append(bbox_area)

        for detection in parsed_detections:
            x1 = detection["x1"]
            y1 = detection["y1"]
            x2 = detection["x2"]
            y2 = detection["y2"]
            bbox_area = detection["bbox_area"]
            cls_id = detection["cls_id"]
            conf = detection["conf"]
            class_name = detection["class_name"]

            risk_info = RISK_DATABASE.get(class_name)
            risk_class_name = class_name
            is_unknown_small_object = False

            if risk_info is None:
                if not self._is_unknown_small_object(bbox_area, frame_area, reference_areas):
                    continue
                risk_info = RISK_DATABASE.get(self.unknown_small_object_class_name)
                if risk_info is None:
                    self.get_logger().warn(
                        f'{self.unknown_small_object_class_name} 항목이 risk_db.py에 없습니다.',
                        throttle_duration_sec=2.0)
                    continue
                risk_class_name = self.unknown_small_object_class_name
                is_unknown_small_object = True
            elif self.target_classes and class_name not in self.target_classes:
                continue

            pipeline_detections.append(Detection(
                class_id=cls_id,
                class_name=class_name,
                confidence=conf,
                bbox=[float(x1), float(y1), float(x2), float(y2)],
            ))

            center_coords = (int((x1 + x2) / 2), int((y1 + y2) / 2))
            base_level = risk_info["level"]
            current_action = risk_info["robot_action_cmd"]
            warning_msg = risk_info["msg"]
            color = (0, 255, 255)  # 기본 주의: 노란색

            if is_unknown_small_object:
                warning_msg = (
                    f"작은 미확인 물체({class_name}) 감지. "
                    f"{risk_class_name} 기준 Lv{base_level}로 처리합니다.")

            # [예외처리] 인식률이 낮아(conf < 0.6) 형체가 애매한 경우 무조건 최소 Level 2 이상으로 간주
            if conf < 0.6 and base_level < 2:
                base_level = 2
                warning_msg = f"불확실성 높음: {class_name}을(를) Lv2로 상향 처리합니다."

            is_grabbed = False
            grabbed_hand = None
            is_mouth_threat = False

            for hand in hands:
                hx, hy = hand
                if x1 <= hx <= x2 and y1 <= hy <= y2:
                    is_grabbed = True
                    grabbed_hand = hand
                    break

            if mouth:
                if is_grabbed:
                    dist_hand_mouth = self._calculate_distance(grabbed_hand, mouth)
                    if dist_hand_mouth < self.mouth_threshold_px:
                        is_mouth_threat = True
                dist_obj_mouth = self._calculate_distance(center_coords, mouth)
                if dist_obj_mouth < self.mouth_threshold_px:
                    is_mouth_threat = True

            dynamic_priority = base_level
            if is_mouth_threat:
                dynamic_priority += 10
                current_action = risk_info["mouth_action_cmd"]
                warning_msg = f"🚨 비상! {class_name} 삼킴 위험 감지!"
                color = (0, 0, 255)
                self.get_logger().warn(warning_msg)
                if self.enable_visualization and cv2 is not None:
                    target_point = grabbed_hand if grabbed_hand else center_coords
                    cv2.line(frame, target_point, mouth, (0, 0, 255), 3)
            elif base_level == 3:
                color = (0, 0, 255)

            legacy_objects.append({
                "class": class_name,
                "risk_class": risk_class_name,
                "is_unknown_small_object": is_unknown_small_object,
                "confidence": round(conf, 2),
                "base_level": base_level,
                "dynamic_priority": dynamic_priority,
                "bounding_box": {
                    "x1": x1,
                    "y1": y1,
                    "x2": x2,
                    "y2": y2,
                    "area": bbox_area,
                    "area_ratio": round(bbox_area / frame_area, 5) if frame_area else 0.0,
                },
                "center_coords": {"x": center_coords[0], "y": center_coords[1]},
                "robot_action": current_action,
                "message": warning_msg,
            })

            if self.enable_visualization and cv2 is not None:
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                cv2.putText(frame, f"{class_name} P:{dynamic_priority} {current_action}",
                            (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        # ================================================================
        # [3. 발행] - 신규 파이프라인(DetectionArray) + 원본 호환 JSON 둘 다 발행
        # ================================================================
        if pipeline_detections:
            detection_array = DetectionArray()
            detection_array.header = msg.header
            detection_array.detections = pipeline_detections
            self.detections_pub.publish(detection_array)
            self.get_logger().info(
                f'{len(pipeline_detections)}개 관심 객체 검출, /detections 발행',
                throttle_duration_sec=2.0)

        if legacy_objects:
            legacy_objects.sort(key=lambda x: x['dynamic_priority'], reverse=True)
            legacy_msg = String()
            legacy_msg.data = json.dumps(legacy_objects, ensure_ascii=False)
            self.legacy_json_pub.publish(legacy_msg)

        if self.enable_visualization and cv2 is not None:
            cv2.imshow('Capstone Vision Main Node', frame)
            cv2.waitKey(1)

    def destroy_node(self):
        if self.enable_visualization and cv2 is not None:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
