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
  - 위 Level 2 기준에 걸리지 않은 미확인 객체 중 아이 입 크기보다 큰 bbox는
    unknown_large_object(Level 1, NONE)로 인식만 합니다.

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
from robot_perception.risk_db import DEFAULT_CLASS_ALIASES, RISK_DATABASE

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
        self.declare_parameter('enable_unknown_object_risk', True)
        self.declare_parameter('unknown_small_object_class_name', 'unknown_small_object')
        self.declare_parameter('unknown_large_object_class_name', 'unknown_large_object')
        self.declare_parameter('small_object_reference_classes', ['battery', 'coin', 'lego'])
        self.declare_parameter('small_object_reference_area_scale', 1.0)
        self.declare_parameter('small_object_fallback_max_area_ratio', 0.035)
        self.declare_parameter('mouth_size_area_scale', 1.0)
        self.declare_parameter('mouth_fallback_area_ratio', 0.035)
        self.declare_parameter(
            'class_aliases',
            [f'{src}:{dst}' for src, dst in sorted(DEFAULT_CLASS_ALIASES.items())])
        self.declare_parameter('stable_detection_frames', 3)
        self.declare_parameter('lost_track_ttl_frames', 5)
        self.declare_parameter('track_iou_threshold', 0.3)
        self.declare_parameter('mouth_threshold_scale', 4.0)
        self.declare_parameter('mouth_min_threshold_px', 60.0)
        self.declare_parameter('mouth_max_threshold_px', 180.0)

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
        self.enable_unknown_object_risk = bool(
            self.get_parameter('enable_unknown_object_risk').value)
        self.unknown_small_object_class_name = (
            self.get_parameter('unknown_small_object_class_name').value.lower())
        self.unknown_large_object_class_name = (
            self.get_parameter('unknown_large_object_class_name').value.lower())
        self.small_object_reference_classes = {
            c.lower() for c in self.get_parameter('small_object_reference_classes').value
        }
        self.small_object_reference_area_scale = float(
            self.get_parameter('small_object_reference_area_scale').value)
        self.small_object_fallback_max_area_ratio = float(
            self.get_parameter('small_object_fallback_max_area_ratio').value)
        self.mouth_size_area_scale = float(self.get_parameter('mouth_size_area_scale').value)
        self.mouth_fallback_area_ratio = float(
            self.get_parameter('mouth_fallback_area_ratio').value)
        self.class_aliases = self._parse_class_aliases(
            self.get_parameter('class_aliases').value)
        self.stable_detection_frames = max(
            1, int(self.get_parameter('stable_detection_frames').value))
        self.lost_track_ttl_frames = max(
            1, int(self.get_parameter('lost_track_ttl_frames').value))
        self.track_iou_threshold = float(self.get_parameter('track_iou_threshold').value)
        self.mouth_threshold_scale = float(self.get_parameter('mouth_threshold_scale').value)
        self.mouth_min_threshold_px = float(self.get_parameter('mouth_min_threshold_px').value)
        self.mouth_max_threshold_px = float(self.get_parameter('mouth_max_threshold_px').value)
        self._tracks = {}
        self._next_track_id = 1

        if YOLO is None:
            raise RuntimeError(
                "ultralytics가 설치되어 있지 않습니다. 'pip install ultralytics'를 먼저 실행하세요.")

        self.get_logger().info(f'YOLO11 모델 로드 중... ("{model_path}", device={self.device})')
        self.model = YOLO(model_path)
        self.get_logger().info(
            f'모델 로드 완료. target_classes={sorted(self.target_classes)}, '
            f'conf_threshold={self.confidence_threshold}, '
            f'class_aliases={len(self.class_aliases)}개')

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

    def _parse_class_aliases(self, alias_entries):
        """ROS 파라미터의 "model_class:risk_class" 목록을 dict로 변환합니다."""
        aliases = {}
        for entry in alias_entries:
            if ':' not in entry:
                self.get_logger().warn(f'잘못된 class_alias 형식입니다: {entry}')
                continue
            src, dst = entry.split(':', 1)
            src = src.strip().lower()
            dst = dst.strip().lower()
            if not src or not dst:
                self.get_logger().warn(f'비어 있는 class_alias 항목입니다: {entry}')
                continue
            aliases[src] = dst
        return aliases

    def _risk_class_for_model_class(self, class_name):
        """모델 출력 클래스를 안전 판단용 상위 클래스로 정규화합니다."""
        return self.class_aliases.get(class_name.lower(), class_name.lower())

    @staticmethod
    def _bbox_iou(a, b):
        """두 bbox(x1, y1, x2, y2)의 IoU를 계산합니다."""
        ax1, ay1, ax2, ay2 = a
        bx1, by1, bx2, by2 = b
        inter_x1 = max(ax1, bx1)
        inter_y1 = max(ay1, by1)
        inter_x2 = min(ax2, bx2)
        inter_y2 = min(ay2, by2)
        inter_area = max(0, inter_x2 - inter_x1) * max(0, inter_y2 - inter_y1)
        if inter_area <= 0:
            return 0.0
        area_a = max(0, ax2 - ax1) * max(0, ay2 - ay1)
        area_b = max(0, bx2 - bx1) * max(0, by2 - by1)
        union_area = area_a + area_b - inter_area
        return inter_area / union_area if union_area else 0.0

    def _update_track(self, risk_class_name, bbox, matched_track_ids):
        """IoU 기반으로 같은 위험물을 시간축에서 묶고 안정 검출 여부를 반환합니다."""
        best_track_id = None
        best_iou = 0.0
        for track_id, track in self._tracks.items():
            if track_id in matched_track_ids:
                continue
            if track['risk_class_name'] != risk_class_name:
                continue
            iou = self._bbox_iou(bbox, track['bbox'])
            if iou > best_iou:
                best_track_id = track_id
                best_iou = iou

        if best_track_id is None or best_iou < self.track_iou_threshold:
            best_track_id = self._next_track_id
            self._next_track_id += 1
            self._tracks[best_track_id] = {
                'risk_class_name': risk_class_name,
                'bbox': bbox,
                'seen_frames': 0,
                'missed_frames': 0,
            }

        track = self._tracks[best_track_id]
        track['bbox'] = bbox
        track['seen_frames'] += 1
        track['missed_frames'] = 0
        matched_track_ids.add(best_track_id)
        return best_track_id, track['seen_frames'], track['seen_frames'] >= self.stable_detection_frames

    def _age_unmatched_tracks(self, matched_track_ids):
        """현재 프레임에서 매칭되지 않은 오래된 track을 제거합니다."""
        stale_track_ids = []
        for track_id, track in self._tracks.items():
            if track_id in matched_track_ids:
                continue
            track['missed_frames'] += 1
            if track['missed_frames'] > self.lost_track_ttl_frames:
                stale_track_ids.append(track_id)
        for track_id in stale_track_ids:
            del self._tracks[track_id]

    def _mouth_distance_threshold(self, mouth_width_px, frame_area):
        """고정 픽셀값 대신 입 크기/프레임 크기를 반영한 입 근접 거리 기준을 만듭니다."""
        if mouth_width_px and mouth_width_px > 0:
            threshold = mouth_width_px * self.mouth_threshold_scale
            source = 'mouth_width_scaled'
        else:
            frame_side = math.sqrt(frame_area) if frame_area > 0 else self.mouth_threshold_px
            threshold = min(self.mouth_threshold_px, frame_side * 0.16)
            source = 'fallback_frame_scale'

        threshold = max(self.mouth_min_threshold_px, threshold)
        threshold = min(self.mouth_max_threshold_px, threshold)
        return threshold, source

    @staticmethod
    def _risk_score(base_level, conf, is_mouth_threat, is_grabbed,
                    is_track_stable, is_unknown_small_object):
        """위험도, 신뢰도, 행동 맥락, 시간축 안정성을 하나의 우선순위 점수로 통합합니다."""
        score = base_level * 20.0
        score += max(0.0, min(conf, 1.0)) * 10.0
        if is_track_stable:
            score += 10.0
        if is_grabbed:
            score += 15.0
        if is_mouth_threat:
            score += 40.0
        if is_unknown_small_object:
            score += 5.0
        return round(score, 2)

    def _mouth_area_threshold(self, mouth_width_px, frame_area):
        """입꼬리 간 거리로 미확인 객체 분류용 입 크기 면적 기준을 계산합니다."""
        if mouth_width_px and mouth_width_px > 0:
            return (mouth_width_px ** 2) * self.mouth_size_area_scale, "mediapipe_mouth"
        if frame_area > 0 and self.mouth_fallback_area_ratio > 0:
            return frame_area * self.mouth_fallback_area_ratio, "fallback_area_ratio"
        return None, "unavailable"

    def _is_unknown_small_object(self, bbox_area, frame_area, reference_areas):
        """미등록 객체 bbox가 기존 소형 위험물 기준보다 작은지 판단합니다."""
        if bbox_area <= 0 or frame_area <= 0:
            return False

        if reference_areas:
            max_reference_area = max(reference_areas) * self.small_object_reference_area_scale
            return bbox_area <= max_reference_area

        area_ratio = bbox_area / frame_area
        return area_ratio <= self.small_object_fallback_max_area_ratio

    def _unknown_object_risk_class(self, bbox_area, frame_area, reference_areas,
                                   mouth_area_threshold):
        """미등록 객체를 기존 소형 위험물 기준과 입 크기 기준으로 DB 항목에 매핑합니다."""
        if not self.enable_unknown_object_risk:
            return None
        if bbox_area <= 0:
            return None

        if self._is_unknown_small_object(bbox_area, frame_area, reference_areas):
            return self.unknown_small_object_class_name

        if mouth_area_threshold is None:
            return None
        if bbox_area > mouth_area_threshold:
            return self.unknown_large_object_class_name
        return None

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
        mouth_width_px = None
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
                mouth_left = (int(m_left.x * w), int(m_left.y * h))
                mouth_right = (int(m_right.x * w), int(m_right.y * h))
                mouth = (int((mouth_left[0] + mouth_right[0]) / 2),
                         int((mouth_left[1] + mouth_right[1]) / 2))
                mouth_width_px = self._calculate_distance(mouth_left, mouth_right)

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
        matched_track_ids = set()
        mouth_area_threshold, mouth_size_source = self._mouth_area_threshold(
            mouth_width_px, frame_area)
        mouth_distance_threshold, mouth_distance_source = self._mouth_distance_threshold(
            mouth_width_px, frame_area)

        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                class_name = self.model.names[cls_id].lower()
                aliased_risk_class_name = self._risk_class_for_model_class(class_name)
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
                    "aliased_risk_class_name": aliased_risk_class_name,
                })

                if aliased_risk_class_name in self.small_object_reference_classes:
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
            aliased_risk_class_name = detection["aliased_risk_class_name"]

            risk_info = RISK_DATABASE.get(aliased_risk_class_name)
            risk_class_name = aliased_risk_class_name
            is_unknown_small_object = False
            is_unknown_large_object = False
            reason_codes = []

            if risk_class_name != class_name:
                reason_codes.append(f'CLASS_ALIAS:{class_name}->{risk_class_name}')

            if risk_info is None:
                risk_class_name = self._unknown_object_risk_class(
                    bbox_area, frame_area, reference_areas, mouth_area_threshold)
                if risk_class_name is None:
                    continue
                risk_info = RISK_DATABASE.get(risk_class_name)
                if risk_info is None:
                    self.get_logger().warn(
                        f'{risk_class_name} 항목이 risk_db.py에 없습니다.',
                        throttle_duration_sec=2.0)
                    continue
                is_unknown_small_object = risk_class_name == self.unknown_small_object_class_name
                is_unknown_large_object = risk_class_name == self.unknown_large_object_class_name
                if is_unknown_small_object:
                    reason_codes.append('UNKNOWN_SMALL_OBJECT')
                if is_unknown_large_object:
                    reason_codes.append('UNKNOWN_LARGE_OBJECT')

            center_coords = (int((x1 + x2) / 2), int((y1 + y2) / 2))
            base_level = risk_info["level"]
            current_action = risk_info["robot_action_cmd"]
            warning_msg = risk_info["msg"]
            color = (0, 255, 255)  # 기본 주의: 노란색
            reason_codes.append(f'RISK_LEVEL_{base_level}')

            if is_unknown_small_object:
                warning_msg = (
                    f"기존 소형 위험물보다 작은 미확인 물체({class_name}) 감지. "
                    f"{risk_class_name} 기준 Lv{base_level}로 처리합니다.")
            elif is_unknown_large_object:
                warning_msg = (
                    f"입 크기보다 큰 미확인 물체({class_name}) 감지. "
                    f"{risk_class_name} 기준 Lv{base_level}, action={current_action}로 인식만 합니다.")
                color = (0, 255, 0)

            # [예외처리] 인식률이 낮아(conf < 0.6) 형체가 애매한 경우 무조건 최소 Level 2 이상으로 간주
            is_recognition_only = current_action == "NONE" and risk_info["mouth_action_cmd"] == "NONE"
            if conf < 0.6 and base_level < 2 and not is_recognition_only:
                base_level = 2
                warning_msg = f"불확실성 높음: {class_name}을(를) Lv2로 상향 처리합니다."
                reason_codes.append('LOW_CONFIDENCE_LEVEL_UP')

            is_grabbed = False
            grabbed_hand = None
            is_mouth_threat = False

            for hand in hands:
                hx, hy = hand
                if x1 <= hx <= x2 and y1 <= hy <= y2:
                    is_grabbed = True
                    grabbed_hand = hand
                    reason_codes.append('HAND_INSIDE_BBOX')
                    break

            if mouth:
                if is_grabbed:
                    dist_hand_mouth = self._calculate_distance(grabbed_hand, mouth)
                    if dist_hand_mouth < mouth_distance_threshold:
                        is_mouth_threat = True
                        reason_codes.append('HAND_NEAR_MOUTH')
                dist_obj_mouth = self._calculate_distance(center_coords, mouth)
                if dist_obj_mouth < mouth_distance_threshold:
                    is_mouth_threat = True
                    reason_codes.append('OBJECT_NEAR_MOUTH')

            track_id, seen_frames, is_track_stable = self._update_track(
                risk_class_name, (x1, y1, x2, y2), matched_track_ids)
            if is_track_stable:
                reason_codes.append(f'STABLE_TRACK_{seen_frames}_FRAMES')
            else:
                reason_codes.append(f'UNSTABLE_TRACK_{seen_frames}_OF_{self.stable_detection_frames}')

            dynamic_priority = self._risk_score(
                base_level, conf, is_mouth_threat, is_grabbed,
                is_track_stable, is_unknown_small_object)
            if is_mouth_threat and base_level >= 2:
                current_action = "EMERGENCY_STOP"
                warning_msg = f"🚨 비상! {class_name} 삼킴 위험 감지!"
                color = (0, 0, 255)
                self.get_logger().warn(warning_msg)
                if self.enable_visualization and cv2 is not None:
                    target_point = grabbed_hand if grabbed_hand else center_coords
                    cv2.line(frame, target_point, mouth, (0, 0, 255), 3)
            elif not is_track_stable and current_action == "REMOVE":
                current_action = "NONE"
                warning_msg = (
                    f"{class_name} 후보 감지: {seen_frames}/{self.stable_detection_frames}프레임 "
                    "연속 확인 후 수거 대상으로 확정합니다.")
            elif base_level == 3:
                color = (0, 0, 255)

            pipeline_detections.append(Detection(
                class_id=cls_id,
                class_name=class_name,
                confidence=conf,
                bbox=[float(x1), float(y1), float(x2), float(y2)],
                risk_class_name=risk_class_name,
                risk_level=int(base_level),
                risk_score=float(dynamic_priority),
                robot_action=current_action,
                reason_codes=reason_codes,
            ))

            legacy_objects.append({
                "class": class_name,
                "risk_class": risk_class_name,
                "is_unknown_small_object": is_unknown_small_object,
                "is_unknown_large_object": is_unknown_large_object,
                "confidence": round(conf, 2),
                "base_level": base_level,
                "dynamic_priority": dynamic_priority,
                "track": {
                    "id": track_id,
                    "seen_frames": seen_frames,
                    "stable": is_track_stable,
                    "required_stable_frames": self.stable_detection_frames,
                },
                "bounding_box": {
                    "x1": x1,
                    "y1": y1,
                    "x2": x2,
                    "y2": y2,
                    "area": bbox_area,
                    "area_ratio": round(bbox_area / frame_area, 5) if frame_area else 0.0,
                },
                "mouth_size": {
                    "estimated_width_px": round(mouth_width_px, 2) if mouth_width_px else None,
                    "area_threshold_px": round(mouth_area_threshold, 2)
                    if mouth_area_threshold else None,
                    "source": mouth_size_source,
                    "distance_threshold_px": round(mouth_distance_threshold, 2),
                    "distance_source": mouth_distance_source,
                },
                "center_coords": {"x": center_coords[0], "y": center_coords[1]},
                "robot_action": current_action,
                "reason_codes": reason_codes,
                "message": warning_msg,
            })

            if self.enable_visualization and cv2 is not None:
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                cv2.putText(frame, f"{class_name} P:{dynamic_priority} {current_action}",
                            (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        self._age_unmatched_tracks(matched_track_ids)

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
