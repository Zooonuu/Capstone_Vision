# vision_ros2_node.py
import cv2
import json
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ultralytics import YOLO
import mediapipe as mp

# 앞서 만든 위험도 DB 임포트 (risk_db.py)
from risk_db import RISK_DATABASE

class VisionDetectionNode(Node):
    def __init__(self):
        super().__init__('vision_detection_node')
        
        # ROS 2 Publisher 설정 ('vision/detected_objects' 토픽 발행)
        self.publisher_ = self.create_publisher(String, 'vision/detected_objects', 10)
        
        # =====================================================================
        # [1. 객체 및 인체 인식 모듈 구축]
        # YOLOv11(물체 인식) 및 MediaPipe(인체 포즈 추정) 모델 로드 및 초기화
        # =====================================================================
        self.get_logger().info('YOLO & MediaPipe 모델 로드 중...')
        self.model = YOLO('best.pt') 
        
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5)
        self.mp_drawing = mp.solutions.drawing_utils
        
        # 웹캠 연결 및 타이머(FPS) 설정
        self.cap = cv2.VideoCapture(0)
        self.timer = self.create_timer(0.033, self.timer_callback) # 약 30FPS
        self.get_logger().info('비전 감지 노드가 활성화되었습니다. (우선순위 알고리즘 적용)')

    def calculate_distance(self, p1, p2):
        """두 픽셀 좌표점 (x, y) 사이의 유클리드 거리를 계산합니다."""
        if not p1 or not p2: return float('inf')
        return math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret: return
        h, w, _ = frame.shape

        # =====================================================================
        # [1-1. 인체 랜드마크 추출 (MediaPipe)]
        # 아이의 양손 끝(검지)과 입 중앙의 픽셀 좌표를 실시간으로 추출
        # =====================================================================
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        pose_results = self.pose.process(rgb_frame)
        
        hands = [] # [(x, y), (x, y)]
        mouth = None # (x, y)
        mouth_width_px = None

        if pose_results.pose_landmarks:
            landmarks = pose_results.pose_landmarks.landmark
            
            # 손 좌표 추출
            l_index = landmarks[self.mp_pose.PoseLandmark.LEFT_INDEX]
            r_index = landmarks[self.mp_pose.PoseLandmark.RIGHT_INDEX]
            hands.append((int(l_index.x * w), int(l_index.y * h)))
            hands.append((int(r_index.x * w), int(r_index.y * h)))
            
            # 입 좌표 추출 (좌우 입꼬리의 중앙값)
            m_left = landmarks[self.mp_pose.PoseLandmark.MOUTH_LEFT]
            m_right = landmarks[self.mp_pose.PoseLandmark.MOUTH_RIGHT]
            mouth_left = (int(m_left.x * w), int(m_left.y * h))
            mouth_right = (int(m_right.x * w), int(m_right.y * h))
            mouth = (int((mouth_left[0] + mouth_right[0]) / 2),
                     int((mouth_left[1] + mouth_right[1]) / 2))
            mouth_width_px = self.calculate_distance(mouth_left, mouth_right)

            # 시각화
            self.mp_drawing.draw_landmarks(frame, pose_results.pose_landmarks, self.mp_pose.POSE_CONNECTIONS)

        # =====================================================================
        # [2. 객체 인식 및 위험도 DB 연동 로직]
        # YOLOv11 추론 진행 (낮은 신뢰도 객체도 잡기 위해 conf를 0.3으로 하향)
        # =====================================================================
        results = self.model.predict(source=frame, conf=0.3, verbose=False)
        detected_objects = [] # 프레임 내 감지된 모든 객체의 데이터를 임시 저장할 리스트
        parsed_detections = []
        reference_areas = []
        small_object_reference_classes = {"battery", "coin", "lego"}
        small_object_reference_area_scale = 1.0
        small_object_fallback_max_area_ratio = 0.035
        mouth_area_threshold = (
            (mouth_width_px ** 2) if mouth_width_px else (w * h * 0.035)
        )

        for result in results:
            for box in result.boxes:
                # 좌표 및 기본 정보 추출
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                class_name = self.model.names[cls_id].lower() 
                bbox_area = max(0, x2 - x1) * max(0, y2 - y1)

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

                if class_name in small_object_reference_classes:
                    reference_areas.append(bbox_area)

        for detection in parsed_detections:
                x1 = detection["x1"]
                y1 = detection["y1"]
                x2 = detection["x2"]
                y2 = detection["y2"]
                bbox_area = detection["bbox_area"]
                conf = detection["conf"]
                class_name = detection["class_name"]
                center_coords = (int((x1 + x2) / 2), int((y1 + y2) / 2))

                # DB 매칭
                risk_info = RISK_DATABASE.get(class_name, None)
                risk_class_name = class_name
                is_unknown_small_object = False
                is_unknown_large_object = False
                if not risk_info:
                    if reference_areas:
                        small_area_threshold = max(reference_areas) * small_object_reference_area_scale
                        is_small_unknown = bbox_area <= small_area_threshold
                    else:
                        is_small_unknown = (
                            bbox_area / (w * h) <= small_object_fallback_max_area_ratio
                            if w * h else False
                        )

                    if is_small_unknown:
                        risk_class_name = "unknown_small_object"
                    elif bbox_area > mouth_area_threshold:
                        risk_class_name = "unknown_large_object"
                    else:
                        continue

                    risk_info = RISK_DATABASE.get(risk_class_name, None)
                    if not risk_info:
                        continue
                    is_unknown_small_object = risk_class_name == "unknown_small_object"
                    is_unknown_large_object = risk_class_name == "unknown_large_object"

                # 기본 데이터 세팅
                base_level = risk_info["level"]
                current_action = risk_info["robot_action_cmd"]
                warning_msg = risk_info["msg"]
                color = (0, 255, 255) # 노란색 (기본 주의)
                if is_unknown_small_object:
                    warning_msg = (
                        f"기존 소형 위험물보다 작은 미확인 물체({class_name}) 감지. "
                        f"{risk_class_name} 기준 Lv{base_level}로 처리합니다."
                    )
                elif is_unknown_large_object:
                    warning_msg = (
                        f"입 크기보다 큰 미확인 물체({class_name}) 감지. "
                        f"{risk_class_name} 기준 Lv{base_level}, action={current_action}로 인식만 합니다."
                    )
                    color = (0, 255, 0)

                # -------------------------------------------------------------
                # [예외처리 1: 판단]
                # 인식률이 낮아(conf < 0.6) 형체가 애매한 경우 무조건 최소 Level 2 이상으로 간주
                # -------------------------------------------------------------
                is_recognition_only = current_action == "NONE" and risk_info["mouth_action_cmd"] == "NONE"
                if conf < 0.6 and base_level < 2 and not is_recognition_only:
                    base_level = 2
                    warning_msg = f"불확실성 높음: {class_name}을(를) Lv2로 상향 처리합니다."

                # =====================================================================
                # [4. 돌발 상황 알고리즘 (삼킴 차단)]
                # 조건 A (손&&물체&&입) : 손이 바운딩 박스 안에 있고, 그 손이 입과 가까워짐
                # 조건 B (물체&&입) : 물체의 중심이 입과 가까워짐 (던지거나 고개를 숙인 경우)
                # =====================================================================
                is_grabbed = False
                grabbed_hand = None
                distance_to_mouth = float('inf')
                is_mouth_threat = False # 돌발 상황(입 근처) 트리거 플래그

                # 1. 겹침 감지 (손이 BB 안에 들어왔는가?)
                for hand in hands:
                    hx, hy = hand
                    if x1 <= hx <= x2 and y1 <= hy <= y2:
                        is_grabbed = True
                        grabbed_hand = hand
                        break
                
                # 2. 거리 계산 및 판별 (임계치: 120 픽셀 기준)
                MOUTH_THRESHOLD = 120

                if mouth:
                    # 조건 A: 손으로 집은 상태에서 손-입 거리가 가까운가?
                    if is_grabbed:
                        dist_hand_mouth = self.calculate_distance(grabbed_hand, mouth)
                        if dist_hand_mouth < MOUTH_THRESHOLD:
                            distance_to_mouth = dist_hand_mouth
                            is_mouth_threat = True
                    
                    # 조건 B: 물체 자체가 입과 가까운가? (물체 중심 - 입 중앙)
                    dist_obj_mouth = self.calculate_distance(center_coords, mouth)
                    if dist_obj_mouth < MOUTH_THRESHOLD:
                        distance_to_mouth = min(distance_to_mouth, dist_obj_mouth)
                        is_mouth_threat = True

                # =====================================================================
                # [3. 다중 객체 우선 순위 정렬 및 오버라이드 로직]
                # 위험도(Level)를 기반으로 우선순위 점수 부여
                # =====================================================================
                dynamic_priority = base_level

                if is_mouth_threat and base_level >= 2:
                    # [도달 시간 기반 오버라이드] 
                    # 입 주변의 돌발 상황 발생 시, 해당 물체의 우선순위를 최상위(+10점)로 강제 덮어쓰기
                    # (예: 저 멀리 있는 Lv3 건전지보다 입 앞의 Lv2 레고가 우선순위가 높아짐)
                    dynamic_priority += 10 
                    current_action = "EMERGENCY_STOP" # Level 2 이상 입 근접 시 긴급정지
                    warning_msg = f"🚨 비상! {class_name} 삼킴 위험 감지!"
                    color = (0, 0, 255) # 빨간색 경고
                    
                    # 시각화: 경고선
                    target_point = grabbed_hand if grabbed_hand else center_coords
                    cv2.line(frame, target_point, mouth, (0, 0, 255), 3)

                elif base_level == 3:
                    # 평상시 Level 3 (건전지 등)는 빨간색으로 시각화 유지
                    color = (0, 0, 255)

                # 취합된 객체 정보를 딕셔너리로 저장
                obj_data = {
                    "class": class_name,
                    "risk_class": risk_class_name,
                    "is_unknown_small_object": is_unknown_small_object,
                    "is_unknown_large_object": is_unknown_large_object,
                    "confidence": round(conf, 2),
                    "base_level": base_level,
                    "dynamic_priority": dynamic_priority, #  점수 바탕으로 우선순위 판단
                    "bounding_box": {
                        "x1": x1,
                        "y1": y1,
                        "x2": x2,
                        "y2": y2,
                        "area": bbox_area,
                    },
                    "mouth_size": {
                        "estimated_width_px": round(mouth_width_px, 2) if mouth_width_px else None,
                        "area_threshold_px": round(mouth_area_threshold, 2),
                        "source": "mediapipe_mouth" if mouth_width_px else "fallback_area_ratio",
                    },
                    "center_coords": {"x": center_coords[0], "y": center_coords[1]},
                    "robot_action": current_action,
                    "message": warning_msg
                }
                detected_objects.append(obj_data)
                
                # 화면 BB 및 텍스트 시각화
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                cv2.putText(frame, f"{class_name} P:{dynamic_priority} {current_action}", 
                            (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        # =====================================================================
        # [3-1. 위험도 기반 정렬 및 발행 (Publish)]
        # =====================================================================
        if detected_objects:
            # 우선순위가 높은 순서대로 내림차순 정렬
            # 즉, detected_objects[0] 이 항상 로봇이 지금 당장 처리해야 할 1순위 타겟이 됨
            detected_objects.sort(key=lambda x: x['dynamic_priority'], reverse=True)

            msg = String()
            msg.data = json.dumps(detected_objects) # JSON Array 형태로 변환하여 발행
            self.publisher_.publish(msg)

        cv2.imshow('Capstone Vision Main Node', frame)
        cv2.waitKey(1)

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    vision_node = VisionDetectionNode()
    try:
        rclpy.spin(vision_node)
    except KeyboardInterrupt:
        vision_node.get_logger().info('비전 감지 노드가 종료되었습니다.')
    finally:
        vision_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
