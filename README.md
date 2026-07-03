# Capstone_Vision (Ver1.) (7.1 ~ 7.7)

# 26.7.1 업데이트

1. 훈련모델 완료 / test_image로 다른 이미지로 검증 가능 (epoch 너무 낮아서 사실 안 뜸, config 값 0.1로 일단 진행, GPU value = 0으로 두고 epoch 크게 늘려서 실행 필요)
2. 이후 웹켐으로도 인식이 되는 지 확인 (vision_ros2_node.py) (현재 허준우 우분투 환경에서 노트북캠 연결 안되는 문제 발생, 실제 웹캠 연결해서 실행 필요)

# 26.7.2 업데이트

1. 훈련모델 검증 val_model.py 구성 (mAP)
2. vision_ros2_node.py 업데이트 - 
    "비전 파트에서 JSON Array 형태로 객체 리스트를 쏠 거야. 여기서 제일 중요한 건, 우리가 이미 내부적으로 가장 위험하고 시급한 물체를 0번 인덱스(parsed_data[0])로 정렬해 두었어."

    "그러니까 ROS2 쪽에서는 여러 물체가 들어와도 고민할 필요 없이 무조건 배열의 첫 번째 데이터(data[0])만 뽑아서 그 안의 robot_action이 REMOVE면 center_coords로 주행을 시키고, EMERGENCY_STOP이나 ALARM이 뜨면 즉각 주행 취소 및 스피커 모듈을 울려주면 돼!"

3. vision_ros2_node.py (좀 더 직관적인 버전 - 조건문 넣어서 조금 더 직관적이라고는 하는데 잘 모르겠음) 

vision_ros2_node.py (직관적인 버전)
import cv2
import json
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ultralytics import YOLO
import mediapipe as mp

from risk_db import RISK_DATABASE

class VisionDetectionNode(Node):
    def __init__(self):
        super().__init__('vision_detection_node')
        self.publisher_ = self.create_publisher(String, 'vision/detected_objects', 10)
        
        self.get_logger().info('YOLO & MediaPipe 모델 로드 중...')
        self.model = YOLO('best.pt') 
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5)
        self.mp_drawing = mp.solutions.drawing_utils
        
        self.cap = cv2.VideoCapture(0)
        self.timer = self.create_timer(0.033, self.timer_callback) 
        self.get_logger().info('비전 감지 노드 활성화 (제어팀 맞춤형 JSON 발행)')

    def calculate_distance(self, p1, p2):
        if not p1 or not p2: return float('inf')
        return math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret: return
        h, w, _ = frame.shape

        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        pose_results = self.pose.process(rgb_frame)
        
        hands = [] 
        mouth = None 

        if pose_results.pose_landmarks:
            landmarks = pose_results.pose_landmarks.landmark
            l_index = landmarks[self.mp_pose.PoseLandmark.LEFT_INDEX]
            r_index = landmarks[self.mp_pose.PoseLandmark.RIGHT_INDEX]
            hands.append((int(l_index.x * w), int(l_index.y * h)))
            hands.append((int(r_index.x * w), int(r_index.y * h)))
            
            m_left = landmarks[self.mp_pose.PoseLandmark.MOUTH_LEFT]
            m_right = landmarks[self.mp_pose.PoseLandmark.MOUTH_RIGHT]
            mouth = (int(((m_left.x + m_right.x) / 2) * w), int(((m_left.y + m_right.y) / 2) * h))

            self.mp_drawing.draw_landmarks(frame, pose_results.pose_landmarks, self.mp_pose.POSE_CONNECTIONS)

        results = self.model.predict(source=frame, conf=0.3, verbose=False)
        detected_objects = [] 

        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                center_coords = (int((x1 + x2) / 2), int((y1 + y2) / 2))
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                class_name = self.model.names[cls_id].lower() 
                
                risk_info = RISK_DATABASE.get(class_name, None)
                if not risk_info: continue

                base_level = risk_info["level"]
                current_action = risk_info["robot_action_cmd"]
                warning_msg = risk_info["msg"]
                color = (0, 255, 255) 

                if conf < 0.6 and base_level < 2:
                    base_level = 2
                    warning_msg = f"불확실성 높음: {class_name}을(를) Lv2로 상향 처리합니다."

                is_grabbed = False
                grabbed_hand = None
                distance_to_mouth = float('inf')
                is_mouth_threat = False 

                for hand in hands:
                    hx, hy = hand
                    if x1 <= hx <= x2 and y1 <= hy <= y2:
                        is_grabbed = True
                        grabbed_hand = hand
                        break
                
                MOUTH_THRESHOLD = 120

                if mouth:
                    if is_grabbed:
                        dist_hand_mouth = self.calculate_distance(grabbed_hand, mouth)
                        if dist_hand_mouth < MOUTH_THRESHOLD:
                            distance_to_mouth = dist_hand_mouth
                            is_mouth_threat = True
                    
                    dist_obj_mouth = self.calculate_distance(center_coords, mouth)
                    if dist_obj_mouth < MOUTH_THRESHOLD:
                        distance_to_mouth = min(distance_to_mouth, dist_obj_mouth)
                        is_mouth_threat = True

                dynamic_priority = base_level

                if is_mouth_threat:
                    dynamic_priority += 10 
                    current_action = risk_info["mouth_action_cmd"] 
                    warning_msg = f"🚨 비상! {class_name} 삼킴 위험 감지!"
                    color = (0, 0, 255) 
                    
                    target_point = grabbed_hand if grabbed_hand else center_coords
                    cv2.line(frame, target_point, mouth, (0, 0, 255), 3)

                elif base_level == 3:
                    color = (0, 0, 255)

                obj_data = {
                    "class": class_name,
                    "confidence": round(conf, 2),
                    "base_level": base_level,
                    "dynamic_priority": dynamic_priority, 
                    "bounding_box": {"x1": x1, "y1": y1, "x2": x2, "y2": y2},
                    "center_coords": {"x": center_coords[0], "y": center_coords[1]},
                    "robot_action": current_action,
                    "message": warning_msg
                }
                detected_objects.append(obj_data)
                
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                cv2.putText(frame, f"{class_name} P:{dynamic_priority} {current_action}", 
                            (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        # =====================================================================
        # [주제 3-1. 위험도 기반 정렬 및 직관적인 ROS 2 발행 (Publish)]
        # 제어 파트가 배열을 뒤지지 않고 즉각적으로 명령을 내릴 수 있도록 구조화
        # =====================================================================
        if detected_objects:
            # 1. 가장 위험한 타겟 1개만 색출 (우선순위 내림차순 정렬 후 0번 인덱스)
            detected_objects.sort(key=lambda x: x['dynamic_priority'], reverse=True)
            primary_target = detected_objects[0]
            
            # 2. 비상 상황 여부 직관적 플래그 (제어팀은 이 플래그만 보고 모터를 멈추면 됨)
            is_emergency = primary_target["robot_action"] in ["EMERGENCY_STOP", "ALARM"]

            # 3. 제어팀 맞춤형 "상황판" JSON 생성
            ros2_payload = {
                "is_emergency": is_emergency,               # True/False (즉각 정지 여부)
                "action_command": primary_target["robot_action"], # REMOVE, EMERGENCY_STOP 등
                "target_coords": primary_target["center_coords"], # 주행해야 할 x, y 좌표
                "target_name": primary_target["class"],           # 타겟 이름
                "alert_message": primary_target["message"],       # 터미널이나 UI에 띄울 경고문구
                "all_objects": detected_objects             # 필요시 참고할 전체 객체 데이터 (후순위)
            }

            msg = String()
            # ensure_ascii=False 를 넣어야 제어팀 터미널에서 한글이 깨지지 않고 보입니다!
            msg.data = json.dumps(ros2_payload, ensure_ascii=False) 
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

# 26.7.3 업데이트
1. 깃허브 추가 / 노션 공유
2. 판단 / 제어 판단 더 강화해서 설득력을 높이기
3. 2번에 맞는 코드 수정 적용
4. 배터리 / 동전 라벨링