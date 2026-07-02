# vision_ros2_node.py
import cv2
import json
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ultralytics import YOLO
import mediapipe as mp

# 앞서 만든 DB 파일 임포트
from risk_db import RISK_DATABASE

class VisionDetectionNode(Node):
    def __init__(self):
        super().__init__('vision_detection_node')
        
        # ROS 2 Publisher 설정
        self.publisher_ = self.create_publisher(String, 'vision/detected_objects', 10)
        
        # 1. YOLOv11 모델 로드 (학습된 best.pt 사용)
        self.get_logger().info('YOLO 모델 로드 중...')
        self.model = YOLO('best.pt') 
        
        # 2. MediaPipe Pose (포즈 추정) 초기화
        # 손과 입 좌표를 동시에 가져오기 위해 Pose 모델을 사용
        self.get_logger().info('MediaPipe 로드 중...')
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )
        self.mp_drawing = mp.solutions.drawing_utils
        
        # 웹캠 연결
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error('웹캠을 열 수 없습니다!')
        
        # 타이머 설정 (약 30FPS)
        self.timer = self.create_timer(0.033, self.timer_callback)
        self.get_logger().info('비전 감지 노드(YOLO + MediaPipe)가 시작되었습니다.')

    def calculate_distance(self, p1, p2):
        """두 점(x, y) 사이의 유클리드 거리를 계산합니다."""
        return math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning('프레임을 읽을 수 없습니다.')
            return

        h, w, _ = frame.shape
        detected_data_list = []

        # ----------------------------------------------------
        # [단계 1] MediaPipe 로 인체(손, 입) 랜드마크 추출
        # ----------------------------------------------------
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        pose_results = self.pose.process(rgb_frame)
        
        hands = [] # 손 좌표를 담을 리스트 [(x, y), (x, y)]
        mouth = None # 입의 중심 좌표 (x, y)

        if pose_results.pose_landmarks:
            landmarks = pose_results.pose_landmarks.landmark
            
            # 양손(검지 손가락 끝) 픽셀 좌표 추출
            l_index = landmarks[self.mp_pose.PoseLandmark.LEFT_INDEX]
            r_index = landmarks[self.mp_pose.PoseLandmark.RIGHT_INDEX]
            hands.append((int(l_index.x * w), int(l_index.y * h)))
            hands.append((int(r_index.x * w), int(r_index.y * h)))
            
            # 입(좌우 끝점의 중간) 픽셀 좌표 추출
            m_left = landmarks[self.mp_pose.PoseLandmark.MOUTH_LEFT]
            m_right = landmarks[self.mp_pose.PoseLandmark.MOUTH_RIGHT]
            mouth_x = int(((m_left.x + m_right.x) / 2) * w)
            mouth_y = int(((m_left.y + m_right.y) / 2) * h)
            mouth = (mouth_x, mouth_y)

            # 시각화 (뼈대 그리기)
            self.mp_drawing.draw_landmarks(frame, pose_results.pose_landmarks, self.mp_pose.POSE_CONNECTIONS)

        # ----------------------------------------------------
        # [단계 2] YOLOv11 객체 탐지 및 위험도 평가
        # ----------------------------------------------------
        results = self.model.predict(source=frame, conf=0.5, verbose=False)
        
        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                center_x, center_y = int((x1 + x2) / 2), int((y1 + y2) / 2)
                
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                class_name = self.model.names[cls_id].lower() 

                risk_info = RISK_DATABASE.get(class_name, None)
                
                if risk_info:
                    current_robot_action = risk_info["robot_action_cmd"]
                    warning_msg = risk_info["msg"]
                    color = (0, 255, 255) # 기본은 노란색 (주의)
                    
                    # ----------------------------------------------------
                    # [단계 3] 손-입 추적 알고리즘 (충돌 감지 및 거리 계산)
                    # ----------------------------------------------------
                    # 손이 객체의 바운딩 박스 안에 들어왔는지 검사 (충돌 감지)
                    is_grabbed = False
                    grabbed_hand = None
                    
                    for hand in hands:
                        hx, hy = hand
                        if x1 <= hx <= x2 and y1 <= hy <= y2:
                            is_grabbed = True
                            grabbed_hand = hand
                            break
                    
                    # 객체를 집었고, 입의 위치가 파악된 경우
                    if is_grabbed and mouth:
                        distance_to_mouth = self.calculate_distance(grabbed_hand, mouth)
                        
                        # 임계치(예: 100 픽셀) 이하로 거리가 좁혀지면 "삼킴 시도"로 간주
                        if distance_to_mouth < 100:
                            # 상태 격상: 입에 닿을 동작 감지!
                            current_robot_action = risk_info["mouth_action_cmd"]
                            warning_msg = f"🚨 비상! {class_name} 삼킴 위험 감지!"
                            color = (0, 0, 255) # 빨간색으로 변경
                            
                            # 시각화: 손과 입을 잇는 위험 경고선 그리기
                            cv2.line(frame, grabbed_hand, mouth, (0, 0, 255), 3)

                    # ROS 2 송신용 데이터 묶기
                    obj_data = {
                        "class": class_name,
                        "confidence": round(conf, 2),
                        "bounding_box": {"x1": x1, "y1": y1, "x2": x2, "y2": y2},
                        "center_coords": {"x": center_x, "y": center_y},
                        "risk_level": risk_info["level"],
                        "robot_action": current_robot_action, # REMOVE 또는 EMERGENCY_STOP
                        "message": warning_msg
                    }
                    
                    detected_data_list.append(obj_data)
                
                    # 화면에 Bounding Box 시각화
                    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                    cv2.putText(frame, f"{class_name} LV{risk_info['level']} {current_robot_action}", 
                                (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        # ----------------------------------------------------
        # [단계 4] ROS 2 Topic 발행 (Publish)
        # ----------------------------------------------------
        if detected_data_list:
            msg = String()
            msg.data = json.dumps(detected_data_list)
            self.publisher_.publish(msg)

        cv2.imshow('Capstone Vision Camera (YOLO + MediaPipe)', frame)
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