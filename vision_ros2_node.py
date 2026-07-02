# vision_ros2_node.py
import cv2
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ultralytics import YOLO

# 앞서 만든 DB 파일 임포트
from risk_db import RISK_DATABASE

class VisionDetectionNode(Node):
    def __init__(self):
        super().__init__('vision_detection_node')
        
        # ROS 2 Publisher 설정 ('vision/detected_objects' 라는 이름으로 데이터 송신)
        self.publisher_ = self.create_publisher(String, 'vision/detected_objects', 10)
        
        # YOLOv11 모델 로드 (Roboflow 학습 완료된 모델)
        self.get_logger().info('YOLO 모델을 로드하는 중...')
        self.model = YOLO('best.pt') 
        
        # 웹캠 연결
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error('웹캠을 열 수 없습니다!')
        
        # 타이머 설정 (초당 약 30프레임 처리 위해 0.033초 주기)
        timer_period = 0.033 
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('비전 감지 노드가 성공적으로 시작되었습니다.')

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning('프레임을 읽을 수 없습니다.')
            return

        # 1. 모델 추론 (conf 임계값을 주어 너무 낮은 신뢰도 객체는 무시)
        results = self.model.predict(source=frame, conf=0.6, verbose=False)
        
        detected_data_list = []

        # 2. 결과 파싱
        for result in results:
            for box in result.boxes:
                # 좌표 및 클래스 추출
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                center_x = int((x1 + x2) / 2)
                center_y = int((y1 + y2) / 2)
                
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                class_name = self.model.names[cls_id].lower() # 소문자로 변환하여 DB와 매칭

                # 3. 위험도 DB에서 해당 객체의 액션 플랜 가져오기
                risk_info = RISK_DATABASE.get(class_name, None)
                
                if risk_info:
                    # ROS 2 팀원에게 넘겨줄 데이터 포맷 (JSON화 하기 위해 딕셔너리 구성)
                    obj_data = {
                        "class": class_name,
                        "confidence": round(conf, 2),
                        "bounding_box": {"x1": x1, "y1": y1, "x2": x2, "y2": y2},
                        "center_coords": {"x": center_x, "y": center_y},
                        "risk_level": risk_info["level"],
                        "robot_action": risk_info["robot_action_cmd"],
                    }
                    detected_data_list.append(obj_data)
                
                # 화면에 시각화 (디버깅용)
                color = (0, 0, 255) if risk_info and risk_info["level"] == 3 else (0, 255, 255)
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                cv2.putText(frame, f"{class_name} LV{risk_info['level'] if risk_info else '?'}", 
                            (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        # 4. 감지된 객체가 있다면 ROS 2 Topic으로 발행 (Publish)
        if detected_data_list:
            msg = String()
            # 딕셔너리 리스트를 JSON 문자열로 변환하여 송신
            msg.data = json.dumps(detected_data_list)
            self.publisher_.publish(msg)
            # 확인용 로그 출력
            # self.get_logger().info(f'Published: {msg.data}')

        # 5. 영상 출력
        cv2.imshow('Capstone Vision Camera', frame)
        cv2.waitKey(1)

    def destroy_node(self):
        """노드 종료 시 메모리 정리"""
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    vision_node = VisionDetectionNode()
    
    try:
        # 노드를 실행 상태로 유지 (계속해서 웹캠을 읽고 publish 함)
        rclpy.spin(vision_node)
    except KeyboardInterrupt:
        vision_node.get_logger().info('비전 감지 노드가 강제 종료되었습니다.')
    finally:
        vision_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()