# test_image.py
import cv2
from ultralytics import YOLO

def test_on_image(image_path, model_path="best.pt"):
    """
    정적 이미지에 대한 모델 검증 함수 (디버깅/정량적 데이터 확인용)
    """
    print(f"\n🚀 [{image_path}] 이미지 검증을 시작합니다...")
    
    # 1. 모델 로드
    try:
        model = YOLO(model_path)
    except Exception as e:
        print(f"❌ 모델 로드 실패! 경로를 확인하세요: {e}")
        return

    # 2. 이미지 읽기
    img = cv2.imread(image_path)
    if img is None:
        print("❌ 이미지를 불러올 수 없습니다. 파일명이나 경로를 다시 확인해주세요.")
        return

    # 3. YOLOv11 추론 진행
    # 🚨 epoch 작아서 conf = 0.1인데, 신뢰성 있으려면 그래도 0.6 이상은 찍혀야 함
    results = model.predict(source=img, conf=0.1, save=False, verbose=False)

    detected = False

    # 4. 정량적 결과 분석 및 출력
    for result in results:
        boxes = result.boxes
        
        if len(boxes) == 0:
            continue
            
        detected = True
        print("\n[ 📊 모델 정량적 분석 결과 ]")
        
        for idx, box in enumerate(boxes):
            # 바운딩 박스 좌표
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            center_x = int((x1 + x2) / 2)
            center_y = int((y1 + y2) / 2)
            
            # 클래스 정보 및 신뢰도(확률) 추출
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])
            class_name = model.names[cls_id]

            # 콘솔에 상세 값 출력
            print("-" * 30)
            print(f"객체 #{idx+1} : {class_name}")
            print(f"▶ 확신도(확률) : {conf * 100:.2f}% (Raw: {conf:.4f})")
            print(f"▶ 바운딩 박스  : ({x1}, {y1}) ~ ({x2}, {y2})")
            print(f"▶ 중심 좌표    : ({center_x}, {center_y})")

            # 이미지 위에 시각화
            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.circle(img, (center_x, center_y), 4, (0, 0, 255), -1)
            cv2.putText(img, f"{class_name} {conf:.2f}", (x1, y1 - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # 아무것도 못 찾았을 경우의 피드백 (config = 0.1 일 때 가정)
    if not detected:
        print("\n[ 🚨 결과 ]")
        print("신뢰도 10% 이상의 객체를 하나도 찾지 못했습니다.")
        print("👉 원인: Epoch 10번으로는 학습이 턱없이 부족합니다. (최소 50~100 권장)")

    # 5. 결과 이미지 화면에 출력
    cv2.imshow("Image Validation Result", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    test_on_image(image_path="/home/joo/test_image_1.jpg", model_path="best.pt") # image_path 에 경로 쓰면 됨 (test 용 사진)