# test_image.py
# 원본: 프로젝트 루트 test_image.py (팀원 작성)를 그대로 이식.
# ROS2 노드가 아니라 정적 이미지로 학습된 모델을 빠르게 확인하는 개발용 스크립트입니다.
# data.yaml / best.pt가 있는 위치(원래 프로젝트 루트)에서 실행하는 것을 전제로 합니다.
#
# 사용법:
#   python3 test_image.py <이미지_경로> [모델_경로]
import sys

import cv2
from ultralytics import YOLO

__test__ = False


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
    # 원본은 팀원 개인 PC 경로(/home/joo/test_image_1.jpg)가 하드코딩되어 있었습니다.
    # 다른 환경에서도 바로 쓸 수 있도록 커맨드라인 인자로 바꿨습니다 (로직/출력 포맷은 원본 그대로).
    if len(sys.argv) < 2:
        print("사용법: python3 test_image.py <이미지_경로> [모델_경로=best.pt]")
        sys.exit(1)
    image_path_arg = sys.argv[1]
    model_path_arg = sys.argv[2] if len(sys.argv) > 2 else "best.pt"
    test_on_image(image_path=image_path_arg, model_path=model_path_arg)
