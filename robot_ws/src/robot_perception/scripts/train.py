# train.py
# 원본: 프로젝트 루트 train.py (팀원 작성)를 그대로 이식.
# ROS2 노드가 아니라 YOLO11 모델 학습용 개발 스크립트입니다.
# data.yaml이 있는 위치(원래 프로젝트 루트)에서 실행하는 것을 전제로 합니다.
from ultralytics import YOLO
import multiprocessing


def main():
    print("🚀 비전 모델(레고 테스트 버전) 학습을 시작합니다...")

    # 1. 가볍고 빠른 YOLOv11 Nano 모델 불러오기
    model = YOLO('yolo11n.pt')

    # 2. 모델 학습 시작
    # batch: PC 메모리가 부족해서 에러가 나면 16 -> 8 -> 4 로 줄이기
    # epochs: 전체 데이터를 반복 학습할 횟수. (최소테스트 epoch 100)
    results = model.train(
        data='data.yaml',       # 방금 수정한 yaml 파일 (동전/건전지 생기면 수정 예정)
        epochs=5,              # 학습 반복 횟수, 허준우 노트북에서는 10까지만 해봄 (잘 안됨) epoch 100 필요
        imgsz=640,              # 이미지 크기
        batch=16,               # 한 번에 처리할 이미지 수
        device='cpu',           # GPU가 없다면 'cpu'로 변경 (중요!) / gpu있으면 0으로 두면 됨
        name='lego_test_model'  # 결과가 저장될 폴더 이름
    )

    print("✅ 학습이 완료되었습니다!")
    print("결과물(가중치)은 runs/detect/lego_test_model/weights/best.pt 에 저장되었습니다.")


if __name__ == '__main__':
    # 멀티프로세싱 충돌 방지
    multiprocessing.freeze_support()
    main()
