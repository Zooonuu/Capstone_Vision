# val_model.py
# 원본: 프로젝트 루트 val_model.py (팀원 작성)를 그대로 이식.
# ROS2 노드가 아니라 학습된 모델(best.pt)을 mAP 등으로 검증하는 개발 스크립트입니다.
# data.yaml / best.pt가 있는 위치(원래 프로젝트 루트)에서 실행하는 것을 전제로 합니다.
from ultralytics import YOLO
import multiprocessing


def main():
    print("🚀 학습된 최고 성능 모델(best.pt)의 검증을 시작합니다...")

    # 1. 학습이 완료된 모델 불러오기
    # 주의: 이 파일과 같은 폴더에 best.pt 파일이 있어야 합니다.
    model = YOLO('best.pt')

    # 2. 검증(Validation) 실행
    # 학습할 때 사용했던 data.yaml 파일을 기준
    # 이전 학습 때처럼 GPU가 없다면 device='cpu'를 설정하여 에러를 방지
    metrics = model.val(
        data='data.yaml',
        device='cpu'      # GPU 환경이 구축되었다면 '0'으로 변경
    )

    print("\n✅ 검증이 완료되었습니다! 터미널에 출력된 표(Table)를 확인해 주세요.")

    # 코드로 직접 수치를 뽑아보고 싶을 때
    print("\n[ 📊 핵심 점수 ]")
    print(f"▶ mAP50 (기본 정확도) : {metrics.box.map50 * 100:.2f}%")
    print(f"▶ mAP50-95 (엄격한 정확도) : {metrics.box.map * 100:.2f}%")


if __name__ == '__main__':
    multiprocessing.freeze_support()
    main()
