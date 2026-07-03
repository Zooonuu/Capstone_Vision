# train_model.py
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




'''
# train_model.py
from ultralytics import YOLO
import multiprocessing

def main():
    print("🚀 비전 모델 학습을 시작합니다...")

    # 1. 가볍고 빠른 YOLOv11 Nano 모델 불러오기
    model = YOLO('yolo11n.pt') 

    # ====================================================================
    # [현재 사용] 레고 테스트용 (데이터 적음, 빠른 테스트 목적)
    # ====================================================================
    results = model.train(
        data='data.yaml',       
        epochs=5,              
        imgsz=640,              
        batch=16,               
        device='cpu',           
        name='lego_test_model'  
    )

    # ====================================================================
    # [나중에 사용할 코드] 동전/건전지/레고 통합 최종 학습용
    # 나중에 데이터가 모이면 위 코드를 지우거나 주석처리하고, 아래 주석을 푸세요!
    # ====================================================================
    # results = model.train(
    #     data='final_data.yaml',       # TODO: 3가지 객체가 모두 들어간 새로운 yaml 파일명
    #     epochs=100,                   # 데이터가 많으므로 최소 100~300번은 학습해야 똑똑해집니다.
    #     imgsz=640,
    #     batch=16,                     # 메모리 부족(OOM 에러) 시 8 또는 4로 줄이세요.
    #     device='0',                   # TODO: 본격적인 100 epoch 학습은 무조건 GPU('0')를 써야 합니다! (CPU는 며칠 걸림)
    #     patience=20,                  # [추가 옵션] 20번 연속으로 성능 개선이 없으면 100번을 덜 채워도 조기 종료(과적합 방지)
    #     name='final_safety_model'     # 최종 모델이 저장될 폴더명 (runs/detect/final_safety_model/weights/best.pt)
    # )

    print("✅ 학습이 완료되었습니다!")
    print("결과물(가중치)은 runs/detect/ 폴더 안에 저장되었습니다.")

if __name__ == '__main__':
    multiprocessing.freeze_support()
    main()

'''