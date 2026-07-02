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



'''
# val_test.py
from ultralytics import YOLO
import multiprocessing

def main():
    print("🚀 학습된 모델의 검증을 시작합니다...")

    # ====================================================================
    # [현재 사용] 레고 테스트 모델 불러오기
    # ====================================================================
    model = YOLO('best.pt') 
    
    metrics = model.val(
        data='data.yaml', 
        device='cpu'      
    )

    # ====================================================================
    # [나중에 사용할 코드] 동전/건전지/레고 통합 최종 검증용
    # 나중에 최종 학습이 끝나면 위 코드를 주석처리하고 아래 주석을 푸세요!
    # ====================================================================
    # # 주의: 학습이 끝난 후 runs/detect/final_safety_model/weights/best.pt 를 가져와야 합니다.
    # model = YOLO('final_best.pt') 
    #
    # metrics = model.val(
    #     data='final_data.yaml', # 3가지 객체가 모두 정의된 yaml
    #     device='cpu'            # 검증은 금방 끝나므로 GPU가 없다면 CPU도 무방합니다.
    # )

    print("\n✅ 검증이 완료되었습니다!")
    
    # --------------------------------------------------------------------
    # 🌟 캡스톤 방어 발표용 핵심 지표 출력 (Precision, Recall 추가)
    # --------------------------------------------------------------------
    print("\n[ 📊 전체 객체에 대한 핵심 점수 종합 ]")
    print(f"▶ mAP50 (기본 종합 정확도) : {metrics.box.map50 * 100:.2f}%")
    print(f"▶ mAP50-95 (엄격한 정확도) : {metrics.box.map * 100:.2f}%")
    
    # 새로 추가된 핵심 지표
    print(f"▶ Precision (정밀도)       : {metrics.box.mp * 100:.2f}%")
    print(f"▶ Recall (재현율)          : {metrics.box.mr * 100:.2f}%")

    print("\n[ 💡 각 클래스(물체)별 상세 성적표 확인 방법 ]")
    print("터미널에 출력된 표를 확인하거나, 'runs/detect/val/confusion_matrix.png' 파일을 꼭 확인하세요!")
    print("동전/건전지/레고가 추가되면, 모델이 특정 물체를 다른 물체로 헷갈려하지 않는지 분석해야 합니다.")

if __name__ == '__main__':
    multiprocessing.freeze_support()
    main()

'''