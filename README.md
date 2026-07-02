# Capstone_Vision (Ver1.) (7.1 ~ 7.7)

# 26.7.1 업데이트

1. 훈련모델 완료 / test_image로 다른 이미지로 검증 가능 (epoch 너무 낮아서 사실 안 뜸, config 값 0.1로 일단 진행, GPU value = 0으로 두고 epoch 크게 늘려서 실행 필요)
2. 이후 웹켐으로도 인식이 되는 지 확인 (vision_ros2_node.py) (현재 허준우 우분투 환경에서 노트북캠 연결 안되는 문제 발생, 실제 웹캠 연결해서 실행 필요)

# 26.7.2 업데이트

1. 훈련모델 검증 val_model.py 구성 (mAP)
2. vision_ros2_node.py 업데이트 - 
비전 파트에서 JSON Array 형태로 객체 리스트를 쏠 거야. 여기서 제일 중요한 건, 우리가 이미 내부적으로 가장 위험하고 시급한 물체를 0번 인덱스(parsed_data[0])로 정렬 / 여러 물체가 들어와도 무조건 배열의 첫 번째 데이터(data[0])만 뽑아서 그 안의 robot_action이 REMOVE면 center_coords로 주행을 시키고, EMERGENCY_STOP이나 ALARM이 뜨면 즉각 주행 취소 및 스피커 모듈
3. vision_ros2_node.py (좀 더 직관적인 버전 - 조건문 넣어서 조금 더 직관적이라고는 하는데 잘 모르겠음) 
4. 나중에 바꿀 수 있는 yaml 

