# Capstone_Vision 비전팀 Task 정리

작성 기준: 2026-07-04

## 목적

이 문서는 README를 수정하지 않고, 비전팀과 ROS2팀이 앞으로 무엇을 어떤 순서로 해야 하는지 정리한 공유용 md입니다.
현재 코드는 YOLO 탐지, 위험도 DB, class alias, 시간축 안정화, risk_score, reason_codes, ROS2 Detection 메시지 확장까지 들어간 상태입니다.


## 1. 앞으로 해야 할 일: 순서별 Task

### [Phase 0] 코드 이해와 실행 환경 확인
Task 0-1. 전체 실행 흐름 이해
- 담당: 비전팀 전체
- 볼 파일: README.md, howtorun.md, robot_ws/src/robot_perception/robot_perception/yolo_detector_node.py
- 산출물: 팀원이 설명할 수 있는 1페이지 흐름도
- 완료 기준:
  /image_raw -> yolo_detector_node -> /detections, vision/detected_objects 흐름을 말로 설명할 수 있어야 함.

Task 0-2. 빌드와 기본 검증 재현
- 담당: ROS2팀 + 비전팀 1명
- 명령:
  cd /home/joo/Desktop/Capstone_Vision/robot_ws
  source /opt/ros/jazzy/setup.bash
  colcon build --packages-select robot_perception_msgs robot_perception
- 산출물: 빌드 성공 로그 캡처
- 완료 기준:
  robot_perception_msgs, robot_perception 둘 다 빌드 성공.

Task 0-3. 현재 모델과 데이터셋 상태 정리
- 담당: 비전팀
- 볼 파일: data.yaml, reports/validation/validation_report.md
- 산출물: 현재 모델 class 목록, 현재 점수, 한계점 정리
- 완료 기준:
  현재 data.yaml은 레고 세부 class 23개이며, 코드에서는 이를 lego 위험 class로 alias 처리한다는 점을 팀 전체가 공유.


### [Phase 1] 모델 class와 위험 class 정합성 확정
Task 1-1. 최종 class 정책 결정
- 담당: 비전팀 리드
- 선택지:
  A. 모델 class를 battery, coin, lego 3개 상위 class로 통일
  B. 모델은 세부 class를 유지하고, risk_db.py의 DEFAULT_CLASS_ALIASES로 상위 위험 class에 매핑
- 추천:
  현재 코드 구조에서는 B가 자연스럽지만, 최종 발표용 모델은 A가 더 설명하기 쉬움.
- 산출물: 최종 class 정책 표

Task 1-2. battery, coin 포함 여부 확인
- 담당: 비전팀
- 이유:
  현재 data.yaml은 레고 세부 class 중심이므로, 최종 시연에서 동전/배터리가 실제로 검출되는지 확인해야 함.
- 산출물:
  battery/coin/lego 각각의 테스트 이미지 또는 테스트 영상 결과표
- 완료 기준:
  세 class 모두 best.pt에서 검출 가능한지, 아니면 추가 학습이 필요한지 명확히 결론.

Task 1-3. alias/risk_db/yolo_params 동기화
- 담당: 비전팀
- 볼 파일:
  risk_db.py
  robot_ws/src/robot_perception/robot_perception/risk_db.py
  robot_ws/src/robot_perception/config/yolo_params.yaml
- 산출물: class alias 표
- 완료 기준:
  모델이 내보내는 class_name이 risk_class_name으로 어떻게 바뀌는지 전부 표로 정리.


### [Phase 2] 실시간 카메라 입력과 ROS2 토픽 안정화
Task 2-1. 실제 카메라가 /image_raw를 발행하는지 확인
- 담당: ROS2팀
- 비전팀 협업:
  yolo_detector_node가 받을 이미지 해상도, FPS, 색상 encoding 확인
- 산출물:
  ros2 topic list, ros2 topic hz /image_raw, 샘플 frame 캡처
- 완료 기준:
  /image_raw가 끊기지 않고 들어오며, yolo_detector_node가 frame을 정상 수신.

Task 2-2. /detections 메시지 계약 확인
- 담당: ROS2팀 + 비전팀
- 볼 파일:
  robot_ws/src/robot_perception_msgs/msg/Detection.msg
  robot_ws/src/robot_perception_msgs/msg/DetectionArray.msg
- 확인할 필드:
  class_name, confidence, bbox, risk_class_name, risk_level, risk_score, robot_action, reason_codes
- 산출물:
  /detections 샘플 메시지 3개 이상
- 완료 기준:
  ROS2팀이 JSON 파싱 없이 /detections만 보고 위험물 목표를 고를 수 있어야 함.

Task 2-3. headless 실행 옵션 정리
- 담당: ROS2팀
- 볼 파일: robot_ws/src/robot_perception/config/yolo_params.yaml
- 확인 파라미터:
  enable_visualization: false
  device: "cpu" 또는 "0"
  inference_rate_hz
- 완료 기준:
  로봇 본체에서 화면 없이 실행 가능.


### [Phase 3] 위험 판단 threshold 튜닝
Task 3-1. confidence_threshold 튜닝
- 담당: 비전팀
- 볼 파일: yolo_params.yaml, val_model.py, reports/validation/validation_report.md
- 방법:
  confidence_threshold를 0.2, 0.3, 0.4, 0.5로 바꿔서 오탐/미탐 비교
- 산출물:
  threshold별 Precision, Recall, 실제 영상 오탐 수, 미탐 수
- 완료 기준:
  최종 시연용 threshold 1개 확정.

Task 3-2. small-object 기준 튜닝
- 담당: 비전팀
- 파라미터:
  small_object_reference_area_scale
  small_object_fallback_max_area_ratio
- 이유:
  unknown_small_object가 너무 많이 나오면 오탐이 늘고, 너무 적게 나오면 삼킴 위험 후보를 놓침.
- 산출물:
  bbox area ratio 기준표

Task 3-3. 입 근접 기준 튜닝
- 담당: 비전팀 + ROS2팀
- 파라미터:
  mouth_threshold_px
  mouth_threshold_scale
  mouth_min_threshold_px
  mouth_max_threshold_px
- 완료 기준:
  손/물체가 실제로 입 근처에 있을 때만 EMERGENCY_STOP이 발생하도록 조정.

Task 3-4. 시간축 안정화 기준 튜닝
- 담당: 비전팀
- 파라미터:
  stable_detection_frames
  lost_track_ttl_frames
  track_iou_threshold
- 목표:
  한 프레임 오탐은 NONE으로 유지하고, 3프레임 이상 안정적으로 잡힌 객체만 REMOVE로 확정.
- 산출물:
  안정화 전후 오탐 비교표


### [Phase 4] 안전 평가 리포트 강화
Task 4-1. val_model.py 검증 리포트 최신화
- 담당: 비전팀
- 명령:
  python3 val_model.py
- 산출물:
  reports/validation/validation_report.md
  reports/validation/validation_report.json
- 완료 기준:
  mAP/Precision/Recall뿐 아니라 Safety Mapping, Safety Evaluation Protocol이 포함되어야 함.

Task 4-2. 안전 판단 평가표 작성
- 담당: 비전팀
- 평가 항목:
  risk_class_mapping_accuracy
  stable_detection_before_remove
  emergency_stop_false_negative_rate
  emergency_stop_false_positive_rate
  top_priority_target_accuracy
- 산출물:
  safety_eval_result.md 또는 safety_eval_result.xlsx

Task 4-3. 시나리오별 테스트 영상 만들기
- 담당: 비전팀
- 시나리오:
  1. 레고만 바닥에 있음
  2. 동전만 바닥에 있음
  3. 배터리만 바닥에 있음
  4. 여러 물체가 동시에 있음
  5. 손이 물체를 집음
  6. 물체가 입 근처로 이동
  7. 오탐이 나오기 쉬운 바닥/조명
- 완료 기준:
  각 시나리오에서 expected robot_action과 실제 robot_action을 비교.


### [Phase 5] 좌표 변환과 로봇 행동 연결
Task 5-1. bbox -> 지면 좌표 변환 설계
- 담당: ROS2팀 주도, 비전팀 협업
- 필요한 값:
  camera intrinsic
  camera extrinsic
  camera_link -> base_link TF
  bbox bottom-center 또는 center point
- 산출물:
  ground_projection_node 설계서
- 완료 기준:
  /detections의 bbox가 base_link 또는 map 좌표의 목표점으로 변환됨.

Task 5-2. target_manager_node 구현
- 담당: ROS2팀
- 입력:
  /detections 또는 지면 좌표로 변환된 /safety_targets
- 판단 기준:
  risk_score 높은 순
  robot_action이 REMOVE 또는 EMERGENCY_STOP인 객체 우선
  이미 처리한 target 중복 제거
- 완료 기준:
  여러 위험물 중 로봇이 먼저 갈 target 1개를 안정적으로 선택.

Task 5-3. 알람/긴급정지 인터페이스 연결
- 담당: ROS2팀
- 비전팀 협업:
  어떤 reason_codes에서 알람을 울릴지 기준 제공
- 완료 기준:
  EMERGENCY_STOP 발생 시 부저, LED, 로그, 앱 알림 중 최소 1개 이상 작동.


### [Phase 6] 최종 시연 패키징
Task 6-1. 최종 launch 파일 구성
- 담당: ROS2팀
- 포함 노드:
  camera node
  yolo_detector_node
  slam/localization
  ground_projection_node
  target_manager_node
  alarm/actuator node

Task 6-2. 발표용 데모 시나리오 고정
- 담당: 비전팀 + ROS2팀
- 추천 시나리오:
  1. 바닥의 레고를 탐지하고 REMOVE 후보로 지정
  2. 여러 물체 중 배터리를 가장 높은 risk_score로 선택
  3. 손/입 근접 상황에서 EMERGENCY_STOP 발생
  4. reason_codes를 보여주며 왜 위험 판단이 났는지 설명

Task 6-3. 최종 산출물 정리
- 담당: 비전팀
- 산출물:
  best.pt
  data.yaml
  validation_report.md
  safety_eval_result
  class alias 표
  threshold 튜닝표
  오탐/미탐 사례 정리


## 2. 처음 보는 사람에게 공유할 학습용 코드 목록

[입문 순서 1] README.md
- 목적:
  프로젝트 전체 목표, 폴더 구조, 비전/ROS2 역할 분담 이해.
- 공부 포인트:
  카메라 -> YOLO -> 위험도 판단 -> ROS2 메시지 -> SLAM/주행 연결 흐름.

[입문 순서 2] data.yaml
- 목적:
  현재 모델이 어떤 class를 학습/검증하는지 확인.
- 공부 포인트:
  현재는 레고 세부 class 23개이며, 코드에서 이를 lego 위험 class로 묶는다는 점.

[입문 순서 3] risk_db.py
- 목적:
  위험도 DB와 class alias 이해.
- 공부 포인트:
  battery는 Level 3, coin/lego는 Level 2, unknown_small_object는 Level 2, unknown_large_object는 Level 1.
  DEFAULT_CLASS_ALIASES가 모델 class를 위험 class로 변환하는 역할을 함.

[입문 순서 4] robot_ws/src/robot_perception/config/yolo_params.yaml
- 목적:
  실행 중 바꿀 수 있는 파라미터 이해.
- 공부 포인트:
  confidence_threshold, class_aliases, stable_detection_frames, mouth threshold, unknown-object 기준.

[입문 순서 5] robot_ws/src/robot_perception_msgs/msg/Detection.msg
- 목적:
  비전팀 결과가 ROS2팀으로 전달되는 형식 이해.
- 공부 포인트:
  bbox뿐 아니라 risk_class_name, risk_level, risk_score, robot_action, reason_codes가 포함됨.

[입문 순서 6] robot_ws/src/robot_perception/robot_perception/yolo_detector_node.py
- 목적:
  실제 주력 비전 노드 이해.
- 공부 순서:
  1. __init__에서 ROS2 parameter와 publisher/subscriber 확인
  2. _risk_class_for_model_class로 class alias 확인
  3. _unknown_object_risk_class로 DB 밖 객체 분류 확인
  4. _update_track으로 시간축 안정화 확인
  5. _risk_score로 우선순위 점수 계산 확인
  6. _image_callback에서 YOLO, MediaPipe, risk_db, DetectionArray 발행 흐름 확인

[입문 순서 7] val_model.py
- 목적:
  모델 검증과 리포트 생성 이해.
- 공부 포인트:
  Precision, Recall, mAP50, mAP50-95와 Safety Mapping, Safety Evaluation Protocol이 어떻게 report로 저장되는지 확인.

[입문 순서 8] test_image.py
- 목적:
  ROS2 없이 정적 이미지 1장으로 YOLO 추론을 확인.
- 공부 포인트:
  class_name, confidence, bbox, center 좌표가 어떻게 나오는지 확인.

[입문 순서 9] vision_ros2_node.py
- 목적:
  초기 버전 웹캠 테스트 노드 이해.
- 주의:
  현재 주력 실행 코드는 robot_ws 안의 yolo_detector_node.py임.
  이 파일은 비교/학습용으로 보면 좋음.

[입문 순서 10] robot_ws/src/robot_slam/README.md 및 launch/config 파일
- 목적:
  비전 결과가 실제 로봇 좌표계와 어떻게 연결될지 이해.
- 공부 포인트:
  map, odom, base_link, camera_link, laser_link의 TF 흐름.


## 3. 더 업그레이드할 수 있는 부분과 비전팀 Task

### [업그레이드 A] 실제 크기 기반 위험도 판단
현재 한계:
  bbox 면적은 카메라와 물체 거리의 영향을 크게 받음.
업그레이드 방향:
  카메라 보정과 지면 투영을 이용해 물체의 실제 크기 cm를 근사.
비전팀 Task:
  A-1. 같은 물체를 거리별로 촬영해 bbox area 변화표 작성
  A-2. 카메라 높이/각도별 small-object threshold 실험
  A-3. ROS2팀 ground_projection_node가 나오면 bbox 실제 크기 추정 로직 검증

### [업그레이드 B] 모델 class 통합과 최종 안전 모델
현재 한계:
  레고 세부 class 중심 모델과 battery/coin/lego 위험도 DB 사이를 alias로 연결 중.
업그레이드 방향:
  최종 모델은 battery, coin, lego를 직접 검출하거나, 세부 class를 유지하되 alias 표를 완성.
비전팀 Task:
  B-1. 최종 best.pt가 battery/coin/lego를 실제로 검출하는지 검증
  B-2. class별 confusion matrix 확인
  B-3. class별 confidence threshold 분리 적용 검토

### [업그레이드 C] 오탐/미탐 리플레이 시스템
현재 한계:
  실시간 화면에서만 보면 어떤 상황에서 틀렸는지 나중에 분석하기 어려움.
업그레이드 방향:
  탐지 결과, frame, reason_codes, robot_action을 로그로 저장.
비전팀 Task:
  C-1. EMERGENCY_STOP 또는 REMOVE 발생 시 frame snapshot 저장
  C-2. JSONL 형식으로 timestamp, class, risk_score, reason_codes 저장
  C-3. 오탐/미탐 사례를 safety_eval_result에 누적

### [업그레이드 D] 안전 판단 단위 테스트
현재 한계:
  YOLO 없이 판단 로직만 따로 검증하기 어려움.
업그레이드 방향:
  bbox, class_name, confidence, mouth/hands 좌표를 가짜 입력으로 넣어 risk_score와 robot_action을 테스트.
비전팀 Task:
  D-1. class alias 테스트
  D-2. unknown_small_object 테스트
  D-3. stable_detection_frames 테스트
  D-4. mouth proximity -> EMERGENCY_STOP 테스트

### [업그레이드 E] MediaPipe 사용 조건 검증
현재 한계:
  로봇 카메라가 바닥을 향하면 아이 입/손 landmark가 안정적으로 안 잡힐 수 있음.
업그레이드 방향:
  바닥 탐지 카메라와 아이 관찰 카메라를 분리할지 결정.
비전팀 Task:
  E-1. 실제 장착 높이/각도에서 입/손 landmark 검출률 측정
  E-2. enable_pose_risk_scoring true/false 비교
  E-3. landmark가 불안정하면 입-손 근접 기능을 별도 카메라 또는 별도 노드로 분리 제안

### [업그레이드 F] 속도 최적화
현재 한계:
  CPU 실행 시 FPS가 부족할 수 있음.
업그레이드 방향:
  device, inference_rate_hz, 모델 크기, export 형식을 비교.
비전팀 Task:
  F-1. CPU/GPU별 FPS 측정
  F-2. YOLO11n, YOLO11s 성능/속도 비교
  F-3. enable_visualization false일 때 latency 측정
  F-4. 필요 시 ONNX/TensorRT/OpenVINO export 검토

### [업그레이드 G] 발표 설득력 강화
현재 한계:
  mAP만 보여주면 안전 로봇의 판단 능력을 충분히 설명하기 어려움.
업그레이드 방향:
  "모델 정확도"와 "안전 행동 정확도"를 따로 평가.
비전팀 Task:
  G-1. mAP/Precision/Recall 표
  G-2. risk_score 1순위 목표 선택 정확도 표
  G-3. EMERGENCY_STOP 오탐/미탐 표
  G-4. reason_codes 예시 캡처
  G-5. 안정화 전/후 오탐 비교표


## 추천 업무 분담 요약

비전팀 우선순위 TOP 5
1. battery/coin/lego 최종 모델 검출 가능 여부 확인
2. class alias/risk_db/yolo_params 정합성 표 작성
3. confidence, small-object, mouth, stable detection threshold 튜닝
4. safety_eval_result 작성
5. 오탐/미탐 사례와 reason_codes 예시 정리

ROS2팀 우선순위 TOP 5
1. /image_raw 카메라 입력 안정화
2. /detections 구독 확인
3. bbox -> 지면 좌표 변환 ground_projection_node 설계
4. risk_score 기반 target_manager_node 구현
5. EMERGENCY_STOP/REMOVE를 알람, 수거 장치, 주행 제어와 연결

공동 우선순위 TOP 5
1. 실제 카메라 높이와 각도 결정
2. MediaPipe 입/손 검출률 실측
3. 여러 위험물 동시 등장 시 우선순위 기준 합의
4. 최종 시연 시나리오 3개 고정
5. 발표용 지표와 영상 캡처 정리
