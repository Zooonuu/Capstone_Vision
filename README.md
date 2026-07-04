# Capstone_Vision

2026 창의융합 캡스톤디자인 팀 **막아줘 영(Young)** 의 비전/ROS2 작업 공간입니다.

기준일: **2026.07.04**

## 프로젝트 목표

계획서 기준 최종 목표는 **영유아 삼킴 사고 방지를 위한 모바일 로봇**입니다.
로봇이 실내를 자율주행하면서 바닥의 소형 위험물을 탐지하고, 위험도에 따라 보호자에게
알림을 보내거나 물체를 사전에 제거해 사고 자체를 막는 것이 핵심입니다.

계획서의 핵심 흐름은 다음과 같습니다.

```text
카메라로 바닥 객체 탐지
-> 위험도 DB와 bbox 크기 조건으로 위험도 판단
-> LiDAR/엔코더 기반 SLAM으로 로봇 위치 추정
-> 위험물 위치를 지도/로봇 좌표로 변환
-> Nav2 또는 제어 로직으로 위험물까지 이동
-> 스위핑/흡입/격리 수납함으로 위험물 제거
-> 아이의 손-물체-입 상호작용이 감지되면 즉시 알람
```

## 현재 코드 구조

```text
Capstone_Vision/
├─ data.yaml
├─ test_image.py
├─ val_model.py
├─ vision_ros2_node.py
├─ risk_db.py
├─ howtorun.md
└─ robot_ws/
   └─ src/
      ├─ robot_perception/
      ├─ robot_perception_msgs/
      └─ robot_slam/
```

### 루트 코드

루트의 Python 파일들은 ROS2 워크스페이스에 편입되기 전의 개발/검증용 코드입니다.

| 파일 | 역할 |
|---|---|
| `data.yaml` | YOLO 학습/검증 데이터 설정. 현재는 레고 세부 클래스 23개 기준입니다. |
| `test_image.py` | 정적 이미지 1장을 YOLO로 테스트하고 bbox/confidence를 확인합니다. |
| `val_model.py` | `best.pt`를 `data.yaml` 기준으로 검증해 mAP/Precision/Recall과 `reports/validation` 리포트를 확인합니다. |
| `risk_db.py` | 원본 위험도 DB입니다. ROS2 패키지 안에도 같은 목적의 DB가 있습니다. |
| `vision_ros2_node.py` | 초기 ROS2 스타일 웹캠 테스트 노드입니다. 현재 주력 실행 코드는 `robot_ws` 안의 `yolo_detector_node.py`입니다. |

### ROS2 워크스페이스

`robot_ws/`는 `colcon build` 대상인 ROS2 워크스페이스입니다.

| 패키지 | 역할 |
|---|---|
| `robot_perception` | 카메라 이미지를 받아 YOLO/MediaPipe 기반 위험도 판단을 수행하는 Python 패키지 |
| `robot_perception_msgs` | 비전 결과를 다른 ROS2 노드가 쓸 수 있게 만든 커스텀 메시지 패키지 |
| `robot_slam` | LiDAR/엔코더 기반 SLAM, localization, TF tree launch/config 패키지 |

## 현재 구현된 기능

### 1. YOLO 객체 탐지

`robot_ws/src/robot_perception/robot_perception/yolo_detector_node.py`

입력:

```text
/image_raw  (sensor_msgs/Image)
```

출력:

```text
/detections              (robot_perception_msgs/DetectionArray)
vision/detected_objects  (std_msgs/String, JSON payload)
```

현재 노드는 `/image_raw`를 받아 OpenCV 이미지로 변환하고, YOLO 모델(`best.pt`)로 객체를 탐지합니다.
탐지 결과의 bbox, 위험도, 위험 점수, 로봇 행동 명령, 판단 근거는 `/detections`로 구조화되어
발행되고, 기존 코드와의 호환을 위해 같은 내용을 `vision/detected_objects` JSON으로도 발행합니다.

`Detection.msg` 현재 필드:

```text
class_id
class_name
confidence
bbox
risk_class_name
risk_level
risk_score
robot_action
reason_codes
```

### 1-1. 모델 class와 위험 class 매핑

현재 `data.yaml`은 레고 세부 클래스 23개를 사용합니다. 안전 판단에서는 이 세부 클래스를
모두 상위 위험 클래스 `lego`로 변환합니다.

```text
C1x1x1, C2x1x0, ... Z1x1x1 -> lego
```

이 매핑은 `risk_db.py`의 `DEFAULT_CLASS_ALIASES`와
`robot_ws/src/robot_perception/config/yolo_params.yaml`의 `class_aliases`에 정리되어 있습니다.
따라서 모델은 레고 모양을 세밀하게 구분하더라도, 로봇 행동 판단은
`battery`/`coin`/`lego`라는 발표용 상위 위험 class 기준으로 통일됩니다.

### 2. 위험도 DB

`robot_ws/src/robot_perception/robot_perception/risk_db.py`

현재 위험도는 다음과 같습니다.

| class | level | 일반 action | 입 근접 action | 의미 |
|---|---:|---|---|---|
| `battery` | 3 | `REMOVE` | `EMERGENCY_STOP` | 고위험. 즉시 제거/긴급 대응 |
| `coin` | 2 | `REMOVE` | `EMERGENCY_STOP` | 주의. 수거 필요. 입 근접 시 즉시 정지 |
| `lego` | 2 | `REMOVE` | `EMERGENCY_STOP` | 주의. 수거 필요. 입 근접 시 즉시 정지 |
| `unknown_small_object` | 2 | `REMOVE` | `EMERGENCY_STOP` | 기존 소형 위험물 bbox보다 작은 미확인 물체. 삼킴 가능성으로 주의 처리 |
| `unknown_large_object` | 1 | `NONE` | `NONE` | 입 크기보다 큰 미확인 물체. 별도 action 없이 인식만 수행 |

2026.07.04 기준으로 `battery/coin/lego` 같은 DB 등록 물체는 기존 위험도를 그대로 사용합니다.
모델 class가 alias로 상위 위험 class에 매핑되면 해당 상위 class의 위험도를 사용하고,
DB에 없는 미확인 객체만 bbox 크기로 추가 분류합니다.

### 3. DB 밖 미확인 물체 bbox 분류

아이의 입에 들어갈 수 있는 물체가 동전/배터리/레고만은 아니므로, 현재 노드는 다음 기준을 사용합니다.

```text
1. 모델 class를 먼저 상위 위험 class로 alias 매핑
2. battery/coin/lego는 risk_db의 기존 위험도대로 처리
3. risk_db에 없는 클래스도 일단 bbox 면적을 계산
4. bbox가 같은 프레임의 battery/coin/lego 기준 bbox보다 작거나 같으면 unknown_small_object, Level 2, REMOVE
5. 같은 프레임에 기준 객체가 없으면 이미지 전체 면적 대비 small-object fallback ratio로 Level 2 여부 판단
6. Level 2 기준에 걸리지 않은 미확인 객체 중 bbox가 아이 입 크기보다 크면 unknown_large_object, Level 1, NONE
7. MediaPipe가 입을 못 잡으면 이미지 전체 면적 대비 mouth fallback ratio를 임시 입 크기로 사용
8. 위 두 조건에 모두 걸리지 않은 DB 밖 객체는 위험도 DB 대상으로 발행하지 않음
```

관련 파라미터:

```yaml
enable_unknown_object_risk: true
unknown_small_object_class_name: "unknown_small_object"
unknown_large_object_class_name: "unknown_large_object"
small_object_reference_classes: ["battery", "coin", "lego"]
small_object_reference_area_scale: 1.0
small_object_fallback_max_area_ratio: 0.035
mouth_size_area_scale: 1.0
mouth_fallback_area_ratio: 0.035
class_aliases:
  - "C1x1x1:lego"
  - "..."
```

주의: 이 로직은 **YOLO가 bbox를 만든 객체**에 대해서만 작동합니다. YOLO가 아예 검출하지 못한 물체는
bbox가 없으므로 크기 비교도 할 수 없습니다.

### 4. MediaPipe 기반 손-입 근접 위험 판단

`yolo_detector_node.py`는 MediaPipe Pose로 손끝과 입 중심 좌표를 추출합니다.

위험 상황 판단:

```text
조건 A: 손 좌표가 위험물 bbox 안에 있음
조건 B: 그 손이 입 좌표와 가까워짐
조건 C: 또는 물체 중심이 입 좌표와 가까워짐
```

조건이 만족되면:

```text
risk_score = class_risk + confidence + track_stability + hand_overlap + mouth_proximity
robot_action = EMERGENCY_STOP
```

Level 2 이상의 모든 DB 물체는 입 근처에 있으면 `EMERGENCY_STOP`으로 바뀝니다.
Level 1인 `unknown_large_object`는 입 근접 상황에서도 action 없이 인식만 유지합니다.
입 근접 거리 기준은 고정 픽셀값만 쓰지 않고, MediaPipe가 입 너비를 잡으면
`mouth_width * mouth_threshold_scale`을 사용한 뒤 최소/최대 픽셀값으로 제한합니다.

### 4-1. 시간축 안정화와 위험 점수

오탐을 줄이기 위해 같은 위험물이 여러 프레임에서 반복 검출되는지 IoU 기반으로 추적합니다.

```text
stable_detection_frames: 3
lost_track_ttl_frames: 5
track_iou_threshold: 0.3
```

일반 수거 명령(`REMOVE`)은 같은 물체가 `stable_detection_frames`만큼 확인된 뒤 확정합니다.
단, 아이의 입/손 근접으로 판단된 `EMERGENCY_STOP`은 안전을 위해 안정화 대기 없이 즉시 발행합니다.

각 탐지 결과에는 판단 근거가 `reason_codes`로 함께 들어갑니다.

예시:

```text
CLASS_ALIAS:C2x1x1->lego
RISK_LEVEL_2
HAND_INSIDE_BBOX
OBJECT_NEAR_MOUTH
STABLE_TRACK_3_FRAMES
```

### 5. SLAM/Localization 골격

`robot_ws/src/robot_slam`

현재 `robot_slam`은 다음을 담당합니다.

```text
/scan + /wheel/odom
-> robot_localization EKF
-> slam_toolbox
-> map -> odom -> base_link -> camera_link / laser_link TF tree
```

현재 설계는 dual-EKF 구조입니다.

```text
/wheel/odom -> ekf_filter_node_odom -> odom -> base_link
slam_toolbox /pose + /odometry/filtered -> ekf_filter_node_map -> map -> odom
```

즉, 비전 노드가 낸 bbox를 실제 지도 좌표로 바꾸려면 이 TF tree와 카메라 보정값을 이용하는
추가 노드가 필요합니다.

## 계획서 대비 아직 필요한 기능

### MVP 필수 기능

아래 항목들은 계획서의 핵심 목표를 실제 로봇 동작으로 만들기 위해 반드시 필요합니다.

| 우선순위 | 기능 | 설명 | 주 담당 |
|---:|---|---|---|
| 1 | 최종 통합 YOLO 모델 | 현재 레고 세부 class는 `lego`로 alias 처리됨. 추후 `battery`, `coin` 모델까지 통합 필요 | 기존 비전팀 |
| 1 | class 이름 정합성 관리 | `data.yaml`, `DEFAULT_CLASS_ALIASES`, `risk_db.py`, `target_classes` 변경 시 함께 갱신 | 기존 비전팀 |
| 1 | 카메라 입력 노드 확정 | 실제 카메라가 `/image_raw`를 안정적으로 발행해야 함 | ROS2팀 |
| 1 | bbox -> 지면 좌표 변환 | 픽셀 bbox를 로봇 기준/지도 기준 좌표로 변환하는 `ground_projection_node` 필요 | ROS2팀 + 비전팀 |
| 1 | target manager | `/detections`의 `risk_score`를 이용해 지도 좌표 기준 최종 목표 선택 | ROS2팀 |
| 1 | 알람 인터페이스 | `ALARM`, `EMERGENCY_STOP`을 실제 부저/앱/로그/토픽으로 연결 | ROS2팀 |
| 1 | 수거 장치 제어 인터페이스 | `REMOVE` 명령을 스위핑/흡입/격리 수납함 구동으로 연결 | ROS2팀 + 기구팀 |
| 1 | 전체 launch 파일 | 카메라, 비전, SLAM, navigation, actuator, alarm을 한 번에 실행 | ROS2팀 |
| 1 | 실환경 threshold 튜닝 | `mouth_threshold_px`, small-object bbox 기준, 입 크기 기반 Level 1 기준을 실제 카메라 각도에서 보정 | 기존 비전팀 + ROS2팀 |

### 현재 코드에서 바로 확인해야 할 리스크

1. `data.yaml`은 현재 레고 세부 클래스 23개입니다. 코드에서는 alias로 `lego`에 매핑하지만,
   최종 시연 모델에 `battery`, `coin` 데이터가 들어가는지는 별도로 확인해야 합니다.
2. 현재 repo에는 `ground_projection_node`, `robot_navigation`, `goal_manager`, 실제 수거 장치 제어 노드가 없습니다.
3. MediaPipe 입/손 판단은 카메라가 아이 얼굴을 볼 수 있어야 잘 작동합니다. 로봇 상단 카메라가 바닥을 향하면
   입 랜드마크 검출률이 낮을 수 있습니다.

## 팀별 역할 구분

### 기존 비전팀 역할

비전팀은 “무엇이 위험한지 판단하는 눈과 판단 기준”을 책임지는 쪽입니다.

필수 담당:

1. 최종 학습 데이터셋 구성
2. `battery`, `coin`, `lego`, 추가 위험물 클래스 라벨링
3. `data.yaml` class 이름과 `DEFAULT_CLASS_ALIASES`, `risk_db.py` class 이름 일치 관리
4. YOLO 학습, 검증, mAP/Precision/Recall 관리
5. 기존 소형 위험물 bbox 및 입 크기 기반 미확인 물체 기준 튜닝
6. `mouth_threshold_scale`, `mouth_min_threshold_px`, `mouth_max_threshold_px`와 손-입 근접 조건 튜닝
7. `stable_detection_frames`, `track_iou_threshold`를 이용한 오탐 억제 튜닝
8. 오탐/미탐 이미지 수집 및 재학습 루프 구축
9. `/detections`와 `vision/detected_objects` 필드 정의와 유지

추천 추가 위험물 클래스:

```text
magnet
button_battery
screw
bead
small_toy_part
medicine
clip
pin
```

비전팀 산출물:

```text
best.pt 또는 final_safety_model.pt
final_data.yaml
위험도 DB 업데이트
검증 리포트(mAP50, mAP50-95, Precision, Recall, confusion matrix)
안전 판단 평가표(risk mapping, stable detection, emergency stop, priority accuracy)
실환경 threshold 표
오탐/미탐 사례 모음
```

### ROS2팀 역할

ROS2팀은 “비전 결과를 로봇 행동으로 바꾸는 통합 시스템”을 책임지는 쪽입니다.

필수 담당:

1. 카메라 드라이버가 `/image_raw` 발행
2. `robot_perception_msgs` 메시지 계약 관리
3. bbox 픽셀 좌표를 로봇/지도 좌표로 바꾸는 `ground_projection_node` 구현
4. SLAM/localization 실행 안정화
5. Nav2 또는 자체 goal manager로 위험물 접근 경로 생성
6. 위험도 우선순위 기반 목표 선택
7. 수거 장치 제어 노드 구현
8. 알람/긴급정지 노드 구현
9. 전체 시스템 launch 및 bringup 관리
10. 실제 로봇에서 TF, 카메라 높이, 카메라 틸트, 라이다 위치 보정

ROS2팀 산출물:

```text
camera.launch.py
perception.launch.py
full_pipeline.launch.py
ground_projection_node
target_manager_node
alarm_node
sweeper_controller_node
robot_navigation 또는 Nav2 설정
실측 TF/URDF/xacro
```

### 공동으로 맞춰야 하는 인터페이스

두 팀이 가장 먼저 합의해야 할 계약은 다음입니다.

```text
/image_raw
  카메라 이미지 입력

/detections
  bbox, 위험도, risk_score, robot_action, reason_codes를 담은 DetectionArray

vision/detected_objects
  위험도, 우선순위, action, track 상태, reason_codes를 담은 JSON

/target_objects 또는 /safety_targets
  지면/지도 좌표까지 변환된 위험물 후보

/robot_safety_action
  REMOVE, ALARM, EMERGENCY_STOP 같은 최종 행동 명령
```

현재 `Detection.msg`는 아래 안전 판단 필드까지 포함합니다.

```text
risk_class_name
risk_level
risk_score
string robot_action
string[] reason_codes
```

기존 디버깅/시각화 호환성을 위해 `vision/detected_objects` JSON도 계속 병행 발행합니다.

## 더 높은 성능을 위한 개선안

### 1. 모델 성능 개선

현재는 레고 세부 클래스 중심 데이터셋이므로 코드에서 alias로 상위 위험 class를 맞춥니다.
최종 모델은 “삼킴 위험물 중심”으로 통합 검증하는 것이 좋습니다.

권장 방향:

1. `battery`, `coin`, `lego`를 최종 상위 위험 class로 명확히 통일
2. 배터리는 `button_battery`와 일반 원통형 `battery`를 분리 검토
3. 자석, 작은 나사, 구슬, 약, 작은 장난감 부품 추가
4. 밝기/바닥재/거리/각도/가림 상황 augmentation
5. 실환경 테스트 이미지 기반 active learning
6. 오탐이 치명적인 클래스와 미탐이 치명적인 클래스를 분리해 threshold 개별 적용

평가 지표:

```text
mAP50
mAP50-95
Precision
Recall
False Negative rate for battery/button_battery
False Alarm per minute
End-to-end detection latency
Risk class mapping accuracy
Top-priority target accuracy
Emergency-stop false positive/false negative rate
```

### 2. 작은 미확인 물체 탐지 고도화

현재 unknown-object 로직은 YOLO가 검출한 bbox만 대상으로 합니다. 더 강하게 만들려면 다음을 추가할 수 있습니다.

1. YOLO segmentation 모델로 작은 물체 mask 추출
2. 배경 차분/바닥 평면 모델로 “바닥 위 작은 신규 물체” 감지
3. depth camera 또는 stereo camera로 실제 크기(cm) 추정
4. `bbox area` 대신 실제 물리 크기 기반 위험도 판단
5. open-vocabulary detector로 학습하지 않은 위험 후보 탐지

가장 현실적인 다음 단계는 **depth 없이 카메라 보정 + 지면 투영으로 실제 크기 근사**입니다.
픽셀 bbox만 쓰면 물체가 카메라에서 멀수록 작게 보이기 때문에 오판이 생길 수 있습니다.

### 3. 추적과 우선순위 개선

현재 코드는 가벼운 IoU 기반 시간축 안정화를 적용합니다. 더 강하게 만들려면 전용 tracker로 확장할 수 있습니다.

추가 권장:

1. ByteTrack/DeepSORT/SORT 기반 object tracking
2. 같은 물체의 ID 유지
3. confidence smoothing
4. bbox jitter filtering
5. 위험도 + 지도상 거리 + 아이와의 상대 위치를 합친 priority score

추천 priority 예시:

```text
priority =
  risk_level * 100
  + mouth_threat_bonus
  + child_distance_bonus
  + robot_reachability_bonus
  - uncertainty_penalty
```

### 4. 속도 최적화

로봇에서 실시간으로 돌리려면 추론 속도와 안정성이 중요합니다.

추가 권장:

1. YOLO11n 또는 YOLO11s 기준 latency 측정
2. GPU 사용 시 `device: "0"` 적용
3. TensorRT/OpenVINO/ONNX export 검토
4. frame skipping과 inference_rate_hz 튜닝
5. ROI crop 기반 재검출
6. headless 환경에서는 `enable_visualization: false`

### 5. 알람과 보호자 UX

계획서에는 보호자 긴급 알림이 들어가므로 실제 알람 채널이 필요합니다.

추가 기능:

1. 로봇 자체 부저/LED
2. 모바일 알림 또는 Telegram/Discord/웹 대시보드
3. 위험 상황 snapshot 저장
4. 이벤트 로그 저장
5. 보호자가 “수거 완료/오탐/무시”를 피드백하는 UI

### 6. 안전 기능

영유아 대상 로봇이므로 “잘 움직이는 것”보다 “위험하게 움직이지 않는 것”이 더 중요합니다.

추가 기능:

1. 아이와 최소 거리 유지
2. 수거 장치 작동 중 손 접근 감지 시 즉시 정지
3. 배터리/자석 등 Level 3 물체는 수거 실패 시 보호자 긴급 알림
4. 로봇이 물체를 밀어 아이 쪽으로 보내지 않도록 접근 방향 제한
5. 수납함 열림 감지
6. 수거 실패 retry 횟수 제한

## 개발 로드맵

### Phase 1: 비전/ROS2 인터페이스 안정화

1. 최종 상위 위험 class와 class alias 정책 확정
2. `best.pt`, `data.yaml`, `risk_db.py`, `DEFAULT_CLASS_ALIASES`, `target_classes` 정합성 확인
3. `/image_raw -> yolo_detector_node -> /detections` 실환경 테스트
4. `/detections`와 `vision/detected_objects` 샘플 저장
5. small-object Level 2 기준, 입 크기 기반 Level 1 기준, 시간축 안정화 threshold 튜닝

### Phase 2: 좌표 변환과 목표 생성

1. 카메라 intrinsic/extrinsic 측정
2. `ground_projection_node` 구현
3. bbox 중심 또는 하단 중심을 지면 좌표로 변환
4. 위험물 후보를 map/base_link 좌표로 발행
5. 같은 물체 중복 제거 및 tracking

### Phase 3: 주행/수거 연동

1. Nav2 또는 자체 navigation goal 생성
2. 위험물 앞 접근 pose 계산
3. 수거 장치 제어 노드 구현
4. 수거 성공/실패 판단
5. 수거 후 재탐지로 제거 확인

### Phase 4: 돌발행동 대응

1. MediaPipe가 실제 카메라 각도에서 동작하는지 검증
2. 입/손 인식이 어렵다면 별도 아이 관찰 카메라 추가 검토
3. 입 근접 이벤트를 알람 노드와 연결
4. 보호자 알림 채널 연결
5. 실험 시나리오별 latency 측정

### Phase 5: 완성도 고도화

1. 대시보드
2. 이벤트 로그와 리플레이
3. 모델 active learning 루프
4. open-vocabulary/segmentation 기반 unknown object 감지
5. 실제 크기 추정 기반 위험도 판단

## 실행 방법

자세한 실행 절차는 [howtorun.md](howtorun.md)를 참고하세요.

기본 빌드:

```bash
cd /home/joo/Desktop/Capstone_Vision/robot_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

비전 노드 실행 예시:

```bash
ros2 run robot_perception yolo_detector_node --ros-args \
  --params-file src/robot_perception/config/yolo_params.yaml \
  -p model_path:=/absolute/path/to/best.pt
```

빌드 확인:

```bash
colcon build --packages-select robot_perception robot_perception_msgs
```

2026.07.04 기준 확인 결과, `robot_perception`과 `robot_perception_msgs`는 빌드 성공했습니다.
`setup.cfg`의 deprecated option 경고도 `script_dir`, `install_scripts`로 수정해 제거했습니다.

## 2026.07.03 업데이트 요약

1. GitHub/README 정리 필요성이 커짐
2. ROS2 워크스페이스 구조 확인
3. `robot_perception`의 YOLO 탐지 노드와 `robot_perception_msgs` 메시지 패키지 확인
4. `robot_slam`의 SLAM/localization/TF launch 구조 확인
5. 작은 미확인 물체를 bbox 기준으로 Level 2 처리하는 로직 추가
6. `unknown_small_object` 위험도 DB 항목 추가
7. 계획서 대비 미구현 기능과 팀별 역할 정리

## 2026.07.04 업데이트 요약

1. 위험도 DB의 Level 2 이상 물체는 입 근접 시 모두 `EMERGENCY_STOP`으로 통일
2. 미등록 객체 중 기존 소형 위험물 bbox 기준 이하는 `unknown_small_object`로 분류
3. `unknown_small_object`는 Level 2, `REMOVE`, 입 근접 시 `EMERGENCY_STOP` 처리
4. 미등록 객체 중 아이 입 크기보다 큰 bbox는 `unknown_large_object` Level 1, `NONE`으로 등록해 별도 action 없이 인식만 수행
5. `vision/detected_objects` JSON에 입 크기 추정 기준과 unknown 분류 결과를 포함
6. 레고 세부 class 23개를 상위 위험 class `lego`로 묶는 `DEFAULT_CLASS_ALIASES`와 `class_aliases` 추가
7. IoU 기반 시간축 안정화 추가. 일반 `REMOVE`는 연속 프레임 확인 후 확정하고, `EMERGENCY_STOP`은 즉시 발행
8. `risk_score`와 `reason_codes`를 추가해 왜 위험하다고 판단했는지 설명 가능하게 개선
9. `/detections` 메시지에 `risk_class_name`, `risk_level`, `risk_score`, `robot_action`, `reason_codes` 추가
10. 검증 리포트에 Safety Mapping과 Safety Evaluation Protocol 섹션 추가

## 카메라 높이 변경 시 튜닝 메모

아직 실제 카메라 높이와 장착각이 정해지지 않았으므로, 현재 코드는 카메라 높이를 직접 입력받아
계산하지 않습니다. 다만 카메라를 높여 아이의 입/손과 바닥 대부분이 함께 보이도록 바꾸면
아래 항목은 실험 후 조정해야 합니다.

1. `mouth_threshold_px`, `mouth_threshold_scale`, `mouth_min_threshold_px`, `mouth_max_threshold_px`
   - 입 중심과 손/물체 중심이 얼마나 가까우면 삼킴 위험으로 볼지 정하는 거리 기준입니다.
   - MediaPipe가 입 너비를 잡으면 `mouth_width * mouth_threshold_scale`을 우선 사용하고,
     너무 작거나 커지지 않도록 최소/최대 픽셀 기준으로 제한합니다.
2. `mouth_size_area_scale`, `mouth_fallback_area_ratio`
   - `unknown_large_object` Level 1 판정에 쓰는 아이 입 크기 추정값입니다.
   - MediaPipe가 입꼬리 좌표를 안정적으로 잡으면 `mouth_size_area_scale`을 조정하고,
     입을 못 잡는 각도/가림 상황이 많으면 `mouth_fallback_area_ratio`를 별도로 보정해야 합니다.
3. `small_object_reference_area_scale`, `small_object_fallback_max_area_ratio`
   - DB 밖 작은 물체를 Level 2로 올리는 bbox 면적 기준입니다.
   - 카메라 높이가 바뀌면 모든 물체 bbox 면적이 달라지므로 기존 `battery/coin/lego` 기준과
     fallback ratio가 실제 삼킴 가능 크기를 잘 반영하는지 다시 확인해야 합니다.
4. `enable_pose_risk_scoring`
   - 높은 위치에서도 MediaPipe가 아이의 입/손을 안정적으로 잡으면 `true`를 유지합니다.
   - 바닥은 잘 보이지만 얼굴/손 landmark가 불안정하면, 별도 아이 관찰 카메라를 두거나
     입-손 근접 판단을 분리하는 설계를 검토해야 합니다.
5. `stable_detection_frames`, `track_iou_threshold`
   - 오탐이 많으면 `stable_detection_frames`를 늘리거나 `track_iou_threshold`를 높입니다.
   - 빠르게 움직이는 물체를 놓치면 `track_iou_threshold`를 낮추고, 긴급정지는 입 근접 조건으로 즉시 처리합니다.
6. 카메라 보정값
   - bbox를 지도/지면 좌표로 바꾸는 단계가 추가되면 카메라 높이, 틸트각, intrinsic/extrinsic 보정값이
     필요합니다. 현재 `yolo_detector_node.py`는 픽셀 bbox만 발행하고 실제 지면 좌표 변환은
     아직 별도 `ground_projection_node` 구현 대상입니다.

실험 순서는 `enable_visualization: true`로 화면을 보면서 입/손 landmark가 안정적으로 잡히는지 확인한 뒤,
입 근접 기준, small-object bbox 기준, Level 1 입 크기 기준, 시간축 안정화 기준을 차례대로 조정하는 방식이 좋습니다.

## 비전파트 업그레이드 최종 정리

이번 코드 개선으로 비전파트는 단순 YOLO bbox 출력에서 **안전 판단이 가능한 비전 시스템**으로 확장되었습니다.

1. 상위 위험 class 매핑
   - 레고 세부 class 23개를 모두 `lego`로 통합했습니다.
   - 발표에서는 모델 class와 안전 판단 class를 분리했다고 설명할 수 있습니다.
2. 미확인 물체 위험도 판단
   - DB에 없는 물체도 bbox 크기와 입 크기 기준으로 `unknown_small_object` 또는 `unknown_large_object`로 분류합니다.
   - 동전/배터리/레고 외 사진을 추가하지 않아도 안전 논리를 보강할 수 있습니다.
3. 입-손-물체 돌발 상황 판단
   - 손이 bbox 안에 들어가거나 물체/손이 입에 가까워지면 `EMERGENCY_STOP`을 즉시 발행합니다.
   - 입 근접 기준은 고정 픽셀값이 아니라 입 너비 기반으로 보정됩니다.
4. 시간축 안정화
   - 같은 물체가 연속 프레임에서 확인되어야 일반 `REMOVE`로 확정합니다.
   - 한 프레임 오탐으로 로봇이 움직이는 문제를 줄입니다.
5. 위험 점수와 판단 근거
   - `risk_score`는 위험도, confidence, track 안정성, 손 겹침, 입 근접 여부를 합산합니다.
   - `reason_codes`로 `CLASS_ALIAS`, `RISK_LEVEL`, `OBJECT_NEAR_MOUTH`, `STABLE_TRACK` 같은 근거를 남깁니다.
6. 구조화된 ROS2 메시지
   - `/detections`에 bbox뿐 아니라 `risk_class_name`, `risk_level`, `risk_score`, `robot_action`, `reason_codes`를 담습니다.
   - ROS2팀이 JSON 파싱 없이도 위험도 기반 목표 선택을 할 수 있습니다.
7. 평가 리포트 고도화
   - 기존 mAP/Precision/Recall 외에 Safety Mapping과 Safety Evaluation Protocol을 추가했습니다.
   - 평가 항목은 위험 class 매핑 정확도, 안정 검출 후 수거, 긴급정지 오탐/미탐, 1순위 목표 정확도입니다.

## 다음 회의 체크리스트

1. 최종 모델에 `battery`, `coin` class가 실제로 포함되는지 확인
2. 레고 세부 class를 계속 유지할지, 최종 학습 단계에서 `lego` 단일 class로 합칠지 결정
3. `ground_projection_node` 담당자와 좌표계 인터페이스 결정
4. 수거 장치 제어 토픽/action/service 이름 결정
5. 알람 노드 출력 방식 결정
6. 실제 카메라 장착 위치와 MediaPipe 사용 가능성 검증
7. `risk_score` 기준으로 로봇이 어떤 목표를 먼저 처리할지 ROS2팀과 합의
