# Capstone_Vision

2026 창의융합 캡스톤디자인 팀 **막아줘 영(Young)** 의 비전/ROS2 작업 공간입니다.

기준일: **2026.07.05**

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

## 계획서 대비 현재 상태

기존에 “비전팀 역할”로 적혀 있던 안전 판단 로직은 현재 코드에 반영된 상태로 정리합니다.
다만 최종 시연 모델의 성능 검증, threshold 튜닝, 실제 카메라 각도 검증은 아직 해야 할 Task입니다.

### 비전팀 구현 완료 처리 항목

| 항목 | 현재 상태 | 관련 파일 |
|---|---|---|
| YOLO 탐지 노드 | `/image_raw` 입력을 받아 YOLO 추론 후 `/detections`, `vision/detected_objects` 발행 | `robot_ws/src/robot_perception/robot_perception/yolo_detector_node.py` |
| 위험도 DB | `battery`, `coin`, `lego`, `unknown_small_object`, `unknown_large_object` 위험도 등록 | `risk_db.py`, `robot_ws/src/robot_perception/robot_perception/risk_db.py` |
| class alias | 레고 세부 class 23개를 상위 위험 class `lego`로 매핑 | `data.yaml`, `risk_db.py`, `yolo_params.yaml` |
| 미확인 물체 판단 | DB 밖 객체를 bbox 크기와 입 크기 기준으로 Level 2 또는 Level 1로 분류 | `yolo_detector_node.py` |
| 입-손-물체 근접 판단 | 손이 bbox 안에 있고 입에 가까워지거나, 물체 중심이 입에 가까우면 `EMERGENCY_STOP` | `yolo_detector_node.py` |
| 시간축 안정화 | 일반 `REMOVE`는 연속 프레임 확인 후 확정, 입 근접 긴급정지는 즉시 발행 | `yolo_detector_node.py` |
| 판단 근거 출력 | `risk_score`, `reason_codes`, `robot_action`을 구조화 메시지와 JSON에 포함 | `Detection.msg`, `yolo_detector_node.py` |
| 검증 리포트 틀 | mAP/Precision/Recall, Safety Mapping, Safety Evaluation Protocol 저장 | `val_model.py`, `reports/validation/validation_report.md` |

### 아직 없는 기능

| 기능 | 현재 상태 | 주 담당 |
|---|---|---|
| 실제 카메라 입력 확정 | `/image_raw`를 안정적으로 발행하는 카메라 bringup 노드/launch가 repo에 없음 | ROS2팀 |
| bbox -> 지면 좌표 변환 | `ground_projection_node`가 아직 없음. 현재 `/detections`는 픽셀 bbox 기준 | ROS2팀 + 비전팀 |
| target manager | `risk_score`와 지도 좌표를 이용해 최종 목표 1개를 고르는 노드가 없음 | ROS2팀 |
| 알람/긴급정지 인터페이스 | `EMERGENCY_STOP`을 실제 부저, 앱, 로그, 제어 토픽으로 연결하는 노드가 없음 | ROS2팀 |
| 수거 장치 제어 | `REMOVE`를 스위핑/흡입/격리 수납함 구동으로 연결하는 노드가 없음 | ROS2팀 + 기구팀 |
| 전체 bringup launch | 카메라, 비전, SLAM, 주행, 알람, 수거 장치를 한 번에 실행하는 launch가 없음 | ROS2팀 |

### 현재 코드에서 바로 확인해야 할 리스크

1. `data.yaml`은 레고 세부 클래스 23개입니다. 코드에서는 alias로 `lego`에 매핑하지만,
   현재 검증 리포트 기준으로 `battery`, `coin`은 모델 class에 없습니다.
2. `robot_perception/package.xml`과 `setup.py` 설명에는 ground projection까지 언급되어 있지만,
   실제 repo에는 아직 `ground_projection_node`가 없습니다.
3. MediaPipe 입/손 판단은 카메라가 아이 얼굴과 손을 볼 수 있어야 잘 작동합니다. 로봇 카메라가 바닥을 향하면
   `enable_pose_risk_scoring`을 끄거나 별도 아이 관찰 카메라/노드로 분리해야 할 수 있습니다.
4. 2026.07.02 확인된 카메라 이슈처럼 `/dev/video*`가 없으면 ROS2나 YOLO 문제가 아니라
   현재 Ubuntu 실행 환경에 카메라 장치가 노출되지 않은 문제입니다.

## 공동 인터페이스

두 팀이 먼저 맞춰야 할 계약은 다음입니다.

```text
/image_raw
  카메라 이미지 입력

/detections
  bbox, 위험도, risk_score, robot_action, reason_codes를 담은 DetectionArray

vision/detected_objects
  위험도, 우선순위, action, track 상태, reason_codes를 담은 JSON

/safety_targets
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

## 앞으로 해야 할 Todo

### 비전팀 Todo

이미지 추가 수집만이 아니라, 현재 구현된 모델/판단 로직을 더 믿을 수 있게 만드는 작업입니다.

| 우선순위 | Task | 목적 | 완료 기준 |
|---:|---|---|---|
| 1 | 최종 모델 class 검증 | 현재 `data.yaml`은 레고 23개 class만 있음. 최종 `best.pt`가 `battery`, `coin`, `lego`를 실제로 검출하는지 확인 | `battery`/`coin`/`lego` 테스트 이미지 또는 영상 결과표 작성 |
| 1 | class alias 정책 확정 | 세부 class 유지 후 alias 매핑할지, 최종 학습 단계에서 상위 class로 합칠지 결정 | `data.yaml`, `DEFAULT_CLASS_ALIASES`, `yolo_params.yaml`, `risk_db.py` class 표 정리 |
| 1 | confidence threshold 튜닝 | 오탐/미탐 균형 조정 | 0.2, 0.3, 0.4, 0.5 비교표와 최종 threshold 1개 확정 |
| 1 | small-object 기준 튜닝 | `unknown_small_object`가 너무 많이/적게 나오지 않게 조정 | `small_object_reference_area_scale`, `small_object_fallback_max_area_ratio` 실험표 작성 |
| 1 | 입 근접 기준 튜닝 | 실제 카메라 각도에서 `EMERGENCY_STOP` 오탐/미탐 줄이기 | `mouth_threshold_scale`, `mouth_min_threshold_px`, `mouth_max_threshold_px` 최종값 기록 |
| 1 | 시간축 안정화 기준 튜닝 | 한 프레임 오탐으로 `REMOVE`가 나가지 않게 조정 | `stable_detection_frames`, `track_iou_threshold`, `lost_track_ttl_frames` 비교 결과 작성 |
| 2 | 판단 로직 단위 테스트 설계 | YOLO 없이 class/bbox/mouth/hands 가짜 입력으로 위험 판단만 검증 | alias, unknown object, stable track, mouth proximity 테스트 케이스 목록 작성 |
| 2 | 위험 이벤트 로그/리플레이 설계 | 나중에 오탐/미탐을 분석할 수 있게 frame, bbox, reason_codes 저장 | `REMOVE`/`EMERGENCY_STOP` 발생 시 저장할 JSONL 필드와 snapshot 규칙 정의 |
| 2 | 안전 평가표 작성 | mAP와 별도로 안전 행동 정확도 평가 | `risk_class_mapping_accuracy`, `top_priority_target_accuracy`, 긴급정지 오탐/미탐 표 작성 |
| 2 | 추적 로직 개선 검토 | IoU 추적이 부족하면 ID 유지와 smoothing 강화 | ByteTrack/SORT 적용 필요 여부와 현재 IoU 방식 한계 정리 |
| 3 | 추론 속도 최적화 | 로봇 실시간 실행 가능성 확인 | CPU/GPU FPS, `inference_rate_hz`, `enable_visualization: false`, YOLO11n/s 비교표 작성 |
| 3 | 실제 크기 기반 판단 검증 | 픽셀 bbox 면적은 거리 영향을 받으므로 지면 투영 후 cm 기준 판단 가능성 확인 | ROS2팀 `ground_projection_node` 결과로 bbox 실제 크기 근사 검증 |

### ROS2팀 Todo

비전 결과를 실제 로봇 행동으로 연결하기 위해 필요한 작업입니다.

| 우선순위 | Task | 목적 | 완료 기준 |
|---:|---|---|---|
| 1 | 카메라 `/image_raw` 안정화 | 비전 노드 입력 확보 | `ros2 topic hz /image_raw`, 샘플 frame 캡처, 장시간 끊김 여부 확인 |
| 1 | 카메라 장치 노출 확인 | `/dev/video*`가 없으면 YOLO 이전 단계에서 실패함 | `ls /dev/video*`, `v4l2-ctl --list-devices`, 권한/WSL/Docker/VM 여부 확인 |
| 1 | `/detections` 구독 확인 | JSON 대신 구조화 메시지로 후속 노드 연결 | `DetectionArray` 샘플 메시지 3개 이상 저장 |
| 1 | `ground_projection_node` 구현 | 픽셀 bbox를 `base_link` 또는 `map` 좌표로 변환 | `/detections` 입력, `/safety_targets` 출력, 카메라 intrinsic/extrinsic 사용 |
| 1 | target manager 구현 | 여러 위험물 중 처리할 목표 1개 선택 | `risk_score`, 거리, 중복 target 제거 기준으로 목표 발행 |
| 1 | 알람/긴급정지 노드 구현 | `EMERGENCY_STOP`을 실제 행동으로 연결 | 부저, LED, 앱, 로그, 제어 토픽 중 최소 1개 작동 |
| 1 | 수거 장치 제어 인터페이스 | `REMOVE`를 실제 수거 동작으로 연결 | 수거 시작/성공/실패 토픽 또는 action/service 계약 정의 |
| 2 | 전체 launch 구성 | 카메라, 비전, SLAM, target manager, 알람, 수거 노드 동시 실행 | `full_pipeline.launch.py` 또는 bringup launch 작성 |
| 2 | TF/카메라 실측 반영 | 지도 좌표 변환 정확도 확보 | `base_link -> camera_link`, 카메라 높이, 틸트각, 라이다 위치 실측값 반영 |
| 2 | Nav2 또는 자체 주행 연결 | 위험물 위치까지 접근 | `/safety_targets`를 navigation goal로 변환하고 접근 pose 생성 |
| 3 | 수거 후 재탐지 루프 | 제거 성공 여부 확인 | 수거 후 같은 target 재검출 여부로 성공/실패 판정 |

### 공동 Todo

1. 실제 카메라 높이와 각도를 먼저 정하고 MediaPipe 입/손 검출률을 측정합니다.
2. `risk_score`가 높은 물체를 먼저 처리할지, 로봇 거리와 접근 가능성을 함께 볼지 우선순위 정책을 합의합니다.
3. 최종 시연 시나리오는 레고 탐지, 배터리/동전 검출 여부, 입 근접 긴급정지, 수거 후 재탐지 중 가능한 범위로 고정합니다.
4. 발표용 지표는 모델 정확도(mAP/Precision/Recall)와 안전 행동 정확도(오탐/미탐, 1순위 목표 정확도)를 분리해서 제시합니다.
