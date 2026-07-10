# 비전 파트 중간보고서

기준일: 2026.07.10

## 1. 비전 파트 개발 목표

본 프로젝트의 최종 목표는 영유아 삼킴 사고를 예방하기 위해, 모바일 로봇이 실내 바닥의 소형 위험물을 탐지하고 위험도에 따라 수거 또는 긴급 대응을 수행하는 것이다. 이 중 비전 파트는 카메라 영상에서 위험물 후보를 찾아내고, 해당 물체가 얼마나 위험한지 판단한 뒤, ROS2 기반 로봇 시스템이 사용할 수 있는 형태로 결과를 발행하는 역할을 맡는다.

현재 비전 파트의 핵심 목표는 다음과 같다.

| 목표 | 설명 |
|---|---|
| 위험물 탐지 | YOLO 모델을 이용해 카메라 프레임에서 레고, 동전, 배터리 등 소형 물체를 탐지한다. |
| 위험도 판단 | 탐지된 class를 위험도 DB와 연결해 Level 1~3 수준으로 분류한다. |
| 미등록 물체 대응 | DB에 없는 물체도 bbox 크기와 입 크기 기준으로 삼킴 가능성을 추정한다. |
| 삼킴 위험 상황 판단 | MediaPipe Pose를 이용해 손, 입, 물체의 근접 관계를 확인하고 긴급정지 여부를 판단한다. |
| ROS2 연동 | 탐지 결과를 `/detections`와 `vision/detected_objects` 토픽으로 발행해 SLAM, 제어, 알림 파트와 연결할 수 있게 한다. |

## 2. 전체 코드 구조 중 비전 관련 범위

현재 저장소는 루트 개발 스크립트와 ROS2 워크스페이스로 나뉜다.

```text
Capstone_Vision/
├─ data.yaml
├─ test_image.py
├─ val_model.py
├─ vision_ros2_node.py
├─ risk_db.py
├─ reports/validation/
└─ robot_ws/src/
   ├─ robot_perception/
   │  ├─ robot_perception/yolo_detector_node.py
   │  ├─ robot_perception/risk_db.py
   │  ├─ config/yolo_params.yaml
   │  └─ scripts/
   │     ├─ risk_logic_unit_check.py
   │     ├─ val_model.py
   │     └─ test_image.py
   └─ robot_perception_msgs/
      └─ msg/
         ├─ Detection.msg
         └─ DetectionArray.msg
```

루트의 `test_image.py`, `val_model.py`, `vision_ros2_node.py`, `risk_db.py`는 초기 개발 및 단독 검증용 코드이다. 현재 실제 ROS2 연동 기준의 주력 구현은 `robot_ws/src/robot_perception/robot_perception/yolo_detector_node.py`이다.

`robot_perception_msgs` 패키지는 비전 결과를 다른 ROS2 노드가 구독할 수 있도록 커스텀 메시지를 정의한다. `Detection.msg`에는 class, confidence, bbox뿐 아니라 `risk_class_name`, `risk_level`, `risk_score`, `robot_action`, `reason_codes`가 포함되어 있어 단순 탐지 결과가 아니라 안전 판단 결과까지 전달할 수 있다.

## 3. 개발 과정

### 3.1 초기 YOLO 탐지 스크립트 구성

초기 단계에서는 ROS2와 분리된 상태에서 YOLO 모델을 직접 실행해 이미지 또는 카메라 프레임에서 객체를 탐지하는 방식으로 개발을 시작했다. 이 단계의 목적은 모델이 실제로 물체를 인식하는지, bbox와 confidence가 어떤 형태로 나오는지 확인하는 것이었다.

관련 파일은 다음과 같다.

| 파일 | 역할 |
|---|---|
| `test_image.py` | 정적 이미지 1장을 대상으로 YOLO 탐지 결과를 확인하는 테스트 스크립트 |
| `val_model.py` | 학습된 `best.pt` 모델을 `data.yaml` 기준으로 검증하고 Precision, Recall, mAP를 출력하는 스크립트 |
| `vision_ros2_node.py` | 초기 ROS2 스타일 웹캠 테스트 노드 |
| `risk_db.py` | 위험물별 위험도와 로봇 행동 명령을 정의한 초기 DB |

이 단계에서는 모델 성능 확인과 위험도 DB의 기본 구조를 잡는 데 집중했다.

### 3.2 ROS2 패키지로 이식

다음 단계에서는 초기 스크립트의 탐지 로직을 ROS2 워크스페이스 안의 `robot_perception` 패키지로 옮겼다. 이를 통해 카메라 입력, YOLO 추론, 안전 판단, ROS2 토픽 발행이 하나의 노드에서 동작하도록 구성했다.

현재 주력 노드인 `yolo_detector_node.py`는 `/image_raw`를 구독하고, YOLO 추론 결과를 바탕으로 `/detections`와 `vision/detected_objects`를 발행한다.

```text
/image_raw
-> yolo_detector_node
-> /detections
-> vision/detected_objects
```

`/detections`는 후속 ROS2 파이프라인이 사용하기 위한 구조화 메시지이고, `vision/detected_objects`는 기존 디버깅 코드와 호환되는 JSON 문자열 토픽이다.

### 3.3 위험도 DB와 class 정책 정리

비전 파트에서는 모델이 출력한 class를 그대로 로봇 행동으로 연결하지 않고, 안전 판단용 상위 class로 정규화한다.

현재 위험도 DB에는 다음 항목이 있다.

| 위험 class | level | 일반 action | 입 근접 action | 의미 |
|---|---:|---|---|---|
| `battery` | 3 | `REMOVE` | `EMERGENCY_STOP` | 고위험 물체 |
| `coin` | 2 | `REMOVE` | `EMERGENCY_STOP` | 삼킴 가능성이 있는 주의 물체 |
| `lego` | 2 | `REMOVE` | `EMERGENCY_STOP` | 삼킴 가능성이 있는 주의 물체 |
| `unknown_small_object` | 2 | `REMOVE` | `EMERGENCY_STOP` | 작은 미확인 물체 |
| `unknown_large_object` | 1 | `NONE` | `NONE` | 입보다 큰 미확인 물체 |

현재 `data.yaml`은 레고 세부 부품 23개 class 기준이다. 하지만 발표와 안전 판단에서는 세부 부품명을 모두 구분하기보다 `lego`라는 상위 class로 통일하는 것이 더 적합하다. 그래서 `DEFAULT_CLASS_ALIASES`와 `yolo_params.yaml`의 `class_aliases`를 통해 `C1x1x1`, `C2x1x1` 같은 세부 class를 모두 `lego`로 매핑한다.

이 구조 덕분에 기존 23-class 모델을 유지하면서도, 로봇 행동 판단은 `battery`, `coin`, `lego` 중심의 단순하고 설명 가능한 정책으로 정리할 수 있다.

### 3.4 미등록 물체 위험 판단 추가

아이에게 위험한 물체가 학습된 class에만 한정되는 것은 아니므로, DB에 없는 class도 bbox 크기를 이용해 보조적으로 판단하도록 확장했다.

현재 로직은 다음 순서로 동작한다.

1. YOLO가 bbox를 만든 객체를 가져온다.
2. 모델 class를 위험 class로 alias 매핑한다.
3. `battery`, `coin`, `lego`처럼 DB에 있는 class는 DB 위험도를 그대로 사용한다.
4. DB에 없는 class는 bbox 면적을 계산한다.
5. 같은 프레임에 잡힌 `battery`, `coin`, `lego` 기준 bbox보다 작거나 같으면 `unknown_small_object`로 처리한다.
6. 기준 객체가 없으면 전체 이미지 면적 대비 fallback ratio로 작은 물체 여부를 판단한다.
7. 작은 물체가 아니지만 입 크기보다 큰 물체는 `unknown_large_object`로 처리한다.
8. 두 기준에 모두 해당하지 않으면 안전 판단 대상으로 발행하지 않는다.

이 로직은 학습 데이터에 없는 작은 물체까지 일부 대응할 수 있도록 하기 위한 보완책이다. 단, YOLO가 아예 검출하지 못한 물체는 bbox 자체가 없으므로 이 방식으로 판단할 수 없다.

### 3.5 MediaPipe 기반 손-입-물체 근접 판단

단순히 위험물이 바닥에 있는 상황과, 아이가 위험물을 집어 입으로 가져가는 상황은 위험도가 다르다. 이를 반영하기 위해 `yolo_detector_node.py`는 MediaPipe Pose를 이용해 양손 검지와 입 중심 좌표를 추출한다.

긴급 상황은 다음 조건을 기반으로 판단한다.

| 조건 | 설명 |
|---|---|
| 손이 bbox 안에 있음 | 아이가 물체를 잡은 것으로 추정 |
| 손과 입이 가까움 | 잡은 물체를 입으로 가져가는 상황으로 추정 |
| 물체 중심과 입이 가까움 | 손 검출이 완벽하지 않아도 물체가 입 근처에 있는 상황으로 추정 |

위 조건 중 입 근접 위험이 발생하고 위험 level이 2 이상이면 `robot_action`을 `EMERGENCY_STOP`으로 바꾼다. 이때는 시간축 안정화 대기 없이 즉시 발행한다. 반대로 일반적인 수거 명령인 `REMOVE`는 오탐을 줄이기 위해 같은 물체가 여러 프레임에서 반복 확인된 뒤 확정한다.

### 3.6 시간축 안정화와 risk score 도입

YOLO는 한 프레임 단위로 탐지하므로 조명, 흔들림, 가림에 따라 오탐이 발생할 수 있다. 이를 줄이기 위해 현재 노드는 IoU 기반으로 같은 물체를 추적하고, `stable_detection_frames`만큼 연속 확인되었을 때 일반 `REMOVE`를 확정한다.

기본 파라미터는 다음과 같다.

| 파라미터 | 기본값 | 의미 |
|---|---:|---|
| `stable_detection_frames` | 3 | 일반 수거 대상으로 확정하기 위해 필요한 연속 검출 프레임 수 |
| `lost_track_ttl_frames` | 5 | 매칭되지 않은 track을 유지하는 프레임 수 |
| `track_iou_threshold` | 0.3 | 같은 물체로 볼 bbox IoU 기준 |

또한 객체별 우선순위를 비교하기 위해 `risk_score`를 계산한다. 점수에는 위험 level, confidence, track 안정성, 손과 bbox의 겹침, 입 근접 여부, unknown small object 여부가 반영된다. 즉, 단순히 confidence가 높은 물체보다 실제 안전 행동이 필요한 물체가 더 높은 우선순위를 갖도록 설계했다.

## 4. 구현 원리

### 4.1 YOLO 객체 탐지

YOLO는 입력 이미지를 한 번의 추론으로 처리해 각 객체의 class, confidence, bbox를 출력한다. 현재 노드는 ROS2 이미지 메시지를 OpenCV 이미지로 변환한 뒤, `best.pt` 모델에 입력한다.

출력 bbox는 `[xmin, ymin, xmax, ymax]` 형태의 픽셀 좌표이다. 이 좌표는 현재 단계에서는 지면 좌표가 아니라 이미지 안에서의 위치이다. 따라서 후속 로봇 이동을 위해서는 추후 카메라 보정과 지면 투영을 수행하는 별도 노드가 필요하다.

### 4.2 위험도 DB 기반 행동 결정

YOLO가 탐지한 물체는 먼저 안전 판단용 위험 class로 변환된다. 이후 `risk_db.py`의 `RISK_DATABASE`를 조회해 기본 위험 level과 행동 명령을 결정한다.

```text
model class
-> class alias
-> risk class
-> RISK_DATABASE 조회
-> risk_level / robot_action 결정
```

예를 들어 모델이 `C2x1x1`을 출력하면, alias를 통해 `lego`로 변환되고, 위험도 DB에서 level 2와 `REMOVE` action을 가져온다.

### 4.3 bbox 크기 기반 unknown object 판단

미등록 물체는 class 이름만으로 위험도를 알 수 없기 때문에 bbox 면적을 사용한다. 작은 물체는 삼킴 가능성이 있다고 보고 `unknown_small_object`로 분류한다. 반대로 입 크기보다 큰 물체는 삼킴 가능성이 낮다고 보고 `unknown_large_object`로 분류해 인식만 수행한다.

이 방식은 실제 물리 크기가 아니라 이미지상의 픽셀 크기를 이용하므로, 카메라 높이와 각도에 따라 threshold 튜닝이 필요하다. 따라서 현재는 `small_object_reference_area_scale`, `small_object_fallback_max_area_ratio`, `mouth_size_area_scale`, `mouth_fallback_area_ratio` 같은 파라미터로 조정 가능하게 만들어 두었다.

### 4.4 MediaPipe Pose 기반 삼킴 위험 판단

MediaPipe Pose는 사람의 주요 landmark를 추정하는 라이브러리이다. 현재 코드는 왼손/오른손 검지 좌표와 입 양끝 좌표를 사용한다. 입 양끝의 중심을 입 중심으로 보고, 입 너비를 이용해 입 근접 거리 threshold를 동적으로 계산한다.

```text
입 근접 threshold
= mouth_width * mouth_threshold_scale
단, mouth_min_threshold_px와 mouth_max_threshold_px 사이로 제한
```

MediaPipe가 입 좌표를 잡지 못하는 경우에는 프레임 크기 기반 fallback 값을 사용한다.

### 4.5 ROS2 메시지 발행 구조

현재 노드는 두 종류의 출력을 발행한다.

| 토픽 | 메시지 타입 | 목적 |
|---|---|---|
| `/detections` | `robot_perception_msgs/DetectionArray` | 후속 ROS2 노드가 구독할 구조화 탐지/판단 결과 |
| `vision/detected_objects` | `std_msgs/String` JSON | 기존 코드 호환 및 디버깅용 상세 결과 |

`/detections`에는 bbox, confidence, 위험 class, 위험 level, risk score, action, reason codes가 포함된다. 후속 노드는 이 메시지를 받아 로봇 좌표 변환, 목표 선택, 알림, 수거 행동으로 연결할 수 있다.

`reason_codes`는 판단 근거를 사람이 확인할 수 있도록 남기는 필드이다. 예시는 다음과 같다.

```text
CLASS_ALIAS:c2x1x1->lego
RISK_LEVEL_2
HAND_INSIDE_BBOX
OBJECT_NEAR_MOUTH
STABLE_TRACK_3_FRAMES
```

## 5. 현재 진행 현황

### 5.1 구현 완료된 항목

| 항목 | 현재 상태 | 관련 파일 |
|---|---|---|
| YOLO 탐지 노드 | `/image_raw`를 받아 YOLO 추론 후 결과 발행 | `yolo_detector_node.py` |
| ROS2 커스텀 메시지 | bbox와 안전 판단 정보를 담는 `Detection`, `DetectionArray` 정의 | `robot_perception_msgs/msg/` |
| 위험도 DB | `battery`, `coin`, `lego`, `unknown_small_object`, `unknown_large_object` 정의 | `robot_perception/risk_db.py` |
| 레고 세부 class alias | 23개 레고 세부 class를 `lego`로 통합 | `risk_db.py`, `yolo_params.yaml` |
| 미등록 물체 판단 | bbox 크기와 입 크기 기준으로 unknown object 분류 | `yolo_detector_node.py` |
| 입-손-물체 근접 판단 | MediaPipe Pose 기반으로 `EMERGENCY_STOP` 판단 | `yolo_detector_node.py` |
| 시간축 안정화 | IoU 기반 track으로 일반 `REMOVE` 오탐 완화 | `yolo_detector_node.py` |
| 판단 근거 출력 | `reason_codes`, `risk_score`, `robot_action` 발행 | `Detection.msg`, `yolo_detector_node.py` |
| 모델 검증 리포트 | Precision, Recall, mAP와 안전 평가 항목 저장 | `reports/validation/validation_report.md` |
| 위험 로직 단위 점검 | 카메라 없이 정책 로직을 확인하는 스크립트 작성 | `scripts/risk_logic_unit_check.py` |

### 5.2 검증 결과

현재 저장된 YOLO 검증 리포트 기준 성능은 다음과 같다.

| Metric | Value |
|---|---:|
| Precision | 46.17% |
| Recall | 53.51% |
| mAP50 | 57.41% |
| mAP50-95 | 48.52% |

이 결과는 현재 `data.yaml`과 `best.pt` 기준 모델 검증 결과이다. 현재 dataset class는 레고 세부 class 23개이므로, 이 수치는 레고 세부 부품 탐지 성능을 기준으로 해석해야 한다.

카메라 없이 위험 판단 정책만 확인하는 `risk_logic_unit_check.py`도 실행 가능하다. 이 스크립트는 다음 항목을 확인한다.

| 확인 항목 | 의미 |
|---|---|
| lego single-class policy | `lego` 위험 class가 안정화 후 `REMOVE`로 바뀌는지 확인 |
| legacy lego alias compatibility | 세부 레고 class가 `lego`로 매핑되는지 확인 |
| mouth proximity emergency stop | 입 근접 상황에서 `EMERGENCY_STOP`이 즉시 발생하는지 확인 |
| unknown small object mapping | 작은 미등록 물체가 Level 2로 분류되는지 확인 |
| unknown large object observe-only mapping | 큰 미등록 물체가 Level 1, `NONE`으로 처리되는지 확인 |

2026.07.10 현재 실행 결과는 `risk logic unit checks: PASS`이다.

### 5.3 아직 진행 중이거나 미완료인 항목

| 항목 | 현재 상태 | 필요한 작업 |
|---|---|---|
| 최종 class 구성 | 현재 검증 데이터는 레고 23개 class 중심 | 최종 발표 정책에 맞춰 `battery`, `coin`, `lego` 모델 또는 테스트 자료 확보 |
| 실제 카메라 검증 | 코드상 카메라 입력 구조는 준비됨 | 실제 `/image_raw` 입력, 장시간 frame 안정성, 장착각별 검출률 확인 |
| MediaPipe 각도 검증 | 로직은 구현됨 | 로봇 카메라 각도에서 아이 입/손 landmark가 안정적으로 잡히는지 실험 필요 |
| threshold 튜닝 | 파라미터화는 완료됨 | confidence, small object, mouth distance, stable frame 기준을 실제 환경에서 조정 |
| 지면 좌표 변환 | 현재는 픽셀 bbox만 발행 | 카메라 intrinsic/extrinsic과 TF를 이용한 `ground_projection_node` 구현 필요 |
| 최종 행동 연결 | `robot_action` 문자열은 발행됨 | 알림, 정지, 수거 장치, navigation goal과 연결 필요 |
| 안전 행동 평가 | 평가 항목 틀은 있음 | 긴급정지 오탐/미탐률, 1순위 목표 정확도 등 실제 실험표 작성 필요 |

## 6. 현재 완성도 요약

현재 비전 파트는 단순한 객체 탐지 수준을 넘어, 탐지 결과를 위험도 DB와 연결하고, 아이의 입/손 근접 상황까지 고려해 로봇 행동 명령을 판단하는 단계까지 구현되어 있다. 또한 ROS2 메시지 구조와 토픽 발행이 준비되어 있어 다른 파트와 연동할 수 있는 인터페이스도 마련되어 있다.

다만 최종 시연 수준으로 보려면 실제 카메라 환경에서의 검증이 아직 필요하다. 특히 현재 저장된 모델 검증 결과는 레고 세부 class 중심이며, 배터리와 동전까지 포함한 최종 위험물 모델 검증은 별도 데이터가 필요하다. 또한 MediaPipe 기반 입/손 판단은 카메라가 아이의 얼굴과 손을 충분히 볼 수 있어야 안정적으로 작동하므로, 실제 로봇 장착각에서 검출률을 측정해야 한다.

정리하면 현재 진행 상태는 다음과 같다.

| 구분 | 진행 상태 |
|---|---|
| YOLO 기반 객체 탐지 | 구현 완료, 모델 성능 검증 리포트 존재 |
| 위험도 DB 연동 | 구현 완료 |
| 레고 class alias 정책 | 구현 완료 |
| 미등록 물체 보조 판단 | 구현 완료 |
| 입-손-물체 근접 기반 긴급정지 판단 | 코드 구현 완료, 실제 카메라 검증 필요 |
| ROS2 DetectionArray 발행 | 구현 완료 |
| 지면 좌표 변환 및 주행 연동 | 미구현 |
| 실제 수거/알림 장치 연동 | 미구현 |
| 최종 현장 튜닝 | 미완료 |

## 7. 향후 계획

비전 파트에서 우선적으로 진행해야 할 작업은 실제 시연 환경에서 코드가 안정적으로 동작하는지 확인하는 것이다.

1. 실제 카메라 입력 확인
   - `/image_raw` 토픽이 안정적으로 발행되는지 확인한다.
   - 카메라 장착 높이와 각도별로 바닥 물체가 잘 보이는지 확인한다.

2. 최종 위험물 데이터 확보
   - 현재 레고 세부 class 중심인 모델 검증을 `battery`, `coin`, `lego` 기준으로 확장한다.
   - 각 class별 테스트 이미지 또는 짧은 영상을 확보해 결과표를 작성한다.

3. threshold 튜닝
   - `confidence_threshold`를 0.2, 0.3, 0.4, 0.5 등으로 비교한다.
   - `small_object_fallback_max_area_ratio`와 `mouth_threshold_scale`을 실제 카메라 환경에 맞게 조정한다.
   - `stable_detection_frames`를 조정해 오탐과 반응 속도의 균형을 맞춘다.

4. 안전 행동 평가
   - mAP뿐 아니라 실제 안전 행동 기준의 평가표를 만든다.
   - 평가 항목은 위험 class 매핑 정확도, 긴급정지 미탐률, 긴급정지 오탐률, 1순위 목표 정확도 등으로 구성한다.

5. ROS2 후속 파이프라인 연동
   - `/detections`의 픽셀 bbox를 지면 또는 지도 좌표로 변환하는 노드가 필요하다.
   - 변환된 위험물 좌표를 target manager, navigation, 알림, 수거 장치와 연결해야 한다.

## 8. 중간보고 결론

비전 파트는 현재 YOLO 객체 탐지, 위험도 DB, class alias, 미등록 물체 판단, MediaPipe 기반 입/손 근접 판단, ROS2 메시지 발행까지 핵심 소프트웨어 구조가 구현된 상태이다. 또한 모델 검증 리포트와 위험 판단 단위 점검 스크립트가 있어 코드 동작을 일부 정량적으로 확인할 수 있다.

남은 핵심 과제는 실제 로봇 카메라 환경에서 모델과 threshold를 검증하고, 비전 결과를 지면 좌표 및 로봇 행동으로 연결하는 것이다. 따라서 중간 단계 기준으로는 "비전 판단 로직과 ROS2 출력 인터페이스 구현은 완료, 실제 환경 검증과 로봇 행동 연동은 진행 예정"이라고 정리할 수 있다.
