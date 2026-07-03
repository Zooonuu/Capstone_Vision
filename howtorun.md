# How to Run

이 프로젝트는 두 개의 파트로 구성됩니다.

- `vision_before/` : ROS2와 무관하게 독립적으로 돌릴 수 있는 YOLO11 학습/검증/테스트 스크립트 (원본)
- `robot_ws/` : ROS2(colcon) 워크스페이스. `robot_perception`(YOLO 인식 노드), `robot_perception_msgs`(커스텀 메시지), `robot_slam`(라이다+엔코더 SLAM/로컬라이제이션)

## 0. 사전 준비

- ROS2 (Humble 이상, `rclpy` / `tf2_ros` / `cv_bridge` 포함)
- Python 패키지: `ultralytics`, `mediapipe`, `opencv-python`
- `slam_toolbox`, `robot_localization` (robot_slam 실행 시 필요)

```bash
pip install ultralytics mediapipe opencv-python
```

## 1. YOLO 모델 학습/검증 (ROS2 없이)

`data.yaml`이 있는 프로젝트 루트에서 실행합니다. (`vision_before/`의 스크립트와 `robot_ws/src/robot_perception/scripts/`의 스크립트는 동일 내용입니다.)

```bash
# 학습 (결과: runs/detect/lego_test_model/weights/best.pt)
python3 vision_before/train.py

# 학습된 모델 검증 (mAP50 / mAP50-95 출력)
python3 vision_before/val_model.py

# 정적 이미지 한 장으로 빠르게 확인
python3 vision_before/test_image.py <이미지_경로> [best.pt 경로]
```

학습이 끝나면 나온 `best.pt`를 `robot_ws/src/robot_perception/` 하위(또는 원하는 절대경로)에 복사해두면 ROS2 노드에서 그대로 씁니다.

## 2. ROS2 워크스페이스 빌드

```bash
cd robot_ws
colcon build --symlink-install
source install/setup.bash
```

## 3. 인식 노드 실행 (robot_perception)

`yolo_detector_node`는 `/image_raw`를 구독해 YOLO11 추론 + MediaPipe 포즈(입-손 근접 위험 판단)를 수행하고, `/detections`(DetectionArray, SLAM/Nav2용 픽셀 bbox)와 `vision/detected_objects`(원본 호환 JSON, 위험도/우선순위 포함)를 발행합니다.

```bash
ros2 run robot_perception yolo_detector_node --ros-args \
  --params-file src/robot_perception/config/yolo_params.yaml \
  -p model_path:=/absolute/path/to/best.pt
```

주요 파라미터 (`config/yolo_params.yaml`):

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `model_path` | `best.pt` | 학습된 가중치 경로 (절대경로 권장) |
| `target_classes` | `["battery", "coin", "lego"]` | `risk_db.py` 기준 관심 클래스 (자기 모델의 `model.names`와 맞춰야 함) |
| `device` | `cpu` | GPU 사용 시 `"0"` 등으로 변경 |
| `enable_visualization` | `true` | headless 환경(디스플레이 없음)이면 `false`로 설정 |

## 4. SLAM/로컬라이제이션 실행 (robot_slam)

```bash
# 최초 1회: 지도 제작 (조이스틱/텔레옵으로 주행 후 저장)
ros2 launch robot_slam slam_mapping.launch.py
ros2 run nav2_map_server map_saver_cli -f ~/maps/room

# 이후 상시 주행: 로컬라이제이션 모드
ros2 launch robot_slam slam_localization.launch.py map_file_name:=/home/<user>/maps/room
```

자세한 TF 구조(dual-EKF)와 파라미터 근거는 [robot_ws/src/robot_slam/README.md](robot_ws/src/robot_slam/README.md) 참고.
