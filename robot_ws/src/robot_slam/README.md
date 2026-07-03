# robot_slam

라이다(`/scan`) + 휠 엔코더(`/wheel/odom`) 기반 SLAM/로컬라이제이션과, 파이프라인 전체가 공유하는
`map -> odom -> base_link -> {laser_link, camera_link}` TF 트리를 담당하는 패키지입니다.
Nav2 planner/controller는 이 패키지가 아니라 `robot_navigation`에 있습니다.

## 왜 이렇게 구성했는가 (dual-EKF 구조)

`robot_localization` EKF 한 개만으로 "wheel odom + slam pose"를 그대로 섞으면, 두 입력의
기준 좌표계(odom vs map)가 달라 잘못된 융합이 됩니다. 그래서 GPS 융합에 쓰이는
`robot_localization`의 표준 "dual EKF" 패턴을 그대로 차용해, SLAM pose를 GPS 자리에 대입했습니다.

```
/wheel/odom ─────────────► ekf_filter_node_odom (world_frame=odom)
                                 │
                                 ├─► odom -> base_link TF
                                 └─► /odometry/filtered ─────────┐
                                                                  ▼
slam_toolbox(localization, TF 발행 끔) ─► /pose ───────► ekf_filter_node_map (world_frame=map)
                                                                  │
                                                                  ├─► map -> odom TF
                                                                  └─► /odometry/filtered_map
```

- **mapping 모드**: `slam_toolbox`가 직접 `map -> odom`을 발행합니다(표준 동작 그대로).
  `ekf_filter_node_map`은 실행하지 않습니다.
- **localization 모드**: `slam_toolbox_localization.yaml`에서 `transform_publish_period: 0.0`으로
  설정해 slam_toolbox가 TF를 직접 쏘지 않게 하고, 대신 `/pose`만 내보내게 한 뒤
  `ekf_filter_node_map`이 이를 절대좌표 보정으로 융합해 `map -> odom`을 발행합니다.
  이렇게 하면 최종 파이프라인(Nav2 costmap, goal_manager 등)이 항상 매끄러운(jump-free) EKF 출력을
  구독할 수 있습니다.

## TF 트리

```
map ─► odom ─► base_link ─┬─► laser_link
                          └─► camera_link ─► camera_optical_frame
```

`base_link -> laser_link`, `base_link -> camera_link`, `camera_link -> camera_optical_frame`은
`launch/static_tf.launch.py`에서 `tf2_ros static_transform_publisher`로 발행합니다.
로봇 URDF가 아직 없어서(하드웨어 미확정) static TF로 대체했습니다 — 실측 후 URDF/xacro로
승격하는 것을 권장합니다.

`camera_link` 관련 수치(장착 높이, 틸트각)는 여기서는 TF 트리 표시 목적일 뿐이며, 실제
바운딩박스→지면좌표 계산은 `robot_perception/ground_projection_node`가 자신의
`config/camera_extrinsics.yaml`을 직접 읽어 계산합니다 (라이다=회피, 카메라=탐지 역할 분리 원칙,
최상위 README 참고). **실측값을 넣을 때는 두 config를 함께 갱신하세요.**

## 실행법

```bash
# 1) 지도 제작 (최초 1회, 로봇을 조이스틱/텔레옵으로 돌아다니게 한 뒤 map_saver로 저장)
ros2 launch robot_slam slam_mapping.launch.py
ros2 run nav2_map_server map_saver_cli -f ~/maps/room
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
    "{filename: '/home/<user>/maps/room'}"

# 2) 이후 상시 주행 (로컬라이제이션 모드) - full_pipeline.launch.py가 내부적으로 이걸 사용
ros2 launch robot_slam slam_localization.launch.py map_file_name:=/home/<user>/maps/room
```

## 파라미터 (기본값과 근거)

| 파라미터 | 기본값 | 근거 |
|---|---|---|
| `slam_toolbox` `max_laser_range` | 12.0 m | 라이다 모델 미정 → 실내 2D 라이다 통상 스펙의 보수적 기본값. 실제 모델 확정 시 datasheet 값으로 교체 |
| `slam_toolbox` `resolution` | 0.05 m | 실내 소형 로봇 SLAM 통상값 |
| `static_tf` `laser_mount_height` | 0.05 m | "바퀴보다 살짝 위" 요구사항의 임의 기본값 |
| `static_tf` `camera_mount_height` | 0.35 m | "로봇 최상단" 요구사항의 임의 기본값 |
| `static_tf` `camera_tilt_deg` | 30.0 deg | 지면이 보이도록 아래로 기울인 임의 기본값 |
| EKF `frequency` | 30.0 Hz | 실내 저속 주행 기준 통상값 |

모두 `launch` 인자 또는 `config/*.yaml`로 분리되어 있어 하드코딩 없이 실측값으로 교체 가능합니다.

## 토픽

| 토픽 | 타입 | 방향 | 설명 |
|---|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | 구독 | 라이다 드라이버 (기 존재 가정) |
| `/wheel/odom` | `nav_msgs/Odometry` | 구독 | 엔코더 오도메트리 (기 존재 가정, 없으면 `robot_bringup`의 mock 사용) |
| `/pose` | `geometry_msgs/PoseWithCovarianceStamped` | 구독(ekf_map) / 발행(slam_toolbox) | localization 모드에서만 사용 |
| `/odometry/filtered` | `nav_msgs/Odometry` | 발행 | odom 인스턴스 EKF 출력 |
| `/odometry/filtered_map` | `nav_msgs/Odometry` | 발행 | map 인스턴스 EKF 출력 (localization 모드) |
