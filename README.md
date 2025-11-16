# ws_diff_drive ROS2 Workspace

## 1. 프로젝트 개요

`ws_diff_drive` 워크스페이스는 차동 구동 로봇의 실시간 SLAM(동시적 위치 추정 및 지도 작성)을 위한 ROS2 프로젝트입니다. Python으로 작성된 노드와 표준 ROS2 패키지를 조합하여, LiDAR 센서 데이터와 휠 오도메트리 데이터를 융합하는 강건한(robust) 시스템을 구축합니다.

- **핵심 기능**: LiDAR 스캔 데이터와 로봇 자체의 주행 기록(오도메트리)을 `slam_toolbox`를 통해 결합하여, 환경 지도를 생성하고 그 안에서 로봇의 위치를 추정합니다.
- **기술 스택**: ROS2(Python), `sllidar_ros2`, `slam_toolbox

## 2. 시스템 아키텍처 및 데이터 흐름

시스템은 다음과 같은 순서로 데이터를 처리하고 발행합니다.

1. **`/wheel/{left,right}/odometry` (Odometry)**: 외부의 휠 엔코더 노드(가정)가 각 바퀴의 속도 정보를 담은 오도메트리 메시지를 발행합니다.
2. **`fusion_node.py`**:
    - `1`번의 두 토픽을 **구독(Subscribe)**합니다.
    - 두 바퀴의 속도를 융합하여 로봇의 선속도(`v`)와 각속도(`w`)를 계산합니다.
    - 이를 기반으로 로봇의 위치와 자세(`x`, `y`, `yaw`)를 계산(적분)합니다.
    - 계산된 위치/자세/속도 정보를 `nav_msgs/msg/Odometry` 메시지 형태로 `/odom` 토픽에 **발행(Publish)**합니다.
    - 동일한 위치/자세 정보를 `odom` → `base_link` 좌표 변환(TF)으로 **브로드캐스트(Broadcast)**합니다.
3. **`sllidar_node`**:
    - LiDAR 센서로부터 2D 스캔 데이터를 수신합니다.
    - `sensor_msgs/msg/LaserScan` 메시지 형태로 `/scan` 토픽에 **발행**합니다.
4. **`static_transform_publisher`**:
    - 로봇의 기준 좌표계(`base_link`)와 LiDAR 센서의 좌표계(`laser`) 사이의 고정된 관계를 TF로 **브로드캐스트**합니다.
5. **`slam_toolbox`**:
    - `/scan` 토픽(LiDAR 데이터)과 `/odom` 토픽(오도메트리)을 **구독**합니다.
    - TF 트리로부터 `odom`→`base_link`와 `base_link`→`laser` 변환 정보를 모두 수신합니다.
    - 모든 정보를 종합하여 지도를 생성하고, 오차를 보정한 로봇의 최종 위치를 `map` → `odom` TF로 **브로드캐스트**합니다.

## 3. 컴포넌트 심층 분석

### 3.1. 오도메트리 융합 노드 (`fusion_node.py`)

이 노드는 차동 구동 로봇의 운동학 모델을 기반으로 양쪽 바퀴의 속도 정보를 통합하여 로봇 전체의 오도메트리를 계산합니다.

#### 3.1.1 코드 분석

- **클래스**: `DiffFusion(Node)`
- **구독 (Subscribers)**:
  - `/wheel/left/odometry` (`Odometry`): 왼쪽 바퀴의 오도메트리 정보 수신. `cb_left` 콜백 함수 호출.
  - `/wheel/right/odometry` (`Odometry`): 오른쪽 바퀴의 오도메트리 정보 수신. `cb_right` 콜백 함수 호출.
- **발행 (Publishers)**:
  - `/odom` (`Odometry`): 융합된 최종 오도메트리 메시지를 발행.
  - TF (`TransformBroadcaster`): `odom` → `base_link` 좌표 변환을 발행.
- **핵심 로직 (`on_timer`)**: 50Hz 주기로 실행
    1. **시간 간격(`dt`) 계산**: 마지막 호출 이후 경과된 시간을 계산합니다.
    2. **속도 융합**:
        - `v = 0.5 * (self.v_l + self.v_r)`: 로봇의 선속도(v)는 왼쪽과 오른쪽 바퀴 속도의 평균으로 계산합니다.
        - `w = (self.v_r - self.v_l) / self.wheel_base`: 로봇의 각속도(w)는 두 바퀴의 속도 차이를 바퀴 간의 거리(`wheel_base`)로 나누어 계산합니다. 이는 차동 구동 로봇의 기본 운동학 모델입니다.
    3. **상태 적분 (Pose Integration)**:
        - `self.yaw += w * dt`: 현재 각도(yaw)에 `각속도 * 시간`을 더해 새로운 각도를 계산합니다.
        - `self.x += v * cy * dt`, `self.y += v * sy * dt`: 계산된 새 각도를 바탕으로 로봇의 전진 속도를 x, y 성분으로 분해하여 현재 위치에 더합니다.
    4. **메시지 생성 및 발행**: 계산된 `x, y, yaw`와 `v, w`를 `Odometry` 메시지와 `TransformStamped` 메시지에 담아 각각 `/odom` 토픽과 TF로 발행합니다.

### 3.2. 실행 파일 (`lidar_slam.launch.py`)

이 파일은 `LaunchDescription`을 사용하여 전체 SLAM 시스템을 구성하는 모든 노드를 실행하고 파라미터를 설정합니다.

#### 3.2.1 코드 분석

- **`LaunchConfiguration`**: `serial_port`, `frame_id` 등 런치 시점에 변경할 수 있는 변수를 선언합니다. 사용자가 `ros2 launch ... serial_port:=/dev/ttyUSB1`과 같이 값을 바꿀 수 있게 해줍니다.
- **`DeclareLaunchArgument`**: `LaunchConfiguration`으로 선언된 변수들의 기본값을 설정합니다.
- **`Node`**: 실제 노드를 실행하는 액션입니다.
  - **`sllidar_node`**: `sllidar_ros2` 패키지의 `sllidar_node` 실행 파일을 실행합니다. `parameters`를 통해 시리얼 포트, 프레임 ID 등 LiDAR 드라이버의 동작 방식을 설정합니다.
  - **`static_transform_publisher`**: `tf2_ros` 패키지의 `static_transform_publisher`를 실행합니다. `arguments`로 `[x, y, z, roll, pitch, yaw, 부모_프레임, 자식_프레임]`을 전달하여 `base_link` 대비 `laser`의 상대 위치를 지정합니다.
  - **`fusion_node`**: `diff_drive_fusion` 패키지의 `fusion_node` 실행 파일(실제로는 `fusion_node.py`)을 실행합니다. `parameters`를 통해 로봇의 바퀴 간 거리(`wheel_base`) 등을 설정합니다.
  - **`slam_toolbox`**: `slam_toolbox` 패키지의 `sync_slam_toolbox_node`를 실행합니다. `parameters`에 `slam_toolbox.yaml` 파일의 경로를 전달하여 SLAM 알고리즘의 상세 설정을 적용합니다.

### 3.3. SLAM 설정 (`slam_toolbox.yaml`)

이 파일은 `slam_toolbox`의 동작을 제어하는 핵심 파라미터들을 정의합니다.

| 파라미터                      | 값        | 설명                                                                                                                            |
| ----------------------------- | ---------- | ------------------------------------------------------------------------------------------------------------------------------- |
| `odom_frame`                  | `"odom"`   | 오도메트리 기준 좌표계 이름입니다.                                                                                              |
| `map_frame`                   | `"map"`    | SLAM에 의해 생성되는 지도 기준 좌표계 이름입니다.                                                                               |
| `base_frame`                  | `"base_link"` | 로봇 본체의 기준 좌표계 이름입니다.                                                                                             |
| `scan_topic`                  | `"/scan"`  | 구독할 LiDAR 스캔 데이터 토픽 이름입니다.                                                                                       |
| `mode`                        | `"mapping"` | 실시간으로 지도를 생성하는 '매핑' 모드로 동작합니다. (반대: `localization` 모드)                                              |
| `resolution`                  | `0.05`     | 지도의 격자(grid) 한 칸의 크기를 0.05m (5cm)로 설정합니다. 값이 작을수록 지도가 정밀해지지만 계산량이 증가합니다.             |
| `max_laser_range`             | `8.0`      | 8m보다 먼 거리의 LiDAR 측정값은 무시합니다. 노이즈나 불필요한 데이터를 필터링합니다.                                           |
| `minimum_travel_distance`     | `0.10`     | 로봇이 10cm 이상 이동해야만 지도 업데이트를 위한 스캔 매칭을 시도합니다. 계산 부하를 줄입니다.                                 |
| `minimum_travel_heading`      | `0.10`     | 로봇이 0.1 라디안(약 5.7도) 이상 회전해야만 스캔 매칭을 시도합니다.                                                             |
| `map_update_interval`         | `1.0`      | 1초 간격으로 지도 정보를 업데이트하여 발행합니다.                                                                               |
| `use_scan_matching`           | `true`     | 새로운 스캔 데이터를 기존 지도와 정렬하는 스캔 매칭 알고리즘을 사용합니다. SLAM의 핵심 기능입니다.                            |
| `use_loop_closure`            | `true`     | 루프 클로저(Loop Closure) 기능을 활성화합니다.                                                                                  |
| `do_loop_closing`             | `true`     | 실제 루프 클로저 최적화를 수행합니다.                                                                                           |
| `loop_search_maximum_distance`| `3.0`      | 현재 위치로부터 3m 반경 내에서 이전에 방문했던 영역이 있는지 탐색하여 루프 클로저를 시도합니다.                               |
