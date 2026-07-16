# comet_ekf_replay

ArduPilot DataFlash `.BIN`의 EKF3 상태를 ROS 2 Humble 메시지로 원래 로그 시간에 맞춰 재생하고 RViz2에 표시하는 `ament_python` 패키지입니다. Ubuntu 22.04, Python 3.10, ROS 2 Humble을 기준으로 합니다.

이 패키지는 정상 동작이 확인된 저장소 루트의 `bin_ekf_replay`를 파싱 코어로 import합니다. 파서를 복사하지 않으며 의존 방향은 항상 다음과 같습니다.

```text
comet_ekf_replay  →  bin_ekf_replay
```

기존 Matplotlib CLI와 시각화에는 ROS 의존성이 추가되지 않습니다. 이 재생 결과는 실제 참값이 아니라 **ArduPilot EKF의 추정 결과**입니다.

## ROS 2 및 Python 의존성

ROS 시스템 Python을 사용하십시오. 기존 `bin_ekf_replay/.venv`에서 ROS 노드를 실행하지 않습니다.

```bash
source /opt/ros/humble/setup.bash
sudo apt update
sudo apt install python3-pip python3-numpy python3-pandas python3-scipy
python3 -m pip install --user pymavlink
python3 -m pip install --user --no-deps -e ~/Workspace/PX4_comet/bin_ekf_replay
```

마지막 명령은 기존 파서를 `bin_ekf_replay` Python package로 editable 설치할 뿐, 기존 코드나 CLI 동작을 바꾸지 않습니다. 시스템에 `python3-pymavlink` 패키지가 제공되면 apt 설치를 사용해도 됩니다.

## 빌드

```bash
cd ~/Workspace/PX4_comet/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select comet_ekf_replay --symlink-install
source install/setup.bash
```

## 실행

BIN에는 절대경로 사용을 권장합니다.

```bash
ros2 launch comet_ekf_replay bin_ekf_replay.launch.py \
  bin_path:=$HOME/Workspace/PX4_comet/LOGS/00000200.BIN
```

RViz 없이 노드만 확인하려면:

```bash
ros2 launch comet_ekf_replay bin_ekf_replay.launch.py \
  bin_path:=$HOME/Workspace/PX4_comet/LOGS/00000200.BIN \
  use_rviz:=false
```

직접 실행할 수도 있습니다.

```bash
ros2 run comet_ekf_replay bin_ekf_replay_node --ros-args \
  -p bin_path:=$HOME/Workspace/PX4_comet/LOGS/00000200.BIN
```

## 토픽과 frame

RViz Fixed Frame은 `map`이며 노드는 `map → rocket` TF를 발행합니다.

| 토픽 | 형식 | 내용 |
|---|---|---|
| `/bin_ekf/odometry` | `nav_msgs/Odometry` | ENU 위치·자세·선속도 |
| `/bin_ekf/pose` | `geometry_msgs/PoseStamped` | 현재 ENU pose |
| `/bin_ekf/path` | `nav_msgs/Path` | 호환용 최근 trail |
| `/bin_ekf/full_path` | `nav_msgs/Path` | 전체 궤적, transient-local 및 저주기 재발행 |
| `/bin_ekf/trail_path` | `nav_msgs/Path` | 최근 `trail_seconds` 궤적 |
| `/bin_ekf/current_marker` | `visualization_msgs/Marker` | 현재 위치 sphere |
| `/bin_ekf/velocity_marker` | `visualization_msgs/Marker` | ENU 속도 화살표 |
| `/bin_ekf/body_axes` | `visualization_msgs/MarkerArray` | body X/Y/Z의 red/green/blue 화살표 |
| `/bin_ekf/status_marker` | `visualization_msgs/Marker` | RViz health/status 텍스트 |
| `/bin_ekf/status_text` | `std_msgs/String` | 동일 상태의 일반 문자열 |
| `/bin_ekf/current_index` | `std_msgs/UInt32` | 현재 정렬 상태 index |
| `/bin_ekf/replay_time` | `std_msgs/Float64` | 현재 로그 상대시간 [s] |

Odometry는 `header.frame_id=map`, `child_frame_id=rocket`입니다. 전체 Path pose는 시작 시 한 번 계산해 재사용하고, trail만 제한된 구간으로 발행하므로 전체 Path가 callback마다 다시 생성되거나 무한히 커지지 않습니다.

## 좌표계와 quaternion

기존 파서의 ArduPilot NED 상태를 RViz ENU로 변환합니다.

```text
x = PE       vx = VE
y = PN       vy = VN
z = -PD      vz = -VD
```

기존 파서가 정규화한 scalar-first `qw,qx,qy,qz`를 사용한 뒤 NED→ENU 회전을 적용하고, ROS 메시지에는 `x,y,z,w` 순서로 기록합니다. 기본값은 정상 확인된 Matplotlib 설정과 동일하게 conjugate 비활성, model offset identity입니다.

축이 실제 기체 방향과 반대로 보이면 다음 parameter를 비교하십시오.

```bash
ros2 param set /bin_ekf_replay use_quaternion_conjugate true
ros2 param set /bin_ekf_replay model_axis_offset_rpy_deg "[0.0, 0.0, 180.0]"
```

빨강 body X는 전방, 초록 Y는 우측, 파랑 Z는 아래 방향입니다. 비행 방향과 Roll/Pitch/Yaw를 함께 보고 convention을 검증해야 합니다.

## Parameter

| 이름 | 기본값 | 설명 |
|---|---:|---|
| `bin_path` | 빈 문자열 | 필수 BIN 경로 |
| `core` | `0` | EKF core |
| `playback_speed` | `1.0` | 로그 시간 재생 배속 |
| `publish_rate` | `30.0` | ROS timer/publish 빈도 [Hz] |
| `trail_seconds` | `30.0` | 최근 Path 길이 [s] |
| `match_tolerance` | `0.05` | XKQ/XKF4 nearest 매칭 오차 [s] |
| `use_quaternion_conjugate` | `false` | quaternion conjugate 적용 |
| `model_axis_offset_rpy_deg` | `[0,0,0]` | body/model 고정 XYZ Euler offset [deg] |
| `loop` | `false` | 끝에서 처음으로 반복 |
| `start_paused` | `false` | 첫 상태에서 일시정지 시작 |

`bin_path`, `core`, `match_tolerance`, `start_paused`는 노드 재시작 시 적용합니다. `playback_speed`, `publish_rate`, `trail_seconds`, quaternion 옵션과 `loop`는 실행 중 변경할 수 있습니다. 배속 변경은 현재 로그 위치를 기준점으로 다시 고정하므로 timer jitter나 배속 변경으로 재생 위치가 갑자기 점프하지 않습니다.

## 재생 제어

```bash
ros2 service call /bin_ekf/pause std_srvs/srv/SetBool "{data: true}"
ros2 service call /bin_ekf/pause std_srvs/srv/SetBool "{data: false}"
ros2 service call /bin_ekf/reset std_srvs/srv/Trigger "{}"
ros2 param set /bin_ekf_replay playback_speed 2.0
```

reset은 첫 샘플로 이동하고 재생을 다시 시작합니다.

## 확인 명령

```bash
ros2 topic list
ros2 topic echo /bin_ekf/odometry --once
ros2 topic echo /bin_ekf/replay_time --once
ros2 service list
ros2 param get /bin_ekf_replay playback_speed
```

## 알려진 한계

- quaternion 방향은 ArduPilot firmware와 기체 축 정의에 따라 실제 비행 방향과 대조 검증해야 합니다.
- XKF4 health 값은 표시 및 진단에만 사용하며 위치 계산에는 사용하지 않습니다. 결측값은 `n/a`로 표시합니다.
- ORGN을 이용한 WGS84 지도 좌표 변환, STL, Gazebo, Unity, rosbag 변환은 포함하지 않습니다.
- 여러 EKF core 비교, LoRa/SiK 수신, 자체 KF/EKF는 구현하지 않습니다.
- RViz Marker와 Path는 진단용 시각화이며 ground truth를 의미하지 않습니다.

향후 LoRa EKF 및 SiK 자체 EKF 입력도 동일한 `nav_msgs/Odometry` 중심 인터페이스로 연결할 수 있도록 출력 토픽을 입력 파서와 분리했습니다.
