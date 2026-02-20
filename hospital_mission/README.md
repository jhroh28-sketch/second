# hospital_mission

ROS2 Humble + Nav2 환경에서 동작하는 단순 미션 컨트롤러 패키지입니다.

## 1) 빌드

```bash
cd ~/your_ws
colcon build --packages-select hospital_mission
source install/setup.bash
```

## 2) Nav2 bringup (별도 터미널)

아래는 예시이며, 실제 로봇/시뮬레이터 설정에 맞는 bringup을 사용하세요.

```bash
source ~/your_ws/install/setup.bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=true
```

## 3) 미션 컨트롤러 실행

```bash
source ~/your_ws/install/setup.bash
ros2 run hospital_mission mission_controller --ros-args -p use_sim_time:=true
```

필요하면 포즈 파일을 바꿔서 실행할 수 있습니다.

```bash
ros2 run hospital_mission mission_controller --ros-args \
  -p use_sim_time:=true \
  -p poses_file:=/absolute/path/to/poses.yaml \
  -p emergency_return_state:=IDLE
```

## 4) 토픽 테스트

```bash
ros2 topic pub /mission_cmd std_msgs/String "{data: start}"
ros2 topic pub /mission_cmd std_msgs/String "{data: pick_done}"
ros2 topic pub /mission_cmd std_msgs/String "{data: drop_done}"
ros2 topic pub /emergency std_msgs/Bool "{data: true}"
ros2 topic pub /mission_cmd std_msgs/String "{data: reset}"
```

상태 확인:

```bash
ros2 topic echo /mission_status
```

## 5) 상태 흐름

기본 상태 흐름:

`IDLE -> GO_TO_STORAGE -> WAIT_PICK_DONE -> GO_TO_PATIENT -> WAIT_DROP_DONE -> DONE`

비상 상태:

`(any) -> EMERGENCY -> (IDLE 또는 DONE)`

## 6) poses.yaml 규약

`config/poses.yaml`에서 모든 좌표를 관리합니다.

필수 키:

- `initial_pose: {x, y, yaw}`
- `storage_pose: {x, y, yaw}`
- `patient_pose: {x, y, yaw}`
- `emergency_pose: {x, y, yaw}`

선택 키:

- `yaw_unit: "rad" | "deg"`
- `through_poses: [ {x, y, yaw}, ... ]`

## 7) 디버깅 체크리스트

- `/tf`가 정상 갱신되는지 확인
- `/odom`, `/scan`이 수신되는지 확인
- 로컬라이제이션 사용 시 `/amcl_pose` 확인
- Nav2 lifecycle 노드들이 active 상태인지 확인
- `mission_controller` 로그에서 distance_remaining/결과 로그 확인
