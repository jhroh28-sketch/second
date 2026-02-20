# hospital_mission

ROS2 Humble + Nav2 환경에서 동작하는 단순 미션 컨트롤러(ament_python) 패키지입니다.  
Isaac Sim에서 `/tf`, `/odom`, `/scan`, `/cmd_vel`이 제공되는 환경을 가정하고, 일반 노트북에서 `nav2_simple_commander` 기반으로 미션을 자동 수행합니다.

## 1) 빌드

```bash
cd ~/your_ws/src
# 이 저장소를 src 아래에 배치했다고 가정
cd ..
colcon build --packages-select hospital_mission
source install/setup.bash
```

## 2) Nav2 실행 (별도 터미널)

아래는 예시입니다. 실제 로봇/시뮬레이터 설정에 맞는 bringup을 사용하세요.

```bash
source ~/your_ws/install/setup.bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=true
```

## 3) 미션 컨트롤러 실행

```bash
source ~/your_ws/install/setup.bash
ros2 run hospital_mission mission_controller --ros-args -p use_sim_time:=true
```

필요 시 YAML 경로/응급복귀 상태를 파라미터로 지정할 수 있습니다.

```bash
ros2 run hospital_mission mission_controller --ros-args \
  -p use_sim_time:=true \
  -p poses_file:=/absolute/path/to/poses.yaml \
  -p emergency_return_state:=IDLE
```

## 4) 토픽 인터페이스

### 구독
- `/mission_cmd` (`std_msgs/String`)
  - `start`: 미션 시작 (`IDLE -> GO_TO_STORAGE`)
  - `pick_done`: 픽업 완료 (`WAIT_PICK_DONE -> GO_TO_PATIENT`)
  - `drop_done`: 드롭 완료 (`WAIT_DROP_DONE -> DONE`)
  - `reset`: 진행중이면 취소 후 `IDLE`
- `/emergency` (`std_msgs/Bool`)
  - `true`: 즉시 현재 task 취소 + `EMERGENCY` 상태 전환 + emergency pose로 이동

### 발행
- `/mission_status` (`std_msgs/String`)
  - 예: `state=WAIT_PICK_DONE; nav_in_progress=False`

## 5) 테스트 명령 예시

```bash
ros2 topic pub /mission_cmd std_msgs/String "{data: start}" --once
ros2 topic pub /mission_cmd std_msgs/String "{data: pick_done}" --once
ros2 topic pub /mission_cmd std_msgs/String "{data: drop_done}" --once
ros2 topic pub /emergency std_msgs/Bool "{data: true}" --once
ros2 topic pub /mission_cmd std_msgs/String "{data: reset}" --once
```

상태 확인:

```bash
ros2 topic echo /mission_status
```

## 6) poses.yaml 설정

`config/poses.yaml`에서 모든 좌표를 관리합니다 (코드 하드코딩 없음).

필수 필드:
- `initial_pose: {x, y, yaw}`
- `storage_pose: {x, y, yaw}`
- `patient_pose: {x, y, yaw}`
- `emergency_pose: {x, y, yaw}`
- 선택: `through_poses: [ {x,y,yaw}, ... ]`
- 선택: `yaw_unit: rad | deg` (`deg`로 적으면 내부에서 rad로 변환)

## 7) 디버깅 체크리스트

1. 시간 동기화
   - `use_sim_time`가 Nav2와 mission_controller 모두에서 `true`인지 확인
2. 필수 토픽
   - `/tf`, `/odom`, `/scan`, `/amcl_pose`가 정상 발행되는지 확인
3. Nav2 lifecycle
   - Nav2 노드가 active 상태인지 확인
   - `bt_navigator`, `controller_server`, `planner_server`, `amcl` 등
4. 초기 자세
   - YAML의 `initial_pose`가 맵 기준으로 유효한지 확인
5. 목표점 유효성
   - 장애물 내부/맵 외부 좌표가 아닌지 확인

## 8) 상태머신 요약

`IDLE -> GO_TO_STORAGE -> WAIT_PICK_DONE -> GO_TO_PATIENT -> WAIT_DROP_DONE -> DONE`

- `EMERGENCY`는 어느 상태에서든 인터럽트 가능
- Emergency goal 도착 후 `emergency_return_state` 설정(`IDLE` 또는 `DONE`)으로 복귀

