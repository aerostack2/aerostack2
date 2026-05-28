# RViz 2드론 시각화 확인 사항

## 실행 명령

```bash
LIBGL_ALWAYS_SOFTWARE=1 ros2 launch as2_visualization swarm_viz.launch.py \
  namespace_list:=drone0,drone1 \
  use_sim_time:=false
```

> `namespace_list:=drone0`만 지정하면 drone1의 시각화 노드가 실행되지 않음.  
> 반드시 `drone0,drone1` 둘 다 포함해야 한다.

---

## 실행되는 노드 구조

| 노드 | drone0 | drone1 |
|---|---|---|
| `rviz2` | O (1개만 실행) | X |
| `robot_state_publisher` | O | O |
| `marker_publisher` | O | O |

`swarm_viz.launch.py`는 첫 번째 네임스페이스에만 RViz를 실행하고, 이후 네임스페이스는 `rviz: false`로 실행한다.

---

## 기본 RViz 설정의 한계

`as2_default.rviz`는 drone0만 하드코딩되어 있다:

- `RobotModel` → `/drone0/robot_description`, `TF Prefix: drone0`
- `PosesHistory` → `/viz/drone0/last_poses`
- `ReferencePose` → `/viz/drone0/reference_pose`
- `ReferenceVel` → `/viz/drone0/vel`
- `Fixed Frame: earth`

drone1의 시각화 노드는 실행되지만 RViz가 해당 토픽을 구독하지 않으므로 화면에 표시되지 않는다.

---

## drone1 수동 추가 방법 (RViz UI)

`namespace_list:=drone0,drone1`으로 실행 후 RViz에서 아래 항목을 Add:

### RobotModel (3D 모델)
```
Add → By display type → RobotModel
  Description Source: Topic
  Description Topic: /drone1/robot_description
  TF Prefix: drone1
```

### 이동 경로
```
Add → By display type → Marker
  Topic: /viz/drone1/last_poses
```

### 목표 위치 마커
```
Add → By display type → Marker
  Topic: /viz/drone1/reference_pose
```

### 속도 벡터
```
Add → By display type → Marker
  Topic: /viz/drone1/vel
```

---

## 커스텀 rviz_config 사용 방법

`as2_default.rviz`를 복사하여 drone1 항목을 추가한 파일을 만든 후:

```bash
LIBGL_ALWAYS_SOFTWARE=1 ros2 launch as2_visualization swarm_viz.launch.py \
  namespace_list:=drone0,drone1 \
  use_sim_time:=false \
  rviz_config:=/path/to/my_swarm.rviz
```

---

## 사전 조건 확인

### as2_gazebo_assets 패키지

`robot_state_publisher`는 `as2_gazebo_assets` 패키지에서 SDF 모델을 로드한다:

```bash
ros2 pkg list | grep as2_gazebo_assets
```

패키지가 없으면 `robot_state_publisher`가 실패하고 RobotModel 표시가 불가능하다.  
단, TF와 Marker 기반 시각화는 정상 동작한다.

### 토픽 발행 확인

```bash
# 시각화 노드 토픽 확인
ros2 topic list | grep viz

# robot_description 확인
ros2 topic echo /drone1/robot_description --once

# marker_publisher 확인
ros2 topic hz /viz/drone1/last_poses
```

### TF 트리 확인

```bash
ros2 run tf2_tools view_frames
# earth → drone0/map → drone0/odom → drone0/base_link
# earth → drone1/map → drone1/odom → drone1/base_link
```

---

## WSL 환경 주의사항

- `LIBGL_ALWAYS_SOFTWARE=1`: WSL2 GPU 가속 미지원 시 소프트웨어 렌더링 강제 (필수)
- RViz 시작 직후 TF 경고(`two or more unconnected trees`)가 일시적으로 출력될 수 있음 → 정상, state_estimator의 TF 전달 완료 후 사라짐
