# as2_behaviors_swarm_flocking 테스트 방법 및 드론 온보드 모니터링

---

## 1. 테스트 전제 조건

SwarmFlockingBehavior를 테스트하려면 드론 n대가 동시에 아래 스택을 실행 중이어야 한다.

```
[각 drone_n 마다 필수 실행 노드]

① Platform          → /drone_n/platform/info 발행
② State Estimator   → /drone_n/self_localization/pose, /twist 발행
                      /tf: earth→drone_n/base_link 동적 TF 발행
③ Controller        → /drone_n/motion_reference/pose 구독
                      /drone_n/controller/set_control_mode 서비스 제공
④ FollowReferenceBehavior → /drone_n/FollowReferenceBehavior Action Server 제공

[GCS 측]
⑤ SwarmFlockingBehavior → namespace: Swarm
```

---

## 2. 시뮬레이션 환경 실행 순서

### 2-1. Gazebo 시뮬레이션 (드론별)

각 드론마다 별도 터미널에서 실행:

```bash
# drone0
ros2 launch as2_platform_gazebo platform_gazebo_launch.py \
  namespace:=drone0 \
  use_sim_time:=true \
  simulation_config_file:=<world_config.yaml>

# drone1
ros2 launch as2_platform_gazebo platform_gazebo_launch.py \
  namespace:=drone1 \
  use_sim_time:=true \
  simulation_config_file:=<world_config.yaml>
```

### 2-2. 드론별 모션 Behavior 실행

각 드론마다 takeoff/land/go_to + **follow_reference** 를 함께 실행해야 한다.

```bash
# drone0 기본 Behavior (takeoff, land, go_to, follow_path)
ros2 launch as2_behaviors_motion motion_behaviors_launch.py \
  namespace:=drone0 \
  use_sim_time:=true

# drone0 FollowReference (별도 실행 필요)
ros2 launch as2_behaviors_motion follow_reference_behavior_launch.py \
  namespace:=drone0 \
  use_sim_time:=true

# drone1 동일
ros2 launch as2_behaviors_motion follow_reference_behavior_launch.py \
  namespace:=drone1 \
  use_sim_time:=true
```

> **주의**: `motion_behaviors_launch.py`는 `follow_reference`를 포함하지 않는다.
> `follow_reference_behavior_launch.py`를 **별도**로 실행해야 한다.
> (`motion_behaviors_launch.py` 포함 목록: follow_path, go_to, land, takeoff)

### 2-3. GCS 측 SwarmFlockingBehavior 실행

```bash
ros2 launch as2_behaviors_swarm_flocking swarm_flocking_behavior.launch.py \
  use_sim_time:=true
```

실행 후 확인:
```
namespace: Swarm
node name: SwarmFlockingBehavior
Action Server: /Swarm/SwarmFlockingBehavior
```

---

## 3. 미션 스크립트 (Python)

`as2_python_api`를 사용하는 직접 Action 호출 예시:

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from as2_msgs.action import SwarmFlocking
from as2_msgs.msg import PoseWithID
from geometry_msgs.msg import PoseStamped, Pose, Point


def main():
    rclpy.init()
    node = Node('swarm_mission')

    client = ActionClient(node, SwarmFlocking, '/Swarm/SwarmFlockingBehavior')
    client.wait_for_server()

    goal = SwarmFlocking.Goal()

    # 가상 무게중심: earth 기준 (0, 0, 5)
    goal.virtual_centroid = PoseStamped()
    goal.virtual_centroid.header.frame_id = 'earth'
    goal.virtual_centroid.pose.position.x = 0.0
    goal.virtual_centroid.pose.position.y = 0.0
    goal.virtual_centroid.pose.position.z = 5.0

    # 편대 오프셋 (Swarm 기준)
    drone0 = PoseWithID()
    drone0.id = 'drone0'
    drone0.pose = Pose()
    drone0.pose.position = Point(x=0.0, y=0.0, z=0.0)

    drone1 = PoseWithID()
    drone1.id = 'drone1'
    drone1.pose = Pose()
    drone1.pose.position = Point(x=2.0, y=0.0, z=0.0)

    goal.swarm_formation = [drone0, drone1]
    goal.drones_namespace = ['drone0', 'drone1']

    future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, future)
    goal_handle = future.result()

    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 편대 이동 (on_modify)

```python
# 이미 실행 중인 goal_handle에 새 virtual_centroid로 수정 요청
modify_msg = SwarmFlocking.Goal()
modify_msg.virtual_centroid.header.frame_id = 'earth'
modify_msg.virtual_centroid.pose.position.x = 5.0   # 5m 전진
modify_msg.virtual_centroid.pose.position.y = 0.0
modify_msg.virtual_centroid.pose.position.z = 5.0
# swarm_formation, drones_namespace 는 동일하게 유지

goal_handle.cancel_goal_async()  # 재전송 전 취소
# 또는 /Swarm/swarm_modify_srv 서비스 호출로 on_modify 트리거
```

### 동적 편대 변경 토픽 발행

```bash
ros2 topic pub /Swarm/dynamic_swarm_formation as2_msgs/msg/PoseWithIDArray \
  "poses:
  - id: 'drone1'
    pose:
      position:
        x: 0.0
        y: 2.0
        z: 0.0"
```

---

## 4. CLI 단계별 테스트

### 4-1. 필수 사전 상태 확인

```bash
# 각 드론 플랫폼 상태 확인 (FLYING 이어야 함)
ros2 topic echo /drone0/platform/info --once
ros2 topic echo /drone1/platform/info --once

# 출력에서 확인:
#   status:
#     state: 3   ← FLYING (3)
```

### 4-2. FollowReference Action Server 존재 확인

```bash
ros2 action list | grep FollowReference
# /drone0/FollowReferenceBehavior
# /drone1/FollowReferenceBehavior

ros2 action info /drone0/FollowReferenceBehavior
```

### 4-3. SwarmFlocking Action Server 존재 확인

```bash
ros2 action list | grep Swarm
# /Swarm/SwarmFlockingBehavior

ros2 action info /Swarm/SwarmFlockingBehavior
```

### 4-4. CLI로 SwarmFlocking 직접 실행

```bash
ros2 action send_goal /Swarm/SwarmFlockingBehavior as2_msgs/action/SwarmFlocking \
  "{
    virtual_centroid: {
      header: {frame_id: 'earth'},
      pose: {position: {x: 0.0, y: 0.0, z: 5.0}}
    },
    swarm_formation: [
      {id: 'drone0', pose: {position: {x: 0.0, y: 0.0, z: 0.0}}},
      {id: 'drone1', pose: {position: {x: 2.0, y: 0.0, z: 0.0}}}
    ],
    drones_namespace: ['drone0', 'drone1']
  }"
```

---

## 5. 드론 온보드 모니터링

### 5-1. TF 트리 확인 (가장 중요)

```bash
# TF 트리 전체 구조 확인
ros2 run tf2_tools view_frames

# 특정 프레임 간 transform 실시간 조회
ros2 run tf2_ros tf2_echo earth Swarm
ros2 run tf2_ros tf2_echo Swarm Swarm/drone0_ref
ros2 run tf2_ros tf2_echo earth Swarm/drone0_ref   # ← 실제 목표 좌표

# /tf_static 직접 수신 확인
ros2 topic echo /tf_static
```

**정상 상태 TF 트리:**
```
earth
  └─ drone0/base_link   (동적, State Estimator)
  └─ drone1/base_link   (동적, State Estimator)
  └─ Swarm              (Static, SwarmFlockingBehavior 발행)
       └─ Swarm/drone0_ref  (Static, DroneSwarm[drone0] 발행)
       └─ Swarm/drone1_ref  (Static, DroneSwarm[drone1] 발행)
```

### 5-2. 드론 위치 모니터링

```bash
# earth 기준 드론 현재 위치
ros2 topic echo /drone0/self_localization/pose
ros2 topic echo /drone1/self_localization/pose

# 현재 속도
ros2 topic echo /drone0/self_localization/twist
```

### 5-3. 위치 명령 모니터링 (FollowReference → Controller)

```bash
# FollowReference가 발행하는 motion_reference
# frame_id = "Swarm/drone0_ref", position = (0,0,0) 이어야 정상
ros2 topic echo /drone0/motion_reference/pose

# Hover 명령 (pause 또는 stop 시 발행됨)
ros2 topic echo /drone0/motion_reference/twist
```

### 5-4. 목표 거리 Feedback 모니터링

```bash
# FollowReference Action Feedback 실시간 확인
# actual_distance_to_goal < 0.3m 이면 DroneSwarm이 "도달 완료" 판정
ros2 action send_goal --feedback \
  /drone0/FollowReferenceBehavior \
  as2_msgs/action/FollowReference \
  "{target_pose: {header: {frame_id: 'Swarm/drone0_ref'}, point: {x:0,y:0,z:0}},
    yaw: {mode: 0}, max_speed_x: 5.0, max_speed_y: 5.0, max_speed_z: 5.0}"
```

### 5-5. 플랫폼 상태 모니터링

```bash
# 드론 상태: -1=EMERGENCY, 0=DISARMED, 1=LANDED, 2=TAKING_OFF, 3=FLYING, 4=LANDING
watch -n 0.5 "ros2 topic echo /drone0/platform/info --once 2>/dev/null | grep state"
watch -n 0.5 "ros2 topic echo /drone1/platform/info --once 2>/dev/null | grep state"
```

### 5-6. Controller 상태 모니터링

```bash
# Controller가 어떤 제어 모드로 동작 중인지 확인
ros2 topic echo /drone0/controller/info
ros2 topic echo /drone1/controller/info
```

### 5-7. SwarmFlocking Feedback 모니터링

```bash
# SwarmFlockingBehavior의 현재 편대 위치 feedback
# (현재 Action이 실행 중일 때만 수신됨)
ros2 topic echo /Swarm/SwarmFlockingBehavior/_action/feedback
```

---

## 6. RViz 시각화

### 6-1. 다중 드론 RViz 실행

```bash
ros2 launch as2_visualization swarm_viz.launch.py \
  namespace_list:=drone0,drone1 \
  use_sim_time:=true
```

첫 번째 네임스페이스(drone0)의 시각화 노드에서 RViz 창이 열리고,  
나머지 드론은 동일 RViz 세션에 추가된다.

### 6-2. RViz에서 확인 항목

| Display | 토픽 | 확인 내용 |
|---|---|---|
| TF | /tf, /tf_static | Swarm, drone_n_ref 프레임 위치 |
| Odometry | /drone_n/self_localization/pose | 드론 현재 위치 |
| PoseStamped | /drone_n/motion_reference/pose | 목표 위치 방향 |
| RobotModel | URDF (각 드론) | 드론 3D 모델 |

---

## 7. 단계별 동작 검증 체크리스트

```
□ 1. Gazebo 시뮬레이션 실행
     확인: 드론 모델이 월드에 생성됨

□ 2. 드론별 ARM + TAKEOFF
     확인: ros2 topic echo /drone_n/platform/info → state: 3 (FLYING)

□ 3. FollowReference 노드 실행 확인
     확인: ros2 action list | grep FollowReferenceBehavior

□ 4. SwarmFlockingBehavior 실행 확인
     확인: ros2 action list | grep SwarmFlockingBehavior

□ 5. SwarmFlocking Action 전송
     확인: /tf_static 에 earth→Swarm, Swarm→drone_n_ref TF 발행됨
     확인: ros2 run tf2_ros tf2_echo earth Swarm → 목표 위치 출력

□ 6. 드론 이동 확인
     확인: /drone_n/self_localization/pose 위치가 목표를 향해 변경됨
     확인: actual_distance_to_goal 이 점차 감소

□ 7. 편대 대형 확인
     확인: drone0과 drone1 간 거리가 goal의 formation_offset과 일치
     ros2 run tf2_ros tf2_echo Swarm/drone0_ref Swarm/drone1_ref

□ 8. 편대 이동 확인 (on_modify)
     확인: virtual_centroid.x 변경 후 모든 드론이 x 방향으로 이동
     확인: 드론 간 상대 거리 유지됨

□ 9. 동적 편대 변경 확인
     확인: /Swarm/dynamic_swarm_formation 발행 후 TF 재발행됨
     확인: 드론이 새 오프셋 위치로 이동
```

---

## 8. 주요 디버깅 명령어 요약

| 목적 | 명령어 |
|---|---|
| TF 트리 전체 보기 | `ros2 run tf2_tools view_frames` |
| 특정 TF 실시간 조회 | `ros2 run tf2_ros tf2_echo <parent> <child>` |
| Static TF 수신 확인 | `ros2 topic echo /tf_static` |
| 플랫폼 상태 확인 | `ros2 topic echo /drone_n/platform/info --once` |
| 목표 위치 명령 확인 | `ros2 topic echo /drone_n/motion_reference/pose` |
| 현재 위치 확인 | `ros2 topic echo /drone_n/self_localization/pose` |
| Action 서버 목록 | `ros2 action list` |
| FollowRef 즉시 중단 | `ros2 service call /drone_n/FollowReferenceBehavior/_behavior/stop std_srvs/srv/Trigger` |
| SwarmFlocking 중단 | `ros2 action cancel /Swarm/SwarmFlockingBehavior` |
| 노드 목록 확인 | `ros2 node list` |
| 토픽 발행 주파수 확인 | `ros2 topic hz /drone_n/self_localization/pose` |
