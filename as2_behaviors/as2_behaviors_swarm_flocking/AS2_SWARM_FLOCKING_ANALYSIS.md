# as2_behaviors_swarm_flocking 패키지 분석

## 1. 개요

`as2_behaviors_swarm_flocking`은 여러 대의 UAV를 **군집(Swarm)** 으로 편대 비행시키는 behavior 패키지다.  
핵심 원리는 **가상 무게중심(Virtual Centroid)** 과 **TF 기반 상대 위치 참조**를 결합한 Flocking이다.

- 패키지 버전: 1.1.3
- 작성자: Carmen De Rojas Pita-Romero (Universidad Politécnica de Madrid)
- ROS 2 Action 타입: `as2_msgs/action/SwarmFlocking`
- 내부적으로 각 드론은 `FollowReference` behavior를 통해 자신의 편대 위치를 추적한다

---

## 2. 아키텍처

```
SwarmFlockingBehavior 노드 (namespace: Swarm)
│
├── TF Static Broadcaster
│   └── [virtual_centroid_frame] → Swarm (가상 무게중심 TF)
│       └── drone0/Swarm_drone0_ref (각 드론의 편대 오프셋 TF)
│       └── drone1/Swarm_drone1_ref
│       └── ...
│
└── DroneSwarm 객체 (드론별)
    ├── /drone_id/self_localization/pose 구독 (현재 위치)
    └── /drone_id/FollowReferenceBehavior Action Client
        └── 목표 = 자신의 ref TF 프레임의 원점(0,0,0)
```

### 핵심 메커니즘

```
[earth 또는 지정 frame]
    │
    └── [Swarm] ← virtual_centroid offset (Static TF)
            │
            ├── [Swarm/drone0_ref] ← drone0 formation offset (Static TF)
            ├── [Swarm/drone1_ref] ← drone1 formation offset (Static TF)
            └── [Swarm/drone2_ref] ← drone2 formation offset (Static TF)

각 드론은 자신의 _ref TF 프레임 원점(0,0,0)을 FollowReference로 추적
→ Swarm TF가 이동하면 모든 드론이 함께 이동
```

---

## 3. 주요 클래스

### SwarmFlockingBehavior

`as2_behavior::BehaviorServer<as2_msgs::action::SwarmFlocking>`를 상속한 메인 클래스.

| 메서드 | 역할 |
|---|---|
| `on_activate()` | 편대 초기화 및 모든 드론 FollowReference 시작 |
| `on_run()` | 매 루프마다 각 드론의 FollowReference 상태 모니터링 |
| `on_modify()` | 편대 구성 변경 (드론 추가/제거, 위치 변경) |
| `on_pause()` | 현재 위치로 Swarm TF 고정, FollowReference 중단 |
| `on_resume()` | FollowReference 재시작 |
| `on_execution_end()` | 모든 드론 FollowReference 중단 |

내부 상태:
- `drones_`: `unordered_map<string, shared_ptr<DroneSwarm>>` — 드론 ID → DroneSwarm 객체
- `swarm_base_link_frame_id_`: Swarm TF 프레임 이름 (`<node_ns>/Swarm`)
- `goal_future_handles_`: 각 드론의 FollowReference 핸들 목록

### DroneSwarm

개별 드론을 추상화한 클래스. 각 드론 당 하나씩 생성됨.

| 메서드 | 역할 |
|---|---|
| `initFollowReference()` | 드론에게 `/{drone_id}/FollowReferenceBehavior` 액션 전송 |
| `stopFollowReference()` | `/{drone_id}/FollowReferenceBehavior/_behavior/stop` 서비스 호출 |
| `checkPosition()` | `actual_distance_to_goal < 0.3m` 이면 true |
| `updateStaticTf()` | 편대 오프셋 Static TF 갱신 |

---

## 4. 메시지/액션/서비스 정의

### `as2_msgs/action/SwarmFlocking.action`

```
# Goal (요청)
geometry_msgs/PoseStamped   virtual_centroid    # 스웜 가상 무게중심 위치 (기준 프레임 기준 오프셋)
as2_msgs/PoseWithID[]       swarm_formation     # 각 드론의 편대 위치 (무게중심 기준 오프셋)
string[]                    drones_namespace    # 드론 네임스페이스 목록
---
# Result
bool swarm_success
---
# Feedback
geometry_msgs/Pose swarm_pose
```

### `as2_msgs/msg/PoseWithID.msg`

```
string id           # 드론 네임스페이스 (예: "drone0")
geometry_msgs/Pose pose   # 무게중심 기준 오프셋 위치
```

### `as2_msgs/srv/ModifySwarm.srv`

```
bool detach_drone                   # 드론 제거
bool new_drone                      # 드론 추가
bool new_virtual_centroid_ref       # 가상 무게중심 변경
geometry_msgs/PoseStamped virtual_centroid
as2_msgs/PoseWithID[] swarm_formation
---
bool success
```

---

## 5. 전제 조건 (Prerequisites)

SwarmFlockingBehavior를 사용하려면 **각 드론마다** 다음이 실행 중이어야 한다:

| 필수 구성 요소 | 설명 |
|---|---|
| 드론 플랫폼 노드 | `/{drone_id}/platform` |
| State Estimator | `/{drone_id}/self_localization/pose` 토픽 발행 |
| **FollowReference Behavior** | `/{drone_id}/FollowReferenceBehavior` Action Server |
| TF 트리 | `earth` 프레임 기준의 드론 pose TF |

추가로 SwarmFlocking 노드 자체는 `Swarm` 네임스페이스로 실행.

---

## 6. 실행 방법

### 6-1. 기본 실행

```bash
# 1. 각 드론 별 FollowReference Behavior 실행
ros2 launch as2_behaviors_motion follow_reference_behavior_launch.py \
    namespace:=drone0

ros2 launch as2_behaviors_motion follow_reference_behavior_launch.py \
    namespace:=drone1

ros2 launch as2_behaviors_motion follow_reference_behavior_launch.py \
    namespace:=drone2

# 2. SwarmFlocking Behavior 실행 (Swarm 네임스페이스로 뜸)
ros2 launch as2_behaviors_swarm_flocking swarm_flocking_behavior.launch.py
```

### 6-2. 런치 파라미터

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `log_level` | `info` | 로깅 레벨 |
| `use_sim_time` | `false` | 시뮬레이터 시간 사용 여부 |
| `behavior_config_file` | `config/config_default.yaml` | 설정 파일 경로 |

---

## 7. 편대 비행 액션 호출 예시

### Python (rclpy)

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from as2_msgs.action import SwarmFlocking
from as2_msgs.msg import PoseWithID
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

class SwarmClient(Node):
    def __init__(self):
        super().__init__('swarm_client')
        self._client = ActionClient(self, SwarmFlocking, '/Swarm/SwarmFlockingBehavior')

    def send_goal(self):
        goal = SwarmFlocking.Goal()

        # 가상 무게중심: earth 프레임 기준 x=0, y=0, z=5m 지점
        goal.virtual_centroid = PoseStamped()
        goal.virtual_centroid.header.frame_id = 'earth'
        goal.virtual_centroid.pose.position.x = 0.0
        goal.virtual_centroid.pose.position.y = 0.0
        goal.virtual_centroid.pose.position.z = 5.0
        goal.virtual_centroid.pose.orientation.w = 1.0

        # 편대 구성 (무게중심 기준 각 드론의 상대 위치)
        drone0_pos = PoseWithID()
        drone0_pos.id = 'drone0'
        drone0_pos.pose.position.x = 0.0
        drone0_pos.pose.position.y = 0.0
        drone0_pos.pose.position.z = 0.0

        drone1_pos = PoseWithID()
        drone1_pos.id = 'drone1'
        drone1_pos.pose.position.x = 2.0   # 무게중심에서 오른쪽 2m
        drone1_pos.pose.position.y = 0.0
        drone1_pos.pose.position.z = 0.0

        drone2_pos = PoseWithID()
        drone2_pos.id = 'drone2'
        drone2_pos.pose.position.x = -2.0  # 무게중심에서 왼쪽 2m
        drone2_pos.pose.position.y = 0.0
        drone2_pos.pose.position.z = 0.0

        goal.swarm_formation = [drone0_pos, drone1_pos, drone2_pos]
        goal.drones_namespace = ['drone0', 'drone1', 'drone2']

        self._client.wait_for_server()
        future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
```

### CLI (ros2 action)

```bash
ros2 action send_goal /Swarm/SwarmFlockingBehavior as2_msgs/action/SwarmFlocking \
  "{
    virtual_centroid: {
      header: {frame_id: 'earth'},
      pose: {
        position: {x: 0.0, y: 0.0, z: 5.0},
        orientation: {w: 1.0}
      }
    },
    swarm_formation: [
      {id: 'drone0', pose: {position: {x: 0.0, y: 0.0, z: 0.0}}},
      {id: 'drone1', pose: {position: {x: 2.0, y: 0.0, z: 0.0}}},
      {id: 'drone2', pose: {position: {x: -2.0, y: 0.0, z: 0.0}}}
    ],
    drones_namespace: ['drone0', 'drone1', 'drone2']
  }"
```

---

## 8. 런타임 편대 변경

### 8-1. on_modify (액션 목표 재전송)

실행 중인 swarm에 새 goal을 보내면 `on_modify()`가 호출됨:
- 기존 모든 FollowReference 중단
- 새 편대 구성으로 재초기화
- 드론 추가/제거 가능

### 8-2. 동적 편대 토픽

실행 중에 아래 토픽에 메시지를 발행하면 개별 드론 위치만 업데이트됨:

```bash
# 토픽: /Swarm/dynamic_swarm_formation
# 타입: as2_msgs/msg/PoseWithIDArray

ros2 topic pub /Swarm/dynamic_swarm_formation as2_msgs/msg/PoseWithIDArray \
  "{poses: [{id: 'drone1', pose: {position: {x: 3.0, y: 1.0, z: 0.0}}}]}"
```

- `drones_`에 존재하는 드론 ID만 업데이트됨
- Static TF 갱신 후 해당 드론의 FollowReference 재시작

### 8-3. swarm_modify_srv 서비스

```bash
# 서비스: /Swarm/swarm_modify_srv
# 타입: as2_msgs/action/SwarmFlocking::Impl::SendGoalService (내부 서비스)
```

---

## 9. 일시 정지 / 재개

```bash
# 일시 정지: 현재 Swarm 위치 고정, 모든 FollowReference 중단
ros2 service call /Swarm/SwarmFlockingBehavior/_behavior/pause std_srvs/srv/Trigger

# 재개: 저장된 goal로 FollowReference 재시작
ros2 service call /Swarm/SwarmFlockingBehavior/_behavior/resume std_srvs/srv/Trigger
```

---

## 10. 시각화

```bash
# 드론 목록을 콤마 구분으로 입력
ros2 launch as2_visualization swarm_viz.launch.py \
    namespace_list:="drone0,drone1,drone2" \
    use_sim_time:=true
```

---

## 11. 편대 형태 예시

모든 좌표는 `virtual_centroid` 기준의 오프셋 (단위: m).

### 라인 편대 (Line)

```
drone0: (0, 0, 0)
drone1: (2, 0, 0)
drone2: (4, 0, 0)
```

### V자 편대

```
drone0: (0, 0, 0)   ← 선두
drone1: (-2, 2, 0)
drone2: (-2,-2, 0)
```

### 삼각형 편대

```
drone0: (0, 1.15, 0)
drone1: (-1,-0.58, 0)
drone2: ( 1,-0.58, 0)
```

### 수직 스택 편대

```
drone0: (0, 0,  0)
drone1: (0, 0,  2)
drone2: (0, 0, -2)
```

---

## 12. 동작 흐름 (시퀀스)

```
Client                SwarmFlockingBehavior          DroneSwarm[n]
  │                          │                            │
  │──── send_goal ──────────▶│                            │
  │                          │─── setUpVirtualCentroid    │
  │                          │    (Static TF 브로드캐스트) │
  │                          │                            │
  │                          │─── setUpDronesFormation    │
  │                          │    (DroneSwarm 객체 생성)  │
  │                          │    (각 드론 오프셋 TF 설정) │
  │                          │                            │
  │                          │─── initDroneReferences ───▶│
  │                          │    (5초 대기)               │── initFollowReference()
  │                          │                            │   (FollowReference 액션 전송)
  │                          │◀── checkPosition 루프 ─────│
  │                          │    (0.3m 이내 도달 확인)   │
  │                          │                            │
  │◀── RUNNING feedback ─────│                            │
  │                          │─── monitoring 루프 ────────▶│
  │                          │    (FollowReference 상태 확인)
```

---

## 13. 주의 사항

1. **initDroneReferences()에 `sleep(5초)` 하드코딩** — 모든 FollowReference 액션 서버가 준비될 시간. 드론이 많거나 느린 환경에서는 부족할 수 있음.

2. **checkPosition 무한루프** — 모든 드론이 0.3m 이내에 들어올 때까지 블로킹. 드론이 위치에 도달 못하면 `on_activate()`가 영원히 반환되지 않음.

3. **`drones_namespace`와 `swarm_formation`의 ID 일치 필수** — `setUpDronesFormation()`은 두 목록을 순서가 아닌 ID로 매칭함.

4. **FollowReference Behavior 필수** — 각 드론 네임스페이스에 `FollowReferenceBehavior` Action Server가 반드시 실행 중이어야 함.

5. **최대 속도 하드코딩** — `initFollowReference()`에서 `max_speed_x/y/z = 15 m/s`로 고정됨 (변경하려면 소스 수정 필요).

6. **Static TF 사용** — Swarm TF와 각 드론의 ref TF는 Static이므로, `on_modify()` 또는 `dynamic_swarm_formation` 토픽으로만 갱신 가능.

---

## 14. 토픽/서비스/액션 요약

| 종류 | 이름 | 타입 | 방향 |
|---|---|---|---|
| Action Server | `/Swarm/SwarmFlockingBehavior` | `as2_msgs/action/SwarmFlocking` | 수신 |
| Service | `/Swarm/swarm_modify_srv` | `SwarmFlocking::Impl::SendGoalService` | 수신 |
| Subscription | `/Swarm/dynamic_swarm_formation` | `as2_msgs/msg/PoseWithIDArray` | 수신 |
| Action Client | `/{drone_id}/FollowReferenceBehavior` | `as2_msgs/action/FollowReference` | 송신 |
| Service Client | `/{drone_id}/FollowReferenceBehavior/_behavior/stop` | `std_srvs/srv/Trigger` | 송신 |
| Subscription | `/{drone_id}/self_localization/pose` | `geometry_msgs/msg/PoseStamped` | 수신 |
| Behavior pause | `/Swarm/SwarmFlockingBehavior/_behavior/pause` | `std_srvs/srv/Trigger` | 수신 |
| Behavior resume | `/Swarm/SwarmFlockingBehavior/_behavior/resume` | `std_srvs/srv/Trigger` | 수신 |
