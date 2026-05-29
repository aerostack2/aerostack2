# 드론 부팅 시 실행 노드 분석

aerostack2 코드베이스(런치 파일, 소스 코드) 분석을 기반으로
드론 부팅 시 필수 노드와 선택 노드를 정리한다.

---

## 계층 구조 개요

```
┌──────────────────────────────────────────────┐
│  Phase 4. 선택적 UI / 모니터링               │
│  as2_visualization / as2_fleet_manager       │
│  as2_keyboard_teleoperation                  │
├──────────────────────────────────────────────┤
│  Phase 3. 선택적 Behaviors                   │
│  SwarmFlocking / BehaviorTree / PathPlanning │
├──────────────────────────────────────────────┤
│  Phase 2. 대기 준비 Behaviors (Standby)      │
│  Takeoff / Land / GoTo / FollowPath          │
├──────────────────────────────────────────────┤
│  Phase 1. 부팅 필수 (3 노드)                 │
│  platform → state_estimator → controller    │
└──────────────────────────────────────────────┘
```

`as2_motion_reference_handlers`는 독립 노드가 아닌 **라이브러리**로,
Behaviors와 Controller 내부에서 직접 사용된다 — 별도 런치 불필요.

---

## Phase 1 — 부팅 필수 노드 (3개)

이 3개는 **반드시 순서대로** 실행되어야 한다.
하나라도 없으면 드론이 동작하지 않는다.

```
1. platform          →  platform/info, sensor 데이터 발행
2. state_estimator   →  self_localization/pose, twist 발행
3. controller_manager →  actuator_command 발행 → platform 수신
```

---

### 1-1. platform 노드

| 항목 | 내용 |
|------|------|
| **패키지** | `as2_platform_mavlink` (실제 드론) / `as2_platform_multirotor_simulator` (시뮬레이션) |
| **노드 이름** | `platform` |
| **런치 파일** | `as2_platform_mavlink_launch.py` |
| **실행 파일** | `as2_platform_mavlink_node` |
| **필수 설정 파일** | `platform_config_file.yaml`, `control_modes.yaml` |

**런치 인자:**

| 인자 | 기본값 | 설명 |
|------|--------|------|
| `namespace` | `drone0` | 드론 네임스페이스 |
| `platform_config_file` | 패키지 기본 yaml | 플랫폼 설정 파일 |
| `control_modes_file` | 패키지 기본 yaml | 제어 모드 목록 |

**발행 토픽:**

| 토픽 | 타입 | 설명 |
|------|------|------|
| `{ns}/platform/info` | `as2_msgs/PlatformInfo` | connected, armed, offboard, state, control_mode |

**구독 토픽:**

| 토픽 | 타입 | 설명 |
|------|------|------|
| `{ns}/actuator_command/pose` | `geometry_msgs/PoseStamped` | 자세 명령 |
| `{ns}/actuator_command/twist` | `geometry_msgs/TwistStamped` | 속도 명령 |
| `{ns}/actuator_command/thrust` | `as2_msgs/Thrust` | 추력 명령 |
| `{ns}/alert_event` | `as2_msgs/AlertEvent` | 긴급 정지 이벤트 |

**제공 서비스:**

| 서비스 | 타입 | 설명 |
|--------|------|------|
| `{ns}/set_arming_state` | `std_srvs/SetBool` | ARM / DISARM |
| `{ns}/set_offboard_mode` | `std_srvs/SetBool` | Offboard 모드 전환 |
| `{ns}/set_platform_control_mode` | `as2_msgs/SetControlMode` | 제어 모드 설정 |

**실행 명령:**

```bash
ros2 launch as2_platform_mavlink as2_platform_mavlink_launch.py \
  namespace:=drone0
```

---

### 1-2. state_estimator 노드

| 항목 | 내용 |
|------|------|
| **패키지** | `as2_state_estimator` |
| **노드 이름** | `state_estimator` |
| **런치 파일** | `state_estimator_launch.py` |
| **실행 파일** | `as2_state_estimator_node` |
| **구조** | 플러그인 기반 — 환경에 따라 플러그인 선택 |

**사용 가능한 플러그인:**

| 플러그인 | 입력 센서 | 사용 환경 |
|---------|----------|----------|
| `ground_truth` | `ground_truth/pose`, `ground_truth/twist` | 시뮬레이션 |
| `ground_truth_odometry_fuse` | ground_truth + odometry 융합 | 시뮬레이션 + 오도메트리 |
| `mocap_pose` | `sensor_measurements/mocap` | 실내 모션캡처 |
| `raw_odometry` | `sensor_measurements/odom` | 외부 오도메트리 |

**런치 인자:**

| 인자 | 기본값 | 설명 |
|------|--------|------|
| `namespace` | 환경변수 `AEROSTACK2_SIMULATION_DRONE_ID` | 드론 네임스페이스 |
| `plugin_name` | config 파일에서 자동 결정 | 상태 추정 플러그인 |
| `config_file` | 패키지 기본 yaml | 노드 설정 파일 |
| `plugin_config_file` | 플러그인별 기본 yaml | 플러그인 설정 파일 |
| `use_sim_time` | `false` | 시뮬레이션 시간 사용 여부 |
| `log_level` | `info` | 로그 레벨 |

**발행 토픽:**

| 토픽 | 타입 | 설명 |
|------|------|------|
| `{ns}/self_localization/pose` | `geometry_msgs/PoseStamped` | 현재 위치·자세 |
| `{ns}/self_localization/twist` | `geometry_msgs/TwistStamped` | 현재 속도 |
| `{ns}/self_localization/odom` | `nav_msgs/Odometry` | 오도메트리 |

**실행 명령:**

```bash
# 시뮬레이션 (ground_truth 플러그인)
ros2 launch as2_state_estimator state_estimator_launch.py \
  namespace:=drone0 \
  plugin_name:=ground_truth \
  use_sim_time:=true

# 실내 모션캡처
ros2 launch as2_state_estimator state_estimator_launch.py \
  namespace:=drone0 \
  plugin_name:=mocap_pose
```

---

### 1-3. controller_manager 노드

| 항목 | 내용 |
|------|------|
| **패키지** | `as2_motion_controller` |
| **노드 이름** | `controller_manager` |
| **런치 파일** | `controller_launch.py` |
| **실행 파일** | `as2_motion_controller_node` |
| **구조** | 플러그인 기반 — 제어 알고리즘 선택 |

**사용 가능한 플러그인:**

| 플러그인 | 특징 | 사용 환경 |
|---------|------|----------|
| `pid_speed_controller` | PID 기반 속도 제어 | 일반 멀티로터 |
| `differential_flatness_controller` | 미분 평탄화 기반 궤적 제어 | 고기동 비행 |

**런치 인자:**

| 인자 | 기본값 | 설명 |
|------|--------|------|
| `namespace` | 환경변수 `AEROSTACK2_SIMULATION_DRONE_ID` | 드론 네임스페이스 |
| `plugin_name` | config 파일에서 자동 결정 | 제어 알고리즘 플러그인 |
| `config_file` | 패키지 기본 yaml | 노드 설정 파일 |
| `plugin_config_file` | 플러그인별 기본 yaml | 플러그인 파라미터 |
| `plugin_available_modes_config_file` | 플러그인별 yaml | 지원 제어 모드 목록 |
| `use_sim_time` | `false` | 시뮬레이션 시간 사용 여부 |
| `log_level` | `info` | 로그 레벨 |

**구독 토픽:**

| 토픽 | 타입 | 설명 |
|------|------|------|
| `{ns}/motion_reference/pose` | `geometry_msgs/PoseStamped` | 위치 레퍼런스 |
| `{ns}/motion_reference/twist` | `geometry_msgs/TwistStamped` | 속도 레퍼런스 |
| `{ns}/motion_reference/trajectory` | `as2_msgs/TrajectorySetpoints` | 궤적 레퍼런스 |
| `{ns}/self_localization/pose` | `geometry_msgs/PoseStamped` | 현재 위치 피드백 |
| `{ns}/self_localization/twist` | `geometry_msgs/TwistStamped` | 현재 속도 피드백 |

**발행 토픽:**

| 토픽 | 타입 | 설명 |
|------|------|------|
| `{ns}/actuator_command/pose` | `geometry_msgs/PoseStamped` | 자세 명령 |
| `{ns}/actuator_command/twist` | `geometry_msgs/TwistStamped` | 속도 명령 |
| `{ns}/actuator_command/thrust` | `as2_msgs/Thrust` | 추력 명령 |

**실행 명령:**

```bash
# PID 속도 제어기
ros2 launch as2_motion_controller controller_launch.py \
  namespace:=drone0 \
  plugin_name:=pid_speed_controller

# 미분 평탄화 제어기
ros2 launch as2_motion_controller controller_launch.py \
  namespace:=drone0 \
  plugin_name:=differential_flatness_controller
```

---

## Phase 2 — 대기 준비 Behaviors

런치는 하되 **Action 호출이 있을 때만 동작**한다.
4개의 Behavior가 **하나의 ComposableNodeContainer** 안에 함께 실행된다.

| 노드 이름 | 역할 | 실행 조건 |
|----------|------|----------|
| `TakeoffBehavior` | 이륙 제어 | 플랫폼 상태 `LANDED` |
| `LandBehavior` | 착륙 제어 + 자동 DISARM | 플랫폼 상태 `FLYING` |
| `GoToBehavior` | 목표점 이동 | 플랫폼 상태 `FLYING` |
| `FollowPathBehavior` | 경로 추종 | 플랫폼 상태 `FLYING` |

**런치 파일:** `composable_motion_behaviors.launch.py`

모든 비행 Behavior는 아래 조건을 코드 수준에서 강제한다:

```cpp
// 모든 비행 Behavior 공통 (GoTo, FollowPath, FollowReference 등)
if (platform_state_ != as2_msgs::msg::PlatformStatus::FLYING) {
    RCLCPP_ERROR(..., "Behavior reject, platform is not flying");
    return false;
}
```

**런치 인자:**

| 인자 | 기본값 | 설명 |
|------|--------|------|
| `namespace` | 환경변수 `AEROSTACK2_SIMULATION_DRONE_ID` | 드론 네임스페이스 |
| `use_sim_time` | `false` | 시뮬레이션 시간 사용 여부 |
| `log_level` | `info` | 로그 레벨 |
| `takeoff_plugin_name` | 선택 | Takeoff 구현 플러그인 |
| `land_plugin_name` | 선택 | Land 구현 플러그인 |
| `go_to_plugin_name` | 선택 | GoTo 구현 플러그인 |
| `follow_path_plugin_name` | 선택 | FollowPath 구현 플러그인 |

**실행 명령:**

```bash
ros2 launch as2_behaviors_motion composable_motion_behaviors.launch.py \
  namespace:=drone0
```

---

## 플랫폼 상태 머신과 Behavior 관계

```
DISARMED
    │  ARM (set_arming_state=true)
    ▼
LANDED
    │  TakeoffBehavior 호출
    ▼
TAKING_OFF
    │  고도 도달 완료
    ▼
FLYING  ◄─── GoToBehavior / FollowPathBehavior / FollowReferenceBehavior
    │  LandBehavior 호출
    ▼
LANDING
    │  착지 완료 → 자동 DISARM
    ▼
LANDED → DISARMED

※ 어떤 상태에서든 EMERGENCY 이벤트 발생 시 → EMERGENCY 상태
  (복구 불가, 재부팅 필요)
```

**LandBehavior 완료 시 자동 처리 (코드 근거):**

```cpp
// land_behavior.cpp
on_execution_end(SUCCESS) {
    sendEventFSME(PSME::LANDED);   // LANDING → LANDED
    sendDisarm();                   // LANDED → DISARMED 자동 처리
}
```

---

## Phase 3 — 선택적 Behaviors

미션 내용에 따라 필요 시 실행한다.

| 노드 | 패키지 | 런치 파일 | 실행 시점 |
|------|--------|----------|----------|
| `FollowReferenceBehavior` | `as2_behaviors_motion` | 별도 | 실시간 참조값 추종 미션 |
| `SwarmFlockingBehavior` | `as2_behaviors_swarm_flocking` | `swarm_flocking_behavior.launch.py` | 군집 비행 미션 |
| `PathPlanningBehavior` | `as2_behaviors_path_planning` | 별도 | 장애물 회피 미션 |
| BehaviorTree | `as2_behavior_tree` | 별도 | 자율 시퀀스 미션 |

---

## Phase 4 — 선택적 UI / 모니터링

운용 또는 개발 필요에 따라 실행한다.

| 노드 | 패키지 | 런치 파일 | 실행 시점 |
|------|--------|----------|----------|
| RViz + 마커 발행 | `as2_visualization` | `as2_visualization.launch.py` | 3D 시각화 필요 시 |
| 알파뉴메릭 뷰어 | `as2_alphanumeric_viewer` | `alphanumeric_viewer_launch.py` | 터미널 상태 확인 시 |
| 키보드 텔레옵 | `as2_keyboard_teleoperation` | `as2_keyboard_teleoperation_launch.py` | 수동 조종 테스트 시 |
| Fleet Manager | `as2_fleet_manager` | `fleet_manager.launch.py` | 멀티드론 운용 시 |

---

## 전체 실행 순서 체크리스트

### Phase 1 — 시스템 초기화 (필수, 순서 준수)

```bash
# 1. 플랫폼 노드
ros2 launch as2_platform_mavlink as2_platform_mavlink_launch.py \
  namespace:=drone0

# 2. 상태 추정 노드
ros2 launch as2_state_estimator state_estimator_launch.py \
  namespace:=drone0 plugin_name:=ground_truth

# 3. 모션 제어 노드
ros2 launch as2_motion_controller controller_launch.py \
  namespace:=drone0 plugin_name:=pid_speed_controller
```

### Phase 2 — Behaviors 준비 (필수, Phase 1 완료 후)

```bash
ros2 launch as2_behaviors_motion composable_motion_behaviors.launch.py \
  namespace:=drone0
```

### Phase 3 — 운용 시작

```bash
# ARM → Takeoff → 미션 → Land 순서로 Action 호출
```

### Phase 4 — 선택적 UI (필요 시)

```bash
# 시각화
ros2 launch as2_visualization as2_visualization.launch.py namespace:=drone0

# 멀티드론 관리
ros2 launch as2_fleet_manager fleet_manager.launch.py
```

---

## 런치 파일 경로 참조

| 노드 | 런치 파일 경로 |
|------|--------------|
| platform | `as2_aerial_platforms/as2_platform_mavlink/launch/as2_platform_mavlink_launch.py` |
| state_estimator | `as2_state_estimator/launch/state_estimator_launch.py` |
| controller_manager | `as2_motion_controller/launch/controller_launch.py` |
| Motion Behaviors | `as2_behaviors/as2_behaviors_motion/launch/composable_motion_behaviors.launch.py` |
| SwarmFlocking | `as2_behaviors/as2_behaviors_swarm_flocking/launch/swarm_flocking_behavior.launch.py` |
| as2_visualization | `as2_user_interfaces/as2_visualization/as2_visualization/launch/as2_visualization.launch.py` |
| fleet_manager | `as2_user_interfaces/as2_fleet_manager/launch/fleet_manager.launch.py` |
