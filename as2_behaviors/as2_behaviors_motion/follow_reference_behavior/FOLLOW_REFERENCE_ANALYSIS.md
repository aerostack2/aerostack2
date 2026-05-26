# follow_reference_behavior 분석

## 1. 역할 및 개요

`FollowReferenceBehavior`는 드론이 **지정한 TF 프레임의 원점을 지속적으로 추적**하는 behavior다.  
고정 좌표가 아니라 **움직이는 TF 프레임**을 따라가기 때문에, 프레임 위치를 바꾸는 것만으로 드론을 유도할 수 있다.

- 패키지: `as2_behaviors_motion`
- 실행 주파수: **30Hz** (`preset_loop_frequency(30)`)
- Action 타입: `as2_msgs/action/FollowReference`
- 단독 실행도 가능하지만, `SwarmFlockingBehavior`의 핵심 하위 구성 요소로 사용됨

---

## 2. 클래스 구조

```
BehaviorServer<as2_msgs::action::FollowReference>  (as2_behavior)
  └── FollowReferenceBehavior
        ├── tf_handler_           : TfHandler    — TF 조회 (getState, tryConvert)
        ├── position_motion_handler_ : PositionMotion — motion_reference/pose 발행
        ├── hover_motion_handler_    : HoverMotion    — motion_reference/twist 발행 (정지)
        ├── twist_sub_            : /self_localization/twist 구독
        ├── platform_info_sub_    : /platform/info 구독
        ├── goal_                 : 현재 활성 goal 복사본
        ├── actual_pose_          : TF에서 계산한 현재 드론 pose
        ├── actual_twist          : twist 구독으로 받은 현재 속도
        ├── localization_flag_    : twist 수신 여부 플래그
        └── platform_state_       : 플랫폼 상태 (FLYING 여부 확인)
```

---

## 3. 상태 머신

```
          send_goal
              │
              ▼
        ┌─ on_activate() ─┐
        │  checkGoal()    │  FLYING 아님 → REJECT
        │  getState()     │  localization 없음 → REJECT
        │  computeYaw()   │  TF 없음 → REJECT
        └─────────────────┘
              │ true
              ▼
        ┌─────────────────────────────┐
        │  on_run() 루프 (30Hz)       │◀─── on_resume()
        │  sendPositionCommand()      │          │
        │  TF frame 원점을 계속 추적  │     on_pause()
        └─────────────────────────────┘          │
              │                            sendHover() → 제자리 정지
              │ FAILURE (TF 실패 등)
              ▼
        ┌─ on_deactivate() ─┐
        │  sendHover()      │◀─── stop 서비스 호출 시
        └───────────────────┘
```

---

## 4. 핵심 메서드 분석

### 4-1. `on_activate()` — 목표 수락 및 검증

```cpp
// follow_reference_behavior.cpp:150
bool FollowReferenceBehavior::on_activate(goal)
{
  goal_ = *goal;                    // goal 복사본 저장

  if (!process_goal(goal, goal_))   // ① 파라미터 보완 + TF 유효성 검사
    return false;
  if (!getState())                  // ② 현재 드론 위치 TF 조회
    return false;
  if (!checkGoal(goal_))            // ③ FLYING + localization 확인
    return false;
  if (!computeYaw(...))             // ④ yaw 모드에 따른 초기 yaw 계산
    return false;

  return true;
}
```

**`process_goal()` 내부:**
```cpp
// goal.target_pose.header.frame_id 가 TF 버퍼에 존재하는지 검증
tf_handler_->tryConvert(new_goal.target_pose, goal->target_pose.header.frame_id)
// → source == target 이므로 좌표 변환 없음. TF 프레임 존재 확인이 목적.

// max_speed가 0이면 파라미터 기본값으로 채움
new_goal.max_speed_x = (goal->max_speed_x != 0.0f) ?
    goal->max_speed_x : get_parameter("follow_reference_max_speed_x").as_double();
```

**`checkGoal()` 내부:**
```cpp
// 플랫폼이 FLYING 상태가 아니면 거부
if (platform_state_ != as2_msgs::msg::PlatformStatus::FLYING) return false;
// localization(twist) 데이터가 없으면 거부
if (!localization_flag_) return false;
```

---

### 4-2. `on_run()` — 30Hz 제어 루프 (핵심)

```cpp
// follow_reference_behavior.cpp:219
as2_behavior::ExecutionStatus FollowReferenceBehavior::on_run(goal, feedback_msg, result_msg)
{
  // ① 위치 명령 발행 (frame_id 그대로, 좌표 변환 없음)
  if (!position_motion_handler_->sendPositionCommandWithYawAngle(
        goal_.target_pose.header.frame_id,   // "Swarm/drone_n_ref" 등
        goal_.target_pose.point.x,           // 목표 x (스웜에서는 0)
        goal_.target_pose.point.y,           // 목표 y (스웜에서는 0)
        goal_.target_pose.point.z,           // 목표 z (스웜에서는 0)
        goal_.yaw.angle,
        "earth",
        max_speed_x, max_speed_y, max_speed_z))
  {
    result_.follow_reference_success = false;
    return FAILURE;   // Controller 모드 설정 실패 등
  }

  result_.follow_reference_success = true;
  return RUNNING;     // 항상 RUNNING → 외부에서 stop 호출 전까지 무한 실행
}
```

> `sendPositionCommandWithYawAngle()`은 TF 변환 없이 받은 frame_id와 좌표를 그대로  
> `/drone_n/motion_reference/pose` 토픽에 발행한다.  
> **TF 해석은 Controller가 담당.**

---

### 4-3. `state_callback()` — twist 구독 콜백

```cpp
// follow_reference_behavior.cpp:100
void FollowReferenceBehavior::state_callback(twist_msg)
{
  actual_twist = *twist_msg;
  localization_flag_ = true;      // 최초 수신 시 true → checkGoal 통과 가능

  if (getState()) {               // 실행 중이면 yaw 재계산
    computeYaw(goal_.yaw.mode, ...);
  }
}
```

**`getState()` 내부 — feedback 계산용 TF 조회:**
```cpp
// "earth" 기준으로 twist 변환
// goal_.target_pose.header.frame_id 기준으로 드론 pose 계산
auto [pose_msg, twist_msg] = tf_handler_->getState(
    actual_twist,
    "earth",                               // twist 목표 프레임
    goal_.target_pose.header.frame_id,     // pose 목표 프레임 ("Swarm/drone_n_ref")
    base_link_frame_id_);                  // pose 소스 프레임 ("drone_n/base_link")

// feedback 계산
actual_pose_ = pose_msg;   // "Swarm/drone_n_ref" 기준 드론 위치
feedback_.actual_distance_to_goal =
    distance(actual_pose_.position, goal_.target_pose.point);
// → 드론이 "Swarm/drone_n_ref" 원점(0,0,0)에서 얼마나 떨어져 있는가
```

---

### 4-4. `computeYaw()` — yaw 모드 처리

| yaw_mode | 동작 |
|---|---|
| `KEEP_YAW` | 현재 yaw 유지 (스웜에서 기본값) |
| `PATH_FACING` | 목표 방향으로 yaw (0.1m 미만이면 KEEP_YAW로 폴백) |
| `FIXED_YAW` | goal에서 지정한 yaw 고정 |
| `YAW_TO_FRAME` | 현재 위치 기준 프레임 방향으로 yaw |
| `YAW_FROM_TOPIC` | 미지원 → FAILURE |

---

### 4-5. `on_deactivate()` — 정지 및 Hover

```cpp
// follow_reference_behavior.cpp:196
bool FollowReferenceBehavior::on_deactivate(message)
{
  RCLCPP_INFO(this->get_logger(), "FollowReference Stopped");
  goal_.target_pose.header.frame_id = "";  // frame_id 초기화 → getState() false 반환
  sendHover();                              // HoverMotion으로 제자리 정지
  return true;
}
```

`sendHover()`는 `HoverMotion::sendHover()`를 호출하여  
`/motion_reference/twist`에 0속도를 발행 → 제자리 호버링.

---

### 4-6. `on_pause()` / `on_resume()`

```cpp
bool on_pause()  { sendHover(); return true; }   // 현재 위치 유지
bool on_resume() { return true; }                 // on_run 루프 재개만 (특별 처리 없음)
```

---

## 5. 파라미터

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `follow_reference_max_speed_x` | 10.0 m/s | x축 최대 속도 (goal에서 0이면 이 값 사용) |
| `follow_reference_max_speed_y` | 10.0 m/s | y축 최대 속도 |
| `follow_reference_max_speed_z` | 10.0 m/s | z축 최대 속도 |
| `tf_timeout_threshold` | 0.05 s (50ms) | TF 조회 타임아웃 |
| `run_frequency` | 10.0 Hz | BehaviorServer 기본값 (노드에서 30Hz로 재설정) |

> SwarmFlockingBehavior에서 호출 시 `max_speed_x/y/z = 15.0 m/s`로 하드코딩 전달.  
> 파라미터 기본값(10.0)보다 높으므로 goal 값이 우선 적용됨.

---

## 6. 토픽 / 서비스 / Action 요약

| 종류 | 경로 | 타입 | 방향 |
|---|---|---|---|
| Action Server | `/{ns}/FollowReferenceBehavior` | `as2_msgs/action/FollowReference` | 수신 |
| Service | `/{ns}/FollowReferenceBehavior/_behavior/stop` | `std_srvs/srv/Trigger` | 수신 |
| Service | `/{ns}/FollowReferenceBehavior/_behavior/pause` | `std_srvs/srv/Trigger` | 수신 |
| Service | `/{ns}/FollowReferenceBehavior/_behavior/resume` | `std_srvs/srv/Trigger` | 수신 |
| Publisher | `/{ns}/motion_reference/pose` | `geometry_msgs/msg/PoseStamped` | 송신 |
| Publisher | `/{ns}/motion_reference/twist` | `geometry_msgs/msg/TwistStamped` | 송신 |
| Subscriber | `/{ns}/self_localization/twist` | `geometry_msgs/msg/TwistStamped` | 수신 |
| Subscriber | `/{ns}/platform/info` | `as2_msgs/msg/PlatformInfo` | 수신 |
| Service Client | `/{ns}/controller/set_control_mode` | `as2_msgs/srv/SetControlMode` | 송신 |
| TF Lookup | `drone_n/base_link` → `goal_frame` | tf2 | 조회 |

---

## 7. 전체 데이터 흐름

```
[Action Client (SwarmFlockingBehavior)]
  send_goal:
    target_pose.frame_id = "Swarm/drone_n_ref"
    target_pose.point    = (0, 0, 0)
    yaw.mode             = KEEP_YAW
    max_speed_x/y/z      = 15.0
         │
         ▼
[FollowReferenceBehavior] (30Hz 루프)
  ┌── state_callback (twist 수신) ──────────────────────────────┐
  │     localization_flag_ = true                               │
  │     getState() → TF 조회:                                   │
  │       "drone_n/base_link" → "Swarm/drone_n_ref" 기준 위치  │
  │       actual_distance_to_goal 계산 → feedback 전송          │
  └─────────────────────────────────────────────────────────────┘

  on_run():
    sendPositionCommandWithYawAngle(
        "Swarm/drone_n_ref", 0, 0, 0,   ← TF 변환 없음
        yaw_angle, "earth",
        15.0, 15.0, 15.0)
         │
         ▼ /motion_reference/pose 발행
    { frame_id: "Swarm/drone_n_ref", position: (0,0,0) }

[Controller]
  /motion_reference/pose 수신
  TF 조회: "Swarm/drone_n_ref" → earth
    = virtual_centroid_offset + formation_offset
  → 실제 earth 좌표로 비행 setpoint 생성
```

---

## 8. 주의사항

1. **종료 조건 없음**: `on_run()`은 항상 `RUNNING`을 반환. `stop` 서비스 호출 또는 Action cancel로만 종료.

2. **TF 조회 실패 시**: `getState()` 내부에서 `RCLCPP_WARN` 출력 후 `return true` — **FAILURE가 아님**. TF 조회 실패는 feedback 미갱신으로만 나타나고 제어 루프는 계속됨.

3. **`on_run()` TF 실패 시**: `sendPositionCommandWithYawAngle()` 내부 `checkMode()` 실패 시 `FAILURE` 반환. Controller 모드 설정 실패가 주원인.

4. **process_goal의 tryConvert**: TF 좌표 변환이 아닌 **TF 프레임 존재 유효성 검사** 목적. source == target 이므로 좌표 변환 없음.

5. **30Hz vs BehaviorServer 기본 10Hz**: `preset_loop_frequency(30)`으로 노드 수준에서 재설정. `run_frequency` 파라미터보다 우선.

6. **max_speed 우선순위**: goal의 `max_speed != 0` → goal 값 사용. `== 0` → 파라미터 기본값(10 m/s) 사용. SwarmFlocking은 15 m/s로 전달하므로 항상 15 m/s.
