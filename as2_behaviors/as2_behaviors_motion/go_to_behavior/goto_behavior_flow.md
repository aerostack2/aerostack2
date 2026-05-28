# GoToBehavior 실행 플로우 분석

**분석 대상 Action:**
```bash
ros2 action send_goal --feedback /drone0/GoToBehavior as2_msgs/action/GoToWaypoint \
  "{
    target_pose: {
      header: {frame_id: 'earth'},
      point: {x: 2.0, y: 0.0, z: 2.0}
    },
    max_speed: 2.0,
    yaw: {mode: 0}
  }"
```

---

## 연계 노드 구성

```
[Client]
    │ ROS2 Action
    ▼
[/drone0/go_to_behavior]          ← GoToBehavior (BehaviorServer)
    │ TF lookup
    │ subscribe /drone0/self_localization/twist
    │ subscribe /drone0/platform/info
    │ service call set_control_mode
    ▼
[/drone0/controller_manager]      ← as2_motion_controller (pid_speed_controller)
    │ subscribe /drone0/motion_reference/pose
    │ subscribe /drone0/motion_reference/twist
    │ subscribe /drone0/self_localization/twist
    │ service call set_platform_control_mode
    ▼
[/drone0/platform]                ← as2_platform_multirotor_simulator
    │ subscribe /drone0/actuator_command/twist
    │ publish   /drone0/self_localization/twist
    │ publish   /drone0/platform/info
    ▼
[/tf (TF tree)]                   ← earth → drone0/odom → drone0/base_link
```

---

## Phase 1: Goal 수신 및 검증

```
Client
  │
  │  send_goal → /drone0/GoToBehavior/_action/send_goal
  ▼
BehaviorServer::handleGoal()          [behavior_server__impl.hpp:79]
  │
  └─ activate(goal)
       └─ GoToBehavior::on_activate() [go_to_behavior.cpp:169]
            │
            ├─ process_goal()         [go_to_behavior.cpp:130]
            │    ├─ frame_id 'earth' != "" → ✓
            │    ├─ target (2,0,2): z > 0 → ✓
            │    ├─ tf_handler_->tryConvert(target_pose, "earth") → 이미 earth, no-op
            │    ├─ yaw.mode=PATH_FACING(0): yaw quaternion 생성 후 earth로 변환
            │    └─ max_speed = 2.0 (명시값 사용)
            │
            └─ go_to_plugin_->on_activate()  [go_to_base.hpp:114]
                  │
                  ├─ GoToBase::processGoal()  [go_to_base.hpp:167]
                  │    ├─ platform_state_ == FLYING(4)? → ✓
                  │    └─ localization_flag_ == true?   → ✓
                  │
                  └─ own_activate()           [go_to_plugin_position.cpp:54]
                       ├─ computeYaw(PATH_FACING, target=(2,0,2), actual=(0,0,1))
                       │    diff = (2-0, 0-0) = (2, 0), norm=2 > 0.1
                       │    yaw = atan2(0, 2) = 0.0 rad  (+X 방향)
                       └─ goal_ 저장, LOG: "GoTo goal accepted"

BehaviorServer::activate()
  ├─ register_run_timer()   → run_frequency Hz 타이머 생성
  ├─ behavior_status_ = RUNNING
  └─ return ACCEPT_AND_EXECUTE → Client에 GoalResponse 전송
```

### process_goal 검증 조건 요약

| 검증 항목 | 조건 | 실패 시 |
|-----------|------|---------|
| frame_id | `!= ""` | goal reject |
| target z | `> 0` | WARN (계속 진행) |
| TF 변환 | earth 프레임 변환 가능 | goal reject |
| platform_state | `== FLYING(4)` | goal reject |
| localization_flag | `== true` | goal reject |

---

## Phase 2: 병렬 상태 구독 (지속)

```
[/drone0/self_localization/twist] (TwistStamped, 지속 수신)
  │
  └─ GoToBehavior::state_callback()       [go_to_behavior.cpp:112]
       │
       ├─ tf_handler_->getState(twist, "earth", "earth", "drone0/base_link")
       │    TF 조회: earth → drone0/base_link → PoseStamped + TwistStamped 반환
       │    실패 시: TransformException → WARN 로그, localization_flag_ 갱신 안 됨
       │
       └─ go_to_plugin_->state_callback(pose, twist) [go_to_base.hpp:84]
            ├─ actual_pose_ = pose_msg
            ├─ feedback_.actual_speed = ||twist.linear||
            ├─ feedback_.actual_distance_to_goal = ||actual_pos - goal_pos||
            │    초기값: ||(0,0,1) - (2,0,2)|| = √5 ≈ 2.236
            └─ localization_flag_ = true

[/drone0/platform/info] (PlatformInfo, 지속 수신)
  └─ GoToBehavior::platform_info_callback() [go_to_behavior.cpp:124]
       └─ platform_state_ = msg->status.state
            LANDED=2, TAKING_OFF=3, FLYING=4, LANDING=5
```

---

## Phase 3: 실행 루프 (run_frequency Hz 반복)

```
BehaviorServer::timer_callback()           [behavior_server__impl.hpp:152]
  └─ run(goal_handle_)                     [behavior_server__impl.hpp:288]
       │
       ├─ behavior_status_ == RUNNING? → ✓
       │
       └─ on_run(goal, feedback, result)
            └─ GoToBehavior::on_run()      [go_to_behavior.cpp:204]
                 └─ go_to_plugin_->on_run()
                      └─ GoToBase::on_run() [go_to_base.hpp:154]
                           └─ own_run()     [go_to_plugin_position.cpp:124]
                                │
                                ├─ checkGoalCondition()
                                │    localization_flag_ &&
                                │    actual_distance_to_goal < go_to_threshold
                                │    → 2.236 > threshold → 아직 아님
                                │
                                └─ position_motion_handler_->
                                     sendPositionCommandWithYawAngle(
                                       frame="earth",
                                       x=2.0, y=0.0, z=2.0, yaw=0.0,
                                       twist_frame="earth",
                                       vx=2.0, vy=2.0, vz=2.0)
                                          │      [position_motion.cpp:60]
                                          ├─ PoseStamped 생성 (target + yaw quat)
                                          ├─ TwistStamped 생성 (max_speed xyz)
                                          └─ ownSendCommand()
                                               ├─ sendPoseCommand()
                                               │    └─ checkMode() → publish
                                               └─ sendTwistCommand()
                                                    └─ checkMode() → publish
```

---

## Phase 3-1: 컨트롤 모드 전환 (최초 1회)

```
BasicMotionReferenceHandler::checkMode()    [basic_motion_references.cpp:108]
  │
  ├─ current_mode_(from /controller/info) != desired(POSITION+YAW_ANGLE)?
  │
  └─ setMode(POSITION + YAW_ANGLE + UNDEFINED_FRAME)
       │                               [basic_motion_references.cpp:164]
       └─ SyncServiceClient → /drone0/controller/set_control_mode
            │
            └─ ControllerHandler::setControlModeSrvCall() [controller_handler.cpp:306]
                 │
                 ├─ listPlatformAvailableControlModes()
                 │    → /drone0/platform/list_control_modes 서비스 호출 (1회 캐시)
                 │
                 ├─ [use_bypass=true]
                 │    tryToBypassController(POSITION+YAW_ANGLE)
                 │      플랫폼이 POSITION 직접 지원 → bypass_controller_=true
                 │      _control_mode_plugin_in = UNSET (컨트롤러 우회)
                 │    OR
                 │   [use_bypass=false]
                 │    findSuitableControlModes()
                 │      pid_speed_controller: input=POSITION → output=SPEED
                 │      checkSuitabilityInputMode():
                 │        POSITION level(6) >= SPEED level(4) → ✓
                 │
                 ├─ setPlatformControlMode(output_mode)
                 │    → /drone0/platform/set_platform_control_mode 서비스
                 │
                 └─ controller_ptr_->setMode(in, out)
                      → pid_speed_controller 플러그인 모드 설정
                      → control_mode_established_ = true
```

### 컨트롤 모드 8비트 인코딩

```
[control_mode(4bit)][yaw(2bit)][frame(2bit)]

POSITION + YAW_ANGLE + GLOBAL_ENU = 0b01100001 = 0x61
SPEED    + YAW_SPEED + GLOBAL_ENU = 0b01000101 = 0x45
HOVER                              = 0b00010000 = 0x10
```

---

## Phase 3-2: Motion Reference 퍼블리시 및 제어 출력

```
/drone0/motion_reference/pose   ← PoseStamped (x=2, y=0, z=2, yaw=0)
/drone0/motion_reference/twist  ← TwistStamped (vx=2, vy=2, vz=2)
           │
           ▼
ControllerHandler::controlTimerCallback()  100Hz  [controller_handler.cpp:469]
  │
  ├─ platform_info_.offboard && platform_info_.armed → ✓
  ├─ control_mode_out_ != HOVER → ✓
  ├─ control_mode_established_ → ✓
  ├─ state_adquired_ → ✓
  │
  └─ sendCommand()
       │
       ├─ [bypass=true]
       │    command_pose_  = ref_pose_   (motion_reference/pose 그대로)
       │    command_twist_ = ref_twist_  (motion_reference/twist 그대로)
       │
       └─ [bypass=false]
            controller_ptr_->computeOutput(dt, command_pose_, command_twist_, thrust)
              pid_speed_controller:
                position_error = target - actual
                speed_command  = PID(position_error)
                → command_twist_ 갱신
            │
       └─ publishCommand()
            [POSITION 모드]
              /drone0/actuator_command/pose  ← target pose
              /drone0/actuator_command/twist ← speed limit
            [SPEED 모드]
              /drone0/actuator_command/twist ← speed command
                 │
                 ▼
            as2_platform_multirotor_simulator
              물리 시뮬레이션 적용 → 드론 이동
              /drone0/self_localization/twist 업데이트
              /tf 업데이트 (earth → drone0/base_link)
```

---

## Phase 4: Feedback 전송 (루프마다)

```
BehaviorServer::run()  [behavior_server__impl.hpp:305]
  │
  ExecutionStatus::RUNNING
  │
  └─ goal_handle_action->publish_feedback(feedback)
       feedback.actual_speed:             현재 속도 크기 (점점 증가)
       feedback.actual_distance_to_goal:  목표까지 거리 (점점 감소)
```

**정상 feedback 예시:**
```
Feedback:
    actual_speed: 1.23
    actual_distance_to_goal: 1.85
```

---

## Phase 5: 목표 도달 및 종료

```
own_run() [go_to_plugin_position.cpp:126]
  │
  └─ checkGoalCondition()
       localization_flag_ &&
       actual_distance_to_goal < go_to_threshold (기본값: 파라미터)
       → true → ExecutionStatus::SUCCESS

own_execution_end(SUCCESS)              [go_to_plugin_position.cpp:107]
  └─ position_motion_handler_->sendPositionCommandWithYawAngle(
         마지막 목표 위치 유지 명령 재전송)

GoToBase::on_execution_end()            [go_to_base.hpp:147]
  └─ localization_flag_ = false

BehaviorServer::run()                   [behavior_server__impl.hpp:300]
  ├─ goal_handle_->succeed(result)      → result.go_to_success = true
  ├─ behavior_status_ = IDLE
  └─ cleanup_run_timer()
       ├─ on_execution_end(SUCCESS)
       ├─ goal_handle_.reset()
       └─ run_timer_.reset()
```

---

## 토픽/서비스 전체 요약

| 방향 | 이름 | 타입 | 용도 |
|------|------|------|------|
| Client → Behavior | `/drone0/GoToBehavior` | Action | goal 전송/feedback/result |
| Platform → Behavior | `/drone0/self_localization/twist` | Sub | 현재 속도·위치 상태 |
| Platform → Behavior | `/drone0/platform/info` | Sub | FLYING 상태 확인 |
| TF → Behavior | `/tf` | TF Lookup | earth ↔ drone0/base_link 변환 |
| Behavior → Controller | `/drone0/motion_reference/pose` | Pub | 목표 위치 명령 |
| Behavior → Controller | `/drone0/motion_reference/twist` | Pub | 속도 제한값 |
| Behavior → Controller | `/drone0/controller/set_control_mode` | Srv | POSITION 모드 요청 |
| Controller → Platform | `/drone0/platform/list_control_modes` | Srv | 지원 모드 조회 (1회 캐시) |
| Controller → Platform | `/drone0/platform/set_platform_control_mode` | Srv | 플랫폼 모드 설정 |
| Platform → Controller | `/drone0/self_localization/twist` | Sub | 상태 피드백 (PID용) |
| Controller → Platform | `/drone0/actuator_command/pose` | Pub | 위치 명령 출력 |
| Controller → Platform | `/drone0/actuator_command/twist` | Pub | 속도 명령 출력 |

---

## 주요 실패 경로

| 실패 위치 | 원인 | 증상 |
|-----------|------|------|
| `process_goal` | TF earth 프레임 없음 | goal REJECT |
| `GoToBase::processGoal` | platform_state != FLYING | goal REJECT + "Behavior reject, platform is not flying" |
| `GoToBase::processGoal` | localization_flag_ = false | goal REJECT + "Behavior reject, there is no localization" |
| `setMode` | 컨트롤러 POSITION 모드 거부 | FAILURE + "Error sending position command" |
| `setControlModeSrvCall` | output level > input level | "Input control mode has lower level than output control mode" |
| `computeYaw(PATH_FACING)` | XY 거리 < 0.1 | WARN + KEEP_YAW로 fallback |

---

## 관련 소스 파일

| 파일 | 역할 |
|------|------|
| `as2_behaviors/as2_behavior/include/as2_behavior/__impl/behavior_server__impl.hpp` | BehaviorServer 실행 루프 |
| `as2_behaviors/as2_behaviors_motion/go_to_behavior/src/go_to_behavior.cpp` | GoToBehavior 진입점 |
| `as2_behaviors/as2_behaviors_motion/go_to_behavior/include/go_to_behavior/go_to_base.hpp` | GoToBase: 상태관리·goal 검증 |
| `as2_behaviors/as2_behaviors_motion/go_to_behavior/plugins/go_to_plugin_position.cpp` | Position 플러그인: 실행·완료 판정 |
| `as2_motion_reference_handlers/src/basic_motion_references.cpp` | 컨트롤 모드 전환·명령 퍼블리시 |
| `as2_motion_reference_handlers/src/position_motion.cpp` | Position 명령 빌드 |
| `as2_motion_controller/src/controller_handler.cpp` | 컨트롤러 모드 협상·제어 루프 |
| `as2_motion_controller/plugins/pid_speed_controller/config/available_modes.yaml` | pid_speed_controller 지원 모드 |
| `as2_aerial_platforms/as2_platform_multirotor_simulator/config/control_modes.yaml` | 플랫폼 지원 모드 |
