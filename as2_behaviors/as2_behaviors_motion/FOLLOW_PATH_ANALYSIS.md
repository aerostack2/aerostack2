# follow_path_behavior 분석

## 1. 전체 구조

```
┌─────────────────────────────────────────────────────────┐
│                  FollowPathBehavior                      │
│         (as2_behavior::BehaviorServer<FollowPath>)       │
│                                                          │
│  ┌─────────────────────────────────────────────────┐    │
│  │              follow_path_base::FollowPathBase    │    │
│  │  ┌──────────────────┐  ┌──────────────────────┐ │    │
│  │  │ Plugin::position │  │ Plugin::trajectory   │ │    │
│  │  │ (waypoint 이동)   │  │ (다항식 궤적)         │ │    │
│  │  └──────────────────┘  └──────────────────────┘ │    │
│  └─────────────────────────────────────────────────┘    │
│                                                          │
│  구독: self_localization/twist, platform/info            │
│  액션: /drone_ns/FollowPathBehavior                      │
└─────────────────────────────────────────────────────────┘
```

## 2. 초기화 단계 (노드 시작 시)

`follow_path_behavior.cpp:40` — 생성자:

```
FollowPathBehavior 생성자
  ├── 파라미터 선언
  │     ├── plugin_name           (필수)
  │     ├── follow_path_speed     (기본 0.5 m/s)
  │     └── follow_path_threshold (기본 0.2 m)
  │
  ├── pluginlib로 플러그인 동적 로드
  │     └── follow_path_plugin_->initialize(node, tf_handler, params)
  │               └── FollowPathBase::initialize()
  │                     └── HoverMotion 핸들러 생성
  │                     └── ownInit() → PositionMotion 핸들러 생성 (position 플러그인)
  │
  └── 구독자 등록
        ├── self_localization/twist  → state_callback()
        └── platform/info            → platform_info_callback()
```

## 3. Goal 수신 ~ 수락/거부

`ros2 action send_goal` 호출 시:

```
handleGoal()  [behavior_server__impl.hpp:79]
  └── activate(goal)
        └── FollowPathBehavior::on_activate(goal)  [follow_path_behavior.cpp:171]
              │
              ├── [검증 1] process_goal()  [follow_path_behavior.cpp:130]
              │     ├── frame_id 공백 → REJECT
              │     ├── path 비어있음 → REJECT
              │     ├── frame_id != "earth" → TF 변환 (earth 좌표계로 통일)
              │     └── max_speed == 0 → config 기본값 사용
              │
              └── plugin->on_activate(goal)
                    │
                    ├── [검증 2] FollowPathBase::processGoal()  [follow_path_base.hpp:169]
                    │     ├── platform_state != FLYING → REJECT
                    │     ├── localization_flag == false → REJECT
                    │     └── 각 waypoint의 id 공백 확인
                    │
                    └── Plugin::own_activate()  [follow_path_plugin_position.cpp:56]
                          ├── path_ids_ 목록 구성 (전체 waypoint id)
                          ├── path_ids_remaining_ 구성 (아직 방문 안 한 waypoint)
                          ├── initial_yaw_ 저장 (현재 헤딩)
                          ├── updateDesiredPose() → 첫 번째 waypoint 타겟으로 설정
                          └── feedback 초기화
                                ├── next_waypoint_id = path_ids_remaining_[0]
                                └── remaining_waypoints = 전체 개수
```

수락되면 `register_run_timer()` → 10Hz 주기 타이머 시작.

## 4. 주기 실행 루프 (10Hz)

`timer_callback()` → `run()` → `on_run()` → `Plugin::own_run()`:

```
Plugin::own_run()  [follow_path_plugin_position.cpp:139]
  │
  ├── checkGoalCondition()
  │     └── actual_distance_to_next_waypoint < threshold(0.2m) ?
  │           ├── YES → path_ids_remaining_ 에서 현재 waypoint 제거
  │           │         ├── remaining == 0 → return true (완료)
  │           │         └── remaining > 0 → updateDesiredPose(다음 waypoint)
  │           │                              feedback 갱신
  │           └── NO  → return false (계속 진행)
  │
  ├── checkGoalCondition() == true → SUCCESS 반환
  │
  └── position_motion_handler_->sendPositionCommandWithYawAngle(
            desired_pose_, desired_twist_)
              └── /drone_ns/motion_reference/pose 토픽 발행
```

## 5. 위치 명령 생성 상세 (position 플러그인)

`updateDesiredPose()` [follow_path_plugin_position.cpp:194]:

```
desired_pose_
  ├── frame_id = goal.header.frame_id ("earth")
  ├── position = waypoint.pose.position (x, y, z)
  └── orientation = processYaw()
        ├── KEEP_YAW     → 초기 yaw 유지 (initial_yaw_)
        ├── PATH_FACING  → 현재 위치 → 목표 waypoint 방향각 계산
        │                  atan2(next_y - actual_y, next_x - actual_x)
        └── FIXED_YAW    → goal에서 지정한 각도 유지

desired_twist_
  └── linear.x = linear.y = linear.z = goal.max_speed
```

## 6. Feedback 흐름

`state_callback()` (self_localization/twist 수신마다):

```
state_callback()  [follow_path_behavior.cpp:111]
  └── tf_handler_->getState() → 현재 pose 갱신
        └── FollowPathBase::state_callback()  [follow_path_base.hpp:86]
              ├── actual_pose_ 갱신
              ├── feedback_.actual_speed = twist.linear.norm()
              └── feedback_.actual_distance_to_next_waypoint
                    = (getTargetPosition() - actual_pose_).norm()
```

feedback 항목:

| 필드 | 설명 |
|---|---|
| `actual_speed` | 현재 속도 (m/s) |
| `actual_distance_to_next_waypoint` | 다음 waypoint까지 거리 (m) |
| `remaining_waypoints` | 남은 waypoint 수 |
| `next_waypoint_id` | 현재 목표 waypoint id |

## 7. 완료/중단 처리

```
완료 (SUCCESS):
  own_execution_end() [follow_path_plugin_position.cpp:125]
    ├── path_ids_, path_ids_remaining_ 초기화
    └── sendPositionCommandWithYawAngle(desired_pose_, desired_twist_)
          └── 마지막 waypoint 위치 유지 명령 계속 발행

중단 (FAILURE / ABORTED):
  own_execution_end()
    └── sendHover()  ← HoverMotion으로 제자리 호버링
```

## 8. position 플러그인 vs trajectory 플러그인

| 항목 | position 플러그인 | trajectory 플러그인 |
|---|---|---|
| **제어 방식** | waypoint별 위치 명령 | `GeneratePolynomialTrajectory` 액션 서버에 위임 |
| **궤적 형태** | 꺾임 (불연속 방향 전환) | 매끄러운 다항식 곡선 |
| **pause 지원** | 가능 (hover) | 가능 (trajectory generator pause 서비스 호출) |
| **modify 지원** | 가능 (waypoint 추가) | 미구현 (항상 false 반환) |
| **외부 의존** | 없음 | trajectory generator 노드 필요 |

## 9. 전체 상태 전이

```
IDLE
 │  send_goal 수신
 ▼
[검증: FLYING + 위치 수신 + frame_id + path 유효]
 │  성공
 ▼
RUNNING ──────────────────────────────────────────┐
 │  10Hz 루프                                      │
 │  ┌──────────────────────────────────────────┐  │
 │  │ distance < threshold?                    │  │
 │  │   YES → 다음 waypoint로 전환              │  │
 │  │   NO  → sendPositionCommand()            │  │
 │  │ 모든 waypoint 완료?                       │  │
 │  │   YES → SUCCESS                          │  │
 │  └──────────────────────────────────────────┘  │
 │  cancel/stop 서비스 호출                        │
 ▼                                                │
ABORTED ←─────────────────────────────────────────┘
 │  sendHover()
 ▼
IDLE
```

## 10. goal 거부 원인

| 거부 원인 | 로그 메시지 |
|---|---|
| 드론 미비행 | `platform is not flying` |
| 위치 미수신 | `there is no localization` |
| frame_id 공백 | `Path frame_id is empty` |
| path 비어있음 | `Path is empty` |
| waypoint id 공백 | `waypoint id is empty` |

---

# as2_behavior_tree FollowPath 동작 원리 및 테스트 방법

## 1. 전체 구조

```
as2_behavior_tree_node (메인 실행 루프)
  └── BT::BehaviorTreeFactory
        └── FollowPathAction  ("FollowPath" 태그)
              └── BtActionNode<as2_msgs::action::FollowPath>
                    └── rclcpp_action::Client → /drone_ns/FollowPathBehavior
```

## 2. FollowPathAction 노드 포트

`follow_path.hpp:73`:

| 포트 | 방향 | 타입 | 설명 |
|---|---|---|---|
| `path` | Input | `vector<PoseWithID>` | 경로 waypoint 목록 |
| `speed` | Input | `double` | 최대 속도 (m/s) |
| `yaw_mode` | Output | `int` | **미사용** (항상 KEEP_YAW 고정) |
| `server_name` | Input | `string` | 액션 서버 이름 |
| `server_timeout` | Input | `milliseconds` | 서버 응답 타임아웃 |

## 3. path 포트 문자열 형식

`port_specialization.hpp:86`:

```
형식: "x1;y1;z1|x2;y2;z2"
  ├── 좌표는 ; 로 구분
  └── waypoint는 | 로 구분

예: "1.0;0.0;1.5|3.0;2.0;1.5"

⚠️ 제한: 정확히 2개 waypoint만 파싱 가능 (하드코딩)
         id는 "0", "1" 자동 부여
```

## 4. BT tick() 실행 흐름

`bt_action_node.hpp:187`:

```
tick() 호출 (bt_loop_duration마다, 기본 10ms)
  │
  ├── [IDLE → 최초 1회]
  │     ├── setStatus(RUNNING)
  │     ├── on_tick()                      ← follow_path.hpp:63
  │     │     ├── getInput("path", path_)
  │     │     ├── getInput("speed", max_speed_)
  │     │     ├── goal_.path = path_
  │     │     ├── goal_.max_speed = max_speed_
  │     │     └── goal_.yaw.mode = KEEP_YAW  (항상 고정)
  │     └── send_new_goal()
  │           └── action_client_->async_send_goal(goal_)
  │
  ├── [future_goal_handle_ 대기]
  │     ├── 타임아웃 초과 → FAILURE
  │     └── 수락 완료 → goal_handle_ 저장
  │
  └── [RUNNING 루프]
        ├── on_wait_for_result(feedback_)  ← 빈 구현
        ├── callback_group_executor_.spin_some()
        └── goal_result_available_?
              ├── NO  → RUNNING 반환
              └── YES → result_.code 확인
                          ├── SUCCEEDED → BT::SUCCESS
                          ├── ABORTED   → BT::FAILURE
                          └── CANCELED  → BT::SUCCESS
```

## 5. 테스트 방법 A: 에뮬레이터 사용 (실제 드론 불필요)

에뮬레이터 동작 (`follow_path_emulator.hpp:78`):
- goal 수신 → 5초 x 4회 대기 → `follow_path_success = true` 반환 (20초 후 성공)

**터미널 1**: 에뮬레이터 실행
```bash
ros2 launch node_emulators all_emulator.launch.py drone_id:=drone0
```

**테스트 XML** (`follow_path_emulator_test.xml`):
```xml
<?xml version="1.0"?>
<root main_tree_to_execute="BehaviorTree">
    <BehaviorTree ID="BehaviorTree">
        <Sequence>
            <Action ID="FollowPath"
                    path="1.0;0.0;1.5|3.0;2.0;1.5"
                    speed="1.0"
                    server_timeout="30000"/>
        </Sequence>
    </BehaviorTree>
</root>
```

**터미널 2**: BT 노드 실행
```bash
ros2 launch as2_behavior_tree behavior_trees.launch.py \
  drone_id:=drone0 \
  tree:=/path/to/follow_path_emulator_test.xml \
  server_timeout:=30000
```

## 6. 테스트 방법 B: 실제 behavior 노드 사용

**터미널 1**: follow_path_behavior 실행
```bash
ros2 launch as2_behaviors_motion follow_path_behavior_launch.py \
  namespace:=drone0 \
  plugin_name:=follow_path_plugin_position
```

**전체 미션 XML** (`follow_path_mission.xml`):
```xml
<?xml version="1.0"?>
<root main_tree_to_execute="BehaviorTree">
    <BehaviorTree ID="BehaviorTree">
        <Sequence>
            <Action ID="TakeOff" height="1.5" speed="0.5"/>
            <Action ID="FollowPath"
                    path="1.0;0.0;1.5|2.0;2.0;1.5"
                    speed="1.0"
                    server_timeout="60000"/>
            <Action ID="Land" speed="0.3"/>
        </Sequence>
    </BehaviorTree>
</root>
```

**터미널 2**: BT 노드 실행
```bash
ros2 launch as2_behavior_tree behavior_trees.launch.py \
  drone_id:=drone0 \
  tree:=/path/to/follow_path_mission.xml \
  server_timeout:=60000
```

## 7. 현재 구현의 제한 사항

| 항목 | 제한 내용 | 위치 |
|---|---|---|
| **waypoint 수** | XML path 포트에서 정확히 **2개만** 파싱 가능 | `port_specialization.hpp:86-113` |
| **yaw_mode** | XML에서 설정해도 무시, 항상 **KEEP_YAW** 고정 | `follow_path.hpp:70` |
| **frame_id** | goal_.header.frame_id 미설정 (빈 문자열) → behavior에서 REJECT 가능 | `follow_path.hpp:63-71` |
| **feedback 처리** | `on_wait_for_result()` 빈 구현 → feedback 활용 안 됨 | `follow_path.hpp:81` |

## 8. 핵심 파일 위치

| 역할 | 파일 |
|---|---|
| BT 노드 정의 | `as2_behavior_tree/include/as2_behavior_tree/action/follow_path.hpp` |
| BT 액션 노드 베이스 | `as2_behavior_tree/include/as2_behavior_tree/bt_action_node.hpp` |
| path 문자열 파싱 | `as2_behavior_tree/include/as2_behavior_tree/port_specialization.hpp` |
| BT 메인 실행 | `as2_behavior_tree/src/as2_behavior_tree_node.cpp` |
| 에뮬레이터 헤더 | `as2_behavior_tree/tests/node_emulators/include/node_emulators/follow_path_emulator.hpp` |
| 에뮬레이터 런치 | `as2_behavior_tree/tests/node_emulators/launch/all_emulator.launch.py` |
| BT 런치 | `as2_behavior_tree/launch/behavior_trees.launch.py` |
| behavior 노드 런치 | `as2_behaviors_motion/launch/follow_path_behavior_launch.py` |
