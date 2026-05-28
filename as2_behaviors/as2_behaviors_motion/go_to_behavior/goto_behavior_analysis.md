# go_to 동작 시 드론 이동 안함 - 원인 분석

## 환경

| 구성 요소 | 설명 |
|---|---|
| 시뮬레이터 | `as2_platform_multirotor_simulator` |
| 상태 추정 | `as2_state_estimator` |
| 모션 컨트롤러 | `as2_motion_controller` (pid_speed_controller 플러그인) |
| Takeoff 플러그인 | `takeoff_plugin_platform` |
| Go-To 플러그인 | `go_to_plugin_position` |

## 증상

```
goal_id: 2dec09df8b50467783638505250684bf
actual_speed: ~3e-05 ≈ 0
actual_distance_to_goal: ~2.23  (일정하게 유지)
```

- 목표 좌표: `(x:2.0, y:0.0, z:2.0)`, `max_speed: 2.0`
- goal 수락됨, behavior RUNNING 상태 유지
- 드론은 전혀 움직이지 않음

---

## 근본 원인

**`platform_info_.offboard = false`**

offboard 모드가 활성화되지 않은 상태에서 `go_to_plugin_position`이 표준 모션 컨트롤러 파이프라인을 통해 명령을 보내려 하지만, 두 군데에서 조용히 차단된다.

---

## 명령 흐름 분석

### go_to 동작 경로

```
go_to_plugin_position::own_run()
  └─ sendPositionCommandWithYawAngle("earth", 2.0, 0.0, 2.0, ...)
       └─ PositionMotion::ownSendCommand()
            └─ sendPoseCommand() → motion_reference/pose 토픽 발행
            └─ sendTwistCommand() → motion_reference/twist 토픽 발행

ControllerHandler::controlTimerCallback()
  └─ if (!platform_info_.offboard || !platform_info_.armed) { return; }  ← 차단 (1)
  └─ sendCommand()
       └─ computeOutput() → actuator_command/pose, twist 발행

AerialPlatform::sendCommand()
  └─ if (!getOffboardMode()) { return; }  ← 차단 (2)
  └─ ownSendCommand()
       └─ as2_interface_.processCommand() → simulator_.set_reference_position()
```

### 차단 지점 1: `ControllerHandler::controlTimerCallback()`

```cpp
// as2_motion_controller/src/controller_handler.cpp:469
if (!platform_info_.offboard || !platform_info_.armed ||
    control_mode_out_.control_mode == as2_msgs::msg::ControlMode::HOVER)
{
    return;  // actuator_command 토픽에 아무것도 발행하지 않음
}
```

`platform_info_`는 `as2_msgs/msg/PlatformInfo` 토픽을 구독하여 업데이트된다. offboard 모드가 설정되지 않으면 이 콜백에서 즉시 반환한다.

### 차단 지점 2: `AerialPlatform::sendCommand()`

```cpp
// as2_core/src/aerial_platform.cpp:299
} else if (!getOffboardMode()) {
    RCLCPP_DEBUG_THROTTLE(..., "Platform is not in offboard mode");
    return;  // ownSendCommand() 호출 안 됨
}
```

차단 1을 통과하더라도 플랫폼 레벨에서도 offboard 미설정 시 차단된다.

---

## takeoff는 성공하는데 go_to는 실패하는 이유

### takeoff_plugin_platform 경로 (offboard 우회)

```cpp
// as2_aerial_platforms/as2_platform_multirotor_simulator/src/as2_platform_multirotor_simulator.cpp:356
bool MultirotorSimulatorPlatform::ownTakeoff()
{
    // 시뮬레이터 타이머를 직접 호출하는 블로킹 루프
    while (rclcpp::ok() && std::abs(...position.z() - takeoff_height) > 0.2) {
        simulatorTimerCallback();
        simulatorControlTimerCallback();
        simulatorInertialOdometryTimerCallback();
        simulatorStateTimerCallback();
    }
    // sendCommand()를 완전히 우회 → offboard 체크 없음
}
```

- `ownTakeoff()`는 시뮬레이터 타이머를 **직접** 호출한다
- `AerialPlatform::sendCommand()` 경로를 완전히 건너뜀
- offboard 상태와 무관하게 동작 → 이륙 고도(`floor_height + 1.0 = 1.0m`)까지 상승 성공

### go_to_plugin_position 경로 (표준 파이프라인)

- `PositionMotion` 핸들러를 통해 표준 ROS2 토픽 파이프라인 사용
- `ControllerHandler::controlTimerCallback()` 체크 통과 필요
- `AerialPlatform::sendCommand()` 체크 통과 필요
- **offboard=false인 상태에서 두 체크 모두 실패**

---

## 증상 검증

| 증상 | 원인 |
|---|---|
| `actual_distance ≈ 2.234` | takeoff 후 드론 위치 (0,0,1.0). `sqrt((2-0)²+(0-0)²+(2-1)²) = sqrt(5) ≈ 2.236` |
| `actual_speed ≈ 3e-5 ≈ 0` | 시뮬레이터 레퍼런스가 업데이트되지 않음 |
| goal 수락됨 | `processGoal()`의 `platform_state_ == FLYING` 조건 통과 (arm 완료) |
| behavior RUNNING 유지 | `sendPositionCommandWithYawAngle()` 반환값 true (모션 레퍼런스 토픽 발행 자체는 성공) |
| 에러 메시지 없음 | offboard 차단이 `RCLCPP_DEBUG_THROTTLE`로만 기록됨 |

---

## 해결 방법

go_to 실행 전 offboard 모드를 활성화해야 한다:

```bash
# 1. arming
ros2 service call /drone0/platform/set_arming_state std_srvs/srv/SetBool "{data: true}"

# 2. offboard 활성화
ros2 service call /drone0/platform/set_offboard_mode std_srvs/srv/SetBool "{data: true}"

# 3. takeoff
# 4. go_to
```

또는 takeoff behavior 이전에 offboard를 설정하도록 상위 미션 시퀀스에 포함시킨다.

---

## 관련 코드 위치

| 파일 | 라인 | 설명 |
|---|---|---|
| `as2_motion_controller/src/controller_handler.cpp` | 469 | offboard 체크 (차단 1) |
| `as2_core/src/aerial_platform.cpp` | 299 | offboard 체크 (차단 2) |
| `as2_aerial_platforms/as2_platform_multirotor_simulator/src/as2_platform_multirotor_simulator.cpp` | 356 | `ownTakeoff()` - 블로킹 루프로 표준 파이프라인 우회 |
| `as2_behaviors/as2_behaviors_motion/go_to_behavior/plugins/go_to_plugin_position.cpp` | - | `own_run()` - `sendPositionCommandWithYawAngle()` 호출 |
| `as2_motion_reference_handlers/src/basic_motion_references.cpp` | 108 | `checkMode()` - 제어 모드 동기 서비스 호출 |
