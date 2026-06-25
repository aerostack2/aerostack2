# Aerostack2 Runtime ICD — 토픽 인터페이스 정의

namespace: `as2_names::topics` (`as2_core/include/as2_core/names/topics.hpp`). 모든 토픽 = 드론 namespace 하위 상대경로. QoS = 소스 선언값.

**방향 표기**: `Producer → Consumer`. 화살표 = 데이터 흐름. 명령계열(motion_reference / actuator_command)은 위→아래, 상태계열(sensor / localization / info)은 아래→위.

---

## L0 — Sensor Measurements (HW → 시스템)

QoS: `SensorDataQoS` (best-effort)

| Topic | Msg Type | Producer → Consumer | 정의 위치 |
|---|---|---|---|
| `sensor_measurements/imu` | `sensor_msgs/Imu` | Platform → StateEstimator | `sensor.hpp:788` |
| `sensor_measurements/gps` | `sensor_msgs/NavSatFix` | Platform → StateEstimator | `sensor.hpp:789` |
| `sensor_measurements/lidar` | `sensor_msgs/LaserScan` | Platform → MapServer / Estimator | `sensor.hpp:790` |
| `sensor_measurements/battery` | `sensor_msgs/BatteryState` | Platform → 모니터링 | `sensor.hpp:791` |
| `sensor_measurements/odom` | `nav_msgs/Odometry` | Platform → StateEstimator | `sensor.hpp:787` |
| `sensor_measurements/camera` | `sensor_msgs/Image` (+`CameraInfo`) | Platform → Perception | `sensor.hpp:470` (`Camera`) |
| `sensor_measurements/<barometer\|compass\|range>` | `FluidPressure` / `MagneticField` / `Range` | Platform → Estimator | `sensor.hpp:792-794` |

---

## L0b — Ground Truth (Sim 전용)

QoS: `SensorDataQoS`

| Topic | Msg Type | Producer → Consumer |
|---|---|---|
| `ground_truth/pose` | `geometry_msgs/PoseStamped` | Simulator → Estimator (ground_truth plugin) |
| `ground_truth/twist` | `geometry_msgs/TwistStamped` | Simulator → Estimator |

---

## L1 — Self Localization (StateEstimator → 시스템)

QoS: `SensorDataQoS`. Producer = `as2_state_estimator` (plugin: raw_odometry / ground_truth / mocap_pose / ground_truth_odometry_fuse)

| Topic | Msg Type | Producer → Consumer |
|---|---|---|
| `self_localization/pose` | `geometry_msgs/PoseStamped` | StateEstimator → Controller, Behaviors |
| `self_localization/twist` | `geometry_msgs/TwistStamped` | StateEstimator → Controller, Behaviors |
| `self_localization/odom` | `nav_msgs/Odometry` | StateEstimator → Controller, Behaviors |

부수 출력: TF (`earth → map → odom → base_link`).

---

## L2 — Motion Reference (Behaviors → Controller)

QoS: `SensorDataQoS` (기본), waypoint / trajectory = `QoS(10)` reliable

| Topic | Msg Type | Producer → Consumer | QoS |
|---|---|---|---|
| `motion_reference/pose` | `geometry_msgs/PoseStamped` | Behavior / RefHandler → Controller | SensorData |
| `motion_reference/twist` | `geometry_msgs/TwistStamped` | Behavior / RefHandler → Controller | SensorData |
| `motion_reference/thrust` | `as2_msgs/Thrust` | Behavior / RefHandler → Controller | SensorData |
| `motion_reference/trajectory` | `as2_msgs/TrajectorySetpoints` | TrajGen Behavior → Controller | `QoS(10)` |
| `motion_reference/modify_waypoint` | `as2_msgs/PoseStampedWithID` | FollowPath → TrajGen | `QoS(10)` |
| `motion_reference/traj_gen_info` | `as2_msgs/TrajGenInfo` | TrajGen → 모니터링 | SensorData |

---

## L3 — Actuator Command (Controller → Platform)

QoS: `SensorDataQoS`. Producer = `as2_motion_controller` (plugin: pid_speed_controller / differential_flatness_controller). Consumer = `AerialPlatform` (`aerial_platform.hpp:319-322`)

| Topic | Msg Type | Producer → Consumer |
|---|---|---|
| `actuator_command/pose` | `geometry_msgs/PoseStamped` | Controller → Platform |
| `actuator_command/twist` | `geometry_msgs/TwistStamped` | Controller → Platform |
| `actuator_command/thrust` | `as2_msgs/Thrust` | Controller → Platform |
| `actuator_command/trajectory` | `as2_msgs/TrajectorySetpoints` | Controller → Platform |

활성 토픽 = 협상된 `ControlMode`에 의존 (한 종류만 구동).

---

## L4 — Status / Info (아래 → 위, 상태 피드백)

| Topic | Msg Type | Producer → Consumer | QoS |
|---|---|---|---|
| `platform/info` | `as2_msgs/PlatformInfo` | Platform → 전체 | `QoS(10)` |
| `controller/info` | `as2_msgs/ControllerInfo` | Controller → Behaviors | `QoS(10)` |
| `follow_target/info` | `as2_msgs/FollowTargetInfo` | FollowTarget Behavior → 사용자 | `QoS(10)` |

`PlatformInfo` 내용: `armed`, `connected`, `offboard`, `current_control_mode`, `status` (StateMachine) — `aerial_platform.hpp:273-307`.

---

## L5 — Global / Safety

QoS: `QoS(10)`

| Topic | Msg Type | Producer → Consumer | 비고 |
|---|---|---|---|
| `alert_event` | `as2_msgs/AlertEvent` | 안전노드 → Platform | namespace 밖 global. kill / stop 트리거 `aerial_platform.hpp:335` |

---

## 데이터 흐름 요약 (수직)

```
[HW]──sensor_measurements/*──▶[StateEstimator]
                                    │ self_localization/*
                                    ▼
[Behaviors]──motion_reference/*──▶[Controller]──actuator_command/*──▶[Platform]──▶[HW]
     ▲                                 │ controller/info                 │ platform/info
     └─────────────────────────────────┴─────────────────────────────────┘  (status 상향)

[Safety]──alert_event──▶[Platform]  (global, 우회 경로)
```

### 비고
- 명령(L2/L3) · 로컬라이제이션(L1) · 센서(L0) = `SensorDataQoS` best-effort. trajectory / waypoint / info 만 reliable `QoS(10)`.
- 서비스 / 액션 인터페이스(arming, control_mode 협상, behavior goal)는 별도 — `names/services.hpp`, `names/actions.hpp`.

---

_출처: `as2_core/include/as2_core/names/topics.hpp`, `sensor.hpp`, `aerial_platform.hpp` + 각 패키지 pub/sub 선언._
