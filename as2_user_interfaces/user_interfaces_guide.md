# as2 User Interfaces 가이드

`as2_user_interfaces` 하위 패키지인 `as2_alphanumeric_viewer`와
`as2_keyboard_teleoperation`의 역할, 구조, 실행 방법을 정리한다.

---

## 1. as2_alphanumeric_viewer

### 역할

**단일 드론 전용** 터미널 기반 실시간 상태 모니터.
ncurses 라이브러리를 사용해 GUI 없이 터미널에서 드론 상태를 표시한다.
읽기 전용 — 명령을 보내지 않는다.

- 언어: C++ (ament_cmake)
- 노드 타입: Lifecycle Node (as2::Node 상속)

---

### 화면 구성

키보드로 화면을 전환한다.

| 키 | 화면 | 표시 내용 |
|----|------|----------|
| `M` | Summary | 위치/속도/IMU/플랫폼 상태/제어모드 전체 요약 |
| `S` | Sensors | 배터리/IMU/GPS/고도/온도 상세 |
| `N` | Navigation | 측위 위치/속도/자세 상세 |
| `P` | Platform | 액추에이터 명령/레퍼런스/제어모드 상세 |
| `←` `→` | — | 화면 간 순환 이동 |

배터리 잔량에 따라 색상이 변화한다.

| 잔량 | 색상 |
|------|------|
| > 50% | 녹색 |
| 20 ~ 50% | 노랑 |
| < 20% | 빨강 |

---

### 구독 토픽

노드 네임스페이스는 `drone_id` 파라미터로 지정된다.
모든 토픽은 `{drone_id}/` 하위에서 구독한다.

| 토픽 | 메시지 타입 | 표시 화면 |
|------|------------|----------|
| `self_localization/pose` | PoseStamped | Summary, Navigation, Platform |
| `self_localization/twist` | TwistStamped | Summary, Navigation |
| `sensor_measurements/battery` | BatteryState | Summary, Sensors |
| `sensor_measurements/imu` | Imu | Summary, Sensors, Navigation |
| `sensor_measurements/gps` | NavSatFix | Sensors |
| `platform/info` | PlatformInfo | Summary, Platform |
| `actuator_command/pose` | PoseStamped | Platform |
| `actuator_command/thrust` | Thrust | Platform |
| `actuator_command/twist` | TwistStamped | Platform |
| `controller/info` | ControllerInfo | Summary, Platform |
| `motion_reference/pose` | PoseStamped | Platform |
| `motion_reference/twist` | TwistStamped | Platform |

---

### 실행 방법

#### 빌드

```bash
cd ~/ros2_ws
colcon build --packages-select as2_alphanumeric_viewer
source install/setup.bash
```

#### 기본 실행 (drone0)

```bash
ros2 launch as2_alphanumeric_viewer alphanumeric_viewer_launch.py
```

#### 드론 네임스페이스 지정

```bash
ros2 launch as2_alphanumeric_viewer alphanumeric_viewer_launch.py \
  drone_id:=drone1
```

#### 시뮬레이션 시간 사용

```bash
ros2 launch as2_alphanumeric_viewer alphanumeric_viewer_launch.py \
  drone_id:=drone0 \
  use_sim_time:=true
```

#### 런치 인자 목록

| 인자 | 기본값 | 설명 |
|------|--------|------|
| `drone_id` | `drone0` | 모니터링할 드론 네임스페이스 |
| `use_sim_time` | `false` | 시뮬레이션 시간 사용 여부 |

#### 주의사항

- **단일 드론만** 지정 가능하다. 여러 드론을 동시에 보려면 터미널을 여러 개 실행한다.
- `emulate_tty: true` 로 실행되므로 반드시 터미널 환경이 필요하다.
- SSH 원격 접속 환경에서도 동작한다.

---

---

## 2. as2_keyboard_teleoperation

### 역할

**다중 드론 지원** PySimpleGUI 기반 키보드 조종 인터페이스.
드론 목록을 직접 지정하고, 각 드론에 속도/위치 명령을 키보드로 전송한다.

- 언어: Python (ament_python)
- GUI: PySimpleGUI (`DarkBlack1` 테마)

---

### 구성 클래스

```
KeyboardTeleoperation          ← 진입점, 50ms 단위 메인 루프
    ├── DroneManager           ← 키 입력을 드론 제어 명령으로 변환
    │     └── DroneInterfaceTeleop × N  ← 드론 1개당 ROS2 노드 1개
    ├── MainWindow             ← 키 입력 수신 + 드론 선택 체크박스
    ├── SettingsWindow         ← 속도/각도 값 실시간 조정
    └── LocalizationWindow     ← 드론별 현재 위치 표시
```

---

### 제어 모드

| 모드 | 설명 |
|------|------|
| `SPEED_CONTROL` | 속도 명령 (earth 또는 base_link 프레임) |
| `POSE_CONTROL` | 위치 명령 (earth 프레임) |
| `BODY_POSE_CONTROL` | 위치 명령 (body 프레임 — yaw 회전량 고려) |

---

### 공통 키 명령

| 키 | 동작 |
|-----|------|
| TAKE_OFF | arm → offboard → takeoff 순서 자동 실행 |
| LAND | 0.5 m/s 속도로 착륙 |
| HOVER | motion_reference_handler로 제자리 호버 |
| EMERGENCY | 모터 강제 정지 (kill switch) |

---

### 다중 드론 처리 방식

네임스페이스를 콤마(`,`), 콜론(`:`), 공백으로 구분해서 여러 드론을 동시 제어한다.

```
namespace:=drone0,drone1,drone2
```

- 드론마다 `DroneInterfaceTeleop` 노드를 별도 생성한다.
- MainWindow의 체크박스로 개별 드론을 선택/해제할 수 있다.
- 선택된 드론에만 명령이 전송된다.

---

### SwarmBehaviorManager 연동

실행 중인 behavior를 일시정지/재개하는 기능이 내장되어 있다.

| UI 이벤트 | 동작 |
|----------|------|
| PAUSE_BEHAVIORS | 선택된 드론의 behavior 일시정지 |
| RESUME_BEHAVIORS | 선택된 드론의 behavior 재개 |
| PAUSE_ALL_BEHAVIORS | 모든 드론의 behavior 일시정지 |
| RESUME_ALL_BEHAVIORS | 모든 드론의 behavior 재개 |

---

### 설정 파일 (teleop_values_config.yaml)

```yaml
/**:
  ros__parameters:
    namespace: "drone0"       # 드론 네임스페이스 (콤마 구분으로 다중 지정)
    use_sim_time: True
    verbose: False
    speed_value: 0.5          # 속도 모드 이동 속도 (m/s)
    altitude_speed_value: 0.5 # 속도 모드 고도 속도 (m/s)
    turn_speed_value: 0.5     # 속도 모드 회전 속도 (rad/s)
    position_value: 1.00      # 위치 모드 이동 거리 (m)
    altitude_value: 1.00      # 위치 모드 고도 변화 (m)
    turn_angle_value: 0.78    # 위치 모드 회전 각도 (rad, ≈45°)
    drone_frequency: 100.0    # DroneInterface 스핀 주기 (Hz)
    speed_frame_id: "earth"   # 속도 모드 기준 프레임
    pose_frame_id: "earth"    # 위치 모드 기준 프레임
    initial_mode: "pose"      # 시작 제어 모드 (speed / pose)
    modules: ""               # 추가 모듈 (예: "follow_reference")
```

---

### 실행 방법

#### 빌드

```bash
cd ~/ros2_ws
colcon build --packages-select as2_keyboard_teleoperation
source install/setup.bash
```

#### 기본 실행 (drone0, 설정 파일 기본값 사용)

```bash
ros2 launch as2_keyboard_teleoperation as2_keyboard_teleoperation_launch.py
```

#### 단일 드론 네임스페이스 지정

```bash
ros2 launch as2_keyboard_teleoperation as2_keyboard_teleoperation_launch.py \
  namespace:=drone1
```

#### 다중 드론 지정

```bash
# 콤마 구분
ros2 launch as2_keyboard_teleoperation as2_keyboard_teleoperation_launch.py \
  namespace:=drone0,drone1,drone2

# 콜론 구분도 가능
ros2 launch as2_keyboard_teleoperation as2_keyboard_teleoperation_launch.py \
  namespace:=drone0:drone1:drone2
```

#### 시뮬레이션 시간 + 속도 모드로 시작

```bash
ros2 launch as2_keyboard_teleoperation as2_keyboard_teleoperation_launch.py \
  namespace:=drone0,drone1 \
  use_sim_time:=true
```

#### 커스텀 설정 파일 사용

```bash
ros2 launch as2_keyboard_teleoperation as2_keyboard_teleoperation_launch.py \
  config_file:=/path/to/my_teleop_config.yaml
```

#### 런치 인자 목록

| 인자 | 기본값 | 설명 |
|------|--------|------|
| `namespace` | `drone0` | 드론 네임스페이스 (콤마/콜론으로 다중 지정) |
| `use_sim_time` | `true` (설정파일) | 시뮬레이션 시간 사용 여부 |
| `verbose` | `false` | 상세 로그 출력 여부 |
| `config_file` | 패키지 기본 yaml | 파라미터 설정 파일 경로 |

#### 주의사항

- PySimpleGUI가 설치되어 있어야 한다: `pip install PySimpleGUI`
- GUI 창이 열리므로 **디스플레이 환경**이 필요하다 (SSH 원격 시 X forwarding 또는 VNC 필요).
- 드론 목록은 런치 시 고정된다. 실행 중 드론 추가/제거는 불가능하다.
  → fleet_manager 연동으로 개선 가능 (`/fleet_manager/drone_namespaces` 구독).

---

## 두 패키지 비교

| 항목 | as2_alphanumeric_viewer | as2_keyboard_teleoperation |
|------|------------------------|---------------------------|
| 언어 | C++ | Python |
| UI | ncurses (터미널) | PySimpleGUI (GUI 창) |
| 드론 수 | **단일** | **다중** |
| 목적 | 읽기 전용 모니터링 | 키보드 조종 |
| 드론 지정 | `drone_id` 인자 1개 | `namespace` 인자 (콤마 구분) |
| 디스플레이 | 불필요 (터미널만) | 필요 (GUI 창) |
| 제어 명령 | 없음 | 속도/위치/이착륙/긴급정지 |
| fleet_manager 연동 | 해당 없음 (단일 드론) | 가능 (namespace 목록 자동 수신) |
