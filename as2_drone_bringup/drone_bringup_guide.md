# as2_drone_bringup 가이드

드론 부팅 시 필수 노드(platform / state_estimator / controller_manager)와
대기 준비 Behaviors(Takeoff / Land / GoTo / FollowPath)를 자동으로 실행하는
bringup 패키지 사용 가이드.

---

## 패키지 구조

```
as2_user_interfaces/as2_drone_bringup/
├── package.xml
├── setup.py
├── config/
│   └── drone_bringup.yaml       ← 모든 노드 파라미터를 한 파일에서 관리
├── launch/
│   └── drone_bringup.launch.py  ← 4단계 순서 보장 런치
└── scripts/
    ├── drone_bringup.sh          ← 편의 셸 스크립트
    └── as2-drone.service         ← systemd 자동 시작 템플릿
```

---

## 실행 타임라인

`TimerAction` 으로 노드 간 시작 순서를 보장한다.
각 지연 값은 `drone_bringup.yaml`의 `startup_delay` 항목에서 조정할 수 있다.

```
t=0s   [Phase 1-1] platform 시작
           → hardware(PX4/ArduPilot) 또는 시뮬레이터와 연결
           → platform/info 발행 시작

t=3s   [Phase 1-2] state_estimator 시작
           → platform 센서 데이터 수신
           → self_localization/pose, twist 발행 시작

t=5s   [Phase 1-3] controller_manager 시작
           → self_localization/* 피드백 수신
           → motion_reference → actuator_command 제어 루프 가동

t=7s   [Phase 2]  motion behaviors 시작 (ComposableNodeContainer)
           → TakeoffBehavior  (LANDED 상태에서 Action 호출 대기)
           → LandBehavior     (FLYING 상태에서 Action 호출 대기)
           → GoToBehavior     (FLYING 상태에서 Action 호출 대기)
           → FollowPathBehavior (FLYING 상태에서 Action 호출 대기)
```

---

## 설정 파일 (drone_bringup.yaml)

모든 노드 파라미터를 하나의 파일에서 관리한다.
런치 파일이 이 파일을 읽어 각 노드에 파라미터를 분배한다.

```yaml
# 공통 설정
namespace: drone0
use_sim_time: false

# 노드 시작 지연 (초) — 하드웨어 초기화 속도에 맞게 조정
startup_delay:
  state_estimator: 3.0
  controller: 5.0
  behaviors: 7.0

# platform 노드
platform:
  pkg: as2_platform_mavlink    # 실제: as2_platform_mavlink
                                # 시뮬: as2_platform_multirotor_simulator

# state_estimator 노드 — 플러그인 선택
state_estimator:
  plugin_name: ground_truth    # ground_truth / mocap_pose / raw_odometry
                                # / ground_truth_odometry_fuse

# controller_manager 노드 — 플러그인 선택
controller:
  plugin_name: pid_speed_controller  # pid_speed_controller
                                      # / differential_flatness_controller

# motion behaviors — 비워두면 각 behavior 패키지 기본값 사용
behaviors:
  takeoff_plugin_name: ""
  land_plugin_name: ""
  go_to_plugin_name: ""
  follow_path_plugin_name: ""
```

### state_estimator 플러그인 선택 기준

| 플러그인 | 입력 소스 | 사용 환경 |
|---------|----------|----------|
| `ground_truth` | `ground_truth/pose`, `ground_truth/twist` | 시뮬레이션 |
| `ground_truth_odometry_fuse` | ground_truth + odometry | 시뮬레이션 + 오도메트리 |
| `mocap_pose` | `sensor_measurements/mocap` | 실내 모션캡처 |
| `raw_odometry` | `sensor_measurements/odom` | 외부 오도메트리 |

### controller 플러그인 선택 기준

| 플러그인 | 특징 | 사용 환경 |
|---------|------|----------|
| `pid_speed_controller` | PID 기반 속도 제어 | 일반 멀티로터 |
| `differential_flatness_controller` | 미분 평탄화 기반 궤적 제어 | 고기동 비행 |

---

## 빌드

```bash
cd ~/ros2_ws
colcon build --packages-select as2_drone_bringup
source install/setup.bash
```

---

## 실행 방법

### 방법 1 — 직접 런치 (개발/테스트)

```bash
# 기본 실행 (drone_bringup.yaml 기본값 사용)
ros2 launch as2_drone_bringup drone_bringup.launch.py

# 네임스페이스 지정
ros2 launch as2_drone_bringup drone_bringup.launch.py \
  namespace:=drone1

# 시뮬레이션 모드
ros2 launch as2_drone_bringup drone_bringup.launch.py \
  use_sim_time:=true

# 커스텀 설정 파일 지정
ros2 launch as2_drone_bringup drone_bringup.launch.py \
  config:=/path/to/my_config.yaml

# 여러 옵션 조합
ros2 launch as2_drone_bringup drone_bringup.launch.py \
  namespace:=drone0 \
  use_sim_time:=true \
  log_level:=debug
```

**런치 인자 목록:**

| 인자 | 기본값 | 설명 |
|------|--------|------|
| `config` | 패키지 기본 yaml | drone_bringup.yaml 경로 |
| `namespace` | config 파일 값 | 드론 네임스페이스 (지정 시 config 값 override) |
| `use_sim_time` | config 파일 값 | 시뮬레이션 시간 사용 여부 |
| `log_level` | `info` | 로그 레벨 (debug/info/warn/error) |

---

### 방법 2 — 셸 스크립트 (drone_bringup.sh)

ROS2 환경 소스와 런치 호출을 자동화한다.

```bash
# 실행 권한 부여 (최초 1회)
chmod +x ~/ros2_ws/install/as2_drone_bringup/share/as2_drone_bringup/scripts/drone_bringup.sh

# 기본 실행
./drone_bringup.sh

# 드론 네임스페이스 지정
./drone_bringup.sh -n drone1

# 시뮬레이션 모드
./drone_bringup.sh -n drone0 --sim

# 커스텀 설정 파일
./drone_bringup.sh -c /path/to/my_config.yaml

# 디버그 로그
./drone_bringup.sh -n drone0 -l debug
```

**스크립트 옵션:**

| 옵션 | 설명 |
|------|------|
| `-n`, `--namespace <name>` | 드론 네임스페이스 |
| `-c`, `--config <path>` | drone_bringup.yaml 경로 |
| `-s`, `--sim` | 시뮬레이션 시간 사용 (use_sim_time=true) |
| `-l`, `--log-level <level>` | 로그 레벨 |
| `-h`, `--help` | 도움말 출력 |

**환경 변수:**

| 변수 | 기본값 | 설명 |
|------|--------|------|
| `ROS_DISTRO` | `humble` | ROS2 배포판 |
| `AS2_WS` | `~/ros2_ws` | 워크스페이스 경로 |

```bash
# 환경변수로 워크스페이스 경로 지정
AS2_WS=/home/user/my_ws ./drone_bringup.sh -n drone0
```

---

### 방법 3 — systemd 서비스 (부팅 시 자동 실행)

드론 컴퓨터 부팅 시 자동으로 as2 노드를 시작한다.

#### 설치 절차

```bash
# 1. 서비스 파일 복사
sudo cp ~/ros2_ws/install/as2_drone_bringup/share/as2_drone_bringup/scripts/as2-drone.service \
     /etc/systemd/system/

# 2. 서비스 파일 수정 (필수)
sudo nano /etc/systemd/system/as2-drone.service
```

수정이 필요한 항목:

```ini
[Service]
User=ubuntu               # ← 실제 사용자 이름으로 변경
Environment="ROS_DISTRO=humble"       # ← ROS2 배포판
Environment="AS2_WS=/home/ubuntu/ros2_ws"  # ← 워크스페이스 경로
Environment="ROS_DOMAIN_ID=0"        # ← 도메인 ID
Environment="AS2_NAMESPACE=drone0"   # ← 드론 네임스페이스

ExecStart=/home/ubuntu/ros2_ws/install/as2_drone_bringup/share/\
as2_drone_bringup/scripts/drone_bringup.sh --namespace %E{AS2_NAMESPACE}
```

```bash
# 3. systemd 데몬 재로드 및 서비스 활성화
sudo systemctl daemon-reload
sudo systemctl enable as2-drone.service   # 부팅 시 자동 시작 등록
sudo systemctl start  as2-drone.service   # 즉시 시작

# 4. 상태 확인
sudo systemctl status as2-drone.service

# 실시간 로그 확인
journalctl -u as2-drone.service -f
```

#### 서비스 관리 명령

```bash
# 시작 / 정지 / 재시작
sudo systemctl start   as2-drone.service
sudo systemctl stop    as2-drone.service
sudo systemctl restart as2-drone.service

# 자동 시작 활성화 / 비활성화
sudo systemctl enable  as2-drone.service
sudo systemctl disable as2-drone.service

# 로그 확인 (최근 100줄)
journalctl -u as2-drone.service -n 100
```

---

## 다중 드론 운용

드론마다 네임스페이스와 설정 파일을 분리하여 각각 실행한다.

```bash
# 드론별 설정 파일 준비
cp drone_bringup.yaml drone0_config.yaml   # namespace: drone0
cp drone_bringup.yaml drone1_config.yaml   # namespace: drone1

# 각각 별도 터미널에서 실행
ros2 launch as2_drone_bringup drone_bringup.launch.py config:=drone0_config.yaml
ros2 launch as2_drone_bringup drone_bringup.launch.py config:=drone1_config.yaml
```

systemd 방식은 드론마다 서비스 파일을 별도로 생성한다.
```bash
# drone0 서비스
sudo cp as2-drone.service /etc/systemd/system/as2-drone0.service
# Environment="AS2_NAMESPACE=drone0" 설정

# drone1 서비스
sudo cp as2-drone.service /etc/systemd/system/as2-drone1.service
# Environment="AS2_NAMESPACE=drone1" 설정
```

---

## 동작 확인

bringup 후 다음 명령으로 노드와 토픽이 정상 실행되는지 확인한다.

```bash
# 노드 목록 확인
ros2 node list
# /drone0/platform
# /drone0/state_estimator
# /drone0/controller_manager
# /drone0/behaviors (TakeoffBehavior, LandBehavior, GoToBehavior, FollowPathBehavior)

# 필수 토픽 확인
ros2 topic echo /drone0/platform/info
ros2 topic echo /drone0/self_localization/pose
ros2 topic echo /drone0/actuator_command/twist

# 플랫폼 상태 확인 (connected: true 이어야 정상)
ros2 topic echo /drone0/platform/info --once
```

---

## 문제 해결

### 노드가 시작되지 않는 경우

```bash
# 로그 확인
ros2 launch as2_drone_bringup drone_bringup.launch.py log_level:=debug

# 패키지가 설치되었는지 확인
ros2 pkg list | grep as2
```

### 상태 추정 토픽이 발행되지 않는 경우

- `state_estimator.plugin_name` 이 환경(시뮬/실제)에 맞는지 확인
- `startup_delay.state_estimator` 를 늘려 platform 초기화를 더 기다리도록 설정

### 제어 명령이 전달되지 않는 경우

- `startup_delay.controller` 를 늘려 state_estimator 준비 후 시작하도록 설정
- `controller.plugin_name` 이 플랫폼 제어 모드와 호환되는지 확인

### systemd 서비스 시작 실패

```bash
journalctl -u as2-drone.service --no-pager
# User, ExecStart 경로, 환경변수 설정 재확인
```
