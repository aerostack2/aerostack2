# 멀티 드론-GCS 통신 방법 상세 가이드

## 개요

ROS2/DDS 기반 멀티 드론 시스템에서 드론 간, 드론-GCS 간 통신을 구성하는 세 가지 방법을 설명한다.

| 방법 | 적용 범위 | 난이도 | 지연 |
|---|---|---|---|
| 방법 1: DDS 설정 튜닝 | 같은 WiFi, 근거리 | 낮음 | 최저 |
| 방법 2A: DDS Router | 인터넷/LTE, 원거리 | 중간 | 낮음 |
| 방법 2B: rosbridge WebSocket | 인터넷, 브라우저/Python GCS | 낮음 | 높음 |
| 방법 2C: Zenoh Bridge | 인터넷/LTE, 원거리 | 낮음 | 낮음~최저 |
| 방법 3: MAVLink + MAVROS | 모든 환경 | 높음 | 낮음 |

---

## 방법 1: DDS 설정 튜닝 (단거리 WiFi 환경)

### 개념

ROS2의 기본 DDS는 UDP 멀티캐스트로 노드를 발견한다. WiFi 환경에서는 멀티캐스트 패킷이 손실되거나 차단되는 경우가 많아 노드 발견이 실패한다. 이를 유니캐스트 기반 설정으로 교체하고 QoS를 튜닝하여 신뢰성을 높인다.

### 네트워크 구성 전제

```
공유기 (192.168.1.1)
  ├── 드론0 탑재 컴퓨터  192.168.1.101
  ├── 드론1 탑재 컴퓨터  192.168.1.102
  └── GCS 컴퓨터        192.168.1.200
```

각 머신에 고정 IP를 할당해야 한다.

### 설정 파일 작성

각 머신에 동일한 DDS 설정 파일을 배포한다.

**`/etc/cyclonedds.xml`**

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS>
  <Domain id="0">
    <General>
      <!-- 사용할 네트워크 인터페이스 명시 (wlan0: WiFi, eth0: 유선) -->
      <NetworkInterfaceAddress>wlan0</NetworkInterfaceAddress>
      <!-- 멀티캐스트 비활성화 -->
      <AllowMulticast>false</AllowMulticast>
    </General>

    <Discovery>
      <!-- 참가자 인덱스 자동 할당 -->
      <ParticipantIndex>auto</ParticipantIndex>
      <!-- 멀티캐스트 대신 유니캐스트로 직접 지정 -->
      <Peers>
        <Peer Address="192.168.1.101"/>
        <Peer Address="192.168.1.102"/>
        <Peer Address="192.168.1.200"/>
      </Peers>
    </Discovery>

    <Internal>
      <!-- Discovery 재시도 간격 (ms) -->
      <SocketReceiveBufferSize min="1048576"/>
    </Internal>
  </Domain>
</CycloneDDS>
```

### 환경 변수 설정

각 머신의 `.bashrc`에 추가:

```bash
# DDS 설정 파일 경로
export CYCLONEDDS_URI=file:///etc/cyclonedds.xml

# 같은 네트워크의 모든 노드가 같은 도메인에 참여
export ROS_DOMAIN_ID=0

# 멀티캐스트 비활성화 (추가 보호)
export ROS_LOCALHOST_ONLY=0
```

### ROS2 QoS 튜닝

토픽별로 QoS를 조정하여 대역폭을 절약한다.

**대역폭 높은 토픽 → BEST_EFFORT로 변경**

```python
# launch 파일 내 QoS 예시
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

sensor_qos = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,  # 손실 허용
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1
)

# 위치/속도 토픽은 최신값만 필요하므로 BEST_EFFORT 적합
pose_sub = node.create_subscription(PoseStamped, '/drone0/self_localization/pose',
                                    callback, sensor_qos)
```

**명령 토픽 → RELIABLE 유지**

```python
cmd_qos = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,  # 손실 불허
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10
)
```

### 동작 확인

```bash
# 드론0 머신에서 실행
ros2 node list        # 모든 머신의 노드가 보여야 함
ros2 topic list       # 전체 토픽 목록
ros2 topic hz /drone1/self_localization/pose  # 원격 드론 토픽 수신 확인
```

### 한계

- WiFi 범위 내(수백 m)에서만 동작
- 드론 추가 시 모든 머신의 설정 파일을 수동 업데이트해야 함
- 고속 기동 시 패킷 손실 가능

---

## 방법 2: ROS2 토픽 브릿지 (장거리/인터넷 환경)

### 개념

드론 탑재 컴퓨터(로컬 ROS2 도메인)와 원격 GCS(원격 ROS2 도메인)를 **선택적으로 연결**하는 브릿지 프로세스를 중간에 둔다. 전체 토픽을 모두 전달하지 않고 필요한 토픽만 선별하여 대역폭을 절약한다.

### 구조

```
드론 탑재 컴퓨터                              GCS (원격)
┌──────────────────────┐                  ┌─────────────────────┐
│  ROS2 도메인 0        │                  │  ROS2 도메인 0       │
│  (로컬 내부망)         │                  │  (GCS 내부망)        │
│                      │                  │                     │
│  platform            │                  │  rviz2              │
│  state_estimator     │    인터넷/LTE      │  mission_planner    │
│  motion_controller   │                  │  swarm_node         │
│  behaviors           │                  │                     │
│                      │                  │                     │
│  ┌────────────────┐  │                  │  ┌───────────────┐  │
│  │  ros2 bridge   │◄─┼──── TCP/TLS ────►│  │  ros2 bridge  │  │
│  │  (송신 선택)    │  │                  │  │  (수신 선택)   │  │
│  └────────────────┘  │                  │  └───────────────┘  │
└──────────────────────┘                  └─────────────────────┘
```

### 구현 옵션 A: `ros2router` (eProsima DDS Router)

eProsima에서 제공하는 공식 DDS 라우터.

**설치**

```bash
# 패키지 설치
sudo apt install ros-humble-rmw-fastrtps-cpp
pip install eprosima-fastdds-python

# DDS Router 설치
git clone https://github.com/eProsima/DDS-Router
cd DDS-Router && mkdir build && cd build
cmake .. && make -j4
```

**드론 탑재 컴퓨터 설정 `drone_router.yaml`**

```yaml
version: v3.0

participants:
  # 로컬 ROS2 도메인 연결
  - name: local
    kind: local
    domain: 0

  # 원격 GCS로 전달
  - name: remote_gcs
    kind: wan
    listening-addresses:
      - ip: 0.0.0.0       # 외부 접속 허용
        port: 11666
        transport: udp

# 전달할 토픽 목록 (필요한 것만 선택)
builtin-topics:
  - name: /drone0/self_localization/pose
    type: geometry_msgs::msg::dds_::PoseStamped_
  - name: /drone0/self_localization/twist
    type: geometry_msgs::msg::dds_::TwistStamped_
  - name: /drone0/platform/info
    type: as2_msgs::msg::dds_::PlatformInfo_
  # TF는 용량이 크므로 필요한 경우만 포함
```

**GCS 설정 `gcs_router.yaml`**

```yaml
version: v3.0

participants:
  - name: local
    kind: local
    domain: 0

  - name: drone0_connection
    kind: wan
    connection-addresses:
      - ip: 203.0.113.50    # 드론0 공인 IP
        port: 11666
        transport: udp

builtin-topics:
  - name: /drone0/self_localization/pose
    type: geometry_msgs::msg::dds_::PoseStamped_
  - name: /drone0/self_localization/twist
    type: geometry_msgs::msg::dds_::TwistStamped_
  - name: /Swarm/SwarmFlockingBehavior/_action/send_goal
    type: as2_msgs::action::dds_::SwarmFlocking_SendGoal_Request_
```

**실행**

```bash
# 드론 탑재 컴퓨터
ddsrouter --config-path drone_router.yaml

# GCS
ddsrouter --config-path gcs_router.yaml
```

### 구현 옵션 B: `ros2bridge` (경량 WebSocket 방식)

브라우저 기반 GCS나 Python 스크립트에서 사용할 때 적합.

```bash
# 설치
sudo apt install ros-humble-rosbridge-server

# 드론 탑재 컴퓨터에서 실행
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
# → ws://드론IP:9090 으로 접속 가능
```

```python
# GCS Python 예시
import roslibpy

client = roslibpy.Ros(host='203.0.113.50', port=9090)
client.run()

# 드론 위치 구독
pose_listener = roslibpy.Topic(client, '/drone0/self_localization/pose',
                               'geometry_msgs/PoseStamped')
pose_listener.subscribe(lambda msg: print(msg))

# 군집 명령 전송
swarm_pub = roslibpy.Topic(client, '/Swarm/dynamic_swarm_formation',
                           'as2_msgs/PoseWithIDArray')
swarm_pub.publish(roslibpy.Message({'poses': [...]}))
```

### 구현 옵션 C: `zenoh-bridge-ros2dds` (Eclipse Zenoh)

DDS Router보다 설정이 단순하고, WebSocket 방식보다 지연이 낮다. QUIC 전송을 지원하여 인터넷 환경에서 성능이 우수하다.

#### 개념

Zenoh는 pub/sub + 쿼리(request/reply)를 통합한 프로토콜이다. `zenoh-bridge-ros2dds`는 ROS2 토픽/서비스/액션을 Zenoh 키스페이스에 자동 매핑하여 원격 브릿지 인스턴스와 CDR 직렬화 그대로 교환한다. 중간에 메시지 역직렬화가 없으므로 DDS Router와 달리 메시지 타입 등록이 필요 없다.

```
드론 탑재 컴퓨터                              GCS (원격)
┌──────────────────────┐                  ┌─────────────────────┐
│  ROS2 도메인 0        │                  │  ROS2 도메인 0       │
│                      │                  │                     │
│  platform            │                  │  rviz2              │
│  behaviors           │   TCP/TLS/QUIC   │  mission_planner    │
│                      │                  │                     │
│  ┌────────────────┐  │                  │  ┌───────────────┐  │
│  │ zenoh-bridge   │◄─┼─────────────────►│  │ zenoh-bridge  │  │
│  │ -ros2dds       │  │  포트: 7447       │  │ -ros2dds      │  │
│  └────────────────┘  │                  │  └───────────────┘  │
└──────────────────────┘                  └─────────────────────┘
          │                                          │
    로컬 DDS 도메인                             로컬 DDS 도메인
    (자동 검색)                                 (자동 검색)
```

#### 설치

```bash
# ROS2 패키지로 설치 (Humble 이상)
sudo apt install ros-humble-zenoh-bridge-ros2dds

# 또는 Rust 빌드 (최신 버전)
cargo install zenoh-bridge-ros2dds --features "transport_quic"
```

#### 드론 탑재 컴퓨터 설정 `drone_zenoh.json5`

```json5
{
  // "router" 모드: 다른 클라이언트의 연결을 수락
  mode: "router",

  listen: {
    endpoints: ["tcp/0.0.0.0:7447"]
    // QUIC 사용 시 (더 낮은 지연, 패킷 손실 복구):
    // endpoints: ["quic/0.0.0.0:7447"]
  },

  plugins: {
    ros2dds: {
      // 이 브릿지가 담당할 드론 네임스페이스
      namespace: "/drone0",

      // 브릿지할 토픽 선택 (정규식)
      allow: {
        // 드론 → GCS 방향 (발행)
        publishers: [
          ".*/self_localization/.*",   // 위치, 속도
          ".*/platform/info",           // 상태 (armed, offboard)
          ".*/platform/info"
        ],
        // GCS → 드론 방향 (구독)
        subscribers: [
          ".*/Swarm/.*",               // 군집 명령
          ".*/motion_reference/.*"     // 직접 제어 명령
        ],
        // 서비스 서버 (드론 측에서 제공)
        service_servers: [
          ".*/platform/set_arming_state",
          ".*/platform/set_offboard_mode"
        ],
        // 액션 서버 (드론 측에서 제공)
        action_servers: [
          ".*/TakeOffBehavior",
          ".*/GoToBehavior",
          ".*/FollowPathBehavior"
        ],
        // TF는 고빈도이므로 명시적으로 제외
        deny: {
          publishers: ["/tf", "/tf_static"]
        }
      }
    }
  }
}
```

#### GCS 설정 `gcs_zenoh.json5`

```json5
{
  // "client" 모드: 드론 라우터에 연결
  mode: "client",

  connect: {
    endpoints: ["tcp/203.0.113.50:7447"]   // 드론 공인 IP:포트
    // 다중 드론 연결:
    // endpoints: [
    //   "tcp/203.0.113.50:7447",           // 드론0
    //   "tcp/203.0.113.51:7447"            // 드론1
    // ]
  },

  plugins: {
    ros2dds: {
      namespace: "/",    // 전체 네임스페이스 허용
      allow: {
        // GCS에서 수신할 토픽
        subscribers: [
          ".*/self_localization/.*",
          ".*/platform/info"
        ],
        // GCS에서 발행할 토픽
        publishers: [
          ".*/Swarm/.*"
        ],
        // 원격 드론의 서비스 호출
        service_clients: [
          ".*/platform/set_arming_state",
          ".*/platform/set_offboard_mode"
        ],
        // 원격 드론의 액션 호출
        action_clients: [
          ".*/TakeOffBehavior",
          ".*/GoToBehavior",
          ".*/SwarmFlockingBehavior"
        ]
      }
    }
  }
}
```

#### 실행

```bash
# 드론 탑재 컴퓨터
zenoh-bridge-ros2dds --config drone_zenoh.json5

# GCS
zenoh-bridge-ros2dds --config gcs_zenoh.json5

# GCS에서 원격 드론 토픽 확인
ros2 topic list | grep drone0          # 원격 토픽이 로컬처럼 보임
ros2 topic hz /drone0/self_localization/pose

# GCS에서 원격 액션 호출 (투명하게 동작)
ros2 action send_goal /drone0/TakeOffBehavior as2_msgs/action/TakeOff \
  "{takeoff_height: 2.0, takeoff_speed: 1.0}"
```

#### DDS Router / rosbridge와의 차이점

| 항목 | DDS Router | rosbridge | **Zenoh Bridge** |
|---|---|---|---|
| 메시지 타입 등록 | 필요 (builtin-topics) | 불필요 | **불필요** |
| 전송 프로토콜 | UDP | WebSocket/TCP | **TCP/TLS/QUIC** |
| 직렬화 오버헤드 | 없음 (CDR pass-through) | JSON 변환 있음 | **없음 (CDR pass-through)** |
| 액션 지원 | 수동 설정 필요 | 부분적 | **자동 매핑** |
| 연결 방식 | IP 직접 지정 | IP 직접 지정 | **자동 Scouting 또는 직접 지정** |
| 지연 | 낮음 | 높음 | **낮음~최저 (QUIC)** |

#### 한계

- `ros-humble-zenoh-bridge-ros2dds` 패키지는 ROS2 Humble 기준으로 버전 고정; 최신 기능은 Rust 직접 빌드 필요
- TF와 같은 고빈도 토픽을 허용하면 대역폭을 급격히 소모하므로 `deny` 설정 필수
- GCS 측에서 Zenoh 브릿지가 실행 중이어야 토픽이 보임 (브릿지 없으면 투명 연결 없음)

---

### 전달 토픽 선택 기준

```
전달 필요 (GCS가 반드시 알아야 함)
  /droneX/self_localization/pose     위치 모니터링
  /droneX/self_localization/twist    속도 모니터링
  /droneX/platform/info              드론 상태 (armed, offboard, flying)
  /Swarm/*                           군집 명령/상태

전달 불필요 (드론 내부용)
  /droneX/motion_reference/*         내부 제어 명령
  /droneX/actuator_command/*         액추에이터 명령
  /droneX/sensor_measurements/imu   고빈도 센서 데이터
  /tf (dynamic)                      100Hz TF → 대역폭 과다
```

### 한계

- 브릿지 프로세스 자체가 추가 지연(latency) 유발 (수십~수백 ms)
- 인터넷 경유 시 NAT/방화벽 포트 포워딩 필요
- Action의 경우 브릿지를 통한 goal/result/feedback 모두 전달 설정 필요

---

## 방법 3: MAVLink + MAVROS (전통적 드론 통신 방식)

### 개념

ROS2 전체를 무선으로 전송하지 않고, 드론 비행 제어에 특화된 **MAVLink** 프로토콜을 통해 핵심 정보만 교환한다. **MAVROS**는 MAVLink ↔ ROS2 변환 레이어 역할을 한다.

### 구조

```
┌── 드론 탑재 컴퓨터 ──────────────────────────────┐
│                                                  │
│  ┌─────────────────┐    ┌──────────────────────┐ │
│  │  AeroStack2      │    │  비행 제어기 (FCU)    │ │
│  │  ROS2 스택       │    │  (Pixhawk/PX4)       │ │
│  │                 │    │                      │ │
│  │  as2_platform_  │◄──►│  FCU 내부 ROS2        │ │
│  │  mavlink        │    │  (uXRCE-DDS)         │ │
│  └────────┬────────┘    └──────────────────────┘ │
│           │                                       │
│      MAVROS 노드                                  │
│           │                                       │
└───────────┼──────────────────────────────────────┘
            │
       MAVLink over
       UDP/Serial/WiFi
            │
┌── GCS 컴퓨터 ────────────────────────────────────┐
│   QGroundControl   또는   MAVROS(GCS쪽)           │
│   (MAVLink 직접 처리)      ROS2 토픽으로 변환      │
└──────────────────────────────────────────────────┘
```

### MAVLink 메시지 구조

MAVLink는 경량 바이너리 프로토콜이다. 주요 메시지:

```
HEARTBEAT         (1Hz)   드론 생존 확인, 기본 상태
GLOBAL_POSITION_INT (4Hz) GPS 위치
LOCAL_POSITION_NED  (10Hz) 로컬 좌표계 위치
ATTITUDE            (50Hz) 자세 (roll/pitch/yaw)
COMMAND_LONG              비행 명령 (이륙, 착륙, RTL 등)
SET_POSITION_TARGET_LOCAL_NED  위치/속도 목표값 전송
```

패킷 크기: 8~280 bytes → ROS2 메시지(수백 bytes)보다 훨씬 작음.

### 설치 및 설정

```bash
# MAVROS 설치
sudo apt install ros-humble-mavros ros-humble-mavros-extras

# MAVLink 지리 데이터 설치
ros2 run mavros install_geographiclib_datasets.sh
```

**드론 탑재 컴퓨터 launch 파일**

```python
# mavros_launch.py
from launch_ros.actions import Node

mavros_node = Node(
    package='mavros',
    executable='mavros_node',
    name='mavros',
    namespace='drone0',
    parameters=[{
        # FCU 연결 (시리얼 또는 UDP)
        'fcu_url': 'udp://:14540@localhost:14557',  # 시뮬레이터
        # 'fcu_url': '/dev/ttyUSB0:57600',          # 실제 Pixhawk
        'gcs_url': 'udp://@192.168.1.200:14550',   # GCS IP
        'system_id': 1,
        'component_id': 1,
        'target_system_id': 1,
        'target_component_id': 1,
    }]
)
```

### MAVROS가 제공하는 ROS2 토픽

```bash
# 상태 수신 (MAVLink → ROS2)
/drone0/mavros/state                # armed, connected, mode
/drone0/mavros/local_position/pose  # 로컬 위치
/drone0/mavros/global_position/global  # GPS 위치
/drone0/mavros/imu/data             # IMU 데이터
/drone0/mavros/battery              # 배터리 상태

# 명령 전송 (ROS2 → MAVLink)
/drone0/mavros/setpoint_position/local  # 위치 목표
/drone0/mavros/setpoint_velocity/cmd_vel  # 속도 명령
/drone0/mavros/cmd/arming           # 시동
/drone0/mavros/cmd/takeoff          # 이륙
/drone0/mavros/cmd/land             # 착륙
```

### AeroStack2와 MAVLink 연동

AeroStack2는 `as2_platform_mavlink` 패키지를 통해 MAVLink/MAVROS와 연동된다.

```
AeroStack2 behaviors
        │
  motion_controller
        │
  as2_platform_mavlink     ← AeroStack2 ↔ MAVROS 연결 레이어
        │
     MAVROS
        │
     MAVLink (UDP/Serial)
        │
     FCU (Pixhawk/PX4)
```

```bash
# as2_platform_mavlink 실행
ros2 launch as2_platform_mavlink platform_mavlink_launch.py \
  namespace:=drone0 \
  fcu_url:=udp://:14540@localhost:14557 \
  gcs_url:=udp://@192.168.1.200:14550
```

### 멀티 드론 MAVLink 구성

MAVLink는 system_id로 드론을 구분한다.

```
드론0: system_id=1, component_id=1, UDP port=14540
드론1: system_id=2, component_id=1, UDP port=14541
드론2: system_id=3, component_id=1, UDP port=14542

GCS (QGroundControl):
  14550 포트로 모든 드론의 HEARTBEAT 수신 → 자동 감지
```

```python
# 멀티 드론 MAVROS launch 예시
drones = [
    {'ns': 'drone0', 'sysid': 1, 'port': 14540},
    {'ns': 'drone1', 'sysid': 2, 'port': 14541},
    {'ns': 'drone2', 'sysid': 3, 'port': 14542},
]

nodes = []
for drone in drones:
    nodes.append(Node(
        package='mavros',
        executable='mavros_node',
        namespace=drone['ns'],
        parameters=[{
            'fcu_url': f"udp://:{drone['port']}@localhost:{drone['port']+17}",
            'gcs_url': 'udp://@192.168.1.200:14550',
            'system_id': drone['sysid'],
        }]
    ))
```

### 장거리 통신 옵션

```
드론                         GCS (수km 거리)
  MAVROS ──── MAVLink ────► QGroundControl
               │
        전송 매체 선택:
        - WiFi (수백m ~ 1km)
        - RFD900 텔레메트리 (수km, 900MHz)
        - 4G LTE (전국 범위)
        - Starlink (글로벌)
```

**LTE 환경 설정**

```bash
# 드론: 모바일 데이터로 GCS에 연결
# GCS의 공인 IP가 필요
fcu_url: 'udp://:14540@localhost:14557'
gcs_url: 'udp://@[GCS 공인 IP]:14550'
```

---

## 방법 1 + 방법 2C 조합: 드론 간 DDS, 드론-GCS 간 Zenoh

### 개념

두 메커니즘은 완전히 독립적이어서 충돌 없이 병행 동작한다.  
`zenoh-bridge-ros2dds`는 로컬 DDS 도메인을 바라보는 일반 ROS2 노드처럼 동작한다.  
방법 1이 구성한 DDS 도메인에서 토픽을 읽어 Zenoh 네트워크로 내보낼 뿐, 드론 간 DDS 통신에는 간섭하지 않는다.

### 전체 구조

```
드론0 컴퓨터 (192.168.1.101)          드론1 컴퓨터 (192.168.1.102)
┌──────────────────────────┐         ┌──────────────────────────┐
│  ROS2 DDS 도메인 0        │         │  ROS2 DDS 도메인 0        │
│  (CycloneDDS 유니캐스트)  │◄─WiFi──►│  (CycloneDDS 유니캐스트)  │
│                          │  방법 1  │                          │
│  platform / behaviors    │         │  platform / behaviors    │
│  SwarmFlockingBehavior   │         │  FollowReferenceBehavior │
│                          │         │                          │
│  ┌──────────────────┐    │         │  ┌──────────────────┐    │
│  │ zenoh-bridge     │    │         │  │ zenoh-bridge     │    │
│  │ (drone0 토픽만)  │    │         │  │ (drone1 토픽만)  │    │
│  └────────┬─────────┘    │         │  └────────┬─────────┘    │
└───────────┼──────────────┘         └───────────┼──────────────┘
            │  TCP/QUIC 방법 2C                   │  TCP/QUIC
            └─────────────────┬──────────────────┘
                              │
                   ┌──────────▼──────────┐
                   │  GCS 컴퓨터          │
                   │  zenoh-bridge(router)│
                   │  rviz2              │
                   │  mission planner    │
                   └─────────────────────┘
```

### 토픽 흐름

| 경로 | 방법 | 토픽 예시 |
|---|---|---|
| 드론0 ↔ 드론1 | 방법 1 (DDS 직접) | `/drone0/self_localization/pose`, `/tf` |
| 드론 → GCS | 방법 2C (Zenoh) | `/droneX/self_localization/pose`, `/droneX/platform/info` |
| GCS → 드론 | 방법 2C (Zenoh) | `/Swarm/SwarmFlockingBehavior` 액션 goal |

### 설정 파일

**각 드론의 `/etc/cyclonedds.xml` (방법 1 — 기존 설정 그대로)**

GCS IP를 `<Peers>`에 포함하지 않는다. 드론 간 DDS만 허용하고 GCS는 Zenoh로만 연결한다.

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS>
  <Domain id="0">
    <General>
      <NetworkInterfaceAddress>wlan0</NetworkInterfaceAddress>
      <AllowMulticast>false</AllowMulticast>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <Peers>
        <Peer Address="192.168.1.101"/>  <!-- 드론0 -->
        <Peer Address="192.168.1.102"/>  <!-- 드론1 -->
        <!-- GCS(192.168.1.200)는 여기에 넣지 않음 -->
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
```

**드론0의 `drone0_zenoh.json5` (방법 2C)**

```json5
{
  mode: "client",
  connect: {
    endpoints: ["tcp/[GCS 공인 IP]:7447"]
  },
  plugins: {
    ros2dds: {
      namespace: "/drone0",    // 자기 네임스페이스 토픽만 담당
      allow: {
        publishers: [
          "/drone0/self_localization/.*",
          "/drone0/platform/info"
        ],
        subscribers: [
          "/Swarm/.*"           // GCS에서 오는 군집 명령 수신
        ],
        action_servers: [
          "/drone0/TakeOffBehavior",
          "/drone0/GoToBehavior"
        ]
      }
    }
  }
}
```

**드론1의 `drone1_zenoh.json5`** — `namespace`와 `publishers` 경로만 `/drone1`로 변경, 구조 동일

**GCS의 `gcs_zenoh.json5`**

```json5
{
  mode: "router",    // 모든 드론이 여기로 연결
  listen: {
    endpoints: ["tcp/0.0.0.0:7447"]
  },
  plugins: {
    ros2dds: {
      namespace: "/",
      allow: {
        subscribers: [
          ".*/self_localization/.*",
          ".*/platform/info"
        ],
        publishers: ["/Swarm/.*"],
        action_clients: [
          ".*/TakeOffBehavior",
          ".*/GoToBehavior",
          "/Swarm/SwarmFlockingBehavior"
        ]
      }
    }
  }
}
```

### 실행 순서

```bash
# 1. 각 드론: cyclonedds.xml 환경 변수 설정 후 AeroStack2 기동
export CYCLONEDDS_URI=file:///etc/cyclonedds.xml
export ROS_DOMAIN_ID=0
ros2 launch ... # platform, state_estimator, behaviors 등

# 2. GCS: zenoh-bridge router 먼저 기동
zenoh-bridge-ros2dds --config gcs_zenoh.json5

# 3. 각 드론: zenoh-bridge client 기동 (AeroStack2와 별개 프로세스)
zenoh-bridge-ros2dds --config drone0_zenoh.json5   # 드론0
zenoh-bridge-ros2dds --config drone1_zenoh.json5   # 드론1

# 4. GCS: 연결 확인
ros2 topic list | grep self_localization   # 원격 드론 토픽이 로컬처럼 보여야 함
ros2 topic hz /drone0/self_localization/pose
ros2 topic hz /drone1/self_localization/pose
```

### SwarmFlockingBehavior 노드 위치 결정

Zenoh 구성 복잡도가 달라지므로 배치를 명확히 해야 한다.

**권장: 드론0에 배치**

```
GCS → /Swarm/SwarmFlockingBehavior (Zenoh) → 드론0
드론0 → /drone1/FollowReferenceBehavior   (DDS 직접, 방법 1)
```

Zenoh에서 노출해야 할 액션이 `SwarmFlockingBehavior` 하나뿐이다.

**대안: GCS에 배치**

```
GCS → /droneX/FollowReferenceBehavior (Zenoh) → 각 드론
```

각 드론의 `allow.action_servers`에 `FollowReferenceBehavior`를 추가해야 하고,  
GCS ↔ 드론 간 TF 조회(`Swarm/droneX_ref` 등)가 Zenoh를 통과해야 하므로 설정이 복잡해진다.

---

## 방법 비교 및 선택 기준

| 항목 | 방법 1 (DDS 튜닝) | 방법 2A (DDS Router) | 방법 2B (rosbridge) | 방법 2C (Zenoh) | **방법 1+2C (조합)** | 방법 3 (MAVLink) |
|---|---|---|---|---|---|---|
| **통신 거리** | ~수백m (WiFi) | 제한 없음 | 제한 없음 | 제한 없음 | **드론간: WiFi / GCS: 제한 없음** | 제한 없음 |
| **드론 간 지연** | 최저 (<10ms) | N/A | N/A | N/A | **최저 (<10ms, DDS 직접)** | 낮음 |
| **GCS 지연** | N/A | 낮음 (20~50ms) | 높음 (50~200ms) | 낮음~최저 | **낮음~최저 (QUIC)** | 낮음 (20~100ms) |
| **대역폭 사용** | 높음 (전체 토픽) | 조절 가능 | 조절 가능 | 조절 가능 | **드론간: 높음 / GCS: 조절 가능** | 낮음 |
| **구현 복잡도** | 낮음 | 중간 | 낮음 | 낮음 | **중간** | 높음 |
| **메시지 타입 등록** | 불필요 | 필요 | 불필요 | 불필요 | **불필요** | 불필요 |
| **액션 지원** | 자동 | 수동 설정 | 부분적 | 자동 매핑 | **자동 매핑** | MAVLink 한정 |
| **기존 GCS 호환** | 불가 (ROS2 only) | 부분 | 부분 | 부분 | **부분** | 가능 (QGC 등) |
| **드론 추가** | 설정 파일 수정 | 브릿지 설정 추가 | 브릿지 설정 추가 | 엔드포인트 추가 | **cyclonedds.xml + zenoh 설정 추가** | system_id 추가 |
| **보안** | 없음 | TLS 적용 가능 | TLS 적용 가능 | TLS/QUIC 내장 | **드론간: 없음 / GCS: TLS/QUIC** | 기본 없음 |
| **근거리 군집 비행** | 적합 | 부적합 | 부적합 | 부적합 | **적합** | 가능 |
| **원격 GCS 모니터링** | 불가 | 가능 | 가능 | 가능 | **가능** | 가능 |

### 권장 선택 기준

```
실내 시뮬레이션 / 동일 머신
  → 방법 없음 (기본 DDS로 동작)

실외 근거리, 드론 간 군집 비행만 필요 (GCS 없음)
  → 방법 1 (DDS 튜닝)

실외 근거리 군집 비행 + 원격 GCS 모니터링/명령
  → 방법 1 + 2C (조합) 권장
    · 드론 간: DDS 직접 (최저 지연)
    · GCS 연결: Zenoh Bridge (QUIC, 액션 자동 매핑)

장거리 / 인터넷 경유, GCS만 연결 (드론 간 동일 망)
  → 방법 2C (Zenoh Bridge) 단독

장거리 / 인터넷 경유, 특정 토픽만 세밀하게 선별
  → 방법 2A (DDS Router)

장거리 / Python·브라우저 GCS 연동
  → 방법 2B (rosbridge WebSocket)

실제 비행 제어기(Pixhawk) 사용
  → 방법 3 (MAVLink + MAVROS) 필수
    드론 간 편대 비행 + GCS 모니터링은 방법 1+2C 추가 가능

상용 운용 (안전/신뢰성 중요)
  → 방법 3 기반 + 방법 2C 모니터링 병행
```

---

## 참고: WSL 환경에서의 주의사항

WSL2는 NAT 기반 가상 네트워크를 사용하므로 외부 장치(실제 드론)와의 ROS2 직접 통신에 제약이 있다.

```bash
# WSL2에서 포트 포워딩 설정 (Windows PowerShell, 관리자 권한)
netsh interface portproxy add v4tov4 \
  listenport=14550 listenaddress=0.0.0.0 \
  connectport=14550 connectaddress=$(wsl hostname -I)

# 방화벽 허용
netsh advfirewall firewall add rule name="ROS2 DDS" \
  protocol=UDP dir=in action=allow localport=7400-7500

# WSL2 IP 확인 (재부팅마다 변경됨)
wsl hostname -I
```
