# Fleet Manager 설계 문서

## 배경 및 목적

aerostack2 기존 구조는 드론 목록을 런치 파라미터(`namespace_list`)로 수동 입력한다.
드론이 부팅되면 네트워크에서 자동 탐지하고, 상태를 모니터링하며, 군집 그룹을 동적으로
구성할 수 있도록 Fleet Manager를 추가한다.

---

## 기존 구조의 한계

| 패키지 | 현황 |
|--------|------|
| `as2_visualization/swarm_viz.launch.py` | `namespace_list` 파라미터로 하드코딩 |
| `as2_keyboard_teleoperation/drone_manager.py` | `DroneInterface` 목록을 수동 생성 |
| `as2_behaviors_swarm_flocking` | Goal의 `drones_namespace[]` 명시 필요 |

---

## 드론 탐지 전략

### 선택: `get_topic_names_and_types()` 기반

`get_node_names_and_namespaces()` 대신 토픽 기반 탐지를 선택한다.

**이유:** `*/platform/info` 토픽은 as2 드론만 발행하므로 타 ROS2 노드와 오인 없음.

```
get_topic_names_and_types()
  → "/drone0/platform/info" 발견
  → 네임스페이스 "/drone0" 추출
  → 드론으로 등록
```

**탐지 범위:** 동일 `ROS_DOMAIN_ID` + 동일 네트워크(DDS Discovery 범위)

---

## 아키텍처

```
┌──────────────────────────────────────────────────────────┐
│  운용자 UI / as2_visualization                            │
│  /fleet_manager/drone_namespaces 구독                     │
│  /fleet_manager/fleet_status 구독                         │
├──────────────────────────────────────────────────────────┤
│  as2_fleet_manager (신규 패키지)                          │
│  FleetManagerNode                                         │
│    ├── FleetMonitor — ROS2 그래프 스캔 + 상태 구독        │
│    └── SwarmConfigurator — SwarmFlocking 액션 클라이언트  │
├──────────────────────────────────────────────────────────┤
│  기존 as2 인프라 (무수정)                                  │
│  platform/info, self_localization/pose                    │
│  SwarmFlocking action, ModifySwarm service                │
└──────────────────────────────────────────────────────────┘
```

---

## 패키지 구조

```
as2_user_interfaces/as2_fleet_manager/
├── package.xml
├── setup.py
├── resource/as2_fleet_manager
├── as2_fleet_manager/
│   ├── __init__.py
│   ├── fleet_manager_node.py     # 메인 ROS2 노드
│   └── swarm_configurator.py     # 편대 구성 + 액션 클라이언트
├── launch/
│   └── fleet_manager.launch.py
└── config/
    └── fleet_manager.yaml

as2_python_api/as2_python_api/
└── fleet_monitor.py              # 드론 발견/모니터링 라이브러리
```

---

## 발행 토픽

| 토픽 | 타입 | 내용 |
|------|------|------|
| `/fleet_manager/drone_namespaces` | `std_msgs/String` | JSON 배열 — 탐지된 드론 네임스페이스 |
| `/fleet_manager/fleet_status` | `std_msgs/String` | JSON 딕셔너리 — 드론별 상태 |

### fleet_status JSON 구조

```json
{
  "/drone0": {
    "connected": true,
    "armed": false,
    "offboard": false,
    "state": 1,
    "position": [0.0, 0.0, 0.0]
  },
  "/drone1": { ... }
}
```

---

## 모니터링 대상 토픽

| 토픽 | 정보 | 우선순위 |
|------|------|---------|
| `{ns}/platform/info` | connected, armed, offboard, state | 필수 |
| `{ns}/self_localization/pose` | 위치 (x, y, z) | 필수 |

---

## 안전 조건

`request_swarm()` 호출 시 모든 대상 드론에 대해:
1. `drones` 딕셔너리에 존재하는지 확인
2. `platform/info.connected == True` 확인
3. 둘 중 하나라도 실패하면 Goal 전송 거부

---

## FleetMonitor 설계 원칙

`FleetMonitor`는 `Node`를 상속하지 않고, 외부 `Node`를 주입받아 타이머/구독을 등록한다.

**이유:** `FleetManagerNode` 하나만 ROS2 노드로 존재하게 하여 실행 복잡도를 낮춤.
기존 `as2_python_api`의 `shared_data.PlatformInfoData`를 그대로 재사용.

```python
class FleetMonitor:
    def __init__(self, node: Node, scan_interval=5.0, ...):
        node.create_timer(scan_interval, self._scan_network)
        node.create_timer(drone_timeout / 2, self._check_timeouts)
```

---

## 편대 유형 (SwarmConfigurator)

| 타입 | 설명 | 파라미터 |
|------|------|---------|
| `line` | 일렬 종대 (기본) | `spacing` (m) |
| `circle` | 원형 배치 | `radius` (m) |

---

## 연동 계획

### as2_visualization (소수정)
`swarm_viz.launch.py`의 `namespace_list` 파라미터를
`/fleet_manager/drone_namespaces` 토픽 구독으로 대체 가능.

### as2_keyboard_teleoperation (선택적 연동)
`DroneManager` 생성 시 네임스페이스 목록을 fleet_manager에서 조회 가능.

---

## 구현 범위

| 대상 | 변경 유형 | 규모 |
|------|----------|------|
| `as2_fleet_manager` (신규) | 생성 | ~250줄 |
| `as2_python_api/fleet_monitor.py` (신규) | 생성 | ~120줄 |
| `as2_visualization` | 선택적 수정 | 소규모 |
| `as2_behaviors_swarm_flocking` | 무수정 | — |
| `as2_msgs` | 무수정 | — |
