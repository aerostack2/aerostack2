# 군집 드론 시스템 — 군집 조율 레이어 최적 노드 조합 설계

> **운용 조건:** 실외 / 정찰 임무 / GPS 유 / 분산 계층형

---

## 1. 전체 아키텍처

```
┌─────────────────────────────────────────────────────┐
│                  GCS (Ground Control)               │
│         Mission Upload / 실시간 모니터링 / 영상수신       │
└──────────────────────┬──────────────────────────────┘
                       │ MAVLink / CycloneDDS
        ┌──────────────┼──────────────┐
        ▼              ▼              ▼
  [Cluster A]    [Cluster B]    [Cluster C]
  Leader Drone   Leader Drone   Leader Drone
   (정찰구역 1)    (정찰구역 2)    (정찰구역 3)
   ↓  ↓  ↓        ↓  ↓  ↓        ↓  ↓  ↓
  D1 D2 D3       D4 D5 D6       D7 D8 D9
```

---

## 2. 계층별 노드 상세 구성

### 🔵 Layer 1 — GCS Mission Layer

| 노드 | 기능 | 기술 스택 |
|------|------|-----------|
| **Mission Planner Node** | 정찰 구역 분할, 경로 생성 | ROS2 + MAVSDK + QGroundControl |
| **Area Decomposition Node** | 구역을 클러스터별 격자 분할 | Boustrophedon / Voronoi Partition |
| **ISR Fusion Node** | 각 드론 영상·센서 데이터 통합 | OpenCV + GStreamer |
| **Replay & Log Node** | 임무 기록, 재현 | ROS2 bag + InfluxDB |

---

### 🟠 Layer 2 — Cluster Coordination Layer (핵심)

> 각 클러스터 리더 드론이 탑재하는 노드 조합

| 노드 | 기능 | 알고리즘 / 스택 |
|------|------|----------------|
| **Leader Election Node** | 리더 장애 시 자동 재선출 | Raft Consensus |
| **Formation Manager Node** | 정찰 대형 생성·유지·전환 | Line / Wedge / Grid Formation |
| **Task Allocator Node** | 클러스터 내 드론별 구역 배분 | Auction-based (CBBA) |
| **Topology Manager Node** | 통신 링크 품질 감시·재구성 | OLSR + Link Quality Monitoring |
| **GPS Integrity Monitor** | GPS 신호 이상 감지·보정 | RAIM |
| **Relay Node** | 통신 음영지역 중계 | Store-and-Forward |

---

### 🟢 Layer 3 — Individual UAV Layer

> 모든 드론 개별 탑재 노드

| 노드 | 기능 | 알고리즘 / 스택 |
|------|------|----------------|
| **Local Path Planner** | 개별 경로 실시간 계획 | RRT* / DWA |
| **Collision Avoidance Node** | 드론 간 충돌 방지 | ORCA / RVO2 |
| **Localization Node** | 위치 추정 | GPS + IMU EKF (PX4 EKF2) |
| **ISR Sensor Node** | 카메라·IR 센서 제어 | GStreamer + V4L2 |
| **Health Reporter Node** | 배터리·통신·모터 상태 전송 | Heartbeat (50ms 주기) |
| **Return-to-Base Node** | 이상 시 자동 복귀 | Failsafe + RTL |

---

## 3. 정찰 특화 핵심 노드 상세

### 📡 Area Decomposition Node (구역 분할)

```
전체 정찰 구역
      ↓
Voronoi Partition → 클러스터 수만큼 분할
      ↓
Boustrophedon(갈지자) 경로 → 각 드론 할당
      ↓
Coverage Path 생성 완료
```

- 중복 커버리지 최소화
- 드론 이탈·장애 시 잔여 구역 재배분
- 우선순위 구역(고가치 목표) 가중치 설정 가능

---

### 🛡️ GPS Integrity Monitor Node

```
GPS Raw Signal
      ↓
RAIM 알고리즘 → 위성 신호 이상 감지
      ↓
이상 감지 시 → IMU Dead Reckoning 전환
      ↓
인접 드론 위치 공유 → 상대 위치 보정 (Cooperative Positioning)
```

- 실외 정찰 환경에서 GPS 스푸핑·재밍 대응
- 드론 간 상대 거리 공유로 위치 정확도 보완

---

### 🤝 Task Allocator Node (CBBA 기반)

```
임무 목록 (정찰 포인트 N개)
      ↓
CBBA (Consensus-Based Bundle Algorithm)
      ↓
각 드론이 자신의 bid 계산 (거리, 배터리, 센서 상태)
      ↓
Consensus → 중복 없이 최적 배분
      ↓
배터리 임계치 도달 시 → 자동 재배분
```

---

## 4. 통신 스택 구성

```
┌──────────────────────────────────────────┐
│  Application Layer                       │
│  ROS2 Topics / Services / Actions        │
│  (rclcpp / rclpy 노드 인터페이스)          │
├──────────────────────────────────────────┤
│  Middleware Layer  ★ 변경                 │
│  ROS2 + CycloneDDS                       │
│  - RTPS 기반 Pub/Sub                     │
│  - 멀티캐스트 자동 노드 탐색 (mDNS)         │
│  - QoS 프로파일: BestEffort / Reliable    │
├──────────────────────────────────────────┤
│  Transport Layer                         │
│  MAVLink v2 (드론 제어 명령)               │
├──────────────────────────────────────────┤
│  Physical Layer                          │
│  900MHz (장거리) + WiFi 6 (근거리)         │
└──────────────────────────────────────────┘
```

### CycloneDDS 적용 상세

#### 변경 전 / 후 비교

| 항목 | Zenoh (이전) | ROS2 + CycloneDDS (변경) |
|------|-------------|--------------------------|
| **표준** | 독자 프로토콜 | OMG DDS 표준 (RTPS) |
| **ROS2 통합** | 별도 브릿지 필요 | 네이티브 rmw 지원 |
| **노드 탐색** | 수동 Peer 설정 | mDNS 자동 탐색 |
| **QoS 제어** | 제한적 | Deadline / Liveliness / History 등 풍부 |
| **멀티캐스트** | 제한적 | 기본 지원 (동일 서브넷) |
| **레이턴시** | 매우 낮음 | 낮음 (≤ 5ms, 근거리 WiFi 기준) |
| **생태계** | 신흥 | ROS2 공식 지원 DDS |

#### QoS 프로파일 적용 가이드

| 토픽 유형 | QoS Reliability | QoS Durability | 비고 |
|-----------|----------------|----------------|------|
| `/swarm/position` | BestEffort | Volatile | 고빈도 위치 (20Hz) |
| `/swarm/formation_cmd` | Reliable | Transient Local | 대형 명령 유실 방지 |
| `/swarm/health_status` | Reliable | Volatile | 상태 보고 50ms |
| `/swarm/task_alloc` | Reliable | Transient Local | 임무 배분 보장 |
| `/swarm/collision_warn` | BestEffort | Volatile | 충돌회피 저지연 우선 |

#### 환경 변수 설정

```bash
# CycloneDDS를 ROS2 기본 DDS로 설정
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 도메인 ID 분리 (클러스터별 격리 가능)
export ROS_DOMAIN_ID=42

# CycloneDDS 설정 파일 지정
export CYCLONEDDS_URI=file:///etc/cyclonedds/swarm_config.xml
```

#### CycloneDDS 설정 파일 예시 (`swarm_config.xml`)

```xml
<CycloneDDS>
  <Domain>
    <General>
      <!-- 실외 WiFi Mesh 인터페이스 지정 -->
      <NetworkInterfaceAddress>wlan0</NetworkInterfaceAddress>
      <AllowMulticast>true</AllowMulticast>
      <MaxMessageSize>65500B</MaxMessageSize>
    </General>
    <Discovery>
      <!-- 클러스터 내 멀티캐스트 자동 탐색 -->
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>50</MaxAutoParticipantIndex>
    </Discovery>
    <Internal>
      <!-- 저지연 튜닝 -->
      <MinimumSocketReceiveBufferSize>10MB</MinimumSocketReceiveBufferSize>
      <Watermarks>
        <WhcHigh>500kB</WhcHigh>
      </Watermarks>
    </Internal>
  </Domain>
</CycloneDDS>
```

#### 클러스터 간 DDS Domain 격리 전략

```
Cluster A → ROS_DOMAIN_ID=10  (리더 + D1~D3)
Cluster B → ROS_DOMAIN_ID=20  (리더 + D4~D6)
Cluster C → ROS_DOMAIN_ID=30  (리더 + D7~D9)
GCS       → ROS_DOMAIN_ID=0   (전체 브릿지)
                ↓
     [Domain Bridge Node]
     domain_bridge 패키지로
     클러스터 간 토픽 중계
```

| 링크 | 용도 | 주파수 |
|------|------|--------|
| GCS ↔ Leader | 임무 명령, 상태 보고 | 900MHz |
| Leader ↔ Member | 대형 제어, 로컬 조율 | WiFi 6 / 5.8GHz |
| 드론 ↔ 드론 | 충돌회피, 위치 공유 | 2.4GHz Mesh |

---

## 5. Failsafe 시나리오별 대응

| 상황 | 대응 노드 | 동작 |
|------|-----------|------|
| 리더 드론 장애 | Leader Election Node | Raft로 새 리더 선출 (< 2초) |
| GPS 신호 이상 | GPS Integrity Monitor | IMU DR + Cooperative Positioning 전환 |
| 드론 배터리 부족 | Health Reporter → Task Allocator | 구역 재배분 후 RTL |
| 통신 두절 | Relay Node → Return-to-Base | 30초 무응답 시 자동 복귀 |
| 클러스터 분리 | Topology Manager | 링크 재구성 또는 클러스터 병합 |

---

## 6. 최종 권장 노드 조합 요약

```
✅ 실외 정찰 / GPS유 / 분산 계층형 최적 구성

GCS Layer
  └─ Mission Planner Node
  └─ Area Decomposition Node
  └─ ISR Fusion Node
  └─ Replay & Log Node

Cluster Layer (리더 드론)
  └─ Leader Election Node      (Raft Consensus)
  └─ Formation Manager Node    (Wedge / Grid)
  └─ Task Allocator Node       (CBBA)
  └─ Topology Manager Node     (OLSR)
  └─ GPS Integrity Monitor     (RAIM)
  └─ Relay Node                (Store-and-Forward)

UAV Layer (전 드론)
  └─ Local Path Planner        (RRT*)
  └─ Collision Avoidance Node  (ORCA / RVO2)
  └─ Localization Node         (GPS + IMU EKF2)
  └─ ISR Sensor Node           (GStreamer + V4L2)
  └─ Health Reporter Node      (Heartbeat 50ms)
  └─ Return-to-Base Node       (Failsafe RTL)
```

---

## 7. 핵심 설계 원칙

| 원칙 | 설명 |
|------|------|
| **Fault Tolerance** | 리더 노드 이중화, Raft 기반 재선출로 단일 장애점 제거 |
| **Latency Budget** | 조율 주기 ≤ 50ms 목표, 충돌회피 루프 ≤ 20ms |
| **Scalability** | 클러스터 단위 수평 확장, 드론 추가 시 CBBA 자동 재배분 |
| **Observability** | 전 노드 상태를 GCS에서 실시간 시각화 (InfluxDB + Grafana) |
| **GPS Resilience** | RAIM + IMU DR + Cooperative Positioning 3중 보완 |

---

*작성일: 2026-06-17*
*운용 조건: 실외 / 정찰 / GPS 유 / 분산 계층형*
*v1.1 변경: Middleware Zenoh → ROS2 + CycloneDDS 적용*
