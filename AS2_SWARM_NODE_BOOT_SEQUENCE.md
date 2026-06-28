# 군집 정찰 시스템 — 노드 부트 시퀀스

> **범위**: `ros2 launch` 실행 → `BootstrapAll` 첫 BT tick 시작까지의 노드별 초기화 순서.  
> **기반**: `AS2_SWARM_MODULE_DECOMPOSITION.md` (M01~M26), `swarm_bt/bootstrap/bootstrap_subtrees.xml`.  
> **관점**: 노드 프로세스 기동 순서 → ROS2 인터페이스 등록 → 리더 활성화 → BT 엔진 시작.

---

## 부트 단계 정의

```
Phase 0 │ Launch   ros2 launch → 전 노드 프로세스 OS 스폰 (병렬)
Phase 1 │ Init     rclcpp::Node 생성자 → 파라미터 선언·로드 → 인터페이스 등록
Phase 2 │ Ready    첫 timer·heartbeat 발행 → 생존맵 구성 → election 수렴
Phase 3 │ Activate /swarm/leader_id 확정 → 리더 전용 노드 활성화 (M05·M07~M09·M11·M12)
Phase 4 │ BT Start BT 엔진 기동 → 플러그인 로드 → XML 파싱 → 첫 tick (AwaitBoot)
```

---

## 시퀀스 1 — Phase 0·1: Launch → 노드 생성자 완료

> 전 노드는 **비동기 병렬** 기동. 아래 순서는 런치 파일 내 선언 순서 기준이며 실제 OS 스케줄 의존.

```mermaid
sequenceDiagram
  participant LAUNCH as ros2 launch (M26)
  participant AS2 as AS2 기반 노드
  participant HM as health_monitor (M02)
  participant HB as heartbeat (M03)
  participant EL as election (M04)
  participant REG as registry (M05)
  participant AGG as aggregator (M08)
  participant ALLOC as allocator (M09)
  participant CEN as centroid_driver (M07)
  participant DFUSE as detection_fusion (M11)
  participant DETECT as detect_objects (M13~M15)
  participant BT as BT executor

  LAUNCH->>AS2: spawn process (platform·state_estimator·controller·behaviors)
  LAUNCH->>HM: spawn process (per-drone)
  LAUNCH->>HB: spawn process (per-drone)
  LAUNCH->>EL: spawn process (per-drone)
  LAUNCH->>REG: spawn process (per-drone, leader만 활성)
  LAUNCH->>AGG: spawn process (per-drone, leader만 활성)
  LAUNCH->>ALLOC: spawn process (per-drone, leader만 활성)
  LAUNCH->>CEN: spawn process (per-drone, leader만 활성)
  LAUNCH->>DFUSE: spawn process (per-drone, leader만 활성)
  LAUNCH->>DETECT: spawn process (per-drone)
  LAUNCH->>BT: spawn process (per-drone)

  Note over AS2,BT: rclcpp::init(argc, argv) — 전 프로세스 병렬

  AS2->>AS2: Node 생성자 완료 (platform 드라이버·FCU 연결 시도)
  HM->>HM: Node 생성자 완료 (파라미터 로드: timeouts·thresholds)
  HB->>HB: Node 생성자 완료 (파라미터: T_dead·publish_rate)
  EL->>EL: Node 생성자 완료 (파라미터: 없음)
  REG->>REG: Node 생성자 완료 (파라미터: N_min·mission_version)
  AGG->>AGG: Node 생성자 완료 (파라미터: T_barrier·altitudes·thresholds)
  ALLOC->>ALLOC: Node 생성자 완료 (파라미터: N_drones·recon_area)
  CEN->>CEN: Node 생성자 완료 (파라미터: centroid_rate=50Hz)
  DFUSE->>DFUSE: Node 생성자 완료 (파라미터: dedup_dist·T_track)
  DETECT->>DETECT: Node 생성자 완료 (파라미터: model_path·min_score)
  BT->>BT: Node 생성자 완료 (파라미터: bt_rate·xml_path·blackboard_init)
```

---

## 시퀀스 2 — Phase 1: 노드별 ROS2 인터페이스 등록

> 생성자 내에서 pub/sub/srv/action을 등록. 등록 완료 시점이 "인터페이스 Ready".

### 2-a. health_monitor (M02)

```mermaid
sequenceDiagram
  participant HM as health_monitor (M02)
  participant AS2 as AS2 platform/sensors
  participant TF as TF2 Buffer

  HM->>HM: create_subscription platform/info
  HM->>HM: create_subscription self_localization/pose
  HM->>HM: create_subscription battery_state
  HM->>TF: tf2_ros::Buffer + TransformListener 생성
  HM->>HM: create_wall_timer (check_period) — 폴링 시작
  Note over HM: state = CHECKING
  HM->>HM: [timer 최초 발화] FCU·센서·TF 응답 여부 확인
  alt FCU 연결됨 + 센서 응답 + TF 유효
    HM->>HM: health = OK
  else 일부 미응답
    HM->>HM: health = DEGRADED (재시도 카운트 시작)
  else 임계 초과 실패
    HM->>HM: health = FAIL (SwarmJoin 거부 대상)
  end
  Note over HM: 인터페이스 Ready
```

### 2-b. heartbeat (M03)

```mermaid
sequenceDiagram
  participant HM as health_monitor (M02)
  participant HB as heartbeat (M03)

  HB->>HB: create_publisher /swarm/heartbeat
  HB->>HB: create_subscription /swarm/heartbeat (전 드론 수신)
  HB->>HB: create_wall_timer (10 Hz) — 자기 발행
  Note over HB: 생존맵 map<drone_id, Heartbeat> 초기화 (빈 맵)
  HM-->>HB: health enum 콜백 등록 (내부 API)
  Note over HB: 인터페이스 Ready
  HB-)HB: 첫 /swarm/heartbeat 발행 (health=CHECKING)
  HB->>HB: 타 드론 heartbeat 수신 → 생존맵 갱신 시작
```

### 2-c. election (M04)

```mermaid
sequenceDiagram
  participant HB as heartbeat (M03)
  participant EL as election (M04)

  EL->>EL: create_publisher /swarm/leader_id
  EL->>HB: 생존맵 변경 콜백 등록 (내부 API)
  Note over EL: 인터페이스 Ready
  Note over EL: 생존맵 변경 이벤트 대기

  HB->>HB: 생존맵 갱신 (타 드론 heartbeat 수신)
  HB-->>EL: 생존맵 변경 통지

  EL->>EL: alive_ids = 생존맵 키 목록
  EL->>EL: leader_id = min(alive_ids)
  EL-)EL: /swarm/leader_id 발행 (std_msgs/String)
  Note over EL: 이후 생존맵 변화마다 재계산
```

### 2-d. registry (M05) — 리더 전용 노드 등록

```mermaid
sequenceDiagram
  participant EL as election (M04)
  participant HB as heartbeat (M03)
  participant REG as registry (M05)

  REG->>REG: create_service swarm/join (서버 등록)
  REG->>REG: create_publisher /swarm/registry
  REG->>REG: create_subscription /swarm/leader_id
  REG->>HB: 생존맵 변경 콜백 등록 (DEAD 드론 제거용)
  Note over REG: 인터페이스 Ready — 상태: STANDBY
  Note over REG: swarm/join 요청 도착해도 STANDBY 중 거부

  EL-)REG: /swarm/leader_id = "drone_0"
  REG->>REG: leader_id == self_ns? → YES
  Note over REG: 상태: ACTIVE — SwarmJoin 요청 수락 시작
```

### 2-e. 리더 전용 노드 공통 패턴 (M07·M08·M09·M11·M12)

```mermaid
sequenceDiagram
  participant EL as election (M04)
  participant N as 리더 전용 노드

  N->>N: create_subscription /swarm/leader_id
  N->>N: [기타 pub/sub/srv/action 등록]
  Note over N: 인터페이스 Ready — 상태: STANDBY

  EL-)N: /swarm/leader_id 수신
  N->>N: leader_id == self_ns?

  alt 리더인 경우
    N->>N: 상태: ACTIVE
    N->>N: 집계·계획·TF broadcast 등 기능 시작
  else 팔로워인 경우
    N->>N: 상태: STANDBY 유지 (구독만)
  end
  Note over N: 이후 leader_id 변경 시 재평가 (리더 승계 지원)
```

---

## 시퀀스 3 — Phase 1: BT 엔진 기동

```mermaid
sequenceDiagram
  participant LAUNCH as ros2 launch (M26)
  participant BT as BT executor
  participant PLG as BT Plugin Libraries
  participant XML as XML 파일 (M22·M23)
  participant BB as Blackboard

  BT->>BT: rclcpp::Node 생성자 완료
  BT->>BT: 파라미터 로드 (bt_rate, xml_path, blackboard_init YAML)

  Note over BT,PLG: Phase 1-BT-1: 플러그인 라이브러리 로드
  BT->>PLG: dlopen aiss_swarm_bt_plugins.so
  PLG->>PLG: M17 조율 노드 7종 등록 (factory)
  PLG->>PLG: M18 barrier 노드 4종 등록
  PLG->>PLG: M19 게이트/슬롯 노드 등록
  PLG->>PLG: M20 P3 임무 노드 12종 등록
  PLG->>PLG: M21 부트스트랩 노드 7종 등록
  PLG-->>BT: 플러그인 팩토리 등록 완료

  Note over BT,XML: Phase 1-BT-2: XML 로드·파싱
  BT->>XML: load swarm_root.xml
  XML->>XML: include bootstrap_subtrees.xml
  XML->>XML: include swarm_mission_root.xml
  XML->>XML: include swarm_perdrone_root.xml
  XML->>XML: include chassis/chassis_drone.xml
  XML->>XML: include nodes_model.xml
  XML-->>BT: TreeNodesModel 파싱 완료

  Note over BT,BB: Phase 1-BT-3: 블랙보드 초기화
  BT->>BB: 생성 (공유 블랙보드)
  BT->>BB: recon_area = {폴리곤} (launch 파라미터)
  BT->>BB: home = {x, y, z}
  BT->>BB: target_classes = [list]
  BT->>BB: formation = "line"
  BT->>BB: my_slot = "" (SwarmJoin 후 갱신)

  Note over BT: Phase 1-BT-4: BT 인스턴스 생성
  BT->>BT: factory.createTree("SwarmRoot", BB)
  BT->>BT: SwarmRoot 트리 인스턴스 생성 완료

  Note over BT: Phase 1-BT-5: tick 타이머 기동
  BT->>BT: create_wall_timer (bt_rate = 50Hz)
  Note over BT: BT Ready — 첫 tick 대기
```

---

## 시퀀스 4 — Phase 2·3: 첫 BT tick → BootstrapAll 진입

> 전 노드 초기화 완료 직후 BT가 첫 tick. BootstrapAll은 내부 재시도 루프로 노드 준비 지연을 흡수.

```mermaid
sequenceDiagram
  participant BT as BT executor
  participant HM as health_monitor (M02)
  participant HB as heartbeat (M03)
  participant EL as election (M04)
  participant REG as registry (M05)
  participant ALLOC as allocator (M09)
  participant GCS as GCS (M24)

  Note over BT: 첫 tick — SwarmRoot 진입
  Note over BT: Parallel: KeepRunning(PublishHeartbeat) + BootstrapAll

  BT-)HB: PublishHeartbeat.tick() → /swarm/heartbeat 발행 시작

  Note over BT: BootstrapAll → SequenceStar 시작

  loop AwaitBoot — health OK/DEGRADED 대기
    BT->>HM: AwaitBoot.tick() — health 상태 폴링
    alt health = CHECKING / 초기화 중
      HM-->>BT: RUNNING (아직 미결정)
    else health = OK or DEGRADED
      HM-->>BT: SUCCESS
    else health = FAIL
      HM-->>BT: RUNNING (무한 대기 — 운용자 개입 필요)
    end
  end
  Note over BT: AwaitBoot SUCCESS

  BT->>HB: ReportHealth.tick() → health 반영
  HB-)HB: /swarm/heartbeat 갱신 (health=OK 포함)
  Note over BT: ReportHealth SUCCESS

  Note over BT: SwarmJoin — 리더 registry 서비스 호출

  loop 서비스 대기 (registry 미활성 시)
    BT->>REG: swarm/join.srv 요청
    alt registry STANDBY (leader 미확정)
      REG-->>BT: 서비스 없음 / RUNNING
    else registry ACTIVE
      REG-->>BT: {accepted=true, registry[], mission_version}
    end
  end
  Note over BT: SwarmJoin SUCCESS → my_slot 블랙보드 저장

  loop QuorumReady — 정족수 대기
    BT->>REG: /swarm/registry 구독 → 등록 수 확인
    alt N < N_min
      REG-->>BT: RUNNING
    else N >= N_min
      REG-->>BT: SUCCESS (QuorumReady = true)
    end
  end
  Note over BT: QuorumReady SUCCESS

  GCS-)ALLOC: /swarm/mission_intent 발행
  ALLOC->>ALLOC: 슬롯 배정 → /{ns}/swarm_agent/task 발행

  loop MissionReady — MissionIntent + BatteryOK 대기
    BT->>BT: MissionReady.tick()
    alt MissionIntent 미수신
      BT-->>BT: RUNNING
    else MissionIntent 수신 + BatteryOK
      BT-->>BT: SUCCESS (★2 게이트)
    end
  end
  Note over BT: MissionReady SUCCESS → BootstrapAll SUCCESS

  Note over BT: IsLeader 자기선택 → SwarmCoord / SwarmDrone 진입
```

---

## 노드별 초기화 상세

### 인터페이스 등록 목록

| 노드 | Publishers | Subscribers | Srv Server | Srv Client | Action Server | Action Client |
|------|-----------|-------------|------------|------------|---------------|---------------|
| **M02** health_monitor | — | platform/info, pose, battery | — | — | — | — |
| **M03** heartbeat | /swarm/heartbeat | /swarm/heartbeat | — | — | — | — |
| **M04** election | /swarm/leader_id | (M03 생존맵 API) | — | — | — | — |
| **M05** registry | /swarm/registry | /swarm/leader_id | swarm/join | — | — | — |
| **M07** centroid_driver | — | /swarm/leader_id, DriveCentroid 트리거 | — | — | — | — |
| **M08** aggregator | /swarm/barrier | /swarm/leader_id, platform/info×N, pose×N, /swarm/heartbeat, /swarm/registry | — | — | — | — |
| **M09** allocator | /{ns}/swarm_agent/task | /swarm/leader_id, /swarm/mission_intent, /swarm/registry, /swarm/targets, /swarm/emitters | — | — | — | — |
| **M11** detection_fusion | /swarm/targets | /swarm/leader_id, /swarm/registry, /{ns}/perception/detections×N | — | — | — | — |
| **M12** emitter_fusion | /swarm/emitters | /swarm/leader_id, /{ns}/rf/bearings×N | — | — | — | — |
| **M13~M15** detect_objects | /{ns}/perception/detections | eo/image_raw, ir/image_raw, camera_info | — | — | DetectObjectsBehavior | — |
| **M16** rf_survey | /{ns}/rf/bearings | /{ns}/self_localization/pose | — | — | RfSurveyBehavior | — |
| **BT executor** | — | /swarm/leader_id, /swarm/mission_phase, /swarm/barrier, /swarm/registry | — | swarm/join | — | Takeoff, Land, GoTo, FollowPath, SwarmFlocking, DetectObjects |

### 노드 상태 전이

```
health_monitor (M02):
  INIT → CHECKING → OK / DEGRADED / FAIL

heartbeat (M03):
  INIT → READY → BROADCASTING (첫 /swarm/heartbeat 발행 이후)

election (M04):
  INIT → READY → MONITORING → LEADER_PUBLISHED

registry (M05):
  INIT → READY → STANDBY → ACTIVE (IsLeader 확인 후)

aggregator (M08):
  INIT → READY → STANDBY → ACTIVE (IsLeader 확인 후)

allocator (M09):
  INIT → READY → STANDBY → ACTIVE (IsLeader 확인 후)

centroid_driver (M07):
  INIT → READY → STANDBY → ACTIVE (IsLeader 확인 후) → DRIVING (DriveCentroid 호출 후)

detection_fusion (M11):
  INIT → READY → STANDBY → ACTIVE (IsLeader 확인 후) → FUSING (detections 수신 중)

detect_objects behavior (M13~M15):
  INIT → MODEL_LOAD → IDLE → RUNNING (action goal 수신 후)

BT executor:
  INIT → PLUGINS_LOADED → XML_PARSED → BB_INIT → TICKING
```

---

## Phase별 완료 기준 (노드 Ready 게이트)

```
Phase 0 완료 ─── 전 노드 프로세스 OS 스폰 완료
                  판단: pgrep / /proc 기반 (launch 내부 상태)

Phase 1 완료 ─── 전 노드 ROS2 인터페이스 등록 완료
                  판단: ros2 node list에 전 노드 출현 + ros2 topic list 확인 가능
                  키 노드: health_monitor(health timer 기동), heartbeat(첫 발행),
                           BT executor(플러그인 로드·XML 파싱 완료)

Phase 2 완료 ─── election 수렴 → /swarm/leader_id 발행
                  판단: `ros2 topic echo /swarm/leader_id` 안정
                  소요 시간: T_dead 내 생존맵 안정화 (통상 1~2초)

Phase 3 완료 ─── 리더 전용 노드 ACTIVE
                  판단: registry→ swarm/join 서비스 활성, aggregator→ /swarm/barrier 발행 시작
                  소요 시간: Phase 2 완료 후 수십 ms

Phase 4 완료 ─── BT BootstrapAll SUCCESS
                  판단: /swarm/mission_phase = TAKEOFF 발행
                  소요 시간: 전 드론 SwarmJoin 완료 + GCS MissionIntent 수신 후
```

---

## 부트 타임라인 (N=3, Sim 기준)

```
T+0.0s  ros2 launch 실행
T+0.1s  전 노드 프로세스 스폰 (OS 병렬)
T+0.3s  AS2 platform/state_estimator 초기화 완료 (FCU 연결)
T+0.5s  health_monitor health=OK (FCU + 센서 응답 확인)
T+0.5s  heartbeat 첫 발행 → 생존맵 구성 시작
T+0.7s  BT executor: 플러그인 로드 + XML 파싱 완료 → 첫 tick
T+0.7s  AwaitBoot tick 시작 (health 폴링)
T+0.8s  AwaitBoot SUCCESS (health=OK)
T+1.0s  election: 전 드론 heartbeat 수렴 → leader_id 확정
T+1.0s  registry: ACTIVE → swarm/join 서비스 오픈
T+1.2s  전 드론 SwarmJoin 완료 → /swarm/registry (3기 등재)
T+1.2s  QuorumReady = true
T+2.0s  GCS: MissionIntent 발행
T+2.0s  allocator: 슬롯 배정 → swarm_agent/task 발행
T+2.1s  MissionReady SUCCESS → BootstrapAll SUCCESS
T+2.1s  ★2 게이트 통과 → /swarm/mission_phase = TAKEOFF
```

---

## 부트 실패 케이스

| 실패 상황 | 증상 | 원인 | 대응 |
|-----------|------|------|------|
| **FCU 연결 실패** | AwaitBoot RUNNING 무한 대기 | health=FAIL | FCU 연결 확인 후 노드 재기동 |
| **플러그인 로드 실패** | BT executor 크래시 | .so 누락 / 심볼 오류 | colcon build 재실행, ROS_PLUGIN_PATH 확인 |
| **XML 파싱 실패** | BT executor 예외 종료 | XML 문법 오류 / include 경로 오류 | nodes_model.xml ID 확인, 경로 수정 |
| **리더 미선출** | SwarmJoin RUNNING 무한 대기 | 드론 수 < 2 or heartbeat 미수신 | 네트워크 확인, N_min 파라미터 확인 |
| **swarm/join 서비스 없음** | BT SwarmJoin RUNNING 대기 | registry STANDBY (leader 미확정) | election 정상 동작 확인 |
| **GCS MissionIntent 미발행** | MissionReady RUNNING 대기 | GCS 미기동 / YAML 오류 | `ros2 topic pub /swarm/mission_intent` 확인 |
| **QuorumReady 미충족** | 무한 대기 | N < N_min | N_min 파라미터 조정 또는 추가 드론 기동 |
| **ONNX/TRT 모델 로드 실패** | detect_objects 크래시 | model_path 오류 / GPU 메모리 부족 | model_path 파라미터 확인, GPU 자원 확인 |

---

## 노드 의존성 그래프 (부트 순서 임계 경로)

```
ros2 launch
    │
    ├─ AS2 platform ──────────────────────────────► health_monitor (FCU 연결 필요)
    │                                                    │
    ├─ health_monitor ────────────────────────────────── ▼
    │                                              heartbeat (health 콜백)
    │                                                    │ /swarm/heartbeat
    ├─ election ◄─────────────────────────────────────── ┘
    │      │ /swarm/leader_id
    │      ▼
    ├─ registry (STANDBY → ACTIVE)
    │      │ swarm/join.srv
    │      ▼
    │  [BT: SwarmJoin] ─────────────────────────► [BT: QuorumReady]
    │                                                    │
    ├─ GCS (MissionIntent) ──────────────────────────── ▼
    │      │ /swarm/mission_intent                 [BT: MissionReady ★2]
    │      ▼                                             │
    └─ allocator (STANDBY → ACTIVE)                      ▼
           │ /swarm_agent/task              BootstrapAll SUCCESS → P1 진입
           ▼
     BT: /{ns}/swarm_agent/task 수신

★ 임계 경로: AS2_platform → HM → HB → EL → REG → SwarmJoin → QuorumReady → MissionReady
★ 병렬 트랙: BT 엔진 기동(플러그인·XML) → 첫 tick (AwaitBoot 폴링으로 HM 완료 흡수)
```

---

## BT 플러그인 로드 순서

```
BT executor 기동 시 dlopen 순서 (aiss_swarm_bt CMakeLists 등록 순):

1. M21 BootstrapAll 플러그인
   AwaitBoot · ReportHealth · PublishHeartbeat · SwarmJoin
   IsLeader · QuorumReady · MissionReady

2. M17 조율 플러그인
   HasMissionType · LatchMissionType · PublishPhase
   SwarmFlockingStart · SwarmFlockingStop · ModifyFormation · DriveCentroid

3. M18 barrier 플러그인
   AwaitAllAirborne · AwaitFormed · AwaitAllZonesDone · AwaitAllLanded

4. M19 게이트/슬롯 플러그인
   PhaseIs · WaitForAlert · WaitTakeoffSlot · WaitLandSlot
   SafeHover · AwaitBatteryLow

5. M20 P3 임무 플러그인
   DetectObjects · ObjectDetected · InspectTimeout · RfSurvey
   EmitterDetected · HasRole · TargetLost · PartitionAndAssign
   AssignRfSurvey · FuseDetections · FuseEmitters · AllocateTrackRoles

로드 실패 시: BT executor 예외 종료 (전 플러그인 성공 전제)
```

---

## 부트 완료 확인 명령 (Sim 검증용)

```bash
# Phase 1 확인: 전 노드 등록
ros2 node list | grep -E "health_monitor|heartbeat|election|registry"

# Phase 2 확인: leader_id 발행
ros2 topic echo /swarm/leader_id --once

# Phase 3 확인: registry 활성 (swarm/join 서비스 존재)
ros2 service list | grep swarm/join

# Phase 3 확인: barrier 발행 시작
ros2 topic echo /swarm/barrier --once

# Phase 4 확인: BootstrapAll SUCCESS → mission_phase=TAKEOFF
ros2 topic echo /swarm/mission_phase --once
```

---

_근거: `AS2_SWARM_MODULE_DECOMPOSITION.md`, `swarm_bt/bootstrap/bootstrap_subtrees.xml`, `swarm_bt/swarm_bt_combined.xml`._
