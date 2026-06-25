# 군집 부트스트랩 설계 — Phase 0 (부팅·합류·선출·임무개시)

> **범위**: BT 임무 실행(`AS2_SWARM_BT_DISPATCH_DESIGN.md`의 P1~P5) **이전**의 준비 과정 — GCS 임무 생성/업로드, 드론 부팅, heartbeat, health check, 리더 선출, 군집 참여.
> **근거**: AS2 실코드(`as2_fleet_manager`, `drone_boot_nodes.md`의 부팅 시퀀스, `as2_behaviors_swarm_flocking`) + 신규 aiss 조율 노드.
> **핵심**: 실코드는 **GCS측 `fleet_manager`가 ROS 그래프 discovery로 드론을 찾고 중앙에서 SwarmFlocking을 트리거**한다(heartbeat/election/join 없음). 본 설계는 여기에 **자기조직(heartbeat/health/election/join) 계층을 추가**해, GCS 단절에도 군집이 자율로 리더를 세우고 임무를 개시하게 한다. 선출된 리더가 coordinator BT(`SwarmCoord`)를 실행한다.

---

## 0. 실코드 vs 신규

| 기능 | 실코드 | 신규(aiss) |
|---|---|---|
| 드론 부팅(platform→estimator→controller→behaviors) | ✅ `drone_boot_nodes.md` Phase1~3 | — |
| 드론 discovery·status 모니터 | ✅ `fleet_manager`(ROS그래프 스캔, fleet_status) | — |
| SwarmFlocking 트리거 | ✅ `fleet_manager ~/start_swarm` 서비스 | → **리더 BT로 이관** |
| GCS 임무 생성/업로드 | ◻ 부분(fleet iface) | mission intent 업로드 |
| heartbeat / health | ❌ | **신규** |
| 리더 선출 | ❌ | **신규** |
| 군집 참여(join)·등록부·정족수 | ❌ (discovery로 대체) | **신규** |

> 즉 `fleet_manager`는 **GCS 모니터 + 임무 iface**로 남고, "중앙에서 SwarmFlocking 트리거"하던 역할은 **선출 리더의 coordinator BT(`SwarmFlockingStart`)**가 대신한다 → GCS 단절 생존.

---

## 1. Phase 0 단계열

```
B0 드론 부팅      → AS2 스택(platform/estimator/controller/behaviors) + swarm_agent 노드 기동
B1 health check   → 자가진단(FCU링크/센서/estimator/배터리) → 준비도 판정
B2 heartbeat 발행 → /swarm/heartbeat (id, alive, health, ts) 상시
B3 군집 참여(join)→ registry 등록, 멤버십 구성
B4 리더 선출      → 살아있는 최저 id(or bully) → /swarm/leader_id
B5 정족수 확인    → 등록수 ≥ min_quorum → 임무 게이트 개방
B6 임무 업로드    → GCS → mission_intent 복제(전 드론 캐시)
B7 임무 개시      → 리더 coordinator BT가 P1(순차이륙)로 진입
```

B1~B5는 지상(disarmed)에서 상시 백그라운드. B6은 비동기(언제든). B5∧B6 충족 시 BT가 STANDBY→TAKEOFF.

---

## 2. 노드 구성 (드론 1대 + GCS)

```
┌─ GCS (off-board, 제어루프 밖) ──────────────────────────────┐
│  fleet_manager (실코드)  : discovery, fleet_status 모니터    │
│  gcs_mission_iface (신규): 임무 생성/업로드 → mission_intent  │
└───────────────────────────┬─────────────────────────────────┘
        mission_intent (best-effort, 끊겨도 임무 지속)
┌─ Per-drone (namespace) ─────────────────────────────────────┐
│  [AS2 스택]  platform → state_estimator → controller         │
│              + motion behaviors(standby) + bt_manager        │
│  [swarm_agent 신규]                                          │
│    health_monitor : 자가진단 → health                        │
│    heartbeat_node : /swarm/heartbeat 발행 + 타 드론 수신     │
│    election_node  : leader_id 산출                           │
│    registry_node  : join/등록부/quorum (리더에서 권위)        │
│    coordinator BT : SwarmCoord (리더만 활성)                  │
│    per-drone BT   : SwarmDrone (전 드론)                      │
└──────────────────────────────────────────────────────────────┘
```

- swarm_agent = BT 위·옆의 **백그라운드 서비스군**. 무거운 상태(등록부)·합의(선출)는 BT 밖, BT는 결과를 조건으로 소비.

---

## 3. 단계별 상세

### B0. 드론 부팅
- 순서(실코드 `drone_boot_nodes.md`): **platform → state_estimator → controller** (필수3) → motion behaviors(standby) → bt_manager + swarm_agent.
- 게이트: `platform/info.connected` ∧ `self_localization` 유효 → swarm_agent 기동 허용.
- 산출: AS2 스택 가동, BT는 STANDBY phase.

### B1. health check
- 입력: `platform/info`(connected/armed), 센서 토픽 생존, estimator TF 유효, 배터리 SOC, FCU 링크.
- 출력: `health ∈ {OK, DEGRADED, FAIL}` + 상세.
- 용도: join 자격, 이륙 자격, BatteryLow 등 안전 게이트 입력.

### B2. heartbeat 발행
- 토픽: `/swarm/heartbeat` (전역). 메시지: `{drone_id, alive, health, stamp, position?}`.
- 주기: 고정(예 2~5Hz). 타 드론 heartbeat 수신·타임아웃 감시 → 멤버 생존맵.
- 용도: 선출 입력(B4), 분리감시, 리더상실 감지(BT `WaitForLeaderLost`).

### B3. 군집 참여(join)·등록부
- 서비스: `swarm/join` (req: id+health → res: accepted, mission_version).
- registry: 등록 멤버 목록(`/swarm/registry`). 권위는 리더 보유(선출 후).
- 부트스트랩 순환 해소: 선출(B4)은 heartbeat만으로 가능 → 리더 결정 후 리더가 join 수락·registry 구성.
- BT 훅: per-drone `SwarmJoin`(재시도) — 등록 실패해도 트리 안 죽음.

### B4. 리더 선출
- 알고리즘: **살아있는 최저 drone_id**(단순·결정적) 또는 bully. 입력 = heartbeat 생존맵.
- 출력: `/swarm/leader_id`. 리더 사망(heartbeat 타임아웃) → 차순위 자동 승계(SPOF 제거).
- BT 훅: `IsLeader` 조건 → 참이면 `SwarmCoord`(coordinator) 활성. 매tick 재평가 → 리더 교체 시 즉시 이양.

### B5. 정족수(quorum)
- 조건: `registry 크기 ≥ min_quorum`.
- BT 훅: `QuorumReady` 조건 → 임무 개시 게이트(B7). 미달 시 대기/지상 standby.

### B6. GCS 임무 생성·업로드
- gcs_mission_iface: 운용자가 임무 정의(타입 EOIR_RECON/RF_RECON/SURVEIL/TRACK + 구역 폴리곤 + 표적 클래스 + 편대/제약) → `mission_intent` 발행.
- coordinator(리더)가 수신 → 전 드론 복제(캐시) → 블랙보드 `{mtype}` 등 설정.
- best-effort: 업로드 후 GCS 끊겨도 캐시로 임무 지속.

### B7. 임무 개시
- 게이트: `IsLeader` ∧ `QuorumReady` ∧ `mission_intent 수신`.
- 동작: 리더 `SwarmCoord`가 STANDBY → `Cx_SequentialTakeoff`(P1)로 진입. per-drone는 `/swarm/mission_phase` 따라 동작.

---

## 4. 메시지·토픽 추가 (Phase 0)

| 토픽/서비스 | 타입(요지) | 방향 | 단계 |
|---|---|---|---|
| `/swarm/heartbeat` | `{id, alive, health, stamp}` | 드론↔드론(전역) | B2 |
| `/swarm/leader_id` | `uint16`/string | election → 전체 | B4 |
| `swarm/join` (srv) | req{id,health}/res{accepted,mission_version} | 드론 → 리더 | B3 |
| `/swarm/registry` | `string[] members` | 리더 → 전체 | B3 |
| `/swarm/mission_intent` | `{type, area, classes, formation, constraints}` | GCS → 리더 → 복제 | B6 |
| `health`(per-drone) | `{status, detail}` | health_monitor → 로컬/heartbeat | B1 |

> 기존 실코드: `fleet_manager ~/drone_namespaces`, `~/fleet_status`, `~/start_swarm` 재활용(모니터). `as2_msgs/PlatformInfo`는 health 입력.

---

## 5. BT 통합 (dispatch 설계 개정)

`AS2_SWARM_BT_DISPATCH_DESIGN.md`의 "고정 coordinator" → **선출 리더가 coordinator BT 실행**으로 개정. 루트에 Phase 0 게이트 추가.

### 5.1 통합 루트 (coordinator+per-drone 단일 배포, IsLeader 자기선택)
```
SwarmRoot (전 드론 동일 배포)
 Parallel
 ├─ WaitForAlert → Emergency                         (안전 선점)
 └─ SequenceStar
    ├─ AwaitBoot (platform connected ∧ estimator ok)  (B0)
    ├─ RetryUntilSuccessful: SwarmJoin                 (B3, 재시도)
    └─ Parallel(상시 ∥ 임무)
       ├─ KeepRunning: PublishHeartbeat               (B2)
       ├─ KeepRunning: ReportHealth                   (B1)
       └─ ReactiveFallback
          ├─ ReactiveSequence: IsLeader ∧ QuorumReady (B4,B5)
          │    → Parallel{ SwarmCoord(리더 조율) ∥ SwarmDrone }
          └─ SwarmDrone                                (팔로워: per-drone만)
```
- **단일 트리 배포**: `IsLeader`가 리더/팔로워 자기선택(기존 `swarm_mission_example.xml` 관용구). 리더 교체 시 `ReactiveFallback` 매tick 재평가 → 즉시 이양.
- `SwarmCoord`는 §dispatch의 임무타입 selector + chassis + P3Coord 그대로. 단 진입 게이트에 `QuorumReady` + `mission_intent 수신` 추가.
- `SwarmDrone`도 그대로 — 단 phase 방송 전(STANDBY)엔 `SafeHover`/지상 대기.

### 5.2 신규 BT 노드 (Phase 0)
| 노드 | 유형 | 역할 |
|---|---|---|
| `AwaitBoot` | Condition | platform connected ∧ estimator 유효 |
| `PublishHeartbeat` | Action | `/swarm/heartbeat` 상시 발행 |
| `ReportHealth` | Action | health 상시 발행 |
| `SwarmJoin` | Action | `swarm/join` 호출(재시도) |
| `IsLeader` | Condition | `leader_id == 자기 id` |
| `QuorumReady` | Condition | registry ≥ min_quorum |
| `MissionReady` | Condition | mission_intent 수신·캐시됨 |

---

## 6. 전체 타임라인 (한 드론)

```
부팅 → AS2스택 → swarm_agent
  │ health OK
  ├─ heartbeat 발행 시작 ─────────────────────────────(상시)
  │ 타 드론 heartbeat 수신 → 생존맵
  ├─ election: leader_id 산출 ── (최저 id=리더)
  ├─ join → registry 등록
  │ registry ≥ quorum ?  ──── 아니오 → 지상 standby(대기)
  │           예
  ├─ mission_intent 수신(GCS) ── 아니오 → standby
  │           예
  └─ [B7] 리더면 SwarmCoord 시작 → P1 순차이륙 → ... (P5)
      팔로워면 SwarmDrone이 phase 추종
※ 어느 시점이든: alert→Emergency 선점, 리더 heartbeat 끊김→재선출/자율생존
```

---

## 7. 부트스트랩 난제·완화

| # | 난제 | 완화 |
|---|---|---|
| C1 | 선출 vs join 순환(누가 먼저) | 선출은 heartbeat만으로 가능 → 리더 먼저, 리더가 join 수락 |
| C2 | 리더 교체 중 임무 공백 | `IsLeader` 매tick 재평가 + mission_intent 전 드론 캐시 → 즉시 이양 |
| C3 | split-brain(2 리더) | 결정적 규칙(최저 id) + heartbeat 정족수 확인 |
| C4 | GCS 단절 시 개시 불가 | mission_intent 캐시 + 리더 자율 개시(GCS는 best-effort) |
| C5 | 부팅 미완 드론 조기 합류 | `AwaitBoot` + health OK 게이트 |
| C6 | 정족수 미달 장기화 | 타임아웃 정책(축소 임무/지상 대기) |

---

## 8. 구현 로드맵 (Phase 0)

| M | 범위 | 산출 |
|---|---|---|
| M0a | swarm_agent 골격 + heartbeat/health | `/swarm/heartbeat` 발행·수신, health 판정 |
| M0b | election + registry/join + quorum | `/swarm/leader_id`, `swarm/join`, registry |
| M0c | gcs_mission_iface | mission_intent 업로드·복제 |
| M0d | Phase0 BT 노드(§5.2) + 루트 게이트 통합 | IsLeader/QuorumReady/MissionReady 게이트 |
| M0e | 통합: 부팅→선출→정족수→개시 Sim | N기 지상 standby→자율 개시 |

→ Phase 0(M0a~e) 완료 후 기존 P1~P5 BT(`AS2_SWARM_BT_DISPATCH_DESIGN.md`)에 연결.

---

## 9. 결론

- **실코드 기반**: 부팅(platform→estimator→controller) + `fleet_manager` discovery/monitor 재활용. SwarmFlocking 트리거는 **리더 BT로 이관**.
- **신규 = 자기조직 계층**: heartbeat/health/election/registry/join → GCS 단절 생존. swarm_agent 백그라운드 서비스 + Phase0 BT 게이트.
- **BT 개정**: 고정 coordinator → **선출 리더가 `SwarmCoord` 실행**(IsLeader 자기선택), 진입 게이트에 QuorumReady+MissionReady 추가.
- **개시 조건**: `AwaitBoot` → join → `IsLeader`/`QuorumReady`/`MissionReady` → P1 순차이륙.

---

_근거 소스: `as2_user_interfaces/as2_fleet_manager/`(fleet_manager_node.py, swarm_configurator.py), `drone_boot_nodes.md`, `as2_behaviors_swarm_flocking/`, `as2_msgs/action/SwarmFlocking.action`, `AS2_SWARM_BT_DISPATCH_DESIGN.md`._
