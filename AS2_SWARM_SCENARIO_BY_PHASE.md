# 군집 정찰 시스템 — 구간별 동작 시나리오

> **근거**: `AS2_SWARM_WBS.md`, `AS2_RECON_SWARM_PHASED_DESIGN.md`, `AS2_SWARM_SEQUENCE_DIAGRAM.md`, `AS2_SWARM_NODE_RESPONSIBILITY.md`.
> **범위**: P0 부트스트랩 ~ P5 착륙. 각 구간의 트리거·액터·동작순서·완료조건·인터페이스·리스크를 기술.
> **표기**: `(P)` = Publisher, `(S)` = Subscriber, `(AC)` = Action Client, `(AS)` = Action Server, `(SrvC/S)` = Service Client/Server.

---

## 구간 맵

```
P0 부트스트랩
  └─ 부팅 → heartbeat 안정 → 선출 → 합류·정족수 → 임무 하달 → 개시 게이트
     ↓ ★2 게이트 통과
P1 순차 이륙
  └─ 슬롯 게이트 → 드론별 Arm/Takeoff → 전기 airborne barrier
     ↓
P2 편대 transit
  └─ FORM_UP (편대 수렴) → TRANSIT (centroid 이동) → 정찰구역 진입점 도달
     ↓
P3 구역분할 독립정찰         ★ 핵심 임무 구간
  └─ DISBAND (편대 해제) → 구역배정 → 드론별 독립 FollowPath+탐지 → 전구역 완료
     ↓
P4 완료판정 · 복귀
  └─ REGROUP (재편대) → RETURN (home 복귀)
     ↓
P5 순차 착륙
  └─ 슬롯 게이트 → 드론별 Land → 전기 landed barrier → DONE
```

---

## P0. 부트스트랩 (Boot → 임무 개시)

**구간 목표**: 전원 인가부터 임무 수행 자격 획득까지. 수동 명령 없이 자율 개시.

### P0-1. 자가진단 (health_monitor)

| 항목 | 내용 |
|---|---|
| 트리거 | 드론 부팅, 노드 기동 |
| 액터 | `health_monitor` (드론별) |
| 동작 | FCU 연결 확인 → 센서 응답 확인(EO/IR/RF) → TF 프레임 확인 → 배터리 수준 폴링 |
| 출력 | `health` enum (OK / DEGRADED / FAIL) → `heartbeat`로 전달 |
| 실패 시 | FAIL → 해당 드론 `SwarmJoin` 요청 거부됨 |

### P0-2. heartbeat 발행 및 생존맵 구성

| 항목 | 내용 |
|---|---|
| 트리거 | health_monitor 초기화 완료 |
| 액터 | `heartbeat` 노드 (드론별, P/S) |
| 동작 순서 | ① 자기 `Heartbeat.msg` 발행(`/swarm/heartbeat`) → ② 타 드론 heartbeat 수신 → ③ 생존맵 갱신 |
| 생존맵 소비자 | `election`, `aggregator` |
| 인터페이스 | `/swarm/heartbeat` (P/S×N) |

### P0-3. 리더 선출 (election)

| 항목 | 내용 |
|---|---|
| 트리거 | 생존맵에 2기 이상 등재 |
| 액터 | `election` (드론별 독립 실행, 결과는 전역 합의) |
| 동작 | 생존맵 감시 → **최저 drone_id** 규칙으로 리더 산정 → `/swarm/leader_id` 발행 |
| 승계 | 현 리더 heartbeat 소실 → 차순위 id가 리더 승계 (SPOF 완화) |
| 효과 | `IsLeader` 게이트 활성 → `registry`, `allocator`, `centroid_driver`, `detection_fusion`, `emitter_fusion`, `aggregator` 리더 인스턴스 기동 |

### P0-4. 군집 합류·정족수 (registry)

| 항목 | 내용 |
|---|---|
| 트리거 | 리더 확정 + 각 드론 `BT:SwarmDrone` 부트스트랩 루트 진입 |
| 드론 측 | `SwarmJoin.srv` 요청 (req: `{drone_id, health}`) → 리더 registry로 전송 |
| 리더 측 | `SwarmJoin.srv` 수락 (res: `{accepted, registry[], mission_version}`) → 등록부 갱신 |
| 정족수 판정 | 등록부 ≥ N_min → `QuorumReady` 플래그 발행 |
| 인터페이스 | `swarm/join` (SrvC→SrvS), `/swarm/registry` (P) |

### P0-5. 임무 하달 (GCS → allocator)

| 항목 | 내용 |
|---|---|
| 트리거 | QuorumReady 확인 후 GCS 운용자가 임무 업로드 |
| 액터 | `gcs_mission_iface` (off-board) |
| 동작 | 임무 정의(타입, 구역 폴리곤, 표적 클래스, 편대 형태) → `MissionIntent.msg` 발행 |
| 수신 | `allocator` + `BT:SwarmCoord` 리더 BT |
| 인터페이스 | `/swarm/mission_intent` (P) |

### P0-6. MissionReady 게이트 (★2 게이트)

| 항목 | 내용 |
|---|---|
| 통과 조건 | QuorumReady + MissionIntent 수신 + 전기 BatteryOK |
| 통과 시 | BT:SwarmCoord가 P1(`TAKEOFF` 상태) 진입 |
| 실패 시 | 해당 게이트 노드 `FAILURE` → BT 대기 (타임아웃 정책 적용 가능) |

---

## P1. 순차 이륙

**구간 목표**: N드론을 수직 충돌 없이 차례로 이륙, 전기 airborne 확인 후 다음 구간 진입.

### P1-1. 이륙 슬롯 게이트

| 항목 | 내용 |
|---|---|
| 액터 | `BT:SwarmCoord` (리더) → `WaitTakeoffSlot` 노드 |
| 동작 | 순번 i = drone_id 정렬(또는 takeoff_slot 파라미터) → i번 드론에만 이륙 허가 |
| 분리 수단 | 시간 간격(t_stagger) + 수직 고도 분리 (선택) |

### P1-2. 드론 자기 이륙 (per-drone)

| 항목 | 내용 |
|---|---|
| 트리거 | `PhaseIs(TAKEOFF)` 참 + `WaitTakeoffSlot` 허가 |
| 액터 | `BT:SwarmDrone` → `Arm` → `TakeoffBehavior` (AS2 재활용) |
| 동작 | Arm 명령 → Offboard 모드 전환 → `takeoff_behavior` 목표 고도 상승 |
| 완료 신호 | `platform/info.armed == true` AND `self_localization 고도 ≥ 임계` |

### P1-3. 전기 airborne barrier (aggregator)

| 항목 | 내용 |
|---|---|
| 액터 | `aggregator` (리더), `BT:SwarmCoord` → `AwaitAllAirborne` 노드 |
| 동작 | 전 드론 `platform/info` + `self_localization/pose` 수집 → 고도 임계 판정 |
| 통과 | 전 등록 드론 airborne 확인 → `airborne=true` 발행 |
| 교착 방지 | 타임아웃 경과 + 낙오기 제외 정책 (aggregator §1.8.2) |
| 통과 후 | BT:SwarmCoord → `FORM_UP` (P2 진입) |

---

## P2. 편대 Transit (FORM_UP → TRANSIT)

**구간 목표**: 이격된 드론들을 편대로 수렴 후, 편대 강체 이동으로 정찰구역 진입점까지 이동.

### P2-1. 편대 수렴 (FORM_UP)

| 항목 | 내용 |
|---|---|
| 트리거 | `AwaitAllAirborne` 통과 |
| 액터 | `BT:SwarmCoord` → `SwarmFlockingStart` AC → `SwarmFlockingBehavior` AS (리더) |
| goal | `{virtual_centroid=현 군집 중심, swarm_formation=line-abreast 오프셋[], drones_namespace[]}` |
| per-drone | `SwarmFlockingBehavior` → `DroneSwarm::FollowReference` AC (×N) 발행 |
| 수렴 판정 | `DroneSwarm::checkPosition(0.3m)` 전 드론 통과 |
| 인터페이스 | `SwarmFlocking.action` (AC→AS), `FollowReference.action` (AC→AS×N) |
| 편대 형태 | **line-abreast (횡대)** — P3 swath 폭 최대화 |

### P2-2. centroid 구동 (TRANSIT)

| 항목 | 내용 |
|---|---|
| 트리거 | `AwaitAllFormed` (편대 수렴 barrier) 통과 |
| 액터 | `BT:SwarmCoord` → `DriveCentroid` 노드 → `centroid_driver` 노드 |
| 동작 | `centroid_driver`가 virtual_centroid를 정찰구역 진입점까지 연속 보간 이동 → **centroid TF broadcast** (고빈도, `modify_swarm` 폴링 지양) |
| per-drone | `SwarmFlockingBehavior`가 centroid TF 참조 → 드론별 FollowReference 슬롯 갱신 → 편대 강체 이동 |
| 완료 | centroid가 진입점 도달 (거리 임계) + 편대 대형 유지 |
| 인터페이스 | centroid TF broadcast (earth → virtual_centroid), `modify_swarm.srv` (필요 시) |

---

## P3. 구역분할 독립정찰 (DISBAND → RECON)

**구간 목표**: 편대 해제 후 N드론이 각자의 sub-zone을 병렬 독립 커버리지+탐지. 군집 고유 핵심 구간.

### P3-1. 편대 해제 + 구역 배정 (DISBAND)

| 단계 | 동작 |
|---|---|
| ① SwarmFlocking 해제 | `BT:SwarmCoord` → `SwarmFlockingStop` AC → `SwarmFlockingBehavior::on_deactivate` + `DroneSwarm::stopFollowReference` (×N) |
| ② 드론 호버 | 각 드론 현 위치 호버 (FollowReference 해제 후 자동) |
| ③ 구역 분할 | `allocator` → `zone_partitioner(구역 폴리곤, N)` → sub-zone[] 산출 (균등 면적·경로 길이) |
| ④ 경로 생성 | `allocator` → `coverage_planner` (boustrophedon, FOV 기반 swath) → 드론별 `sub_route` (lawnmower waypoints) |
| ⑤ 배분 | `allocator` → `/{ns}/swarm_agent/task` 발행 (payload: `{zone, sub_route, classes, role, takeoff/land_slot}`) |

**리스크 R1**: 동시 산개 시 경로 교차 충돌
- **완화**: 산개 순번(순차 GoTo) + 고도층 분리 (드론 i → base_altitude + i × Δh)

### P3-2. 진입점 이동 (GoTo)

| 항목 | 내용 |
|---|---|
| 트리거 | `/{ns}/swarm_agent/task` 수신 + `PhaseIs(RECON)` |
| 액터 | `BT:SwarmDrone` → `GoToBehavior` (AS2 재활용) |
| 동작 | 각 드론이 자기 sub-zone 진입점으로 GoTo (독립, 산개 순번 반영) |

### P3-3. 독립 커버리지 + 탐지 (RECON)

| 항목 | 내용 |
|---|---|
| 트리거 | 진입점 도달 |
| 액터 | `BT:SwarmDrone` → Parallel{ `FollowPathBehavior`, `DetectObjectsBehavior`, `PointGimbalBehavior` } |
| FollowPath | `sub_route` 경로를 순차 웨이포인트 추종 (AS2 재활용) |
| DetectObjects | EO/IR 이미지 추론(ONNX/TensorRT) → bbox+class → **world 좌표 지오로케이션** → `/{ns}/perception/detections` 발행 |
| PointGimbal | 카메라 하향 고정 (AS2 재활용) |
| 표적 발견 시 | `ObjectDetected` 조건 참 → `InspectTarget` (loiter + 재확인) → 커버리지 복귀 |

### P3-4. 탐지 융합 (detection_fusion)

| 항목 | 내용 |
|---|---|
| 액터 | `detection_fusion` 노드 (리더만 활성) |
| 입력 | `/{ns}/perception/detections` (S×N — 전 드론) |
| 처리 | world 좌표 **dedup·병합** (거리 임계 내 같은 표적 → 1개 트랙) + 트랙 수명관리 |
| 출력 | `/swarm/targets` (TargetTrack[]: id, Pose, class, first_seen, last_seen, observed_by) |
| 소비자 | `allocator` (재할당 판단), `GCS` (실시간 모니터) |

**리스크 R3**: 같은 표적 다중기 중복 계수 → world 좌표 dedup + 트랙 수명 timeout으로 완화.

### P3-5. RF 방사원 탐지 (선택 임무, rf_survey + emitter_fusion)

| 항목 | 내용 |
|---|---|
| 액터 | `rf_survey` behavior (드론별), `emitter_fusion` (리더) |
| 동작 | 스캔 + 방향탐지(DF) 기동 → 방위 산출 → `/{ns}/rf/bearings` 발행 |
| 융합 | `emitter_fusion` → 삼각측량(LS/MLE) → `/swarm/emitters` (EmitterTrack[]) |
| 제약 | 삼각측량 유효 → 참여 드론 간 baseline 최소거리 확보 필요 |

### P3-6. 전구역 완료 barrier (aggregator / completion_barrier)

| 항목 | 내용 |
|---|---|
| 액터 | `aggregator` + `BT:SwarmCoord` → `AwaitAllZonesDone` 노드 |
| 완료 신호 | 각 드론 `FollowPath` action SUCCESS = 자기 sub-zone 완료 |
| 집계 | aggregator가 `zonesdone` 배열 수집 → 전 드론 완료 판정 |
| 교착 방지 | 타임아웃 경과 + 낙오기(coverage율 임계치 충족 시) 제외 |
| 통과 | `AwaitAllZonesDone` SUCCESS → P4 진입 |

---

## P4. 완료판정 · 복귀 (REGROUP → RETURN)

**구간 목표**: 산개된 드론들을 재편대로 집결 후 home으로 복귀.

### P4-1. 재편대 (REGROUP)

| 항목 | 내용 |
|---|---|
| 트리거 | `AwaitAllZonesDone` 통과 |
| 액터 | `BT:SwarmCoord` → `SwarmFlockingStart` AC (P2와 동일 기구, 재활성) |
| goal | `{virtual_centroid=현 드론 분포 중심, swarm_formation=line-abreast}` |
| 슬롯 배정 | 현 위치 최근접 슬롯 할당 (헝가리안 매칭) — 수렴 벡터 교차 최소화 |
| 고도층 분리 | 수렴 중 충돌 방지 (R1과 동일 완화) |
| 완료 | `DroneSwarm::checkPosition(0.3m)` 전 드론 통과 |

**리스크 R1 (재발)**: 산개→집결 수렴 경로 교차 → 최근접 슬롯 매칭 + 고도층 분리로 완화.

### P4-2. Home 복귀 (RETURN)

| 항목 | 내용 |
|---|---|
| 액터 | `centroid_driver` → centroid를 home 좌표로 보간 이동 |
| per-drone | FollowReference 슬롯 유지, 편대 강체 복귀 (P2 역순) |
| 완료 | centroid가 home 도달 + 편대 유지 |

---

## P5. 순차 착륙

**구간 목표**: 착륙 지점 충돌 없이 차례로 착륙, 전기 landed 확인 후 임무 종료.

### P5-1. 착륙 슬롯 게이트

| 항목 | 내용 |
|---|---|
| 트리거 | centroid@home 도달 (P4 완료) + `SwarmFlockingStop` |
| 액터 | `BT:SwarmCoord` → `WaitLandSlot` 노드 |
| 동작 | 순번 i → i번 드론에만 착륙 허가. 이전 드론 landed 확인 후 다음 허가. |

### P5-2. 드론 자기 착륙 (per-drone)

| 항목 | 내용 |
|---|---|
| 트리거 | `PhaseIs(LANDING)` + `WaitLandSlot` 허가 |
| 액터 | `BT:SwarmDrone` → `LandBehavior` (AS2 재활용) |
| 완료 신호 | `platform/info.disarmed == true` AND 착지 감지 |

### P5-3. 전기 landed barrier (aggregator)

| 항목 | 내용 |
|---|---|
| 액터 | `aggregator` + `BT:SwarmCoord` → `AwaitAllLanded` 노드 |
| 통과 | 전 드론 landed 확인 → `BT:SwarmCoord` → `DONE` 상태 |
| 임무 완료 | `/swarm/mission_phase` = `DONE` 발행 → GCS 모니터 수신 |

---

## 비정상 시나리오

### E1. 드론 이탈 (heartbeat 소실)

| 구간 | 영향 | 처리 |
|---|---|---|
| P0 | heartbeat 목록에서 제거 → 정족수 재판정 | N_min 미충족 시 임무 개시 보류 |
| P1~P5 | election 재계산 → 리더였으면 승계 | aggregator 낙오기 제외 정책 적용 |
| P3 | 해당 sub-zone 미완료 | 인접 드론 재할당(allocator) 또는 임무 부분 완료 허용 |

### E2. 배터리 경고 (BatteryLow)

| 처리 | `health_monitor` → DEGRADED → heartbeat에 반영 → BT:SwarmDrone `SafeHover` 폴백 → 독립 귀환 |
|---|---|
| 리더이면 | 승계 후 이탈 드론 처리 (E1과 동일) |

### E3. P2↔P3 모드전환 충돌 (R1)

| 상황 | 완화 |
|---|---|
| DISBAND 시 산개 경로 교차 | 순번 GoTo (최대 N초 간격) + 고도층 분리 (드론 i → alt + i×Δh) |
| REGROUP 시 수렴 교차 | 최근접 슬롯 매칭(헝가리안) + 고도층 분리 |

### E4. barrier 교착 (낙오기로 임무 대기)

| 처리 | aggregator 타임아웃 (T_barrier) 경과 → 낙오기 제외 후 barrier 통과 |
|---|---|
| 정책 | coverage율 ≥ 임계(예: 80%) 또는 elapsed ≥ T_barrier → 부분 완료 허용 |

---

## 구간-WBS-게이트 대응표

| 구간 | 관련 WBS | 게이트 | 검증 시나리오 |
|---|---|---|---|
| P0 부트스트랩 | 1.5.1~1.5.4, 1.6.5, 1.6.8 | **★2** | N기 Sim: 지상→heartbeat 안정→리더 1개→정족수→자율 P1 진입 |
| P1 순차이륙 | 1.6.2, 1.6.3, 1.8, 1.9.3 | — | N기 Sim: 충돌 없는 순차 이륙 + 전기 airborne barrier |
| P2 편대 transit | 1.3.3, 1.5.6, 1.6.1, 1.6.6, 1.9.1 | — | N기 Sim: line-abreast 수렴 + centroid 이동 + 진입점 도달 |
| P3 독립정찰 (단일기 박판) | 1.2.1~1.2.3, 1.3.1~1.3.2, 1.9.3 | **★1** | 1기 Sim: takeoff→coverage→탐지 발행→inspect→복귀→land |
| P3 독립정찰 (군집 EO/IR) | 1.4.1~1.4.2, 1.6.7, 1.9.1~1.9.2 | **★3** | N기 Sim: 분할정찰 + 탐지융합(중복 0) + 전구역 barrier |
| P4 복귀 | 1.5.5, 1.5.6, 1.6.1, 1.10.4 | — | N기 Sim: REGROUP 충돌 0 + centroid@home |
| P5 순차착륙 | 1.6.3, 1.8, 1.9.3 | — | N기 Sim: 충돌 없는 순차 착륙 + 전기 landed |
| 일반성 (2번째 임무) | 1.10.5 | **★4** | 감시 임무: chassis 무수정 + SurveilP3 2서브트리만 추가 |

---

## 임계 경로 요약

```
★2 (P0 자율개시)
  ← 1.5 swarm_agent (election/registry/heartbeat)
  ← 1.6.5 부트스트랩 BT 노드
  ← 1.1 인터페이스 동결            ← 전체 임계 시작점

★1 (P3 단일기 박판)
  ← 1.2 detect_objects + 1.3 coverage_planner
  ← 1.9.3 motion behavior 연동

★3 (P3 군집 풀미션)
  ← 1.4 detection_fusion
  ← 1.6.7 EO/IR P3 BT 트리
  ← 1.9.1~1.9.2 SwarmFlocking/FollowReference 연동

★4 (일반성)
  ← ★3 통과
  ← 감시 임무 P3 서브트리 2파일 추가
```

**병렬 트랙**: 1.12 HW(EO/IR/RF 카메라, FCU, 통신)는 S1부터 별도 리드타임 트랙 진행.

---

_근거: `AS2_SWARM_WBS.md` (§2 스케줄·§3 임계경로·§4 게이트), `AS2_RECON_SWARM_PHASED_DESIGN.md` (§5 단계별 상세), `AS2_SWARM_SEQUENCE_DIAGRAM.md`, `AS2_SWARM_NODE_RESPONSIBILITY.md`._
