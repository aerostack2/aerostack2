# 군집 정찰 임무 설계 — 전체 구조부터 상세 설계까지

> **대상 임무**: 순차 이륙 → 편대 transit → 구역분할 독립정찰 → 완료판정·복귀 → 순차 착륙.
> **근거**: AS2 실소스 코드 + `recon_mission_example.xml` + `AS2_RECON_BEHAVIOR_DESIGN.md`. (기존 swarm 설계문서 미참조.)
> **핵심 통찰**: 임무가 **편대 모드(P2,P4)와 독립 모드(P3) 사이를 천이**하는 단계 상태기계. AS2 실코드는 두 비행모드(SwarmFlocking 편대 / FollowPath 독립)를 모두 제공하나, **단계 오케스트레이션·모드전환·동기 barrier는 전무**. 신규의 대부분은 비행동작이 아니라 조율 계층.

---

## 0. 문서 구조 (workflow)

1. 임무 형상 (phase model)
2. 전체 아키텍처 (계층/노드/coordinator-perdrone 분리)
3. 컴포넌트 인벤토리 (재활용 vs 신규, 코드 근거)
4. 핵심 신규: swarm mission orchestrator (단계 상태기계)
5. 단계별 상세 설계 (P1~P5)
6. 모드전환 설계 (편대 ⇄ 독립)
7. 신규 모듈 상세 (7종)
8. 메시지·토픽 ICD 추가
9. BT 구조 (coordinator 트리 + per-drone 트리)
10. 데이터 흐름
11. 구현 로드맵 (마일스톤)
12. 미해결·리스크

---

## 1. 임무 형상 (Phase Model)

```
P1 순차이륙 ─▶ P2 편대 transit ─▶ P3 구역분할 독립정찰 ─▶ P4 완료판정·복귀 ─▶ P5 순차착륙
  [개별]        [편대모드]          [독립모드]               [전환]             [개별]
  stagger       SwarmFlocking       FollowPath×N 독립         재편대            stagger
  + barrier     centroid 구동       + DetectObjects           + barrier         + barrier
```

| Phase | 제어모드 | 동기 | 비행 주체 |
|---|---|---|---|
| P1 순차이륙 | 개별 | 순번 게이트 + 전기 airborne barrier | 드론 자기 takeoff |
| P2 편대 transit | **편대** | centroid 도착 판정 | coordinator가 centroid 구동, 드론은 FollowReference |
| P3 구역분할 정찰 | **독립** | (없음, 병렬) | 드론 자기 FollowPath + 인식 |
| P4 완료·복귀 | 전환→편대 | 전구역 완료 barrier | 완료집계 후 재편대 복귀 |
| P5 순차착륙 | 개별 | 순번 게이트 | 드론 자기 land |

천이 지점 2곳이 설계 난점: **P2→P3 (편대 해제→독립 산개)**, **P3→P4 (독립→재편대 집결)**.

---

## 2. 전체 아키텍처

### 2.1 제어 위상: Coordinator + Per-drone (2계층)

AS2 실코드의 `SwarmFlockingBehavior`가 이미 **중앙 coordinator가 N드론을 FollowReference로 지휘**하는 구조다 (`as2_behaviors_swarm_flocking/`). 본 임무도 동일 위상을 따른다.

```
┌─ Swarm Coordinator (1 노드/프로세스) ──────────────────────────────┐
│  ▸ swarm_mission_orchestrator (신규 ★)  — 5단계 상태기계           │
│  ▸ SwarmFlockingBehavior (재활용)       — P2/P4 편대                │
│  ▸ takeoff/landing sequencer (신규)     — P1/P5 순번·barrier        │
│  ▸ zone_partitioner (신규)              — P3 구역분할               │
│  ▸ coverage_planner (신규)              — P3 sub-zone 경로          │
│  ▸ detection_fusion (신규)              — P3 탐지 융합              │
│  ▸ centroid_driver (신규)               — P2/P4 centroid 궤적       │
│  ▸ completion_barrier (신규)            — P4 전구역 완료 집계       │
└───────────────┬───────────────────────────────────────────────────┘
   per-drone 액션/서비스 호출 (FollowReference/FollowPath/Takeoff/Land/...)
┌─ Per-drone 스택 (N 인스턴스, 드론 namespace) ──────────────────────┐
│  AS2 behaviors: takeoff·go_to·follow_path·follow_reference·land     │
│                 point_gimbal · detect_objects(신규)                 │
│  AS2 substrate: controller · platform · state_estimator             │
└────────────────────────────────────────────────────────────────────┘
```

- **Coordinator = 조율·계획·동기**. 무거운 계산(분할·경로·융합) + 단계 진행.
- **Per-drone = 실행**. AS2 behavior 그대로. 정찰용 `detect_objects`만 신규.

### 2.2 계층 매핑 (AS2 substrate 위)

```
L6 Swarm Coordinator  : orchestrator + SwarmFlocking + 신규 6모듈
L5 AS2 Behaviors       : takeoff/goto/follow_path/follow_reference/land/point_gimbal + detect_objects(신규)
L4 Motion Ref Handlers : (재활용)
L3 Motion Controller   : (재활용)
L2 Aerial Platform     : (재활용)
L1 HW/Sim + EO/IR 센서  : EO/IR 카메라 신규 연동
L0 State Estimator     : self_localization (재활용, 융합 좌표 기준)
```

---

## 3. 컴포넌트 인벤토리 (코드 근거)

### 3.1 재활용 (실코드 존재)

| 컴포넌트 | 파일 | 임무 활용 |
|---|---|---|
| `SwarmFlockingBehavior` | `as2_behaviors_swarm_flocking/.../swarm_flocking_behavior.hpp` | P2/P4 편대 형성·유지 |
| `DroneSwarm` (드론 단위 추종) | `.../drone_swarm.hpp` | 드론별 FollowReference 클라이언트, slot TF, `checkPosition`(0.3m), `stopFollowReference` |
| `SwarmFlocking.action` | `as2_msgs/action/SwarmFlocking.action` | virtual_centroid + swarm_formation(PoseWithID[]) + drones_namespace[] |
| `modify_swarm` srv | `as2_msgs/srv/ModifySwarm` | centroid 변경·드론 추가/분리 |
| `dynamic_swarm_formation` 토픽 | (PoseWithIDArray) | 동적 재편대 |
| `FollowReference` behavior + action | `as2_behaviors_motion/follow_reference_behavior/`, `as2_msgs/action/FollowReference.action` | 슬롯 추종 (target_pose PointStamped, max_speed_xyz, YawMode) |
| `takeoff`/`land`/`follow_path`/`go_to` behavior | `as2_behaviors_motion/` | P1/P5/P3 비행 |
| `point_gimbal` behavior | `as2_behaviors_payload/point_gimbal_behavior/` | EO/IR 지향 |
| `BehaviorServer<actionT>` 템플릿 | `as2_behaviors/as2_behavior/` | 신규 behavior 골격 |
| mission_interpreter (단일드론 순차) | `as2_python_api/.../mission_interpreter/` | per-drone 미션 시퀀스 기반 |

### 3.2 신규 (코드 부재)

| 컴포넌트 | 유형 | 단계 | 사유 |
|---|---|---|---|
| **swarm_mission_orchestrator** | 노드/상태기계 | 전체 | 단계 진행·모드전환·barrier 전무 |
| **takeoff/landing sequencer** | 노드/로직 | P1/P5 | 순차·동기 barrier 전무 |
| **zone_partitioner** | 노드/lib | P3 | 구역분할기 전무 |
| **coverage_planner** | path_planning plugin | P3 | lawnmower 생성기 전무 |
| **detect_objects** behavior | behavior | P3 | EO/IR 객체탐지 전무 (aruco만 존재) |
| **detection_fusion** | 노드 | P3 | 다중기 탐지 dedup 전무 |
| **centroid_driver** | 노드/로직 | P2/P4 | SwarmFlocking centroid는 정적 |
| **completion_barrier** | 노드/로직 | P4 | 전구역 완료 집계 전무 |
| 신규 메시지 (8절) | msg/srv/action | 전체 | 탐지·단계·배정 표현 |

---

## 4. 핵심 신규: swarm_mission_orchestrator (단계 상태기계)

전체 임무를 구동하는 중앙 상태기계. 단계 진행 + 모드전환 + 동기를 단일 책임으로 보유.

### 4.1 상태 정의

```
IDLE ─(mission_start)─▶ TAKEOFF ─(all_airborne)─▶ FORM_UP
  ─(formed)─▶ TRANSIT ─(centroid@entry)─▶ DISBAND
  ─(all_independent)─▶ RECON ─(all_zones_done)─▶ REGROUP
  ─(formed)─▶ RETURN ─(centroid@home)─▶ LANDING ─(all_landed)─▶ DONE
                                  │
   any state ─(alert/abort)──────▶ EMERGENCY
```

| 상태 | 동작 | 진입조건(barrier) |
|---|---|---|
| TAKEOFF | sequencer가 드론별 순차 이륙 트리거 | mission_start |
| FORM_UP | SwarmFlocking 활성, 편대 수렴 대기 | all_airborne |
| TRANSIT | centroid_driver가 진입점으로 구동 | formed (전기 checkPosition) |
| DISBAND | SwarmFlocking 해제, zone 배포, 드론 독립전환 | centroid@entry |
| RECON | 드론별 FollowPath+detect, detection_fusion 가동 | all_independent |
| REGROUP | SwarmFlocking 재활성, 집결 | all_zones_done (completion_barrier) |
| RETURN | centroid_driver가 home으로 구동 | formed |
| LANDING | sequencer가 드론별 순차 착륙 | centroid@home |
| EMERGENCY | 전기 halt→호버→복귀→착륙 | alert |

### 4.2 인터페이스

| 방향 | 토픽/서비스 | 용도 |
|---|---|---|
| 입력 | `/swarm/mission_intent` (구역 폴리곤, 표적 클래스, 편대 형태) | GCS 의도 |
| 입력 | `/{ns}/platform/info`, `/{ns}/self_localization/pose` | 드론 상태·위치 |
| 입력 | per-behavior 완료 피드백 (action result) | barrier 판정 |
| 입력 | `/swarm/targets` (detection_fusion) | 탐지 |
| 출력 | `/swarm/mission_phase` (현 단계) | 모니터·per-drone 게이트 |
| 출력 | SwarmFlocking goal / `modify_swarm` 호출 | 편대 제어 |
| 출력 | `/{ns}/swarm_agent/task` (zone, route, role) | per-drone 배정 |

---

## 5. 단계별 상세 설계

### P1. 순차 이륙
**목표**: N드론을 충돌 없이 차례로 이륙, 전기 airborne 후 다음 단계.

- 입력: drones_namespace[], 이륙 고도, 순번(혹은 id 정렬).
- 동작: orchestrator(TAKEOFF) → sequencer가 순번 i 드론에 `Arm→Offboard→Takeoff` 트리거 → `platform/info.armed && self_localization 고도≥임계` 확인 후 i+1 진행. (옵션: 슬롯 간격 시간 + 수직 분리.)
- 완료 barrier: 전 드론 takeoff action SUCCESS + 고도 도달.
- 재활용: `takeoff_behavior`, `behavior_actions/takeoff_behavior.py`.
- 신규: 순번 게이트 + 동기 barrier.

### P2. 편대 transit
**목표**: 편대 형성 후 정찰구역 진입점까지 편대 유지 이동.

- FORM_UP: orchestrator가 `SwarmFlocking` 활성 — goal = {virtual_centroid=현 군집 중심, swarm_formation=line-abreast 오프셋[], drones_namespace[]}. 각 드론 FollowReference가 슬롯 수렴. 수렴 판정 = 전 DroneSwarm `checkPosition()` (0.3m).
- TRANSIT: `centroid_driver`가 virtual_centroid를 진입점까지 보간 이동 (직선/경유). 편대 강체 이동.
  - 구현: `modify_swarm`(centroid 갱신)을 고빈도 스텝 호출, 또는 centroid TF를 driver가 연속 발행 + SwarmFlocking이 그 frame 참조.
- 완료: centroid가 진입점 도달 (거리 임계) + 편대 유지.
- 편대 형태: line-abreast(횡대) — P3 swath 폭 최대화에 유리.

### P3. 구역분할 독립정찰 ★
**목표**: 정찰구역을 N분할, 각 드론이 자기 sub-zone을 독립 커버리지+탐지.

- DISBAND (모드전환): orchestrator가 `SwarmFlocking` deactivate → 각 DroneSwarm `stopFollowReference`. 드론 독립.
- 분할: `zone_partitioner`가 구역 폴리곤 → N sub-zone (균등 면적/경로길이). 각 sub-zone → `coverage_planner`가 lawnmower waypoint(`sub_route`). 진입점·간격 = 카메라 FOV.
- 배포: `/{ns}/swarm_agent/task` = {zone, sub_route, classes}.
- per-drone 실행: `recon_mission_example.xml` 본체 재활용 —
  `FollowPath {sub_route}` ∥ `DetectObjects {classes}` ∥ `PointGimbal(하향)`.
  탐지 시 자기 `InspectTarget`(loiter) 후 커버리지 복귀 (독립이므로 편대 왜곡 없음 — 산개 상태).
- 탐지 융합: `detection_fusion`이 전 드론 `/{ns}/perception/detections` 수집 → world좌표 dedup → `/swarm/targets`.
- 완료: 드론별 `FollowPath` SUCCESS = 자기구역 완료.
- 공백: 구역 경계 deconfliction (분할이 공간분리 제공하나 경계 중첩 주의).

### P4. 완료판정 + 복귀
**목표**: 전구역 완료 확인 → 재편대 → home 복귀.

- 완료판정: `completion_barrier`가 전 드론 구역완료 수집 → 전구역 완료 시 임무완료. (옵션: coverage율·시간·표적수 종료조건.)
- REGROUP (모드전환 역): orchestrator가 `SwarmFlocking` 재활성 → 산개 드론 슬롯 재수렴. 집결 시 deconfliction 주의(수렴 경로 교차).
- RETURN: `centroid_driver`가 centroid를 home으로 구동 (P2 역순).

### P5. 순차 착륙
**목표**: 착륙지점 충돌 없이 차례로 착륙.

- LANDING: orchestrator가 `SwarmFlocking` 해제 → sequencer가 순번대로 드론 `Land` 트리거 → 착지 확인 후 다음.
- 완료: 전 드론 `disarmed`/landed.
- 재활용: `land_behavior`. 신규: 착륙 슬롯 게이트.

---

## 6. 모드전환 설계 (편대 ⇄ 독립)

천이 안전성이 임무 최대 리스크.

### 6.1 편대 → 독립 (P2→P3, DISBAND)
1. orchestrator: `SwarmFlocking` deactivate 요청.
2. 각 DroneSwarm `stopFollowReference` → 드론 현 위치 호버.
3. zone/route 배포 (`swarm_agent/task`).
4. 각 드론 진입점으로 `GoTo`(산개) → `FollowPath` 시작.
- 리스크: 동시 산개 시 경로 교차 충돌. → 산개 순번/고도층 분리.

### 6.2 독립 → 편대 (P3→P4, REGROUP)
1. orchestrator: `SwarmFlocking` 재활성 (현 위치 기준 centroid + 오프셋).
2. 각 드론 FollowReference 슬롯 수렴.
- 리스크: 수렴 벡터 교차. → 슬롯 배정을 현 위치 최근접으로(헝가리안 매칭) + 고도층.

### 6.3 전환 기구 (실코드)
- 해제: `SwarmFlocking::on_deactivate`, `DroneSwarm::stopFollowReference` 존재.
- 재형성: `SwarmFlocking` 재activate / `modify_swarm`.
- **신규 = 전환 트리거·타이밍·deconfliction 로직** (기구 자체는 재활용).

---

## 7. 신규 모듈 상세

| 모듈 | 입력 | 출력 | 핵심 로직 |
|---|---|---|---|
| **orchestrator** | mission_intent, 드론상태, 완료피드백, targets | mission_phase, SwarmFlocking 제어, task 배포 | 4절 상태기계 |
| **takeoff/landing sequencer** | drones_namespace, 순번 | 드론별 takeoff/land 트리거 | 슬롯 게이트 + airborne/landed barrier |
| **zone_partitioner** | 구역 폴리곤, N | sub-zone[] | 균등분할(스트립/Voronoi) |
| **coverage_planner** (plugin) | sub-zone, FOV/간격 | lawnmower waypoint | boustrophedon |
| **detect_objects** (behavior) | EO/IR image+camera_info | `perception/detections` | DNN/IR CV 추론 + 지오로케이션 |
| **detection_fusion** | `/{ns}/perception/detections`×N | `/swarm/targets` | world좌표 dedup/병합 |
| **centroid_driver** | 목표점, 현 centroid | centroid TF/`modify_swarm` | 보간 궤적 + 도착판정 |
| **completion_barrier** | 드론별 구역완료 | mission_complete | AND 집계 + 종료조건 |

---

## 8. 메시지·토픽 ICD 추가

### 신규 메시지
| 이름 | 필드(요지) | 생산자 |
|---|---|---|
| `DetectionArray.msg` | drone_id, [class, score, PoseStamped(world)] | detect_objects |
| `TargetTrack.msg` | id, PoseStamped, class, first_seen, last_seen | detection_fusion |
| `SwarmTask.msg` | zone(Polygon), sub_route(Path), classes[], role | orchestrator |
| `MissionPhase.msg` | phase(enum), seq | orchestrator |

### 신규 액션
| 이름 | goal | 비고 |
|---|---|---|
| `DetectObjects.action` | classes[], min_score | `DetectArucoMarkers.action` 미러 |

### 신규 토픽 (드론 ns 상대 / `/swarm/*` 전역)
| 토픽 | 타입 | 방향 |
|---|---|---|
| `sensor_measurements/eo/image_raw`(+info) | sensor_msgs/Image(+CameraInfo) | EO → detect_objects |
| `sensor_measurements/ir/image_raw`(+info) | sensor_msgs/Image(+CameraInfo) | IR → detect_objects |
| `perception/detections` | DetectionArray | detect_objects → fusion |
| `/swarm/targets` | TargetTrack[] | fusion → orchestrator/GCS |
| `/swarm/mission_phase` | MissionPhase | orchestrator → 전체 |
| `/{ns}/swarm_agent/task` | SwarmTask | orchestrator → 드론 |

(기존 ICD = `AS2_RUNTIME_TOPIC_ICD.md`.)

---

## 9. BT 구조

실코드 위상상 **coordinator 트리 1개 + per-drone 트리 N개**로 분리. (단일트리 불가 — 모드전환·드론간 barrier 때문.)

### 9.1 Coordinator 트리 (orchestrator 구동, 의사 BT)
```
Parallel
├─ WaitForAlert → EmergencyAll            (안전 선점)
└─ SequenceStar                            (단계 진행)
   ├─ SequentialTakeoff drones={ns[]}      (P1)
   ├─ FormUp formation=line                (P2 FORM_UP, SwarmFlocking 활성)
   ├─ DriveCentroid to={entry}             (P2 TRANSIT)
   ├─ DisbandToZones zones={partition}     (P3 DISBAND, 분할 배포 + 해제)
   ├─ AwaitAllZonesDone                    (P3→P4 completion_barrier)
   ├─ FormUp formation=line                (P4 REGROUP)
   ├─ DriveCentroid to={home}              (P4 RETURN)
   └─ SequentialLand drones={ns[]}         (P5)
```

### 9.2 Per-drone 트리 (mission_phase 게이트, `recon_mission_example.xml` 본체 재활용)
```
Parallel
├─ WaitForAlert → Emergency
└─ ReactiveFallback (phase 게이트)
   ├─ Phase==TAKEOFF  → ArmTakeoff(slot)
   ├─ Phase==TRANSIT  → FollowReference(slot)        ← 편대 추종
   ├─ Phase==RECON    → Parallel{ FollowPath{sub_route}
   │                              ∥ DetectObjects{classes}
   │                              ∥ PointGimbal(하향)
   │                              + ObjectDetected→InspectTarget }   ← recon_mission_example.xml
   ├─ Phase==REGROUP/RETURN → FollowReference(slot)   ← 재편대 추종
   ├─ Phase==LANDING  → Land(slot)
   └─ SafeHover                                       (기본 호버)
```
- per-drone는 **`/swarm/mission_phase` 구독**으로 동작 전환. 비행동작은 전부 기존 AS2 behavior + recon 신규 `DetectObjects`.

---

## 10. 데이터 흐름

```
GCS 의도 ─▶ orchestrator
  P1: orchestrator → sequencer → 드론별 Takeoff ─(airborne)─▶ barrier
  P2: orchestrator → SwarmFlocking(activate) ; centroid_driver → centroid TF
      → 드론 FollowReference(슬롯) ─(checkPosition)─▶ 도착
  P3: orchestrator → SwarmFlocking(deactivate) ; zone_partitioner+coverage → task 배포
      → 드론 FollowPath+DetectObjects ; EO/IR → detections → detection_fusion → /swarm/targets
      → FollowPath SUCCESS → completion_barrier
  P4: completion → orchestrator → SwarmFlocking(activate) ; centroid_driver → home
  P5: orchestrator → sequencer → 드론별 Land ─(landed)─▶ DONE
센서 ─▶ state_estimator ─▶ self_localization ─▶ (FollowReference/FollowPath/fusion 좌표)
```

---

## 11. 구현 로드맵 (마일스톤)

| M | 범위 | 산출 | 검증 |
|---|---|---|---|
| M1 | 메시지·액션 (8절) | DetectObjects.action, DetectionArray/TargetTrack/SwarmTask/MissionPhase | 빌드 |
| M2 | detect_objects behavior | aruco 템플릿 → EO/IR 2-카메라 → 더미 detector | 단일기 탐지 pub |
| M3 | coverage_planner + zone_partitioner | plugin + 분할 lib | 폴리곤→N route 시각화 |
| M4 | per-drone 정찰 (P3) | recon BT + detect_objects 통합 | 1기 sub-zone 정찰 Sim |
| M5 | SwarmFlocking 편대 (P2/P4) | 재활용 + centroid_driver | N기 편대 transit Sim |
| M6 | orchestrator 상태기계 | 5단계 + barrier | 전 단계 순차 진행 Sim |
| M7 | 모드전환 + sequencer | DISBAND/REGROUP deconfliction, 순차 이착륙 | N기 풀 미션 Sim |
| M8 | detection_fusion | dedup → /swarm/targets | 다중기 중복탐지 융합 |
| M9 | 실기 연동 | EO/IR HW, MAVLink platform | 야외 시험 |

**의존**: M1→(M2,M3)→M4→M5→M6→M7. detect_objects(M2)와 편대(M5)는 병행 가능.

---

## 12. 미해결 · 리스크

| # | 항목 | 리스크 | 완화 |
|---|---|---|---|
| R1 | P2↔P3 모드전환 | 산개/집결 시 경로 교차 충돌 | 고도층 분리 + 순번 + 최근접 슬롯 매칭 |
| R2 | centroid 정적 한계 | 부드러운 편대 이동 어려움 | centroid_driver 연속 TF 발행(modify_swarm 폴링 지양) |
| R3 | 다중기 탐지 중복 | 같은 표적 N중 계수 | world좌표 dedup + 트랙 수명관리 |
| R4 | 구역 경계 | 인접 드론 경계 충돌/누락 | overlap 마진 + 경계 deconfliction |
| R5 | 순차 이착륙 동기 | barrier 미충족 시 교착 | 타임아웃 + 낙오기 제외 정책 |
| R6 | coordinator SPOF | 1노드 장애 시 전체 정지 | (본 설계는 중앙형. 분산 선출은 별도 과제) |
| R7 | EO/IR 지오로케이션 | ray-ground 오차 | 지형고도/짐벌자세 보정 |

---

## 13. 결론

- **재활용 2기둥**: `SwarmFlocking`(편대=P2/P4) + `FollowPath`/recon BT(독립=P3). 둘 다 실코드 존재.
- **신규 본질 = 조율 계층**: orchestrator 상태기계 + 순차 sequencer + 모드전환 + barrier. 비행동작은 거의 재활용.
- **공간 신규**: zone_partitioner + coverage_planner + detect_objects + detection_fusion + centroid_driver.
- **최우선 구현**: M1 메시지 → M6 orchestrator 골격. 나머지 모듈은 단계 훅에 장착.
- **최대 난제**: P2↔P3 모드전환의 충돌안전 천이 (R1).

---

_근거 소스: `as2_behaviors/as2_behaviors_swarm_flocking/`(swarm_flocking_behavior.hpp, drone_swarm.hpp), `as2_msgs/action/{SwarmFlocking,FollowReference,DetectArucoMarkers}.action`, `as2_behaviors/as2_behaviors_motion/`, `as2_behaviors/as2_behaviors_perception/detect_aruco_markers_behavior/`, `as2_behaviors/as2_behavior/`(BehaviorServer), `as2_python_api/.../mission_interpreter/`, `recon_mission_example.xml`, `AS2_RECON_BEHAVIOR_DESIGN.md`._
