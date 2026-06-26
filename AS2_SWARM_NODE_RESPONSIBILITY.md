# 군집 정찰 시스템 — 노드별 기능·역할·책임 상세

> `AS2_SWARM_PACKAGE_ARCHITECTURE.md` §3 매트릭스의 각 노드를 기능/역할/책임 단위로 상세 기술.
> 근거: `AS2_SWARM_MODULE_MANIFEST.md`(§C4/C5/C8/C9), `AS2_SWARM_PACKAGE_ARCHITECTURE.md`, AS2 실코드.

---

## aiss_swarm_core — 군집 두뇌 (L6)

### health_monitor (드론별)
- **기능**: 자가진단. FCU/센서/TF/배터리 폴링.
- **역할**: 단일 드론 건강 판정자. `health(OK/DEGRADED/FAIL)` enum 산출.
- **책임**: platform/info·센서·TF·배터리 읽어 로컬 상태 1개 값 도출. heartbeat에 푸시. 외부 토픽 없음(내부 화살표만).
- **I/O**: 입력 platform/info, 센서, TF, 배터리 → 출력 health enum.

### heartbeat (드론별)
- **기능**: heartbeat 발행 + 타 드론 수신.
- **역할**: 군집 생존 신경망. `/swarm/heartbeat` **P/S**(자기 발행 + N 구독).
- **책임**: 로컬 health 받아 `Heartbeat.msg`(drone_id, alive, health, stamp, position) 주기 발행. 수신분 모아 **생존맵** 유지 → election·aggregator 소비.

### election (드론별)
- **기능**: 리더 선출. **최저 id** 규칙.
- **역할**: split-brain 차단기. `/swarm/leader_id` 단일 생산자.
- **책임**: 생존맵 감시 → 리더 산정. 리더 사망 시 차순위 **승계**(SPOF 완화). leader_id 발행 → 전 조율노드·BT `IsLeader` 게이트.

### registry (리더만 활성)
- **기능**: 합류 수락 + 등록부 + 정족수.
- **역할**: 군집 명부 관리자. `swarm/join` **SrvS**, `/swarm/registry` 발행.
- **책임**: `SwarmJoin.srv`(req{id,health} → res{accepted, registry, mission_version}) 처리. 등록부 유지, **정족수(quorum)** 판정 → BT `QuorumReady`/allocator.

### allocator (리더만 활성)
- **기능**: 임무 분해·배정·재할당.
- **역할**: 작업 분배 두뇌. `/{ns}/swarm_agent/task` 단일 생산자.
- **책임**: intent + registry + targets 받아 `zone_partitioner`(구역분할) + coverage planner(lawnmower 경로) 호출 → 드론별 `SwarmTask.msg`(zone, sub_route, classes, role, takeoff/land_slot) 발행. 추적은 `AllocateTrackRoles` 반응형 재할당.

### centroid_driver (리더만 활성)
- **기능**: virtual_centroid 궤적 구동.
- **역할**: 군집 중심 조종사. centroid **TF** broadcast(토픽 아님).
- **책임**: `swarm_flocking`(재활용) 위에서 **centroid만** 이동. 목표점 + 현 centroid → TF 갱신 / modify_swarm. 개별 드론 추종은 SwarmFlocking이 처리.

### aggregator (리더만 활성, C9)
- **기능**: barrier 상태 집계.
- **역할**: phase 전환 게이트키퍼. BT `Await*` 입력 공급.
- **책임**: 전 드론 platform/info·self_localization·WORK 완료·heartbeat 수집 → `airborne/formed/zonesdone/landed` 판정값 발행. 임계: 고도, checkPosition 0.3m, barrier 타임아웃.

---

## aiss_swarm_perception — 융합 (L6)

### detection_fusion (리더만 활성, C4.1) ★
- **기능**: 다드론 탐지 융합.
- **역할**: 전역 표적 생성기. `/swarm/targets` 단일 생산자.
- **책임**: N 드론 `detections` **S(×N)** → world 좌표 **dedup/병합** → `TargetTrack[]`(id, Pose, class, first/last_seen). 임계: dedup 거리, 트랙 수명 timeout, 병합 정책. → allocator·GCS.

### emitter_fusion (리더만 활성, C4.2)
- **기능**: RF 삼각측량 융합.
- **역할**: 방사원 위치 추정기. `/swarm/emitters` 단일 생산자.
- **책임**: N 드론 `rf/bearings` **S(×N)** → 삼각측량(LS/MLE) → `EmitterTrack[]`(id, Pose, freq). 임계: baseline 최소거리.

---

## aiss_behaviors_perception — 인식 behavior (L5, 드론별)

### detect_objects (C2.1) ★
- **기능**: EO/IR 객체탐지 + 지오로케이션.
- **역할**: 드론 눈. `BehaviorServer<DetectObjects>`(detect_aruco 템플릿).
- **책임**: EO/IR 이미지 추론(ONNXRuntime/TensorRT) → bbox+class → **world 좌표 변환**. `DetectObjects.action` **AS**(BT 호출) + `perception/detections` **P**(→fusion). 의존 OpenCV, cv_bridge, Camera×2, TF.

### rf_survey (C2.2, 드론별)
- **기능**: RF 스펙트럼 스캔 + 방향탐지(DF).
- **역할**: 드론 귀. behavior.
- **책임**: 스캔 + DF 기동 → 방위 산출. mode(triangulate/df/loiter_peak). RF HW 데이터 + pose → `rf/bearings` **P**(→emitter_fusion).

---

## 재활용 + BT + GCS

### SwarmFlockingBehavior (재활용 as2)
- **기능**: 편대 비행 실행.
- **역할**: N드론 지휘관(실코드 위상, 리더 상주).
- **책임**: `SwarmFlocking.action` **AS** 받음 → 드론별 `FollowReference` **AC(×N)** 발행. centroid_driver가 주는 TF 따라 편대 이동.

### bt_manager: SwarmCoord (리더 BT)
- **기능**: 전역 임무 조율 트리.
- **역할**: 리더 오케스트레이터.
- **책임**: mission_intent **S**, targets/emitters **S** → phase 전개. `PublishPhase`(mission_phase **P**), `SwarmFlockingStart` **AC**, `DriveCentroid`(centroid_driver), allocator 경유 task 분배. aggregator barrier로 phase 게이트.

### bt_manager: SwarmDrone (드론 BT)
- **기능**: 드론 로컬 실행 트리.
- **역할**: 드론 실행기.
- **책임**: mission_phase **S** + swarm_agent/task **S** 소비 → 비행/탐지 실행. `swarm/join` **SrvC**, `FollowReference`/`DetectObjects` **AC**. PhaseIs/WaitTakeoffSlot 게이트.

### gcs_mission_iface (C8, off-board)
- **기능**: 임무 생성·업로드·모니터.
- **역할**: 인간↔군집 게이트.
- **책임**: 임무 정의(타입/구역/클래스/편대) → `MissionIntent.msg` 발행 + `targets` **S** 모니터. fleet_manager(discovery/fleet_status) 재활용.

---

## 배치 원칙 (PACKAGE_ARCHITECTURE §5)

| 위치 | 노드 |
|---|---|
| per-drone 상주 | health_monitor, heartbeat, election, detect_objects, rf_survey, BT(SwarmCoord+SwarmDrone) |
| **리더만 활성** | registry, allocator, centroid_driver, detection_fusion, emitter_fusion, aggregator |
| off-board(GCS) | gcs_mission_iface, fleet_manager(재활용) |
| 리더 SwarmFlocking | SwarmFlockingBehavior |

> `IsLeader` 게이트 또는 리더에서만 spawn. 리더 교체 → 차순위 승계(SPOF 완화).

---

_근거: `AS2_SWARM_MODULE_MANIFEST.md`, `AS2_SWARM_PACKAGE_ARCHITECTURE.md`, `AS2_SWARM_INTERFACE_SPEC.md`, AS2 실코드(`as2_behaviors_swarm_flocking`, `as2_fleet_manager`)._
