# 군집 정찰 시스템 — 구현 모듈 매니페스트

> 전 설계문서에 흩어진 **신규 구현 대상**을 단일 카탈로그로 통합. 모듈별 패키지·유형·책임·I/O·파라미터·재활용 의존·마일스톤·상태.
> 근거: `AS2_SWARM_IMPLEMENTATION_WORKFLOW.md`, `AS2_SWARM_BOOTSTRAP_DESIGN.md`, `AS2_SWARM_BT_DISPATCH_DESIGN.md`, `AS2_RECON_SWARM_PHASED_DESIGN.md`, `AS2_RECON_BEHAVIOR_DESIGN.md`, AS2 실코드.

---

## 0. 요약

| 분류 | 신규 모듈 수 | 마일스톤 |
|---|---|---|
| C1 메시지/인터페이스 | 9 | A1 |
| C2 인식·페이로드 behavior (L5) | 2 | A2/C5 |
| C3 계획 (L5) | 2 | A3 |
| C4 융합 | 2 | C2/C5 |
| C5 swarm_agent 서비스 (L6) | 6 | B3/B4 |
| C6 커스텀 BT 노드 | 26 | B1/C1 |
| C7 BT 트리 (XML) | 6 (골격 존재) | B2 |
| C8 GCS | 1 | D1 |
| C9 barrier 집계 서비스 | 1 | B2 |
| **합계** | **~51 항목** | |

**재활용(무수정)**: AS2 substrate L0~L4, takeoff/go_to/land/follow_path/follow_reference/point_gimbal behavior, `swarm_flocking`(SwarmFlocking/DroneSwarm), `fleet_manager`, `as2_behavior_tree` 엔진+단일기 노드.

---

## C1. 메시지·인터페이스 (패키지: `as2_msgs` 확장 또는 신규 `aiss_swarm_msgs`)

| # | 이름 | 유형 | 필드(요지) | 생산자 | 마일스톤 |
|---|---|---|---|---|---|
| 1 | `DetectObjects.action` | action | goal{classes[],min_score} / result{status} / feedback{count} | detect_objects | A1 |
| 2 | `DetectionArray.msg` | msg | drone_id, [class,score,PoseStamped(world)] | detect_objects | A1 |
| 3 | `TargetTrack.msg` | msg | id, PoseStamped, class, first_seen, last_seen | detection_fusion | A1 |
| 4 | `EmitterTrack.msg` | msg | id, PoseStamped, freq, first_seen | emitter_fusion | A1 |
| 5 | `SwarmTask.msg` | msg | zone(Polygon), sub_route(Path), classes[], role, takeoff_slot, land_slot | allocator/orchestrator | A1 |
| 6 | `MissionPhase.msg` | msg | phase(enum), seq, stamp | PublishPhase | A1 |
| 7 | `MissionIntent.msg` | msg | type, area(Polygon), classes[], formation, constraints | gcs_mission_iface | A1 |
| 8 | `Heartbeat.msg` | msg | drone_id, alive, health(enum), stamp, position | heartbeat | A1 |
| 9 | `SwarmJoin.srv` | srv | req{id,health} / res{accepted,registry,mission_version} | registry | A1 |

재활용: `vision_msgs/Detection2DArray`(픽셀 bbox 옵션), `as2_msgs/{PoseWithID,PoseWithIDArray,SwarmFlocking,FollowReference,PlatformInfo}`.

---

## C2. 인식·페이로드 behavior (L5, 패키지: `as2_behaviors_perception` / 신규)

### C2.1 `detect_objects` — EO/IR 객체탐지 ★
| 항목 | 내용 |
|---|---|
| 패키지 | `as2_behaviors_perception/detect_objects_behavior` (신규) |
| 유형 | behavior (`BehaviorServer<DetectObjects>`) — `detect_aruco_markers_behavior` 템플릿 |
| 책임 | EO/IR 이미지 추론 → 객체 bbox+class → world 지오로케이션 |
| 입력 | `sensor_measurements/eo/{image_raw,camera_info}`, `sensor_measurements/ir/{image_raw,camera_info}` |
| 출력 | `perception/detections` (DetectionArray) |
| 파라미터 | model_path, classes[], min_score, eo/ir 융합 모드, camera intrinsics |
| 의존 | OpenCV, cv_bridge(perception pkg 기존), ONNXRuntime/TensorRT, `as2::sensors::Camera`×2, TF |
| 마일스톤 | A2 |

### C2.2 `rf_survey` — RF 스캔·측위
| 항목 | 내용 |
|---|---|
| 패키지 | `as2_behaviors_perception/rf_survey_behavior` (신규) |
| 유형 | behavior |
| 책임 | 스펙트럼 스캔 + DF(방향탐지) 기동 → 방위 산출 |
| 입력 | RF 수신기 데이터(HW의존), `self_localization/pose` |
| 출력 | `rf/bearings` (방위+주파수) |
| 파라미터 | freq_range, mode(triangulate/df/loiter_peak), scan_dwell |
| 의존 | RF HW 드라이버 |
| 마일스톤 | C5 |

---

## C3. 계획 (L5)

### C3.1 coverage planner — lawnmower 경로 plugin
| 항목 | 내용 |
|---|---|
| 패키지 | `as2_behaviors_path_planning/plugins/coverage` (신규, a_star/voronoi 옆) |
| 유형 | pluginlib plugin |
| 책임 | 폴리곤 sub-zone → boustrophedon(lawnmower) waypoint |
| 입력 | sub-zone 폴리곤, FOV/간격, 편대폭(군집) |
| 출력 | Path (waypoint) |
| 파라미터 | swath_width, overlap, altitude, heading |
| 마일스톤 | A3 |

### C3.2 zone_partitioner — 구역 분할
| 항목 | 내용 |
|---|---|
| 패키지 | `aiss_swarm_core/zone_partitioner` (신규 lib/노드) |
| 유형 | lib + 노드 |
| 책임 | 정찰구역 폴리곤 → N sub-zone (균등 면적/경로) |
| 입력 | 구역 폴리곤, N(드론수), capability |
| 출력 | sub-zone[] → `swarm_agent/task` |
| 파라미터 | 분할 방식(스트립/Voronoi), 균등 기준 |
| 마일스톤 | A3 |

---

## C4. 융합

### C4.1 detection_fusion — 다중기 탐지 융합 ★
| 항목 | 내용 |
|---|---|
| 패키지 | `aiss_swarm_perception/detection_fusion` (신규 노드) |
| 책임 | 전 드론 `detections` 수집 → world 좌표 dedup/병합 → 전역 표적 |
| 입력 | `/{ns}/perception/detections` ×N |
| 출력 | `/swarm/targets` (TargetTrack[]) |
| 파라미터 | dedup 거리임계, 트랙 수명(timeout), 병합 정책 |
| 마일스톤 | C2 |

### C4.2 emitter_fusion — RF 삼각측량 융합
| 항목 | 내용 |
|---|---|
| 패키지 | `aiss_swarm_perception/emitter_fusion` (신규 노드) |
| 책임 | 다기 방위(`rf/bearings`) → 삼각측량 → 방사원 위치 |
| 입력 | `/{ns}/rf/bearings` ×N |
| 출력 | `/swarm/emitters` (EmitterTrack[]) |
| 파라미터 | baseline 최소거리, 측위 알고리즘(LS/MLE) |
| 마일스톤 | C5 |

---

## C5. swarm_agent 서비스 (L6, 패키지: `aiss_swarm_core`)

BT 밖 백그라운드. 무거운 상태/계산/합의. BT는 결과 구독·조건 소비.

| # | 모듈 | 책임 | 입력 | 출력 | 마일스톤 |
|---|---|---|---|---|---|
| 1 | `health_monitor` | 자가진단(FCU/센서/TF/배터리) | platform/info, 센서, TF, 배터리 | health(OK/DEGRADED/FAIL) | B4 |
| 2 | `heartbeat` | heartbeat 발행+수신, 생존맵 | 로컬 health | `/swarm/heartbeat`, 생존맵 | B4 |
| 3 | `election` | 리더 선출(최저 id), 승계 | 생존맵 | `/swarm/leader_id` | B4 |
| 4 | `registry` | join 수락·등록부·정족수 | `swarm/join` | `/swarm/registry`, QuorumReady | B4 |
| 5 | `allocator` | 임무 분해·배정(분할/경로/역할), 재할당 | intent, registry, targets | `/{ns}/swarm_agent/task` | B4/C2 |
| 6 | `centroid_driver` | virtual_centroid 궤적 구동 | 목표점, 현 centroid | centroid TF / modify_swarm | B3 |

> `allocator`는 `zone_partitioner`(C3.2)+coverage(C3.1) 호출. 추적은 `AllocateTrackRoles` 반응형.
> `centroid_driver`는 `swarm_flocking`(SwarmFlocking 재활용) 위에서 centroid만 이동.

---

## C6. 커스텀 BT 노드 (패키지: `aiss_swarm_bt`)

`as2_behavior_tree` 팩토리에 등록. nodes_model.xml에 포트 선언됨(`swarm_bt/`).

### C6.1 군집 조율 (7)
| 노드 | 유형 | 호출 대상 |
|---|---|---|
| `HasMissionType` | Condition | 블랙보드 {mtype} |
| `LatchMissionType` | Decorator | mission_intent (타입 latch) |
| `PublishPhase` | Action | `/swarm/mission_phase` |
| `SwarmFlockingStart` | Action | SwarmFlocking.action (재활용) |
| `SwarmFlockingStop` | Action | deactivate/stopFollowReference |
| `DriveCentroid` | Action | centroid_driver |
| `ModifyFormation` | Action | modify_swarm |

### C6.2 동기 barrier (4) — 집계서비스 폴링
`AwaitAllAirborne`, `AwaitFormed`, `AwaitAllZonesDone`, `AwaitAllLanded` (전부 Condition)

### C6.3 per-drone 게이트/슬롯 (4)
`PhaseIs`(Cond), `WaitTakeoffSlot`(Decorator), `WaitLandSlot`(Decorator), `SafeHover`(Action)

### C6.4 임무 P3 전용 (8)
`DetectObjects`(Act), `ObjectDetected`(Cond), `InspectTimeout`(Cond), `RfSurvey`(Act), `EmitterDetected`(Cond), `HasRole`(Cond), `TargetLost`(Cond), 그리고 coord측 `PartitionAndAssign`/`AssignRfSurvey`/`FuseDetections`/`FuseEmitters`/`AllocateTrackRoles`(Act)

### C6.5 Phase 0 부트스트랩 (7)
`AwaitBoot`(Cond), `PublishHeartbeat`(Act), `ReportHealth`(Act), `SwarmJoin`(Act), `IsLeader`(Cond), `QuorumReady`(Cond), `MissionReady`(Cond)

> 합계 ~26 노드. 대부분 얇은 래퍼(서비스/액션 호출 또는 토픽 구독 조건). 마일스톤 B1(조율/barrier/게이트) + C1(부트스트랩).

---

## C7. BT 트리 (XML, 패키지: `aiss_swarm_bt/trees`) — 골격 존재

| 파일 | 내용 | 상태 |
|---|---|---|
| `swarm_bt/swarm_mission_root.xml` | coordinator 루트 + 4임무 Coord + P3Coord | ✅ 골격 |
| `swarm_bt/swarm_perdrone_root.xml` | per-drone 루트 + 4임무 Drone + P3Drone | ✅ 골격 |
| `swarm_bt/chassis/chassis_coord.xml` | Cx_* 서브트리 | ✅ |
| `swarm_bt/chassis/chassis_drone.xml` | Dx_* 서브트리 | ✅ |
| `swarm_bt/nodes_model.xml` | TreeNodesModel | ✅ |
| `swarm_bt/swarm_bt_combined.xml` | Groot2 통합본 | ✅ |
| `swarm_bootstrap_root.xml` | Phase0 게이트 + IsLeader 래퍼 | ◻ 미작성 |

> 부트스트랩 래퍼(SwarmRoot)는 `AS2_SWARM_BOOTSTRAP_DESIGN.md §5.1` 기준 신규 필요 — 기존 루트를 감쌈.

---

## C8. GCS

### `gcs_mission_iface` — 임무 생성·업로드
| 항목 | 내용 |
|---|---|
| 패키지 | `aiss_gcs/gcs_mission_iface` (신규, off-board) |
| 책임 | 임무 정의(타입/구역/클래스/편대) → `mission_intent` 발행 + 모니터 |
| 출력 | `/swarm/mission_intent` (MissionIntent) |
| 의존 | `fleet_manager`(discovery/fleet_status 재활용) |
| 마일스톤 | D1 |

---

## C9. barrier 집계 서비스

### `swarm_aggregator` — barrier 상태 집계
| 항목 | 내용 |
|---|---|
| 패키지 | `aiss_swarm_core/aggregator` (신규 노드) |
| 책임 | 전 드론 상태 수집 → barrier 판정값 발행 (BT `Await*` 입력) |
| 입력 | `/{ns}/platform/info`, `self_localization`, WORK 완료 신호, heartbeat |
| 출력 | airborne/formed/zonesdone/landed 집계 (토픽/서비스) |
| 파라미터 | 고도임계, checkPosition 임계(0.3m), barrier 타임아웃 |
| 마일스톤 | B2 |

---

## 패키지 구성 (제안) — 별도 오버레이 워크스페이스

AS2 트리 무수정. 전 신규 패키지는 `aiss_ws/src/aiss/`. (상세: `AS2_SWARM_PACKAGE_ARCHITECTURE.md` §0)

```
e:/Project/AISS.os/
├─ aerostack2_ws/src/aerostack2/   # [AS2, 무수정] 재활용
└─ aiss_ws/src/aiss/               # [신규 오버레이]
    ├─ aiss_swarm_msgs/            # C1 인터페이스
    ├─ aiss_swarm_core/            # C5 health/heartbeat/election/registry/allocator/centroid_driver
    │                              #    + C3.2 partitioner + C9 aggregator
    ├─ aiss_swarm_perception/      # C4 detection_fusion / emitter_fusion
    ├─ aiss_swarm_bt/              # C6 노드 + C7 트리
    ├─ aiss_gcs/                   # C8 gcs_mission_iface
    ├─ aiss_behaviors_perception/  # C2.1 detect_objects + C2.2 rf_survey (AS2 트리 밖)
    └─ aiss_coverage_planner/      # C3.1 coverage plugin (as2_behaviors_path_planning base 상속)
```

빌드: `aerostack2_ws`(베이스) build·source → `aiss_ws`(오버레이) build·source.

---

## 구현 우선순위 (워크플로우 연계)

| 순위 | 모듈 | 마일스톤 | 관문 |
|---|---|---|---|
| 1 | C1 메시지(9) | A1 | build |
| 2 | C2.1 detect_objects | A2 | 탐지 발행 |
| 3 | C3 coverage+partitioner | A3 | route 생성 |
| 4 | (단일기 박판 통합) | A4 | ★1 |
| 5 | C6 BT 노드 조율/barrier/게이트 | B1 | Groot2 |
| 6 | C7 chassis + C9 aggregator | B2 | 빈P3 순차 |
| 7 | C5 centroid_driver + 편대 | B3 | 편대 transit |
| 8 | C5 health/heartbeat/election/registry | B4 | 선출/정족수 |
| 9 | C6 부트스트랩 노드 + C7 래퍼 | C1 | ★2 |
| 10 | C4.1 detection_fusion + EO/IR P3 | C2 | ★3 군집정찰 |
| 11 | 감시 P3 | C4 | ★4 일반성 |
| 12 | C2.2 rf_survey + C4.2 emitter_fusion | C5 | RF |
| 13 | C6 AllocateTrackRoles + 추적 | C6 | 추적 |
| 14 | C8 gcs_mission_iface | D1 | GCS |

**즉시 착수**: C1(메시지) + C6 조율/barrier/게이트 노드 — 설계·포트 확정, 의존 최소.

---

_근거: `AS2_SWARM_IMPLEMENTATION_WORKFLOW.md`, `AS2_SWARM_BOOTSTRAP_DESIGN.md`, `AS2_SWARM_BT_DISPATCH_DESIGN.md`, `AS2_RECON_SWARM_PHASED_DESIGN.md`, `AS2_RECON_BEHAVIOR_DESIGN.md`, `swarm_bt/`, AS2 실코드(`as2_behaviors_swarm_flocking`, `as2_fleet_manager`, `as2_behaviors_perception`, `as2_behavior_tree`)._
