# 군집 조율 구현 Workflow / 태스크 관리

> **목적**: 설계(주력 B, BT 네이티브)를 구현 태스크로 분해·추적. 이 문서로 진행 관리.
> **설계 기준**: `swarm_bt_native_design.md`, `swarm_bt_node_reference.md`, `swarm_mission_example.xml`, `swarm_a_to_c2_migration.md`.
> **상태 범례**: ⬜ TODO / 🟧 WIP / ✅ DONE / ⛔ BLOCKED.

---

## 1. 사용법

- 각 태스크 = ID + 산출물 + 의존 + 수용기준 + 참조문서 + 상태.
- 상태는 작업 진행 시 갱신. 마일스톤 단위로 출하·검증.
- 범위 = **Phase 0(BT 네이티브 베이스라인)** 우선. C2 확장(P1~P3)은 `swarm_a_to_c2_migration.md`로 별도 관리.

---

## 2. 구현 원칙

| # | 원칙 | 근거 |
|---|---|---|
| 0 | **AS2 일체 무변경 (최우선)** | substrate(platform/estimator/controller/behaviors)뿐 아니라 **AS2 BT 노드·base class도 수정 금지**. 동작상 수정 필요 시 → `aiss_swarm_bt`에 **커스텀 노드/데코 신규 생성**해 대체 (AS2 파일 손대지 않음). |
| 1 | **AS2 substrate 무변경** | 정합성 (graft 분석) |
| 2 | **P0 먼저, 완성품 확보** | 점진 (a_to_c2) |
| 3 | **Day-1 불변식** (노드어휘/selector/version 배선/서브트리 모듈화/보존키) | C2 대비 (a_to_c2 §2) |
| 4 | **백그라운드 서비스 먼저 기동** | BtServiceNode 무한 wait 회피 (BT 분석 §17.4) |
| 5 | **단위→통합→시뮬** 순 검증 | 위험 분리 |

---

## 3. 마일스톤 개요

| MS | 이름 | 산출 | 의존 |
|---|---|---|---|
| **M0** | 인프라 | overlay 워크스페이스 + msg 패키지(B) + 빌드 | — |
| **M1** | 백그라운드 서비스 | election / registry / allocator | M0 |
| **M2** | BT 노드 | `aiss_swarm_bt` 15 커스텀 노드 + factory | M0 |
| **M3** | 트리·런치 | example.xml + bringup(순서보장) | M1, M2 |
| **M4** | 통합·시뮬 | gazebo 멀티드론 e2e 정찰 | M3 |
| **M5** | 다종·수단 | TRACK/SURVEIL + VISION/RF | M4 |
| **C-ext** | 동적트리 | P1~P3 (a_to_c2) | M5, 요구 시 |

---

## 3.5 패키지 레이아웃 (구현 위치)

**전 구현은 `e:\Project\AISS.os\aiss_ws\src\aiss\` 하위에 패키지별로.** (aerostack2_ws = underlay, 무변경)

```
aiss_ws/                                  ◀ OVERLAY (독립 git)
└─ src/aiss/
   ├─ aiss_swarm_msgs/        srv JoinRequest/Allocate, msg Heartbeat/Registry      [M0]
   ├─ aiss_swarm_core/        swarm_election / swarm_registry / swarm_allocator     [M1]
   ├─ aiss_swarm_bt/          15 BT 노드 (BtAction/BtService/Condition/Decorator)   [M2]
   ├─ aiss_swarm_perception/  vision_detect / rf_scan (+ RF sensor 인터페이스)       [M5]
   └─ aiss_bringup/           launch (서비스→bt_manager 순서) + swarm_mission_*.xml  [M3]
```

> 현재 `aiss_ws/src/aiss/` **비어있음** — 이전 생성된 `aiss_swarm_msgs`(설계 A판) 삭제됨. M0-2는 **B판으로 신규 생성**(A 정리 불필요).
> 빌드: `aerostack2_ws` source → `aiss_ws` colcon build (overlay 체이닝).

---

## 4. 작업 분해 (WBS)

### M0 — 인프라

| ID | 산출물 | 의존 | 수용기준 | 참조 | 상태 |
|---|---|---|---|---|---|
| M0-1 | `aiss_ws/src/aiss/` overlay (msgs 패키지 포함) | — | `colcon build` 통과, underlay(as2) source됨 | a_to_c2 | 🟧 (콜콘 빌드 미검증) |
| M0-2 | `aiss_swarm_msgs` 생성(B판) — msg Heartbeat/Health/DroneInfo/Registry, srv JoinRequest/Allocate | M0-1 | `ros2 interface list` 6종 노출 | node_ref §J | 🟧 파일 생성 완료, 빌드검증 대기 |
| M0-4 | `Allocate.srv` resp = {accepted, mtype, modality, role(string), route[], zone} | M0-2 | 3축(type/modality/role) string 배정 (D2/D3 반영) | intent §12 | ✅ |

### M1 — 백그라운드 서비스 (`aiss_swarm_core`)

| ID | 산출물 | 의존 | 수용기준 | 참조 | 상태 |
|---|---|---|---|---|---|
| M1-1 | `swarm_election` (heartbeat 5Hz, lowest-alive-id) | M0-2 | 3드론 기동 시 최저 id가 `swarm/leader_id` 발행 | node_ref A | ⬜ |
| M1-2 | election 재선출 | M1-1 | 리더 kill → 차순위 승계 < T1, registry 캐시 인수 | bt_native §10-4 | ⬜ |
| M1-3 | `swarm_registry` (Join 수락, quorum, version sync) | M0-2 | Join N개 → `swarm/registry`, quorum_ok 판정 | node_ref A | ⬜ |
| M1-4 | `swarm_allocator` (의도분해: Voronoi/패턴/경로, capability-aware) | M0-4 | Allocate 요청 → 3축+route 반환, 능력↔수단 매칭 | intent §4,§12 | ⬜ |
| M1-5 | allocator 재분해 (표적/손실/재지정) | M1-4 | 이벤트 시 해당 드론 갱신값 반환 | intent §11,§12 | ⬜ |

### M2 — BT 노드 (`aiss_swarm_bt`)

| ID | 산출물 | 의존 | 수용기준 | 참조 | 상태 |
|---|---|---|---|---|---|
| M2-1 | 패키지 + factory 등록 골격 (bt_action_node/bt_service_node 상속) | M0-2 | 빈 노드 register + bt_manager 로드 | BT 분석 §18 | ⬜ |
| M2-2 | Condition 6: IsLeader/QuorumReady/HasMissionType/HasModality/HasRole/EvalTermination | M2-1 | mock 입력으로 S/F 단위검증 | node_ref B | ⬜ |
| M2-3 | Action(srv) 2: SwarmJoin/RequestAllocation (+블랙보드 출력) | M2-1,M1-3,M1-4 | mock srv로 S/R/F + bb 적재 확인 | node_ref C | ⬜ |
| M2-4 | FollowReference/GatherRegistry/AllocateTasks (action 래핑) + SwarmFormation(분산 편대 pub `/swarm/formation`) | M2-1 | mock action R→S halt-cancel; SwarmFormation 발행 확인 | node_ref D, D8 | ⬜ |
| M2-5 | SyncAction ReportHealth + RfSurvey(RF 기동) | M2-1 | 발행/기동 확인 | node_ref D,E | ⬜ |
| M2-6 | Decorator: WaitForLeaderLost(리더상실 시 halt+F), WaitForSwarmCmd | M2-1 | 이벤트 전 R, 수신 시 자식 / 상실 시 F | node_ref F, I#3 | ⬜ |
| M2-7 | FollowPath 포트 N점 일반화 패치 (현 2점 한계) | M2-1 | 3+ waypoint 파싱 | BT 분석 §17.2 | ⬜ |

### M3 — 트리·런치

| ID | 산출물 | 의존 | 수용기준 | 참조 | 상태 |
|---|---|---|---|---|---|
| M3-1 | `swarm_mission_example.xml` 배포 + Groot2 로드 | M2-* | 트리 파싱 OK, Groot2 시각화 | example.xml | ⬜ |
| M3-2 | bringup launch: **서비스 3 → bt_manager** 순서보장 | M1-*,M3-1 | 단일드론 부팅, 무한wait 없음 | workflow §2-4 | ⬜ |
| M3-3 | 블랙보드 보존키 규약(node/home/leader_id) 적용 | M3-2 | 키 초기화·보존 확인 | a_to_c2 §2 | ⬜ |
| M3-4 | version_hash 배포·비교 배선 (정적이라도) | M1-3 | mission_version 동기확인 | a_to_c2 §2 (Day-1) | ⬜ |

### M4 — 통합·시뮬

| ID | 산출물 | 의존 | 수용기준 | 참조 | 상태 |
|---|---|---|---|---|---|
| M4-1 | gazebo 멀티드론(3+) 런치 + 네임스페이스 | M3-2 | 3드론 각자 트리 tick, 선출 수렴 | bt_native §10 | ⬜ |
| M4-2 | e2e 정찰: GCS intent → 분해 → 이륙 → 커버리지 | M4-1,M1-4 | AO 분할·경로추종·편대 확인 | intent §4 | ⬜ |
| M4-3 | 단절 시나리오: GCS off / 리더 kill | M4-2 | 임무 무중단, 재선출, 자율폴백(AutonomousCached) | bt_native §10-4 | ⬜ |
| M4-4 | 안전 선점: AlertEvent → Emergency | M4-2 | halt-cancel → Emergency 진입 | node_ref F | ⬜ |

### M5 — 다종·수단

| ID | 산출물 | 의존 | 수용기준 | 참조 | 상태 |
|---|---|---|---|---|---|
| M5-1 | TRACK/SURVEIL 종류 e2e | M4-2 | HasMissionType 분기, 종류별 동작 | example §종류 | ⬜ |
| M5-2 | per-drone 종류 이질성(re-task) | M5-1,M1-5 | drone3만 SURVEIL 전환, 편대 detach | intent §11 | ⬜ |
| M5-3 | VISION 정찰 (카메라+gimbal+vision_detect) | M4-2 | EO/IR 커버리지+탐지 | intent §12 | ⬜ |
| M5-4 | RF 정찰 (신규 sensor 인터페이스 + rf_scan + RfSurvey) | M2-5 | 스펙트럼 스캔, emitters 출력 | intent §12 | ⬜ |
| M5-5 | capability-aware 할당 (Health 능력필드) | M1-4 | 능력↔수단 매칭 배정 | intent §12 | ⬜ |

---

## 5. 빌드 의존 그래프

```
as2(underlay) ─┬─ aiss_swarm_msgs(M0) ─┬─ aiss_swarm_core(M1: election/registry/allocator)
               │                        └─ aiss_swarm_bt(M2: BT 노드)
               │                                 │
               └─────────────── bringup(M3) ◀────┘  (서비스→bt_manager 순서)
                                     │
                                gazebo 통합(M4) → 다종·수단(M5)
신규 sensor: aiss_swarm_perception(vision_detect/rf_scan) + RF 인터페이스 ── M5
```

---

## 6. 테스트·검증 전략

| 레벨 | 대상 | 방법 |
|---|---|---|
| 단위 | BT 노드 | mock action/srv로 tick S/F/R (AS2 node_emulators 패턴 재사용) |
| 단위 | 백그라운드 서비스 | ros2 srv/topic 직접 호출, 선출·할당 로직 |
| 통합 | 트리+서비스 | 단일드론 부팅, 블랙보드 흐름 |
| 시뮬 | 멀티드론 | gazebo 3+, e2e 정찰·단절·안전 |
| 정적 | XML | 파싱+참조무결성 (이미 통과: well-formed/dangling/orphan NONE) |

---

## 7. 리스크 / 블로커

| # | 리스크 | 대응 | 상태 |
|---|---|---|---|
| 1 | BtServiceNode 생성자 무한 wait (클라이언트가 서버 뜰 때까지 블로킹; 서버측 요청대기와 무관) | **1차=런치순서**(서버 선기동, M3-2)로 미발생. 커스텀 타임아웃 base는 옵션(강건화). AS2 무수정 | ⬜ |
| 2 | TakeoffAction 500ms sleep 블로킹 → 안전 Parallel 동결 | **커스텀 `SwarmTakeoff`** 노드(sleep 없이 Takeoff action wrap). AS2 TakeoffAction 무수정 + 안전 FCU 병행 | ⬜ |
| 3 | FollowPath 2점 한계 | **커스텀 `SwarmFollowPath`** 노드(다점 port 파서). AS2 FollowPathAction 무수정 | ⬜ |
| 4 | RF AS2 표준 없음 | M5-4 신규 sensor 인터페이스 | ⬜ |
| 5 | per-drone mtype (전역 가정) | Allocate per-drone 필드 (M0-4) | ⬜ |
| 6 | WaitForLeaderLost 커스텀 시맨틱 | leader-lost 시 halt+F (M2-6) | ⬜ |
| 7 | **🔴 평범 Fallback 비반응형 → 동적 재선택 불가** | selector를 **ReactiveFallback+ReactiveSequence**로 (예제 XML 반영됨) | ✅ XML 반영 |
| 8 | **🔴 WaitForEvent/Alert 다중tick 자식 버림(§11.1)** | 데코 3종 신규 구현(자식 RUNNING 유지) (M2-6) | ⬜ |
| 9 | action 서버 생성자 throw | 런치순서 엄격화 — behavior 서버 전 기동(M3-2) | ⬜ |
| 10 | ~~SwarmFlocking 단일노드 ↔ 선출 리더~~ **해소** | **분산 편대(옵션3)**: 리더 `/swarm/formation` 발행 → 각 드론 FollowReference. swarm_flocking 미사용 | ✅ 결정 |
| 11 | `/swarm/*` 전역 ↔ 드론 ns 혼재 | swarm/ 절대토픽명 강제 | ⬜ |

---

## 8. 진행 현황 (롤업)

| 마일스톤 | 태스크 | 완료 |
|---|---|---|
| M0 인프라 | 3 | 0 |
| M1 서비스 | 5 | 0 |
| M2 BT 노드 | 7 | 0 |
| M3 트리·런치 | 4 | 0 |
| M4 통합·시뮬 | 4 | 0 |
| M5 다종·수단 | 5 | 0 |
| **합계** | **28** | **0** |

> 갱신 규칙: 태스크 상태 변경 시 §4 표 + §8 롤업 동시 갱신.

---

## 9. 다음 액션 (즉시 착수 가능)

1. **M0-1** `aiss_ws/src/aiss/` overlay 초기화 (현재 빈 디렉토리).
2. **M0-2** `aiss_swarm_msgs` 신규 생성 (B판: JoinRequest/Allocate/Heartbeat/Registry).
3. **M2-1** `aiss_swarm_bt` 패키지 + factory 골격.

> `aiss_ws/src/aiss/`는 현재 비어있음 (이전 A판 aiss_swarm_msgs 삭제됨). M0-2 = B판 신규 생성.
</content>
