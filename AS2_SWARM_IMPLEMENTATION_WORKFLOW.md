# 군집 정찰 시스템 — 구현 워크플로우 (마스터)

> `AS2_SWARM_FULL_DIAGRAM.md`의 전체 동작 구조를 **빌드 가능한 작업 순서**로 환원. 무엇을 어떤 순서·의존으로 만들고 어디서 검증하는지.
> 원칙: AS2 substrate(L0~L5 대부분)는 재활용 → **신규는 L6 swarm_agent + 정찰 behavior + chassis BT**에 집중. 아래→위(substrate→조율) + 좌→우(단일기→군집).

---

## 0. 빌드 전략

| 원칙 | 내용 |
|---|---|
| **재활용 우선** | L0~L4 substrate, takeoff/goto/land/follow_path/follow_reference/point_gimbal/swarm_flocking = 무수정 재활용. 손대지 않음 |
| **신규 격리** | 신규 = `as2_msgs`(메시지) + `detect_objects`/coverage(L5) + `swarm_agent`(L6) + chassis/P3 BT + 집계서비스 |
| **수직 박판(thin vertical slice) 먼저** | 단일기 정찰 1줄(센서→탐지→비행)을 먼저 끝까지 → 그 다음 군집 폭 확장 |
| **검증 게이트** | 각 마일스톤은 Sim 검증 통과해야 다음 진행 |
| **병렬 트랙** | 메시지·detect_objects·swarm_agent는 초기 병렬 |

---

## 1. 작업 분해 (다이어그램 layer 매핑)

| Track | 다이어그램 §  | 컴포넌트 | 신규/재활용 |
|---|---|---|---|
| **T0 메시지** | §7 | `as2_msgs`: DetectObjects.action, DetectionArray, TargetTrack, SwarmTask, MissionPhase, Heartbeat, Health, MissionIntent | 신규 |
| **T1 인식(L5)** | §6,§7 | `detect_objects` behavior(EO/IR), `rf_survey`(RF) | 신규 |
| **T2 계획(L5)** | §6 | coverage planner plugin, zone_partitioner | 신규 |
| **T3 융합** | §7,§8 | detection_fusion, emitter_fusion | 신규 |
| **T4 swarm_agent(L6)** | §1,§4,§8 | heartbeat·health·election·registry·join | 신규 |
| **T5 편대 구동** | §5,§7 | centroid_driver (SwarmFlocking 위) | 신규 |
| **T6 BT 노드** | §5 | 커스텀 BT 노드(조율/barrier/게이트/Phase0) | 신규 |
| **T7 BT 트리** | §5,§6 | chassis(Cx_/Dx_) + 4임무 P3 + 부트스트랩 루트 | 신규(XML 골격 존재) |
| **T8 GCS** | §1,§4 | gcs_mission_iface (fleet_manager 재활용) | 일부 신규 |
| **R  substrate** | §1 | platform/estimator/controller/behaviors/swarm_flocking | 재활용 |

---

## 2. 의존 그래프 (DAG)

```
T0 메시지 ──┬─▶ T1 인식 ──┐
            ├─▶ T3 융합 ◄─┘
            ├─▶ T4 swarm_agent ──┐
            └─▶ T6 BT 노드 ──────┤
T2 계획 ───────────────────────┐ │
T5 centroid_driver(◄SwarmFlocking 재활용)─┐
                                │ │       │
                                ▼ ▼       ▼
                          T7 BT 트리 (chassis+P3+부트스트랩)
                                │
                                ▼
                          통합 Sim (단일기→군집→4임무)
T8 GCS ─────────────────────────┘ (병렬, 후기 연결)
```

핵심 경로: **T0 → (T1·T4·T6 병렬) → T7 → 통합**. T2/T3/T5는 T7 직전 합류. T8은 후기.

---

## 3. 마일스톤 (검증 게이트 포함)

### 단계 A — 기반 (수직 박판)
| M | 범위 | 산출 | 검증 게이트 |
|---|---|---|---|
| **A1** | T0 메시지·액션 | DetectObjects.action + 6 msg | `colcon build` 통과 |
| **A2** | T1 detect_objects | aruco 템플릿→EO/IR 2-카메라→더미 detector | 1기 `perception/detections` 발행 (Sim) |
| **A3** | T2 coverage + partitioner | plugin + 분할 lib | 폴리곤→N route 시각화 |
| **A4** | 단일기 정찰 박판 | `recon_mission_example.xml` + detect_objects | **1기 takeoff→coverage→탐지→inspect→복귀→land (Sim)** ★1차 관문 |

### 단계 B — 군집 조율
| M | 범위 | 산출 | 검증 게이트 |
|---|---|---|---|
| **B1** | T6 BT 노드 골격 | HasMissionType/PhaseIs/PublishPhase/SwarmFlockingStart/Await*/Drive* + 등록 팩토리 | nodes_model 로드, Groot2 unknown 0 |
| **B2** | T7 chassis | Cx_*/Dx_ 서브트리 + barrier 집계서비스 | 빈 P3로 전 단계 순차 진행 (2~3기 Sim) |
| **B3** | T5 centroid_driver + 편대 | SwarmFlocking 재활용 + centroid 구동 | N기 편대 transit(centroid 추종) Sim |
| **B4** | T4 swarm_agent | heartbeat·health·election·registry·join | N기 지상 선출·정족수, leader_id 안정 |

### 단계 C — 통합·임무 확장
| M | 범위 | 산출 | 검증 게이트 |
|---|---|---|---|
| **C1** | 부트스트랩 BT 통합 | SwarmRoot(IsLeader 게이트) + Phase0 노드 | **부팅→선출→정족수→개시 (지상 standby→자율 P1) Sim** ★2차 관문 |
| **C2** | EO/IR 정찰 군집 | EoirReconP3Coord/Drone + T3 detection_fusion | **N기 분할 정찰+탐지융합 풀 미션 (P1~P5) Sim** ★3차 관문 |
| **C3** | 모드전환 안전 | DISBAND/REGROUP deconfliction + latch + barrier 타임아웃 | 편대⇄독립 천이 충돌 0 (Sim 반복) |
| **C4** | 2번째 임무(감시) | SurveilP3Coord/Drone (chassis 무수정) | **chassis 무수정 + P3 2서브트리 추가만으로 동작** ★일반성 입증 |
| **C5** | RF 정찰 | rf_survey + emitter_fusion + RfReconP3 | 편대 baseline 측위 Sim |
| **C6** | 추적 | AllocateTrackRoles + HasRole 핸드오프 | 표적 핸드오프 반응전환 Sim |

### 단계 D — GCS·실기
| M | 범위 | 산출 | 검증 게이트 |
|---|---|---|---|
| **D1** | T8 GCS | gcs_mission_iface(intent 업로드) + fleet_manager 모니터 | GCS 업로드→임무 개시, GCS 단절 생존 |
| **D2** | 실기 연동 | EO/IR HW, MAVLink platform, 실 FCU | HIL → 야외 단일기 → 군집 |

---

## 4. 검증 관문 (게이트) 4개

```
★1 (A4): 단일기 정찰 수직 박판 — 센서→탐지→비행→복귀 일주
★2 (C1): 부트스트랩 — 부팅→선출→정족수→자율 개시
★3 (C2): 군집 EO/IR 정찰 풀 미션 — P1~P5 + 탐지융합
★4 (C4): 일반성 — 2번째 임무가 chassis 무수정 + P3 2파일로 동작
```
각 관문 미통과 시 다음 단계 진입 금지.

---

## 5. 병렬 트랙 편성

```
스프린트1: A1(메시지) ─┬─ A2(detect_objects) 시작
                       └─ B1(BT 노드) 시작        ← 병렬
스프린트2: A3(coverage) + A4(단일기 박판=★1)
스프린트3: B2(chassis) + B3(편대) + B4(swarm_agent)  ← 병렬
스프린트4: C1(부트스트랩=★2) → C2(군집 정찰=★3)
스프린트5: C3(모드전환) + C4(감시=★4)
스프린트6: C5(RF) + C6(추적)
스프린트7: D1(GCS) → D2(실기)
```

---

## 6. 산출물 추적 (현재 상태)

| 산출물 | 상태 |
|---|---|
| 설계: 정찰 behavior / phased / BT dispatch / 부트스트랩 / 다이어그램 | ✅ 완료 |
| BT XML 골격(`swarm_bt/` 5+통합본) | ✅ 골격 (노드 미구현) |
| `recon_mission_example.xml` | ✅ 단일기 |
| 토픽 ICD | ✅ |
| T0 메시지 | ◻ 미착수 (A1) |
| T1 detect_objects | ◻ (A2) |
| T6 BT 노드 C++ | ◻ (B1) |
| T4 swarm_agent | ◻ (B4) |

→ **다음 실착수 = A1(메시지) + B1(BT 노드 C++ 골격)** 병렬. 둘 다 설계·포트 확정돼 바로 코딩 가능.

---

## 7. 리스크 게이트 (다이어그램 §9 연계)

| 리스크 | 관문 | 완화 |
|---|---|---|
| 모드전환 충돌(편대⇄독립) | ★3 직전 C3 | 고도층+순번+최근접 슬롯 매칭, Sim 반복 |
| split-brain 2리더 | ★2 C1 | 결정적 최저id + 정족수 확인 |
| barrier 교착 | ★2,★3 | 타임아웃 + 낙오기 제외 |
| detection 중복계수 | ★3 C2 | world dedup + 트랙 수명 |
| EO/IR 지오로케이션 오차 | ★1 A2 | 지형고도+짐벌자세 보정 |

---

## 8. 결론

- **빌드 순서**: T0 메시지 → (인식·BT노드·swarm_agent 병렬) → chassis/편대 → 부트스트랩 통합 → 군집 정찰 → 임무 확장 → GCS/실기.
- **4 관문**: 단일기 박판(★1) → 부트스트랩(★2) → 군집 정찰(★3) → 일반성(★4).
- **즉시 착수**: A1(`as2_msgs` 메시지/액션) + B1(커스텀 BT 노드 C++) — 설계 확정, 의존 없음.
- **일반성 입증점**: ★4(C4) — 감시 임무가 chassis 무수정 + P3 2서브트리로 동작하면 프레임워크 성립.

---

_근거: `AS2_SWARM_FULL_DIAGRAM.md`(구조), `AS2_SWARM_BOOTSTRAP_DESIGN.md`(Phase0), `AS2_SWARM_BT_DISPATCH_DESIGN.md`(BT), `AS2_RECON_SWARM_PHASED_DESIGN.md`/`AS2_RECON_BEHAVIOR_DESIGN.md`(정찰), `swarm_bt/`(XML 골격)._
