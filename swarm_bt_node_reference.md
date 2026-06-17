# BT-네이티브 (설계 B) 노드 책임·IO 레퍼런스

> **기준**: `swarm_bt_native_design.md`(주력) + `swarm_mission_example.xml`.
> **범위**: 군집 조율 BT 노드 + 백그라운드 서비스의 책임·입출력·tick 결과(S/F/R).
> tick 결과 표기: **S**=SUCCESS, **F**=FAILURE, **R**=RUNNING. bb=블랙보드.

---

## A. 백그라운드 서비스 (3, BT 아님 — 상시 동작)

BT tick과 독립. BT 리프가 구독/호출하는 상태·계산 제공.

| 노드 | 책임 | 구독(in) | 발행/서비스(out) |
|---|---|---|---|
| `swarm_election` | lowest-alive-id 선출, heartbeat | `swarm/heartbeat`(전 드론) | pub `swarm/leader_id`, `swarm/heartbeat`(5Hz) |
| `swarm_registry` | Join 수락·quorum·version sync | `{drone}/health`, JoinRequest | srv `swarm/join`, pub `swarm/registry`, `swarm/mission_version` |
| `swarm_allocator` | **의도 분해·할당** (Voronoi/패턴/경로 생성) | registry, intent, detections | srv `swarm/allocate`(→ role/zone/route/type), pub `swarm/mission_type` |

> `swarm_allocator`는 GCS 의도를 구체 배정으로 변환하는 핵심 엔진 (`swarm_intent_processing.md` 참조).

---

## B. BT 노드 — Condition (조건, RUNNING 없음)

| ID | 책임 | 입력 | 출력 | 결과 |
|---|---|---|---|---|
| `IsLeader` | 자신이 리더인지 | sub `swarm/leader_id` | bb `leader_id` | S=리더 / F=팔로워 |
| `QuorumReady` | 정족수 충족? | sub `swarm/registry` | — | S=등록≥min_quorum / F=미달 |
| `HasMissionType` | 임무종류 매칭 | port `type`({mtype} **per-drone** string), `match`(RECON/TRACK/SURVEIL) | — | S=일치 / F=불일치 |
| `HasRole` | 역할 매칭 | port `role`({role} string), `match`(**제네릭 4종** SCOUT/TRACKER/RELAY/STANDBY) | — | S=일치 / F=불일치 |
| `HasModality` | 정찰 수단 매칭 | port `modality`({modality} string), `match`(VISION/RF) | — | S=일치 / F=불일치 |
| `EvalTermination` | 종료조건 평가 | port `coverage_target`,`batt_min`; sub coverage/batt/time | — | **S=계속 / F=종료** (KeepRunning이 F서 중단) |
| `IsFlying` (재사용) | 비행 중? | sub `platform/info` | — | S=FLYING / F=지상 |

---

## C. BT 노드 — Action (Service 래핑)

| ID | 책임 | 입력 | 출력 | 결과 |
|---|---|---|---|---|
| `SwarmJoin` | 군집 등록 | port `service_name`; health | srv `swarm/join` 응답 version | S=수락 / F=거부·timeout |
| `RequestAllocation` | 할당 수신 → 블랙보드 | port `service_name` | **bb `type`/`modality`/`role`/`route`/`zone`** (전부 string, per-drone) | S=수신 / R=대기 / F=timeout |
| `AllocateTasks` (리더) | 할당 계산·배포 트리거 | registry/intent | srv `swarm/allocate` 호출 | S=배포완료 / F=실패 |
| `Arm` / `Offboard` (재사용) | 무장 / 오프보드 | port `service_name` | srv SetBool | S=success / F |

---

## D. BT 노드 — Action (Action 래핑, 장시간 → RUNNING)

| ID | 책임 | 입력 | 출력 | 결과 |
|---|---|---|---|---|
| `SwarmFormation` (리더, SyncAction) | **분산 편대**: 형태→centroid+오프셋 계산 후 발행 | port `formation`,`spacing` | pub `/swarm/formation`(`PoseStampedWithIDArray`) | 발행 후 S |
| `FollowReference` | 편대 오프셋 추종 | port `server_name` | action `FollowReferenceBehavior` | R=추종중 / S / F |
| `GatherRegistry` (리더) | Join 수집·registry 구성 | swarm/join 결과 | registry 갱신 | R=수집중 / S=정족수 도달 |
| `RfSurvey` | RF 정찰 특화 기동+스캔 (삼각측량/DF/peak loiter) | port `route`,`mode` | RF 측위, pub emitters | R=측위중 / S / F |
| `SwarmTakeoff` (**커스텀**) | 이륙. AS2 TakeoffAction의 on_success **500ms sleep 제거** | port `height`,`speed` | action `TakeoffBehavior` | R / S / F |
| `SwarmFollowPath` (**커스텀**) | 경로 추종. AS2 **2점 한계 회피**(다점 port 파서) | port `path`,`speed` | action `FollowPathBehavior` | R / S / F |
| `PointGimbal`/`GoTo`/`Land` (재사용) | 카메라 지향/이동/착륙 | port pose/...; gimbal | AS2 behavior action | R=수행 / S=완료 / F=abort |

---

## E. BT 노드 — SyncAction (즉시)

| ID | 책임 | 입력 | 출력 | 결과 |
|---|---|---|---|---|
| `ReportHealth` | 자가진단 발행 | platform/info, imu, gps, batt | pub `{drone}/health` | 항상 S |

---

## F. BT 노드 — Decorator (이벤트 대기·감시)

| ID | 책임 | 입력 | 동작 | 결과 |
|---|---|---|---|---|
| `WaitForLeaderLost` (신규) | 리더 heartbeat timeout 감시 (자식 통과 게이트) | port `topic_name`(swarm/heartbeat) | 정상 → 자식 tick / 리더상실+timeout → 자식 halt | **정상=자식 결과 / 리더상실 timeout=F** (ReactiveFallback이 F서 AutonomousCached 폴백) |
| `WaitForSwarmCmd` (신규) | 리더 명령 대기 | port `topic_name`(swarm/cmd) | cmd 전 R, 수신 후 **자식 RUNNING 동안 계속 tick** | R → 자식 결과 |
| `WaitForAlert` (재사용 ⚠️수정필요) | AlertEvent 대기·선점 | port `topic_name`; out `alert` | alert 전 R, 수신 → 자식(Emergency, **다중tick 유지**) | R → 자식 결과 |

> ⚠️ **AS2 `WaitForEvent`/`WaitForAlert` 그대로 재사용 불가**: 원 구현은 이벤트 후 flag 즉시 리셋 → **다중-tick 자식(RUNNING) 버림**(분석 §11.1). 세 데코 다 **자식 RUNNING 동안 계속 tick하도록 신규/수정** 구현 필요. WaitForSwarmCmd는 "WaitForEvent 재사용" 아님 — 신규.

---

## G. 제어노드 (BT.CPP 내장, 참고)

커스텀 아님. 사용처만:

| 노드 | 사용처 |
|---|---|
| `Parallel(s,f)` | 루트(안전∥조율), 리더(조율∥비행), ScoutMission(편대∥경로) |
| `Fallback` | IsLeader 분기, 종류/역할 selector |
| `Sequence` / `SequenceStar` | 단계 게이트 |
| `SubTree` | 서브트리 호출 (`__shared_blackboard`) |
| `KeepRunningUntilFailure` | EvalTermination 폴링 루프 |

---

## H. 블랙보드 데이터흐름

| 변수 | writer 노드 | reader 노드 | 갱신 |
|---|---|---|---|
| `leader_id` | IsLeader | (분기 판정) | heartbeat마다 |
| `type` (mtype) | RequestAllocation | HasMissionType | 할당/재지정 |
| `modality` | RequestAllocation | HasModality | 할당/재지정 (VISION/RF) |
| `role` | RequestAllocation | HasRole | 할당/재지정 |
| `route` / `zone` | RequestAllocation | FollowPath/GoTo/Scout/AutonomousCached | 할당/replan |
| `track_target` | (TRACK 갱신원) | GoTo | 표적 추적 시 |
| `home` | 부팅 1회 | Emergency/AutonomousCached | 고정 |
| `node`(ROS2) | bt_manager | 전 BT 노드 | 부팅 1회 |

---

## I. 구현 주의

| # | 항목 | 내용 |
|---|---|---|
| 1 | `RequestAllocation` 포트 방향 | 예제는 `type="{mtype}"` 표기 — 실제 = **output**(서비스 응답→블랙보드). TreeNodesModel엔 output_port. 구현 시 방향 확정 |
| 2 | `EvalTermination` 결과 반전 | KeepRunningUntilFailure 아래라 **S=계속 / F=종료**. 직관과 반대, 명시 |
| 3 | `WaitForLeaderLost` 폴백 | 표준 Fallback 패턴 사용: `Fallback{ WaitForLeaderLost(정상임무), AutonomousCached }`. 데코는 **리더상실+timeout 시 자식 halt 후 FAILURE 반환**(정상은 자식결과 통과) → Fallback이 AutonomousCached로 폴백. 데코 자체는 서브트리 직접호출 안 함 |
| 4 | 리더 전용 노드 | `SwarmFormation`/`AllocateTasks`/`GatherRegistry`는 IsLeader 가지 안에서만 tick |
| 5 | per-drone `type` (D2 결정) | 블랙보드 `type` 권위 = **per-drone `Allocate` 응답 필드**(전역 `/swarm/mission_type`은 모니터링 default). 균질=전원 동일값, 이질=일부 override → 단일 메커니즘. EvalTermination은 **task-group별** 평가 |
| 13 | 역할/종류/수단 = string (D3 결정) | enum 강제 X, `Allocate`가 string 반환. **제네릭 역할 4종** SCOUT/TRACKER/RELAY/STANDBY (종류별 특수명 WATCHER→SCOUT, RELIEF→RELAY 흡수). string이라 C2/신규종류 시 확장 자유 |
| 6 | BtServiceNode 생성자 무한 wait | `SwarmJoin`/`RequestAllocation`은 서비스 **클라이언트** — 생성자가 대상 **서버 뜰 때까지** 블로킹(§17.4, 서버측 요청대기와 무관). **1차 해결=런치순서**(서버 선기동)로 미발생. 커스텀 타임아웃 base는 옵션 강건화. AS2 무수정 |
| 7 | **반응형 selector 필수** | 종류/수단/역할/리더 selector = **`ReactiveFallback`+`ReactiveSequence`**. 평범한 Fallback/Sequence는 RUNNING 자식서 재개해 **조건 재평가 안 함** → 동적 재지정·리더교체 작동 안 함 (예제 XML 반영됨) |
| 8 | **데코 신규 구현** | WaitForAlert/SwarmCmd/LeaderLost = AS2 WaitForEvent 재사용 불가(다중tick 자식 버림, 분석 §11.1). 자식 RUNNING 유지하도록 신규 |
| 9 | **TakeoffAction 500ms sleep** | BT 루프 블로킹(분석 §17.1) → 이륙 순간 안전 Parallel 동결. sleep 우회 패치 or 안전 FCU failsafe 병행 |
| 10 | **action 서버 생성자 대기** | BtActionNode ctor `wait_for_action_server(1s)` throw — SwarmFormation/FollowReference 등 대상 서버가 트리 로드 전 다 떠있어야. 런치순서 엄격화 |
| 11 | ~~SwarmFlocking 호스팅~~ **해소(옵션3)** | AS2 `swarm_flocking`(단일 고정노드+Static TF 권위) ↔ 이동 리더 충돌 → **분산 편대로 대체**. 리더가 `/swarm/formation` 발행, 각 드론 FollowReference 추종. swarm_flocking 미사용(존치). 편대권위=리더와 동행 |
| 12 | **`/swarm/*` 절대토픽** | 전역 스코프라 드론 네임스페이스 bt_manager서 **절대경로** 강제 (platform/info는 ns 상대). 혼재 주의 |

---

## J. 노드 수 요약

- **커스텀 신규 BT 노드**: 15 (Condition 6: IsLeader/QuorumReady/HasMissionType/HasModality/HasRole/EvalTermination, Action 7: SwarmJoin/RequestAllocation/AllocateTasks/SwarmFormation/FollowReference/GatherRegistry/RfSurvey, SyncAction 1: ReportHealth, Decorator 1: WaitForLeaderLost)
- **재사용(기존/패턴)**: WaitForSwarmCmd(=WaitForEvent), WaitForAlert → 합산 시 17
- **커스텀 래퍼**(AS2 무수정, aiss가 대체 노드 신규): `SwarmTakeoff`(sleep 제거), `SwarmFollowPath`(다점 파서), `WaitForAlert`(다중tick). [옵션] aiss service-node base(생성자 타임아웃, 강건화용)
- **재사용 AS2 액션**(문제없음 그대로): IsFlying/Arm/Offboard/Land/GoTo/FollowReference/PointGimbal (factory 등록)
- **원칙**: AS2 일체 무수정. 문제 있는 AS2 BT 노드(TakeoffAction sleep, FollowPath 2점, WaitForEvent 버그, BtServiceNode 무한wait)는 **커스텀으로 대체**, AS2 파일은 손대지 않음.
- **백그라운드 서비스**: 3 (election/registry/allocator)
- **인식 노드(배경, modality별)**: vision_detect(카메라→bbox), rf_scan(스펙트럼→emitters)

> 정찰 selector 계층: **종류(RECON) → 수단(VISION/RF) → 역할(SCOUT/TRACK)** 3계층. modality는 type/role과 직교 축, allocator가 `Allocate{type, modality, role, route}`로 동시배정.

---

## K. 노드 간 연관 관계

### K-1. 3계층 관계도

```
[백그라운드 서비스]  election · registry · allocator   (aiss_swarm_core, 상시·계산)
        ▲ 구독/호출          ▲ srv               ▲ srv
        │                    │                    │
[BT 노드]  IsLeader      SwarmJoin           AllocateTasks(리더 푸시)
           WaitForLeaderLost  QuorumReady    RequestAllocation(팔로워 풀)
           GatherRegistry(리더)
        │ 블랙보드(type/modality/role/route)
        ▼
[BT 노드]  HasMissionType · HasModality · HasRole (selector)
        │ 액션/서비스 호출
        ▼
[AS2 substrate]  FollowReference · GoTo/FollowPath/Takeoff/Land · platform srv · PointGimbal   (swarm_flocking 미사용)
```

### K-2. 백그라운드 서비스 ↔ BT 노드 (서비스를 어느 BT가 감싸나)

| 서비스 | 제공 인터페이스 | 소비 BT 노드 | 방향 |
|---|---|---|---|
| `swarm_election` | pub `swarm/leader_id`, `swarm/heartbeat` | `IsLeader`(구독→분기), `WaitForLeaderLost`(heartbeat timeout 감시) | 구독 |
| `swarm_registry` | srv `swarm/join`, pub `swarm/registry`·`mission_version` | `SwarmJoin`(호출), `QuorumReady`(구독), `GatherRegistry`(리더 집계) | 호출+구독 |
| `swarm_allocator` | srv `swarm/allocate` | `AllocateTasks`(리더 **푸시 트리거**), `RequestAllocation`(팔로워 **풀**) | 호출 |

> `AllocateTasks`(리더가 할당 시작) ↔ `RequestAllocation`(팔로워가 자기몫 수신) = **같은 allocator를 양쪽서 감싼 BT 노드**.

### K-3. BT 노드 → AS2 substrate (어느 behavior/srv 호출)

| BT 노드 | AS2 대상 | 종류 |
|---|---|---|
| `SwarmFormation` | pub `/swarm/formation` (분산, swarm_flocking 미사용) | topic |
| `FollowReference` | `FollowReferenceBehavior` | action |
| `GoTo`/`FollowPath`/`TakeOff`/`Land` | 해당 behavior | action |
| `Arm`/`Offboard` | `set_arming_state`/`set_offboard_mode` | srv |
| `PointGimbal` | `PointGimbalBehavior` | action |
| `RfSurvey` | RF sensor 측위(신규) | (신규) |
| `ReportHealth` | pub `{drone}/health` | topic |

### K-4. 인식 노드 ↔ 조율 (배경 → allocator/GCS)

| 인식 노드 | 입력 | 출력 | 소비 |
|---|---|---|---|
| `vision_detect` | `sensor_measurements/camera` | detections(bbox) | `swarm_allocator`(재할당 트리거), GCS 융합 |
| `rf_scan` | `sensor_measurements/rf`(신규) | emitters(방위/주파수) | `swarm_allocator`, GCS |

### K-5. 데이터 의존 체인 (한 임무 사이클)

```
election ─leader_id─▶ IsLeader ─(리더)─▶ AllocateTasks ─srv─▶ allocator
                                                                  │ 분해(type/modality/role/route)
registry ◀─join── SwarmJoin                                       │
   │ registry/quorum                                              ▼
QuorumReady ◀────┘                          RequestAllocation ◀─srv─ allocator
                                                  │ 블랙보드 쓰기
                                                  ▼
                          HasMissionType→HasModality→HasRole (selector)
                                                  │
                                                  ▼
                          GoTo/FollowPath/FollowReference/PointGimbal/RfSurvey
                                                  │ 관측
                          vision_detect/rf_scan ──▶ allocator(재할당 루프)
```

### K-6. 관계 요약 (Mermaid)

```mermaid
flowchart TB
    subgraph CORE["aiss_swarm_core (서비스)"]
        EL[swarm_election]
        RG[swarm_registry]
        AL[swarm_allocator]
    end
    subgraph BT["aiss_swarm_bt (BT 노드)"]
        ISL[IsLeader]
        WLL[WaitForLeaderLost]
        SJ[SwarmJoin]
        QR[QuorumReady]
        AT[AllocateTasks]
        RA[RequestAllocation]
        SFM[SwarmFormation]
        SEL[HasMissionType/Modality/Role]
    end
    subgraph AS2["AS2 substrate"]
        FR[FollowReference]
        MV[GoTo/FollowPath/Takeoff/Land]
        GB[PointGimbal]
    end
    subgraph PER["aiss_swarm_perception"]
        VD[vision_detect]
        RF[rf_scan]
    end

    EL -->|leader_id| ISL
    EL -->|heartbeat| WLL
    SJ -->|join| RG
    RG -->|registry| QR
    AT -->|srv| AL
    AL -->|allocate| RA
    RA -->|블랙보드 type/modality/role/route| SEL
    SEL --> MV
    SEL --> FR
    SFM -.리더.->|/swarm/formation| FR
    SEL --> GB
    VD -->|detections| AL
    RF -->|emitters| AL
```

---

## L. 노드별 역할 상세

### L-1. 백그라운드 서비스

- **`swarm_election`** — 군집의 리더를 정한다. 전 드론이 5Hz heartbeat를 주고받고, 살아있는 최저 id를 리더로 계산해 `swarm/leader_id`로 광고. 리더가 죽으면 차순위가 자동 승계 → 단일장애점 제거. 임무패키지가 복제돼 있어 권위상태 인수 = 캐시 읽기뿐(가벼운 선출).
- **`swarm_registry`** — 군집 멤버십을 관리한다. 팔로워 Join을 수락해 등록부(registry) 구성, 정족수(quorum) 충족 여부와 임무버전 동기화를 추적. "지금 몇 대가 임무에 묶여있나"의 단일 출처. boot 직후 GCS 없이 형성.
- **`swarm_allocator`** — 의도를 구체 배정으로 푸는 **계산 엔진**. GCS 추상의도(종류/AO/제약)를 받아 Voronoi 구역분할·탐색패턴·역할·수단·경로를 생성. 무거운 compute라 BT가 아닌 서비스. 표적탐지·기체손실 시 재분해도 담당.

### L-2. Condition (분기 판정, RUNNING 없음)

- **`IsLeader`** — 자신이 현재 리더인지 매 tick 판정. 같은 XML을 전 드론이 돌리되 이 조건이 리더/팔로워 가지를 자기선택하게 함. 리더 교체 시 다음 tick 즉시 반영(반응형).
- **`QuorumReady`** — 등록 기체수가 최소 정족수 이상인지. 미달이면 계획·arm 진행을 막는 게이트(기체 부족 임무 방지).
- **`HasMissionType`** — 현재 임무종류({mtype})가 지정값(RECON/TRACK/SURVEIL)과 일치하는지. 종류 selector의 분기 판정. {mtype}는 **per-drone**(Allocate 필드, D2) → 드론별 이질 임무 가능.
- **`HasModality`** — 정찰 수단({modality})이 VISION/RF 중 무엇인지. 정찰 안에서 비전/RF 별도동작을 가르는 분기.
- **`HasRole`** — 드론 역할({role})이 지정값과 일치하는지. 종류·수단 안에서 드론별 행동 분기. 역할 = **제네릭 4종 string**(SCOUT/TRACKER/RELAY/STANDBY, D3) — 종류가 맥락 부여(감시 SCOUT=관측, 추적 TRACKER=전담).
- **`EvalTermination`** — 임무 종료조건(coverage/배터리/시간) 평가. KeepRunningUntilFailure 아래라 **계속이면 S, 종료조건 충족 시 F**(루프 탈출). GCS 없이 온보드 판정.

### L-3. Action — Service 래핑

- **`SwarmJoin`** — 부팅 후 군집에 등록 요청. health를 실어 registry에 가입, 임무버전 수신. 이게 SUCCESS여야 임무에 합류.
- **`RequestAllocation`** — 자기 배정(type/modality/role/route/zone)을 allocator서 받아 **블랙보드에 적재**. 팔로워가 "내가 뭘 할지" 가져오는 노드. 재지정 시 갱신값 반영.
- **`AllocateTasks`** (리더) — 리더가 allocator에 **전체 할당 시작을 트리거**. RequestAllocation(팔로워 풀)과 짝 — 같은 allocator를 리더 측에서 푸시로 감쌈.
- **`Arm`/`Offboard`** (재사용) — platform 무장/오프보드 서비스 호출.

### L-4. Action — Action 래핑 (장시간 → RUNNING)

- **`SwarmFormation`** (리더) — **분산 편대**. 편대형태(line/wedge/...)에서 virtual centroid + 드론별 오프셋을 계산해 `/swarm/formation`(`PoseStampedWithIDArray`, 드론별 reference)로 **발행**. 각 드론은 자기 reference를 `FollowReference`로 추종. AS2 `swarm_flocking` 노드는 **미사용**(단일노드+Static TF 권위가 이동 리더와 충돌, 옵션3). 편대 권위 = 리더와 동행(토픽 발행자)이라 리더 교체 시 발행자만 바뀜.
- **`FollowReference`** — 자기 편대 오프셋(ref frame 원점)을 추종. 편대유지의 개별 드론 측 실행. 정찰/대기에서 편대 점유.
- **`GatherRegistry`** (리더) — Join들을 모아 registry를 구성·정족수 도달까지 대기. LeaderCoord 진입 게이트.
- **`RfSurvey`** — RF 정찰 특화 기동(삼각측량/DF/peak loiter) + 스펙트럼 스캔. 비전 정찰의 커버리지 비행과 달리 신호측위 패턴. (신규 구현)
- **`GoTo`/`FollowPath`/`TakeOff`/`Land`/`PointGimbal`** (재사용) — 이동/경로/이착륙/카메라지향. AS2 기존 behavior.

### L-5. SyncAction

- **`ReportHealth`** — 자가진단(gps/imu/배터리/통신/측위)을 `{drone}/health`로 상시 발행. registry·pre-flight·이상감시의 입력. 즉시 SUCCESS(발행만).

### L-6. Decorator (이벤트 대기·감시)

- **`WaitForLeaderLost`** — 리더 heartbeat를 감시하는 게이트. 정상이면 자식(임무) 통과, 리더상실+timeout이면 자식 halt 후 FAILURE → 상위 Fallback이 AutonomousCached(캐시 단독비행)로 폴백. 단절 자율성의 핵심.
- **`WaitForSwarmCmd`** — 리더 명령(swarm/cmd String)을 대기. 명령 전 RUNNING, 수신 시 자식(역할 수행) 실행. 임무 시작/재개 트리거.
- **`WaitForAlert`** (재사용) — AlertEvent 대기. 수신 시 Emergency 자식 실행 + halt-cancel로 실행중 임무 선점. 안전 최우선.

### L-7. 인식 노드 (배경, modality별)

- **`vision_detect`** — 카메라(EO/IR) 영상서 표적 탐지(bbox/클래스/신뢰도). 비전 정찰의 관측 산출. allocator 재할당·GCS 융합 입력.
- **`rf_scan`** — 스펙트럼 분석으로 방사원(emitter) 방위·주파수 추출. RF 정찰 관측. allocator·GCS 입력.
