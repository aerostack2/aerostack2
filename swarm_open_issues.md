# 군집 설계 미해결 이슈 트래커

> 주력 설계 B(BT 네이티브) 기준 열린 이슈. 상태: ⬜ 미해결 / 🟧 진행 / ✅ 해결.
> 설계 ↔ AS2 구현 대조 검토 결과 (node_reference §I, workflow §7와 동기).

---

## 🔴 설계 결정 (구현 전 결정 필요)

| # | 이슈 | 상태 | 결정/방향 |
|---|---|---|---|
| D1 | **SwarmFlocking 호스팅** — 단일 고정노드+Static TF 권위 ↔ 이동 리더(D2) 충돌 (리더 교체 시 편대 사령탑 같이 죽음 / 고정 호스트면 단절 시 상실) | ✅ **해결** | **분산 편대(옵션3)**: 리더가 `/swarm/formation`(centroid+오프셋, `PoseStampedWithIDArray`) 발행 → 각 드론 `FollowReference` 추종. AS2 `swarm_flocking` **존치하되 미사용**. 편대권위=리더와 동행 → 리더교체=발행자 교체뿐, SPOF·TF권위 제거. D8 개정. |
| D2 | **per-drone mtype** — 암묵 전역 ↔ 이질임무 | ✅ **해결** | 권위 = **per-drone `Allocate.mtype`** 필드(블랙보드 {mtype}). 전역 `/swarm/mission_type`=모니터링 default. 균질=전원 동일, 이질=일부 override(단일 메커니즘). EvalTermination **task-group별** 평가 |
| D3 | **역할 enum 통일** — 종류마다 다른 역할명 | ✅ **해결** | **제네릭 역할 4종 string**: SCOUT/TRACKER/RELAY/STANDBY. 종류별 특수명 흡수(WATCHER→SCOUT, RELIEF→RELAY). enum 강제 X(`Allocate.role`=string) → C2/신규종류 확장 자유. 예제 XML 통일 반영 |

## 🔴 트리 구조 결함 (2차 검토 — 연속 vs 1회성)

| # | 이슈 | 상태 | 해결 |
|---|---|---|---|
| NEW-1 | **RequestAllocation 1회성** — SequenceStar 안 srv pull-once → 블랙보드 1회 세팅 → 반응형 selector 입력 갱신 안 됨 → 동적 재할당 무력 | ✅ | **`UpdateAllocation`**(push `/{drone}/swarm_agent/task` 상시구독→블랙보드). Parallel KeepRunning. `Task.msg` 추가. (XML 반영) |
| NEW-2 | **SwarmFormation 1회성** — /swarm/formation 1회발행 → centroid 고정 → 편대 정지 | ✅ | LeaderCoord Parallel KeepRunning → centroid 연속발행 (XML 반영) |
| NEW-3 | **ReportHealth 1회성** — 상시 health 없음 | ✅ | Parallel KeepRunning 상시 브랜치 (XML 반영) |
| NEW-4 | **분산편대 plumbing** — /swarm/formation(pose) ↔ FollowReference(TF frame) 변환 미명세 | ⬜ | per-drone `formation_ref` 노드: pose→TF broadcast→FollowReference (§plumbing) |
| NEW-5 | **배정 thrash** — {role}/{mtype} 잦은변경 → halt-cancel·재goal 진동 | ⬜ | allocator debounce/hysteresis |
| NEW-6 | **핸드오버 편대 갭** — 리더 교체 시 /swarm/formation 공백 → reference stale | ⬜ | latch QoS(transient_local) + 갭 hover |
| NEW-7 | **파티션 split-brain** — 두 리더 동시 발행 | ⬜ | quorum-gated leadership (소수파 자율/RTH) |

> NEW-1/2/3 근본: 연속노드를 1회성 SequenceStar에 배치 → Parallel KeepRunning으로 재배치 해결(XML 개편 완료).

### 전체 설계 검토 추가 발견 (NEW-8~12)

| # | 이슈 | 상태 | 해결방향 |
|---|---|---|---|
| NEW-8 | **의도 복제 누락** — 의도가 GCS→리더에만 → 리더 교체 시 재할당 불가 → 단절생존 전제 붕괴 | ✅ **해결** | `MissionIntent.msg`+`MissionUpload.srv` 추가. `swarm_coordinator`가 검증 후 `/swarm/mission_intent`(transient_local) 발행 → **전 드론 로컬 캐시**. allocator가 로컬 의도로 분해 → 리더 교체에도 연속. (msg/node_ref/intent 반영) |
| NEW-9 | **드론간 분리·충돌 감시 부재** — 충돌 AlertEvent 생성 노드 없음 | ✅ **해결** | `separation_monitor`(드론별 배경): `/swarm/telemetry` 구독→CPA/TCPA→`AlertEvent`(FORCE_HOVER). `PublishTelemetry`(BT 상시) + `SwarmTelemetry.msg` 추가. F8 입력원 확보. 2계층(예측분리+임박호버) |
| NEW-10 | **이륙 순서·충돌회피 누락** — 밀집 동시이륙 충돌 | ✅ **해결** | `Task.launch_slot` 배정 + `WaitLaunchSlot` 데코(하위 슬롯 airborne까지 RUNNING, telemetry 분산판정). ArmTakeoff 전 게이트. (msg/XML/데코 반영) |
| NEW-11 | **배터리 종료 = 전군 vs 개별RTB 혼동** | ✅ **해결** | `BatteryLow`→`IndividualRTB`(개별 드론만 복귀, 반응형 선점). `EvalTermination`에서 batt_min 제거 → swarm종료=coverage/time/quorum. 과다 RTB는 정족수 미달로 자연종료. (XML/노드 반영) |
| NEW-12 | **SURVEIL 지속임무 미종료 + 이질 group 종료** | ✅ **해결** | `EvalTermination` **group-aware**(내부 로직): 종류별 조건(RECON=coverage, SURVEIL=time/GCS, TRACK=target-lost), 전 active group 충족 시만 종료. 먼저 끝난 group은 allocator 재배정. (node_ref/intent §11-7 반영) |

> NEW-8 = **최우선**(단절생존 핵심요구 전제). NEW-9/10/11 = 실비행 안전 갭.

### 추가 검토 (NEW-13~15, 배선 버그)

| # | 이슈 | 상태 | 해결 |
|---|---|---|---|
| NEW-13 | **WaitLaunchSlot ↔ launch_slot 순서버그** — UpdateAllocation(launch_slot 세팅)이 arm **다음** Parallel서 실행 → arm 단계 WaitLaunchSlot이 launch_slot 못 읽음 | ✅ **해결** | WaitLaunchSlot이 `/{drone}/swarm_agent/task` **직접 구독**해 launch_slot 읽음(블랙보드 의존 제거, 자기완결) |
| NEW-14 | **임무완료 종료 미전파** — 리더 EvalTermination=F가 팔로워에 안 감 → 임무 끝나도 팔로워 복귀 안 함 | ✅ **해결** | 리더 종료판정 → `/swarm/cmd` RTH 브로드캐스트. 팔로워 `WaitForSwarmCmd`가 RTH 수신 → IndividualRTB 경로. 종료전파 배선 |
| NEW-15 | **연속 BT 노드 100Hz 발행부하** — 텔레메트리 플러드 + separation O(N²) | ⬜ | 연속 노드 **내부 throttle**(실발행 10~20Hz) + `bt_loop_duration` 20~50Hz 검토 |
| NEW-16 | **트리 사망** — AS2 bt_manager가 트리 S/F 반환 시 정지 → 비행 중 transient F(서비스 timeout/behavior abort)가 **드론 추락** | ✅ **해결** | 취약노드 `RetryUntilSuccessful`(join/arm), 임무·Emergency 최종 `SafeHover`(항상 R), Parallel f=2. **트리는 landed-SUCCESS 외 절대 F 안 함** (XML/노드 반영) |

---

## 🟠 구현 패치 (방향 정해짐 — AS2 한계 우회)

| # | 이슈 | 상태 | 해결방향 | MS |
|---|---|---|---|---|
| P0 | **반응형 selector** — 평범 Fallback 비반응형 → 동적 재선택 불가 | ✅ | ReactiveFallback+ReactiveSequence (예제 XML 반영) | — |
| P1 | **데코 3종 신규구현** — AS2 WaitForEvent 다중tick 자식 버림(§11.1) | 🟧 **설계 완료** (`swarm_bt_decorators.md`) | 자식 RUNNING 유지 + 반응형. 구현=M2-6 | M2-6 |
| P2 | **WaitForLeaderLost 시맨틱** — leader-lost+timeout 시 자식 halt+F | 🟧 **설계 완료** | 연속게이트형, ReactiveFallback 하위 halt+F | M2-6 |
| P3 | **Takeoff 500ms sleep** — 안전 Parallel 동결 | 🟧 | **커스텀 `SwarmTakeoff`**(sleep 없이 wrap) + FCU 병행. AS2 무수정 (예제 XML 반영) | M2 |
| P4 | **BtServiceNode 생성자 무한wait** — BtServiceNode=서비스 **클라이언트**, 생성자가 대상 **서버 뜰 때까지** 블로킹(§17.4). 서버측 요청대기와 무관 | ⬜ | **1차=런치순서**(서버를 bt_manager보다 먼저 기동, M3-2) → 무한wait 미발생. 변경 0. **커스텀 타임아웃 base=옵션**(서버 크래시/미기동 시 graceful fail) | M3-2 |
| P5 | **action 서버 생성자 throw** | ⬜ | 런치순서 엄격화 | M3-2 |
| P6 | **`/swarm/*` 절대토픽** — 전역↔드론ns 혼재 | ⬜ | swarm/ 절대경로 강제 | M3 |
| P7 | **FollowPath 2점 한계** | 🟧 | **커스텀 `SwarmFollowPath`**(다점 port 파서). AS2 무수정 (예제 XML 반영) | M2 |
| P8 | RequestAllocation 포트 방향(in/out) | ⬜ | output 확정 | M2-3 |

## 🟡 성능·정리

| # | 이슈 | 상태 | 비고 |
|---|---|---|---|
| Q1 | 노드마다 자체 executor + 중복 구독(@100Hz) | ⬜ | 공유 검토 |
| Q2 | allocator 무거운 compute vs tick | ⬜ | 서비스 멀티스레드 |
| Q3 | IsFlying 등 토픽 하드코딩(ns 의존) | ⬜ | — |

## 🆕 신규 개발

| # | 항목 | 상태 | MS |
|---|---|---|---|
| N1 | RF sensor 인터페이스 (AS2 표준 없음) | ⬜ | M5-4 |
| N2 | perception 노드 modality별 (vision_detect/rf_scan) | ⬜ | M5 |
| N3 | aiss_swarm_msgs B판 신규생성 | 🟧 **파일 생성 완료** (msg 7: Heartbeat/Health/DroneInfo/Registry/Task/MissionIntent/SwarmTelemetry + srv 3: JoinRequest/Allocate/MissionUpload). colcon 빌드검증만 대기 | M0-2 |
| N4 | per-drone `formation_ref` 노드 (NEW-4 plumbing) | ⬜ | /swarm/formation[me] → TF broadcast → FollowReference | M2/M5 |

## ⏭️ 확장 (요구 시)

| # | 항목 | 조건 |
|---|---|---|
| E1 | 동적트리 C2 (a_to_c2 P1~P3) | 현장 신규구조 정의 요구 시 |
| E2 | 멀티클러스터 (9+ 드론) | 대규모, 단일도메인+ns |
| E3 | 협조측위 estimator 플러그인 | GPS 스푸핑 대응 강화 시 (현 EKF2 DR로 드롭) |

---

## 차단(blocker) 요약 — 구현 진입 전 처리

| 우선 | 이슈 | 이유 |
|---|---|---|
| 1 | N3 aiss_swarm_msgs | 모든 노드 인터페이스 선행 |
| 2 | P1/P2 데코 신규 | 안전·수행·단절 의존 |

> **해결됨**: D1·D2·D3(설계결정), P0(반응형 selector), P3/P7(커스텀 노드), NEW-1/2/3(연속노드 재배치), **NEW-8(의도복제)·NEW-9(분리감시)·NEW-10(이륙순서)·NEW-11(배터리분리)·NEW-12(group종료)**.
> **남은 ⬜**: NEW-5(allocator debounce), NEW-6(핸드오버 latch QoS), NEW-7(quorum-gated leader), NEW-4(formation_ref 구현), P4/P5/P6(런치순서·절대토픽).
> **남은 blocker(구현)**: N3(빌드검증), P1/P2(데코구현). 처리 후 M0 진입.
>
> NEW-5/6/7 = allocator·election 구현 시 처리할 견고화 항목(설계방향 정해짐). 별도 노드 추가 불필요.

---

## NEW-4 plumbing 상세 — 분산편대 ↔ FollowReference

**문제**: 리더가 `/swarm/formation`(`PoseStampedWithIDArray`, 드론별 목표 pose)을 발행. AS2 `FollowReference` behavior는 **TF frame 추종**(pose 토픽 직접 안 받음). 변환 계층 필요.

**해법 (per-drone `formation_ref` 노드, aiss_swarm_core)**:
```
/swarm/formation (리더 발행, 드론별 pose array)
   │ 구독
[formation_ref] (각 드론, 배경)
   ├ 자기 entry(id==self) 추출
   └ TF broadcast: earth → {drone}/formation_ref   (동적 TF, 매 갱신)
   │
[FollowReference behavior] ← goal: frame_id = {drone}/formation_ref
   → 자기 formation_ref 프레임 추종 → 편대 유지
```
- swarm_flocking이 하던 "Swarm centroid + ref TF" 역할을 **드론별 분산**으로: 리더는 pose만 발행, 각 드론이 자기 TF broadcast.
- 리더 교체 = pose 발행자만 바뀜(TF broadcast는 각 드론 로컬 → 끊김 없음, NEW-6 완화).
- BT `FollowReference`의 `server_name`은 그대로, goal의 reference frame = `{drone}/formation_ref`.

> 구현: `formation_ref` = 경량 배경 노드(구독+TF broadcast). M2(또는 M5 편대 단계)에서.
