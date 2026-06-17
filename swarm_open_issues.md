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
| N3 | aiss_swarm_msgs B판 신규생성 | 🟧 **파일 생성 완료** (msg 4 + srv 2, D2/D3 반영). colcon 빌드검증만 대기 | M0-2 |

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

> 해결됨: **D1**(분산 편대), **D2**(per-drone mtype), **D3**(제네릭 역할 string), **P0**(반응형 selector). 남은 blocker = N3 + P1/P2. 처리 후 M0 진입.
</content>
