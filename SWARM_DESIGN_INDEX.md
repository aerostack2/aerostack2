# 군집 조율 설계 문서 인덱스

AS2(Aerostack2) 위에 리더-팔로워 군집 지능을 얹는 설계. **주력 = 설계 B (BT 네이티브).**

> 모든 문서는 **AS2 소스 직접 분석** 기반. 코드 변경 전 설계 단계.

---

## 읽는 순서 (현행 = 설계 B)

```
[개요]   swarm_overview.md                ← 기능 단위 설명자료 (처음 보는 사람용)
         swarm_full_architecture.md       ← AS2+swarm 전체 계층구조 (L0~L7, 레이어별 노드)
[기반]   AS2_NODES_ANALYSIS.md            ← AS2 구현 이해 (필독 선행)
            │
[설계]   swarm_bt_native_design.md        ★ 주력 — AS2 BT 네이티브 설계
         swarm_mission_example.xml        ← 예제 트리 (종류→수단→역할 selector)
         swarm_bt_node_reference.md       ← 노드별 책임·IO·tick결과 레퍼런스
         swarm_bt_decorators.md           ← 커스텀 데코 3종 구현설계
         swarm_intent_processing.md       ← GCS 의도→분해→실행 절차
            │
[구현]   swarm_implementation_workflow.md ★ 구현 태스크 (M0~M5 WBS/체크리스트)
         swarm_open_issues.md             ← 미해결 이슈 트래커
            │
[확장]   swarm_dynamic_tree_design.md     ← 동적 트리(옵션 C)
         swarm_a_to_c2_migration.md       ← A→C2 점진 로드맵
```

---

## 문서 목록 (현행)

| 파일 | 목적 | 종류 |
|---|---|---|
| `swarm_overview.md` | **기능(10) 단위 설명 — 역할·IO·연관, 처음 보는 사람용** | 개요 |
| `swarm_full_architecture.md` | **AS2+swarm 전체 계층구조 (L0~L7, 레이어별 주요 노드)** | 아키텍처 |
| `AS2_NODES_ANALYSIS.md` | AS2 노드 책임·pub/sub·연계구조 (소스 grep) | 분석 |
| `swarm_bt_native_design.md` | **AS2 BT 네이티브 군집조율 (주력)** | 설계 B |
| `swarm_mission_example.xml` | 예제 트리 (ReactiveFallback + Parallel 연속노드) | 예제 |
| `swarm_bt_node_reference.md` | 노드별 책임·IO·tick결과(S/F/R)·연관관계 | 레퍼런스 |
| `swarm_bt_decorators.md` | 커스텀 데코 3종(WaitForAlert/SwarmCmd/LeaderLost) 설계 | 구현설계 |
| `swarm_intent_processing.md` | GCS 의도(추상목표) → 분해 → BT 실행 + 다종임무/수단 | 절차 |
| `swarm_implementation_workflow.md` | 구현 태스크 (M0~M5 WBS·의존·수용기준) | 구현 |
| `swarm_open_issues.md` | 미해결 이슈 트래커 (설계결정/패치/신규/확장) | 트래커 |
| `swarm_dynamic_tree_design.md` | 동적 트리 옵션 C (런타임 생성/교체) | 확장 |
| `swarm_a_to_c2_migration.md` | A(정적)→C2(템플릿) 점진 로드맵 | 로드맵 |

**아카이브** (`swarm_archive/`, 진화 이력·참고): `swarm_intelligence_layer_spec_v1.md`(설계 A), `swarm_drone_coordination_design.md`(원본 야심설계), `swarm_design_graft_analysis.md`(접목 충돌분석), `swarm_mission_scenario_v1.md`(정찰 시나리오). B로 수렴된 입력·중간산물 — 근거 추적용.

---

## 구현 위치

**`e:\Project\AISS.os\aiss_ws\src\aiss\`** (overlay, 패키지별). aerostack2_ws = underlay 무변경.
- `aiss_swarm_msgs` — msg 7(Heartbeat/Health/DroneInfo/Registry/Task/MissionIntent/SwarmTelemetry) + srv 3(JoinRequest/Allocate/MissionUpload). **생성 완료**(빌드검증 대기).
- `aiss_swarm_core` — 6 서비스(election/registry/coordinator/allocator/formation_ref/separation_monitor) [M1].
- `aiss_swarm_bt` — 커스텀 BT 노드 20(데코 4 포함) + 커스텀 래퍼 2(SwarmTakeoff/SwarmFollowPath) [M2].
- `aiss_swarm_perception` — vision_detect/rf_scan + RF 인터페이스 [M5].
- `aiss_bringup` — launch(서비스→bt_manager) + 트리 [M3].
- 레이아웃 상세: `swarm_implementation_workflow.md` §3.5.

---

## 핵심 결정 (현 합의)

| 항목 | 결정 |
|---|---|
| 통합 방식 | **AS2 BT 네이티브** (별도 오케스트레이션 레이어 X), AS2 일체 무수정 |
| 리더 | on-drone 선출 (lowest-alive-id), 단절돼도 임무지속 |
| GCS | 제어루프 밖, best-effort 오버라이드 |
| 편대 | **분산** — 리더 `/swarm/formation` 발행 → 드론별 `formation_ref` TF → FollowReference. `swarm_flocking` 미사용(존치) |
| 임무종류/역할 | per-drone string. 종류(RECON/TRACK/SURVEIL)→수단(VISION/RF)→역할(SCOUT/TRACKER/RELAY/STANDBY) 3계층 selector |
| 동적성 | ReactiveFallback+ReactiveSequence selector + 배정 push 토픽 상시구독(UpdateAllocation) |
| AS2 문제노드 | 커스텀 대체 — SwarmTakeoff(sleep), SwarmFollowPath(2점), 데코 3종(WaitForEvent 버그) |
| 다종 임무 | 정적 selector(A) 기본, 동적트리(C2) 옵션확장 |

---

## 현 상태

| 구분 | 상태 |
|---|---|
| 설계 | 주력 B 확정. 핵심 이슈(D1·D2·D3·P0·NEW-1/2/3) 해결 |
| msg 패키지 | `aiss_swarm_msgs` 파일 생성 완료 (colcon 빌드검증 대기) |
| 데코/커스텀 노드 | 설계 완료 (구현 M2 대기) |
| 남은 blocker | 데코 구현(P1/P2), msg 빌드검증(N3) |
| 미해결(설계) | NEW-4(formation_ref 구현), NEW-5(allocator debounce), NEW-6(latch QoS), NEW-7(quorum-gated leader) — `swarm_open_issues.md` |

> 전 문서 설계 단계, git 미추적. 구현 착수 = M0(빌드검증)→M1(서비스)→M2(BT노드).
