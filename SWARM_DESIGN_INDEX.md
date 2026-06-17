# 군집 조율 설계 문서 인덱스

AS2(Aerostack2) 위에 리더-팔로워 군집 지능을 얹는 설계 문서 모음. 분석 → 설계 → 진화 순.

> 모든 문서는 **AS2 소스 직접 분석** 기반. 코드 변경 전 설계 단계.

---

## 읽는 순서

```
[기반]   1. AS2_NODES_ANALYSIS.md          ← AS2 구현 이해 (필독 선행)
[요구]   2. swarm_mission_scenario_v1.md    ← 정찰 임무 시나리오
[입력]   3. swarm_drone_coordination_design.md  ← 원본 야심설계 (멀티클러스터)
            │
[설계A]  4. swarm_intelligence_layer_spec_v1.md ← 독립 상위레이어 spec
[검토]   5. swarm_design_graft_analysis.md  ← 3을 AS2에 접목 시 충돌분석 + 순수상위 변환
            │
[설계B]  6. swarm_bt_native_design.md       ★ 현 주력 — AS2 BT 네이티브 설계
         7. swarm_mission_example.xml       ← 6의 예제 트리 (정찰/추적/감시)
         11. swarm_bt_node_reference.md     ← 6 노드별 책임·IO·tick결과 레퍼런스
         10. swarm_intent_processing.md     ← 6 기반 의도(추상목표) 처리 절차
            │
[확장]   8. swarm_dynamic_tree_design.md    ← 동적 트리(옵션 C) 검토
         9. swarm_a_to_c2_migration.md      ← A→C2 점진 확장 로드맵
            │
[구현]   12. swarm_implementation_workflow.md ★ 구현 태스크 관리 (M0~M5, WBS/체크리스트)
```

---

## 문서 목록

| # | 파일 | 목적 | 종류 |
|---|---|---|---|
| 1 | `AS2_NODES_ANALYSIS.md` | AS2 노드 책임·pub/sub·연계구조 (소스 grep) | 분석 |
| 2 | `swarm_mission_scenario_v1.md` | 정찰 임무 14단계 노드 흐름 | 요구 |
| 3 | `swarm_drone_coordination_design.md` | 멀티클러스터/Raft/CBBA/ORCA/RAIM 원본설계 | 입력 |
| 4 | `swarm_intelligence_layer_spec_v1.md` | AS2 독립 상위레이어 (노드/IO/시퀀스/pre-flight) | 설계 A |
| 5 | `swarm_design_graft_analysis.md` | 3 접목 충돌 10건 + 순수상위 변환 + spec 비교 | 검토 |
| 6 | `swarm_bt_native_design.md` | **AS2 BT 네이티브 군집조율** (주력) | 설계 B |
| 7 | `swarm_mission_example.xml` | 6 기반 예제 트리 (2계층 selector) | 예제 |
| 8 | `swarm_dynamic_tree_design.md` | 동적 트리 옵션 C (런타임 생성/교체) | 확장 |
| 9 | `swarm_a_to_c2_migration.md` | A(정적)→C2(템플릿) 점진 로드맵 | 로드맵 |
| 10 | `swarm_intent_processing.md` | GCS 의도(추상목표) → 분해 → BT 실행 절차 | 절차 |
| 11 | `swarm_bt_node_reference.md` | 설계 B 노드별 책임·IO·tick결과(S/F/R) 레퍼런스 | 레퍼런스 |
| 12 | `swarm_implementation_workflow.md` | 구현 태스크 관리 (M0~M5 WBS·의존·수용기준·체크리스트) | 구현 |
| 13 | `swarm_open_issues.md` | 미해결 이슈 트래커 (설계결정/패치/신규/확장, blocker) | 트래커 |
| 14 | `swarm_bt_decorators.md` | 데코 3종(WaitForAlert/SwarmCmd/LeaderLost) 설계 — AS2 WaitForEvent 버그 수정, tick 의사코드 | 구현설계 |

구현 위치: **`e:\Project\AISS.os\aiss_ws\src\aiss\`** (overlay, 패키지별). 패키지: `aiss_swarm_msgs`/`aiss_swarm_core`/`aiss_swarm_bt`/`aiss_swarm_perception`/`aiss_bringup` — 레이아웃은 `swarm_implementation_workflow.md` §3.5. (현재 디렉토리 비어있음, M0부터 신규 구현)

> **주력 = 설계 B (BT 네이티브, 문서 6).** 설계 A(문서 4)는 참고/비교용 — 단절생존·pre-flight·시퀀스 등 *개념*은 B로 이어지나, A의 노드/메시지 구성은 B에서 BT 리프 + 백그라운드 서비스로 대체됨.
>
> **분기 주의 (구현 전 통일 필요):**
> - `aiss_swarm_msgs` 패키지(12 msg + 2 srv)는 **설계 A 기준**. B는 `srv 2 + msg 2`(`JoinRequest`/`Allocate` + `Heartbeat`/`Registry`)만 필요 → B 착수 시 슬림화 대상.
> - 역할 enum: `Task.msg`(A) = SCOUT/TRACK/STANDBY ↔ 예제 트리(B) = TRACKER/RELAY/WATCHER/RELIEF(종류별). B는 블랙보드 문자열 사용 — 구현 시 종류별 역할집합 확정·통일 필요.

---

## 설계 진화 (왜 6이 주력인가)

```
3 원본설계 (멀티클러스터/Raft/CBBA/ORCA)
   │ AS2 접목 검토(5) → 충돌 10건. 3곳이 AS2 아래로 침투(제어/추정/디스커버리)
   ▼
4 독립 상위레이어 spec — AS2 무변경, 단절생존(복제+캐시+자율폴백) 체계화
   │ AS2 Behavior Tree 정합성 검토 → follower_agent ↔ BT 역할중복 발견
   ▼
6 BT 네이티브 — 조율을 BT 트리로 표현. 노드 17msg → srv2+msg2 급감.
   AS2 BT 어댑터·반응형·halt선점 그대로 활용 → 정합성 최고
   │ 다종 임무 → 정적 selector(A). 현장 신규구조 → 동적트리(C)
   ▼
8 동적트리(C) + 9 A→C2 로드맵
```

---

## 핵심 결정 (현 합의)

| 항목 | 결정 | 근거 문서 |
|---|---|---|
| 통합 방식 | **AS2 BT 네이티브** (별도 오케스트레이션 레이어 X) | 6 |
| 리더 위치 | on-drone 선출 (lowest-alive-id) | 4 D2 |
| GCS | 제어루프 밖, best-effort 오버라이드, 단절돼도 임무지속 | 4 D3/D4 |
| 편대 | **분산** — 리더 `/swarm/formation` 발행 → 각 드론 FollowReference. `swarm_flocking` 미사용(옵션3, D8 개정) | 4 D8 / open_issues D1 |
| 다종 임무 | 정적 selector(A) 기본, C2 옵션확장 | 6 §12, 9 |
| 침투 3종 | 이산화/네이티브위임/모니터화로 순수상위 유지 | 5 |
| 협조측위 | 드롭 (EKF2 네이티브로 충분), 향후 estimator 플러그인 | 5 |

---

## 현 권장 경로

1. **Phase 0 (즉시)**: BT 네이티브(6) — `aiss_swarm_bt` 노드 + `aiss_swarm_core` 서비스 + 예제트리(7). 정적 selector로 다종 임무.
2. **Day-1 불변식**: version_hash 배선 + 서브트리 모듈화 (C2 대비, 9 §2).
3. **확장**: 현장 재구성 요구 생기면 P1~P3(9)으로 순차. 없으면 P0 종착.

---

## 미결 / 백로그

- BT 노드 골격 구현 (`aiss_swarm_bt` 15종) + 백그라운드 서비스 3.
- `JoinRequest`/`Allocate` srv, `Heartbeat`/`Registry` msg 정의.
- FollowPath 포트 2점 한계(§17.2) 일반화 패치.
- 협조측위 estimator 플러그인 (옵션, 백로그).
- 멀티클러스터 확장 (대규모 시, 단일도메인+네임스페이스).
- 동적트리 P1~P3 (현장 재구성 요구 시).

---

## 상태

전 문서 **설계 단계 (코드 미구현)**. git 미추적 작업본. 구현 착수 = Phase 0 패키지 골격부터.
