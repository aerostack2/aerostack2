# 옵션 A → C2 점진 확장 로드맵

> **목적**: 정적 selector(A)로 시작해 동적 트리(C2, descriptor→template)로 **재작성 없이 점진 확장**. 각 단계 독립 출하 + 직전 단계로 롤백 가능.
> **근거 문서**: `swarm_bt_native_design.md`(A), `swarm_dynamic_tree_design.md`(C), AS2 `as2_behavior_tree` 분석.
> **핵심 전제**: `dynamic_bt_runtime`은 **정적 트리 상위호환** — A 트리를 그대로 실행. 그래서 A→C2는 교체가 아니라 *덧붙이기*.

---

## 1. 왜 점진 확장

- C 한 번에 = bt_manager 포크 + 보안 + swap안전 + 군집배포 동시 → 위험 폭증.
- A→C2 분할 = 각 능력을 **독립 검증**, 직전으로 롤백. A로 임무는 이미 돌아감(가치 즉시).
- 비결: **C2가 A를 흡수**(상위호환). 버리는 작업 0.

---

## 2. 설계 불변식 (Day-1에 박아야 C2가 덧붙이기로 됨)

A 단계에서 아래를 **미리 지키면** C2가 추가작업만 됨. 어기면 C2서 재작성.

| 불변식 | A에서 | C2가 재사용 |
|---|---|---|
| **BT 노드 어휘 고정** | `aiss_swarm_bt` 14+1 노드 factory 등록 | 템플릿이 같은 노드로 합성 — 노드 작업 0 재사용 |
| **종류·역할 = 블랙보드 selector** | `HasMissionType`/`HasRole` | 템플릿 산출물이 동일 구조 |
| **allocator 종류-aware** | `Allocate(type)` → role/route/formation | 그대로 |
| **서브트리 모듈화·명명** | ReconMission/Scout/Track... named | 템플릿이 named 서브트리 조립 |
| **트리 버전 plumbing** | 정적이라도 `version_hash` 배포·비교 배선 | C2 트리 배포가 동일 메커니즘 재사용 |
| **블랙보드 보존키 규약** | `node/home/leader_id/role` 보존 명시 | swap 시 그대로 |

> Day-1에 version_hash 배선 + 서브트리 모듈화가 가장 중요. 이거 빠지면 C2서 군집배포·합성 갈아엎음.

---

## 3. 단계별 로드맵

### Phase 0 — A 베이스라인 (정적 selector)
**산출**: 작동하는 다종 임무 (정찰/추적/감시).
- `aiss_swarm_bt` 노드 + `aiss_swarm_core` 서비스(election/registry/allocator).
- `swarm_mission_example.xml` — 2계층 selector(종류→역할).
- **표준 `bt_manager`** 그대로.
- 런치: 백그라운드 3서비스 → bt_manager.

상태: AS2 무변경. 알려진 임무 완전동작.

---

### Phase 1 — 런타임 셸 (dynamic_bt_runtime, 여전히 정적 트리)
**목표**: bt_manager 포크의 **무위험 도입**. 행동 변화 0.
- `bt_manager` → `dynamic_bt_runtime` 포크. 초기엔 **같은 정적 파일 로드**.
- 상태기계 골격 넣되 `IDLE→RUNNING`만 활성 (swap 미사용).
- tree-source 서비스/토픽 **배선만** (미사용), parse 검증 스켈레톤.

검증 포인트: **dynamic_bt_runtime이 Phase 0 트리를 동일하게 실행** = 포크 정상. 기능차 0 → 안전한 기반교체.
롤백: bt_manager로 되돌리면 끝.

---

### Phase 2 — 재로드 + 라이브러리 (C3급, 검증된 파일 런타임 교체)
**목표**: 재시작 없이 **사전배포 검증트리 간 런타임 전환**. 합성은 아직 X.
- 상태기계 `VALIDATING→BUILDING→SWAPPING` 활성, **build-then-swap**.
- tree-source = **라이브러리 파일 id**(사전배포, 서명). GCS가 tree_id 전송 → 재로드.
- **swap 안전전이**(Hover 삽입) + 블랙보드 보존키 적용.
- 노드 생성자 블로킹 대응(별 스레드+타임아웃).

상태: 알려진 트리 집합 내 런타임 전환. 신규구조 합성은 없음.
롤백: tree-source 무시하고 부팅 트리만 → Phase 1과 동일.

---

### Phase 3 — C2 템플릿 합성 (현장 정의 임무)
**목표**: GCS 디스크립터 → 드론이 **검증 템플릿으로 트리 합성**.
- `template_engine` + 검증된 템플릿 라이브러리.
- `MissionDescriptor{type, params}` → XML 합성 → 전체검증(서명/화이트리스트/구조한계/정책) → build.
- **군집 배포**: leader 검증 → 브로드캐스트 → version+staged 배리어 동시 swap.

상태: 템플릿 바운드 내 현장 신규임무. 완전 C2.
롤백: 디스크립터 거부 → Phase 2(라이브러리 트리)로.

---

## 4. 단계별 변화 / 불변

| 구성요소 | P0 | P1 | P2 | P3 |
|---|---|---|---|---|
| aiss_swarm_bt 노드 | ✅ | = | = | = (재사용) |
| aiss_swarm_core 서비스 | ✅ | = | = | +template_engine |
| 트리 런타임 | bt_manager | **dynamic_bt_runtime** | +swap | +합성 |
| 트리 소스 | 정적 파일 | 정적 파일 | +라이브러리 id | +descriptor |
| 검증 | 빌드타임 | +parse | +서명 | +full(whitelist/limit/policy) |
| swap | — | — | ✅ build-then-swap | +배리어 |
| 군집배포 | XML 동일배포 | = | version | +staged 배리어 |

> 노드·서비스·selector 구조는 **P0~P3 불변**. 바뀌는 건 트리 *공급 방식*뿐. = 점진성의 핵심.

---

## 5. 호환성 계약

**모든 단계서 정적 A 트리는 로드·실행된다.**
- `dynamic_bt_runtime`은 정적 트리 상위집합. P1 이후도 `swarm_mission_example.xml` 그대로 동작.
- C2(P3)는 A를 **대체 안 함** — A 트리 위에 합성옵션 추가.
- 어느 단계서 멈춰도 제품 — P0만으로도 다종 임무 완성품.

---

## 6. 단계 전환 트리거 (언제 다음으로)

| 전환 | 트리거 |
|---|---|
| P0→P1 | 동적 인프라 깔되 행동위험 없이 (선제 기반작업) |
| P1→P2 | 재배포 없이 **알려진 트리 간 런타임 전환** 필요 |
| P2→P3 | 현장서 **새 임무구조 정의** 필요 + 서명/검증 인프라 감당 가능 |

> 트리거 없으면 **머물러도 됨**. P0/P2가 종착이어도 정상. P3는 현장재구성이 진짜 요구일 때만.

---

## 7. 단계별 리스크 / 롤백

| Phase | 주 리스크 | 롤백 |
|---|---|---|
| P0 | selector 비대, FollowPath 2점한계(§17.2) | — (베이스) |
| P1 | 포크 회귀버그 | bt_manager 복귀 |
| P2 | swap 무지령, 생성자 블로킹 | tree-source 무시 |
| P3 | 트리 주입, 합성 오류, 부분 swap | 디스크립터 거부→P2 |

> 각 단계 롤백 = 직전 단계로 **무손실 복귀**(상위호환 덕).

---

## 8. 실행 체크리스트

**Phase 0 (지금 가능)**
- [ ] `aiss_swarm_bt` 15 노드 골격
- [ ] `aiss_swarm_core` election/registry/allocator
- [ ] `swarm_mission_example.xml` 2계층 selector
- [ ] allocator 종류-aware (`Allocate(type)`)
- [ ] **version_hash 배포·비교 배선** (정적이라도 — C2 대비)
- [ ] 서브트리 모듈화·명명 규약
- [ ] 블랙보드 보존키 규약 문서화

**Phase 1**
- [ ] `dynamic_bt_runtime` 포크 (정적 로드 유지)
- [ ] 상태기계 골격(IDLE→RUNNING)
- [ ] tree-source 인터페이스 배선(미사용)
- [ ] 회귀테스트: P0 트리 동일동작 확인

**Phase 2**
- [ ] build-then-swap + 상태기계 완성
- [ ] 라이브러리 tree_id 로드 + 서명검증
- [ ] swap 안전전이(Hover)
- [ ] 블랙보드 보존 적용
- [ ] 생성자 블로킹 타임아웃 패치

**Phase 3**
- [ ] `template_engine` + 템플릿 라이브러리
- [ ] `MissionDescriptor` srv/msg
- [ ] full 검증(whitelist/limit/policy)
- [ ] 군집 version+staged 배리어 swap

---

## 9. 결론

- **A→C2 = 교체 아닌 점진 덧붙이기.** dynamic_bt_runtime 상위호환이 핵심 가능요인.
- **Day-1 불변식**(노드어휘/selector/version 배선/서브트리 모듈화/보존키)만 지키면 C2가 추가작업으로 한정 — 재작성 0.
- **단계 독립 출하 + 무손실 롤백.** P0만으로 완성품, 필요할 때만 전진.
- 권장: **P0 즉시 + version 배선/모듈화 미리(P3 대비)**. P1~P3은 현장 재구성 요구 생길 때 순차.
