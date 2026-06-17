# 동적 트리 방식 (옵션 C) 설계 검토

> **위치**: `swarm_bt_native_design.md` §12-5의 옵션 C 상세화. 정적 selector(옵션 A)로 안 되는 **런타임 신규 임무구조** 요구 시.
> **기준**: AS2 `as2_behavior_tree` v1.1.3 (bt_manager는 `main()`서 `createTreeFromFile` 1회 로드 — 분석 §12).

---

## 1. 목적 / 배경

옵션 A(종류 selector)는 **사전 enumerate한 임무종류**만 처리. 한계:
- 현장에서 정의하는 신규 임무구조 불가 (XML 재배포 필요).
- 메가트리 비대 (모든 종류·역할 한 파일).

옵션 C = **트리를 런타임에 생성/교체**. GCS가 임무구조를 전달하면 드론이 그 자리서 트리 재구성.

| | 옵션 A (정적 selector) | **옵션 C (동적 트리)** |
|---|---|---|
| 신규 임무구조 | XML 재배포·재빌드 | **런타임 추가** |
| 트리 크기 | 전 종류 메가트리 | 현 임무만 |
| mid-flight 전환 | 가지 selector | 트리 통째 교체 |
| 복잡도/보안 | 낮음 | **높음 (실행가능 트리 주입)** |

---

## 2. AS2 제약 (왜 그냥 안 되나)

`as2_behavior_tree_node.cpp` (분석 §12):
- `main()` → `factory.createTreeFromFile(tree_param, blackboard)` **1회**.
- `while(ok && RUNNING) tickWhileRunning()` 루프. **재로드 메커니즘 없음.**
- `BtActionNode` 생성자 = `wait_for_action_server(1s)` → 없으면 throw.
- `BtServiceNode` 생성자 = `wait_for_service()` **무한 대기** (§17.4).

→ 동적 트리 = **bt_manager 확장/포크 필수**. 그냥 못 얹음.

---

## 3. 아키텍처 — `dynamic_bt_runtime`

기존 bt_manager를 대체하는 확장 노드. 트리를 런타임에 받아 빌드·교체.

```
GCS / leader
   │ 트리 정의 (아래 3 변형 중 하나)
   ▼
[dynamic_bt_runtime]  (각 드론, bt_manager 대체)
   ├─ tree_source 수신 (topic/service)
   ├─ validate (parse + whitelist + sign)
   ├─ build (createTreeFromText)
   ├─ build-then-swap (구 트리 halt-cancel → 신 트리 활성)
   └─ tickWhileRunning() 재개
   ▼
[AS2 behaviors] (액션/서비스 — 무변경)
```

### 상태기계
```
IDLE ──tree 수신──▶ VALIDATING ──ok──▶ BUILDING ──ok──▶ SWAPPING ──▶ RUNNING
  ▲                     │fail              │fail            │
  │                     ▼                  ▼                │
  └──────── REJECT (구 트리 유지, GCS 보고) ◀──────────────┘
RUNNING ──새 tree 수신──▶ VALIDATING (구 트리 계속 tick)
```
핵심: **구 트리는 신 트리 검증·빌드 완료까지 계속 tick** (build-then-swap). 무중단.

---

## 4. 트리 전달 방식 (3 변형)

| 변형 | GCS 전송 | 드론 처리 | 유연성 | 보안 |
|---|---|---|---|---|
| **C1** full XML | 트리 XML 문자열 전체 | `createTreeFromText` | 최대 (임의 트리) | 최악 (임의 실행) |
| **C2 (권장)** descriptor→template | 임무 디스크립터(type+params) | 드론측 검증된 템플릿 라이브러리서 XML 생성 | 중 (템플릿 조합) | **양호 (바운드)** |
| **C3** library id | 사전배포 트리 파일 id | 해당 파일 로드(런타임) | 낮음 (= 옵션 B + 재로드) | 최상 |

**권장 = C2.** GCS는 "무엇을(type/zone/params)"만 보내고, **드론이 검증된 템플릿으로 트리를 합성**. 노드는 등록된 BT 노드로 바운드, 구조는 템플릿으로 바운드 → C1의 임의주입 위험 제거 + C3보다 유연. 진짜 개방형만 C1.

```
C2 흐름:
GCS → MissionDescriptor{ type:"PATROL_CORRIDOR", waypoints, dwell, formation }
드론 template_engine: 라이브러리에서 PATROL_CORRIDOR.template + params
                    → XML 생성 → validate → build
```

---

## 5. Reload 메커니즘

### 5-1. Build-then-swap (원자성)
```
1. 신 트리 VALIDATING (구 트리 tick 계속)
2. BUILDING: factory.createTreeFromText(new_xml, blackboard)
   ├─ 성공 → 3
   └─ throw(노드 미등록/액션서버 부재) → REJECT, 구 트리 유지
3. SWAPPING: old_tree.haltTree()  ← 실행중 AS2 액션 halt→async_cancel_goal
4. active_tree = new_tree
5. Groot2Publisher 재생성 (구 트리 바인딩 해제)
6. RUNNING 재개
```

### 5-2. 블랙보드 연속성
swap 시 블랙보드 선택적 보존:
| 키 | swap 동작 |
|---|---|
| `node`(ROS2), `home`, `leader_id`, `role` | **보존** (지속 상태) |
| 임무별 `route`,`zone`,`track_target` | 신 임무 디스크립터로 갱신 |
| BT 내부(`number_recoveries` 등) | 리셋 |

→ 새 블랙보드 생성 후 보존키 복사, 또는 동일 블랙보드 재사용+임무키 갱신.

### 5-3. 노드 생성자 블로킹 (§2 제약)
신 트리 빌드 = 전 노드 생성자 재실행 → `wait_for_action_server(1s)`/`wait_for_service` 재호출.
- 대응: 빌드를 **별 스레드 + 타임아웃**으로, BtServiceNode 무한대기를 타임아웃판으로 패치. 액션서버 부재 시 build 실패→REJECT(구 트리 유지). 비행중단 안 됨.

---

## 6. 검증 / 보안 (동적 트리 핵심 위험)

트리 = **실행가능 행동 정의**. 주입 = 드론 탈취. 다층 방어:

| 계층 | 방어 | 비고 |
|---|---|---|
| 1 인증 | GCS 서명 트리(sign) + 드론 검증 | 임의 발신자 주입 차단 (필수) |
| 2 파싱 | `createTreeFromText` dry-run try/catch | 문법오류 거부 (삭제한 .mmd_verify의 BT판) |
| 3 화이트리스트 | 등록된 BT 노드만 (factory가 미등록 ID throw) | 임의 노드 불가 — 이미 강제됨 |
| 4 구조한계 | 깊이·노드수·서브트리수 상한 | 자원고갈/무한루프 트리 차단 |
| 5 정책 | 금지조합 검사 (예: Disarm 비행중) | 위험 시퀀스 차단 |
| 6 샌드박스 | 검증 전 구 트리 유지 (build-then-swap) | 검증 실패가 현 임무 안 깸 |

> C2(템플릿) 채택 시 1·3·5가 구조적으로 충족 — 드론이 검증된 템플릿만 합성하므로 임의 노드/조합 원천 불가. **C2가 보안상 강력 권장 이유.**

---

## 7. 비행 중 swap 안전

swap = `haltTree()`로 실행중 액션 취소 → 드론이 순간 무지령. 안전전이 필요:
```
swap 요청 → 안전상태 진입 (현 위치 Hover, FollowReference 유지)
         → 신 트리 빌드·검증
         → 성공 시 swap, Hover 해제
         → 실패 시 Hover 유지 + 구 트리 복귀
```
- 정책: **swap은 HOLD/Hover 상태서만** 허용. 또는 swap 직전 자동 Hover behavior 삽입.
- 안전레이어(WaitForAlert)는 swap과 무관하게 platform-level로 상시(트리 밖 FCU failsafe).

---

## 8. 군집 배포 / 일관성

동적 트리도 군집 전체 일관 필요:
```
GCS → leader: 트리/디스크립터 + version_hash
leader: 검증 → 전 follower 브로드캐스트 (mission_version과 동일 복제 메커니즘)
각 드론: 동일 version 트리로 swap
```
- **버전 태그 + idempotent**: 중복/지연 수신 안전.
- **partition**: 새 트리 놓친 드론은 구 트리 유지(버전 비교) → 재연결 시 catch-up.
- 일부만 swap 위험: **배리어** — 전원 "신 트리 staged" 확인 후 동시 swap (정적 설계의 staged 배리어 재사용).

---

## 9. 옵션 A vs C — 채택 가이드

| 기준 | A 정적 selector | C 동적 트리 |
|---|---|---|
| 임무종류 알려짐·고정 | ✅ 최적 | 과함 |
| 현장 신규구조 정의 | ❌ | ✅ |
| 복잡도 | 낮음 | 높음 (bt_manager 확장) |
| 보안위험 | 없음 | 높음 (C2로 완화) |
| mid-flight 전환 | 가지 selector | 트리 교체(+안전전이) |
| AS2 무변경 | ✅ | ❌ (포크 필요) |

**권장 경로:**
1. **기본 = A** (정적 selector). 알려진 정찰/추적/감시엔 충분, 안전, AS2 무변경.
2. **현장 재구성 요구 시 = C2** (descriptor→template). A 위에 점진 추가.
3. **C1(full XML)은 지양** — 보안비용 과대. 진짜 필요할 때만, 서명+샌드박스 완비 후.

→ A와 C는 배타 아님. **A를 깔고, C2를 옵션 확장**으로. dynamic_bt_runtime이 정적 selector 트리도 그대로 실행(상위호환).

---

## 10. 리스크

| # | 리스크 | 대응 |
|---|---|---|
| 1 | 트리 주입 = 드론 탈취 | 서명+화이트리스트+C2 템플릿 |
| 2 | swap 중 비행 무지령 | HOLD/Hover 상태서만 swap |
| 3 | 빌드 시 노드 생성자 블로킹(§2) | 별 스레드+타임아웃, 실패시 구 트리 유지 |
| 4 | 블랙보드 불연속 | 보존키 정책 (5-2) |
| 5 | 군집 부분 swap | staged 배리어 동시 swap |
| 6 | AS2 bt_manager 포크 유지보수 | 업스트림 추적 부담 — C2 최소확장으로 한정 |
| 7 | Groot2 재바인딩 | swap 시 Publisher 재생성 |

---

## 11. 결론

- **동적 트리 = bt_manager 확장(`dynamic_bt_runtime`) 필수.** AS2 기본은 부팅1회 로드라 불가.
- **C2(descriptor→template) 권장** — 런타임 유연성 + 보안(바운드) 균형. C1(full XML)은 보안비용 과대로 지양.
- **build-then-swap + staged 배리어**로 무중단·군집일관 보장. swap은 안전상태서만.
- **A를 기본, C2를 옵션 확장** — 배타 아님, dynamic_bt_runtime이 정적 트리도 실행(상위호환).
- 핵심 비용: **bt_manager 포크 + 보안 다층방어**. 알려진 임무집합이면 A로 충분 — C는 현장 재구성이 진짜 요구일 때만.

### 구현 단계 (옵션 C 채택 시)
1. `dynamic_bt_runtime` — bt_manager 포크 + 상태기계(§3) + build-then-swap(§5).
2. C2 `template_engine` + 검증된 템플릿 라이브러리.
3. 트리 검증기 (parse/whitelist/sign/limit — §6).
4. swap 안전전이 (Hover 삽입 — §7).
5. 군집 배포 (version+배리어 — §8).
6. `MissionDescriptor` srv/msg 정의.
