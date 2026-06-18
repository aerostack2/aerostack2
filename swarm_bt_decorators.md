# 군집 BT 데코레이터 설계 (P1/P2) — WaitForAlert / WaitForSwarmCmd / WaitForLeaderLost

> **문제(P1)**: AS2 `WaitForEvent`/`WaitForAlert`는 이벤트 수신 후 flag 즉시 리셋 → **다중-tick(RUNNING) 자식을 버림**(분석 §11.1). Emergency(GoTo→Land), 역할수행 등 RUNNING 자식이 멈춤.
> **해결**: 세 데코를 신규 구현 — **자식이 RUNNING인 동안 매 tick 계속 tick**. BehaviorTree.CPP `BT::DecoratorNode` 상속.
> 대상: `aiss_swarm_bt`, 워크플로 M2-6.
>
> **원칙(AS2 무수정)**: AS2 `WaitForEvent`/`WaitForAlert` 파일은 **손대지 않는다**. aiss 자체 factory가 커스텀 데코를 등록해 사용(AS2 노드 미등록/미사용). 같은 부류의 커스텀 대체: `SwarmTakeoff`(TakeoffAction sleep 회피), `SwarmFollowPath`(2점 한계 회피) — 전부 AS2 원본 무수정.
> 단 **BtServiceNode 무한wait는 커스텀 불필요** — 클라이언트 생성자가 서버 뜰 때까지 블로킹하는 것이라 **런치순서(서버 선기동)로 해결**(서버측 요청대기는 정상 설계). 커스텀 타임아웃 base는 서버 미기동 대비 옵션 강건화만.

---

## 1. AS2 WaitForEvent 버그 (수정 대상)

```cpp
// AS2 원본 (잘못된 패턴)
tick() {
  spin_some();
  if (!flag_) return RUNNING;        // 이벤트 전 대기
  flag_ = false;                     // ★ 즉시 리셋
  return child()->executeTick();     // 자식 1회 tick
}
// 자식이 RUNNING 반환 → 다음 tick: flag_=false라 자식 안 돌리고 RUNNING(대기) 복귀
// → 실행중 자식(Emergency 등) 방치 → 결과 못 받음 → 멈춤
```

**고칠 핵심**: 이벤트 후 **"활성" 상태를 유지**하고, 자식 RUNNING 동안 계속 tick.

---

## 2. 공통 패턴 — 2가지 폴라리티

| 폴라리티 | 데코 | 의미 |
|---|---|---|
| **래치(latch)형** | WaitForSwarmCmd, WaitForAlert | 이벤트 1회 수신 → 활성 래치 → 이후 자식 계속 tick |
| **연속 게이트형** | WaitForLeaderLost | 매 tick 조건 평가 → 양호 시 자식 tick / 불량 시 자식 halt+F |

공통 구현 요소:
- 생성자: 독립 `CallbackGroup` + `SingleThreadedExecutor` + 토픽 구독 (AS2 BT 패턴).
- `tick()` 진입 시 `callback_group_executor_.spin_some()` 로 콜백 처리.
- `halt()`: 자식 halt + 상태 리셋.

---

## 3. WaitForAlert (래치형, 안전 선점)

**책임**: AlertEvent 수신 전엔 RUNNING(대기). 수신 시 자식(Emergency)을 **완료까지 계속 tick**.

| 항목 | 값 |
|---|---|
| 기반 | `BT::DecoratorNode` |
| 포트 | in `topic_name`(alert_event), out `alert`(코드) |
| 구독 | `as2_msgs/AlertEvent` |
| 상태 | `bool latched_`, `int alert_code_` |

```text
on_alert(msg):           # 콜백
  alert_code_ = msg.alert
  latched_ = true

tick():
  spin_some()
  if not latched_:
    return RUNNING                  # alert 전: Emergency 안 돌림
  setOutput("alert", alert_code_)
  return child.executeTick()        # ★ latched: 매 tick Emergency tick (RUNNING 유지)
  # 자식 SUCCESS(착륙완료) → SUCCESS 전파 → 루트 Parallel(s=1) → 임무 종료

halt():
  haltChild(); latched_ = false
```

> 루트 Parallel 최상위에 위치. latched 후 commit(재무장 안 함) — 안전은 되돌리지 않음. halt-cancel로 조율 브랜치 선점은 Parallel 완료 의미로 처리.

---

## 4. WaitForSwarmCmd (래치형, 명령 게이트)

**책임**: 리더 명령(String) 수신 전 RUNNING. 수신 시 자식(임무 수행)을 **계속 tick**. HOLD 명령 시 재게이트(선택).

| 항목 | 값 |
|---|---|
| 기반 | `BT::DecoratorNode` |
| 포트 | in `topic_name`(swarm/cmd) |
| 구독 | `std_msgs/String` |
| 상태 | `bool active_`, `string last_cmd_` |

```text
on_cmd(msg):
  last_cmd_ = msg.data
  if msg.data in {START, RESUME}: active_ = true
  if msg.data == HOLD:            active_ = false   # 재게이트(선택)

tick():
  spin_some()
  if not active_:
    haltChild()                   # HOLD면 실행중 임무 정지
    return RUNNING                # 명령 대기
  return child.executeTick()      # ★ active: 매 tick 임무 selector tick

halt():
  haltChild(); active_ = false
```

> START/RESUME=활성, HOLD=비활성(재게이트). RTH/ABORT 같은 안전계열은 AlertEvent로(WaitForAlert 경로). idempotent(version_tag)는 상위서 처리.

---

## 5. WaitForLeaderLost (연속 게이트형, 단절 폴백)

**책임**: 리더 heartbeat 양호 시 자식(임무) tick. 리더상실+timeout 시 자식 halt + **FAILURE** 반환 → 상위 `ReactiveFallback`이 `AutonomousCached`로 폴백. 리더 복귀 시 자동 재개(reactive).

| 항목 | 값 |
|---|---|
| 기반 | `BT::DecoratorNode` |
| 포트 | in `topic_name`(swarm/heartbeat), in `timeout_ms`(기본 T1=2000) |
| 구독 | `aiss_swarm_msgs/Heartbeat` |
| 상태 | `rclcpp::Time last_hb_` |

```text
on_heartbeat(msg):
  last_hb_ = now()                # (리더 id 확인은 election이, 여기선 생존만)

tick():
  spin_some()
  if (now() - last_hb_) > timeout_ms:   # 리더 상실
    haltChild()
    return FAILURE                # ★ ReactiveFallback → AutonomousCached
  return child.executeTick()      # 리더 양호: 임무 계속 tick

halt():
  haltChild()
```

> 상위 구조: `ReactiveFallback{ WaitForLeaderLost(임무), AutonomousCached }`. ReactiveFallback이 매 tick WaitForLeaderLost부터 재평가 → 리더 복귀 시 임무 자동 재개(AutonomousCached halt).
> 주의: `last_hb_` 초기값 = 부팅 시각(아직 heartbeat 없으면 즉시 timeout 안 되게 grace 부여).

---

## 5b. WaitLaunchSlot (래치-게이트형, 이륙 충돌회피 NEW-10)

**책임**: 이륙 전, 자기 `launch_slot`보다 낮은 슬롯 드론이 전부 airborne(FLYING)일 때까지 대기 → 차례 되면 자식(ArmTakeoff) 실행. 밀집 동시이륙 충돌 방지.

| 항목 | 값 |
|---|---|
| 기반 | `BT::DecoratorNode` (래치-게이트) |
| 포트 | in `topic_name`(swarm/telemetry), `task_topic`(swarm_agent/task) |
| 구독 | `SwarmTelemetry`(전 드론) + `Task`(자기, launch_slot) |
| 상태 | `map<id,airborne>`, `int my_slot_` |

```text
on_telemetry(msg): airborne_[msg.drone_id] = (msg가 FLYING 충족)
on_task(msg):      my_slot_ = msg.launch_slot   # ★ task 직접 구독 (NEW-13: 블랙보드 의존 X)

tick():
  spin_some()
  if my_slot_ 미수신: return RUNNING          # 배정 전 대기
  if 모든 (slot < my_slot_) 드론 airborne 아님:
    return RUNNING                            # 내 차례 전 대기
  return child.executeTick()                  # 차례: ArmTakeoff

halt(): haltChild()
```

> **NEW-13 수정**: launch_slot을 블랙보드 아닌 **task 토픽 직접 구독**으로 읽음 → UpdateAllocation(arm 후 실행)에 순서 의존 안 함. 자기완결.
> 분산 판정 — 중앙 grant 없이 telemetry로 자기 차례 계산. 리더는 `Task.launch_slot`만 배정. slot=0 즉시 이륙.

---

## 6. 의미 비교 요약

| 데코 | 이벤트 전 | 이벤트/조건 후 | 자식 RUNNING |
|---|---|---|---|
| WaitForAlert | RUNNING(대기) | 래치 → 자식 계속 tick | **유지** |
| WaitForSwarmCmd | RUNNING(대기) | 활성 → 자식 계속 tick (HOLD=재게이트) | **유지** |
| WaitForLeaderLost | 자식 tick(양호) | 상실 → halt+F | **유지(양호 시)** |

핵심 공통 수정: **자식이 RUNNING이면 매 tick `child.executeTick()` 호출** (AS2 버그 = 안 함).

---

## 7. BT.CPP 구현 노트

- `BT::DecoratorNode` 상속, `child_node_` 단일 자식.
- 자식 tick = `child_node_->executeTick()`. halt = `haltChild()` 또는 `child_node_->halt()`.
- 구독 콜백 = AS2 BT 패턴: ctor서 독립 `create_callback_group(MutuallyExclusive,false)` + `executor.add_callback_group` → tick서 `spin_some()`.
- 블랙보드 공유 노드 = `config().blackboard->get<rclcpp::Node::SharedPtr>("node")`.
- `providedPorts()`에 `topic_name`(+timeout_ms) 선언.
- **절대토픽 주의**(P6): `/swarm/*` 구독은 절대경로(드론 ns 해석 방지).

---

## 8. 수용 기준 (테스트)

| # | 케이스 | 기대 |
|---|---|---|
| T1 | WaitForAlert: alert 후 Emergency(GoTo 5tick→Land 3tick) | 8 tick 동안 자식 계속 tick, 중단 없음 → SUCCESS |
| T2 | WaitForSwarmCmd: START 후 임무 selector RUNNING 지속 | 매 tick 자식 tick, 방치 없음 |
| T3 | WaitForSwarmCmd: HOLD 수신 | 자식 halt, RUNNING(대기) 복귀 |
| T4 | WaitForLeaderLost: heartbeat 끊김 T1 초과 | 자식 halt + FAILURE |
| T5 | WaitForLeaderLost: 끊김 후 복귀 | 다음 tick 자식 재tick (ReactiveFallback 재선택) |
| T6 | 부팅 직후 heartbeat 없음 | grace 내 timeout 미발생 |

> AS2 `tests/node_emulators` 패턴으로 mock 자식(N tick RUNNING 후 SUCCESS) 만들어 검증.

---

## 9. 요약

- 세 데코 = **신규 구현**(AS2 WaitForEvent 재사용 불가).
- 공통 = 자식 RUNNING 동안 매 tick 계속 tick (버그 수정).
- 폴라리티: 래치형 2(Alert/SwarmCmd) + 연속게이트형 1(LeaderLost).
- WaitForLeaderLost는 `ReactiveFallback` 하위에서 halt+F로 폴백 트리거.
- 구현 = M2-6, 수용기준 T1~T6.
