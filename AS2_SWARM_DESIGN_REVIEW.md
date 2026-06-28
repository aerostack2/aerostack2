# 군집 정찰 시스템 — 설계 오류 검토 보고서

> **검토 대상**: `AS2_SWARM_MODULE_OPERATION_SEQUENCE.md` (M01~M26)  
> **참조**: `AS2_SWARM_MODULE_DECOMPOSITION.md`, `swarm_bt/swarm_bt_combined.xml`, `swarm_bt/bootstrap/bootstrap_subtrees.xml`, `swarm_bt/chassis/chassis_drone.xml`  
> **심각도 기준**: Critical(런타임 크래시·기능불능) / High(중요 동작 오류) / Medium(엣지케이스·설계 결함) / Low(품질·정확도 이슈)

---

## 총평

| 심각도 | 건수 |
|--------|------|
| **Critical** | 5 |
| **High** | 4 |
| **Medium** | 12 |
| **Low** | 7 |
| 이상 없음 | M02, M10, M13(골격), M14, M23 |

---

## M01 · IDL 정의

**심각도: Critical**

**문제 1: `SwarmJoin.srv` 응답에 `slot` 필드 누락**

IDL 정의:
```
SwarmJoin.srv  res: {accepted, registry[], mission_version}
```
그러나 M21 BT `SwarmJoin.tick()` 구현(line 858):
```
BT->>BT: BB[my_slot] = res.slot   ← res에 slot 필드 없음
```
`WaitTakeoffSlot` / `WaitLandSlot` 데코레이터가 `BB[my_slot]`에 의존하므로, 이 필드가 없으면 슬롯 기반 이착륙 시퀀스 전체가 동작 불능이 된다.

**영향 범위**: M01 IDL → M05 registry → M21 SwarmJoin BT 노드 → M19 WaitTakeoffSlot/WaitLandSlot → M22 Dx_ArmTakeoff/Dx_LandSlot — P1·P5 이착륙 시퀀스 전체

**권장 수정**: `SwarmJoin.srv` 응답에 `takeoff_slot: int32`, `land_slot: int32` 필드 추가. 또는 M09 allocator가 task 발행 후 WaitTakeoffSlot이 `swarm_agent/task`에서 직접 슬롯을 읽는 방식으로 설계를 통일.

---

**문제 2: `RfBearing`, `EmitterTrack`, `MissionPhase` 메시지 필드 미정의**

IDL 빌드 시퀀스에는 `msg/RfBearing.msg`, `msg/EmitterTrack.msg`, `msg/MissionPhase.msg`가 포함되어 있으나, 메시지 필드 요약 표에서 누락. M12(emitter_fusion) 수신 콜백이 `{frequency, bearing_deg, drone_pose, stamp}` 필드를 가정하지만 IDL에 명시 없음.

**영향 범위**: M12 emitter_fusion, M16 rf_survey, M19 PhaseIs

**권장 수정**: M01 메시지 필드 요약 표에 3개 메시지 정의 추가.

---

## M02 · health_monitor

**이상 없음** — 독립 타이머 폴링 구조, 상태 전이, 콜백 알림 흐름 모두 일관성 있음.

---

## M03 · heartbeat

**심각도: Critical**

**문제: DEAD 판정 불가능 — elapsed 계산 순서 오류**

```
# 현재 시퀀스 (lines 150-156):
HB->>HB: survival_map[id].last_seen = now()    # ① last_seen 갱신
HB->>HB: elapsed = now() - survival_map[id].last_seen  # ② 갱신 직후 계산 → elapsed ≈ 0
alt elapsed > T_dead   # ③ 항상 false → DEAD 판정 불가
```

`last_seen`을 `now()`로 갱신한 직후 `elapsed`를 계산하므로 elapsed ≈ 0 이 된다. 수신 콜백 경로에서 DEAD 판정은 구조적으로 불가능하다.

**영향 범위**: M03 생존맵 → M04 election → M05 registry DEAD 제거 → M08 aggregator 드론 목록. 탈락 드론이 영원히 ALIVE로 남아 barrier 판정이 영구 대기 상태가 될 수 있음.

**권장 수정**: 수신 콜백에서는 `last_seen` 갱신만 수행하고, DEAD 판정은 별도 sweep 타이머(예: 1Hz)에서 수행:
```python
# 수신 콜백
def heartbeat_cb(msg):
    survival_map[msg.drone_id].last_seen = now()
    survival_map[msg.drone_id].status = ALIVE

# 별도 sweep 타이머 (1 Hz)
def sweep_timer():
    for id, entry in survival_map:
        elapsed = now() - entry.last_seen
        if elapsed > T_dead and entry.status != DEAD:
            entry.status = DEAD
            on_survival_map_changed()
```

---

## M04 · election

**심각도: High**

**문제: 리더 교체 시 신규 리더의 registry 초기화 방법 미명시**

승계 시나리오에서 `drone_1`이 리더가 되면 M05가 ACTIVE로 전환되며 "기존 registry 유지"로 기술되어 있다. 그러나 `drone_1`의 M05는 자신이 leader가 아닌 동안 `registry = {}`로 초기화된 상태이므로, 이전 리더(`drone_0`)가 발행한 `/swarm/registry` 토픽의 최신값을 캐시하지 않았다면 registry가 비어 있는 채로 ACTIVE 상태가 된다.

**영향 범위**: M05 registry → M08 aggregator drone_list → M09 allocator → 군집 임무 할당 전체

**권장 수정**: M05가 STANDBY 상태에서도 `/swarm/registry` 토픽을 구독하여 최신값을 캐시하고, ACTIVE 전환 시 캐시된 registry를 초기값으로 사용하도록 명시.

---

## M05 · registry

**심각도: High**

**문제: SwarmJoin.srv 동시 요청 시 레이스 컨디션**

루프 핸들러에서 여러 드론이 동시에 SwarmJoin 요청을 보낼 때:
```python
# 두 요청이 거의 동시에 도달
if drone_id not in registry:  # A, B 모두 false 확인
    registry[drone_id] = ...  # A 먼저 삽입
    # B도 else 분기로 진입해 중복 삽입
```
ROS2 서비스 콜백이 단일 executor 스레드에서 직렬 처리된다면 문제가 없으나, multi-threaded executor 사용 시 `registry` 딕셔너리 동시 접근으로 데이터 손상 가능성이 있다.

**영향 범위**: M05 registry 일관성 → quorum_check 정확도

**권장 수정**: `SwarmJoin` 서비스 핸들러를 단일 콜백 그룹(MutuallyExclusive)으로 명시하거나, `registry` 접근에 mutex 적용 명시.

---

## M06 · zone_partitioner

**심각도: Low**

**문제: 면적 균등화 bisection 최대 반복 횟수 미명시**

```python
while 면적 오차 > 5%:   # 수렴 실패 시 무한 루프
    경계 조정 (bisection)
```
비정형 폴리곤(오목 다각형, 좁은 협곡형)에서 bisection이 수렴하지 않는 경우 무한 루프 가능.

**영향 범위**: M09 allocator 호출 시 블로킹 → BT action timeout 유발

**권장 수정**: `max_iterations=50` 제한 추가. 수렴 실패 시 현재 최선의 분할로 조기 반환.

---

## M07 · centroid_driver

**심각도: Low**

**문제: 목표 도달 후 고정 broadcast 루프에서 새 goal 수신 시 전환 메커니즘 미명시**

goal 완료 후 50Hz 고정 broadcast 루프에 진입하는데, 새로운 `DriveCentroid` goal이 도달했을 때 루프를 중단하고 이동 루프로 재진입하는 방법이 시퀀스에 기술되어 있지 않다.

**영향 범위**: EoirReconCoord 연속 구간 (`DriveCentroid(recon_entry)` → ... → `DriveCentroid(home)`) 시 두 번째 goal이 무시될 수 있음

**권장 수정**: Action 서버 goal 수신 콜백에서 현재 루프 타입(고정/이동)을 atomic flag로 전환하는 로직 추가 명시.

---

## M08 · aggregator

**심각도: High**

**문제: T_barrier_start 공유 타이머의 의미 모호**

```
AGG->>AGG: T_barrier 타이머 시작 (barrier 전환 시마다 리셋)
```
4개의 barrier(`airborne`, `formed`, `zonesdone`, `landed`)가 단일 `T_barrier_start` 타이머를 공유한다. "barrier 전환 시마다 리셋"의 의미가 불명확하다:

- **해석 A**: 어떤 barrier가 `true`가 될 때마다 리셋 → 후속 barrier는 정상 타이머 적용
- **해석 B**: aggregator 5Hz 주기마다 리셋 → 타임아웃 무의미
- **해석 C**: 모든 barrier에 T_barrier 경과 후 일괄 강제 통과 → `airborne` 타임아웃이 `formed` 체크 중에도 실행

barrier별 타이머가 독립되지 않으면, TAKEOFF 단계의 T_barrier 타임아웃이 WORK 단계의 zonesdone 체크에도 영향을 줄 수 있다.

**영향 범위**: P1(이착륙 barrier), P2(formation barrier), P3(zonesdone barrier), P5(landing barrier)

**권장 수정**: 각 barrier 필드별 독립 타이머 사용:
```python
timer_start = {
    'airborne': None, 'formed': None, 'zonesdone': None, 'landed': None
}
# 각 barrier 판정 시작 시 해당 timer_start 초기화
```

---

## M09 · allocator

**심각도: Medium**

**문제 1: 탈락 드론의 구역 재할당이 부분 완료 상태를 무시**

```
ALLOC->>ALLOC: orphaned_zone 식별
ALLOC->>ALLOC: 남은 드론으로 orphaned_zone 재배분
```
`drone_2`가 자기 구역의 70%를 완료하고 탈락한 경우, 재할당은 전체 구역을 대상으로 하므로 이미 커버된 70% 영역을 재비행하게 된다.

**영향 범위**: 임무 효율성 저하, 배터리 낭비

**권장 수정**: 탈락 드론의 마지막 알려진 위치를 기반으로 미완료 구역만 추출하여 재할당. `SwarmTask` 또는 `swarm/telemetry`에 per-drone 경로 진행 인덱스(current_wp) 포함.

---

**문제 2: 역할 재배정 이중 트리거**

M09 allocator가 `/swarm/targets` 변화를 직접 구독하여 역할 재배정(step ⑤)을 수행하면서, M20 BT 노드 `AllocateTrackRoles`도 동일한 작업을 트리거한다. 동일 이벤트에 두 경로가 모두 반응할 수 있어 역할 발행이 중복될 수 있다.

**영향 범위**: M09 ↔ M20 간 역할 발행 경쟁

**권장 수정**: 역할 재배정 책임을 M09 단독으로 명확히 하고, M20 `AllocateTrackRoles`는 단순 상태 조회 노드로 변경. 또는 반대로 M09에서 targets 직접 구독 제거.

---

## M10 · coverage_planner

**이상 없음** — boustrophedon 알고리즘, 웨이포인트 생성, 경계 클리핑 흐름 논리적으로 일관성 있음. 경계 인접 웨이포인트의 short segment 가능성은 `padding_margin` 파라미터로 완화 가능.

---

## M11 · detection_fusion

**심각도: Medium**

**문제 1: weighted_avg 가중치 미명시**

```
DFUSE->>DFUSE: track.world_pose = weighted_avg(track.world_pose, p)
```
가중 평균의 가중치(점수 기반, 시간 기반, 관측 횟수 기반 등)가 명시되지 않아 구현자마다 다른 결과를 낳을 수 있다.

**권장 수정**: `w = detection.score`를 가중치로 사용하는 지수 이동 평균 공식 명시.

---

**문제 2: T_track 타이머 ↔ 탐지 콜백 mutex 부재**

T_track 만료 타이머가 `tracks[uuid]`를 삭제하는 동시에, 탐지 콜백이 `find_nearest(tracks, p)`를 실행하면 삭제 중인 항목에 접근 가능. Python GIL이 없는 C++ 구현에서 데이터 경쟁(data race) 발생.

**권장 수정**: `tracks` 딕셔너리 접근 시 mutex (rclcpp::Mutex) 명시.

---

## M12 · emitter_fusion

**심각도: Medium**

**문제 1: T_window 파라미터 미정의**

```
RFUSE->>RFUSE: 오래된 항목 제거 (T_window 초과)
```
`T_window`가 M01 IDL, M26 파라미터 체계 어디에도 정의되어 있지 않다.

**권장 수정**: `global_params.yaml`에 `emitter_fusion.bearing_window_sec: float` 추가.

---

**문제 2: 편대 비행 시 RF 기저선 부족**

M16 `RfReconP3Drone`에서 드론은 `Dx_HoldFormation` 상태(formation spacing=5m)로 RF 측정을 수행한다. M12의 기저선 체크 `max_baseline < min_baseline` 조건에서, 편대 드론 간격 5m이 `min_baseline` 이하이면 삼각측량이 항상 보류 상태가 된다.

**영향 범위**: RF_RECON 임무 전체에서 emitter 위치추정 불가

**권장 수정**: RF 임무에서는 formation spacing을 min_baseline 이상으로 설정하거나, RfReconP3Drone BT에서 분산 비행 후 측정하도록 설계 변경. 또는 `min_baseline`을 formation spacing보다 작은 값으로 파라미터화.

---

## M13 · detect_objects (서버 골격)

**이상 없음** — Action 서버 lifecycle, 이미지 콜백 활성/비활성 흐름 적절.

---

## M14 · ONNX/TRT 추론

**이상 없음** — 추론 파이프라인(전처리→추론→NMS→필터) 구조 일관성 있음.

---

## M15 · 지오로케이션

**심각도: Critical**

**문제 1: ray_earth.z ≈ 0 시 division by zero**

```python
t = -drone_altitude / ray_earth.z   # ray_earth.z = 0 → ∞
world_pos = drone_pos + t * ray_earth  # overflow
```
카메라가 수평에 가깝게 지향(피치 ≈ 0°)되거나 드론이 급격히 롤링하는 경우 `ray_earth.z ≈ 0`이 되어 수치 오버플로우 또는 NaN이 발생한다. 방어 코드가 전혀 없다.

**영향 범위**: detection_fusion으로 NaN 위치 전달 → TargetTrack 오염 → swarm/targets 오염

**권장 수정**:
```cpp
constexpr double MIN_RAY_Z = -0.1;  // 10° 이상 하향각 요구
if (ray_earth.z > MIN_RAY_Z) {
    // 하향각 불충분: 탐지 결과 폐기
    continue;
}
double t = -drone_altitude / ray_earth.z;
```

---

**문제 2: TF 조회 시각 불일치 (image stamp vs. now())**

```
DET->>DET: T = tf.lookup(earth, camera_link, now())
```
이미지 타임스탬프(`image.header.stamp`)가 아닌 `now()`로 TF를 조회하면, 이미지 캡처 시각과 TF 조회 시각 사이의 드론 자세 변화가 geolocation 오차로 직결된다. 10Hz 이미지, 100ms 처리 지연 가정 시 최대 10°/s 롤링 드론에서 ~1.0m 추가 오차 발생.

**영향 범위**: M15 geolocate 정확도 → DoD ≤2m 요건 위반 가능성

**권장 수정**: `tf.lookup(earth, camera_link, image.header.stamp, timeout=0.05s)` 사용.

---

## M16 · rf_survey behavior

**심각도: Medium**

**문제: route 파라미터가 실제로 사용되지 않음**

`RfSurvey.action goal {route, mode=triangulate}`에서 `route`를 수신하지만, `RfReconP3Drone` BT에서 드론은 `Dx_HoldFormation`(편대 유지)으로 동작한다. RF survey 드론은 route에 명시된 웨이포인트로 이동하지 않고, 편대가 이동하는 경로를 따라간다. `route` 파라미터는 사실상 dead parameter이다.

**영향 범위**: M16 설계 의도(웨이포인트별 측정)와 실제 BT 동작(편대 추종) 불일치

**권장 수정**: 설계를 통일. 옵션 A: `RfSurvey`는 단순 측정기로서 route 파라미터를 제거하고 현재 위치에서 측정. 옵션 B: `RfReconP3Drone`에서 `Dx_HoldFormation` 대신 `FollowPath(route)` 사용.

---

## M17 · 조율 BT 노드

**심각도: Medium**

**문제 1: PublishPhase가 발행 실패를 무시하고 즉시 SUCCESS 반환**

```
BT-)PHASE: /swarm/mission_phase = phase 발행
BT->>BT: return SUCCESS (즉시)
```
네트워크 버퍼 포화 등으로 발행이 실패해도 SUCCESS를 반환한다. `SequenceStar`는 한 번 SUCCESS한 노드를 재실행하지 않으므로, 단계 변경 메시지가 팔로워에게 전달되지 않은 상태에서 코디네이터가 다음 단계로 진행할 수 있다.

**권장 수정**: QoS를 `Reliable + Transient_Local(depth=1)`로 설정하거나, publish 완료 후 subscriber 확인 로직 추가.

---

**문제 2: SwarmFlockingStart ↔ AwaitFormed 이중 대기**

`Cx_FormUp` 서브트리:
```xml
<SequenceStar>
  <Action ID="SwarmFlockingStart" .../>  <!-- 편대 수렴까지 대기 (RUNNING) -->
  <Condition ID="AwaitFormed"/>          <!-- 다시 formed 확인 -->
</SequenceStar>
```
`SwarmFlockingStart.tick()`이 AS2 SwarmFlocking action 완료(모든 드론 수렴)를 기다려 SUCCESS를 반환하고, 그 직후 `AwaitFormed`가 M08 barrier에서 동일한 "formed" 조건을 다시 확인한다. 중복 대기가 발생하거나, AS2 action 완료와 M08 aggregator의 formed 판정 타이밍이 어긋나면 `AwaitFormed`가 추가 지연을 유발한다.

**권장 수정**: 두 방식 중 하나를 선택. 옵션 A: `SwarmFlockingStart`는 편대 명령만 발행하고 즉시 SUCCESS (fire-and-forget), `AwaitFormed`가 단독 barrier로 사용. 옵션 B: `SwarmFlockingStart`가 수렴을 기다리면 `AwaitFormed` 제거.

---

## M18 · barrier BT 노드

**심각도: Medium**

**문제: EmergencyAll의 AwaitAllLanded가 응답 불능 드론에 무기한 블록될 수 있음**

```xml
<BehaviorTree ID="EmergencyAll">
  <SequenceStar>
    <Action ID="SwarmFlockingStop"/>
    <Action ID="PublishPhase" phase="EMERGENCY"/>
    <Condition ID="AwaitAllLanded"/>  <!-- 추락 드론이 있으면? -->
  </SequenceStar>
</BehaviorTree>
```
추락하거나 통신 두절된 드론은 `landed` 상태가 되지 않는다. M08의 `T_barrier 초과` 로직이 EMERGENCY 단계에서도 동작해야 하지만, T_barrier_start가 EMERGENCY 단계에서 별도로 리셋되는지 명시되지 않았다.

**영향 범위**: 비상 착륙 절차 무기한 대기 → 살아있는 드론도 착륙 명령 못 받음

**권장 수정**: EMERGENCY 단계 전환 시 T_barrier_start를 명시적으로 리셋하고, T_barrier 초과 시 응답 없는 드론을 laggard로 간주하여 landed 통과.

---

## M19 · 게이트/슬롯 BT 노드

**심각도: Critical**

**문제: `swarm/telemetry` 토픽이 M01 IDL에 정의되지 않음**

`WaitTakeoffSlot` / `WaitLandSlot` 데코레이터:
```xml
<Decorator ID="WaitTakeoffSlot" topic_name="swarm/telemetry" .../>
<Decorator ID="WaitLandSlot"   topic_name="swarm/telemetry" .../>
```
M19 시퀀스에서 이 토픽으로 "현재 순번 드론 airborne/landed 여부"를 읽는다. 그러나 M01 IDL에 `swarm/telemetry` 토픽과 메시지 타입이 존재하지 않는다.

현재 시스템에서 드론별 airborne/landed 상태에 가장 가까운 정보는:
- `/swarm/barrier` (M08 aggregator) — 집계된 boolean만 있음
- `/{ns}/platform/info` (AS2) — 개별 드론 상태 있음

**영향 범위**: M22 Dx_ArmTakeoff, M22 Dx_LandSlot — P1 순차 이착륙, P5 순차 착륙 전체 기능 불능

**권장 수정**: 두 가지 선택지:
1. M01에 `SwarmTelemetry.msg {drone_id, airborne, landed, battery_level, stamp}` 추가, M08이 발행
2. 또는 `/swarm/barrier`를 확장하여 per-drone 필드 포함 (`airborne_mask: uint64` 비트필드 등)

---

## M20 · P3 임무 BT 노드

**심각도: Medium**

**문제: InspectTimeout 타이머의 halt() 시 리셋 보장 없음**

`InspectTarget` subtree가 중단(`halt()`)된 후 재진입할 때, `InspectTimeout` 노드의 타이머가 리셋되지 않으면 이전 15s 카운트가 계속 흐른다. BT.CPP에서 Condition 노드는 `halt()` 시 `on_halted()`를 호출받으나, 노드 구현이 `on_halted()`에서 타이머를 명시적으로 리셋하지 않으면 잔여 시간이 유지된다.

예) 드론이 탐지물을 발견 → InspectTarget 실행 10s → 탐지물 소실 → 커버리지 복귀 → 재탐지 → InspectTarget 재진입 → 5s 후 타임아웃(기대: 15s)

**영향 범위**: 정밀 관측 지속 시간 감소

**권장 수정**: `InspectTimeout::on_halted()` 구현에서 타이머 명시적 리셋 필요.

---

## M21 · 부트스트랩 BT 노드

**심각도: Medium**

**문제: SwarmJoin FAILURE 시 트리 전체 실패로 전파**

```
alt accepted == false:
  BT->>BT: return FAILURE
```
`SwarmJoin`이 FAILURE를 반환하면 부모 `SequenceStar`가 FAILURE 상태로 고착되고, `Parallel(success=1, failure=1)` 내 `KeepRunningUntilFailure`가 여전히 RUNNING이므로 Parallel은 FAILURE로 전파된다. 결국 `SwarmRoot`의 `SequenceStar`가 FAILURE → 트리 전체 종료.

건강 이상으로 거부된 드론은 안전 대기(SafeHover) 상태를 유지해야 하는데, 트리가 종료되면 AS2 플랫폼이 마지막 명령 상태로 남게 된다.

**영향 범위**: 건강 이상 드론의 안전한 대기 불가

**권장 수정**: `SwarmJoin`이 FAILURE 반환 대신 RUNNING으로 전환하여 무기한 대기하거나, `BootstrapAll` 부모에 FAILURE → SafeHover 폴백 추가.

---

## M22 · chassis XML 서브트리

**심각도: Medium**

**문제 1: `{home}` 미설정 시 비상 RTB 실패 → SafeHover 무기한 유지**

`Dx_Emergency` / `Dx_BatteryRTB`:
```
GoTo(home) → Land
실패 시 → SafeHover (격리)
```
MissionIntent가 아직 수신되지 않은 상태(부트스트랩 중)에서 비상이 발생하면 `BB[home]`이 빈 값이라 `GoTo`가 FAILURE → `SafeHover` 진입. 드론은 공중에서 무기한 정지한다.

**영향 범위**: 부트스트랩 단계의 비상 처리 안전성

**권장 수정**: `BB[home]`의 기본값을 `global_params.yaml`의 `home_pose`로 M26 launch 시 주입.

---

**문제 2: RetryUntilSuccessful(-1) 무한 재시도에 탈출 경로 없음**

`Dx_ArmTakeoff`에서 `num_attempts="-1"` 무한 재시도. FCU 고장으로 Arm이 영구 실패하는 경우 드론은 지상에서 계속 시도하고, M08이 T_barrier 후 해당 드론을 laggard로 제외한 이후에도 재시도를 계속한다. 이 상태에서 배터리를 소모하거나 이상 명령이 FCU에 지속 전달될 수 있다.

**권장 수정**: `T_boot_max` 파라미터 기반 시간 제한 추가 또는 `AwaitBoot`가 FAIL 상태에서 재시도 차단.

---

## M23 · 임무 루트 XML

**이상 없음** — SwarmRoot/SwarmCoord/SwarmDrone 흐름 기술이 XML과 일관성 있음.

---

## M24 · gcs_mission_iface

**심각도: High**

**문제: ABORT 명령이 WORK/LANDING 단계에서 차단됨**

`LatchMissionType` 데코레이터:
```python
alt 현재 phase가 lock_phases에 포함 (WORK·LANDING):
  BT->>BT: mtype 변경 금지 (Latch 유지)
```
운용자가 `MissionIntent {type=ABORT}`를 발행해도 WORK 또는 LANDING 단계에서는 임무 타입이 변경되지 않아 ABORT가 무효화된다.

**영향 범위**: 임무 중단 명령이 가장 필요한 활성 임무 단계에서 무시됨 — 안전 임계 결함

**권장 수정**: ABORT 처리를 `WaitForAlert` → `EmergencyAll` 경로로 분리. GCS가 ABORT 시 `alert_event` 토픽에 ABORT alert를 발행하거나, `LatchMissionType`에서 ABORT 타입만 예외적으로 통과시키는 로직 추가:
```python
if latest_intent.mission_type == ABORT:
    # Latch 우선순위 무시, 즉시 적용
    BB[mtype] = ABORT
```

---

## M25 · Sim 환경

**심각도: Low**

**문제: Static TF에서 Dynamic TF로의 전환 시점 미명시**

launch 시 `static_transform_publisher`로 `earth → drone_i/base_link` TF를 발행하지만, AS2 플랫폼이 기동하면 상태 추정기(state_estimator)가 동적 TF를 발행해야 한다. 두 소스가 동일한 TF 체인에 동시에 발행하면 TF tree에서 충돌이 발생한다.

**권장 수정**: launch 파일에서 AS2 플랫폼 노드 기동 후 static_transform_publisher를 종료하거나, static TF를 초기 배치용 별도 프레임(`drone_i/init_pose`)으로 분리.

---

## M26 · Launch/파라미터 체계

**이상 없음** — launch 순서, 파라미터 오버레이 레이어, 블랙보드 초기값 구조 적절. 단, `my_slot` 초기값 `""` 은 M01·M21 이슈가 해결되면 자동으로 해소됨.

---

## 우선순위 수정 로드맵

### Phase 1 — Critical (즉시 수정)

| # | 모듈 | 수정 내용 |
|---|------|-----------|
| 1 | M03 | DEAD 판정을 수신 콜백이 아닌 별도 sweep 타이머로 이전 |
| 2 | M01+M21 | SwarmJoin.srv 응답에 slot 필드 추가 또는 WaitTakeoffSlot이 swarm_agent/task에서 직접 읽도록 변경 |
| 3 | M15 | ray_earth.z 근사 0 가드 추가 (|z| < epsilon 시 탐지 폐기) |
| 4 | M19 | swarm/telemetry 토픽 IDL 정의 또는 /swarm/barrier 확장 |
| 5 | M24 | ABORT를 WaitForAlert 경로로 분리, LatchMissionType 우회 |

### Phase 2 — High (설계 단계 수정)

| # | 모듈 | 수정 내용 |
|---|------|-----------|
| 6 | M08 | barrier별 독립 타이머 (T_barrier_airborne_start 등) |
| 7 | M05 | STANDBY 중 /swarm/registry 캐시 + ACTIVE 시 복원 |
| 8 | M15 | TF lookup에 now() 대신 image.header.stamp 사용 |
| 9 | M04+M05 | 리더 교체 시 registry 복원 절차 명시 |

### Phase 3 — Medium (구현 전 명확화)

- M09: 부분 완료 구역 재할당 로직, 역할 재배정 이중 트리거 해소
- M11: weighted_avg 공식 및 mutex 명시
- M12: T_window 파라미터 IDL 등록
- M16: route 파라미터 사용 여부 결정
- M17: PublishPhase QoS, SwarmFlockingStart ↔ AwaitFormed 역할 통일
- M18: EmergencyAll T_barrier_start 리셋 명시
- M19/M21: SwarmJoin FAILURE 시 안전 폴백
- M20: InspectTimeout halt() 리셋 보장
- M22: home 기본값 주입, 무한 재시도 탈출 조건

---

_검토 기준: `AS2_SWARM_MODULE_OPERATION_SEQUENCE.md` (1298 lines), `swarm_bt/swarm_bt_combined.xml`, `swarm_bt/bootstrap/bootstrap_subtrees.xml`, `swarm_bt/chassis/chassis_drone.xml`_
