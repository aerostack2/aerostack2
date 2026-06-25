# 군집 임무 BT 설계 — 계층적 dispatch + 공용 chassis 서브트리

> **목표**: 다양한 상위 임무(정찰/매핑/배송/감시/추적…)를 **단일 루트 BT**에서 임무타입 selector로 분기하고, 이착륙·편대이동·복귀 같은 **공통 골격은 chassis 서브트리로 재사용**하는 군집 임무 프레임워크.
> **근거**: AS2 실소스(`as2_behavior_tree`, `as2_behaviors_swarm_flocking`, `as2_behaviors_motion`) + 앞선 분석(`AS2_RECON_SWARM_PHASED_DESIGN.md`).
> **핵심 결론**: AS2는 BT-native. orchestrator를 별도 상태기계로 만들지 않고 **coordinator BT가 단계열을 인코딩**한다. 임무별 차이는 **P3 work 서브트리 + selector 분기**로 격리. 공통 단계(P1/P2/P4/P5)는 chassis 서브트리 라이브러리로 공유.

---

## 1. 설계 원칙

| 원칙 | 내용 |
|---|---|
| **단일 루트, 서브트리 분기** | 임무타입별 별도 트리 로드 ✗. 하나의 루트에서 `HasMissionType` selector → 임무 서브트리. selector가 비활성 분기는 tick 안 함 → 효율 동등, 정적검증·반응선점 일관 |
| **공용 chassis 재사용** | 이착륙/편대이동/재편대/복귀/착륙 = 임무 무관. chassis 서브트리로 한 번 작성, 전 임무가 include |
| **임무 고유 = P3만** | 임무별 차이는 work phase(P3) 서브트리 + 페이로드/인식 behavior. 나머지 단계는 공통 |
| **2계층(coordinator/per-drone)** | 실코드 위상(`SwarmFlocking`이 중앙에서 N드론 FollowReference 지휘) 준수. coordinator BT가 단계·편대 제어, per-drone BT가 실행 |
| **phase 게이트 동기** | coordinator가 `/swarm/mission_phase` 방송, per-drone BT가 구독해 동작 전환 |
| **임무타입 latch** | 선점(alert)은 반응형 유지, 임무타입은 임계단계 중 전환금지(latch)로 flap 방지 |
| **파일 분리(include)** | 논리적 단일트리, 물리적 분할. 루트 + chassis + 임무별 서브트리 파일 |
| **AS2 무수정** | swarm은 BT 계층에만. 문제 노드는 커스텀 BT 래퍼로 대체 |

---

## 2. 2계층 구조

```
┌─ Coordinator BT (1 노드: GCS측 또는 지정 기체) ───────────────────┐
│  루트: Parallel{ 선점 ∥ 임무타입 selector }                        │
│   임무 selector → <임무>Coord 서브트리                              │
│     = SequenceStar{ chassis(이륙) · chassis(편대이동)             │
│                     · <임무>P3Coord · chassis(재편대) · chassis(착륙) } │
│   호출: SwarmFlocking / modify_swarm / task 배포 / phase 방송       │
└───────────────────────┬───────────────────────────────────────────┘
       /swarm/mission_phase · /{ns}/swarm_agent/task · SwarmFlocking
┌─ Per-drone BT (N 인스턴스, 드론 namespace) ────────────────────────┐
│  루트: Parallel{ 선점 ∥ 임무타입 selector }                        │
│   임무 selector → <임무>Drone 서브트리                              │
│     = phase 게이트(ReactiveFallback by /swarm/mission_phase):       │
│        TAKEOFF→ArmTakeoff · TRANSIT/REGROUP→FollowReference         │
│        · WORK→<임무>P3Drone · LANDING→Land                          │
│  실행: AS2 behavior (Takeoff/FollowReference/FollowPath/Land/...)   │
└────────────────────────────────────────────────────────────────────┘
```

- **coordinator** = 단계 진행·편대·동기. 무거운 계산(분할/경로/융합)은 백그라운드 서비스로.
- **per-drone** = phase 따라 AS2 behavior 실행. 임무 고유 P3Drone만 신규.

---

## 3. 계층적 dispatch 모델

3계층 selector (위→아래로 좁힘):

```
L1 임무타입   HasMissionType  EOIR_RECON / RF_RECON / SURVEIL / TRACK
   └─ L2 단계  SequenceStar    (coordinator) 또는 phase 게이트(per-drone)
        └─ L3 역할/수단  HasRole/HasModality  (임무 내 분기, 선택)
```

- **L1**: 임무종류 분기 4종. 사용자 제안 핵심. latch로 안정화.
- **L2**: coordinator는 단계열(SequenceStar), per-drone은 phase 게이트(ReactiveFallback).
- **L3**: 임무 안에서 역할(SCOUT/TRACKER/RELAY) 세분 — 특히 **추적**의 TRACKER/RELAY 분기.

---

## 4. 공용 chassis 서브트리 라이브러리

임무 무관 골격. 전 임무 coordinator/per-drone가 include. **신규 작성 1회**.

### 4.1 Coordinator chassis 서브트리
| 서브트리 ID | 동작 | 사용 노드 |
|---|---|---|
| `Cx_SequentialTakeoff` | 드론 순번 이륙 + 전기 airborne barrier | `SwarmSequence`(순번) + `AwaitAllAirborne`(barrier) |
| `Cx_FormUp` | SwarmFlocking 활성 + 편대 수렴 대기 | `SwarmFlockingStart` + `AwaitFormed` |
| `Cx_DriveCentroid` | centroid를 목표점으로 구동 + 도착판정 | `DriveCentroid` |
| `Cx_Regroup` | 산개→재편대 (SwarmFlocking 재활성, deconfliction) | `SwarmFlockingStart`(modify) + `AwaitFormed` |
| `Cx_SequentialLand` | 드론 순번 착륙 + 전기 landed barrier | `SwarmSequence` + `AwaitAllLanded` |
| `Cx_PublishPhase` | 현 단계 `/swarm/mission_phase` 방송 | `PublishPhase` |

### 4.2 Per-drone chassis 서브트리
| 서브트리 ID | 동작 | 사용 노드 |
|---|---|---|
| `Dx_ArmTakeoff` | 자기 순번 슬롯 대기 → Arm/Offboard/Takeoff | `WaitTakeoffSlot` + `Arm`/`Offboard`/`Takeoff`(AS2) |
| `Dx_HoldFormation` | 자기 슬롯 FollowReference 추종 | `FollowReference`(AS2) |
| `Dx_LandSlot` | 자기 순번 슬롯 대기 → Land | `WaitLandSlot` + `Land`(AS2) |
| `Dx_SafeHover` | 기본 호버 (최종 폴백) | `SafeHover` |
| `Dx_Emergency` | alert 시 호버→복귀→착륙 | `GoTo`/`Land`(AS2) |

---

## 5. 임무 고유 P3 서브트리 — 계약(contract)

임무별로 이 두 서브트리만 구현하면 chassis에 꽂힌다.

```
<임무>P3Coord  (coordinator 측)
  책임: 작업 계획·배포·완료판정
  입력: mission_intent, 드론 상태
  동작: [모드 선언] + 계획(분할/경로/거점) + task 배포 + 완료 barrier
  출력: /{ns}/swarm_agent/task, 완료 신호

<임무>P3Drone  (per-drone 측, WORK phase일 때)
  책임: 배정받은 작업 실행
  입력: /{ns}/swarm_agent/task
  동작: 비행패턴(FollowPath/GoTo/loiter) ∥ 페이로드/인식
```

### 임무별 P3 매핑 (4종)
| 임무 | type | 모드 | P3Coord | P3Drone |
|---|---|---|---|---|
| **EO/IR 정찰** | `EOIR_RECON` | 독립 분할 | zone 분할 + coverage 경로 + 탐지융합 + 완료집계 | `FollowPath{sub_route}` ∥ `DetectObjects`(EO/IR) ∥ `PointGimbal`(하향) + 탐지시 `InspectTarget` |
| **RF 정찰** | `RF_RECON` | 편대(측위 baseline) 또는 분산 | 측위 경로/형태 배정 + emitter 융합(삼각측량) + 완료집계 | `RfSurvey`(스펙트럼 스캔+DF 기동) ∥ 방위측정 → `/{ns}/rf/bearings` |
| **감시** | `SURVEIL` | 거점 분산(또는 편대 loiter) | 감시 거점 배정 + 교대 스케줄 | 순환 `FollowPath`(거점 loiter) ∥ `DetectObjects` ∥ `PointGimbal` |
| **추적** | `TRACK` | 혼합(TRACKER 독립 + RELAY 편대) | 표적 핸드오프(반응형 allocator 서비스) + 역할 배정 | `HasRole`: TRACKER→`GoTo{track_target}`(동적) ∥ `PointGimbal`(표적지향) / RELAY→`FollowReference`(중계 위치) |

비고:
- **EO/IR·감시**는 동일 인식 스택(`DetectObjects`/`PointGimbal`) 공유 — 감시 = 거점고정 정찰. 차이는 P3Coord 계획(분할 coverage vs 거점 loiter)뿐.
- **RF 정찰**은 측위 특성상 다기 baseline 필요 → P3에서도 **편대 유지 가능**(WORK phase가 독립일 필요 없음). chassis는 그대로, P3Coord가 모드=편대 선언.
- **추적**은 표적 핸드오프가 선형 phase 밖 → **반응형 allocator 서비스**가 역할 재지정, BT는 `HasRole`로 흡수(L3 selector).

→ chassis(P1/P2/P4/P5)는 4종 전부 동일. **P3Coord/P3Drone 2개만 임무별 신규.**

---

## 6. 커스텀 BT 노드 카탈로그 (신규)

`as2_behavior_tree`에 swarm 노드 전무 → 신규. (단일기 노드 Arm/Takeoff/GoTo/FollowPath/Land/FollowReference/PointGimbal은 재활용.)

### 6.1 군집 조율
| 노드 | 유형 | I/O | 역할 |
|---|---|---|---|
| `HasMissionType` | Condition | in: type(blackboard), match | 임무타입 분기 (latch 적용) |
| `LatchMissionType` | Decorator | in: topic, lock_phases | 임계단계 중 타입전환 금지 |
| `PublishPhase` | Action | in: phase | `/swarm/mission_phase` 방송 |
| `SwarmFlockingStart` | Action | in: formation, spacing, centroid | `SwarmFlocking.action` 활성(래퍼) |
| `SwarmFlockingStop` | Action | — | deactivate / `stopFollowReference` |
| `DriveCentroid` | Action | in: target, speed | centroid를 목표로 구동 + 도착 SUCCESS |
| `ModifyFormation` | Action | in: formation | `modify_swarm`(재편대/detach) |

### 6.2 동기 barrier (드론간 — BT 비-native, 집계토픽 폴링)
| 노드 | 유형 | 조건 |
|---|---|---|
| `AwaitAllAirborne` | Condition | 전 드론 `platform/info.armed` + 고도≥임계 |
| `AwaitFormed` | Condition | 전 DroneSwarm `checkPosition`(0.3m) |
| `AwaitAllZonesDone` | Condition | 전 드론 WORK 완료(완료집계) |
| `AwaitAllLanded` | Condition | 전 드론 landed/disarmed |
| `SwarmSequence` | Control | 자식을 드론 순번대로 1기씩 실행 |

### 6.3 per-drone 게이트/슬롯
| 노드 | 유형 | 역할 |
|---|---|---|
| `PhaseIs` | Condition | `/swarm/mission_phase` == match |
| `WaitTakeoffSlot` | Decorator | 자기 이륙 순번까지 RUNNING |
| `WaitLandSlot` | Decorator | 자기 착륙 순번까지 RUNNING |
| `SafeHover` | Action | 항상 RUNNING 호버(최종 폴백) |

### 6.4 임무 P3 전용 (인식/측위)
| 노드 | 유형 | 임무 | 역할 |
|---|---|---|---|
| `DetectObjects` | Action | EOIR_RECON·SURVEIL | EO/IR 추론 behavior 래퍼 → `perception/detections` |
| `ObjectDetected` | Condition | EOIR_RECON·SURVEIL | 탐지 수신 → world좌표 출력 |
| `InspectTimeout` | Condition | EOIR_RECON | 정밀관측 경과 |
| `RfSurvey` | Action | RF_RECON | 스펙트럼 스캔 + DF/삼각측량 기동 → `rf/bearings` |
| `EmitterDetected` | Condition | RF_RECON | 방사원 탐지 → 추정 위치 출력 |
| `HasRole` | Condition | TRACK | TRACKER / RELAY 분기 (L3) |
| `TargetLost` | Condition | TRACK | 표적 소실 → 재탐색/복귀 트리거 |

> barrier 노드가 읽는 집계상태(`AwaitAll*`)는 **백그라운드 집계 서비스**가 공급. BT는 흐름제어, 무거운 상태/계산은 서비스. (FSM이든 BT든 이 서비스는 필요 — 추가비용 아님.)

---

## 7. phase 게이트 동기

```
coordinator SequenceStar:
   Cx_SequentialTakeoff → [PublishPhase TRANSIT] → Cx_FormUp → Cx_DriveCentroid(entry)
   → [PublishPhase WORK] → <임무>P3Coord (배포+완료barrier)
   → [PublishPhase REGROUP] → Cx_Regroup → Cx_DriveCentroid(home)
   → [PublishPhase LANDING] → Cx_SequentialLand

per-drone ReactiveFallback (매tick /swarm/mission_phase 재평가):
   PhaseIs TAKEOFF  → Dx_ArmTakeoff
   PhaseIs TRANSIT  → Dx_HoldFormation        (FollowReference)
   PhaseIs WORK     → <임무>P3Drone
   PhaseIs REGROUP  → Dx_HoldFormation
   PhaseIs LANDING  → Dx_LandSlot
   else             → Dx_SafeHover
```

- coordinator가 단계 전이를 `PublishPhase`로 방송 → per-drone가 반응형 전환.
- phase는 단조 진행(SequenceStar 메모리). 단 alert 선점은 phase 무관 최상위.

---

## 8. 파일 구성 (include)

```
swarm_bt/
├─ swarm_mission_root.xml        # 루트 + 임무타입 selector (coordinator)
├─ swarm_perdrone_root.xml       # 루트 + 임무타입 selector (per-drone)
├─ chassis/
│   ├─ chassis_coord.xml         # Cx_* 서브트리
│   └─ chassis_drone.xml         # Dx_* 서브트리
├─ missions/
│   ├─ eoir_recon_coord.xml / eoir_recon_drone.xml
│   ├─ rf_recon_coord.xml    / rf_recon_drone.xml
│   ├─ surveil_coord.xml     / surveil_drone.xml
│   ├─ track_coord.xml       / track_drone.xml
│   └─ ...
└─ nodes_model.xml               # TreeNodesModel (Groot2)
```

BT.CPP `registerBehaviorTreeFromFile` 다중 등록 + `SubTree ID` 참조로 결합. 임무 추가 = `missions/<x>_coord.xml`+`<x>_drone.xml` 2파일 + selector 1줄.

---

## 9. Coordinator BT 골격 (XML)

```xml
<root main_tree_to_execute="SwarmCoord">
  <BehaviorTree ID="SwarmCoord">
    <Parallel success_threshold="1" failure_threshold="1">
      <Decorator ID="WaitForAlert" topic_name="alert_event">
        <SubTree ID="EmergencyAll"/>
      </Decorator>
      <Decorator ID="LatchMissionType" topic_name="swarm/mission_intent" lock_phases="WORK,LANDING">
        <ReactiveFallback>
          <ReactiveSequence>
            <Condition ID="HasMissionType" type="{mtype}" match="EOIR_RECON"/>
            <SubTree ID="EoirReconCoord" __shared_blackboard="true"/>
          </ReactiveSequence>
          <ReactiveSequence>
            <Condition ID="HasMissionType" type="{mtype}" match="RF_RECON"/>
            <SubTree ID="RfReconCoord" __shared_blackboard="true"/>
          </ReactiveSequence>
          <ReactiveSequence>
            <Condition ID="HasMissionType" type="{mtype}" match="SURVEIL"/>
            <SubTree ID="SurveilCoord" __shared_blackboard="true"/>
          </ReactiveSequence>
          <ReactiveSequence>
            <Condition ID="HasMissionType" type="{mtype}" match="TRACK"/>
            <SubTree ID="TrackCoord" __shared_blackboard="true"/>
          </ReactiveSequence>
          <SubTree ID="StandbyCoord"/>
        </ReactiveFallback>
      </Decorator>
    </Parallel>
  </BehaviorTree>

  <!-- 임무 coord = chassis + 임무 P3Coord 조립. EO/IR 정찰 예시 -->
  <BehaviorTree ID="EoirReconCoord">
    <SequenceStar>
      <SubTree ID="Cx_SequentialTakeoff"/>
      <Action ID="PublishPhase" phase="TRANSIT"/>
      <SubTree ID="Cx_FormUp" formation="line" spacing="5.0"/>
      <Action ID="DriveCentroid" target="{recon_entry}" speed="4.0"/>
      <Action ID="PublishPhase" phase="WORK"/>
      <SubTree ID="EoirReconP3Coord"/>      <!-- 임무 고유: 분할+배포+완료barrier -->
      <Action ID="PublishPhase" phase="REGROUP"/>
      <SubTree ID="Cx_Regroup" formation="line"/>
      <Action ID="DriveCentroid" target="{home}" speed="4.0"/>
      <Action ID="PublishPhase" phase="LANDING"/>
      <SubTree ID="Cx_SequentialLand"/>
    </SequenceStar>
  </BehaviorTree>

  <!-- EO/IR 정찰 P3Coord: 분할→경로→배포→탐지융합→완료집계 -->
  <BehaviorTree ID="EoirReconP3Coord">
    <SequenceStar>
      <Action ID="PartitionAndAssign" zones="{recon_area}" route_out="swarm_agent/task"/>
      <Parallel success_threshold="1" failure_threshold="1">
        <Action ID="FuseDetections" in="perception/detections" out="swarm/targets"/>
        <Condition ID="AwaitAllZonesDone"/>   <!-- 전구역 완료 시 SUCCESS → Parallel 종료 -->
      </Parallel>
    </SequenceStar>
  </BehaviorTree>

  <!-- RF 정찰 P3Coord: 측위 baseline 편대 유지(WORK도 편대) + emitter 삼각측량 융합 -->
  <BehaviorTree ID="RfReconP3Coord">
    <SequenceStar>
      <Action ID="AssignRfSurvey" route_out="swarm_agent/task" mode="triangulate"/>
      <Parallel success_threshold="1" failure_threshold="1">
        <Action ID="FuseEmitters" in="rf/bearings" out="swarm/emitters"/>   <!-- 방위 → 삼각측량 -->
        <Condition ID="AwaitAllZonesDone"/>
      </Parallel>
    </SequenceStar>
  </BehaviorTree>
  <!-- 주의: RF는 WORK phase에서 SwarmFlockingStop 생략(편대 유지). DriveCentroid가 측위 경로 구동 -->

  <!-- 추적 P3Coord: 반응형 역할 배정(allocator 서비스) + 표적 핸드오프 -->
  <BehaviorTree ID="TrackP3Coord">
    <KeepRunningUntilFailure>
      <Action ID="AllocateTrackRoles" target_in="swarm/targets" role_out="swarm_agent/task"/>
    </KeepRunningUntilFailure>
  </BehaviorTree>
</root>
```

> `RfReconCoord` / `SurveilCoord` / `TrackCoord`는 `EoirReconCoord`와 **동일 chassis 골격**(이륙→편대→DriveCentroid→[P3Coord]→재편대→DriveHome→착륙)에서 가운데 `EoirReconP3Coord`만 각 `RfReconP3Coord` / `SurveilP3Coord` / `TrackP3Coord`로 교체. RF/감시는 WORK phase에서 편대 유지면 `SwarmFlockingStop` 생략. 추적은 phase 골격이 짧고(WORK가 지속 추종) `TrackP3Coord`가 KeepRunning 반응형.

---

## 10. Per-drone BT 골격 (XML)

```xml
<root main_tree_to_execute="SwarmDrone">
  <BehaviorTree ID="SwarmDrone">
    <Parallel success_threshold="1" failure_threshold="1">
      <Decorator ID="WaitForAlert" topic_name="alert_event">
        <SubTree ID="Dx_Emergency"/>
      </Decorator>
      <ReactiveFallback>
        <ReactiveSequence>
          <Condition ID="HasMissionType" type="{mtype}" match="EOIR_RECON"/>
          <SubTree ID="EoirReconDrone" __shared_blackboard="true"/>
        </ReactiveSequence>
        <ReactiveSequence>
          <Condition ID="HasMissionType" type="{mtype}" match="RF_RECON"/>
          <SubTree ID="RfReconDrone" __shared_blackboard="true"/>
        </ReactiveSequence>
        <ReactiveSequence>
          <Condition ID="HasMissionType" type="{mtype}" match="SURVEIL"/>
          <SubTree ID="SurveilDrone" __shared_blackboard="true"/>
        </ReactiveSequence>
        <ReactiveSequence>
          <Condition ID="HasMissionType" type="{mtype}" match="TRACK"/>
          <SubTree ID="TrackDrone" __shared_blackboard="true"/>
        </ReactiveSequence>
        <SubTree ID="Dx_SafeHover"/>
      </ReactiveFallback>
    </Parallel>
  </BehaviorTree>

  <!-- EO/IR 정찰 per-drone: phase 게이트 (RF/감시/추적도 동일 게이트 골격, WORK만 상이) -->
  <BehaviorTree ID="EoirReconDrone">
    <ReactiveFallback>
      <ReactiveSequence>
        <Condition ID="PhaseIs" phase="TAKEOFF"/>
        <SubTree ID="Dx_ArmTakeoff" height="20.0" speed="2.0"/>
      </ReactiveSequence>
      <ReactiveSequence>
        <Condition ID="PhaseIs" phase="TRANSIT"/>
        <SubTree ID="Dx_HoldFormation"/>      <!-- FollowReference 슬롯추종 -->
      </ReactiveSequence>
      <ReactiveSequence>
        <Condition ID="PhaseIs" phase="WORK"/>
        <SubTree ID="EoirReconP3Drone"/>      <!-- 임무 고유 -->
      </ReactiveSequence>
      <ReactiveSequence>
        <Condition ID="PhaseIs" phase="REGROUP"/>
        <SubTree ID="Dx_HoldFormation"/>
      </ReactiveSequence>
      <ReactiveSequence>
        <Condition ID="PhaseIs" phase="LANDING"/>
        <SubTree ID="Dx_LandSlot"/>
      </ReactiveSequence>
      <SubTree ID="Dx_SafeHover"/>
    </ReactiveFallback>
  </BehaviorTree>

  <!-- EO/IR 정찰 P3Drone: 독립 커버리지 + 탐지 (recon_mission_example.xml 본체) -->
  <BehaviorTree ID="EoirReconP3Drone">
    <ReactiveFallback>
      <ReactiveSequence>
        <Condition ID="ObjectDetected" topic_name="perception/detections"
                   min_score="0.6" target_pose="{detection_pose}"/>
        <SubTree ID="InspectTarget"/>
      </ReactiveSequence>
      <Parallel success_threshold="1" failure_threshold="3">
        <Action ID="FollowPath" path="{sub_route}" speed="3.0" yaw_mode="1"/>
        <KeepRunningUntilFailure>
          <Action ID="PointGimbal" frame_id="earth" mode="2" pitch="1.57"/>
        </KeepRunningUntilFailure>
        <KeepRunningUntilFailure>
          <Action ID="DetectObjects" server_name="DetectObjectsBehavior" min_score="0.5"/>
        </KeepRunningUntilFailure>
      </Parallel>
    </ReactiveFallback>
  </BehaviorTree>

  <!-- RF 정찰 P3Drone: WORK도 편대(FollowReference) 유지 + 스펙트럼 스캔/DF -->
  <BehaviorTree ID="RfReconP3Drone">
    <Parallel success_threshold="1" failure_threshold="2">
      <SubTree ID="Dx_HoldFormation"/>                       <!-- 측위 baseline 편대 -->
      <KeepRunningUntilFailure>
        <Action ID="RfSurvey" route="{sub_route}" mode="triangulate"/>  <!-- → rf/bearings -->
      </KeepRunningUntilFailure>
    </Parallel>
  </BehaviorTree>

  <!-- 감시 P3Drone: 거점 loiter 순환 + 탐지 (EO/IR 인식 재사용) -->
  <BehaviorTree ID="SurveilP3Drone">
    <Parallel success_threshold="1" failure_threshold="3">
      <KeepRunningUntilFailure>
        <Action ID="FollowPath" path="{loiter_route}" speed="1.0" yaw_mode="1"/>
      </KeepRunningUntilFailure>
      <KeepRunningUntilFailure>
        <Action ID="PointGimbal" frame_id="earth" mode="2" pitch="1.57"/>
      </KeepRunningUntilFailure>
      <KeepRunningUntilFailure>
        <Action ID="DetectObjects" server_name="DetectObjectsBehavior" min_score="0.5"/>
      </KeepRunningUntilFailure>
    </Parallel>
  </BehaviorTree>

  <!-- 추적 P3Drone: L3 역할 selector (TRACKER 추종 / RELAY 중계) -->
  <BehaviorTree ID="TrackP3Drone">
    <ReactiveFallback>
      <ReactiveSequence>
        <Condition ID="HasRole" role="{role}" match="TRACKER"/>
        <Parallel success_threshold="1" failure_threshold="2">
          <Action ID="GoTo" pose="{track_target}" max_speed="4.0" yaw_mode="1"/>
          <KeepRunningUntilFailure>
            <Action ID="PointGimbal" frame_id="earth" mode="2" pitch="1.0"/>
          </KeepRunningUntilFailure>
        </Parallel>
      </ReactiveSequence>
      <ReactiveSequence>
        <Condition ID="HasRole" role="{role}" match="RELAY"/>
        <SubTree ID="Dx_HoldFormation"/>                     <!-- 중계 위치 유지 -->
      </ReactiveSequence>
      <SubTree ID="Dx_SafeHover"/>
    </ReactiveFallback>
  </BehaviorTree>
</root>
```

---

## 11. 안전 설계

| 항목 | 메커니즘 |
|---|---|
| **선점 일관** | 루트 Parallel 최상위 `WaitForAlert→Emergency`. 전 임무 공통, 임무타입 무관 |
| **임무타입 latch** | `LatchMissionType lock_phases="WORK,LANDING"` — 임계단계 중 타입전환 금지(flap 방지). alert는 latch 우회 |
| **phase 단조성** | coordinator SequenceStar 메모리 → 단계 역행 없음 |
| **트리 사망 방지** | per-drone 최종 `Dx_SafeHover`(항상 RUNNING), coordinator barrier 타임아웃 |
| **blackboard 격리** | 임무 서브트리간 키 네임스페이스 분리, `__shared_blackboard` 신중 |
| **모드전환 deconfliction** | DISBAND/REGROUP 시 고도층 분리 + 순번 + 최근접 슬롯 매칭 |
| **barrier 교착 방지** | `AwaitAll*` 타임아웃 + 낙오기 제외 정책 |

---

## 12. 재활용 vs 신규

| 구분 | 항목 | 근거 |
|---|---|---|
| 재활용 | `SwarmFlocking`/`FollowReference`/`DroneSwarm` | `as2_behaviors_swarm_flocking/` |
| 재활용 | Arm/Offboard/Takeoff/GoTo/FollowPath/Land/PointGimbal BT 노드 | `as2_behavior_tree/`, `as2_behaviors_motion/` |
| 재활용 | `WaitForAlert` 선점 | `as2_behavior_tree/decorator/wait_for_alert.hpp` |
| 재활용 | recon P3Drone 본체 | `recon_mission_example.xml` |
| 신규 | chassis 서브트리 Cx_*/Dx_* | §4 |
| 신규 | 커스텀 BT 노드(조율/barrier/게이트) | §6 |
| 신규 | 집계 서비스(airborne/formed/zonesdone/landed) | barrier 공급 |
| 신규 | 임무별 P3Coord/P3Drone | §5 |
| 신규 | DetectObjects/detection_fusion/partitioner/coverage | 정찰 P3 |

---

## 13. 구현 로드맵

| M | 범위 | 산출 |
|---|---|---|
| M1 | 커스텀 BT 노드 골격(§6) + nodes_model | HasMissionType/PhaseIs/PublishPhase/SwarmFlockingStart/Await* |
| M2 | chassis 서브트리(§4) + 집계서비스 | Cx_*/Dx_* + airborne/formed/landed barrier |
| M3 | coordinator/per-drone 루트 + phase 게이트(§7,9,10) | 빈 P3로 전 단계 순차 Sim |
| M4 | 정찰 P3(분할/coverage/DetectObjects/fusion) | recon 풀 미션 Sim |
| M5 | 2번째 임무(매핑 또는 배송) P3 | dispatch 일반성 검증 |
| M6 | 모드전환 deconfliction + latch + barrier 타임아웃 | 안전 시험 |
| M7 | 실기(EO/IR HW, MAVLink) | 야외 |

**핵심 검증점(M5)**: 2번째 임무가 chassis 무수정 + P3 2파일 추가만으로 동작하면 프레임워크 일반성 입증.

---

## 14. 결론

- **단일 루트 BT + 임무타입 selector + chassis 서브트리** = AS2 BT-native에 정합, 효율(비활성 분기 미tick) + 안전(정적검증·반응선점·latch).
- **임무 추가 비용 = P3Coord/P3Drone 2파일 + selector 1줄.** chassis·조율노드·집계서비스는 1회 구축 후 전 임무 공유.
- **2계층 불가피**: coordinator BT(단계·편대·동기) + per-drone BT(phase 실행). 실코드 위상 반영.
- **신규 본질 = chassis + 커스텀 조율노드 + 집계서비스.** 비행동작은 거의 재활용.
- **최우선**: M1 노드 → M3 골격(빈 P3) → M4 정찰 → M5 2번째 임무로 일반성 검증.

---

_근거 소스: `as2_behavior_tree/`(BT 노드·wait_for_alert), `as2_behaviors_swarm_flocking/`(SwarmFlocking/DroneSwarm), `as2_behaviors_motion/`, `as2_msgs/action/{SwarmFlocking,FollowReference}.action`, `recon_mission_example.xml`, `AS2_RECON_SWARM_PHASED_DESIGN.md`, `AS2_RECON_BEHAVIOR_DESIGN.md`._
