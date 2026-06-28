# 군집 정찰 시스템 — 모듈간 동작 시퀀스

> **기반**: `AS2_SWARM_MODULE_DECOMPOSITION.md` (M01~M26), `AS2_SWARM_SCENARIO_BY_PHASE.md`  
> **목적**: P0~P5 전 구간의 모듈간 ROS2 인터페이스(토픽·서비스·액션·TF) 교환 순서를 명시.  
> **표기**: `-)` publish(비동기), `->>` / `-->>` service·action 요청/응답, `TF` = tf2 broadcast.

---

## 참여자 범례

| 약어 | 모듈 | 역할 |
|------|------|------|
| GCS | M24 gcs_mission_iface | 임무 업로드 (off-board) |
| HM | M02 health_monitor | FCU·센서 자가진단 (per-drone) |
| HB | M03 heartbeat | 생존 발행·생존맵 (per-drone) |
| EL | M04 election | 리더 선출·승계 (per-drone) |
| REG | M05 registry | 합류·등록부·정족수 (리더 전용) |
| AGG | M08 aggregator | barrier 상태 집계 (리더 전용) |
| ALLOC | M09 allocator | 구역분할·배정·재할당 (리더 전용) |
| CEN | M07 centroid_driver | virtual_centroid TF 구동 (리더 전용) |
| DFUSE | M11 detection_fusion | 탐지 dedup·트랙 (리더 전용) |
| RFUSE | M12 emitter_fusion | RF 삼각측량 (리더 전용) |
| DETECT | M13~M15 detect_objects | EO/IR 추론·지오로케이션 (per-drone) |
| RF | M16 rf_survey | RF 스캔·방위 (per-drone) |
| FLOCK | AS2 SwarmFlocking | 편대 슬롯 구동 (AS2 재사용) |
| COORD | BT:SwarmCoord | 리더 BT 트리 (M22·M23) |
| DRONE | BT:SwarmDrone[i] | 팔로워 BT 트리 (M22·M23) |
| AS2 | AS2 behaviors | Takeoff·Land·GoTo·FollowPath (재사용) |

---

## 전체 구간 흐름

```
P0 부트스트랩 ──► P1 순차 이륙 ──► P2 편대 Transit ──► P3 독립정찰 ──► P4 복귀 ──► P5 순차 착륙
  ★2 게이트                                                  ★1·★3 게이트
  (자율 개시)                                              (단일기·군집 정찰)
```

**구간별 핵심 모듈 활성 매핑**

| 구간 | 신규 활성 모듈 | 트리거 → 완료 조건 |
|------|---------------|--------------------|
| P0 | M02·M03·M04·M05·M09·M21·M24 | 부팅 → MissionReady ★2 |
| P1 | M08·M18·M19·M22 | PublishPhase(TAKEOFF) → AwaitAllAirborne |
| P2 | M07·M17·M18 (FLOCK) | PublishPhase(TRANSIT) → centroid@entry |
| P3 | M06·M09·M10·M11·M12·M13~M16·M20 | PublishPhase(WORK) → AwaitAllZonesDone |
| P4 | M07·M08·M17·M18 | PublishPhase(REGROUP) → centroid@home |
| P5 | M08·M18·M19·M22 | PublishPhase(LANDING) → AwaitAllLanded |

---

## 시퀀스 1 — P0 부트스트랩

> 트리거: 전원 인가  
> 완료 조건: ★2 게이트 통과 (MissionReady SUCCESS) → P1 진입  
> 전 드론이 동일 `BootstrapAll` 서브트리 실행. IsLeader로 리더/팔로워 자기선택.

```mermaid
sequenceDiagram
  participant GCS
  participant HM as HM (M02)
  participant HB as HB (M03)
  participant EL as EL (M04)
  participant REG as REG (M05)
  participant ALLOC as ALLOC (M09)
  participant BT as BT:BootstrapAll (M21)
  participant COORD as BT:SwarmCoord

  Note over HM,BT: 전 드론 병렬 실행

  HM->>HM: FCU·센서·TF·배터리 폴링
  HM-->>HB: health enum 콜백 (OK/DEGRADED/FAIL)

  loop 10 Hz — P0~P5 전 구간 지속
    HB-)HB: /swarm/heartbeat 발행 (자기)
    HB->>HB: 타 드론 heartbeat 수신 → 생존맵 갱신
  end

  HB-->>EL: 생존맵 내부 API 공유
  EL->>EL: min(alive_drone_id) 선출
  EL-)EL: /swarm/leader_id 발행

  Note over BT: ① AwaitBoot — health OK/DEGRADED 대기
  BT->>HM: health 폴링
  HM-->>BT: OK → SUCCESS

  Note over BT: ② ReportHealth — heartbeat 노드에 health 갱신
  BT-)HB: health 상태 전달

  Note over REG: IsLeader=true 드론만 — SwarmJoin.srv 서버 기동
  REG->>REG: /swarm/leader_id 구독 → 자기 ns 일치 확인

  Note over BT: ③ SwarmJoin — 리더 registry에 합류 요청
  BT->>REG: SwarmJoin.srv req {drone_id, health}
  REG-->>BT: res {accepted=true, registry[], mission_version}
  REG-)REG: /swarm/registry 발행 (등록부 배열)
  REG->>REG: 등록 수 N ≥ N_min → QuorumReady = true

  Note over BT: ④ QuorumReady — 정족수 플래그 폴링
  BT->>REG: QuorumReady 확인
  REG-->>BT: true → SUCCESS

  GCS->>GCS: 임무 YAML 파싱
  GCS-)ALLOC: /swarm/mission_intent 발행

  ALLOC->>ALLOC: takeoff_slot·land_slot 번호 배정 (drone_id 순)
  ALLOC-)BT: /{ns}/swarm_agent/task 발행 (slot 정보 포함)

  Note over BT: ⑤ MissionReady — MissionIntent 수신 + QuorumReady 동시 충족
  BT->>BT: BootstrapAll SUCCESS → ★2 게이트 통과

  Note over BT,COORD: IsLeader 자기선택
  BT->>EL: /swarm/leader_id 구독
  EL-->>BT: leader_id == self_ns → IsLeader=true

  Note over COORD: 리더 → SwarmCoord 진입
  COORD->>COORD: LatchMissionType → EOIR_RECON 확인
  COORD->>COORD: Cx_SequentialTakeoff 진입
  COORD-)COORD: PublishPhase(TAKEOFF) → /swarm/mission_phase

  Note over BT: 팔로워 → SwarmDrone 진입 (PhaseIs(TAKEOFF) 대기)
```

---

## 시퀀스 2 — P1 순차 이륙

> 트리거: PublishPhase(TAKEOFF)  
> 완료 조건: AwaitAllAirborne SUCCESS → PublishPhase(TRANSIT) → P2 진입

```mermaid
sequenceDiagram
  participant COORD as BT:SwarmCoord (COORD)
  participant DRONE as BT:SwarmDrone[i]
  participant AGG as AGG (M08)
  participant AS2 as AS2 Behaviors

  COORD-)COORD: /swarm/mission_phase = TAKEOFF 발행

  Note over COORD: Cx_SequentialTakeoff — AwaitAllAirborne 대기

  loop 전 드론 순서대로
    Note over DRONE: PhaseIs(TAKEOFF) = true → Dx_ArmTakeoff 진입
    Note over DRONE: WaitTakeoffSlot — takeoff_slot 순번 대기
    DRONE->>DRONE: swarm_agent/task.takeoff_slot 확인
    DRONE->>AS2: set_arming_state.srv (Arm)
    AS2-->>DRONE: armed=true
    DRONE->>AS2: set_offboard_mode.srv (Offboard)
    AS2-->>DRONE: offboard=true
    DRONE->>+AS2: Takeoff.action goal {height=20.0, speed=2.0}
    AS2-->>-DRONE: result SUCCESS (고도 도달)
  end

  loop 지속 모니터링
    AGG->>AGG: /{ns}/platform/info + /self_localization/pose 수집
    AGG->>AGG: 전 드론 고도 임계 판정
    AGG-)AGG: /swarm/barrier {airborne: bool} 발행
  end

  Note over COORD: AwaitAllAirborne — /swarm/barrier.airborne 구독
  COORD->>AGG: /swarm/barrier 폴링
  AGG-->>COORD: airborne=true → SUCCESS

  COORD-)COORD: PublishPhase(TRANSIT) 발행
  Note over COORD: P2 진입
```

---

## 시퀀스 3 — P2 편대 Transit (FORM_UP → TRANSIT)

> 트리거: PublishPhase(TRANSIT)  
> 완료 조건: centroid@recon_entry 도달 → PublishPhase(WORK) → P3 진입

```mermaid
sequenceDiagram
  participant COORD as BT:SwarmCoord
  participant DRONE as BT:SwarmDrone[i]
  participant FLOCK as AS2:SwarmFlocking
  participant CEN as CEN (M07)
  participant AGG as AGG (M08)

  COORD-)COORD: /swarm/mission_phase = TRANSIT 발행

  Note over DRONE: PhaseIs(TRANSIT)=true → Dx_HoldFormation 진입

  Note over COORD: Cx_FormUp — SwarmFlockingStart 실행
  COORD->>+FLOCK: SwarmFlocking.action goal {formation=line, spacing=5.0, drones[]}
  Note over FLOCK: 드론별 슬롯 오프셋 계산 → TF 등록

  loop 편대 수렴 중
    FLOCK->>+DRONE: FollowReference.action goal {slot_frame}
    DRONE-->>-FLOCK: feedback (position error)
    FLOCK->>AGG: 각 드론 슬롯 오차 전달
    AGG->>AGG: checkPosition(0.3m) 전 드론 판정
    AGG-)AGG: /swarm/barrier {formed: bool} 발행
  end

  COORD->>AGG: /swarm/barrier 폴링
  AGG-->>COORD: formed=true → AwaitFormed SUCCESS

  Note over COORD: DriveCentroid — centroid_driver 에 목표 전달
  COORD->>CEN: DriveCentroid(target=recon_entry, speed=4.0)
  CEN->>CEN: 현 centroid → recon_entry 선형 보간 시작

  loop 50 Hz — centroid 이동 중
    CEN-)CEN: TF broadcast earth→swarm/virtual_centroid
    FLOCK->>FLOCK: centroid TF 참조 → 슬롯 절대 위치 갱신
    FLOCK->>DRONE: FollowReference 슬롯 갱신 → 편대 강체 이동
  end

  CEN->>CEN: dist(centroid, recon_entry) < 임계 → 도달
  CEN-->>COORD: DriveCentroid SUCCESS

  FLOCK-->>-COORD: SwarmFlocking.action 유지 (RUNNING)
  COORD-)COORD: PublishPhase(WORK) 발행
  Note over COORD: P3 진입
```

---

## 시퀀스 4 — P3 EO/IR 독립정찰 (DISBAND → RECON → FUSE)

> 트리거: PublishPhase(WORK) — ★1(단일기)·★3(군집) 게이트  
> 완료 조건: AwaitAllZonesDone SUCCESS → PublishPhase(REGROUP) → P4 진입

### 4-a. DISBAND + 구역 배정

```mermaid
sequenceDiagram
  participant COORD as BT:SwarmCoord
  participant DRONE as BT:SwarmDrone[i]
  participant FLOCK as AS2:SwarmFlocking
  participant ALLOC as ALLOC (M09)
  participant ZP as zone_partitioner (M06)
  participant CP as coverage_planner (M10)

  COORD->>FLOCK: SwarmFlockingStop (M17)
  FLOCK->>DRONE: FollowReference.action cancel (x N)
  DRONE->>DRONE: 현 위치 호버

  Note over COORD: EoirReconP3Coord — PartitionAndAssign 실행
  COORD->>ALLOC: PartitionAndAssign {zones=recon_area, N=드론수}

  ALLOC->>ZP: zone_partitioner(polygon, N)
  ZP-->>ALLOC: sub_zones[] (균등 분할)

  loop 드론별
    ALLOC->>CP: coverage_planner(sub_zone[i], swath_width, overlap)
    CP-->>ALLOC: sub_route[i] (boustrophedon 경로)
  end

  ALLOC-)DRONE: /{ns}/swarm_agent/task 발행 {zone, sub_route, classes, role}
  Note over DRONE: task 수신 → sub_route, detection classes 블랙보드 저장
```

### 4-b. 독립 커버리지 + 탐지 (per-drone 병렬)

```mermaid
sequenceDiagram
  participant DRONE as BT:SwarmDrone[i]
  participant AS2 as AS2:GoTo/FollowPath
  participant DETECT as DETECT (M13~M15)
  participant DFUSE as DFUSE (M11)
  participant COORD as BT:SwarmCoord
  participant AGG as AGG (M08)

  Note over DRONE: PhaseIs(WORK)=true → EoirReconP3Drone 진입

  DRONE->>+AS2: GoTo.action goal {entry_point}
  AS2-->>-DRONE: SUCCESS (진입점 도달)

  Note over DRONE: Parallel — FollowPath + PointGimbal + DetectObjects

  DRONE->>+AS2: FollowPath.action goal {sub_route, speed=3.0}
  DRONE->>+DETECT: DetectObjects.action goal {classes, min_score=0.5}
  DRONE->>AS2: PointGimbal (하향 고정)

  loop 커버리지 중
    DETECT->>DETECT: EO/IR 이미지 수신
    DETECT->>DETECT: ONNX/TensorRT 추론 → bbox+class+score (M14)
    DETECT->>DETECT: ray-ground 교차 → world_pose 지오로케이션 (M15)
    DETECT-)DETECT: /{ns}/perception/detections 발행 (DetectionArray)

    DFUSE->>DFUSE: 전 드론 /perception/detections 수집
    DFUSE->>DFUSE: world좌표 dedup (거리 임계 내 병합)
    DFUSE->>DFUSE: 트랙 수명관리 (T_track 미관측 → 삭제)
    DFUSE-)DFUSE: /swarm/targets 발행 (TargetTrackArray)

    alt 표적 발견 시 (ObjectDetected)
      DRONE->>DRONE: /perception/detections 구독 → min_score 초과 확인
      DRONE->>DRONE: InspectTarget 선점
      DRONE->>AS2: GoTo.action goal {detection_pose}
      Note over DRONE: 15초 타임아웃 → 커버리지 복귀
    end
  end

  AS2-->>-DRONE: FollowPath SUCCESS (sub_route 완료)
  DETECT-->>-DRONE: DetectObjects cancel

  DRONE-)AGG: zone 완료 신호 (FollowPath SUCCESS)
  AGG->>AGG: zonesdone 배열 갱신
  AGG-)AGG: /swarm/barrier {zonesdone: bool[]} 발행

  COORD->>AGG: /swarm/barrier 폴링 (AwaitAllZonesDone)
  AGG-->>COORD: 전 드론 zonesdone=true → SUCCESS

  COORD-)COORD: PublishPhase(REGROUP) 발행
  Note over COORD: P4 진입
```

---

## 시퀀스 5 — P3 RF 방사원 탐지 (선택 임무)

> RF_RECON 임무 타입 시 활성. EO/IR 커버리지와 병행 또는 대체.

```mermaid
sequenceDiagram
  participant COORD as BT:SwarmCoord
  participant DRONE as BT:SwarmDrone[i]
  participant RF as RF (M16)
  participant RFUSE as RFUSE (M12)
  participant CEN as CEN (M07)

  Note over COORD: RfReconP3Coord — AssignRfSurvey

  COORD->>COORD: AssignRfSurvey {mode=triangulate}
  COORD-)DRONE: /{ns}/swarm_agent/task {rf_route, mode=triangulate}

  Note over COORD: Parallel — DriveCentroid + FuseEmitters + AwaitAllZonesDone

  COORD->>CEN: DriveCentroid(target=rf_survey_route, speed=2.0)

  loop 50 Hz
    CEN-)CEN: TF broadcast earth→virtual_centroid (측위 경로 이동)
  end

  loop RF 스캔 중 (per-drone)
    Note over DRONE: PhaseIs(WORK)=true → RfReconP3Drone 진입
    DRONE->>RF: RfSurvey.action goal {sub_route, mode=triangulate}
    RF->>RF: 주파수 스윕 → 피크 감지 (스캔)
    RF->>RF: 위상 비교 → 방위각 산출 (DF)
    RF-)RF: /{ns}/rf/bearings 발행 (RfBearing)

    RFUSE->>RFUSE: N드론 /rf/bearings 수집
    RFUSE->>RFUSE: baseline 거리 확인 (최소거리 미달 시 보류)
    RFUSE->>RFUSE: 2D 최소제곱 삼각측량 (LS/MLE)
    RFUSE->>RFUSE: 트랙 수명관리
    RFUSE-)RFUSE: /swarm/emitters 발행 (EmitterTrackArray)
  end
```

---

## 시퀀스 6 — P4 재편대 · 복귀 (REGROUP → RETURN)

> 트리거: PublishPhase(REGROUP)  
> 완료 조건: centroid@home 도달 → Cx_SequentialLand → P5 진입

```mermaid
sequenceDiagram
  participant COORD as BT:SwarmCoord
  participant DRONE as BT:SwarmDrone[i]
  participant FLOCK as AS2:SwarmFlocking
  participant CEN as CEN (M07)
  participant AGG as AGG (M08)

  COORD-)COORD: /swarm/mission_phase = REGROUP 발행

  Note over DRONE: PhaseIs(REGROUP)=true → Dx_HoldFormation 진입
  DRONE->>+FLOCK: FollowReference.action goal {slot_frame}

  Note over COORD: Cx_Regroup — ModifyFormation(line)
  COORD->>FLOCK: ModifyFormation {formation=line} (M17)
  FLOCK->>FLOCK: 현 드론 위치 → 최근접 슬롯 매칭 (헝가리안)
  Note over FLOCK: 고도층 분리 적용 (충돌 방지)

  loop 재편대 수렴 중
    FLOCK->>DRONE: FollowReference 슬롯 갱신
    AGG->>AGG: checkPosition(0.3m) 판정
    AGG-)AGG: /swarm/barrier {formed: bool} 발행
  end

  COORD->>AGG: AwaitFormed 폴링
  AGG-->>COORD: formed=true → SUCCESS

  Note over COORD: DriveCentroid — home으로 복귀
  COORD->>CEN: DriveCentroid(target=home, speed=4.0)

  loop 50 Hz
    CEN-)CEN: TF broadcast earth→virtual_centroid (home 방향)
    FLOCK->>FLOCK: centroid TF 참조 → 슬롯 위치 갱신
    DRONE-->>FLOCK: FollowReference 추종
  end

  CEN->>CEN: dist(centroid, home) < 임계 → 도달
  CEN-->>COORD: DriveCentroid SUCCESS
  FLOCK-->>-DRONE: FollowReference 유지

  Note over COORD: Cx_SequentialLand 진입
```

---

## 시퀀스 7 — P5 순차 착륙

> 트리거: Cx_SequentialLand (SwarmFlockingStop → PublishPhase(LANDING))  
> 완료 조건: AwaitAllLanded SUCCESS → PublishPhase(DONE)

```mermaid
sequenceDiagram
  participant COORD as BT:SwarmCoord
  participant DRONE as BT:SwarmDrone[i]
  participant FLOCK as AS2:SwarmFlocking
  participant AGG as AGG (M08)
  participant AS2 as AS2 Behaviors

  COORD->>FLOCK: SwarmFlockingStop
  FLOCK->>DRONE: FollowReference.action cancel (x N)
  DRONE->>DRONE: 현 위치 호버

  COORD-)COORD: /swarm/mission_phase = LANDING 발행

  loop 전 드론 순서대로
    Note over DRONE: PhaseIs(LANDING)=true → Dx_LandSlot 진입
    Note over DRONE: WaitLandSlot — land_slot 순번 대기
    DRONE->>DRONE: swarm_agent/task.land_slot 확인
    DRONE->>+AS2: Land.action goal {speed=0.5}
    AS2-->>-DRONE: SUCCESS (disarmed + 착지)
  end

  loop 착륙 상태 모니터링
    AGG->>AGG: /{ns}/platform/info.disarmed 수집
    AGG->>AGG: 전 드론 landed 판정
    AGG-)AGG: /swarm/barrier {landed: bool} 발행
  end

  COORD->>AGG: AwaitAllLanded 폴링
  AGG-->>COORD: landed=true → SUCCESS

  COORD-)COORD: /swarm/mission_phase = DONE 발행
  Note over COORD: 임무 완료
```

---

## 시퀀스 8 — E1 드론 이탈 (heartbeat 소실)

> 발생: 임의 구간에서 드론 i 통신 두절 또는 충돌

```mermaid
sequenceDiagram
  participant DEAD as DroneX (이탈)
  participant HB as HB (M03)
  participant EL as EL (M04)
  participant REG as REG (M05)
  participant AGG as AGG (M08)
  participant ALLOC as ALLOC (M09)
  participant COORD as BT:SwarmCoord (신규 리더)

  Note over DEAD: DroneX heartbeat 소실

  HB->>HB: T_dead 타임아웃 초과 → DroneX DEAD 처리
  HB-->>EL: 생존맵 갱신 (DroneX 제거)

  alt DroneX가 리더였던 경우
    EL->>EL: min(alive_id) 재계산 → 차순위 드론이 리더
    EL-)EL: /swarm/leader_id 갱신 발행
    Note over COORD: 신규 리더 드론 — IsLeader=true → SwarmCoord 진입
    COORD->>COORD: registry·aggregator·allocator 인스턴스 기동
  else DroneX가 팔로워였던 경우
    Note over EL: 기존 리더 유지
  end

  REG->>REG: 생존맵 기반 등록부 갱신 (DroneX 제거)
  REG-)REG: /swarm/registry 재발행

  AGG->>AGG: 등록부 기반 모니터 대상 갱신
  AGG->>AGG: 낙오기 제외 후 barrier 재판정
  AGG-)AGG: /swarm/barrier 갱신 발행

  alt P3 구간 중 이탈 — 미완료 구역 재할당
    ALLOC->>ALLOC: 남은 드론으로 DroneX.sub_zone 재배분
    ALLOC-)ALLOC: 인접 드론 /{ns}/swarm_agent/task 갱신 발행
  end

  Note over AGG: T_barrier 경과 또는 coverage율 >= 임계 → 낙오기 제외 후 barrier 통과 (E4 방지)
```

---

## 시퀀스 9 — E2 배터리 경고 (BatteryLow RTB)

> 발생: 임의 구간에서 드론 i 배터리 < 25%

```mermaid
sequenceDiagram
  participant HM as HM (M02)
  participant HB as HB (M03)
  participant DRONE as BT:SwarmDrone[i]
  participant EL as EL (M04)
  participant AS2 as AS2 Behaviors

  HM->>HM: 배터리 토픽 폴링 → level < threshold
  HM-->>HB: health = DEGRADED

  HB->>HB: 자기 Heartbeat.health 갱신
  HB-)HB: /swarm/heartbeat 발행 (DEGRADED 상태 포함)

  Note over DRONE: AwaitBatteryLow (BT 선점 Parallel 채널)
  DRONE->>DRONE: AwaitBatteryLow threshold=0.25 → 트리거

  Note over DRONE: Dx_BatteryRTB 선점 실행
  DRONE->>+AS2: GoTo.action goal {home, max_speed=4.0}
  AS2-->>-DRONE: SUCCESS

  DRONE->>+AS2: Land.action goal {speed=0.5}
  AS2-->>-DRONE: SUCCESS

  alt GoTo/Land 실패 시
    DRONE->>DRONE: SafeHover 폴백 (항상 RUNNING 유지)
  end

  Note over EL: DroneX DEGRADED → 생존 유지, 등록부 잔류
  Note over EL: 리더였다면 착륙 전 승계 가능 (E1 동일 로직)
```

---

## 시퀀스 10 — E4 barrier 교착 방지 (낙오기 타임아웃)

> 발생: 낙오기로 인해 barrier 조건 미충족 → 무한 대기 위험

```mermaid
sequenceDiagram
  participant COORD as BT:SwarmCoord
  participant AGG as AGG (M08)
  participant DRONE_OK as DroneA,B (정상)
  participant DRONE_SLOW as DroneC (낙오)

  Note over AGG: T_barrier 타이머 시작

  loop 정상 드론 barrier 조건 충족
    DRONE_OK-)AGG: 상태 정보 발행 (airborne/formed/zonesdone/landed)
    AGG->>AGG: DroneA,B 조건 확인 → 충족
  end

  loop 낙오 드론 조건 미충족
    DRONE_SLOW->>AGG: 응답 지연 또는 coverage 미완료
    AGG->>AGG: DroneC 조건 미충족 → barrier 대기
  end

  AGG->>AGG: T_barrier 경과 또는 coverage율 >= 80%
  AGG->>AGG: DroneC 낙오 판정 → 경고 로그 발행
  AGG->>AGG: 낙오기 제외 후 barrier 재계산
  AGG-)AGG: /swarm/barrier {airborne/formed/zonesdone/landed: true} 발행

  COORD->>AGG: barrier 폴링 (AwaitAll*)
  AGG-->>COORD: true → SUCCESS (낙오기 제외 통과)

  Note over COORD: 임무 계속 진행 (부분 완료 허용)
```

---

## 핵심 인터페이스 요약

### ROS2 토픽 (Publish / Subscribe)

| 토픽 | 발행자 | 구독자 | 메시지 타입 | 구간 |
|------|--------|--------|-------------|------|
| `/swarm/heartbeat` | M03 (전 드론) | M03·M04 (전 드론) | `Heartbeat` | 전체 |
| `/swarm/leader_id` | M04 (전 드론) | M21:IsLeader | `std_msgs/String` | P0~ |
| `/swarm/registry` | M05 (리더) | M08·M09 | `SwarmRegistry` | P0~ |
| `/swarm/mission_intent` | M24 (GCS) | M09·M21:MissionReady | `MissionIntent` | P0-5 |
| `/{ns}/swarm_agent/task` | M09 (리더) | M19·M22 BT 노드 | `SwarmTask` | P0~ |
| `/swarm/mission_phase` | M17:PublishPhase | M19:PhaseIs (전 드론) | `MissionPhase` | P1~ |
| `/swarm/barrier` | M08 (리더) | M18 BT 노드 | `SwarmBarrier` | P1~ |
| `/{ns}/perception/detections` | M13~M15 (per-drone) | M11 (리더) | `DetectionArray` | P3 |
| `/swarm/targets` | M11 (리더) | M09·M24 | `TargetTrackArray` | P3 |
| `/{ns}/rf/bearings` | M16 (per-drone) | M12 (리더) | `RfBearing` | P3-RF |
| `/swarm/emitters` | M12 (리더) | M24 | `EmitterTrackArray` | P3-RF |

### ROS2 서비스

| 서비스 | 서버 | 클라이언트 | 구간 |
|--------|------|------------|------|
| `swarm/join` | M05 registry (리더) | M21:SwarmJoin (전 드론) | P0-3 |
| `set_arming_state` | AS2 (per-drone) | M22:Dx_ArmTakeoff | P1 |
| `set_offboard_mode` | AS2 (per-drone) | M22:Dx_ArmTakeoff | P1 |

### ROS2 액션

| 액션 | 서버 | 클라이언트 | 구간 |
|------|------|------------|------|
| `SwarmFlocking.action` | AS2 SwarmFlocking | M17:SwarmFlockingStart | P2·P4 |
| `FollowReference.action` | AS2 (per-drone) | AS2 SwarmFlocking | P2·P3-RF·P4 |
| `Takeoff.action` | AS2 (per-drone) | M22:Dx_ArmTakeoff | P1 |
| `Land.action` | AS2 (per-drone) | M22:Dx_LandSlot | P5 |
| `GoTo.action` | AS2 (per-drone) | M22 BT | P3·P4·E1·E2 |
| `FollowPath.action` | AS2 (per-drone) | M20:FollowPath BT | P3 |
| `DetectObjects.action` | M13~M15 (per-drone) | M20:DetectObjects BT | P3 |

### TF 프레임

| broadcast | 발행자 | 소비자 | 구간 |
|-----------|--------|--------|------|
| `earth → swarm/virtual_centroid` | M07 centroid_driver (50 Hz) | AS2 SwarmFlocking | P2·P4 |
| `earth → {ns}/base_link` | AS2 상태추정 | M15:지오로케이션 | P3 |
| `earth → {ns}/camera_link` | AS2 상태추정 | M15:ray-ground | P3 |

---

## 모듈 활성 상태 타임라인

```
구간:    │  P0  │  P1  │  P2  │    P3    │  P4  │  P5  │
─────────┼──────┼──────┼──────┼──────────┼──────┼──────┤
M02 HM   │ ████ │ ████ │ ████ │   ████   │ ████ │ ████ │ (전 구간)
M03 HB   │ ████ │ ████ │ ████ │   ████   │ ████ │ ████ │ (전 구간)
M04 EL   │ ████ │ ████ │ ████ │   ████   │ ████ │ ████ │ (전 구간)
M05 REG  │ ████ │      │      │          │      │      │ (P0만)
M08 AGG  │      │ ████ │ ████ │   ████   │ ████ │ ████ │ (P1~P5)
M09 ALLOC│ ████ │      │      │   ████   │      │      │ (P0·P3)
M07 CEN  │      │      │ ████ │          │ ████ │      │ (P2·P4)
M10 CP   │ ████ │      │      │   ████   │      │      │ (P0·P3)
M11 DFUSE│      │      │      │   ████   │      │      │ (P3 EO/IR)
M12 RFUSE│      │      │      │   ████   │      │      │ (P3 RF)
M13~15   │      │      │      │   ████   │      │      │ (P3 EO/IR)
M16 RF   │      │      │      │   ████   │      │      │ (P3 RF)
```

---

_근거: `AS2_SWARM_MODULE_DECOMPOSITION.md`, `AS2_SWARM_SCENARIO_BY_PHASE.md`, `swarm_bt/swarm_bt_combined.xml`, `swarm_bt/chassis/chassis_drone.xml`._
