# swarm_drone_coordination_design 접목 분석 — AS2 충돌 + 순수 상위레이어 변환

**대상 문서**: `swarm_drone_coordination_design.md` (멀티클러스터 3리더, Raft, CBBA, ORCA, RAIM, CycloneDDS 도메인격리)
**기준 구현**: 현재 aerostack2 (소스 분석: `AS2_NODES_ANALYSIS.md`, topics.hpp, swarm_flocking, state_estimator, mavlink platform)
**비교 대상**: `swarm_intelligence_layer_spec_v1.md` (기존 설계)
**결론 요약**: 문서를 AS2 위 순수 상위레이어로 변환 **가능**. 단 협조측위 보정 1건만 불가(드롭/연기). 5개 알고리즘 교체 + 1 개념정정 + 1 설정 필요.

---

## 1. AS2 구현과의 충돌점 (코드 근거)

순수 상위레이어 정의: AS2 표준토픽 **읽기** + AS2 액션/서비스(FollowReference/GoTo/FollowPath/Takeoff/Land + platform srv)로만 **구동**. `motion_reference` 직접 발행·estimator 수정·DDS 도메인격리(액션 디스커버리 단절) 금지.

| # | 문제 | AS2 실제 | 등급 |
|---|---|---|---|
| 1 | ORCA/DWA/Formation 연속 twist → motion_reference 2-writer | `controller_handler.cpp` 단일 writer 전제, behavior→handler→motion_reference 체인 | 🔴 |
| 2 | 충돌회피 20ms 폐루프 | behavior=action server, 왕복 느림. 20ms 불가 | 🔴 |
| 3 | DDS 도메인 격리(10/20/30) + domain_bridge | swarm_flocking 리더가 멤버 액션클라이언트 직접호출=단일도메인 전제 | 🔴 |
| 4 | Raft 선출 | 선출 없음. Raft=CP→partition 시 블록, "링크끊겨도 임무계속" 위배 | 🔴 |
| 5 | GPS RAIM + 협조측위 | 측위=state_estimator 플러그인→self_localization. 추정단계 침투 | 🟠 |
| 6 | CBBA 분산 consensus | 손실 mesh서 미수렴. AS2 무관 | 🟠 |
| 7 | CycloneDDS 멀티홉 mesh 디스커버리 | reliable QoS(10) 다수 → 재전송폭주, discovery O(n²) | 🟠 |
| 8 | Formation/Planner/Localization 재발명 | swarm_flocking·path_planning(A*/Voronoi)·estimator 이미 존재 | 🟠 |
| 9 | MAVLink를 군집 transport로 오인 | mavlink platform=MAVROS 브리지, FCU 내부링크일 뿐 | 🟡 |
| 10 | QoS 미스매치 | self_localization/sensor=SensorDataQoS(best-effort) 고정 | 🟡 |

**침투 3종**(AS2 아래로 내려가는 것): (a) 제어경로 #1/#2, (b) 추정 #5, (c) 디스커버리 #3. 이 셋이 "순수 상위레이어" 전제를 깸 → 변환 필요.

---

## 2. 순수 상위레이어 변환표

| # | 재설계 | Trade-off |
|---|---|---|
| 1 | 충돌회피를 **이산 reference 조정**으로: FollowReference goal / GoTo waypoint / swarm_flocking 동적오프셋. motion_reference 직접 안 씀 | 타이트 속도제어 포기 |
| 2 | swarm층 = **예측분리(CPA/TCPA) 5~10Hz**. 근접 최후수단은 AS2 `alert_event` EMERGENCY_HOVER / FCU failsafe 위임 | 20ms를 FCU/platform로 내림 |
| 3 | 단일 도메인 + **DDS Partition/네임스페이스** 논리분리. 클러스터=네임스페이스. GCS만 edge bridge 1개(out-of-loop라 SPOF 허용) | 도메인격리 포기 |
| 4 | **lowest-alive-id**(spec D2). 선출은 이미 상위 — 알고리즘만 AP로 | 강일관성 포기(드론엔 가용성이 옳음) |
| 5 | **분할**: RAIM 탐지만 상위 모니터(gps 구독→degraded alert→RTB/역할전환). 보정은 PX4 EKF2 네이티브 GPS-loss DR | 협조측위 보정만 불가 ↓ |
| 6 | **중앙집중 할당**(리더 task_allocator 계산). 손실링크서 분산보다 견고 | 분산 자율성 포기 |
| 7 | CycloneDDS peer 명시 + best-effort 크로스드론 + discovery 클러스터 한정 (config, AS2 무관) | 멀티캐스트 자동탐색 끔 |
| 8 | **재사용**: swarm_flocking wrap, path_planning 액션, estimator 그대로 | RRT*/DWA/Wedge 신규구현 포기 |
| 9 | MAVLink=platform↔FCU 내부 한정. 군집=DDS | 없음(개념정정) |
| 10 | 구독측 QoS를 AS2 발행자(best-effort)에 매칭 | reliable 승격 불가 |

---

## 3. 유일한 불가분 — 협조측위 보정

이웃거리 융합으로 **위치 추정값 자체 변경** = estimator 입력. self_localization 위에서 불가.

| 선택지 | 내용 | 순수상위 |
|---|---|---|
| **1. 드롭(권장)** | PX4 EKF2 GPS-loss 내장 IMU DR로 충분. GPS유 실외 정찰엔 족함 | 유지 |
| 2. 별 워크스트림 | 신규 `as2_state_estimator` 플러그인(cooperative_positioning). 분리 프로젝트 | 아님 |
| 3. 하이브리드 | RAIM 탐지만 상위, 보정 플러그인 별도 | 부분 |

**결정: 1번 드롭.** 협조측위는 "향후 estimator 플러그인" 백로그. RAIM 탐지는 상위 모니터로 유지.

---

## 4. 결과 — 순수 상위레이어 아키텍처

```
GCS Layer (off-board)
  Mission Planner · Area Decomposition(Voronoi/Boustrophedon, 리더측 compute)
  ISR Fusion(GStreamer 별 파이프) · Replay&Log
Cluster/Swarm Layer (리더 드론)
  leader_election(lowest-alive-id) · task_allocator(중앙집중)
  formation_manager(swarm_flocking WRAP) · topology_monitor(링크품질 읽기)
  gps_integrity_monitor(RAIM 탐지만) · route_planner(path_planning 재사용)
Follower Layer (전 드론)
  follower_agent → AS2 액션(GoTo/FollowReference/Land) 구동
  separation_monitor(CPA/TCPA 예측, waypoint 조정) · health_reporter
  RTB → 기존 behaviors+alert_event
─────────────────────────────────
AS2 substrate (무변경)
  estimator(EKF2 GPS-loss DR) · controller · platform(mavros) · behaviors · swarm_flocking
근접 최후수단 회피 → AS2 alert_event / FCU failsafe (위임)
```

---

## 5. spec 노드표 비교 분석

`swarm_intelligence_layer_spec_v1.md` 노드 로스터와 문서 노드를 매핑.

### 5-1. 문서 노드 → spec 대응

| 문서 노드 | 레이어 | spec 대응 | 판정 |
|---|---|---|---|
| Mission Planner | GCS | `gcs_mission_iface` | 대응 |
| Area Decomposition | GCS | `swarm_route_planner`(AO 분할 일부) | 부분 — 보강 필요 |
| **ISR Fusion** | GCS | 없음 (spec=per-drone detection만) | **추가 후보** |
| **Replay & Log** | GCS | 부분(`mission_reporter`는 보고서, bag 아님) | **추가 후보** |
| Leader Election | Cluster | `leader_election` | 대응 (Raft→lowest-alive-id) |
| Formation Manager | Cluster | `formation_manager` | 대응 (wrap) |
| Task Allocator | Cluster | `task_allocator` | 대응 (CBBA→중앙집중) |
| **Topology Manager** | Cluster | 없음 | **추가 후보(모니터)** |
| **GPS Integrity Monitor** | Cluster | 없음 (safety=separation/geofence) | **추가 후보(RAIM 탐지)** |
| Relay (Store-and-Forward) | Cluster | 없음 | 추가 후보 or 드롭 |
| Local Path Planner | UAV | `swarm_route_planner` + AS2 `path_planning` 재사용 | 대응(재사용) |
| Collision Avoidance | UAV | `safety_monitor`+`local_replanner` | 대응(이산화) |
| Localization | UAV | AS2 estimator (substrate) | substrate |
| ISR Sensor | UAV | `sensor_processor` | 대응 |
| Health Reporter | UAV | `follower_agent`(health) | 대응 |
| Return-to-Base | UAV | `follower_agent` RTH + behaviors | 대응 |

### 5-2. spec-only (문서가 빠뜨린 것 — 문서 약점)

| spec 노드 | 역할 | 문서 결여 영향 |
|---|---|---|
| `swarm_coordinator` | registry/version sync/quorum | 멤버십·임무동기 메커니즘 부재 → 단절생존 불가 |
| `mission_manager` | 생명주기/종료조건 온보드평가 | 종료판정·자율중단 로직 없음 |
| `follower_agent` | 리더cmd→AS2 액션 브리지 + 캐시 | **AS2 접목 핵심이 통째 없음** |
| `tracking` | Kalman 표적추적 | detection만 있고 추적 없음 |
| `mission_reporter` | 보고 통합 | Replay&Log와 성격 다름 |

> 문서의 가장 큰 구조적 결함: **AS2와의 접합 메커니즘(follower_agent) 부재** + **단절생존 설계(coordinator/mission_manager) 부재**. 문서는 "노드 나열"이고 spec은 "AS2 결합 + 단절생존"이 핵심.

### 5-3. 구조 차이

| 축 | 문서 | spec |
|---|---|---|
| 토폴로지 | **멀티클러스터 3리더**(A/B/C) | 단일 스웜 1리더 |
| 선출 | Raft | lowest-alive-id |
| 할당 | 분산 CBBA | 중앙집중 |
| 충돌회피 | ORCA 20ms | CPA/TCPA 예측 + AS2 위임 |
| 통신 | 도메인격리+domain_bridge | 단일도메인+네임스페이스 |
| 단절생존 | Relay/RTL 단편 | 체계적(복제+캐시+자율폴백) |
| AS2 접합 | 불명시 | follower_agent 명시 |

---

## 6. spec 반영 권고 (문서서 채택할 가치)

문서가 spec보다 나은 부분 = **채택 후보**:

1. **ISR Fusion (GCS)** — spec엔 GCS 영상통합 없음. 정찰 임무 필수. GStreamer 별 파이프 + GCS 융합 노드 추가.
2. **Topology Monitor (Cluster)** — 링크품질 감시. 단절생존 설계(D7 채널분리)와 시너지. 모니터로 추가(읽기전용, 순수상위).
3. **GPS Integrity Monitor (RAIM 탐지)** — safety_monitor에 RAIM 탐지 기능 추가 또는 별 노드. GPS 스푸핑/재밍 대응(실외 정찰).
4. **Replay & Log** — ROS2 bag + 시계열DB. 임무 재현/디버깅.
5. **멀티클러스터 옵션** — 대규모(9+드론)면 spec 단일스웜 → 클러스터 계층 확장. 단 단일도메인+네임스페이스로(도메인격리 X). 향후 확장 항목.

**채택하지 말 것**: Raft, 분산CBBA, ORCA 20ms, 도메인격리, 협조측위 보정 — AS2 정합성 깸.

---

## 7. 최종 판정

- **문서 → 순수 상위레이어 변환 가능.** 5교체(Raft→lowest-alive-id, 분산CBBA→중앙, ORCA연속→이산, RRT*→path_planning재사용, 도메인격리→네임스페이스) + 1정정(MAVLink) + 1설정(DDS) + 1드롭(협조측위).
- **버리는 것**(GPS유 실외 정찰엔 수용가능): 20ms ORCA, Raft 강일관성, 분산자율성 일부, 협조측위.
- **수렴점**: 변환 결과 ≈ `swarm_intelligence_layer_spec_v1.md` + 문서의 GCS/ISR/Topology/RAIM 모니터 추가분. 두 설계가 사실상 같은 곳으로 수렴.
- **spec이 우월한 핵심**: follower_agent(AS2 접합) + coordinator/mission_manager(단절생존). 문서엔 없음.
- **문서가 보강하는 것**: ISR Fusion, Topology Monitor, RAIM 탐지, Replay&Log, 멀티클러스터 확장.
