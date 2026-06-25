# 군집 정찰 시스템 — 구현 WBS (Work Breakdown Structure)

> **산출물 지향 분해**(WBS) + **우선순위/DAG/게이트 스케줄 오버레이** 2층 분리.
> WBS 트리(1.1~1.12) = 매니페스트 C1~C9 + 가로지르는 3(통합/인프라/HW) 직매핑. 100% 규칙(상위=하위 합).
> 스케줄 = 매니페스트 14순위 + 워크플로우 DAG + 게이트(★1~4)를 WBS 위에 배열.
> 근거: `AS2_SWARM_MODULE_MANIFEST.md`, `AS2_SWARM_IMPLEMENTATION_WORKFLOW.md`, `AS2_SWARM_INTERFACE_SPEC.md`, `AS2_SWARM_PACKAGE_ARCHITECTURE.md`.

> 견적 단위: **S**=1~2일, **M**=3~5일, **L**=1~2주, **XL**=2주+. (러프, 상대크기.) 말단 work package = {구현 + 단위테스트 + Sim검증}.

---

## 1. WBS 트리 (산출물 분해)

### 1.1 인터페이스 (C1 → `aiss_swarm_msgs`)
| WP | 산출물/작업 | 목적 (무엇을 위한가) | 의존 | M | 견적 |
|---|---|---|---|---|---|
| 1.1.1 | 메시지 IDL 11종 (Detection/TargetTrack/EmitterTrack/RfBearing/ SwarmTask/MissionPhase/MissionIntent/Heartbeat + array) | 노드 간 통신 **데이터 계약**. 탐지·표적·배정·단계·생존을 표준 타입으로 정의 → 전 패키지가 공유 | — | A1 | M |
| 1.1.2 | srv/action (SwarmJoin.srv, DetectObjects.action) | 군집 참여(요청/응답)·탐지(목표/피드백/결과) **요청형 계약** 정의 | — | A1 | S |
| 1.1.3 | 패키지 빌드(CMake/package.xml, rosidl) | IDL을 실 코드로 생성·링크 가능화 (전 패키지 빌드 전제) | 1.1.1-2 | A1 | S |
| 1.1.4 | **인터페이스 동결 리뷰** | 변경비용 큰 계약을 **조기 고정** → 다운스트림 병렬 개발 안전 확보 | 1.1.3 | A1 | S |

### 1.2 인식·페이로드 (C2 → `aiss_behaviors_perception`)
| WP | 산출물/작업 | 목적 (무엇을 위한가) | 의존 | M | 견적 |
|---|---|---|---|---|---|
| 1.2.1 | detect_objects 골격 (aruco 템플릿 복제, EO/IR 2-카메라 sub) | EO/IR 영상 **입력 파이프라인** 구축 — 추론 통합의 토대 | 1.1 | A2 | M |
| 1.2.2 | detector 추론 통합 (ONNX/TensorRT 또는 IR CV) | 실제 **객체탐지 능력** — 정찰 임무 핵심 기능 | 1.2.1 | A2 | L |
| 1.2.3 | 지오로케이션 (intrinsic+TF, ray-ground) → world pose | 픽셀 탐지를 **world 좌표 표적**으로 — 융합·재방문·보고의 입력 | 1.2.2 | A2 | M |
| 1.2.4 | rf_survey behavior (스캔+DF 기동 → rf/bearings) | RF **방사원 탐지·방위 측정** — RF 정찰 핵심 기능 | 1.1 | C5 | L |

### 1.3 계획 (C3 → `aiss_coverage_planner`, `aiss_swarm_core`)
| WP | 산출물/작업 | 목적 (무엇을 위한가) | 의존 | M | 견적 |
|---|---|---|---|---|---|
| 1.3.1 | coverage planner plugin (boustrophedon, swath/overlap) | 구역을 **빠짐없이 훑는 경로** 생성 — 정찰 완전성(누락 0) | as2_path_planning base | A3 | M |
| 1.3.2 | zone_partitioner lib (폴리곤→N sub-zone) | 구역을 드론 수로 **분할** — 병렬 정찰(시간 단축) | — | A3 | M |
| 1.3.3 | 편대폭 반영 swath (군집 swath = 편대폭) | 편대로 **한 번에 넓게** 훑기 — 군집 커버리지 효율 | 1.3.1 | B3 | S |

### 1.4 융합 (C4 → `aiss_swarm_perception`)
| WP | 산출물/작업 | 목적 (무엇을 위한가) | 의존 | M | 견적 |
|---|---|---|---|---|---|
| 1.4.1 | detection_fusion (N드론 detections → dedup → /swarm/targets) | 다중기 **중복 탐지 제거** → 전역 표적 단일 뷰 (군집 고유 난제 해소) | 1.1, 1.2.3 | C2 | M |
| 1.4.2 | 트랙 수명관리 (first/last_seen, observed_by) | 표적 **지속 추적·소실 판정** — 안정적 표적 목록 유지 | 1.4.1 | C2 | S |
| 1.4.3 | emitter_fusion (방위 삼각측량 → /swarm/emitters) | 다기 방위를 **삼각측량**해 방사원 위치 산출 — RF 측위 | 1.1, 1.2.4 | C5 | M |

### 1.5 swarm_agent 조율 (C5 → `aiss_swarm_core`)
| WP | 산출물/작업 | 목적 (무엇을 위한가) | 의존 | M | 견적 |
|---|---|---|---|---|---|
| 1.5.1 | health_monitor (FCU/센서/TF/배터리 진단) | 비행·합류 **자격 판정** + 안전 게이트(BatteryLow 등) 입력 | 1.1 | B4 | M |
| 1.5.2 | heartbeat (발행+수신, 생존맵) | 멤버 **생존 공유** — 선출·분리감시·리더상실 감지 기반 | 1.1 | B4 | S |
| 1.5.3 | election (최저 id, 승계) | **단일 리더 결정**(조율 주체) + 자동 승계 → SPOF 제거 | 1.5.2 | B4 | M |
| 1.5.4 | registry/join (등록부, quorum, SwarmJoin srv) | **멤버십 구성·정족수** 판정 — 임무 개시 자격 게이트 | 1.5.2 | B4 | M |
| 1.5.5 | allocator (분할/경로/역할 배정, 재할당) | 추상 임무 의도를 **드론별 구체 배정**으로 분해 + 변화 시 재할당 | 1.3, 1.4 | B4/C2 | L |
| 1.5.6 | centroid_driver (virtual_centroid 궤적 TF) | 편대를 **경로 따라 이동** — SwarmFlocking 정적 centroid 한계 극복 | swarm_flocking | B3 | M |

### 1.6 BT (C6 노드 + C7 트리 → `aiss_swarm_bt`)
| WP | 산출물/작업 | 목적 (무엇을 위한가) | 의존 | M | 견적 |
|---|---|---|---|---|---|
| 1.6.1 | 조율 노드 7 (HasMissionType/PublishPhase/SwarmFlockingStart·Stop/ DriveCentroid/ModifyFormation/LatchMissionType) | BT가 **군집·편대·단계를 제어**하는 동작 노드 — 조율의 손발 | 1.1 | B1 | M |
| 1.6.2 | barrier 노드 4 (AwaitAllAirborne/Formed/ZonesDone/Landed) | **드론 간 동기** — 순차이륙/편대수렴/전구역완료/착륙 대기 | 1.8(aggregator) | B1 | S |
| 1.6.3 | 게이트/슬롯 노드 4 (PhaseIs/WaitTakeoffSlot/WaitLandSlot/SafeHover) | **단계 전환·순번 제어** + 안전 호버 폴백 | 1.1 | B1 | S |
| 1.6.4 | P3 노드 8 (DetectObjects/ObjectDetected/InspectTimeout/RfSurvey/ EmitterDetected/HasRole/TargetLost + coord측 Action) | 임무별 **작업 실행·트리거** 인터페이스 (탐지/측위/역할) | 1.2,1.4,1.5 | B1/C2 | M |
| 1.6.5 | 부트스트랩 노드 7 (AwaitBoot/PublishHeartbeat/ReportHealth/SwarmJoin/ IsLeader/ QuorumReady/MissionReady) | **부팅→임무 개시 게이트** — Phase 0 흐름제어 | 1.5 | C1 | M |
| 1.6.6 | chassis 트리 Cx_*/Dx_* (XML 골격 존재 → 노드 연결·검증) | **공통 골격**(이착륙/편대/복귀) — 전 임무 재사용 단위 | 1.6.1-3 | B2 | S |
| 1.6.7 | 4임무 P3 트리 (EO/IR·RF·감시·추적 Coord/Drone, 골격 존재) | 임무별 **WORK 동작** — chassis에 꽂히는 교체 부품 | 1.6.4 | C2~C6 | M |
| 1.6.8 | 부트스트랩 루트(SwarmRoot, IsLeader 자기선택) — 미작성 | 리더/팔로워 **자기선택 + 전 트리 통합** 진입점 | 1.6.5 | C1 | S |

### 1.7 GCS (C8 → `aiss_gcs`)
| WP | 산출물/작업 | 목적 (무엇을 위한가) | 의존 | M | 견적 |
|---|---|---|---|---|---|
| 1.7.1 | gcs_mission_iface (임무 정의→mission_intent 발행) | 운용자가 **임무 생성·업로드** — 군집에 의도 주입 | 1.1 | D1 | M |
| 1.7.2 | fleet_manager 연동(모니터 재활용) | 군집 상태·영상 **가시화** — 운용 모니터링 | as2_fleet_manager | D1 | S |

### 1.8 집계·barrier (C9 → `aiss_swarm_core`)
| WP | 산출물/작업 | 목적 (무엇을 위한가) | 의존 | M | 견적 |
|---|---|---|---|---|---|
| 1.8.1 | swarm_aggregator (전드론 상태→airborne/formed/zonesdone/landed) | barrier 판정용 **전드론 상태 집계** — BT Await* 입력 공급 | 1.1, 1.5.2 | B2 | M |
| 1.8.2 | barrier 타임아웃·낙오기 제외 정책 | barrier **교착 방지** — 낙오기로 임무 멈춤 방지 | 1.8.1 | B2 | S |

### 1.9 재활용 통합 (코드 0, 연동·검증 필수)
| WP | 산출물/작업 | 목적 (무엇을 위한가) | 의존 | M | 견적 |
|---|---|---|---|---|---|
| 1.9.1 | SwarmFlocking 연동 (SwarmFlockingStart/Stop 래퍼↔실액션) | 신규 BT 노드를 **실 편대 액션에 결선** — 편대 동작 실현 | 1.6.1 | B3 | S |
| 1.9.2 | FollowReference 연동 (Dx_HoldFormation↔DroneSwarm) | 드론 **슬롯 추종 결선** — 편대 유지 실현 | 1.6.1 | B3 | S |
| 1.9.3 | motion behavior 연동 (Takeoff/GoTo/FollowPath/Land/PointGimbal) | **기본 비행동작 결선** — BT가 AS2 behavior 호출 | 1.6 | A4 | S |
| 1.9.4 | fleet_manager discovery 연동 | GCS **드론 발견·상태 결선** | 1.7 | D1 | S |

### 1.10 검증·통합 (게이트 = work package)
| WP | 산출물/작업 | 목적 (무엇을 위한가) | 의존 | 게이트 | 견적 |
|---|---|---|---|---|---|
| 1.10.1 | **★1 단일기 정찰 박판** (takeoff→coverage→탐지→inspect→복귀→land Sim) | **수직 박판 동작 입증** — 센서→비행 일주, 파이프라인 성립 확인 | 1.2,1.3,1.9.3 | ★1 | M |
| 1.10.2 | **★2 부트스트랩** (부팅→선출→정족수→자율개시 Sim) | **자율 개시 입증** — GCS 없이 군집 스스로 임무 시작 | 1.5,1.6.5,1.6.8 | ★2 | M |
| 1.10.3 | **★3 군집 EO/IR 정찰** (P1~P5 + 융합 풀미션 N기 Sim) | **군집 풀미션 입증** — 5단계 + 분할정찰 + 탐지융합 통합 | 1.4.1,1.6.7,1.9.1-2 | ★3 | L |
| 1.10.4 | 모드전환 deconfliction (DISBAND/REGROUP 충돌 0) | 편대⇄독립 천이 **충돌 안전** 확보 (최대 리스크 R1) | 1.10.3 | — | M |
| 1.10.5 | **★4 일반성** (감시 임무 chassis 무수정 + P3 2서브트리) | **프레임워크 일반성 입증** — 임무 추가 비용 = P3 2파일임을 확인 | 1.10.3 | ★4 | M |

### 1.11 인프라 (가로지름)
| WP | 산출물/작업 | 목적 (무엇을 위한가) | 의존 | M | 견적 |
|---|---|---|---|---|---|
| 1.11.1 | 워크스페이스 scaffold (aiss_ws/src/aiss, 패키지 골격) | 개발 **토대** — 패키지 뼈대, 이후 전 작업의 그릇 | — | A1 | S |
| 1.11.2 | CI/빌드 (colcon, 오버레이, lint/test) | **회귀 방지·빌드 자동화** — 다인 병렬개발 안전 | 1.11.1 | A1 | M |
| 1.11.3 | Sim 환경 (gazebo/멀티로터 sim, N기 launch, 네임스페이스) | **HW 없이 검증** — 게이트 ★1~4 시험 무대 | — | B2 | M |
| 1.11.4 | TF/프레임 규약 (earth/swarm_base_link/centroid) | **좌표계 일관성** — 편대·지오로케이션·융합 정합 | — | B3 | S |
| 1.11.5 | launch·파라미터 체계 (mission yaml, 블랙보드 주입) | **배포·설정 체계** — 임무/편대/구역을 외부 주입 | 1.6 | C1 | M |

### 1.12 HW (외부 리드타임, 병렬 선행)
| WP | 산출물/작업 | 목적 (무엇을 위한가) | 의존 | M | 견적 |
|---|---|---|---|---|---|
| 1.12.1 | EO/IR 카메라 조달·드라이버·캘리브레이션 | 실 **영상 센서** 공급 — detect_objects 입력 | — | D2 | L |
| 1.12.2 | RF 수신기 조달·드라이버 | 실 **RF 센서** 공급 — rf_survey 입력 | — | C5/D2 | XL |
| 1.12.3 | FCU(MAVLink) platform 연동·HIL | 실 **비행제어** 연동 — Sim→실기 천이 | — | D2 | L |
| 1.12.4 | 시각동기(chrony/PTP)·메시(Wi-Fi/Zenoh) | 군집 **통신·시각동기** 기반 — 동시이륙/편대 정합 | — | D2 | M |

---

## 2. 스케줄 오버레이 (WBS → 스프린트, 우선순위/DAG/게이트)

| 스프린트 | WP | 게이트 | 비고 |
|---|---|---|---|
| **S1 기반** | 1.11.1-2, 1.1.* (인터페이스 동결), 1.6.1·1.6.3 시작 | — | 메시지+CI+BT노드 병렬 |
| **S2 단일기 박판** | 1.2.1-3, 1.3.1-2, 1.9.3, 1.10.1 | **★1** | 수직 박판 |
| **S3 BT·집계** | 1.6.2·1.6.4·1.6.6, 1.8.*, 1.11.3-4 | — | chassis+aggregator+Sim |
| **S4 편대·조율** | 1.5.*, 1.5.6, 1.3.3, 1.9.1-2 | — | swarm_agent + 편대 transit |
| **S5 부트스트랩** | 1.6.5·1.6.8, 1.6.7(EO/IR), 1.10.2 | **★2** | 자율 개시 |
| **S6 군집 정찰** | 1.4.1-2, 1.10.3, 1.10.4 | **★3** | 풀미션+융합+모드전환 |
| **S7 일반성·확장** | 1.10.5(감시), 1.2.4·1.4.3·1.6.7(RF), 1.6.7(추적) | **★4** | 2번째 임무 입증 |
| **S8 GCS·실기** | 1.7.*, 1.11.5, 1.12.* | — | HW(1.12)는 S1부터 병렬 선행 |

> HW(1.12)는 리드타임 길어 **S1부터 병렬 발주·개발**. Sim(1.11.3)으로 소프트웨어 선검증, HW는 후기 합류.

---

## 3. 임계 경로 (Critical Path)

```
1.1 인터페이스 동결
   → 1.6.1 BT 조율노드 ─┐
   → 1.2 detect_objects ┤→ 1.10.1 ★1 박판
   → 1.3 coverage ──────┘
        → 1.8 aggregator + 1.6.2 barrier
        → 1.5 swarm_agent(election/registry) + 1.5.6 centroid
        → 1.6.5 부트스트랩 노드 → 1.10.2 ★2
        → 1.4.1 fusion + 1.6.7 EO/IR P3 → 1.10.3 ★3
        → 1.10.5 ★4 일반성
```
**임계 = 1.1 → BT/인식 → ★1 → 조율 → ★2 → 융합 → ★3 → ★4.** 1.1(인터페이스) 지연 시 전체 지연 → **최우선 동결**.

---

## 4. 게이트 진입 기준 (work package 완료 정의)

| 게이트 | 통과 조건 (DoD) |
|---|---|
| ★1 (1.10.1) | 1기 Sim: takeoff→coverage 경로 완주→객체탐지 발행→inspect→복귀→착륙, 크래시 0 |
| ★2 (1.10.2) | N기 Sim: 지상 standby→heartbeat·선출 안정(leader 1개)→정족수→intent 수신→자율 P1 진입 |
| ★3 (1.10.3) | N기 Sim: P1~P5 완주 + 분할 정찰 + 탐지융합(중복 0) + 전구역 완료 barrier |
| ★4 (1.10.5) | 감시 임무가 chassis(Cx_/Dx_) 무수정 + SurveilP3Coord/Drone 2서브트리 추가만으로 동작 |

각 게이트 미통과 = 다음 스프린트 진입 보류.

---

## 5. 견적 요약 (러프)

| 분류 | WP 수 | 합산(상대) |
|---|---|---|
| 1.1 인터페이스 | 4 | ~M |
| 1.2 인식 | 4 | L+L |
| 1.3 계획 | 3 | ~M+M |
| 1.4 융합 | 3 | ~M+M |
| 1.5 조율 | 6 | L+다수M |
| 1.6 BT | 8 | 다수M+S |
| 1.7 GCS | 2 | M |
| 1.8 집계 | 2 | M |
| 1.9 재활용통합 | 4 | 다수S |
| 1.10 검증 | 5 | L+다수M |
| 1.11 인프라 | 5 | 다수M |
| 1.12 HW | 4 | L~XL (외부) |

> SW 임계경로 ≈ S1~S7. HW(1.12)는 병렬 외부 트랙. 인력·기간 환산은 팀 규모 대입 필요(WP 견적이 입력).

---

## 6. 사용법

1. **WBS(1.1~1.12)** = 산출물·작업 분해. 할당·추적 단위.
2. **스케줄(§2)** = WP를 스프린트에 배열(우선순위/DAG/게이트).
3. **게이트(§4)** = 스프린트 전환 관문.
4. 변경 시: WBS는 안정(산출물), 스케줄만 재배열.

---

_근거: `AS2_SWARM_MODULE_MANIFEST.md`(C1~C9·14순위), `AS2_SWARM_IMPLEMENTATION_WORKFLOW.md`(마일스톤·DAG·★1~4), `AS2_SWARM_INTERFACE_SPEC.md`, `AS2_SWARM_PACKAGE_ARCHITECTURE.md`._
