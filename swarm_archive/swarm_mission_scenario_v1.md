# 군집드론 정찰 임무 시나리오 — 노드 흐름 정리

**기준 문서**: 군집드론 노드 설계서 v1  
**시나리오**: 임무 생성 → 정찰 수행 → 복귀 및 보고  
**범위**: 노드별 역할과 개념적 데이터 입출력 (토픽·타입 명세 제외)

---

## 레이어 및 노드 구성

| 레이어 | 노드 | 역할 요약 |
|--------|------|-----------|
| **GCS** | `gcs_mission` | 임무 생성·업로드·보고 수신 |
| **GCS** | `gcs_monitor` | 전체 상태 모니터링 |
| **Leader** | `mission_manager` | 생명주기 관리·종료 조건 평가 |
| **Leader** | `task_allocator` | 역할·구역 할당·재배정 |
| **Leader** | `path_planner` | 6개 경로 생성·패턴 선택 |
| **Leader** | `formation_manager` | 편대 형태 결정 |
| **Leader** | `swarm_coordinator` | Registry·Version Sync |
| **Leader** | `mission_reporter` | 보고서 생성 |
| **Follower** | `follower_controller` | 명령 수신·편대 유지·자가진단 |
| **Follower** | `sensor_processor` | EO/IR/LiDAR 전처리 |
| **Follower** | `detection` | 온보드 AI 추론 |
| **Follower** | `tracking` | Kalman 표적 추적 |
| **Follower** | `ardupilot_bridge` | MAVLink 변환 |
| **Safety** | `safety_monitor` | 6개 Safety 기능 통합 감시 |
| **Safety** | `local_replanner` | 실시간 우회 경로 생성 (200ms) |

> Safety Layer는 이륙부터 착륙까지 Mission Layer와 독립적으로 상시 동작한다.

---

## 단계별 노드 흐름

---

### 사전 임무 단계 (Pre-Mission)

---

#### 1단계 · 임무 생성 및 업로드

> **GCS → Leader**  
> 운용자가 GCS에서 임무를 구성하고 Leader에게 전달한다. Leader는 내용을 검증하고 임무 버전을 확정한다.

**`gcs_mission` — 임무 생성·검증 요청**

운용자 입력을 바탕으로 임무 패키지를 구성한다. 업로드 전 임무 유효성을 확인한다.

- 내보내는 데이터: 임무 패키지 (구역·우선순위·시간제한·최소 기체 수)
- 받는 데이터: 임무 검증 결과, 보고서 (임무 종료 후)

**`mission_manager` — 임무 수신·검증**

수신한 임무 패키지의 스키마와 AO 유효성을 확인하고 버전 ID를 발급한다. 검증 실패 시 GCS에 사유를 반환한다.

- 내보내는 데이터: 임무 검증 결과 (성공/실패·사유)
- 받는 데이터: 임무 패키지

---

#### 2단계 · 환경 분석

> **Leader 내부**  
> Leader가 임무 구역을 분석해 위험 구역·안전 통로·탐색 패턴 권고를 도출하고 전 노드에 배포한다.

**`mission_manager` — AO·위험 분석 실행**

임무 모델을 바탕으로 위험 구역·접근 제약·우선 탐색 구역을 분석한다.

- 내보내는 데이터: 분석 결과 (위험구역·안전통로·패턴 권고)
- 받는 데이터: 임무 패키지·AO 데이터

**`task_allocator` — 분석 결과 수신**

위험 구역과 구역 우선순위를 파악해 이후 역할 할당의 기준으로 삼는다.

- 받는 데이터: 분석 결과

**`path_planner` — 분석 결과 수신**

위험 구역·NFZ·권고 탐색 패턴을 경로 계획의 제약 조건으로 등록한다.

- 받는 데이터: 분석 결과

---

#### 3단계 · Pre-flight 점검 및 군집 등록

> **전 기체**  
> 각 Follower가 자가진단을 수행하고 Leader에게 보고한다. Leader는 quorum을 확인하고 Mission Version을 동기화한다.

**`follower_controller` — 자가진단 수행**

GPS·IMU·배터리·센서·통신 상태를 점검하고 결과를 Leader에게 보고한다.

- 내보내는 데이터: 기체 건강 상태 (준비/불량·사유)

**`swarm_coordinator` — 군집 등록·Version Sync**

Follower Join Request를 수락하고 Registry에 등록한다. Mission Version Hash를 브로드캐스트해 전 기체 동기화를 확인한다.

- 내보내는 데이터: Mission Version Hash, Swarm Ready 신호
- 받는 데이터: 기체 건강 상태, Join Request (기체별)

**`mission_manager` — Quorum 판단**

등록된 기체 수가 최소 quorum 이상인지 확인한다. 미달 시 임무를 보류하고 GCS에 알린다.

- 내보내는 데이터: Quorum OK / 임무 보류 통보
- 받는 데이터: Swarm Ready 신호

---

#### 4단계 · 역할 및 구역 할당

> **Leader → Follower**  
> Leader가 기체 성능·배터리·센서 유형을 고려해 정찰·추적·대기 역할과 담당 구역을 배정한다.

**`task_allocator` — 역할·구역 결정**

분석 결과와 기체 성능 프로파일을 조합해 최적 역할을 결정한다. AO를 Voronoi 분할로 나누고 구역 간 중첩 마진을 적용한다.

- 내보내는 데이터: 역할 배정 (정찰/추적/대기), 담당 구역·탐색 패턴
- 받는 데이터: 분석 결과, 기체 건강 상태

**`follower_controller` — Task 수신·확인**

배정된 역할과 구역을 수신하고 확인 응답을 보낸다. 이후 모든 동작의 기준이 된다.

- 내보내는 데이터: Task 수신 확인
- 받는 데이터: 역할 배정, 담당 구역·탐색 패턴

**`detection` — 탐지 클래스 필터 설정**

Task에서 지정된 탐지 대상 클래스를 AI 모델 필터에 등록한다.

- 받는 데이터: Task 수신 (탐지 클래스 포함)

---

#### 5단계 · 경로 계획

> **Leader → Follower**  
> `path_planner`가 6개 하위 경로를 생성하고, `formation_manager`가 경로 유형에 맞는 편대를 결정한다.

**`path_planner` — 6개 하위 경로 생성**

Rally Point·Ingress·Search·Egress·RTB 경로를 순차 생성한다. AO 면적·긴급도·기체 수를 기준으로 탐색 패턴(Lawnmower/Spiral/Sector)을 자동 선택한다.

- 내보내는 데이터: 경유점 집합 (기체별), Geofence·안전 통로, 탐색 패턴
- 받는 데이터: 분석 결과, 역할 배정, 기체 성능 데이터

**`formation_manager` — 편대 형태 결정**

Ingress 경로 폭을 보고 편대 유형을 선택한다 (좁은 통로→Column, 탐색 구역→Line/Diamond). 기체별 상대 오프셋을 계산한다.

- 내보내는 데이터: 편대 유형·기체별 오프셋·이격 거리
- 받는 데이터: 경로 계획 (경로 유형 참조), 기체 목록

**`follower_controller` — 경로·편대 수신**

경유점과 편대 파라미터를 로컬에 캐시하고 비행 준비를 완료한다.

- 받는 데이터: 경유점 집합, 편대 유형·오프셋

---

#### 6단계 · Arm 및 이륙

> **GCS 승인 → 전 기체**  
> GCS가 5개 선행조건을 확인하고 승인하면, Leader가 Arm 명령을 전파한다. Safety Layer가 이 시점부터 활성화된다.

**`gcs_mission` — Arm 승인**

5개 선행조건 (Follower READY·Geofence 업로드·기상·Version Hash·Safety 활성) 확인 후 명시적 승인을 전달한다.

- 내보내는 데이터: Arm 승인 신호
- 받는 데이터: 선행조건 체크 결과

**`mission_manager` — Arm 명령 전파·이륙 제어**

GCS 승인 수신 후 전 기체에 Arm 명령을 전파한다. 이륙 순서를 제어해 충돌을 방지한다.

- 내보내는 데이터: Arm 명령, 이륙 명령 (순차)
- 받는 데이터: Arm 승인 신호, Armed State (기체별)

**`follower_controller` — Arm·이륙 수행**

Arm 명령을 수신해 ArduPilot에 전달하고, 이륙 명령 수신 후 목표 고도까지 상승한다. 도달 후 Hover 대기.

- 내보내는 데이터: Armed 상태, Airborne 상태, 실시간 위치·속도
- 받는 데이터: Arm 명령, 이륙 명령

> **Safety Layer 동작**: Safety Layer 활성화. Separation Manager가 이격 거리를 감시하고 Geofence Monitor가 AO 경계를 등록한다.

---

### 임무 실행 단계 (Execution)

---

#### 7단계 · 집결 및 편대 구성

> **전 기체**  
> 전 기체가 Rally Point로 이동해 집결하고, 지정된 편대 형태로 위치를 점유한다.

**`path_planner` — Rally Point 전파**

집결 좌표를 전체 기체에 전달한다.

- 내보내는 데이터: Rally Point 좌표

**`follower_controller` — 집결·위치 점유**

Rally Point로 이동 후 편대 파라미터에 따라 오프셋 위치를 점유한다. 허용 오차 (±0.5m) 내 안착 시 Position Acquired를 보고한다.

- 내보내는 데이터: 도착 확인, Position Acquired
- 받는 데이터: Rally Point 좌표, 편대 유형·오프셋

**`formation_manager` — 편대 구성 완료 판정**

전 기체 Position Acquired를 수집해 Formation Established를 선언하고 Transit 단계로 전환한다.

- 내보내는 데이터: Formation Established 신호
- 받는 데이터: Position Acquired (기체별), 실시간 위치

> **Safety Layer 동작**: Collision Avoidance가 CPA·TCPA를 계산해 기체 간 충돌 위험을 실시간 감시한다.

---

#### 8단계 · AO 진입 비행

> **전 기체**  
> 편대를 유지하며 Ingress Route를 따라 임무 구역으로 이동한다. 장애물 감지 시 자동 우회가 발동된다.

**`swarm_coordinator` — 진행 상황 집계**

전 기체의 실시간 위치·상태를 수집하고 AO 도달 확인을 대기한다.

- 받는 데이터: 실시간 위치·상태 (기체별)

**`follower_controller` — Waypoint 추종·편대 유지**

Ingress Route 경유점을 순차 추종하며 편대 오프셋을 유지한다. 실시간 위치·속도·상태를 전송한다.

- 내보내는 데이터: 실시간 위치·속도·비행 상태
- 받는 데이터: 경로 경유점, 편대 오프셋

**`path_planner` — Dynamic Replan 대기**

진입 중 경로 이탈이나 장애물 감지 신호를 수신하면 즉시 우회 경로를 재생성해 배포한다.

- 내보내는 데이터: 재계획 경로 (이벤트 발생 시)
- 받는 데이터: 실시간 위치, Safety 우회 요청

> **Safety Layer 동작**: Obstacle Avoidance가 LiDAR·Radar로 전방 장애물을 감지한다. 감지 시 `local_replanner`가 200ms 내 우회 경로를 생성하고 `follower_controller`에 즉시 반영한다.

---

#### 9단계 · 정찰 탐색

> **Follower (센서·AI)**  
> 각 Follower가 담당 구역을 탐색하며 센서 데이터를 수집하고, 온보드 AI가 표적을 탐지한다.

**`sensor_processor` — EO/IR/LiDAR 전처리**

카메라·LiDAR 원시 데이터를 전처리한다. 영상 안정화·지면 필터링을 적용해 AI 추론 품질을 높인다.

- 내보내는 데이터: 전처리 EO 영상, 전처리 IR 영상, 필터링 포인트 클라우드
- 받는 데이터: 카메라 원시 영상, LiDAR 포인트 클라우드

**`detection` — 온보드 AI 추론**

Jetson GPU에서 YOLOv8 추론을 30Hz로 실행한다. 신뢰도 임계값 이상 탐지 결과에 NMS를 적용해 발행한다.

- 내보내는 데이터: 탐지 결과 (위치·클래스·신뢰도)
- 받는 데이터: 전처리 영상 (EO/IR)

**`mission_manager` — 구역 완료율 추적**

각 기체의 탐색 완료율을 집계한다. 전체 구역 95% 도달 시 임무 완료 조건 충족으로 판정한다.

- 받는 데이터: 실시간 위치, 탐지 결과

> **Safety Layer 동작**: 탐색 중에도 Geofence Monitor가 AO 경계를 감시하고, Separation Manager가 기체 간 이격 거리를 유지한다.

---

#### 10단계 · 표적 탐지 및 추적

> **Follower → Leader → GCS**  
> 탐지된 표적에 대해 추적 전담 기체가 지정되고, Kalman 필터 기반 추적이 시작된다.

**`task_allocator` — 추적 자산 재배치**

탐지 결과를 수신하면 가장 가깝고 배터리가 충분한 기체를 추적 전담으로 재지정한다. 해당 기체의 역할을 TRACKER로 변경하고 새 Task를 배포한다.

- 내보내는 데이터: 추적 담당 기체 지정, 변경된 Task 배정
- 받는 데이터: 탐지 결과, 실시간 위치·배터리 (기체별)

**`tracking` — 표적 추적**

탐지 결과를 입력으로 Kalman 필터 추적을 시작한다. 표적의 위치·속도·방향을 시계열로 추정하고 전송한다.

- 내보내는 데이터: 추적 상태 (위치·속도·방향·신뢰도)
- 받는 데이터: 탐지 결과, 기체 현재 위치

**`path_planner` — 추적 경로 갱신**

추적 기체의 경로를 표적 이동 방향으로 동적 갱신한다.

- 내보내는 데이터: 갱신된 추적 경로
- 받는 데이터: 추적 상태, 추적 기체 현재 위치

---

#### 11단계 · 임무 종료 판정

> **Leader**  
> `mission_manager`가 5개 종료 조건을 우선순위 순서로 평가하고, 조건 충족 시 임무 완료를 선언한다.

**`mission_manager` — 종료 조건 평가**

다음 우선순위로 종료 조건을 평가한다.

1. GCS 강제 종료
2. 배터리 ≤ 30%
3. 임무 시간 초과
4. Quorum 미달
5. 구역 탐색 완료율 ≥ 95%

최초 충족 조건으로 Complete 상태를 선언하고 사유를 기록한다.

- 내보내는 데이터: Complete 상태·완료 사유, 미완료 구역 목록
- 받는 데이터: 구역 완료율, 기체별 배터리, 경과 시간, Quorum 상태, GCS 명령

**`gcs_monitor` — 종료 확인**

Complete 신호와 미완료 구역 목록을 수신하고 운용자에게 표시한다.

- 내보내는 데이터: 복귀 승인 (운용자 확인 후)
- 받는 데이터: Complete 상태·미완료 구역

---

### 복귀 및 보고 단계 (Post-Mission)

---

#### 12단계 · 귀환 경로 계획 및 복귀 비행

> **Leader → 전 기체**  
> 현재 기체 상태를 기준으로 RTB 경로를 재생성하고, 전 기체가 편대를 유지하며 복귀한다.

**`path_planner` — RTB 경로 재생성**

현재 위치·배터리 잔량·기상·Threat 현황을 반영해 귀환 경로를 생성한다. 배터리 위급 기체에는 최단 경로를 우선 배정한다.

- 내보내는 데이터: 귀환 경로 (기체별), 귀환 예상 시간
- 받는 데이터: 현재 위치 (기체별), 배터리 잔량, 현재 기상·Threat

**`follower_controller` — 편대 유지 복귀**

귀환 경로를 수신하고 편대를 유지하며 이동한다. 배터리 위급 발생 시 대열 이탈 후 개별 최단 복귀로 전환한다.

- 내보내는 데이터: 실시간 위치·ETA·배터리
- 받는 데이터: 귀환 경로, 편대 파라미터

**`mission_manager` — 착륙 순서 결정**

배터리 잔량 기준으로 착륙 순서를 결정하고 각 기체에 착륙 시각을 배정한다.

- 내보내는 데이터: 착륙 순서·시각 배정
- 받는 데이터: 실시간 배터리 (기체별)

> **Safety Layer 동작**: 귀환 중에도 전체 Safety 기능이 동작한다. DDS Loss 발생 시 Follower는 60초 Hover 후 개별 RTH를 자율 수행한다.

---

#### 13단계 · 착륙

> **전 기체**  
> 배터리 잔량이 적은 기체부터 순차 착륙하며, Safety Layer가 착륙 중 수직 이격 거리를 유지한다.

**`mission_manager` — 착륙 명령 순차 전파**

착륙 순서에 따라 기체별 착륙 명령을 전파한다. 착륙 완료를 확인하고 Disarm 명령을 전달한다.

- 내보내는 데이터: 착륙 명령 (순차), Disarm 명령
- 받는 데이터: 착륙 완료 보고 (기체별)

**`follower_controller` — 자동 착륙·Disarm**

착륙 명령 수신 후 자동 착륙을 수행한다. 착륙 완료를 보고하고 Disarm한다. 착륙 실패 시 Hover 후 재시도.

- 내보내는 데이터: 착륙 완료 보고, Disarmed 상태
- 받는 데이터: 착륙 명령, 착륙장 좌표

**`ardupilot_bridge` — MAVLink 명령 변환**

ROS2 착륙 명령을 ArduPilot MAVLink 프로토콜로 변환해 실제 비행 제어기에 전달한다.

- 내보내는 데이터: ArduPilot 비행 상태
- 받는 데이터: 착륙 명령 (ROS2 형식)

> **Safety Layer 동작**: 착륙 중 Separation Manager가 수직 이격 거리 5m 이상을 유지한다. 착륙 실패 기체는 Emergency Hover 후 재시도한다.

---

#### 14단계 · 임무 보고서 생성

> **Follower → Leader → GCS**  
> 각 Follower가 비행·탐지 로그를 업로드하고, `mission_reporter`가 통합 보고서를 생성해 GCS에 전달한다.

**`follower_controller` — 비행 로그 업로드**

임무 중 누적된 비행 통계 (거리·시간·배터리 소모·Safety 이벤트)를 Leader에게 전송한다.

- 내보내는 데이터: 비행 통계 로그, Safety 이벤트 로그

**`detection` — 탐지 로그 업로드**

임무 중 발행된 탐지 결과 전체를 전송한다.

- 내보내는 데이터: 탐지 결과 누적 로그

**`mission_reporter` — 보고서 통합 생성**

수집한 비행·탐지·추적·Safety 로그를 병합해 통합 보고서를 생성한다. 구역별 탐색 완료율, 기체별 비행 통계, Safety 이벤트 요약을 포함한다.

- 내보내는 데이터: 통합 임무 보고서 (GCS 전송)
- 받는 데이터: 비행 로그 (기체별), 탐지 로그, 추적 로그, Safety 이벤트 로그

**`gcs_mission` — 보고서 수신·저장**

통합 보고서를 수신하고 운용자에게 표시한다. 데이터를 영구 저장소에 기록한다.

- 받는 데이터: 통합 임무 보고서

---

## 전체 흐름 요약

```
사전 임무 단계
  1  임무 생성 및 업로드     gcs_mission → mission_manager
  2  환경 분석               mission_manager → task_allocator, path_planner
  3  Pre-flight 점검         follower_controller → swarm_coordinator → mission_manager
  4  역할·구역 할당          task_allocator → follower_controller, detection
  5  경로 계획               path_planner → formation_manager → follower_controller
  6  Arm 및 이륙             gcs_mission → mission_manager → follower_controller
                             [Safety Layer 활성화]

임무 실행 단계
  7  집결 및 편대 구성       path_planner → follower_controller → formation_manager
  8  AO 진입 비행            follower_controller → path_planner (Replan 대기)
  9  정찰 탐색               sensor_processor → detection → mission_manager
  10 표적 탐지 및 추적       detection → task_allocator → tracking → path_planner
  11 임무 종료 판정          mission_manager (5개 조건) → gcs_monitor

복귀 및 보고 단계
  12 귀환 경로 계획 및 비행  path_planner → follower_controller → mission_manager
  13 착륙                    mission_manager → follower_controller → ardupilot_bridge
  14 임무 보고서 생성        follower_controller, detection → mission_reporter → gcs_mission
```

---

## Safety Layer 동작 시점

| 단계 | 동작 내용 |
|------|-----------|
| 6단계 이륙 | Separation Manager 이격 거리 감시 시작, Geofence 등록 |
| 7단계 집결 | Collision Avoidance — CPA·TCPA 기반 충돌 위험 감시 |
| 8단계 진입 | Obstacle Avoidance — LiDAR·Radar 감지, `local_replanner` 200ms 우회 경로 생성 |
| 9단계 탐색 | Geofence Monitor — AO 경계 감시, Separation Manager 이격 유지 |
| 12단계 복귀 | DDS Loss 시 Follower 독자적 60초 Hover → 개별 RTH |
| 13단계 착륙 | Separation Manager — 수직 이격 5m 유지, 착륙 실패 시 Emergency Hover |
