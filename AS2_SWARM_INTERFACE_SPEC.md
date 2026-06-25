# 군집 정찰 시스템 — C1 메시지·인터페이스 상세 스펙 (ROS2 IDL)

> `AS2_SWARM_MODULE_MANIFEST.md` C1의 메시지/서비스/액션 **필드 레벨 정의**. ROS2 `.msg/.srv/.action` 실제 데이터 타입 포함.
> 패키지: `aiss_swarm_msgs` (신규). 스타일: as2_msgs 컨벤션(enum 상수 `대문자 = N`, `std_msgs/Header`, `string id`) 준수.
> 의존: `std_msgs`, `geometry_msgs`, `nav_msgs`, `builtin_interfaces`, `as2_msgs`(PoseWithID 재활용).

---

## 0. 공통 enum (상수 정의)

전 메시지가 참조하는 enum. 각 메시지에 인라인 정의(ROS2는 별도 enum 타입 없음 → 상수 + 필드).

| enum | 값 | 정의 위치 |
|---|---|---|
| **MissionType** | UNKNOWN=0, EOIR_RECON=1, RF_RECON=2, SURVEIL=3, TRACK=4 | MissionIntent / SwarmTask |
| **Phase** | STANDBY=0, TAKEOFF=1, TRANSIT=2, WORK=3, REGROUP=4, RETURN=5, LANDING=6, DONE=7, EMERGENCY=8 | MissionPhase |
| **Health** | OK=0, DEGRADED=1, FAIL=2 | Heartbeat / SwarmJoin |
| **Role** | NONE=0, SCOUT=1, TRACKER=2, RELAY=3 | SwarmTask |

---

## 1. DetectObjects.action — EO/IR 객체탐지

`detect_aruco_markers` 미러. 패키지: `aiss_swarm_msgs/action/DetectObjects.action`

```
# === Goal ===
string[]  target_classes      # 탐지 대상 클래스 필터 (빈 배열 = 전체)
float32   min_score           # 탐지 신뢰도 임계 (0.0~1.0)
bool      use_eo              # EO 카메라 사용
bool      use_ir              # IR 카메라 사용
---
# === Result ===
bool      success             # 정상 종료 여부
uint32    total_detections    # 누적 탐지 수
---
# === Feedback ===
uint16    current_detections  # 현재 프레임 탐지 수
float32   processing_hz       # 추론 처리율
```

---

## 2. Detection / DetectionArray — 탐지 결과

`aiss_swarm_msgs/msg/Detection.msg`
```
uint16                     class_id        # 클래스 정수 id
string                     class_name      # 클래스 이름
float32                    score           # 신뢰도 (0.0~1.0)
geometry_msgs/PoseStamped  pose            # world(earth) 프레임 추정 위치
float32[4]                 bbox            # 픽셀 bbox [x_min,y_min,x_max,y_max]
string                     sensor          # "EO" 또는 "IR"
```

`aiss_swarm_msgs/msg/DetectionArray.msg`
```
std_msgs/Header   header                   # stamp + frame_id(earth)
string            drone_id                 # 탐지 드론 namespace
Detection[]       detections               # 탐지 목록
```
- 발행: `detect_objects` → `perception/detections` (드론 ns 상대). QoS: `QoS(10)` reliable.

---

## 3. TargetTrack / TargetTrackArray — 융합 표적

`aiss_swarm_msgs/msg/TargetTrack.msg`
```
uint32                     track_id        # 전역 트랙 id (융합기 부여)
uint16                     class_id        # 클래스 id
string                     class_name      # 클래스 이름
geometry_msgs/PoseStamped  pose            # world 융합 위치
float32                    confidence      # 융합 신뢰도
builtin_interfaces/Time    first_seen      # 최초 관측
builtin_interfaces/Time    last_seen       # 최근 관측
uint16                     observation_count  # 관측 횟수
string[]                   observed_by     # 관측 드론 id 목록 (dedup 근거)
```

`aiss_swarm_msgs/msg/TargetTrackArray.msg`
```
std_msgs/Header     header
TargetTrack[]       tracks
```
- 발행: `detection_fusion` → `/swarm/targets` (전역). QoS: `QoS(10)`.

---

## 4. EmitterTrack / EmitterTrackArray — RF 방사원

`aiss_swarm_msgs/msg/EmitterTrack.msg`
```
uint32                     emitter_id      # 전역 방사원 id
geometry_msgs/PoseStamped  pose            # 삼각측량 추정 위치
float32                    freq_mhz        # 중심 주파수 (MHz)
float32                    bandwidth_mhz   # 대역폭 (MHz)
float32                    power_dbm       # 수신 세기 (dBm)
float32                    confidence      # 측위 신뢰도
builtin_interfaces/Time    first_seen
builtin_interfaces/Time    last_seen
string[]                   observed_by     # 방위 기여 드론 id
```

`aiss_swarm_msgs/msg/EmitterTrackArray.msg`
```
std_msgs/Header     header
EmitterTrack[]      emitters
```
- 발행: `emitter_fusion` → `/swarm/emitters`. QoS: `QoS(10)`.

### 4.1 RfBearing — 드론별 방위 (rf_survey 출력)
`aiss_swarm_msgs/msg/RfBearing.msg`
```
std_msgs/Header   header                   # frame_id = base_link
string            drone_id
float32           azimuth_rad              # 방위 (rad, body)
float32           elevation_rad            # 고각 (rad)
float32           freq_mhz                 # 주파수
float32           power_dbm                # 세기
```
- 발행: `rf_survey` → `rf/bearings`. → `emitter_fusion` 입력.

---

## 5. SwarmTask — 드론별 배정

`aiss_swarm_msgs/msg/SwarmTask.msg`
```
std_msgs/Header        header
string                 drone_id            # 대상 드론
uint8                  mission_type        # MissionType enum
uint8                  role                # Role enum (SCOUT/TRACKER/RELAY)
geometry_msgs/Polygon  zone                # 배정 sub-zone
nav_msgs/Path          sub_route           # 커버리지/측위/loiter 경로
string[]               target_classes      # 탐지 대상
as2_msgs/PoseWithID    formation_slot      # 편대 슬롯 오프셋 (centroid 기준, 재활용)
uint8                  takeoff_slot        # 이륙 순번 (0=먼저)
uint8                  land_slot           # 착륙 순번
geometry_msgs/PoseStamped  track_target    # 추적 표적 (TRACK 전용)
```
- 발행: `allocator` → `/{ns}/swarm_agent/task` (드론 ns 상대). QoS: `QoS(10)` (latched/transient_local 권장).

---

## 6. MissionPhase — 현 단계 방송

`aiss_swarm_msgs/msg/MissionPhase.msg`
```
# Phase enum
uint8 STANDBY   = 0
uint8 TAKEOFF   = 1
uint8 TRANSIT   = 2
uint8 WORK      = 3
uint8 REGROUP   = 4
uint8 RETURN    = 5
uint8 LANDING   = 6
uint8 DONE      = 7
uint8 EMERGENCY = 8

std_msgs/Header   header
uint8             phase               # 위 enum
uint32            seq                 # 단계 시퀀스 번호 (단조 증가)
string            mission_id          # 임무 인스턴스 id
```
- 발행: `PublishPhase`(coordinator) → `/swarm/mission_phase` (전역). QoS: `QoS(10)` transient_local (늦게 뜬 드론도 현 단계 수신).

---

## 7. MissionIntent — GCS 임무 의도

`aiss_swarm_msgs/msg/MissionIntent.msg`
```
# MissionType enum
uint8 UNKNOWN    = 0
uint8 EOIR_RECON = 1
uint8 RF_RECON   = 2
uint8 SURVEIL    = 3
uint8 TRACK      = 4

std_msgs/Header        header
string                 mission_id          # 임무 인스턴스 id
uint32                 mission_version      # 버전 해시 (동기 확인)
uint8                  mission_type         # 위 enum

geometry_msgs/Polygon  area                 # 임무 구역 (정찰/감시)
string[]               target_classes       # 탐지 대상 클래스
geometry_msgs/PoseStamped  home             # 복귀 지점

# 편대
string                 formation            # "line"/"wedge"/"grid"
float32                formation_spacing    # 기체 간격 (m)

# 제약
float32                cruise_altitude      # 순항 고도 (m)
float32                cruise_speed         # 순항 속도 (m/s)
float32                coverage_target      # 탐색 완료율 임계 (0~1)
float32                time_limit           # 임무 시간제한 (sec, 0=무제한)
uint8                  min_quorum           # 최소 정족수
```
- 발행: `gcs_mission_iface` → `/swarm/mission_intent`. QoS: `QoS(10)` transient_local (best-effort, 캐시).

---

## 8. Heartbeat — 생존·상태 메시

`aiss_swarm_msgs/msg/Heartbeat.msg`
```
# Health enum
uint8 OK       = 0
uint8 DEGRADED = 1
uint8 FAIL     = 2

std_msgs/Header        header
string                 drone_id            # 발신 드론 namespace
uint16                 numeric_id          # 선출용 정수 id (최저=리더)
bool                   alive               # 생존 플래그
uint8                  health              # 위 enum
float32                battery             # SOC (0.0~1.0)
bool                   armed               # 무장 여부
uint8                  phase               # 현 Phase (모니터)
geometry_msgs/Point    position            # world 위치 (분리감시·편대)
```
- 발행: 각 드론 `heartbeat` → `/swarm/heartbeat` (전역). QoS: `SensorDataQoS` best-effort, 2~5Hz.
- 소비: `election`(생존맵·최저 numeric_id), `aggregator`(barrier), 분리감시.

---

## 9. SwarmJoin.srv — 군집 참여

`aiss_swarm_msgs/srv/SwarmJoin.srv`
```
# === Request ===
string                 drone_id            # 가입 드론 namespace
uint16                 numeric_id          # 정수 id
uint8                  health              # Health enum
geometry_msgs/Point    position            # 현 위치
---
# === Response ===
bool                   accepted            # 수락 여부
string[]               registry            # 현 등록 멤버 namespace 목록
uint32                 mission_version      # 현 임무 버전 (동기)
uint8                  assigned_role        # 초기 Role (선택)
string                 reject_reason        # 거부 사유 (미수락 시)
```
- 서버: 리더 `registry`. 클라이언트: per-drone `SwarmJoin` BT 노드(재시도).

---

## 10. 토픽·인터페이스 바인딩 요약

| 인터페이스 | 타입 | 토픽/서비스 | 스코프 | QoS | 생산자→소비자 |
|---|---|---|---|---|---|
| DetectObjects | action | `DetectObjectsBehavior` | ns 상대 | action | BT → detect_objects |
| DetectionArray | msg | `perception/detections` | ns 상대 | QoS(10) | detect_objects → fusion |
| RfBearing | msg | `rf/bearings` | ns 상대 | QoS(10) | rf_survey → emitter_fusion |
| TargetTrackArray | msg | `/swarm/targets` | 전역 | QoS(10) | fusion → allocator/GCS |
| EmitterTrackArray | msg | `/swarm/emitters` | 전역 | QoS(10) | emitter_fusion → allocator |
| SwarmTask | msg | `/{ns}/swarm_agent/task` | ns 상대 | QoS(10) TL | allocator → 드론 |
| MissionPhase | msg | `/swarm/mission_phase` | 전역 | QoS(10) TL | coordinator → 전체 |
| MissionIntent | msg | `/swarm/mission_intent` | 전역 | QoS(10) TL | GCS → 리더 |
| Heartbeat | msg | `/swarm/heartbeat` | 전역 | SensorData | 드론 ↔ 드론 |
| SwarmJoin | srv | `swarm/join` | 전역 | service | 드론 → 리더 |

`TL` = transient_local (늦게 합류한 노드도 최신 상태 수신).

---

## 11. 재활용 메시지 (신규 정의 불필요)

| 메시지 | 패키지 | 용도 |
|---|---|---|
| `as2_msgs/PoseWithID`, `PoseWithIDArray` | as2_msgs | 편대 슬롯 오프셋 (SwarmFlocking) |
| `as2_msgs/SwarmFlocking.action` | as2_msgs | 편대 제어 (재활용) |
| `as2_msgs/FollowReference.action` | as2_msgs | 슬롯 추종 |
| `as2_msgs/PlatformInfo` | as2_msgs | health 입력(connected/armed/offboard) |
| `as2_msgs/AlertEvent` | as2_msgs | 안전 선점 |
| `geometry_msgs/{Polygon,Point,Pose,PoseStamped}`, `nav_msgs/Path` | 표준 | 구역/경로/위치 |
| `vision_msgs/Detection2DArray` | vision_msgs | 픽셀 bbox 표준(옵션) |

---

## 12. 패키지 빌드 (aiss_swarm_msgs)

위치: `aiss_ws/src/aiss/aiss_swarm_msgs/` (오버레이 워크스페이스, AS2 트리 밖 — `AS2_SWARM_PACKAGE_ARCHITECTURE.md` §0).

```
aiss_swarm_msgs/
├─ msg/  Detection.msg DetectionArray.msg TargetTrack.msg TargetTrackArray.msg
│        EmitterTrack.msg EmitterTrackArray.msg RfBearing.msg
│        SwarmTask.msg MissionPhase.msg MissionIntent.msg Heartbeat.msg
├─ srv/  SwarmJoin.srv
├─ action/ DetectObjects.action
├─ CMakeLists.txt   # rosidl_generate_interfaces + DEPENDENCIES std_msgs geometry_msgs nav_msgs builtin_interfaces as2_msgs
└─ package.xml      # depend: std_msgs geometry_msgs nav_msgs builtin_interfaces as2_msgs
```

`rosidl_generate_interfaces(${PROJECT_NAME} msg/*.msg srv/*.srv action/*.action DEPENDENCIES ...)`.
빌드 검증 = 마일스톤 A1 게이트.

---

_근거: `AS2_SWARM_MODULE_MANIFEST.md`(C1), as2_msgs 컨벤션(PoseWithID/AlertEvent/TrajGenInfo/DetectArucoMarkers), `AS2_RUNTIME_TOPIC_ICD.md`(토픽 스코프/QoS), `AS2_SWARM_BT_DISPATCH_DESIGN.md`(노드 I/O)._
