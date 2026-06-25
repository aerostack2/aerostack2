# Aerostack2 정찰 비행 기능 설계 — EO/IR 촬영 + Object Detection

목표: 특정 구역을 EO/IR 카메라로 촬영하며 object detection 수행하는 정찰(reconnaissance) 비행 기능 추가.

---

## 1. 설계 원칙

AS2 철학 = **모듈 단일책임 + behavior 병렬 조합**. "정찰"을 단일 거대 노드로 만들지 않는다. 비행 / 탐지 / 짐벌을 분리된 behavior로 두고 미션 레이어에서 병렬 실행.

근거: 기존 `detect_aruco_markers_behavior`가 정확히 이 패턴 — 탐지만 수행, 비행 제어 안 함 (이미지 sub → 탐지 → 결과 pub). 비행은 `follow_path` 등 별도 behavior 담당.

- 참조 파일: `as2_behaviors/as2_behaviors_perception/detect_aruco_markers_behavior/include/detect_aruco_markers_behavior/detect_aruco_markers_behavior.hpp:78-80`

---

## 2. 추가 구성요소 (4개)

### 2.1 `detect_objects_behavior` (신규, perception 레이어) — 핵심

- 위치: `as2_behaviors/as2_behaviors_perception/detect_objects_behavior`
- 템플릿: `detect_aruco_markers_behavior` 복제 후 확장

| 항목 | 내용 |
|---|---|
| Base 클래스 | `as2_behavior::BehaviorServer<as2_msgs::action::DetectObjects>` |
| 입력 (Sub) | EO `sensor_measurements/eo/image_raw` + `camera_info`, IR `sensor_measurements/ir/image_raw` + `camera_info` |
| 처리 | `on_run()`에서 detector 추론 (DNN = ONNX / TensorRT, 또는 IR 열 blob CV) |
| 출력 (Pub) | `perception/detections` |
| 의존성 | OpenCV, cv_bridge (이미 perception pkg 포함), + 추론 런타임(ONNXRuntime / TensorRT) |

비고:
- EO/IR = 카메라 2개. `as2::sensors::Camera` 인스턴스 2개 (aruco는 1개).
- 융합 옵션: IR로 후보 검출(열원) → EO로 분류. 또는 독립 처리 후 결과 병합.
- behavior 인터페이스(`on_activate / on_modify / on_deactivate / on_pause / on_resume / on_run / on_execution_end`)는 aruco와 동일 시그니처.

### 2.2 탐지 결과 메시지 (신규)

현재 `as2_msgs/msg`에 detection 전용 메시지 없음 (aruco는 `PoseStampedWithIDArray` 재활용).

- 권장 1순위: **`vision_msgs/Detection2DArray`** (ROS 표준, 재발명 금지) — 픽셀 bbox + class + score.
- 지오로케이션 필요 시: 신규 **`as2_msgs/DetectionArray`** — 필드: class, score, `geometry_msgs/PoseStamped`(world-frame).
  - 산출법: aruco가 카메라 intrinsic + TF로 pose 추정하는 방식과 동일. ray-ground 교차로 world 좌표 계산.

### 2.3 구역 커버리지 비행

- 옵션 A (권장, AS2 idiom): **coverage planner plugin** 신규
  - 위치: `as2_behaviors/as2_behaviors_path_planning/plugins/coverage` (기존 `a_star`, `voronoi` 옆)
  - 동작: 폴리곤 입력 → lawnmower / boustrophedon waypoint 생성 → 기존 `FollowPath` behavior가 실행
  - 장점: 신규 비행 behavior 불필요, 기존 path_planning pluginlib 구조 재활용
- 옵션 B (간단): 미션에서 waypoint 미리 계산 → `FollowPath.action` 직접 호출

### 2.4 짐벌 제어 — 재활용

- `point_gimbal_behavior` + `PointGimbal.action` 그대로 사용. EO/IR 시선 하향 / 스캔.
- 신규 작업 없음.

---

## 3. 오케스트레이션 (병렬 실행)

behavior = ROS2 action server이므로 동시 구동 가능. 미션 레이어(`as2_python_api/mission_interpreter` 또는 군집 BT)에서 조합:

```
[FollowPath: 커버리지 경로]  ──┐
[PointGimbal: 하향 스캔]      ──┼─ 병렬 실행
[DetectObjects: EO/IR 추론]   ──┘
        │
   perception/detections ──▶ 미션 로직(로깅 / 마킹 / 재방문 트리거)
```

- `FollowPath`가 구역을 훑는 동안 `DetectObjects`는 독립 run-loop로 매 프레임 추론.
- 탐지 발생 → 미션이 hover / 재방문 / `GoToWaypoint` 트리거.

---

## 4. 데이터 흐름 (Runtime ICD 확장)

기존 ICD(`AS2_RUNTIME_TOPIC_ICD.md`)에 추가되는 신규 토픽:

| Topic | Msg Type | Producer → Consumer |
|---|---|---|
| `sensor_measurements/eo/image_raw` | `sensor_msgs/Image` | EO 드라이버 → DetectObjects |
| `sensor_measurements/eo/camera_info` | `sensor_msgs/CameraInfo` | EO 드라이버 → DetectObjects |
| `sensor_measurements/ir/image_raw` | `sensor_msgs/Image` | IR 드라이버 → DetectObjects |
| `sensor_measurements/ir/camera_info` | `sensor_msgs/CameraInfo` | IR 드라이버 → DetectObjects |
| `perception/detections` | `vision_msgs/Detection2DArray` 또는 `as2_msgs/DetectionArray` | DetectObjects → 미션 |

신규 액션: `as2_msgs/action/DetectObjects.action`
- goal: class 필터 / score 임계값
- feedback: 탐지 수
- result: 종료 상태
- 구조: `DetectArucoMarkers.action` 미러링

---

## 5. 작업량 요약

| 작업 | 신규 / 재활용 | 난이도 |
|---|---|---|
| DetectObjects behavior | 신규 (aruco 템플릿) | 중 — 추론 통합이 핵심 |
| DetectObjects.action + Detection msg | 신규 | 하 |
| coverage path plugin | 신규 (plugin) | 중 |
| FollowPath 비행 | 재활용 | — |
| PointGimbal 짐벌 | 재활용 | — |
| EO/IR 카메라 드라이버 | 신규 / HW 의존 | HW별 |
| 미션 병렬 조합 | 기존 interpreter / BT | 하 |

핵심 신규 작업 = **DetectObjects behavior 1개 + coverage plugin 1개 + Detection msg/action**. 나머지 재활용.

---

## 6. 구현 순서 (권장)

1. `as2_msgs`: `DetectObjects.action` + (필요 시) `DetectionArray.msg` 추가
2. `detect_objects_behavior`: aruco 복제 → EO/IR 2-카메라 sub → 더미 detector로 파이프라인 검증
3. detector 추론 통합 (DNN / IR CV) + 지오로케이션
4. coverage path plugin 추가 → FollowPath 연동
5. 미션 시나리오: FollowPath + PointGimbal + DetectObjects 병렬 조합 + 탐지 트리거 로직
6. EO/IR HW 드라이버 연동 (Sim → Real)

---

_참조 소스: `as2_behaviors/as2_behaviors_perception/detect_aruco_markers_behavior/`, `as2_behaviors/as2_behaviors_payload/point_gimbal_behavior/`, `as2_behaviors/as2_behaviors_path_planning/plugins/`, `as2_msgs/action/`, `as2_core/include/as2_core/sensor.hpp`._
