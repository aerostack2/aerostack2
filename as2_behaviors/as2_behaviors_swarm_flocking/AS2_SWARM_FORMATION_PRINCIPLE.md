# SwarmFlocking ↔ FollowReference 연계 구조 및 대형 유지 원리

---

## 1. 연계 구조 한눈에 보기

```
┌─────────────────────────────────────────────────────────────┐
│  GCS                                                        │
│                                                             │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  SwarmFlockingBehavior  (namespace: Swarm)           │  │
│  │                                                      │  │
│  │  역할: TF 트리 관리자 + FollowReference 지휘관       │  │
│  │                                                      │  │
│  │  ① Static TF 발행: earth → Swarm                    │  │
│  │  ② Static TF 발행: Swarm → Swarm/drone_n_ref (×N)   │  │
│  │  ③ FollowReference Action Goal 전송 (×N)            │  │
│  └──────────────────────────────────────────────────────┘  │
└───────────────────────────┬─────────────────────────────────┘
                    ③ Action Goal
          target=(0,0,0) in "Swarm/drone_n_ref"
                            │
          ┌─────────────────┼──────────────────┐
          ▼                 ▼                  ▼
┌──────────────┐  ┌──────────────┐  ┌──────────────┐
│   drone0     │  │   drone1     │  │   drone2     │
│              │  │              │  │              │
│FollowRef     │  │FollowRef     │  │FollowRef     │
│Behavior      │  │Behavior      │  │Behavior      │
│              │  │              │  │              │
│30Hz 루프:    │  │30Hz 루프:    │  │30Hz 루프:    │
│/motion_ref   │  │/motion_ref   │  │/motion_ref   │
│/pose 발행    │  │/pose 발행    │  │/pose 발행    │
│              │  │              │  │              │
│Controller    │  │Controller    │  │Controller    │
│TF 조회 →    │  │TF 조회 →    │  │TF 조회 →    │
│실제 좌표     │  │실제 좌표     │  │실제 좌표     │
└──────────────┘  └──────────────┘  └──────────────┘
```

---

## 2. TF 트리: 위치 정보의 실제 저장 위치

SwarmFlocking의 핵심은 **드론의 목표 좌표를 숫자가 아니라 TF 프레임 이름으로 전달**하는 것이다.

```
[TF 트리 구조]

earth (글로벌 고정 프레임)
  │
  └─── Swarm                         ← ① virtual_centroid 오프셋
         │   translation: (cx, cy, cz)   (SwarmFlockingBehavior 발행)
         │
         ├─── Swarm/drone0_ref        ← ② drone0 편대 오프셋
         │       translation: (f0x, f0y, f0z)  (DroneSwarm[drone0] 발행)
         │
         ├─── Swarm/drone1_ref        ← ② drone1 편대 오프셋
         │       translation: (f1x, f1y, f1z)  (DroneSwarm[drone1] 발행)
         │
         └─── Swarm/drone2_ref        ← ② drone2 편대 오프셋
                 translation: (f2x, f2y, f2z)  (DroneSwarm[drone2] 발행)


[drone0의 earth 기준 실제 목표 좌표]
  = earth → Swarm → Swarm/drone0_ref → (0,0,0)
  = (cx + f0x,  cy + f0y,  cz + f0z)
     ↑ 무게중심    ↑ 편대 오프셋
```

**모든 TF는 Static** → 한 번 발행하면 모든 드론의 TF buffer에 영구 캐싱.

---

## 3. FollowReference가 목표를 받는 방식

### DroneSwarm → FollowReferenceBehavior (Action Goal)

```
[drone_swarm.cpp:97-103]

goal.target_pose.header.frame_id = "Swarm/drone0_ref"   ← 프레임 이름만 전달
goal.target_pose.point.x         = 0.0                  ← 항상 원점
goal.target_pose.point.y         = 0.0
goal.target_pose.point.z         = 0.0
goal.yaw.mode                    = KEEP_YAW
goal.max_speed_x/y/z             = 15.0
```

`FollowReferenceBehavior`는 "Swarm/drone0_ref 프레임의 원점(0,0,0)으로 가라"는 명령을 받는다.  
실제 earth 좌표는 전혀 모른다 — TF 시스템이 알아서 해석한다.

### FollowReferenceBehavior → Controller (30Hz)

```
[follow_reference_behavior.cpp:226-230]

/drone0/motion_reference/pose 발행:
  frame_id  = "Swarm/drone0_ref"    ← 변환 없이 그대로
  position  = (0.0, 0.0, 0.0)
  yaw       = 현재 yaw (KEEP_YAW)
```

### Controller → 비행 제어기 (TF 조회)

```
TF 조회: "Swarm/drone0_ref" 의 (0,0,0) → earth 기준 좌표

결과: (cx + f0x, cy + f0y, cz + f0z) ← 실제 목표 좌표
```

---

## 4. 대형 유지 원리

### 4-1. 정지 상태에서의 대형 유지

```
가상 무게중심 위치: Swarm TF = earth 기준 (0, 0, 5)

편대 오프셋:
  drone0_ref: Swarm 기준 ( 0,  0, 0)  → earth 기준 ( 0,  0, 5)
  drone1_ref: Swarm 기준 ( 2,  0, 0)  → earth 기준 ( 2,  0, 5)
  drone2_ref: Swarm 기준 (-2,  0, 0)  → earth 기준 (-2,  0, 5)

각 드론은 자신의 ref 프레임 원점을 향해 position 제어
→ 각자 목표 좌표에 도달하면 대형 완성
```

```
실제 공간:
                 drone0
                  (0,5)
         ●         ●         ●
      drone2      (가상)    drone1
      (-2,5)     무게중심   (2,5)
                  (0,5)
```

### 4-2. 이동 시 대형 유지 — 핵심 원리

**Swarm TF(무게중심)를 이동시키면 모든 드론이 자동으로 따라간다.**

Static TF를 재발행하면 모든 드론의 TF buffer에서 `Swarm` 프레임 위치가 갱신된다.  
각 드론의 `FollowReferenceBehavior`는 여전히 자신의 `ref` 프레임 원점(0,0,0)을 추적 중이므로,  
`ref` 프레임이 이동하면 드론도 이동한다.

```
[이동 전]                     [이동 후: Swarm TF 재발행]
earth                         earth
  └─ Swarm (0,0,5)              └─ Swarm (5,0,5) ← Static TF 재발행
       └─ drone0_ref (0,0,0)          └─ drone0_ref (0,0,0) ← 변경 없음

drone0 목표 = (0,0,5)          drone0 목표 = (5,0,5) ← 자동 갱신!
```

```
[이동 시퀀스]

1. SwarmFlockingBehavior: Swarm TF 재발행
   earth → Swarm: (0,0,5) → (5,0,5)

2. TF buffer 자동 갱신
   모든 노드의 TF buffer에 새 Swarm 위치 반영

3. Controller: 다음 TF 조회 시 새 좌표 획득
   "Swarm/drone0_ref" 원점 = (5+0, 0+0, 5+0) = (5,0,5)

4. Controller: 새 좌표로 위치 setpoint 갱신
   드론이 (0,0,5) → (5,0,5) 으로 이동 시작

5. 편대 간격은 변하지 않음
   drone0: (5, 0, 5)
   drone1: (7, 0, 5)   ← 항상 drone0 기준 +2m
   drone2: (3, 0, 5)   ← 항상 drone0 기준 -2m
```

### 4-3. 리더 드론을 따라가는 구조

`virtual_centroid.header.frame_id = "drone0/base_link"` 로 설정하면:

```
earth
  └─ drone0/base_link  (State Estimator가 실시간 갱신하는 동적 TF)
       └─ Swarm         ← virtual_centroid 오프셋
            └─ Swarm/drone1_ref
            └─ Swarm/drone2_ref

drone0가 이동하면 → drone0/base_link TF 갱신
                  → Swarm TF 위치 갱신
                  → drone1_ref, drone2_ref 위치 자동 갱신
                  → drone1, drone2가 drone0 따라감
```

단, 이 경우 Static TF와 Dynamic TF가 혼용되어 TF 체인이 복잡해질 수 있음.

---

## 5. 전체 제어 루프 타이밍

```
[GCS] SwarmFlocking
  on_run() 10Hz ──── monitoring만 수행 (goal_handle 상태 체크)
  TF 재발행 ────────── on_modify() 또는 dynamic_swarm_formation 토픽 수신 시만

[드론 온보드] FollowReferenceBehavior
  on_run() 30Hz ──── sendPositionCommandWithYawAngle() 매 루프 호출

[드론 온보드] Controller
  motion_reference/pose 수신 → TF 조회 → 비행 제어기 setpoint 전송

[드론 온보드] State Estimator
  /self_localization/pose, /self_localization/twist 발행
  /tf: drone_n/base_link → earth 동적 TF 발행

타이밍 요약:
  TF 갱신: on_modify 또는 dynamic_formation 토픽 수신 시
  위치 명령: 30Hz (FollowReference 루프)
  TF 해석 및 비행 제어: Controller 내부 루프 (드론별 상이)
```

---

## 6. 동적 편대 변경 시 대형 전환 원리

```
기존 대형 (가로 배열):             새 대형 (세로 배열):
  drone0 (0, 0, 5)                   drone0 (0,  0, 5)
  drone1 (2, 0, 5)     →             drone1 (0,  2, 5)
  drone2 (-2,0, 5)                   drone2 (0, -2, 5)

변경 방법:
  /Swarm/dynamic_swarm_formation 토픽 발행
    drone1: {id:"drone1", pose:{position:{x:0, y:2, z:0}}}
    drone2: {id:"drone2", pose:{position:{x:0, y:-2, z:0}}}

내부 처리:
  1. dynamicSwarmFormationCallback() 호출
  2. DroneSwarm[drone1].updateStaticTf({x:0, y:2, z:0})
     → Swarm → Swarm/drone1_ref TF 재발행
  3. DroneSwarm[drone1].initFollowReference() 재호출
     → FollowReference Action 재전송 (목표 프레임 동일, TF만 변경됨)
  4. Controller: 새 TF 기준으로 좌표 재계산
     → drone1이 새 위치로 이동 시작
```

---

## 7. 의존성 및 전제 조건

```
SwarmFlockingBehavior 동작을 위한 필수 조건:

[각 드론마다]
  ├── FollowReferenceBehavior 실행 중
  │     → /drone_n/FollowReferenceBehavior Action Server
  ├── Controller 실행 중
  │     → /motion_reference/pose 구독 및 TF 조회 가능
  ├── State Estimator 실행 중
  │     → /self_localization/pose, /twist 발행
  │     → /tf: drone_n/base_link 동적 TF 발행
  ├── Platform 실행 중 (FLYING 상태)
  │     → /platform/info 발행
  └── TF 트리 연결: earth → drone_n/base_link 경로 존재

[SwarmFlockingBehavior]
  └── virtual_centroid.header.frame_id 가 TF 트리에 존재해야 함
        (예: "earth" 또는 "drone0/base_link")
```

---

## 8. 설계 핵심 요약

| 구성 요소 | 역할 | 핵심 동작 |
|---|---|---|
| **SwarmFlockingBehavior** | TF 트리 설정 + FollowRef 지휘 | Static TF 발행, Action Goal 전송 |
| **DroneSwarm 객체** | 드론별 TF 및 클라이언트 관리 | TF 발행, FollowRef Action Client |
| **TF 시스템** | 위치 정보 전달 채널 | earth→Swarm→drone_n_ref 체인 |
| **FollowReferenceBehavior** | 30Hz 위치 명령 발행 | frame_id만 전달, 좌표 변환 없음 |
| **Controller** | TF 해석 + 비행 setpoint 생성 | TF 조회로 earth 좌표 계산 |

**핵심 설계 원칙:**
> 위치 정보는 숫자(좌표)가 아니라 **TF 프레임 이름**으로 전달된다.  
> SwarmFlocking이 TF 트리만 업데이트하면,  
> FollowReference와 Controller는 변경 없이 자동으로 새 위치를 추적한다.
