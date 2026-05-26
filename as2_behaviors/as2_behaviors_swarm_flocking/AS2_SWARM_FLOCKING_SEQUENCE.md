# as2_behaviors_swarm_flocking 동작 시퀀스

## 전체 구성 요소

```
[미션 노드]          GCS 측 실행
[SwarmFlockingBehavior]   GCS 측 실행  (namespace: Swarm)
[DroneSwarm 객체]    SwarmFlockingBehavior 내부 객체 (드론별 1개)
[TF System]          ROS 2 TF 브로드캐스트 / 버퍼
[FollowReferenceBehavior]  드론 온보드 실행 (드론별 1개)
[Controller]         드론 온보드 실행 (드론별 1개)
```

---

## Phase 1. 초기화 (on_activate)

```mermaid
sequenceDiagram
    participant M  as 미션 노드
    participant SFB as SwarmFlockingBehavior
    participant TF  as TF System
    participant DS  as DroneSwarm[n]
    participant FRB as FollowReferenceBehavior<br/>(drone_n)

    M  ->>+ SFB : send_goal<br/>{ virtual_centroid, swarm_formation, drones_namespace }

    Note over SFB: on_activate() 진입

    rect rgb(220, 240, 255)
        Note over SFB,TF: ① 가상 무게중심 TF 설정
        SFB ->> TF : Static TF 발행<br/>earth → Swarm<br/>(virtual_centroid 오프셋)
    end

    rect rgb(220, 255, 220)
        Note over SFB,DS: ② 편대 구성 (드론별 DroneSwarm 객체 생성)
        loop 드론마다 (drone0, drone1, ...)
            SFB ->> DS  : new DroneSwarm(drone_id, formation_pose)
            DS  ->> TF  : Static TF 발행<br/>Swarm → Swarm/drone_n_ref<br/>(편대 오프셋)
        end
        SFB ->> SFB : dynamic_swarm_formation 구독 등록
    end

    rect rgb(255, 240, 220)
        Note over SFB,FRB: ③ FollowReference 시작 (sleep 5초 후)
        SFB ->> SFB : sleep(5s) — Action Server 준비 대기
        loop 드론마다
            SFB ->> DS  : initFollowReference()
            DS  ->>+ FRB : FollowReference Action Goal<br/>target = (0,0,0) in "Swarm/drone_n_ref"<br/>max_speed = 15 m/s
            FRB -->> DS  : goal accepted
        end
    end

    rect rgb(255, 220, 240)
        Note over SFB,FRB: ④ 편대 위치 도달 대기 (블로킹 루프)
        loop 모든 드론이 0.3m 이내 도달할 때까지
            SFB ->> DS  : checkPosition()
            DS  ->> DS  : actual_distance_to_goal < 0.3m ?
            DS  -->> SFB: true / false
        end
    end

    SFB -->> M  : goal accepted → RUNNING 상태 진입
```

---

## Phase 2. 실행 루프 (on_run)

```mermaid
sequenceDiagram
    participant SFB as SwarmFlockingBehavior
    participant DS  as DroneSwarm[n]
    participant TF  as TF System
    participant FRB as FollowReferenceBehavior<br/>(drone_n)
    participant CTL as Controller<br/>(drone_n)

    Note over SFB: run_timer 콜백 (10Hz)

    rect rgb(220, 240, 255)
        Note over SFB,DS: ⑤ 상태 모니터링
        loop goal_future_handles_ 순회
            SFB ->> DS  : goal_handle->get_status()
            alt STATUS_EXECUTING / ACCEPTED / SUCCEEDED
                DS  -->> SFB: RUNNING
            else STATUS_ABORTED / CANCELED
                DS  -->> SFB: FAILURE
            end
        end
    end

    rect rgb(220, 255, 220)
        Note over FRB,CTL: ⑥ 드론 온보드 제어 루프 (독립 실행)
        loop run_timer (FRB 내부, 10Hz)
            FRB ->> FRB : sendPositionCommandWithYawAngle()<br/>frame_id = "Swarm/drone_n_ref"<br/>pos = (0, 0, 0)
            FRB ->> CTL : /drone_n/motion_reference/pose 발행<br/>{ frame_id:"Swarm/drone_n_ref", pos:(0,0,0) }
            CTL ->> TF  : TF 조회<br/>earth → Swarm → Swarm/drone_n_ref
            TF  -->> CTL: 실제 earth 좌표 반환<br/>(virtual_centroid + 편대 오프셋)
            CTL ->> CTL : 위치 setpoint → 비행 제어
        end
    end

    rect rgb(255, 240, 220)
        Note over FRB,SFB: ⑦ Feedback 흐름
        FRB ->> FRB : getState() — TF로 현재 거리 계산
        FRB -->> DS  : feedback { actual_distance_to_goal }
        DS  -->> SFB : follow_reference_feedback_
    end
```

---

## Phase 3. 동적 편대 변경

```mermaid
sequenceDiagram
    participant EXT as 외부 노드
    participant SFB as SwarmFlockingBehavior
    participant DS  as DroneSwarm[n]
    participant TF  as TF System
    participant FRB as FollowReferenceBehavior<br/>(drone_n)

    alt 토픽으로 변경 (개별 드론 위치)
        EXT ->> SFB : /Swarm/dynamic_swarm_formation 발행<br/>PoseWithIDArray { id:"drone1", pose:새 오프셋 }
        SFB ->> SFB : dynamicSwarmFormationCallback()
        SFB ->> DS  : updateStaticTf(new_pose)
        DS  ->> TF  : Static TF 재발행<br/>Swarm → Swarm/drone1_ref (새 오프셋)
        SFB ->> DS  : initFollowReference() 재호출
        DS  ->>+ FRB: FollowReference Action Goal (재전송)
        FRB -->> DS : goal accepted
    else on_modify (전체 편대 재구성)
        EXT ->> SFB : send_goal (새 목표)
        Note over SFB: on_modify() 진입
        SFB ->> DS  : stopFollowReference() — 전체 중단
        DS  ->> FRB : /_behavior/stop 서비스 호출
        FRB ->> FRB : on_deactivate() → sendHover()
        SFB ->> SFB : drones_.clear()
        SFB ->> TF  : Static TF 재발행 (새 virtual_centroid)
        loop 새 편대 드론마다
            SFB ->> DS  : new DroneSwarm() 생성
            DS  ->> TF  : Static TF 재발행 (새 편대 오프셋)
            SFB ->> DS  : initFollowReference()
            DS  ->> FRB : FollowReference Action Goal
        end
    end
```

---

## Phase 4. 일시정지 / 재개 / 종료

```mermaid
sequenceDiagram
    participant M   as 미션 노드
    participant SFB as SwarmFlockingBehavior
    participant DS  as DroneSwarm[n]
    participant TF  as TF System
    participant FRB as FollowReferenceBehavior<br/>(drone_n)

    alt 일시정지 (on_pause)
        M   ->> SFB : /_behavior/pause 서비스 호출
        SFB ->> TF  : lookupTransform(earth → Swarm)<br/>현재 Swarm 위치 읽기
        SFB ->> TF  : Static TF 재발행 (현재 위치로 고정)<br/>earth → Swarm (현재값으로 덮어쓰기)
        loop 드론마다
            SFB ->> DS  : stopFollowReference()
            DS  ->> FRB : /_behavior/stop 서비스 호출
            FRB ->> FRB : on_deactivate() → sendHover()
        end
        SFB ->> SFB : goal_future_handles_.clear()
    else 재개 (on_resume)
        M   ->> SFB : /_behavior/resume 서비스 호출
        SFB ->> TF  : Static TF 재발행 (goal_.virtual_centroid)
        loop 드론마다
            SFB ->> DS  : initFollowReference()
            DS  ->>+ FRB: FollowReference Action Goal
            FRB -->> DS : goal accepted
        end
    else 종료 (on_execution_end)
        M   ->> SFB : cancel_goal 또는 abort
        Note over SFB: on_execution_end() 진입
        loop 드론마다
            SFB ->> DS  : stopFollowReference()
            DS  ->> FRB : /_behavior/stop 서비스 호출
            FRB ->> FRB : on_deactivate() → sendHover()
        end
        SFB -->>- M  : result { swarm_success }
    end
```

---

## TF 트리 구조 (실행 중 상태)

```
earth (글로벌 기준 프레임)
  │
  └── Swarm                     ← SwarmFlockingBehavior 발행 (virtual_centroid)
        │                          frame_id: virtual_centroid.header.frame_id
        │                          translation: virtual_centroid.pose.position
        │
        ├── Swarm/drone0_ref    ← DroneSwarm[drone0] 발행 (편대 오프셋)
        │                          translation: swarm_formation[0].pose.position
        │
        ├── Swarm/drone1_ref    ← DroneSwarm[drone1] 발행
        │                          translation: swarm_formation[1].pose.position
        │
        └── Swarm/drone2_ref    ← DroneSwarm[drone2] 발행
                                   translation: swarm_formation[2].pose.position

※ 모든 TF는 Static TF — 한 번 발행 후 각 드론 TF buffer에 영구 캐싱
※ virtual_centroid.frame_id 를 drone0/base_link 로 설정하면 리더-팔로워 구조
```

---

## 컨트롤 명령 데이터 흐름

```
SwarmFlockingBehavior
  └─ Static TF 발행: earth→Swarm→drone_n_ref  ← 위치 정보가 여기에 인코딩됨
  └─ FollowReference Goal: target=(0,0,0) in "Swarm/drone_n_ref"
                              │
                              ▼
FollowReferenceBehavior (drone_n)
  └─ /drone_n/motion_reference/pose 발행
       { frame_id: "Swarm/drone_n_ref", position: (0,0,0) }  ← TF 변환 없이 그대로 발행
                              │
                              ▼
Controller (drone_n)
  └─ TF 조회: "Swarm/drone_n_ref" (0,0,0) → earth 좌표 변환
       = virtual_centroid 위치 + 편대 오프셋
  └─ 위치 setpoint → 비행 제어기 전송
```
