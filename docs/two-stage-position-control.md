# 공 위치·높이 단순 정렬 제어

## 목적

현재 real과 GUI sim의 직접 제어는 카메라가 예측한 접수점을 임팩트 지점으로 삼는다.
레일과 팔을 동시에 이동시켜 그 지점에 정지 정렬한다. 별도 스윙은 실행하지 않는다.

## 입력

```text
CommitRequest {
    trajectory: vision::Trajectory {
        seq,
        origin,
        measured,
        predicted,
    },
    at,
}
```

비전은 접수 평면을 정하지 않고 전체 궤적과 축별 불확실성을 넘긴다. 제어 워커가
`vision::Track::at_time`으로 채널 지연만큼 현재 상태를 전진시킨 뒤 접수 구간
`y=0.08~0.35m`의 평면을 선택한다.
공 도착까지 남은 시간은 현재 결합 궤적의 실행 가능 여부를 제한하지 않는다.

## 출력

```text
선택 공 위치 (x,y,z)
    → 공 위치와 상대 네트 중앙 방향 자세 IK
    → 레일·팔 동시 이동 궤적 검사
    → 원래 예측 위치에 정지 정렬
    → 중립 자세 복귀
```

라켓 면 방향은 상대 네트 중앙을 향하도록 계산한다. 백스윙, 전진 타격 속도,
팔로스루 및 공 도착 시각 동기화는 계산하지 않는다.

## 시작·복귀 자세

- 레일: 실기 안전 범위 `0.0100~1.3395m`, 보드 0에 대응하는 제어 원점은 `0.705m`, 준비 위치는 탁구대 실측 중앙 보정값 `RAIL_READY_X_M = 0.675m`(보드 `+0.030m`)
- 관절: `READY_JOINTS_4DOF`

기존 감긴 `ready_prewind` 자세는 활성 real·GUI sim 경로에서 사용하지 않는다.
직접 복귀가 테이블을 관통하면 상승 중간 자세와 최종 중립 복귀가 모두 안전한지
먼저 검사하고, 두 구간이 모두 유효할 때만 실행한다.

## 실패 처리

위치 IK, 관절 속도·가속도, 토크, 레일 범위 또는 테이블 충돌 검사에 실패하면
그 공은 모터 명령 없이 건너뛴다. 같은 `track_seq`의 반복 요청은 막지만 제어
워커는 종료하지 않으며 다음 공을 계속 받는다.

## 실측 확인

정렬 이동이 끝나면 복귀 전에 포즈를 다시 읽는다.

```text
rail_commanded_m
rail_measured_m
rail_commanded_minus_measured_m
joints_commanded
joints_measured
joints_commanded_minus_measured
```

이 로그로 카메라 예측 위치, 계산된 정렬 목표, 실제 모터 도달값을 분리해서 확인한다.

## 공통 구현

- 실기: `src/real/control_worker.rs`
- GUI sim: `src/sim/physics/world.rs`
- 공통 위치·방향 정렬 플래너: `Planner::ball_alignment`
- 공통 중립 복귀: `Planner::return_to_center`

기존 전체 백스윙·임팩트 계획기는 보존되어 있지만 이 직접 제어 경로에서는 호출하지 않는다.
