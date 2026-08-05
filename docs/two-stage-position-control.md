# 공 위치·높이 단순 정렬 제어

## 목적

현재 real과 GUI sim의 직접 제어는 타격하지 않는다. 카메라가 예측한 접수점에
라켓 중심이 실제로 도달하는지 확인하는 기초 정렬 모드다.

## 입력

```text
CommitRequest {
    track_seq,
    trajectory: BallTrajectory,
    stage: Provisional | Refined,
    at,
}
```

목표 선택에는 `BallTrajectory`와 접수 구간 `y=0.08~0.35m`만 사용한다.
공 도착까지 남은 시간은 정렬 궤적의 실행 가능 여부를 제한하지 않는다.

## 출력

```text
선택 공 위치 (x,y,z)
    → 공 x 주변 레일 후보 생성
    → 위치 전용 IK
    → 정지→정지 궤적 검사
    → 가장 가까운 안전 후보 실행
    → 0.2초 유지
    → 중립 자세 복귀
```

라켓 면 방향은 상대 네트 중앙을 향하도록 계산한다. 임팩트 속도와 끝속도,
백스윙, 팔로스루는 계산하지 않는다.
정렬 궤적의 끝속도는 모든 관절과 레일에서 0이다.

## 시작·복귀 자세

- 레일: `RAIL_READY_X_M = 0.71m`
- 관절: `READY_JOINTS_4DOF`

기존 감긴 `ready_prewind` 자세는 활성 real·GUI sim 경로에서 사용하지 않는다.
직접 복귀가 테이블을 관통하면 상승 중간 자세와 최종 중립 복귀가 모두 안전한지
먼저 검사하고, 두 구간이 모두 유효할 때만 실행한다.

## 실패 처리

위치 IK, 관절 속도·가속도, 토크, 레일 범위 또는 테이블 충돌 검사에 실패하면
그 공은 모터 명령 없이 건너뛴다. 같은 `track_seq`의 반복 요청은 막지만 제어
워커는 종료하지 않으며 다음 공을 계속 받는다.

## 실측 확인

정렬 자세 유지가 끝나면 복귀 전에 포즈를 다시 읽는다.

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
- 공통 위치 플래너: `Planner::ball_alignment`
- 공통 중립 복귀: `Planner::return_to_center`

스윙·임팩트 라이브러리는 보존되어 있지만 이 직접 제어 경로에서는 호출하지 않는다.
