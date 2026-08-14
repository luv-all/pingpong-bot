# 공 위치·방향 정렬 제어

## 목적

현재 real과 GUI sim의 직접 제어는 예측 궤적에서 선택한 접수점에
라켓을 정지 정렬한다. real은 첫 본 예측에서 레일과 팔을 함께 이동하고,
같은 공의 후속 예측은 레일을 고정한 채 팔만 미세 보정한다. 별도 스윙은
실행하지 않는다.

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
요청이 큐에서 기다린 시간만큼 유효 현재 시각을 앞으로 옮긴 뒤, 접수 구간
`y=0.08~0.35m`의 미래 평면 중 구간 중심에 가장 가까운 점을 고른다.

비전 적합은 기본 트리거가 한 번 성립하면 궤적을 생성한다. 실제 모터 명령은
제어 워커의 `refined_prediction_ready`가 마지막 관측의 위치·속도 불확실성
기준을 통과한 뒤에만 나간다. 별도의 최소 관측 시간 조건은 없다.

## 명령 흐름

```text
vision::Trajectory
    → 제어 접수 평면 선택
    → Planner::ball_alignment
    → Hardware::command
    → 같은 공의 후속 예측은 Planner::ball_alignment_fixed_rail
    → Hardware::command_joints
    → 현재 모드의 준비 자세 복귀
```

공 x는 발사기 기준 오른쪽 3cm로 보정하고, 비전 공 z에는 기존 1.5cm와
추가 3cm를 합친 +4.5cm를 적용해 블레이드 중심에 닿도록 한다. 라켓 면은 네트 너머 상대편 반코트의
무게중심을 향한다. 백스윙·전진 타격 속도·팔로스루는 계산하지 않지만,
예측 도착 시각은 복귀 시점을 정하는 데 사용한다.

## 시작·복귀 자세

- 레일 안전 범위: `0.0100~1.3395m`
- AXL 보드 0에 대응하는 제어 원점: `0.730m`
- 기본 준비 위치: `RAIL_READY_X_M = 0.675m`(보드 `+0.055m`)
- 기본 관절 자세: `READY_JOINTS_4DOF`
- 시작 관절 보정: 2° 수렴 기준, 최대 6회 누적 폐루프 보정

프리뷰 키 `1`/`2`/`3`으로 구간 모드를 선택하면 타격 후에도 해당 모드의
준비 레일 x로 돌아간다. `4`는 필터를 해제하고 기본 위치 `0.675m`로 복귀한다.
직접 복귀 궤적이 안전 검사에 실패하면 상승 중간 자세를 거치는 2구간 경로를 시도한다.

## 실패 처리

- 위치·방향 IK, 관절 속도·가속도, 토크, 레일 범위, 테이블 충돌 검사를 통과한 궤적만 실행한다.
- 같은 `track_seq`의 최신 요청은 복귀 시점 전까지 팔 보정에 사용한다.
- 정렬·복귀 중 새로 생긴 다른 `track_seq`는 현재 상태를 덮어쓰지 못한다.
- 계획 실패는 해당 예측만 건너뛰고, 하드웨어 명령 실패는 제어 워커를 종료한다.

## 실측 확인

예측 도착 시각과 유지 시간이 지나 복귀하기 직전에 포즈를 다시 읽고,
레일과 모든 관절의 `commanded`, `measured`, `commanded - measured`를 로그로 남긴다.

`PendingVerification`의 주기적 수렴 판정과 3회 연속 실패 중단 경로는 현재 실기
루프에서 활성화되지 않으며 유닛 테스트에서만 사용한다.

## 공통 구현

- 실기: `src/real/control_worker.rs`
- GUI sim: `src/sim/physics/world.rs`
- 위치·방향 정렬: `Planner::ball_alignment`, `Planner::ball_alignment_fixed_rail`
- 기본 중립 복귀: `Planner::return_to_center`
- 구간 모드 준비 x 복귀: `Planner::return_to_center_at`

기존 백스윙·임팩트·팔로스루 계획기는 `src/robot/motion/`에 보존되어 있지만
이 직접 정렬 경로에서는 호출하지 않는다.
