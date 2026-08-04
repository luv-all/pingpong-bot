# 상대 코트 중앙 조준·리니어 레일 2단계 제어

## 목적

현재 실기 제어는 동적 스윙을 실행하지 않는다. 카메라 예측에 따라 리니어
레일을 먼저 이동하고, 예측이 안정되면 공을 상대 코트 중앙으로 돌려보내는
라켓 면 방향과 정지 자세를 계산해 전체 관절을 이동한다.

## 입력

추정기는 다음 정보를 제어 워커에 보낸다.

```text
CommitRequest {
    track_seq,
    trajectory: BallTrajectory,
    stage: Provisional | Refined,
    at,
}
```

`track_seq`는 공이 바뀌었음을 명시한다. 새 번호를 받으면 이전 공의 단계 상태를
버리므로, 이전 공이 정밀 단계까지 도달하지 못해도 다음 공을 정상 처리한다.

## 단계 판정

1. EKF가 위치와 속도를 추정하고 목표를 선택하면 `Provisional`이다.
2. 첫 정상 3D 관측 후 0.25초 이상 지나고, 최근 목표 세 개가 최신 목표에서
   10cm 이내이면 `Refined`다.

예측 공분산은 화면 진단에는 표시하지만 제어 시작 게이트로 사용하지 않는다.

## 출력

| 단계 | 리니어 레일 | 라켓 자세 |
|---|---|---|
| `Provisional` | 목표 x로 선행 이동 | 현재 자세 유지(손목은 준비각) |
| `Refined` | 자세 IK가 선택한 정밀 위치 | 상대 코트 중앙 반환 법선을 만족하는 전체 관절 자세 |

정밀 단계는 `Impact::rally_return`으로 상대 코트 절반의 중앙
`(WIDTH_X/2, LENGTH_Y×0.75)`에 바운드하는 출사 속도 `v_out`을 구한다. 공의
입사 속도를 `v_in`이라 하면 요구 라켓 법선은 `(v_out - v_in).normalize()`다.
라켓 중심은 공 중심에서 공 반지름과 라켓 두께만큼 법선 반대쪽으로 물린다.
자세 IK의 달성 법선이 요구 법선에서 10°보다 더 벗어나면 명령하지 않는다.

레일 목표는 물리 범위 `0.000~1.410m`로 제한한다. 현재 포즈에서 목표 자세까지
정지→정지 궤적을 만들고 관절·레일 속도와 가속도, 토크, 테이블 충돌을 검사한다.
공 도착까지 남은 시간이 안전 궤적보다 짧으면 명령을 보내지 않는다. 명령 시간은
기본적으로 `0.05~0.30초`이며, 남은 시간이 0.05초보다 짧으면 그 남은 시간,
실제 최소 이동 시간이 0.30초보다 길면 그 최소 이동 시간을 쓴다. 어떤 경우에도
계산된 명령 시간은 공 도착까지 남은 시간을 넘지 않는다.

## 명령 후 실측 오차

실기 제어 워커는 예상 도착 시점부터 20ms 간격으로 레일 엔코더와 전체 관절을
다시 읽는다. 레일·모든 관절·라켓 법선이 허용 오차 안에 두 번 연속 들어와야
`converged`로 인정한다.
예상 도착 후 500ms까지 수렴하지 않으면 `timeout`이다.

```text
rail_commanded_m
rail_measured_m
rail_commanded_minus_measured_m
wrist_commanded_rad
wrist_measured_rad
wrist_commanded_minus_measured_rad
joints_commanded_rad
joints_measured_rad
joints_commanded_minus_measured_rad
max_joint_error_deg
aim_error_deg
```

명령 시점에는 `requested`(제어기 요청)와 `applied`(하드웨어 clamp·틱 양자화 후
실제 전송)도 따로 남긴다. 오차는 `applied - measured`로 계산한다.
메인 스레드의 `Commanded` 이벤트에도 요청값이 아니라 실제 적용값이 들어간다.

레일 절대 오차가 20mm, 관절 하나라도 3°, 라켓 조준 오차가 10°를 넘으면 `WARN`, 그 안이면
`INFO`로 기록한다. 이전 명령이 수렴하기 전에 정밀 명령이 나가면 직전 포즈를
`superseded`로 남기고 새 명령을 시작한다. `timeout`이 3회 연속 나면 레일을 정지하고
전체 관절을 현재 위치에 홀드한 뒤 제어 워커를 종료한다. 정상 수렴하면 연속 timeout
횟수는 0으로 돌아가며, `superseded`는 timeout으로 세지 않는다.

## 실기·시뮬레이션 공통 경계

`DirectController`가 목표 선택, 중앙 반환 법선, 자세 IK, 안전 궤적, 명령 시간을
한 번만 계산한다. 실기 워커와 GUI 시뮬레이션은 모두 이 `DirectControlCommand`를
받는다. 실기는 `Hardware::command_rail_and_racket`으로 실제 장치에 보내고,
GUI 시뮬레이션은 같은 정지→정지 궤적을 `robot::State`에 재생한다.
별도의 generic pipeline에서 쓰는 `SimHardware`도 같은 `Hardware` 경계를 구현한다.

## 현재 제외한 기능

- 임팩트 순간의 라켓 타격 속도 계산
- 백스윙·임팩트·팔로스루 동적 타격 궤적
- 자동 센터 복귀
- 스윙 커밋·포기 상태 머신

정지→정지 위치 플래너는 정밀 자세와 시작 시 선택적 홈 이동에 사용한다.
임팩트 속도를 만드는 동적 스윙 플래너는 아직 활성 경로에서 사용하지 않는다.
