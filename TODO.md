# TODO — pingpong-bot

새 개발 순서를 위에서부터 차근차근 진행한다. 한 단계를 구현하고 테스트한 뒤
다음 단계로 넘어간다.

## 현재 상태 (2026-08-04)

- **1.1~1.3 핵심 구현 완료:** `N×7` 규약, 채택 관측 버퍼, 미래 궤적 샘플링.
- **1.4 제어 전환 완료:** real `CommitRequest`가 `BallTrajectory`와
  `Provisional | Refined` 단계를 전달한다.
- **레일·라켓 조준 공통 제어 완료:** `DirectController`를 real·GUI sim이
  공유해 목표 선택, 라켓 헤드 x 보정, 레일 clamp, 조준각, 명령 시간을 같이 계산한다.
- **2단계 예측 제어 완료:** 추적 성립 즉시 1차 목표로 이동하고,
  관측 0.25초·최근 3회 10cm 수렴 후 갱신된 공 x와 같은 상대편 끝선 조준 함수를 다시 적용한다.
- **실기 오차 계측 완료:** 명령 후 레일·조준축을 재측정해
  `requested`, `applied`, `measured`, `applied - measured`를 구분해 남긴다.
  20ms 간격으로 확인하고 허용치 안에 2회 연속 들어오면 수렴으로 판정한다.
- **안전 중단 추가:** 예상 도착 후 500ms까지 수렴하지 못한 상태가 3회
  연속이면 레일을 정지하고 조준축을 현재 위치에 홀드한 뒤 제어를 종료한다.
- **로그 정리:** 관전 창의 공 감지 상태는 `false ↔ true`로 바뀔 때만 한 번 기록한다.
- `fly_07/08/09` 검출·삼각측량·EKF 예측 진단 통과. 단, 이것은 아직
  `Target → 위치 이동`까지의 통합 테스트가 아니다.
- 현재 변경은 정적 검사와 관련 단위·물리 sim 테스트를 통과했다. GUI 시뮬레이션과
  Windows 실물 장비에서의 최종 수렴 동작은 별도 검증이 필요하다.

## 1. 공 궤적 반환 API

공 위치 검출·삼각측량·EKF·탄도 예측을 하나의 궤적 출력으로 묶는다.
실제 관측 궤적과 미래 예측 궤적을 같은 형식으로 반환한다.

**이 API는 타격 예측 지점을 반환하지 않는다.** 시간 순서대로 샘플링된
전체 궤적만 반환한다. 최고 타격 지점·타격 시각·타격 가능성은 나중에
이 궤적을 입력으로 받는 별도 계산기에서 구한다.

### 1.1 반환 형식 확정

각 궤적은 `N×7` 행렬이다. 행 하나는 한 시점의 공 상태를 뜻한다.

```text
[x, y, z, vx, vy, vz, t]
```

| 열 | 뜻 | 단위 |
|---|---|---|
| `x, y, z` | 월드 좌표계의 공 중심 | m |
| `vx, vy, vz` | 해당 시점의 속도 벡터 | m/s |
| `t` | 기준 시각으로부터의 상대 시간 | s |

시간 기준은 **가장 최근의 EKF 채택 관측 시각**으로 통일한다.

- 실제 관측 궤적: 과거 행은 `t ≤ 0`, 가장 최근 행은 `t = 0`
- 미래 예측 궤적: 첫 미래 행부터 `t > 0`
- 행은 항상 `t` 오름차순으로 정렬

내부 코드는 열 인덱스 실수를 막기 위해 명시적인 샘플 타입을 사용하고,
외부 반환 경계에서만 `N×7` 행렬로 변환한다.

- [x] `TrajectorySample { position, velocity, time_secs }` 타입 추가
- [x] `BallTrajectory { observed, predicted, reference_time }` 타입 추가
- [x] `TrajectorySample` 목록 ↔ `N×7` 행렬 변환 API 추가
- [x] 행렬의 열 순서·단위·시간 기준 단위 테스트

### 1.2 실제 관측 궤적

스테레오 삼각측량을 통과한 3D 관측과 EKF가 추정한 속도를 기록한다.
재투영 오차나 EKF 게이트에서 거부된 점은 반환 궤적에 포함하지 않는다.

- [x] EKF가 채택한 3D 관측을 샷별 링 버퍼에 보관
- [x] 각 관측 시각의 EKF 속도를 함께 보관
- [x] 새 공으로 EKF를 리셋할 때 관측 궤적도 분리
- [x] 버퍼 최대 길이와 최대 보관 시간을 `defaults` 값으로 제한
- [x] 두 번째 관측 이전에는 속도가 없으므로 해당 행을 반환하지 않도록 처리

### 1.3 미래 예측 궤적

현재의 `predict_to(HitPlane) -> Prediction` 중심 출력을 전체 궤적 출력으로
바꾼다. 탄도 적분기가 일정한 간격으로 미래 상태 여러 개를 샘플링한다.

- [x] `Kinematics` 또는 `ballistics`에 궤적 샘플링 API 추가
- [x] EKF의 최신 위치·속도에서 예측 시작
- [x] 현재 물리 모델의 중력·drag·Magnus·테이블 바운스를 그대로 사용
- [x] `integrate_dt`를 샘플 간격으로 사용하고 `max_lead`까지 반환
- [x] 유효 작업 영역 밖, 예측 시간 초과 종료 조건 적용
- [ ] 테이블 아래 상태의 명시적 종료 조건 추가
  - 현재 적분기는 테이블 바운스 시 z를 표면으로 보정하므로 정상 탄도는 아래로 내려가지 않는다.
- [x] 출력에 `impact_position`, `HitPlane`, 대표 타격 점을 포함하지 않음
- [x] 첫 행부터 마지막 행까지 위치·속도·시간이 연속적인지 회귀 테스트

### 1.4 통합 반환

- [x] `Estimator::trajectory()`가 `observed` 궤적과 `predicted` 궤적을 함께 생성
- [x] `BallTrajectory` 반환값에는 두 궤적과 `reference_time`만 포함
- [x] 기존 `Prediction`/hit-plane 계산은 궤적 생성 API에서 분리
- [x] 기존 제어가 필요하면 임시 어댑터가 궤적에서 평면 교차를 계산하게 하고,
  궤적 반환 API에는 타격점을 다시 넣지 않음
- [x] real 추정 워커의 `CommitRequest` 예측 목록 payload를 `BallTrajectory`로 교체
- [ ] 프리뷰·시뮬레이션·텔레메트리가 두 궤적을 구분해 소비할 수 있게 전달
- [x] 실제 관측은 `t ≤ 0`, 미래 예측은 `t > 0`인지 EKF 통합 테스트
- [x] 새 공 리셋 후 이전 공의 궤적이 섞이지 않는지 EKF 통합 테스트
- [ ] `cargo test --workspace`
  - 2026-08-01: 260 통과, 46 ignored, 기존 Dynamixel 매핑 테스트 1개 실패.

### 1.5 진행 순서

1. 반환 타입과 `N×7` 행렬 규약을 먼저 구현한다.
2. 실제 관측 궤적 버퍼를 붙인다.
3. 미래 탄도 샘플링을 붙인다.
4. 두 궤적을 하나의 결과로 묶는다.
5. 타격점 계산이 궤적 반환 기능과 분리됐는지 검증한다.

### 1.6 2026-08-01 영상 검증 기록

2026-08-01, `data/clips/fly_07~09` 진단. 예측 오차는 현재
`max_impact_sigma = 0.15 m`, `drag = 0` 게이트를 통과한 값이다.

| 클립 | 겹치는 비행 구간 동시 검출 | 3D 궤적 | 예측 횟수 | 평균 / 최대 오차 |
|---|---:|---:|---:|---:|
| `fly_07` | 43/54 (80%) | 41점 | 13회 | 12.2 / 17.0 cm |
| `fly_08` | 44/221 (20%) | 41점 | 7회 | 6.4 / 12.1 cm |
| `fly_09` | 46/280 (16%) | 44점 | 11회 | 4.9 / 15.9 cm |

- `fly_07` 실제 교차 x=1.598 m를 테이블 폭 밖이라는 이유로 예측에서
  잘라내던 경계 문제를 발견했고, 로봇 작업 공간 여유 0.5 m를 반영했다.
- `fly_08/09`는 검출 구간의 동시 검출률이 20% 안팁이므로,
  제어 전환과 별개로 검출 연속성 개선이 남아 있다.
- 당시 검증은 `BallTrajectory → Prediction` 임시 어댑터까지였다. 현재 활성
  제어는 이 어댑터가 아니라 `BallTrajectory → HitTargetSelector →
  DirectController` 경로를 사용한다.

---

## 2. 현재 활성 제어 — 리니어 레일·라켓 조준 2단계

현재 런타임은 적응형 전체 스윙 대신 발사기 시험용 고정 임팩트 푸시를 실행한다. `BallTrajectory`에서 선택한
공 x에 라켓 헤드 x를 맞추고, 레일 위치→수평각 함수로 ID 3만 회전해
라켓이 상대편 탁구대 끝선 중앙을 바라보게 한다.

```text
BallTrajectory
    → HitTargetSelector
    → PredictionStability (Provisional | Refined)
    → DirectController
    → DirectControlCommand
    → Hardware::command_rail_and_racket
```

### 2.1 명령 계산

- [x] 현재 포즈를 읽고 레일 이동 거리와 조준축 이동 각도를 계산
- [x] 레일 가속·최고속도와 조준축 최고속도로 최소 도달 시간 계산
- [x] 공 도착까지 시간이 부족하면 하드웨어 명령 전송 전 거부
- [x] 레일 목표를 `0.000~1.410m` 물리 범위로 제한
- [x] 라켓 헤드 x가 공 x와 같아지도록 FK 오프셋만큼 레일 보정
- [x] 두 단계 모두 레일 위치→상대편 끝선 중앙 조준각 함수 사용
- [x] 공마다 각 단계를 최대 한 번 전송하고 최신 요청만 유지
- [x] real, GUI sim, 호출부 없는 generic pipeline이 같은 `DirectController` 사용

### 2.2 적용값과 수렴 확인

- [x] 하드웨어가 clamp·틱 양자화 뒤의 `AppliedRailRacketCommand` 반환
- [x] 예상 도착 시점부터 20ms 간격으로 레일·조준축 재측정
- [x] 레일 20mm·조준축 3° 안에 2회 연속 들어오면 `converged`
- [x] 정밀 명령이 1차 명령을 덮으면 기존 측정을 `superseded`로 종료
- [x] 예상 도착 후 500ms까지 수렴하지 못하면 `timeout`
- [x] `timeout` 3회 연속이면 레일 정지·조준축 홀드 후 제어 종료
- [x] 최신 공 목표 시각 후 실기·GUI sim 모두 중앙 준비 자세로 자동 복귀
- [x] 레일·조준과 동시에 5cm 고정 푸시 시작, 공 도착 시 최대 1.80m/s 목표 임팩트, 0.06초 팔로스루
- [ ] Windows 실물 장비에서 clamp·양자화·수렴 로그 최종 검증

### 2.3 현재 활성 경로에서 제외

- 전체 팔 IK와 point-to-point 관절 궤적
- 테이블–팔·라켓 충돌을 고려한 전체 자세 계획
- 적응형 백스윙·임팩트 속도와 반환 탄도 계산
- 물리 E-stop 입력

`PositionController`와 스윙 플래너 코드는 라이브러리에 남아 있지만 현재 real·GUI
sim의 직접 제어 경로에서는 호출하지 않는다. 선택적 `--home` 이동과 고정 임팩트
푸시가 전체 관절용 `Hardware::command` 경계를 사용한다.

### 2.4 남은 정리·검증

- [ ] 보존 중인 구형 위치·스윙 계획기의 향후 사용 여부 결정
- [ ] 사용하지 않기로 결정한 타입·설정·테스트·문서 제거
- [ ] 관측 → 궤적 → 단계 판정 → 명령 → 실측의 실물 통합 검증
- [ ] 물리 E-stop 입력과 복구 정책 정의
- [ ] 전체 workspace 테스트 재실행

### 2.5 실기 검증 발견 사항 (2026-08-04, Windows COM8)

- [x] **`track_seq`가 첫 공 이후로 고정되던 문제 — 수정함.** 지금 실기는 아직
  공을 실제로 쳐 보내지 않아 `BallReceding`(y 증가)이 다음 공 신호를 못 잡았고,
  `CommandLatch`가 첫 공 이후 모든 명령을 조용히 무시했다.
  `estimator_worker.rs::is_new_ball_reacquisition`으로 "추적이 끊겼다가
  재획득"도 새 공 신호로 추가해 해결. 실기 로그로 `track_seq`가 계속
  올라가는 것 확인함(`5 → 6 → 7 → … → 23`).
- [ ] **재획득 전까지 예측이 계속 stale해지는 문제 — 미해결, 원인만 파악.**
  `BallTrajectory::reference_time`(`ekf.rs::trajectory`)은 마지막 **채택**
  관측 시각인데, 그 정리(`stale_gap_secs=0.5s` 하드 리셋, `gate_reject_limit=5`)는
  전부 `Ekf::update_position` 안에서만 실행된다 — 즉 **새 3D 점이 실제로
  들어와야만** 발동한다. 카메라 동기 삼각측량이 계속 실패하면(로그에서
  `cam{id=0}` `detection_rate`가 길게 `0.0`) 그동안 EKF는 그냥 마지막 상태에
  얼어붙고, `reference_time`도 안 움직인다. 다음에 겨우 새 점이 들어올 때까지
  제어 워커는 `레일·라켓 조준 명령 계산 생략 ... 목표 시각이 N초 지남`을
  반복 — 한 트랙에서 17초 이상 관측됨. 근본 원인 후보 두 가지:
  1. 코드: EKF에 새 점이 없어도 주기적으로 "너무 오래 안 들어옴"을 스스로
     감지하는 선제적 타임아웃이 없다 (지금은 다음 점이 와야만 반응).
  2. 비전: `cam0` 검출률이 구간별로 `0.0~1.0`을 오가는 것 자체가 별도 조사
     대상 (조명·임계값·검출기 설정 등).
  사용자 요청으로 지금은 기록만 하고 코드 수정은 보류.
- [ ] **`PendingVerification` 경로가 실기 루프에서 도달 불가 — 2026-08-05 확인, 미해결.**
  `pending_verification`은 선언 시 `None`, 명령 직후 재설정 `None` 외에는 실제
  `spawn()` 루프에서 `Some(...)`으로 대입되지 않는다 — 유닛 테스트가 직접
  구성해 `verify_due_command`를 호출할 때만 그 경로가 실행된다. 즉 `src/real/README.md`의
  "제어 괴리 로그"·"제어 워커" 섹션이 완료로 적은 재측정 수렴 판정·3회 연속
  실패 시 중단은 현재 실기에서 발동하지 않는다. 부활·제거 결정은 보류.
  `docs/superpowers/specs/2026-08-05-control-worker-state-machine-design.md` 참고.
- [x] **한 공에 스윙은 최대 한 번 — 확정된 의도, 2026-08-05 latch로 명시화.**
  명령이 하나 성공하면 단계와 무관하게 그 `track_seq`의 이후 요청을 전부
  막는다. 리팩터 중 `BallControlState::Idle` 복귀 후 이 차단이 풀리는 틈이
  잠깐 생겼었는데, `CommandLatch::mark_struck()`을 추가해 latch가
  track_seq당 영구히 막도록 고쳤다(`src/real/control_worker.rs`).
  `Provisional`이 거의 즉시 도착하므로 `Refined`(0.25초 관측 후)는 도착
  전에 이미 막히는 것도 확정된 동작이다 — 사용자 확인: 일단 친 공은 다시
  스윙하지 않는다(핑퐁에 재시도 없음). `src/real/README.md:14`도 이에
  맞춰 갱신함.
- [ ] **vision control integration — `main`의 새 비전 스택을 제어 경로에 연결.**
  `main`이 `detector`/`estimator`(재귀 EKF)를 전부 지우고 `vision::Fit`
  (배치 Gauss-Newton 곡선 피팅) + `vision::Trajectory` 계약으로 새로 짰다
  (2026-08-05 merge). 지금 `control_worker.rs`·`estimator_worker.rs`는
  옛 `estimator::Ekf`/`detector::Detector` 스택 위에 그대로 얹혀 있고,
  이번 merge는 그 스택을 `defaults::estimator`/`defaults::detector`로
  이름만 분리해 나란히 살려 둔 것 — 통합은 보류했다. 작업 범위:
  1. `estimator_worker.rs`를 `Ekf` 대신 `vision::Fit`으로 갈아끼운다.
  2. `CommitRequest`가 나르는 타입을 `estimator::BallTrajectory`에서
     `vision::Trajectory`로 바꾸고, `robot::control.rs`(`DirectController`,
     `HitTargetSelector`)의 필드 접근을 거기 맞춘다.
  3. `PredictionStage::{Provisional, Refined}` 게이팅을 다시 설계한다 —
     지금은 "관측 구간 길이"(옛 EKF의 점진적 관측 누적)로 판단하는데,
     `vision::Fit`은 배치 피팅(`Outcome::{Seeded,Accepted,Rejected,Idle}`)이라
     그대로 옮겨지지 않는다.
  끝나면 `src/detector/`·`src/estimator/`·`defaults::detector`·
  `defaults::estimator`를 통째로 지운다.

---

다음 기능은 요구사항을 받으면 `3.` 섹션으로 추가한다.
