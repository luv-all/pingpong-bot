# 임팩트 knot 가속도 0 강제 제거 — 저크 최소화로 대체

**날짜**: 2026-07-31
**대상**: `QuinticSegment`(`src/robot/motion/quintic_segment.rs`), `Trajectory`
(`src/robot/motion/trajectory.rs`), `trajectory_with_follow_through`
(`src/robot/motion/physics.rs`)
**기반 계획**: `.omc/plans/2026-07-31-nonzero-impact-knot-acceleration.md`
(선행 계획 `.omc/plans/2026-07-31-pre-impact-freeze-fix.md`은 기각·대체됨)

---

## 1. 결론 요약

사용자가 보고한 "로봇팔이 타격 시점에서 swing을 하는 게 아니라, 순간
멈추는 것으로 보여" 증상의 원인을 재조사했다. 1차 가설(`COARSE_TRACK_JOINT_FRACTION`
낮추기)은 실측 결과 네트통과율 회귀(100%→83.3%)로 **기각**했다(상세는
`2026-07-31-pre-impact-freeze-fix.md`). 사용자의 재지적("타격 시점에
모터 토크를 쓰고 있어야 한다")을 따라가 재조사한 결과, **진짜 원인은
`QuinticSegment`가 모든 세그먼트의 시작·끝 가속도를 항상 0으로 강제하는
설계**였다 — 임팩트 순간은 타격-전 세그먼트의 끝이자 팔로스루 세그먼트의
시작이므로, **공을 맞추는 바로 그 순간 모든 관절의 명령 가속도가 정확히
0**이 된다. 이 궤적은 sim뿐 아니라 `pipeline::run` → `hardware.command()`를
통해 **실제 로봇**에도 그대로 전달된다.

**수정**: `QuinticSegment`를 시작/끝 가속도를 명시적으로 받도록 일반화하고,
임팩트 knot에서 두 세그먼트가 공유하는 가속도를 **저크(jerk) 최소화**로
구한다(표준 다중 구간 스플라인 기법, 새 물리 가정 없음). 결과값은 실기
가속 한계의 50%로 보수적으로 클램프한다. 라이브 eval+랜덤 격자 재검증
결과 **커밋률·접촉률·네트통과율·인코트율 전부 회귀 없음**(100/100/100/100
eval 기준 그대로), `|v_out|/desired`도 0.6962→0.6969로 사실상 동일(소폭
개선). 임팩트 순간 관절 가속도는 예전 항상 정확히 `0.000`에서 **관절별
1.5~10 rad/s²**로 바뀌었다 — 이 수정이 실제로 목표한 효과가 직접
계측으로 확인된다.

---

## 2. 근본원인 (소스 직접 확인)

`QuinticSegment::new`(수정 전)은 3×3 경계조건 solve의 세 번째 행을 항상
`0.0`으로 하드코딩했다:

```rust
let b = NaVector3::new(qf - q0 - v0 * t, vf - v0, 0.0);
```

파일 자체 doc comment도 명시했다: "위치/속도 경계, **시작/끝 가속 0**".
`Trajectory::pre_impact_segments()`의 끝(`t = impact_time_secs`)과
`Trajectory::follow_through_segments()`의 시작(`t=0`)은 물리적으로
**같은 순간**(공 접촉 시각)인데, 둘 다 독립적으로 이 하드코딩된 0을
써서 우연히 연속(0=0)이지만 **항상 0**이었다.

물리적으로 토크 ≈ 관성×가속도 + 코리올리/중력항이므로, 가속도가 정확히
0인 순간에는 순수 관성 구동 토크 요구량도 최소에 가깝다 — "타격 순간에도
모터가 일을 하고 있어야 한다"는 사용자의 지적이 정확히 이 지점을 겨냥한다.

**sim 전용이 아니다**: `plan_swing`(`physics.rs`)이
`Trajectory::with_follow_through`의 유일한 프로덕션 호출자이고, 그 결과는
`src/pipeline/pipeline.rs::run` → `hardware.command(&trajectory)`로
`hardware::sim`과 `hardware::real::RealHardware` 양쪽에 동일하게 전달된다.

---

## 3. 설계

### 3.1 `QuinticSegment` 일반화

`q(t) = q0 + v0·t + (a0/2)·t² + c3·t³ + c4·t⁴ + c5·t⁵`로 확장(기존엔
`t²`항 자체가 없어 시작 가속도가 구조적으로 0이었다). 3×3 계수 행렬은
그대로, 경계값 벡터만 `[qf-q0-v0T-½a0T², vf-v0-a0T, af-a0]`로 바뀐다.
`a0=0, af=0`이면 예전과 **바이트 단위로 동일**한 출력을 낸다 —
`zero_acceleration_boundary_matches_legacy_shape` 테스트(독립적으로
재구현한 예전 수식과 비교)로 확인.

### 3.2 knot 가속도 선정 — 저크 최소화

`QuinticSegment::jerk_cost()`(`∫₀ᵀ q'''(t)² dt`, 닫힌 형태)를 추가했다.
knot 가속도 `a`는 타격-전/팔로스루 두 세그먼트의 저크 비용 합의 변수인데,
이 합이 `a`의 **정확한 2차식**(경계조건이 `a`의 아핀함수 → 계수도
아핀함수 → 비용은 2차식)이라는 사실을 이용해, 세 점(`-100, 0, 100 rad/s²`)
표본으로 2차식 정점을 대수적으로 그대로 복원한다(수치 탐색 아님).
`QuinticSegment::jerk_minimizing_knot_acceleration`.

이 선택은 표준 "natural spline" 기법이고 새로운 물리 가정이 없다 —
사용자가 제안한 "요구 라켓 동역학에서 역산"(Option B)은 새 물리 모델이
필요해 후속 과제로 미뤘다(계획서 참고).

### 3.3 안전장치

- 결과값을 `0.5 × max_joint_accel`로 클램프(`physics.rs::impact_knot_accelerations`)
  — 실물 로봇 벤치 검증 전 보수적으로 시작.
- 기존 `kinematic_limit_violation`(관절 각가속도 한계)·`peak_torque_utilization`
  (RNEA 기반 토크 한계)이 궤적 **전체**의 피크를 이미 다시 검증한다 —
  이 수정은 이 두 게이트를 우회하지 않고 그대로 통과시킨다.
- 방어 테스트(`extreme_knot_acceleration_trips_kinematic_limit_violation`):
  클램프가 없다고 가정한 극단값이 실제로 `kinematic_limit_violation`에
  걸리는지 확인.

---

## 4. 라이브 계측 — 임팩트 순간 관절 가속도

`tests/diag_wp12_pre_post_impact_travel.rs` 확장(`impact_acceleration`
필드 추가, `SimDebugSnapshot`에서 직접 읽음 — 재구현 아님). eval 30샷
기준(수정 후):

| 관절 | 임팩트 순간 속도/peak | 임팩트 knot \|가속도\| [rad/s²] |
|---|---|---|
| q0 base yaw | 0.230 | 1.69 |
| q1 shoulder | 0.970 | 6.10 |
| q2 elbow | 0.560 | 2.34 |
| q3 wrist | 0.972 | 8.79 |

수정 **전**에는 이 마지막 열이 모든 관절·모든 샷에서 정확히 `0.000`
이었다(설계상 강제). q1(shoulder)·q3(wrist)처럼 임팩트 순간 속도가 이미
peak에 가까운(0.97) 관절도 예전엔 가속도가 정확히 0이었다 — "빠르게
움직이지만 그 순간 가속하지도 감속하지도 않는" 상태. 지금은 5~10 rad/s²
의 실제 가속도를 낸다.

**참고**: q0(base yaw)의 임팩트 순간 속도/peak 비율(0.22~0.23)은 이 수정
전후로 거의 변화가 없다 — 이건 knot **가속도** 경계조건이 아니라 임팩트
**속도** 자체가 IK 단계(`linear_velocities_for_racket_velocity`, WP11/WP4a
영역)에서 낮게 배분되기 때문이며, 이 수정의 범위 밖이다(계획서 "이 계획이
주장하지 않는 것" 참고).

---

## 5. A/B — 회귀 없음

`tests/diag_wp10_coarse_track_per_joint.rs`(`COARSE_TRACK_JOINT_FRACTION=0.80`
고정, WP9/WP2b/WP11/WP4a 이후 이 세션의 참 베이스라인과 비교):

| 지표 | 수정 전 | 수정 후 |
|---|---|---|
| 커밋률 (eval) | 100.0% | 100.0% |
| 접촉률 (eval) | 100.0% | 100.0% |
| 접촉률 (random 5x5) | 84.0% | 84.0% |
| 네트통과율 (eval all) | 100.0% | 100.0% |
| 인코트율 (eval all) | 100.0% | 100.0% |
| `\|v_out\|/desired` (eval all) | 0.6962 | 0.6969 |

전 지표 회귀 없음. `COARSE_TRACK_JOINT_FRACTION`을 낮췄을 때(기각된 1차
시도) 봤던 네트통과율 붕괴(100%→83.3%)가 이 수정에서는 전혀 나타나지
않는다 — 서로 다른 메커니즘을 건드리기 때문(경계조건의 형태 vs 사전
이동 비율).

`cargo test --lib`: **248 passed, 1 failed**(기존 무관 실패
`hardware::dynamixel::tests::motor_mapping_matches_python_reference`,
origin에 이미 존재, 이 수정 범위 밖), **46 ignored** — 이 세션 시작 시점
베이스라인(246/1/46) 대비 새 테스트 2개만 추가, 회귀 없음.

`cargo clippy --lib --tests`: 사전에 이미 존재하던 4개 에러(`defaults/robot.rs`
의 `approx_constant`, `physics.rs:738`의 `if false && ...`)는 `git stash`로
확인한 결과 **수정 전 원본 `main`에도 동일하게 존재** — 이 수정이 만든
새 에러/새 경고 카테고리는 없음(기존 `needless_return` 스타일 경고만).

---

## 6. 미검증 사항 (정직하게 기록)

이 세션은 실물 로봇 벤치에 접근할 수 없다. sim의 강체 동역학·이상적
PD 추종에서는 안전해 보이는 nonzero knot 가속도가 실기에서는 백래시,
실제 모터 전류 한계, 구조적 공진 등 sim이 모델링하지 않는 방식으로
문제를 일으킬 가능성을 배제할 수 없다. 완화책으로 knot 가속도를
`max_joint_accel`의 50%로 클램프해 보수적으로 시작했다 — 실기 벤치
검증 후 이 클램프를 완화할지 결정할 것.

---

## 7. 재현 절차

```bash
# 단위테스트 (QuinticSegment 일반화·저크 최소화·회귀 안전성)
cargo test --lib motion::quintic_segment
cargo test --lib motion::trajectory
cargo test --lib motion::physics

# 임팩트 순간 가속도 라이브 계측
cargo test --release --test diag_wp12_pre_post_impact_travel -- --ignored --nocapture

# 회귀 게이트 (커밋률·접촉률·네트통과율·달성 세기)
cargo test --release --test diag_wp10_coarse_track_per_joint -- --ignored --nocapture

# 전체 테스트
cargo test --lib
```

---

## 8. 후속 과제

- Option B(요구 라켓 동역학에서 knot 가속도 역산) — 이 수정의 효과가
  실전에서 충분한지 확인 후 검토.
- 실물 로봇 벤치 검증 — 클램프 완화 여부 결정에 필요.
- base-yaw(q0)의 낮은 임팩트 순간 속도(peak 대비 0.22) — 이 수정의
  범위 밖(IK 속도 배분 문제, WP2b/WP11/WP4a 영역). 계속 눈에 띄는
  증상이면 별도 계획 필요.
