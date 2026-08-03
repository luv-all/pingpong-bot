# 스윙 파이프라인 진단 보고서

작성일: 2026-07-29 · HEAD: `95bb4ae` · 대상: "로봇팔이 원하는 대로 스윙/타격하지 못한다"

이 문서는 **타점 선정 → 필요 라켓 속도 역산 → 관절 궤적 생성 → 실제 구동(시뮬/실물)**
으로 이어지는 전체 스윙 파이프라인을 코드 레벨로 추적하고, 실제로 하네스를 실행해 얻은
측정값으로 실패 원인을 진단한다. 모든 주장에는 `파일:줄` 인용을 붙였다.

---

## 0. 결론 요약 (TL;DR)

라이브 측정(30샷 eval, release build, HEAD `95bb4ae`):

```
EVAL total=30/90  pass=false  (합격선 45점 초과)
contact=30/30  cleared_net=30/30  returned_in=30/30  하지만 score=1점 × 30
```

`contact ∧ cleared_net ∧ returned_in`이 전부 참인데 3점이 아니라 1점인 이유는
`eval_protocol.rs:140`의 파울 강등 규칙 — `bounced_own_half || double_hit` — 때문이다.
즉 **모든 리턴이 네트도 넘고 상대 코트 안에도 들어가지만, 그 전에 로봇 쪽 자기 코트에
먼저 튕기는 파울**이다. 이전 기록(`f445860`, 07-27)의 "리턴이 네트를 못 넘는다"는 진단은
더 이상 사실이 아니다.

물리적 원인은 라켓이 **접촉 순간 자기 면 법선 방향으로 거의 정지해 있다**는 것이다.

| 존 | 필요 `v_r·n` | 실제 `v_r·n` | 달성률 |
|---|---|---|---|
| Left | ~1.05 m/s | **0.048 m/s** | 4.6% |
| Center | ~1.05 m/s | **0.167 m/s** | 15.9% |
| Right | ~1.05 m/s | **−0.018 m/s** (역방향!) | 0% |

임팩트 모델 `v_out·n = (1+e)v_r·n − e·v_in·n`에 대입하면 스윙 자체의 기여는 출사
속도의 **~10%**뿐이고, 나머지 ~90%는 라켓이 정지해 있어도 나오는 수동 반발(passive
restitution)이다. 그 결과 리턴은 앞으로 나가는 힘(y 성분 43~58% 달성)보다 위로 뜨는
힘(z 성분 87~91% 달성)이 상대적으로 커서 **드라이브가 아니라 로브**가 되고, 로브는
멀리 못 나가 자기 코트에 먼저 떨어진다.

**이 결과를 만드는 원인은 하나가 아니라 파이프라인 여러 단계에 걸쳐 있다** (아래 §5에서
층별로 정리). 미리 밝혀둘 것: **PD 모터 추종 오차는 원인이 아니다** — 접촉 순간 관절
위치 오차는 최대 6.5 mrad(0.37°)로 육안으로 안 보이는 수준이다(§4.2). 문제는 계획
단계에서 애초에 약한 목표 속도를 만들고, 실행 단계에서 그마저 접촉 전에 다 못 쓰는
구조에 있다.

---

## 1. 파이프라인 개관

```
① 타점 선정        estimator::predict_hit_plane × InterceptWindow (y=0.08..0.35, step 0.03)
      ↓  Prediction { impact_position, incoming_velocity, time_to_impact_secs }
② 목표 속도 역산     planner::impact::rally_return_velocity → required_racket_velocity_parts (e_eff=0.55)
      ↓  desired v_out, required v_r(normal+lift)
③ IK + 관절속도 역산  planner::swing::physics::best_impact_candidate (다중 시드 IK, 조작성 랭킹)
      ↓  ImpactTarget { pose, joint_velocities, rail_velocity }
      ↓  (근특이점이면 NEAR_SINGULARITY_SPEED_RATIO로 균일 다운스케일)
④ quintic 궤적 생성   build_feasible_trajectory → fit_end_velocity (반복 다운스케일) → 팔로스루 추가
      ↓  SwingTrajectory
⑤ 커밋 상태기계       SimWorld::try_auto_swing — commit window [0.08s,0.35s], coarse-track 선추종
⑥ Rapier 모터 실행    drive_arm_motors → 위치모터 τ=k(q_tgt−q)−d·q̇, motor_max_force 클램프
⑦ (실물만) 하드웨어    Goal Position + Goal Current SyncWrite @ 200Hz, MX 내부 PID가 위치루프 담당
```

①~④는 `src/planner/`, ⑤~⑥은 `src/sim/physics/world.rs` + `src/robot/state.rs`, ⑦은
`src/hardware/`에 있다. 아래 §2~§4에서 각 단계를 순서대로 상세히 다룬다.

---

## 2. 계획 단계 (① ~ ④) — 타점 선정부터 궤적 생성까지

### 2.1 ① 타점 선정 — `predict_hit_plane`

`src/estimator/ballistics.rs:63-141`

로봇은 미리 정해진 y 평면 집합(`InterceptWindow::default()`, `src/defaults/planner.rs:27-36`:
`y_min=0.08, y_max=0.35, sample_step=0.03` → 10개 평면, `src/planner/mod.rs:53-71`
`hit_planes()`)에 대해, 공의 현재 위치·속도·각속도로부터 **반암시적 오일러 적분**을
`est.integrate_dt` 간격으로 굴려 각 평면과의 교차 시각·위치를 구한다.

게이트(모두 만족해야 `Some(Prediction)` 반환):
- `vy > -min_approach_speed_y` (공이 로봇 쪽으로 충분히 빠르게 오지 않으면 제외) — `:72-74`
- `is_table_rolling(position, velocity)`이면 제외 (테이블 위에서 구르는 공은 "임팩트"가 아님) — `:75-77`
- 이미 평면을 지났으면(`position.y <= plane.y`) 제외 — `:79-81`
- 적분 중 네트 라인을 통과할 때 클리어 높이 미달이면(`z_at_net + 0.012 < net_clear_z`) 그 평면 자체를 폐기 — `:101-114`
- 평면 교차 시각 `t_cross`가 `[min_lead, max_lead]` 밖이면 제외 — `:124-126`
- 교차 시점 `impact.z`가 바닥보다 낮으면 바닥으로 클램프, `SURFACE_Z+1.2`보다 높으면 폐기 — `:127-134`

이렇게 살아남은 예측들의 리스트가 `plan_best_swing`으로 넘어간다.

### 2.2 ③의 일부 — `plan_best_swing`이 "어느 타점"을 최종 선택하는가

`src/planner/swing/physics.rs:358-418`

1. `in_swing_commit_window(time_to_impact_secs)`로 커밋 창 `[0.08, 0.35]`초 안의 예측만 남긴다 — `:374`, 창 정의는 `ControlParams::min_swing_secs`/`swing_commit_max_secs` (`src/defaults/control.rs:50-51`).
2. 남은 예측들을 **현재 라켓 위치에서 가까운 순**으로 정렬한다(`:376-382`) — 즉 "가장 치기 쉬운 평면"이 아니라 "가장 이동이 적은 평면"을 우선한다.
3. 가까운 것부터 `plan_swing`을 시도하고, IK/궤적 생성이 실패하면(`InfeasibleSwing`) 다음 후보로 넘어간다(`:384-391`).
4. 성공한 궤적의 끝점에서 순방향기구학으로 실제 접촉점을 계산해, 예측 임팩트 위치와 `MAX_CONTACT_ERROR = 0.005`m 이상 벗어나면 그 후보도 버린다(`:400-405`).
5. 첫 성공 후보를 그대로 채택한다 — **"가장 강한 스윙이 가능한 평면"을 찾는 탐색은 없다.** 가까운 순으로 첫 번째로 실행 가능한 평면을 쓴다.

> `f445860`(07-27) 조사에서 "power-first hit-plane ranking"(스윙 세기 기준으로 평면을
> 재정렬)을 시도했으나 eval 점수에 중립적이었다고 기록돼 있다(§4.4 git 이력 참고) —
> 즉 평면 선택 순서 자체는 약한 스윙의 원인이 아니라는 것이 이미 실측으로 배제됐다.

### 2.3 ② 목표 출사 속도 — `rally_return_velocity`

`src/planner/impact.rs:15-31`

```
target = (WIDTH/2, LENGTH*0.75, SURFACE_Z + BALL_RADIUS)   // 상대 코트 중앙 바운드 목표
t = ImpactParams::rally_time_to_bounce = 0.55s
v_out = (target − impact − (0,0, 0.5·G_Z·t²)) / t          // 탄도 경계값 문제의 닫힌 해
if |v_out| > max_return_speed(6.0):  v_out *= 6.0/|v_out|   // 상한 클램프
```

무저항 탄도로 "0.55초 뒤 상대 코트 중앙에 떨어지는" 속도를 역산하는 순수 기하 공식이며,
IK나 로봇 상태와 무관하다. 여기서 나온 `v_out`이 다음 단계의 입력이다.

### 2.4 ② 필요 라켓 속도 — `required_racket_velocity_parts` (법선/리프트 분리)

`src/planner/impact.rs:59-101`

임팩트 모델(법선 성분만 결정): $v_{out}\!\cdot\!n = (1+e)(v_r\!\cdot\!n) - e(v_{in}\!\cdot\!n)$

```rust
v_r_n = (v_out_n + e·v_in_n) / (1+e)          // 법선 성분 역산
lift_t = (0,0, v_out_t.z) 투영 후 법선성분 제거  // 월드 +Z 리프트만, 수평 접선은 버림
반환: (n·v_r_n,  lift_t)                       // 두 벡터를 따로 반환
```

`e = ImpactParams::racket_effective_restitution = 0.55` (`src/defaults/impact.rs:37`,
`docs/measure-physics.md:51`에 "튜닝값, 스윙·장착 면으로 측정 필요"라고 명시 — **실측
아님**).

주석(`impact.rs:33-58`)에 남은 이력이 중요하다: 예전에는 접선 전체(횡방향 포함)를
"점착 가정"으로 합쳐 넘겼는데, 실측(`tests/diag_weak_return.rs`, 2026-07-27)으로
**법선 1.070 / 리프트 1.157**의 비율이 확인됐고, 최소노름 IK가 제곱 노름으로 예산을
나누므로 **관절 예산의 54%가 출사 속도에 전혀 기여하지 않는 리프트 축으로 새고**, 그 결과
정작 유효한 법선 성분이 **필요치의 1/6인 0.178 m/s**로 뭉개졌다. 지금 코드는 그래서
법선과 리프트를 분리해 반환하고, 호출부(`best_impact_candidate`)가 둘을 합쳐서 쓴다
(`physics.rs:182-193`에서 `required_racket_velocity`로 재합산 — **분리했다가 다시
합쳐서 IK에 넘긴다**는 점에 주의. 아래 §5-RC2 참고).

### 2.5 ③ IK + 관절속도 역산 — `best_impact_candidate`

`src/planner/swing/physics.rs:145-231`

1. `v_out = rally_return_velocity(...)`, `desired_normal = normalize(v_out - v_in)`.
2. 손목 시드각을 `Arm::wrist_open_for_return(v_out - v_in)`로 구해 초기 힌트를 만든다(`:155`). *(주의: `with_wrist_open` 내부에서 부호가 반전된다 — §5-RC4 참고.)*
3. `candidate_ik_hints`(physics.rs:107-127)로 **어깨/팔꿈치 한계 중점 기준 반사 시드 최대 4개**를 만든다 — 같은 목표 자세라도 팔꿈치가 위로 굽었는지 아래로 굽었는지(elbow-up/down) 등 다른 관절 조합(basin)에 수렴하게 해서, 자코비안 조작성이 우연히 나쁜 해에 갇히는 것을 피한다. 주석에 "동일 목표에 대해 조작성이 최대 7배 차이 남을 실측 확인"이라고 적혀 있다(`:103-106`).
4. 각 시드에 대해:
   - `arm.inverse_pose_with_rail(...)`로 위치 IK를 푼다.
   - `table_penetration > 1e-3`이면 버린다(테이블 관통 자세 배제, `:176-178`).
   - 그 자세의 실제 법선으로 `required_racket_velocity`를 다시 계산한다(자세마다 법선이 다르므로 목표 라켓 속도도 다시 구해야 함, `:182-193`).
   - `arm.linear_velocities_for_racket_velocity`로 **위치 3제약만**(방향 유지 없음)의 최소노름 속도 IK를 푼다(`:197-204`). 주석: "실제 스윙도 접촉 순간 라켓이 계속 회전 중이라 방향 고정은 물리적으로 과잉제약이었다"(2026-07-23 실측).
   - `peak_joint_speed_ratio = max(|q̇_i|) / arm.max_joint_speed`가 가장 낮은(=조작성이 가장 좋은) 후보를 채택(`:205-220`).
5. 모든 시드가 실패하면 마지막 에러를 반환.

**5제약(위치+방향유지) 버전은 코드에 존재하지만(`src/robot/mod.rs:726-836`) 사용되지
않는다** — 테스트에서만 호출된다. 그 doc comment가 남긴 실측: 방향유지 제약을 제거한
것만으로 **측정 피크 관절속도가 17.55 → 11.25 rad/s로 줄었다**(`mod.rs:898-908`).
11.25 rad/s도 여전히 실제 한계 2.88 rad/s의 **3.9배**다.

### 2.6 ③ 근특이점 스케일링 — `NEAR_SINGULARITY_SPEED_RATIO`

`src/planner/swing/physics.rs:73-86, 233-283`

`peak_joint_speed_ratio > 2.5`이면(요구 관절속도가 실제 한계의 2.5배 초과) 그 자세를
버리지 않고, **모든 관절속도·레일속도·라켓속도를 `1/ratio`로 균일 축소**해서 스윙을
"약하게라도" 내보낸다(`:256-274`).

이 상수의 주석(`:73-86`)에 실측이 남아 있다: 재보정된 관절 한계(~2.88 rad/s) 아래서
손목이 거의 다 펴지거나 접힌 자세 근처(reach 경계)의 IK 해가 **평범한 목표(2 m/s급)에도
한 축이 한계의 6배(17.5 rad/s)**로 튀는 걸 확인했고, 예전에는 이걸 `NearSingularity`로
하드 거절했으나 그러면 **실기 관절속도 + 현재 마운트/슈터 조합에서 거의 모든 샷이
걸려 스윙이 한 번도 커밋되지 않았다**(`:251-255`). 그래서 지금은 거절 대신 다운스케일을
택했고, 그 대가로 실측된 사례: **목표 라켓속도 2.0 m/s → 실제 피크 0.332 m/s (17%만
남음)**.

즉 이 단계는 **"커밋이 아예 안 되는 것보다는 약하게라도 치는 게 낫다"는 명시적 트레이드오프**이며, 지금 관측되는 약한 스윙의 상당 부분이 여기서 만들어진다.

### 2.7 ④ quintic 궤적 생성 — `build_feasible_trajectory` / `fit_end_velocity`

`src/planner/swing/physics.rs:603-671, 830-895`(§는 agent 조사에서 인용, 직접 재확인)

`fit_end_velocity`는 최대 32회 반복하며 매번:
```
torque_util = peak_torque_utilization(arm, trajectory)   // 이상적 RNEA 피드포워드 가정!
if torque_util <= 1.0 && kinematic_limits_ok: 반환
speed_scale  = min(1, max_joint_speed/peak_speed * 0.95)
accel_scale  = min(1, max_joint_accel/peak_accel * 0.95)
torque_scale = min(1, 1/torque_util * 0.95)
scale = min(speed_scale, accel_scale, torque_scale)
end_velocity *= scale   // 전 관절에 동일 배율
rail.end_velocity *= scale
```

여기서도 **전 관절 동일 배율** 다운스케일이 반복된다(§2.6과 같은 패턴이 계획 단계에서
두 번 일어난다). `peak_torque_utilization`은 **완벽한 계산 토크 피드포워드를 가정한
이상적 RNEA**로 판정하므로(마찰·로터 관성·기어 손실 없음, §3의 RNEA 한계 참고), 계획은
"완벽하게 추종하면 이 토크로 충분하다"고 판단하지만 실행은 포화 PD로 이뤄진다 — 계획과
실행의 모델이 다르다.

`build_feasible_trajectory`는 마지막에 세 가지를 검사해 실패하면 원인을 구분해
반환한다(`:634-669`): ① `kinematic_limit_violation`(관절각/속도 초과, 기구학·마운트
문제), ② `peak_torque_utilization > 1.0`(토크 초과, 모터 선정 문제), ③
`trajectory_collision_free`(테이블 관통).

성공한 궤적에는 `swing_follow_through_secs = 0.06`초(`control.rs:52`)만큼의 팔로스루
구간이 덧붙는다(`trajectory_with_follow_through`, `:673-710`) — 임팩트 속도로 관성
운동을 이어가는 구간이며, 이 구간의 관절 이동량이 §4.1에서 언급하는 "임팩트 전:후
이동량 1:36 비율"의 "후" 쪽을 만든다.

---

## 3. 실행 단계 (⑤ ~ ⑥) — 커밋 상태기계와 Rapier 모터 (전량 조사 에이전트 실측·소스 추적)

> 이 절과 §4는 코드베이스를 직접 실행해 확보한 실측치를 포함한다. 정확한 함수 서명과
> 줄 번호는 각 항목에 인용돼 있다.

### 3.1 커밋 이전 — coarse 추종의 숨은 결함

`try_auto_swing`(`src/sim/physics/world.rs:573-807`)은 공이 midcourt를 넘기 전
(`ball_past_midcourt_for_commit`, `physics.rs:68-71`: `ball_y > LENGTH_Y*0.55`)까지는
`plan_coarse_track_targets`로 **대략적인** 목표 자세를 계산해 미리 이동시킨다
(`world.rs:637-655`). 이때 관절 목표는 정지 자세(`default_joints`)와 예측 임팩트
자세 사이를 `COARSE_TRACK_JOINT_FRACTION = 0.80`(`world.rs:167`)만큼 블렌드한 값이다.

**문제**: 이 coarse 목표는 `RobotState::set_targets`로 그대로 꽂히고, 실제 이동을
담당해야 할 `step_toward_targets`(관절 rate limit + `clamp_above_table`을 가진 유일한
함수, `src/robot/state.rs:335-364`)는 **프로덕션 코드 경로에서 전혀 호출되지 않는다**
(전체 리포지토리에서 호출부가 `src/robot/tests.rs:88` 단 하나). `world.rs:624-626,
641-644`의 주석은 "실제 이동은 rate-limited·table-clamped 추종 루프가 처리한다"고
적혀 있지만 **이는 사실이 아니다** — coarse 목표는 매 물리 틱(1kHz) **무제한 스텝
입력**으로 Rapier 모터에 직접 들어가고, 유일한 제한은 `motor_max_force` 토크 클램프뿐이다.

**실측 결과**(`diag_motor_tracking`, `tests/diag_weak_return.rs:282-359`, 실제 실행):

```
joint  peak_err_swing[rad]   peak_err_all[rad]   post_swing_dir_flips
  q0            0.00112             0.46526                    7
  q1            0.00051             0.16717                   11
  q2            0.01018             0.14270                    3
  q3            0.00469             0.43269                    6
```

스윙 중(swing) 오차는 0.5~10 mrad로 무시할 만하지만, **전체 구간(all) 오차는 최대
465 mrad = 26.7°** — 이것이 바로 이 coarse 추종 구간에서 나온다(스윙 중 오차의 최대
46배). 스윙 종료 후 방향 반전이 3~11회 있어 채터/진동도 관측된다.

즉 **사용자가 눈으로 보는 "팔이 명령대로 안 움직인다"는 인상의 지배적 성분은 실제
타격 스윙이 아니라 그 이전의 coarse 선추종 단계에서 나온다.**

### 3.2 커밋 — 상태기계와 트레이드오프

`world.rs:573-807`, 순서대로:

| 단계 | 조건 | 실패 시 |
|---|---|---|
| 비행 중 아니면 스킵 | `ball_state == InFlight` | 대기 |
| midcourt 전 | `!ball_past_midcourt_for_commit` | coarse 추종만(§3.1) |
| `tti < 0.08s` | `min_swing_secs` 미달 | **이번 공 포기** |
| `tti ∉ [0.08, 0.35]` | commit 창 밖 | 대기 |
| 20ms 스로틀 후 `plan_best_swing` | | |
| `JointOrTorqueLimit` | | **즉시 이번 공 포기**(모터 보호) |
| 성공 | | `replace_swing`, `swing_committed=true` |

`COARSE_TRACK_JOINT_FRACTION`의 doc comment(`world.rs:134-166`)에 실측 스윕 표가
남아 있다: 0.80(현재값)에서 커밋률이 가장 높고, 0.50에서 91%→50%로, 0.00(선추종
없음)에서 20%로 떨어진다. 즉 **선추종을 줄이면 임팩트 전 이동량은 늘어 스윙이
세지지만 커밋 자체가 실패하는 빈도가 급격히 늘어난다** — 이것이 §0에서 말한 "세게
치기 vs 아예 치기"의 트레이드오프이며, 현재 0.80은 커밋률만을 기준으로 고른 값이다.

### 3.3 Rapier 관절 위치 모터 — 정확한 제어 법칙

`src/sim/physics/arm_bodies.rs:126-162`(스폰), `:271-287`(매 틱 갱신)

```
τ_i = k_i·(q_target,i − q_i) − d_i·q̇_i         // target_vel = 0 고정
τ_i ← clamp(τ_i, −motor_max_force_i, +motor_max_force_i)
```

게인은 `src/defaults/sim_motor.rs:56-74`에서 공통 대역폭 `ω_n = 2000 rad/s`, 임계감쇠
`ζ=1`로 유도: `k_i = ω_n²·I_i`, `d_i = 2ω_n·I_i`, `I = [3.373e-2, 1.617e-2, 1.429e-2,
2.196e-3]` (관절별 반사 관성, `mass_matrix` 대각에서 측정) → `k = [134920, 64680,
57160, 8784]`, `d = [134.92, 64.68, 57.16, 8.784]`.

**포화 구간의 오차 법칙**: `motor_max_force`로 클램프될 때 `k`·`d`의 절대 크기는
사라지고 비율 `d/k = 2ζ/ω_n`만 남아, `|q−q_cmd| ≈ (d/k)·q̇`가 된다 — **가장 빠른
관절이 가장 뒤처진다.** 이 모델은 실측으로 검증됐다: ζ=1·ω_n=1000 설계가 옛 균일
게인(k=5000, d=10)과 관절별 0.01 mrad 이내로 일치했는데, `10/5000 = 2/1000`이 정확히
그 ω_n=1000의 `d/k`와 같기 때문이다.

`motor_max_force` 자체는 `drive_arm_motors`(`world.rs:1166-1191`)에서 매 틱 재계산된다:

```
motor_max_force_i = clamp( 1.15·|τ_RNEA,i(q_cmd,q̇_cmd,q̈_cmd)|,  0.25·τ_max,i,  τ_max,i )
```

`τ_RNEA`는 **직전 틱**에 계산된 값(1ms stale, 영향 미미)이며, RNEA는 **항상 중력을
포함**하는데 시뮬 팔 링크는 `gravity_scale(0.0)`으로 **무중력**이라(`arm_bodies.rs:
166-171`) 이 클램프는 시뮬에서 대체로 여유 있게 작동한다(binding하지 않음).

### 3.4 실제 포화 계산

접촉 근처 elbow 실측(§4.1): `q̇ = 2.736 rad/s`, 위치오차 `0.33 mrad`.
```
k_2·e   = 57160 × 3.3e-4  =  18.9 N·m
d_2·q̇   = 57.16 × 2.736   = 156.4 N·m
τ_PD    = 18.9 − 156.4    = −137.5 N·m       (한계 1.25 N·m의 110배)
```
즉 스윙 고속 구간에서 elbow 모터는 **최대 제동 토크로 완전 포화**하며, yaw도 비슷한
계산으로 한계의 10.7배로 포화한다. 다만 이는 **감속 방향**(운동을 막는 방향)의 포화이지,
"명령을 못 따라가서 느려지는" 종류의 지연이 아니다 — §4.2가 보여주듯 실제 위치 추종
자체는 매우 정확하다(≤10 mrad). 포화는 진동을 억제하는 감쇠 항이 하는 정상적인
동작이며, 문제의 근원이 아니라 §2.6·§2.7에서 이미 약해진 명령 속도를 그대로 실행하고
있을 뿐이다.

### 3.5 라켓 속도 미스터리 — 명령의 48.9%만 실현 (제어 문제 아님)

`diag_swing_timeseries`(`tests/diag_weak_return.rs:192-274`) 실측(release):
```
step  elapsed  cmd|vr|  act|vr|   ...
439   0.229    0.414    0.201     (0.4855)
460   0.250    0.591    0.290     (0.4907)
480   0.270    0.715    0.350     (0.4895)
490   0.280    0.744    0.364     (0.4892)  ← 접촉 직전
```

실제 라켓 속도(FK/Rapier 링크 속도)가 명령 라켓 속도의 **48.5~49.1%로, 스윙 전
구간에서 ±1% 이내로 일정**하다. 이 비율이 **상수**라는 사실 자체가 제어 지연이나 토크
포화로는 설명되지 않는다는 신호다 — 포화라면 비율이 가속도·속도에 따라 변해야 한다.
게다가 위치 오차(≤10 mrad)와 속도 오차(~50%)는 같은 기구학 모델 안에서 양립할 수
없다: 0.38 m/s의 지속적 속도 결손이 0.28초 누적되면 ~280 mrad의 위치 지연이 남아야
하는데, 실측 위치 오차는 그 1/30 수준이다.

궤적 자체의 자기모순은 배제됐다(위치·속도·가속이 모두 같은 `QuinticSegment::sample`의
해석적 미분에서 나옴, `trajectory.rs:96-106, 275-330`). 유력한 후보는 Rapier
멀티바디의 속도 필드 읽기 시점 불일치다: `Multibody::update_rigid_bodies`(Rapier
0.34.0 소스, `multibody.rs:1195-1227`)는 위치(`pos`)만 갱신하고 `rb.vels`는 건드리지
않으며, `SimWorld::step`은 물리 파이프라인 **이후**에 `forward_kinematics` +
`update_rigid_bodies`를 호출한다(`world.rs:452-457`) — 즉 읽어오는 `body.linvel()`이
그 위치 갱신과 정합되지 않는, 스텝 시작 시점의 값일 수 있다.

다만 **이것이 읽기 버그이든 실제 물리 손실이든, 시뮬 안에서는 실재하는 현상이다** —
Rapier의 공-라켓 충격량 계산도 같은 `rb.vels` 필드를 쓰기 때문에, 라켓은 실제로 절반
속도로 공을 친다. `diag_weak_return`이 측정하는 "리턴이 약하다"는 결과는 이 필드
위에 세워져 있다.

### 3.6 접촉 타이밍 불일치

`swing_bench --sim-verify` 실측:
```
planned impact_time_secs: 0.2937 s
actual contact at:        0.2890 s   (계획보다 4.7 ms 이름)

joint      err@real_contact   err@planned_impact   peak_commanded_speed
0 yaw      0.00049 rad        0.00006 rad          0.477 rad/s
1 shoulder 0.00000 rad        0.00000 rad          0.000 rad/s   ← 설계상 정지
2 elbow    0.00074 rad        0.00033 rad          2.736 rad/s
3 wrist    0.00653 rad        0.01987 rad          1.926 rad/s
```

Rapier `ContactPair`는 실제 라켓 형상 기준으로 발동하므로 계획된 `impact_time_secs`와
동기화되지 않는다 — 팔이 계획된 임팩트 속도(가속 구간의 끝)에 **도달하기 전에** 공을
맞힌다. 그리고 **shoulder(joint 1)의 명령 속도는 0.000 rad/s로, 이 샷에서는 설계상
정지 상태**다 — "칠 때 base/shoulder가 안 움직인다"는 증상의 상당 부분은 추종 실패가
아니라 **계획이 애초에 그 관절을 안 쓰기로 결정한 것**이다(§2.5의 조작성 랭킹과
§2.6의 근특이점 스케일링이 어느 관절을 쓸지 결정한다).

wrist의 `err@planned_impact = 19.87 mrad(1.14°)`는 추종 오차가 아니라 **공 충격의
반작용**이다 — 접촉 프레임에는 이미 공의 임펄스가 관절을 밀어낸 뒤이기 때문이다
(`world.rs:2697-2705` doc comment도 동일 현상 기록).

---

## 4. 실물 하드웨어 경로 — 시뮬과 공유하지 않는 별도 장애물

실물 경로(`src/hardware/real.rs:149-230`, `src/hardware/dynamixel.rs`,
`src/hardware/rail/axl.rs`)는 §3의 Rapier 모터 모델을 **전혀 공유하지 않는다.**
`SwingTrajectory`를 200 Hz(`stream_hz`)로 샘플해 Goal Position + (옵션) Goal Current를
SyncWrite하고, 위치 제어 루프는 MX 서보 내부 PID가 돈다. 인코더 피드백은 스윙 중
**한 번도 읽지 않는 순수 개루프**다.

시뮬에는 없는, 실물에만 있는 1차 장애물 목록:

| # | 문제 | 근거 | 영향 |
|---|---|---|---|
| 1 | **Profile Velocity 80 = 1.9185 rad/s** < 플래너 명령(elbow 2.736, wrist 1.926 rad/s) | `defaults/hardware.rs:26` (레지스터 80) vs `dxl_limits.rs:16-17`(한계 2.88) — 변환식 `dynamixel.rs:26-28` | 실측 명령 피크 **둘 다 이 상한을 초과** — 서보가 내부적으로 속도를 잘라 스윙이 계획보다 늦고 느려짐. 교차검증 코드 없음 |
| 2 | Goal Current가 **가산 피드포워드가 아니라 전류 상한**(모드 5) | `hardware.rs:23`(모드 5=Current-based Position), `real.rs:200-206`, `dynamixel.rs:399-420` | 시뮬은 최소 `×1.15` 마진 + `0.25·τ_max` 바닥을 두지만(`world.rs:1184-1186`) **실물엔 마진이 0**. RNEA가 가속도 항이 빠지는 속도 피크 근처에서 전류 상한이 최소가 되는 시점이 하필 임팩트 직전과 겹침 |
| 3 | 듀얼 yaw 미러 슬레이브 전류 **부호 미반전** | `dynamixel.rs:434-454` — 위치는 미러링(`2·zero−m`)하지만 전류는 마스터와 동일 부호로 전송 | 기구가 거울 배치라면 두 MX-64가 서로 반대 방향 토크를 내 **서로 싸울 위험**. `τ_max[0]=6.0`(두 모터 협력 전제)의 근거 전체가 이 위에 있음. 실기 검증 흔적 없음 — **최고 위험 미검증 가정** |
| 4 | AXL 레일 `AxmMovePos` 5ms 재명령, `accel=12 m/s²`로 0→5m/s에 0.42초 필요 | `axl.rs:225-237`, `defaults/hardware.rs:60-62` | 스윙 전체(0.29초)보다 김. 시뮬 레일은 스윙 중 오차 0·가속 무한으로 시뮬레이션됨(`state.rs:242`) — **레일은 실물이 시뮬보다 훨씬 느림** |
| 5 | 스윙 중 인코더 피드백 없음(개루프), 통신 시간 미보상 `sleep(tick)` | `real.rs:165-228` | 실제 루프 주기가 5ms보다 늘어날 수 있음(재시도 시 20ms 블로킹, `hardware.rs:27-28`), 계측 없음 |

시뮬-실물 대표성 비교(핵심 항목만, 전체는 §6 부록):

| 항목 | 시뮬 | 실물 | 판정 |
|---|---|---|---|
| 위치 루프 | Rapier PD (§3.3) | MX 내부 PID(게인 미지) | 시뮬 전용 모델, **미실측**(`docs/measure-physics.md` 명시) |
| 중력 | 팔 링크 `gravity_scale(0.0)` | 실제로 자중 버팀 | 시뮬 전용 조작 — 자중 유지 능력 미검증 |
| 링크 질량/관성 | 하드코딩 0.04/0.08kg 점질량, 회전관성 사실상 0(§6 부록 상세) | URDF 실측 관성 | 시뮬 팔이 실물보다 훨씬 가볍고 "과게인" 상태 |
| 레일 | 텔레포트, 오차 0 | AXL 사다리꼴 프로파일, accel 12 m/s² | 시뮬이 실물보다 훨씬 빠름 |
| coarse 추종 rate limit | 없음(§3.1, 죽은 코드) | 서보 Profile Velocity가 강제 | **반대 방향 불일치** — 시뮬이 실물보다 관대 |

---

## 5. 근본 원인 — 층별 정리

증상("의도대로 스윙하지 않는다")은 단일 버그가 아니라 파이프라인에 누적되는 **네 개의
독립적인 층**이다. 우선순위(기여도 추정 순):

### RC-1. 계획 단계에서 이미 목표 속도가 약하게 정해진다 (가장 큰 기여, 시뮬·실물 공통)

- §2.5: 위치+방향 5제약 IK는 조작성이 나빠 쓰지 않고, 3제약만으로도 최선의 조작성 조합을 찾지만 여전히 elbow(MX-28T, 가장 작은 토크 예산)가 병목이다.
- §2.6: 근특이점(ratio>2.5)이면 전 관절 균일 다운스케일 — 실측 사례 2.0→0.332 m/s(17%).
- §2.7: `fit_end_velocity`가 다시 최대 32회 전 관절 균일 다운스케일.
- §2.4: 법선/리프트 분리로 예산 낭비는 줄였지만(과거 54% 낭비 대비), 여전히 두 성분을 합쳐서 최소노름 IK에 넘기므로 완전히 해소되지는 않았다.
- §3.6 실측: shoulder 명령 속도 **0.000 rad/s** — 이 관절은 이번 스윙에서 설계상 아예 쓰이지 않는다.

이 층의 증거는 §4.1(swing_bench)의 `v_r_cmd = 0.546 m/s` 자체가 필요한 법선 성분
(~1.05 m/s)의 절반 수준이라는 것이다 — **실행이 완벽해도 이미 부족한 목표를 받는다.**

### RC-2. coarse 추종이 rate-limit 없는 스텝 입력이다 (시뮬 전용 결함, 실측 최대 오차원)

§3.1: `step_toward_targets`(관절 rate limit + 테이블 클램프 보유)가 프로덕션에서
호출되지 않는 죽은 코드이고, 관련 주석은 사실과 다르다. 실측 `peak_err_all = 465
mrad(26.7°)`은 스윙 중 오차의 46배 — **사용자가 체감하는 "명령대로 안 움직인다"의
가장 큰 성분**이다.

### RC-3. 접촉이 계획된 임팩트보다 먼저 발동한다 (시뮬·실물 공통 개념, 실측은 시뮬) — ✅ 2026-07-30 해결

§3.6: Rapier `ContactPair`는 계획 시각과 무관하게 실제 형상 접촉으로 발동 — 실측
−4.7ms. 팔이 속도 피크에 도달하기 전에 공을 맞혀, 계획된 목표 속도조차 다 못 쓰고
임팩트가 끝난다.

**후속 조사(WP6, `tests/diag_contact_timing.rs` · `tests/diag_table_restitution.rs`)로
근본 원인을 확정했다.** 처음 가정했던 두 후보(라켓 면의 기하학적 스윕, 예측기·Rapier
적분 파라미터 불일치) 둘 다 아니었다 — 실제 원인은 **Rapier
`num_solver_iterations`가 12로 너무 낮아 접촉이 서브틱 위상 아티팩트에 크게
좌우된 것**이었다. 같은 원인이 테이블 반발계수 실현치도 갉아먹고 있었다: 설정값
`e=0.88`인데 실측 유효 \(e\)는 평균 0.789(산포 0.10)였고, 낙하 높이를 0.1mm만
바꿔도 \(e\)가 0.69~0.85로 요동해 산포의 원인이 속도 의존 물리가 아니라 이산
시간 접촉 해석임을 확인했다(`diag_effective_restitution_subtick_phase`).
`num_solver_iterations`를 **12→32**로 올리자 두 증상이 동시에 사라졌다:

| 지표 | 이전(12) | 이후(32) | 설정/목표값 |
|---|---|---|---|
| 접촉 타이밍 오차 평균 (`d_total`) | −3.91 ms | **+0.02 ms** | 0 |
| 테이블 유효 \(e\) 평균 | 0.789 (산포 0.101) | **0.878 (산포 0.002)** | 0.88 |
| `swing_bench --sim-verify` 갭 | −4.7 ms | **+0.1 ms** | 0 |
| 물리 틱 예산초과 횟수 | 80/3000 | **1/3000** | — (부수 개선) |

`src/sim/physics/world.rs`의 `SimWorld::with_physics`에 반영됐고, 근거는
`docs/measure-physics.md`의 "Rapier 솔버 반복 횟수" 절에 상세 기록했다.
`normalized_prediction_distance`를 낮추는 대안도 비슷한 효과가 있었으나
(e 0.878, d_total +0.59ms) 두 지표 모두 `solver_iters=32`가 더 나았다.

부수 발견: 진단 하네스(`diag_contact_timing.rs`)의 예측 평면 통과 시각
계측(`plane_cross_secs`)이 30/30 샷에서 전부 `None`으로 나와, "예측기 오차 vs
기하 스윕" 성분 분해는 이번에 검증하지 못했다 — 테스트 자체의 결함(커밋된
스윙이 타겟한 평면과 `debug_prediction()`이 반환하는 평면이 어긋날 수 있음)으로
보이며 후속 과제로 남겼다. `d_total`(계획 대비 실제 접촉 시각)은 이 버그와
무관한 별도 계측이라 위 결론에는 영향이 없다.

### RC-4. 토크 실현성 판정이 이상화돼 있다 (RNEA 낙관 편향)

§2.7, §3.3: 계획 단계 `peak_torque_utilization`은 완벽한 계산 토크 피드포워드(마찰
없음, 로터/기어 관성 없음, 공 충격 반작용 없음, 레일 가속 관성 없음, 관절 friction
없음)를 가정한 RNEA로 판정한다. 유일한 안전마진은 `CONTINUOUS_TORQUE_DERATE = 0.5`
(`defaults/dxl_limits.rs:10`)인데 이 자체가 "실측 확인 필요"로 명시된 값이다. 계획이
"이 토크면 충분하다"고 판단한 스윙이 실물에서는 마찰·기어 손실 때문에 부족할 수 있다 —
아직 정량화되지 않았다.

### 부수 원인 — 라켓 속도 읽기 상수 배율 (§3.5)

명령 대비 실제 라켓 속도가 항상 48.9%라는 것은 제어 문제가 아니라 Rapier 멀티바디
속도 필드 읽기 타이밍 문제일 가능성이 높지만(§3.5), **원인이 무엇이든 시뮬 안에서는
실재하는 손실**이며 `diag_weak_return`의 모든 "약한 리턴" 판정이 이 위에 있다. 가장
먼저 확인해야 할 항목이다(§5의 권고 1 참고) — 이게 만약 읽기 버그라면, RC-1~RC-4를
다 고쳐도 진단 도구 자체가 실제보다 훨씬 나쁜 숫자를 계속 보여주게 된다.

### 배제된 가설 (이미 실측으로 부정됨, `f445860` 기록)

- 모터 PD 추종 오차 — 스윙 중 ≤10 mrad, 육안으로 안 보임
- 접촉 `e_eff` 자체의 오류 — 스윕(0.42~0.82) 결과 최선이 22% 클리어율, 지배적 원인 아님
- `NEAR_SINGULARITY` 재분배, 가중 최소노름, 백스윙 선자세, 마운트 재배치 — 모두 eval 점수에 중립

---

## 6. 권고 (우선순위순)

| # | 조치 | 노력 | 근거/기대효과 |
|---|---|---|---|
| 1 | **라켓 속도 48.9% 배율의 정체 확정** | 소 | 나머지 모든 결론의 전제. `diag_swing_timeseries`에 관절속도 유한차분(FK) vs `trajectory.sample_velocity_at(t)` 비교 열 추가. 같으면 `body.linvel()` 읽기 문제(§3.5), 다르면 진짜 속도 손실 |
| 2 | **coarse 추종에 rate limit 복원** | 소~중 | 실측 최대 오차(465 mrad) 직접 제거. `step_toward_targets` 경로를 실제로 배선하거나 coarse 목표에 별도 slew limit 추가. **단, `COARSE_TRACK_JOINT_FRACTION` 스윕을 다시 돌려 커밋률 회귀가 없는지 반드시 확인**(`diag_swing_commit_rate_across_shot_grid`) |
| 3 | **접촉 시점 동기화 여유 확보** | 중 | `impact_time_secs` 계산에 라켓 형상 반경만큼 여유를 두거나, 접촉 감지를 팔 속도 피크 이후로 유도 |
| 4 | **RNEA에 마찰/로터 관성 반영, `CONTINUOUS_TORQUE_DERATE` 재실측** | 중(벤치 측정 필요) | 모든 토크 게이트와 `fit_end_velocity`의 판정 기준이 이 위에 있음. 현재는 반증 불가능한 가정 |
| 5 | **`e_eff`, 입사속도(v_in) 실측** | 중(하드웨어) | `f445860` 민감도 분석: 입사속도 20% 증가가 스윙 기여 300% 증가와 동급. 현재 두 값 다 튜닝 추정치 |
| 6 | **실물 Profile Velocity를 플래너 한계와 교차검증** | 소 | 이미 있는 `dynamixel_profile_velocity_to_rad_s` 변환기를 `RealHardware::new`/`validate`에서 실제로 사용해 레지스터 값(80)이 `arm.max_joint_speed`(2.88 rad/s → 필요 레지스터값 ~120 이상)보다 작으면 경고/거부 |
| 7 | **듀얼 yaw 미러 전류 부호를 실기에서 검증** | 소(작업)/최고(위험) | yaw만 저속 구동하며 ID1/ID2 Present Current 부호 비교. 반대여야 하면 코드 수정 필요 — `τ_max[0]=6.0`의 근거 전체가 걸려 있음 |

---

## 부록 A. 실측 데이터 원본

`diag_weak_return`(30샷 전수, release 빌드) 존별 대표값(존 내 10샷 결정론적으로 동일):

| 존 | v_out_desired | v_out_actual | \|v_out\| 목표/실제 | v_racket(actual) | v_r·n | vout_n 모델/실제 | 역산 e_eff | z@net |
|---|---|---|---|---|---|---|---|---|
| Left | [0.92,3.58,2.20] | [0.37,1.54,1.91] | 4.30/2.48 | [-0.31,0.08,0.16] | 0.048 | 2.448/1.96 | 0.436 | 0.934 |
| Center | [0.00,3.57,2.24] | [0.00,2.06,1.94] | 4.22/2.83 | [0.00,0.14,0.17] | 0.167 | 2.659/2.41 | 0.494 | 1.082 |
| Right | [-0.90,3.47,2.21] | [-0.43,1.92,2.02] | 4.21/2.82 | [0.33,0.03,0.13] | −0.018 | 2.337/2.34 | 0.550 | 1.141 |

`net_top = 0.9325` — Left는 1.5mm 차이로 겨우 클리어.

y(전진) 성분 달성률: Left 43%, Center 58%, Right 55%. z(상승) 성분 달성률: Left 87%,
Center 87%, Right 91% — **전진력이 상승력보다 훨씬 부족**해 로브성 리턴이 된다.

전체 eval 결과: `contact=30/30 cleared_net=30/30 returned_in=30/30 score합=30/90(1점×30)`.

---

## 부록 B. 참고 위치 색인

**계획 단계**
- 타점 예측: `src/estimator/ballistics.rs:63-141`
- 인터셉트 창: `src/defaults/planner.rs:27-36`, `src/planner/mod.rs:29-71`
- 목표 속도: `src/planner/impact.rs:15-101`
- IK/조작성 랭킹: `src/planner/swing/physics.rs:107-283`
- 궤적 생성: `src/planner/swing/physics.rs:358-418, 603-710`
- 위치+방향 5제약(미사용): `src/robot/mod.rs:726-836, 898-908`

**실행 단계**
- 커밋 상태기계: `src/sim/physics/world.rs:573-807`
- coarse 추종: `src/sim/physics/world.rs:134-167, 637-655`
- 죽은 rate-limit 함수: `src/robot/state.rs:335-364`
- Rapier 모터 구성: `src/sim/physics/arm_bodies.rs:126-287`
- 토크 클램프: `src/sim/physics/world.rs:1166-1191`
- 게인 유도: `src/defaults/sim_motor.rs`

**실물 경로**
- 스윙 실행 스레드: `src/hardware/real.rs:149-230`
- Dynamixel 변환/전송: `src/hardware/dynamixel.rs`
- AXL 레일: `src/hardware/rail/axl.rs:225-237`
- 하드웨어 기본값: `src/defaults/hardware.rs`

**진단 도구**
- `tests/diag_weak_return.rs` (6개 진단 테스트, `#[ignore]`)
- `tools/swing_bench/src/main.rs` (`--sim-verify` 모드)
- `src/sim/physics/world.rs:2759-3097` (`diag_random_shot_speed_reachability`,
  `diag_pre_vs_post_contact_commanded_travel`, `diag_swing_commit_rate_across_shot_grid`,
  `every_joint_reaches_commanded_pose_at_real_ball_contact`)
- `src/sim/eval_protocol.rs:136-150` (채점 규칙, 파울 강등)

**프로젝트 기록**
- `docs/measure-physics.md` — 미실측 물리 상수 목록과 측정 절차
- `.omc/plans/2026-07-29-swing-impact-joint-desync.md` — 07-29 계획 문서 (자기 정정 포함)
- `.omc/research/known-regressions-realistic-joint-speed.md` — 07-23 이후 회귀 이력
