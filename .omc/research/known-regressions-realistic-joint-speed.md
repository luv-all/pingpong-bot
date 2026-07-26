# Realistic Joint-Speed Recalibration — 조사·수정 기록

**최종 갱신: 2026-07-23 (3차 — 휴지 자세 재설계 + rough 관절 선추종으로 스윙 커밋 복구)**

| 차수 | 상태 |
|---|---|
| 1차 | 16개 테스트 `#[ignore]`. 원인을 "리치 부족 + `NearSingularity` 게이트"로 진단 |
| 2차 | **그 진단이 틀렸음을 실측 확인.** 진짜 원인은 관절공간 **이동시간** (§2) |
| 3차 | 원인 수정 → **테스트 126 passed / 0 failed / 2 ignored** (1차 시작점 111/0/16) |

---

## 1. 3차에서 무엇을 고쳤나

### (a) 휴지(ready) 자세 재설계 — `constants::arm::READY_JOINTS_4DOF`

이전 휴지 자세는 "관절 한계 중점"(URDF) 또는 `[0,0,-0.262,0]`(primitive)로,
**중립적으로 보이지만 스윙 시작점으로는 나빴다**. 임팩트 자세까지 관절공간
이동거리 Δq가 0.71~2.00 rad여서 quintic이 commit 창에 절대 못 들어왔다.

`tools/shot_tune --rest-pose-search`로 산출:
테이블 폭 전역(x 10~90%) × 접수 창(y 0.20~0.55) × 실현가능 높이 대역
(면 위 10~30cm) × 대표 입사속도 3종 = 240 시나리오 중 IK 해가 있는 165개의
임팩트 자세를 모아, **관절마다 그 각도 구간의 중점**(1D Chebyshev 중심)을 취했다.

비용 `max_시나리오 max_관절 |Δq|`는 두 max가 교환 가능해 관절별로 분리되므로
이 중점이 **정확한** minimax 최적해다(근사 아님). 평균이 아니라 최악값 기준인
이유: quintic 소요시간은 가장 많이 움직이는 한 관절이 지배한다.

```
READY_JOINTS_4DOF = [0.1207, 0.0, 0.1719, -0.6756]
최악 Δq: 2.00 rad → 1.183 rad   |   필요시간: 1.30s → 0.770s
```

### (b) rough 단계 관절 선추종 — `src/sim/world.rs::try_auto_swing`

이전에는 rough 단계에서 **레일만** 옮기고 팔 관절은 일부러 두었다("미리 펴
두면 windup이 줄어 리턴이 약해진다"). 그 대가로 Δq 전부가 commit 창으로
떠넘겨져 **스윙이 아예 시작되지 않았다**. 이제 `plan_coarse_track`이 이미
계산해 두고 버리던 관절 목표를 `RobotState::set_targets`로 함께 넘긴다.
실제 이동은 기존 rate-limited·충돌 안전 추종 루프가 하므로 안전 특성은 그대로.

(a)와 (b)는 **반드시 함께**여야 한다 — (b) 단독으로는 네트 통과 전 여유가
~0.19s뿐이라 2.88 rad/s로 Δq를 못 덮고, (a) 단독으로도 0.770s > 0.175s다.

### (c) 마운트 위치 — `constants::arm::{BASE_Y, MOUNT_HEIGHT_OFFSET_M}`

커밋이 가능해진 뒤 다시 스윕하니 확실한 신호가 나왔다(시드 8종 × 12발 = 96발):

| 마운트 | 성공률 |
|---|---|
| `base_y=+0.02, height=0.0` (이전) | 최고 **75/96**, 슈터 한 칸 차이로 **0/96**까지 무너지는 칼날 능선 |
| `base_y=-0.02, height=0.05` | 여러 슈터 설정에서 **96/96**, 넓은 고원 |

고원(`base_y` -0.10~-0.02)에서 실기 배치를 가장 덜 바꾸는 끝을 골랐다.
height 0.05는 "실기 브래킷이 면보다 ~3cm 위"라는 실측 보고와도 겹친다 —
이전의 `0.0`이 오히려 실기와 다른 단순화였다.

primitive `Arm::competition()`과 URDF 카탈로그 로봇이 **같은 상수**를 공유하도록
묶었다(같은 실물 로봇의 두 모델인데 마운트가 어긋나면 안 된다).

⚠️ 이 두 상수는 **실기 하드웨어 배치**를 뜻한다. 실기 적용 여부는 사람 판단.

### (d) 슈터 기본값 — `src/sim/shooter.rs`

`shot_tune`으로 "성공률 8/8이 연속 유지되는 속도 대역"이 가장 넓은 조합을 골랐다.

```
default: speed 5.0 → 7.1 m/s,  pitch -2.0 → -4.0°,  height_offset 0.19 → 0.17 m
RANDOM_SHOT_SPEED: [7.0, 10.0] → [6.8, 7.4] m/s
```

속도 대역이 크게 좁아진 것이 핵심 발견이다 — 고정 pitch/height에서 실제로
받아낼 수 있는 입사속도 폭은 ~0.6 m/s뿐이다. 이전 [7.0, 10.0]은
`swing_feasibility`(순간 조작성)로만 고른 값이라 실제 커밋 여부는 검증된 적이
없었다.

---

## 2. (2차) 진짜 근본 원인과, 1차 진단이 빗나간 이유

로봇은 "공을 못 친다"가 아니라 **"스윙을 시작조차 못 한다"**였다.
실패 지점은 IK도 `NearSingularity` 게이트도 아니고 `build_feasible_trajectory`의
quintic 검사였다.

| 항목 | 2차 실측 |
|---|---|
| Δq (최대 관절) | 0.71 ~ 2.00 rad |
| 필요 시간 (quintic 피크계수 1.875, 2.88 rad/s) | 0.46 ~ 1.30 s |
| commit 시점 남은 시간 | 0.125 ~ 0.175 s |
| **부족 배수** | **3 ~ 8배** |

`fit_end_velocity`는 **끝속도만** 스케일다운하므로 이 문제를 못 고친다 —
줄여야 하는 건 **이동거리**다.

**1차가 빗나간 이유**: `build_feasible_trajectory`의 실패 경로가 전부
`InverseKinematicsNoSolution`("목표가 도달 범위 밖")로 보고되고 있었다.
IK 해가 멀쩡하고 필요 관절속도가 한계의 60%인 상황에서도 "도달 범위 밖"이
떠서, 조사가 리치/속도 재보정 쪽으로 잘못 유도됐다.

**수정**: 정직한 variant로 분리 — `TrajectoryExceedsLimits { violated }`
(위반한 한계 이름: 관절 속도/각가속도/각도 범위, 레일 속도/범위),
`TrajectoryExceedsTorque { utilization }`. 이 수정 덕에 병목이 `[관절 속도]`임이
한 줄로 드러났다.

### 같이 발견해 고친 잠재 버그: URDF 마운트가 기구학에 반영되지 않았음

`arm_from_urdf::to_arm`이 레일을 primitive 템플릿에서 통째로 복사해,
`SimRobotMount`의 y·z가 **뷰어 배치에만** 쓰이고 FK/IK에는 전혀 반영되지
않았다. 실측: `base_y`를 **-1.0 ↔ +1.0 m**로 바꿔도 출력이 소수점까지 동일.
마운트 튜닝이 원리적으로 불가능한 상태였다.
회귀 테스트: `mount_position_reaches_arm_kinematics_not_just_the_viewer`.

---

## 3. 현재 상태 — 무엇이 되고 무엇이 안 되나

### 되는 것 (실측, `tools/shot_tune`, 4-dof 카탈로그 로봇)

| 지표 | 1·2차 | 3차 |
|---|---|---|
| 스윙 커밋 | **0** / 5,152 랠리 | **48/48** |
| 라켓 접촉 | (수동 접촉만) | 48/48 |
| 네트 통과 리턴 | — | 48/48 |
| 레일이 테이블 중앙에서 시작(반복 랠리 재현) | — | 48/48 |

**"로봇이 얼어붙는다"는 사용자 보고 증상은 해결됐다.**

### 안 되는 것 — 리턴 배치(placement)

리턴이 **너무 길어 상대 코트에 떨어지지 않는다**. 실측: 네트를 z=1.381
(면 위 62cm)로 넘어 최대 y=2.889까지 날아간다(테이블 끝 2.74 초과 = 아웃).

엄격 기준(리턴이 상대 코트에 실제 낙하)으로는 **48발 중 3발**만 성공.

`RACKET_EFFECTIVE_RESTITUTION`을 0.42~0.82로 스윕해봤지만 최대 22%(e=0.58)에
그쳐 **지배적 원인이 아니다** — 그래서 문서화된 캘리브레이션 값 0.42를 유지했다
(나쁜 값을 다른 나쁜 값으로 바꾸는 churn 회피).

⚠️ **측정 기준 주의**: `cleared_net`만 보면 이 결함이 안 보인다(48/48 통과).
`contact`/`returned`도 마찬가지로 **가만히 있는 라켓에 맞고 튕기는 것**까지
켜진다(2차에서 커밋 0인데 `cleared_net` 9/12였던 사례). `shot_tune`의
`success`는 이제 `커밋 ∧ 적법한 입사 ∧ 리턴이 상대 코트 낙하`를 모두 요구한다.

---

## 4. `#[ignore]` 남은 테스트 2개 (16 → 2)

1. `planner::bang_bang::plan_bang_bang_swing_converges_for_a_reachable_impact`
   — 순수 토크 bang-bang 경로가 실기 한계상 수렴 불가. `swing_bench` 실측:
   관절 4개 전부 **토크 100%·속도 100% 포화**로 2s cutoff까지 못 감(위치오차
   0.039, 라켓속도 목표의 3.5%, 방향오차 102°). commit 창 상한 0.35s라 애초에
   들어올 수 없다. **quintic 게임플레이 경로와 무관한 GUI 디버그 전용 경로**라
   사용자 영향 없음.
2. `sim::world::ground_truth_rally_contacts_racket_clears_net_and_bounces_near_center`
   — §3의 리턴 배치 결함을 정확히 가리키는 **유효한 테스트**. 껍데기가 아니라
   실제 미해결 결함이라 단언을 약화시키지 않고 그대로 둔다.

### 갱신한 픽스처 (문서가 예고했던 대로)

- `physics.rs::sample_prediction`, `bang_bang.rs::sample_prediction`:
  "휴지 자세의 FK 위치"를 임팩트로 쓰던 것 → 실제 접수 평면·실현가능 높이
  대역 안의 점. 휴지 자세를 옮긴 뒤 그 점이 오히려 특이점 근처가 됐다.
- `world.rs::auto_swing_plans_with_strike_velocity`: 임팩트 z 1.05 → 0.932,
  v_in (0,-4.22,0.37) → (0,-6.01,1.51) — **재튜닝된 슈터가 실제로 만드는 값**
  (`shot_tune --explain` 실측). 이전 값은 옛 슈터 기준이라 이제 안 나온다.
- `dynamics.rs::two_link_static_sanity_gravity_only`: yaw=0에서 평가하도록.
  새 휴지 자세는 yaw≈0.12라 shoulder 축이 정확히 수직이 아니어서 중력
  모멘트가 9.1e-6 N·m 나온다(stall의 3e-6 수준, 물리적으로 무시 가능하지만
  이 테스트의 명제 자체가 "수직 축"이라 축을 수직으로 두고 봐야 의미가 있다).
- 마운트 테스트 2개: "베이스가 면에 딱 붙어 있다" → "면 + `MOUNT_HEIGHT_OFFSET_M`".

---

## 5. 유지되는 결론 (되돌리지 않음)

- `max_joint_speed` ~2.88 rad/s — 실기 Dynamixel 스펙 기반. 되돌리는 건 문제 은폐.
- `NearSingularity` 게이트 — 2차에서 **병목이 아님**이 밝혀졌지만(비율 0.6에서도
  커밋 실패), 저속 스윙으로 조용히 "성공"하는 걸 막는 역할은 유효.
- 듀얼모터 yaw 토크 수정, 다중 IK 시드 조작성 선택,
  `Arm::linear_velocities_for_racket_velocity`.

## 6. 다음 작업 후보

1. **리턴 배치 수정** (§3) — 유일하게 남은 사용자 영향 결함.
   `required_racket_velocity` 모델 vs 실제 Rapier 접촉의 운동량 전달 차이를
   재조사할 것. restitution 단일 상수로는 안 된다는 건 확인됨.
2. `fourdof_ground_truth_rally_contacts_racket_and_returns`는 여전히
   `contact`/`returned`만 봐서 **능동 스윙을 검증하지 못한다**(수동 접촉으로도
   통과). `swing_committed()` 단언 추가 권장.

## Sources

- `.omc/research/torque-derate-analysis.md`, `.omc/research/dynamixel-specs.md`
- 본 문서의 모든 수치는 `tools/shot_tune` / `tools/swing_bench` 실행 결과 (2026-07-23).
