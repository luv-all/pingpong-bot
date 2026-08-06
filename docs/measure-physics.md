# 물리 계수 측정 가이드

보드에서 잠글 값 · 측정 방법 · `e_eff` 정의.  
산출물은 stdout 스니펫 → `PhysicsParams::default()` / `ImpactParams::default()`에 붙여넣기.

관련 툴: [measure-restitution](../tools/measure_restitution/README.md) · [measure-friction](../tools/measure_friction/README.md).  
의도·이력: [decisions.md](decisions.md) A4 · E3.

---

## `e_eff`란?

플래너·Rapier 라켓 접촉에 넣는 **유효 반발계수**다.

법선 임팩트 모델 (`src/robot/motion/impact.rs`):

\[
v_{\mathrm{out}}\cdot n = (1+e)\,v_r\cdot n - e\,v_{\mathrm{in}}\cdot n
\]

여기서 \(e\)가 `impact().racket_effective_restitution` (`e_eff`)다.

| | 러버 재질 COR | `e_eff` |
|--|---------------|---------|
| 의미 | 공–러버(시편) 법선 반발 | 위 식에 넣는 **한 숫자** |
| 포함 | 러버·스펀지 위주 | 러버 + 스펀지 + 블레이드 휨 + 접촉 면적 + 속도 의존 + 스침/불완전 접촉 |
| 상수? | 속도에 따라 변함 | 모델상 **상수 1개**로 퉁침 |

같은 물리량 계열이지만, 시편 COR와 숫자가 다를 수 있다.  
지금 코드는 플래너 역산과 Rapier 라켓 collider가 **같은 `e_eff`**를 쓴다 (`Min` combine → 공–라켓 접촉 e = `e_eff`).

### 왜 스윙(또는 장착 면)으로 재나?

시편 낙하 COR는 “면 재질”에 가깝고, 우리가 쓰는 식은 **움직이는 라켓면 + 실제 접촉**의 유효 e다.

- **권장:** 장착된 라켓(고정 또는 스윙)에 공을 맞혀 법선 속도비 \(e = |v_n'|/|v_n|\) (라켓 면 좌표계, 가능하면 \(v_r\) 보정)
- **비권장(참고만):** 러버 시편만 강판/테이블에 떨어뜨린 COR → `e_eff`와 정의가 다름

전용 CLI는 아직 없다. 멀티캠 궤적 + 임팩트 구간 법선 성분을 뽑아 `ImpactParams::default().racket_effective_restitution`에 넣는다.

---

## 재야 하는 것

### 우선 (보드에서 defaults 잠금)

| 파라미터 | defaults 필드 | 지금 값 | 비고 |
|----------|---------------|---------|------|
| 테이블 반발 \(e\) | `physics().restitution` | 0.88 | ITTF 테이블 근사. **우리 테이블**로 재측정 |
| 테이블 마찰 \(\mu\) | `physics().friction` | 0.4 | 튜닝값. 롤/바운스 접선 감쇠 |
| 라켓 \(e_{\mathrm{eff}}\) | `impact().racket_effective_restitution` | 0.55 | 튜닝값. **스윙·장착 면**으로 측정 |
| 라켓 마찰 \(\mu_r\) | `impact().racket_friction` | 0.5 | 튜닝값. 접선·스핀 변화 |
| 항력 \(k\) | `physics().drag` | 0.0 | 비행 로그 적합 (`--drag-csv`) |

> **주의 (WP6, 2026-07-30 추가):** 위 표의 `restitution=0.88`은 **설정값**일 뿐,
> Rapier가 실제로 그 값을 내는지는 솔버 반복 횟수(`num_solver_iterations`,
> `SimWorld::with_physics`)에 달려 있다. 기본값이었던 12에서는 실측 유효
> \(e\)가 평균 **0.789**로 낮고, 그 산포(0.10)의 지배 성분이 속도 의존 물리가
> 아니라 **서브틱 접촉 위상 아티팩트**였다(낙하고 0.1 mm 차이만으로도 \(e\)가
> 0.69~0.85로 요동 — `tests/diag_table_restitution.rs`의
> `diag_effective_restitution_subtick_phase`). 같은 원인이 라켓 접촉도
> 계획된 임팩트 시각보다 평균 −3.9 ms 일찍 발동시켜 접촉 타이밍 불일치(RC-3,
> 스윙 진단에서 `num_solver_iterations`를
> **32**로 올리면 두 증상이 함께 해소된다 — 유효 \(e\) 평균 0.878(산포
> 0.002), 접촉 타이밍 오차 평균 +0.02 ms(`tests/diag_contact_timing.rs`의
> `diag_contact_timing_solver_knob_sweep`). 물리 틱 비용도 오히려 개선됐다
> (`tests/diag_shoot_lag.rs`: 예산초과 틱 80회→1회, 접촉 해석이 안정화된
> 것으로 보임). 32는 2026-07-30 기준 `SimWorld::with_physics`의 기본값이다
> — 이 값을 다시 낮추면 위 두 실측을 재확인할 것.

### 급하지 않음

| 파라미터 | 필드 | 비고 |
|----------|------|------|
| 공 마찰 | `physics().ball_friction` | Rapier 재료. 제어 역산에 거의 안 탐 |
| 네트 반발 | `physics().net_restitution` | 시뮬용 |
| Magnus \(k_m\) | `physics().magnus` | 식 근사. 스핀 추정 넣을 때 재적합 |

### 시뮬 전용 — 실물 서보와 대조 필요

| 파라미터 | 필드 | 지금 값 | 비고 |
|----------|------|---------|------|
| 모터 위치 게인 \(k_i\) | `sim_motor().position_stiffness` | [134920, 64680, 57160, 8784] | **미실측.** MX-64 내부 위치 루프의 Rapier 모사. 관절별 \(k_i=\omega_n^2 I_i\) |
| 모터 감쇠 \(d_i\) | `sim_motor().position_damping` | [134.9, 64.7, 57.2, 8.78] | **미실측.** 관절별 임계감쇠 \(2\sqrt{k_i I_i}=2\omega_n I_i\) |

### 모터 회전자 반사관성 — 일부 추정 (WP8, 2026-07-29 추가)

| 파라미터 | 상수 | 지금 값 | 상태 |
|----------|------|---------|------|
| MX-64 회전자 관성 \(J_r\) | `MX64_ROTOR_INERTIA_KG_M2` | 3.0e-7 kg·m² | **제3자 식별값**(Rhoban BAM, ±10 %). 실측 아님 |
| MX-28 회전자 관성 \(J_r\) | `MX28_ROTOR_INERTIA_KG_M2` | 5.4e-8 kg·m² | **추정치 — 실측 필요.** 공개 식별 데이터 없음, 범위 3.4e-8~7.4e-8 |
| 연속 토크 derate | `CONTINUOUS_TORQUE_DERATE` | 0.5 | **미실측.** 아래 "연속 토크 derate" 절 |

출처·외삽 근거는 `.omc/research/dynamixel-specs.md` §5.
측정법은 아래 7번.

물리 계수가 아니라 **제어 루프 모델**이다. 실물에는 이 값이 나가지 않는다
(Goal Position + Goal Current만 나가고 위치 루프는 서보 펌웨어가 돈다).
그래서 이 값이 틀려도 기기가 상하지는 않지만, 시뮬의 스윙 속도·성공률이
실물과 어긋난다. 측정법은 아래 6번.

### 안 재도 됨 (규격·기하)

공 질량·반지름·중공 셸 관성 (`constants/ball`), 테이블 치수 (`constants/table`), `ANGULAR_DAMPING`(시뮬 안정용).

---

## 측정 방법

### 1. 테이블 \(e\) — `measure-restitution`

낙하 → 테이블 바운스에서 \(e = |v_z'| / |v_z|\) (또는 높이비 \(\sqrt{h_1/h_0}\)).

```bash
cargo run -p measure-restitution -- --calibration data/calibration.json
cargo run -p measure-restitution -- --heights 0.40,0.29,0.21
cargo run -p measure-restitution -- --sim   # 시뮬 회귀용
```

stdout의 `restitution:` → `PhysicsParams::default()`.

### 2. 테이블 \(\mu\) — `measure-friction`

테이블 위 롤에서 접선 감쇠 \(v_t' \approx (1-\mu) v_t\).

```bash
cargo run -p measure-friction -- --calibration data/calibration.json
cargo run -p measure-friction -- --sim
```

stdout의 `friction:` → `PhysicsParams::default()`.

### 3. 라켓 \(e_{\mathrm{eff}}\) — 스윙/장착 면 (수동·스크립트)

1. 라켓을 고정하거나, 알려진 \(v_r\)로 스윙한다.
2. 멀티캠으로 임팩트 직전·직후 공 속도 \(v_{\mathrm{in}}, v_{\mathrm{out}}\)를 구한다.
3. 면 법선 \(n\)으로 \(v_{in,n}, v_{out,n}, v_{r,n}\)을 투영한다.
4. 정지 라켓이면 \(e = |v_{out,n}| / |v_{in,n}|\).  
   움직이는 라켓이면  
   \(e = -(v_{out}-v_r)\cdot n \;/\; (v_{in}-v_r)\cdot n\)  
   (`verify_impact_model`과 동일).
5. 여러 타속·입사각의 중앙값 → `impact().racket_effective_restitution`.

가능하면 **실제 랠리 타속 구간**에서 잰다 (e는 속도 의존).

### 4. 라켓 \(\mu_r\)

임팩트 전후 접선 속도·스핀 변화로 추정.  
간단 1차: 접선 \(v_t' / v_t\) 감쇠를 Coulomb 근사로 \(\mu_r\)에 매핑해 `impact().racket_friction`에 넣는다.  
(정밀 모델은 러버 stick–slip이라 상수 \(\mu\)는 근사다.)

### 5. drag \(k\) — 비행 로그

```bash
cargo run -p measure-restitution -- --drag-csv traj.csv
```

\(a \approx -k |v| v\) 적합 → `physics().drag`.

### 6. 모터 위치 루프 \(k, d\) — 스텝 응답

`sim_motor()`는 MX-64의 내부 위치 루프를 Rapier 위치 모터로 흉내 낸 것이다.
Rapier는 매 스텝 이 토크를 내고 `motor_max_force`(= RNEA \(\tau\))로 클램프한다.

\[
\tau = k\,(q_{\mathrm{target}} - q) \;-\; d\,\dot q
\]

두 항이 **같은 토크 예산을 나눠 쓴다.** \(d\)가 크면 주어진 추종 오차로 낼 수
있는 관절 속도가 낮아진다 — 미는 힘을 제동이 상쇄하기 때문이다. 반대로 너무
작으면 목표를 지나쳐 진동한다.

측정 절차:

1. 팔을 정지시킨 뒤, 한 관절에 **계단형 Goal Position**(예: 10°)을 준다.
   Goal Current는 평소 스윙과 같은 대역으로 둔다.
2. `present_position`을 `stream_hz`(200 Hz)로 로깅한다.
3. 응답에서 두 값을 읽는다.
   - 상승시간 \(t_r\) (목표의 10 %→90 %)
   - 오버슈트 \(M_p\) (최대 초과량 / 계단 크기)
4. \(M_p\)에서 감쇠비를 얻는다. \(\zeta = \dfrac{-\ln M_p}{\sqrt{\pi^2 + \ln^2 M_p}}\)
   (오버슈트가 없으면 \(\zeta \ge 1\) — 상승시간으로 맞춘다.)
5. \(t_r\)에서 고유진동수 \(\omega_n \approx 1.8 / t_r\)를 얻는다.
6. **관절별** 유효 관성 \(I_i\)로 환산한다 — `robot::dynamics::mass_matrix`의
   대각 \(M_{ii}\)(하위 링크·라켓의 반사 관성 포함), 상수는
   `JOINT_EFFECTIVE_INERTIA_4DOF`. \(k_i = I_i\,\omega_n^2\),
   \(d_i = 2\zeta\sqrt{k_i I_i}\).
7. `SimMotorParams::default()`에 넣는다 (관절별 `[f64; 4]` 배열).

교차 검증: 같은 계단 입력을 시뮬에 주고 \(t_r\)·\(M_p\)가 실물과 맞는지 본다.
스윙 중 추종 오차는 `tests/diag_weak_return.rs`의 `diag_motor_tracking`으로
관절별로 볼 수 있다.

> 현재 값(\(\omega_n\)=2000, \(\zeta\)=1 → 위 표)은 **실측이 아니다.**
> 관절별 반사 관성만 실제 모델(`mass_matrix`)에서 가져왔고, \(\omega_n\)은
> Rapier 추종 오차 실측으로 고른 값이다.
>
> 이전 균일 게인 (k=5000, d=10)은 **링크 하나의 국소 질량**
> (0.04~0.08 kg → \(I\approx\)5e-3~1.5e-2)만 보고 잡은 값이라, base/shoulder가
> 하위 링크 전체를 함께 가속해야 한다는 사실(실제 \(I_0\)=3.4e-2)을 놓쳤다.
> 그래서 관절별 \(\zeta\)가 0.39(base)~1.51(wrist)로 4배 흩어졌다. 그보다
> 이전 값 200은 \(\zeta\approx\)12~20의 과감쇠라 라켓이 명령 속도의 28%밖에
> 못 따라갔다.
>
> 모터 토크가 `motor_max_force`로 클램프돼 스윙 대부분이 **포화 구간**이라,
> 임팩트 시점 추종 오차는 \(k\)·\(d\)의 절대 크기가 아니라 비
> \(d/k = 2\zeta/\omega_n\)에 \(\dot q\)를 곱한 값으로 붙는다 — 가장 빠른
> 관절이 가장 뒤처진다. `607790e`가 임팩트 속도 부담을 base로 옮긴 뒤
> base가 가장 빨라져서 그 desync가 눈에 보이게 됐다.
>
> 실측 전까지 **시뮬 성공률을 하드웨어 준비도로 해석하면 안 된다.**
>
> `control().joint_inertia`(0.015, `robot/state.rs`)는 아직 관절 공통 스칼라
> 근사로 남아 있다 — 같은 방식으로 관절별화할 후속 과제.

### 7. 회전자 반사관성 \(J_r\) — 진자 테스트벤치

지금 값은 MX-64가 제3자 식별값, MX-28이 계열 외삽 **추정치**다
(`.omc/research/dynamixel-specs.md` §5). 이 리그에서 직접 재려면:

1. 관절 하나만 남기고(다른 축 고정) 길이 \(l\), 질량 \(m\)인 봉+추를 단다.
2. 알려진 \((m, l)\) 조합 3개 이상으로 각각 계단/정현 입력을 주고
   `present_position`을 200 Hz로 로깅한다.
3. 각 조합에서 \(\tau_m + \tau_e(\theta) + \tau_f = J\ddot\theta\)를 적합해
   \(J\)를 얻는다 (\(\tau_e = -mgl\sin\theta\)).
4. \(J\) 대 \(ml^2\) 직선을 그으면 **절편이 \(J_m\)**(출력축 기준 반사관성)이다.
   회전자축 값은 \(J_r = J_m / N^2\) (N: 감속비, MX-64 200, MX-28 193).
5. `MX64_ROTOR_INERTIA_KG_M2` / `MX28_ROTOR_INERTIA_KG_M2`에 넣는다.

상세 절차: arXiv:2410.08650 §V (같은 논문의 MX-64 식별값을 지금 쓰고 있다).

---

## 연속 토크 derate (`CONTINUOUS_TORQUE_DERATE`) — WP8 재검토

**결론: 0.5 유지.** 근거는 아래.

### 재검토 계기

`CONTINUOUS_TORQUE_DERATE = 0.5`는 stall 토크를 "안전한 연속 토크"로 깎는
유일한 마진이었고, 주석에 "실측 확인 필요"로 남아 있었다. WP8이 회전자
반사관성(`I_rotor·N²`)을 실제 물리항으로 모델에 넣었으니, 그동안 이 마진이
대신 흡수하던 몫이 줄어 derate를 완화할 근거가 생겼는지 확인했다.

### 계측 — 반사관성 추가 전/후 `peak_torque_utilization`

`planner::swing::physics`의 `diag_reflected_inertia_torque_utilization`
(`cargo test --lib diag_reflected_inertia -- --ignored --nocapture`).
**같은 궤적**(반사관성 없이 계획한 것)을 두 모델로 평가해 모델 변경 효과만
분리했다. 반사관성 [kg·m²] = [0.024, 0.012, 0.0020, 0.0020],
토크한계 [N·m] = [6.0, 3.0, 1.25, 1.25].

primitive_4dof, 대표 임팩트(y=0.18, z=table+0.18):

| time-to-impact | peak util 전 | 후 | 배율 | 관절별 util 전 → 후 |
|---|---|---|---|---|
| 0.25 s | 0.983 | **1.048** | 1.07× | [0.244 0 0.983 0.273] → [0.517 0 1.048 0.297] |
| 0.28 s | 0.887 | 0.940 | 1.06× | [0.235 0 0.887 0.239] → [0.517 0 0.940 0.259] |
| 0.30 s | 0.826 | 0.873 | 1.06× | [0.222 0 0.827 0.219] → [0.487 0 0.873 0.236] |
| 0.35 s | 0.737 | 0.773 | 1.05× | [0.229 0 0.737 0.187] → [0.517 0 0.773 0.200] |
| 0.40 s | 0.660 | 0.688 | 1.04× | [0.214 0 0.660 0.161] → [0.487 0 0.688 0.171] |

urdf_4dof, 휴지 자세 FK 임팩트:

| time-to-impact | peak util 전 | 후 | 배율 | 재계획 peak q̇ 비 |
|---|---|---|---|---|
| 0.22 s | 0.981 | **1.026** | 1.05× | 0.908 |
| 0.25 s | 0.980 | **1.024** | 1.04× | 0.932 |
| 0.30 s | 0.909 | 0.947 | 1.04× | 1.000 |
| 0.35 s | 0.813 | 0.844 | 1.04× | 1.000 |

**읽는 법 — 전체 peak는 4~7%만 오르지만 관절별로는 전혀 다르다:**

- **joint 0(yaw)는 이용률이 거의 2배**가 된다(0.244 → 0.517). 듀얼 MX-64라
  반사관성이 2.4e-2 kg·m²로 링크 관성 3.37e-2의 71%나 되기 때문이다.
  "yaw에 토크 여유가 많다"는 기존 인식은 **틀렸다** — `607790e`가 임팩트
  속도 부담을 base 쪽으로 옮긴 설계(`τ_limit⁴` 가중 최소노름)의 여유
  계산도 이 값으로 다시 봐야 한다.
- 반면 **병목은 여전히 joint 2(elbow)** 다(util ≈ 0.98). elbow는 링크 관성
  1.43e-2이 반사관성 2.0e-3을 압도해 +14%밖에 안 오른다. 그래서 **전체
  peak**가 조금만 움직였다.
- 실질 비용: 짧은 커밋(0.22~0.25 s)에서 게이트가 끝속도를 깎아 달성 peak q̇가
  최대 **−9%**. 더 긴 커밋에서는 변화 없음.
- joint 1(shoulder)은 두 모델 모두 util 0.000으로 계측됨.

### derate를 바꾸지 않는 이유

1. **derate와 반사관성은 애초에 다른 물리량이다.** derate가 깎는 것은
   *stall* 토크 — 4.1 A(MX-64)를 끌어쓰는 순간 최대치이고, 이걸 연속으로
   낼 수 없는 이유는 **열·전류 한계**다. 부하측 관성 항을 모델에 추가했다고
   모터가 지속 가능한 전류가 달라지지는 않는다. 즉 WP8은 derate를 완화할
   근거를 **만들지 않는다**.
2. **마찰은 여전히 미모델이다.** 다만 크기는 작다 — 같은 BAM MX-64 식별값
   (`m1.json`)의 `friction_base = 0.090 N·m`, `friction_viscous = 0.0117
   N·m/(rad/s)` → q̇ = 3 rad/s에서 약 0.13 N·m, stall 6.0 N·m의 **2%**.
   그러니 "마찰 때문에 50%를 남긴다"는 설명도 성립하지 않는다.
3. 결국 0.5의 정당성은 **오직 열 여유**에 걸려 있고, 그 값은 이 리그에서
   측정된 적이 없다. 버스 전압조차 미확인이다(12.0 V 가정,
   `.omc/research/dynamixel-specs.md` §1). 근거 없이 숫자를 올리면 실기에서
   모터가 스톨/과열될 위험을 정량화 못 한 채 지는 셈이다.
4. 스윙은 0.2~0.4초 버스트라 "연속" 정격을 그대로 적용하는 건 보수적일 수
   **있다** — 하지만 이건 duty-cycle 열 모델이 필요한 얘기이지, 계수 하나를
   눈대중으로 올릴 일이 아니다.

### 올리려면 무엇을 재야 하나

- 실기 버스 전압(배터리/PSU) 확인 — 11.1 / 12.0 / 14.8 V 컬럼이 다르다.
- 랠리 지속 구간(예: 5분 연속 스윙)에서 `present_current`·`present_temperature`
  로깅 → 온도가 셧다운 임계 아래로 안정되는 최대 평균 전류를 찾는다.
  그 전류 × \(k_t\) 가 **실측 연속 토크**이고, stall 대비 비가 derate다.
- 앵커가 필요하면 Robotis X 시리즈(정격·stall을 **둘 다** 공개하는 계열)의
  rated/stall 비를 참고 상한으로 쓸 수 있다 — MX 계열은 정격을 공개하지 않는다.

### WP8 이후에도 남은 미모델 항목

| 항목 | 영향 | 비고 |
|---|---|---|
| 관절 마찰 (Coulomb + viscous) | 필요 토크 과소평가 ~2% | BAM 식별값 있음, 넣으려면 RNEA 래퍼에 q̇ 항 추가 |
| 기어박스 효율 / load-dependent 마찰 | 부하 클수록 커짐 | BAM m3~m6 모델 참고 |
| 열 디레이팅 | derate 0.5의 근거 자체 | 위 참고 |
| **Rapier 관절 armature** | 시뮬 팔이 실물보다 가볍다 | `sim/gui/debug/snap.rs::set_torque_now` 주석 참고. 지금은 시뮬 모터 토크 예산에서만 반사관성을 일부러 뺐다 — Rapier 다물체에 armature를 넣으면 양쪽을 함께 켜야 한다 |
| `planner::bang_bang` | `mass_matrix`(강체 전용)로 토크→가속도 역산 | `tools/swing_bench` 전용 경로. 반사관성 미반영 — 별도 과제 |

---

## 갱신 순서 제안

1. 테이블 \(e\), \(\mu\) (툴 있음)  
2. 라켓 \(e_{\mathrm{eff}}\) (스윙·장착 면)  
3. 라켓 \(\mu_r\), drag  
4. 모터 위치 루프 \(k, d\) — 실기로 스윙을 재현하려면 필수  
5. sim 회귀·실기 스윙을 보고 `e_eff` / 마찰만 미세 조정

측정 전에는 ITTF·문헌 근사로 시뮬을 돌리고, 보드 값이 나오면 **defaults만** 바꾼다 (상수 `ball`/`table`은 규격 SSOT).
