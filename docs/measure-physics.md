# 물리 계수 측정 가이드

보드에서 잠글 값 · 측정 방법 · `e_eff` 정의.  
산출물은 stdout 스니펫 → `PhysicsParams::default()` / `ImpactParams::default()`에 붙여넣기.

관련 툴: [measure-restitution](../tools/measure_restitution/README.md) · [measure-friction](../tools/measure_friction/README.md).  
의도·이력: [decisions.md](decisions.md) A4 · E3.

---

## `e_eff`란?

플래너·Rapier 라켓 접촉에 넣는 **유효 반발계수**다.

법선 임팩트 모델 (`planner/impact.rs`):

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

### 급하지 않음

| 파라미터 | 필드 | 비고 |
|----------|------|------|
| 공 마찰 | `physics().ball_friction` | Rapier 재료. 제어 역산에 거의 안 탐 |
| 네트 반발 | `physics().net_restitution` | 시뮬용 |
| Magnus \(k_m\) | `physics().magnus` | 식 근사. 스핀 추정 넣을 때 재적합 |

### 시뮬 전용 — 실물 서보와 대조 필요

| 파라미터 | 필드 | 지금 값 | 비고 |
|----------|------|---------|------|
| 모터 위치 게인 \(k\) | `sim_motor().position_stiffness` | 5000 | **미실측.** MX-64 내부 위치 루프의 Rapier 모사 |
| 모터 감쇠 \(d\) | `sim_motor().position_damping` | 10 | **미실측.** 임계감쇠 \(2\sqrt{kI}\) 기준으로 잡은 값 |

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
cargo run -p measure-restitution -- --calibration calibration.json
cargo run -p measure-restitution -- --heights 0.40,0.29,0.21
cargo run -p measure-restitution -- --sim   # 시뮬 회귀용
```

stdout의 `restitution:` → `PhysicsParams::default()`.

### 2. 테이블 \(\mu\) — `measure-friction`

테이블 위 롤에서 접선 감쇠 \(v_t' \approx (1-\mu) v_t\).

```bash
cargo run -p measure-friction -- --calibration calibration.json
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
6. 관절 유효 관성 \(I\)(`control().joint_inertia` ≈ 0.015)로 환산한다.
   \(k = I\,\omega_n^2\), \(d = 2\zeta\sqrt{kI}\).
7. `SimMotorParams::default()`에 넣는다.

교차 검증: 같은 계단 입력을 시뮬에 주고 \(t_r\)·\(M_p\)가 실물과 맞는지 본다.
스윙 중 추종 오차는 `tests/diag_weak_return.rs`의 `diag_motor_tracking`으로
관절별로 볼 수 있다.

> 현재 값(k=5000, d=10)은 **실측이 아니다.** 이 팔의 관절 유효 관성
> (링크 0.04~0.08 kg → \(I\approx\)5e-3~1.5e-2)에서 \(2\sqrt{kI}\)가 10~17이라
> 임계감쇠 하단으로 잡았다. 이전 값 200은 \(\zeta\approx\)12~20의 과감쇠라
> 라켓이 명령 속도의 28%밖에 못 따라갔다. 실측 전까지 **시뮬 성공률을
> 하드웨어 준비도로 해석하면 안 된다.**

---

## 갱신 순서 제안

1. 테이블 \(e\), \(\mu\) (툴 있음)  
2. 라켓 \(e_{\mathrm{eff}}\) (스윙·장착 면)  
3. 라켓 \(\mu_r\), drag  
4. 모터 위치 루프 \(k, d\) — 실기로 스윙을 재현하려면 필수  
5. sim 회귀·실기 스윙을 보고 `e_eff` / 마찰만 미세 조정

측정 전에는 ITTF·문헌 근사로 시뮬을 돌리고, 보드 값이 나오면 **defaults만** 바꾼다 (상수 `ball`/`table`은 규격 SSOT).
