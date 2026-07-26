# 매니퓰레이터 역동역학 (RNEA)

작성: 2026-07-26.

## 목적

스윙 궤적의 \(\ddot q\)가 모터 \(\tau_{\max}\) 안에 있는지 **해석적으로** 검증하고,
sim HUD·(옵션) Dynamixel Goal Current FF에 같은 \(\tau(t)\)를 쓴다.

\[
\boldsymbol\tau = M(\mathbf q)\,\ddot{\mathbf q}
  + C(\mathbf q,\dot{\mathbf q})\,\dot{\mathbf q}
  + \mathbf g(\mathbf q)
\]

구현은 CRBA가 아니라 **RNEA** (Newton–Euler 역동역학).

## 경계

| 포함 | 제외 |
|------|------|
| URDF `<inertial>` → 체인 정렬 관성 | Rapier `ArmMultibody` mass와 동기화 |
| 관절 4축 \(\tau\) | 리니어 레일 축력 |
| `plan_swing` feasibility / scale | 기본 real FF on |
| sim \(\tau\) HUD | Current-only 토크 모드 |

Rapier 다물체와 플래너 RNEA는 **의도적으로 다른 SSOT** (접촉 시뮬 vs 계획 검증).

## 데이터

- SSOT: `assets/robots/4-dof/urdf/all-4-export.urdf` 링크 `<inertial>` (mass, COM origin, inertia).
- revolute 사이 **fixed 링크 관성은 child revolute 링크로 합산**.
- `Arm.inertias: Option<Vec<LinkInertia>>` — URDF 로봇만. 없으면 플래너는 레거시 대각 \(I\alpha\) 폴백하지 않고 **토크 게이트 스킵 금지** — primitive는 근사 관성 테이블을 심거나 URDF 경로만 쓴다. 기본 `defaults::robot()` = `urdf_4dof()`이므로 URDF 필수.

## API

```rust
// robot/dynamics.rs
fn required_torque(arm: &Arm, q: &[f64], qd: &[f64], qdd: &[f64]) -> Option<Vec<f64>>;
fn is_feasible(tau: &[f64], limits: &[f64]) -> bool;
fn peak_torques_on_trajectory(arm: &Arm, traj: &SwingTrajectory) -> Option<Vec<f64>>;
```

중력: Z-up, \(g=(0,0,-G)\).

## 연동

1. **Planner** — 속도·가속·관절/레일 + RNEA peak \(\lvert\tau_i\rvert \le \tau_{\max,i}\) **하드** 게이트. 초과 시 먼저 `peak_torque_scale`, 그래도 안 되면 `JointOrTorqueLimit` → sim은 이번 공 스윙 포기.
2. **sim HUD** — commit peak + 재생 중 \(\tau(t)\) (초과·포기 표시).
3. **Hardware FF** — `control().torque_feedforward` (default **true**). on이면 Current-based Position + Goal Current \(\approx \tau / k_t\) (real), sim은 RNEA로 다물체 `motor_max_force`를 맞춤.

## 테스트

- 정지 \(q_d=q_{dd}=0\) → \(\tau \approx g(q)\).
- FD로 \(M\) 한 열과 RNEA \(\tau(q,0,e_i) - \tau(q,0,0)\) 교차.
- dry-run Goal Current 페이로드 (FF on).
