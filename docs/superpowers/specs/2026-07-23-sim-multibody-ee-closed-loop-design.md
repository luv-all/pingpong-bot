# Design: Sim multibody EE = FK + closed-loop RobotState (A→B)

**Date:** 2026-07-23  
**Status:** approved (one-shot A→B)  
**Goal:** 시뮬에서 다물체 EE가 `Arm` FK와 맞고, 공이 EE에 맞으며, 관절 상태가 물리 다물체를 따른다.

## Problem

1. **기하:** `ArmMultibody::spawn`이 `SerialChain`의 `mount_rotation` · origin 회전 · 누적 축 · `ee_transform` · 면축(+Y→+Z) 리맵을 무시 → EE ≠ FK 라켓.
2. **역할:** 공은 FK 키네마틱 라켓에만 충돌. `RobotState`는 궤적을 오픈루프로 재생하고, τ 한계는 Rapier 모터에만 걸려 타격 관성이 가짜다.

## Scope (one spec)

| Phase | Deliverable |
|-------|-------------|
| **A** | spawn을 `SerialChain` 1:1로 정합. 키네마틱 라켓 제거. 공↔EE 충돌. 명령은 플래너→모터(`τ_max`). |
| **B** | 매 스텝 다물체 관절각·속도를 `RobotState`에 반영. FK/뷰어/플래너 입력이 그 상태를 봄. |

비범위: 실기 Dynamixel 폐루프, E-stop UI, planner 알고리즘 변경.

## Architecture

```
plan_swing / jog  →  SwingTrajectory
                         │
                         ▼
              RobotState 목표 (모터 set_motor)
                         │
                         ▼
              Rapier ArmMultibody (τ_max)
                         │
                    관절 q, q̇ 읽기  ──► RobotState (B: 진실)
                         │
                         ▼
              EE collider ↔ ball
              FK(q) ≈ EE pose (< 2 mm)
```

### A — Geometry + EE collision

- `ArmMultibody::spawn`이 `SerialChain::forward_with_joint_frames`와 동일한 프레임 규칙으로 링크·축·EE를 배치.
- EE collider local: FK `racket_pose_from_isometry`와 동일한 면축 계약 (CAD +Y 법선 ↔ Rapier +Z).
- `attach_racket_collider = true` on EE; remove `racket_handle` kinematic body + `sync_racket_kinematic`.
- Contact tests use EE handle.

### B — Closed loop

- After physics step (or after motors, before plan): read revolute angles/velocities from `MultibodyJointSet` into `RobotState`.
- Trajectory advance becomes **command source only** (desired motor targets), not forced pose overwrite.
- Prefer: set motor targets from trajectory sample; do **not** teleport joint positions each step.
- Drop or gate `advance_swing` open-loop angle write when multibody is active; use `advance_swing` only to update “command clock” / desired pose for motors.

## Success criteria

1. EE ↔ FK position error **&lt; 2 mm** at default pose and mid-swing samples (unit test).
2. competition / 4-dof ground-truth rally: contact + return still pass.
3. Dual yaw `τ_max` (12 vs 6) still shows better tracking/hit than single in existing tests; retune stall budgets only if needed for realism.

## Tests

- New: `ee_matches_fk_within_2mm` (default + sampled swing).
- Keep: rally contacts, dual_yaw torque tests; update handles to EE.
- Random Shoot: smoke if present / flaky timing noted separately.

## Files

- `src/sim/arm_bodies.rs` — spawn geometry, EE attach, joint read API
- `src/sim/world.rs` — remove kinematic racket; step order A→B
- `src/robot/state.rs` — command vs state split for sim
- `docs/decisions.md` — short D-note if policy changes

## Out of policy

- Do not keep dual racket (kinematic + EE) “just in case”.
- Do not enable torque-limited kinematic advance as the long-term ball contact path (option C).
