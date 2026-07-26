# Torque Derate Analysis — Dual-Motor Yaw Fix + Minimum Safe Derate Fraction

Investigation date: 2026-07-23, on branch `feat/rough-to-fine-hitting-dynamics`.
Follow-up to `.omc/research/dynamixel-specs.md` (motor spec sourcing) and the
rough-to-fine dynamics work. Triggered by a hardware-owner report: the base
link carries **two** MX-64R motors sharing the yaw axis, which the software
model did not account for.

## 1. Dual-motor yaw axis — confirmed from the URDF

`assets/robots/4-dof/urdf/all-4-export.urdf`:

```
<joint name="Rigid 4" type="fixed">
  <origin xyz="-0.06625 0.017 0.0148"/>
  <parent link="base_link"/>
  <child link="MX-64R_v1__2__1"/>
</joint>
<joint name="Rigid 5" type="fixed">
  <origin xyz="0.06625 0.017 0.0148"/>
  <parent link="base_link"/>
  <child link="MX-64R_v1__1__1"/>
</joint>
<joint name="Revolute 6" type="continuous">   <!-- yaw -->
  <parent link="MX-64R_v1__2__1"/>
  <child link="FR05-H101_v1__1__1"/>
</joint>
```

Two MX-64R motor bodies (`MX-64R_v1__2__1`, `MX-64R_v1__1__1`) are both fixed
directly to `base_link`, mounted symmetrically at ±6.625 cm — a side-by-side
dual-motor layout. `Revolute 6` (yaw) uses only ONE of them
(`MX-64R_v1__2__1`) as its kinematic parent; the other (`MX-64R_v1__1__1`)
has no downstream `Revolute` joint in the URDF at all. This was already
flagged as an oddity in `.omc/research/dynamixel-specs.md` §3 ("a 3rd 0.126 kg
link... does not drive any modeled joint — not counted in the mapping") but
at the time was treated as inert CAD clutter, not a second actuator.

**Confirmed by the hardware owner (2026-07-23): both motors are mechanically
coupled to the same yaw shaft and contribute torque together.** The
kinematic model is still correct as a single revolute DOF (two motors geared
to one shaft don't add a degree of freedom), but the **torque budget** for
joint 0 (yaw) must reflect two motors, not one.

### Fix applied

- `src/robot/mod.rs` (`Arm::competition()`): `joint_torque_limits[0]` changed
  from `MX64_STALL_TORQUE_NM * CONTINUOUS_TORQUE_DERATE` to
  `2.0 * MX64_STALL_TORQUE_NM * CONTINUOUS_TORQUE_DERATE`.
- `src/robot/urdf/arm_from_urdf.rs`: same doubling applied to the
  URDF-loading path's `motor_derived_limit` for joint index 0.
- Only **joint 0 (yaw)** is affected — shoulder/elbow/wrist show no evidence
  of a second motor (single motor ID each in `config/real-hardware.toml`'s
  `motor_ids = [1, 3, 4, 5]`, and no extra unmatched link in the URDF near
  those joints).

### Important: this does NOT fix the "arm too slow for rally returns" problem

This session's investigation (see main plan/session notes) found the
recalibrated joint-speed cap (~2.88 rad/s, from real Dynamixel no-load speed)
is the binding constraint for typical rally-return racket speeds (~2 m/s),
via three independent lines of evidence (reach/speed-budget calculation,
IK-seed manipulability analysis, mount-position sweep). **That problem is a
velocity (kinematic) limit, entirely independent of torque.** The dual-motor
fix corrects a real *torque*-budget error but does not move that bottleneck —
confirmed empirically: re-running the 90-scenario `tools/mount-search`
feasibility sweep after the fix showed no change in `peak_joint_speed_ratio`
(the metric is purely kinematic, via `Arm::linear_velocities_for_racket_velocity`,
independent of `joint_torque_limits`). The two issues are orthogonal:

| Concern | Constant | Affects |
|---|---|---|
| Achievable racket speed for a swing | `DYNAMIXEL_MAX_JOINT_SPEED_RAD_S` (~2.88 rad/s) | `peak_joint_speed_ratio` (the actual bottleneck) |
| Whether the swing's required acceleration is safe for the motor | `joint_torque_limits` (this doc) | `trajectory_within_limits`'s torque check, `plan_swing`'s Newton-Euler gate |

## 2. Derate-fraction reference table

`joint_torque_limits[i] = stall_torque[i] * CONTINUOUS_TORQUE_DERATE`, where
stall values (12.0 V, Robotis "recommended" column — actual rig bus voltage
still undocumented, see `.omc/research/dynamixel-specs.md` §1) are:

| Joint | Motor(s) | Stall torque used |
|---|---|---|
| 0 (yaw) | 2x MX-64R (dual-motor, this fix) | 12.0 N·m |
| 1 (shoulder) | 1x MX-64R | 6.0 N·m |
| 2 (elbow) | 1x MX-28T | 2.5 N·m |
| 3 (wrist) | 1x MX-28T | 2.5 N·m |

Resulting `joint_torque_limits` (N·m) at each derate fraction:

| Fraction | yaw | shoulder | elbow | wrist |
|---|---|---|---|---|
| 0.10 | 1.20 | 0.60 | 0.25 | 0.25 |
| 0.15 | 1.80 | 0.90 | 0.38 | 0.38 |
| 0.20 (Robotis's own "continuous torque" figure — see `dynamixel-specs.md`... actually see note below) | 2.40 | 1.20 | 0.50 | 0.50 |
| 0.25 | 3.00 | 1.50 | 0.63 | 0.63 |
| 0.30 | 3.60 | 1.80 | 0.75 | 0.75 |
| 0.35 | 4.20 | 2.10 | 0.88 | 0.88 |
| **0.40** | **4.80** | **2.40** | **1.00** | **1.00** |
| 0.45 | 5.40 | 2.70 | 1.13 | 1.13 |
| **0.50 (current default, `CONTINUOUS_TORQUE_DERATE`)** | **6.00** | **3.00** | **1.25** | **1.25** |
| 0.60 | 7.20 | 3.60 | 1.50 | 1.50 |
| 0.80 | 9.60 | 4.80 | 2.00 | 2.00 |
| 1.00 (= stall, not sustainable) | 12.00 | 6.00 | 2.50 | 2.50 |

Note on 0.20: Robotis's official torque-ratings guidance (see
`dynamixel-specs.md` for URL) states **continuous torque ≈ 20% of stall
torque** for Dynamixel servos generally — this is the manufacturer's own
figure for indefinitely-sustained (continuous-duty) operation. Our current
default of 50% is **2.5x more permissive** than that continuous-duty figure.

## 3. Minimum feasible fraction — empirical measurement

Method: took a representative moderate swing state (default rest pose,
joint velocities `[1.0, 0.5, -1.5, 2.0] rad/s`, joint accelerations
`[200, 150, -300, 350] rad/s²` — all within the arm's speed/accel caps) and
computed the actual required per-joint torque via the recursive Newton-Euler
solver (`planner::dynamics::required_joint_torques`), independent of any
IK/reachability question. Then compared against `stall * fraction` per joint
across a fraction sweep.

**Required torque for this representative swing**: `[3.556, 2.174, -0.153,
-0.154] N·m` (yaw, shoulder, elbow, wrist).

| Fraction | yaw util | shoulder util | elbow util | wrist util | Feasible? |
|---|---|---|---|---|---|
| 0.10 | 2.96 | 3.62 | 0.61 | 0.62 | ❌ |
| 0.20 | 1.48 | 1.81 | 0.31 | 0.31 | ❌ |
| 0.30 | 0.99 | 1.21 | 0.20 | 0.21 | ❌ |
| 0.35 | 0.85 | **1.04** | 0.17 | 0.18 | ❌ (shoulder just over) |
| **0.40** | **0.74** | **0.91** | 0.15 | 0.15 | **✅ first feasible fraction** |
| 0.50 (current) | 0.59 | 0.72 | 0.12 | 0.12 | ✅ (comfortable margin) |
| 1.00 (stall) | 0.30 | 0.36 | 0.06 | 0.06 | ✅ |

**Minimum feasible fraction for this representative swing: 0.40 (40%).**
The binding joint is the **shoulder** (single MX-64R, no dual-motor benefit),
not yaw or the elbow/wrist (MX-28T) — elbow/wrist have large margin at every
fraction tested here because this particular swing's angular
acceleration/velocity combination happens to load the shoulder joint most
heavily (largest moved mass at largest lever arm). A different swing
(different direction/speed) could load a different joint hardest; this
number is representative, not a universal guarantee for every possible
swing — the per-sample Newton-Euler check in `trajectory_within_limits`
(already in the codebase) is what enforces this per-trajectory at runtime,
not a single derate number.

**Why the dual-motor yaw fix matters even though yaw isn't this scenario's
bottleneck**: at fraction 0.40, if yaw were still single-motor (stall 6.0
instead of 12.0), yaw's utilization would be `3.556 / (6.0*0.40) = 1.48` —
**infeasible on yaw too**. The fix changes whether this exact scenario is
even torque-feasible on yaw at the current 50% derate, independent of the
shoulder finding above.

## 4. Recommendation

- **Current default (`CONTINUOUS_TORQUE_DERATE = 0.5`) has real margin above
  the measured minimum (0.40) for this representative swing** — roughly 25%
  headroom (0.50 vs 0.40). Not risky by this evidence, but also not
  aggressively safe by Robotis's own continuous-duty figure (0.20).
- **This measurement is scenario-specific.** It does not prove 0.40 is safe
  for every possible swing this arm could attempt — only for this
  representative moderate one. Treat 0.40 as "the floor observed for one
  swing," not "the floor for all swings." The codebase's actual runtime gate
  (`trajectory_within_limits`, sampling the real planned trajectory) is the
  correct per-swing check; this document is about picking the *constant*
  that gate compares against, not replacing the gate.
- **What would make this rigorous rather than illustrative**: real bench
  testing of the actual duty cycle (how often a swing fires per minute,
  motor temperature over a match-length session) to justify anything above
  Robotis's conservative 20% continuous figure. That data does not exist in
  this repo or in Robotis's public specs (Robotis does not publish an
  intermittent/burst torque rating distinct from stall vs. continuous,
  confirmed in `dynamixel-specs.md` §1). Until that bench data exists, **0.5
  is a judgment call, not a verified-safe number** — it is defensible (2.5x
  above Robotis's continuous figure, with real per-swing margin above the
  measured 0.40 floor for a representative case) but not proven safe under
  sustained rally play.

## Sources

- `assets/robots/4-dof/urdf/all-4-export.urdf` (dual-motor yaw evidence)
- `.omc/research/dynamixel-specs.md` (stall torque values, Robotis continuous-torque
  guidance, motor-joint mapping)
- `src/planner/dynamics.rs` (`required_joint_torques`, recursive Newton-Euler)
- `src/robot/mod.rs`, `src/robot/urdf/arm_from_urdf.rs` (the applied fix)
- Hardware owner report (2026-07-23, this conversation): dual-motor yaw axis confirmation
