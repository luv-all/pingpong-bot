# Dynamixel/Robotis Motor Specs + Motor-Joint Mapping

Research task for `feat/rough-to-fine-hitting-dynamics`. All numbers below are
sourced from Robotis's official e-Manual pages (Protocol 2.0 variants,
`*-2` URL suffix). Retrieved 2026-07-23. No numbers were taken from memory —
every figure has a source URL next to it.

## 1. Official spec sheet values (Protocol 2.0 pages)

Robotis publishes torque/speed at three bus-voltage points: 11.1 V, 12.0 V
(recommended), and 14.8 V. **Robotis's MX-series datasheets do not publish a
separate "rated"/continuous torque distinct from stall torque** — only stall
torque (at a given current draw) and no-load speed are listed for each
voltage. This is unlike Robotis's newer X-series (XM/XH), which do list a
continuous rated torque. Treat "stall torque" below as the number to derate
from, not as a continuous-duty rating.

### MX-64R (product code 902-0065-000; "R" = RS-485 variant, same electro-mechanical specs as MX-64T)

Source: https://emanual.robotis.com/docs/en/dxl/mx/mx-64-2/ (retrieved 2026-07-23)

| Bus voltage | Stall torque | Stall current | No-load speed |
|---|---|---|---|
| 11.1 V | 5.5 N·m | 3.9 A | 58 rev/min |
| 12.0 V | 6.0 N·m | 4.1 A | 63 rev/min |
| 14.8 V | 7.3 N·m | 5.2 A | 78 rev/min |

- Input voltage range: 10.0 – 14.8 V (recommended: 12.0 V)
- Weight: 126 g (R/T variant) — matches the URDF's `0.126` kg link masses (see §3)
- Gear ratio: 200:1

### MX-28T (product code 902-0067-000 per task description — see note below on R vs T)

Source: https://emanual.robotis.com/docs/en/dxl/mx/mx-28-2/ (retrieved 2026-07-23)

| Bus voltage | Stall torque | Stall current | No-load speed |
|---|---|---|---|
| 11.1 V | 2.3 N·m | 1.3 A | 50 rev/min |
| 12.0 V | 2.5 N·m | 1.4 A | 55 rev/min |
| 14.8 V | 3.1 N·m | 1.7 A | 67 rev/min |

- Input voltage range: 10.0 – 14.8 V (recommended: 12.0 V)
- Weight: matches the URDF's `0.072` kg link masses (see §3); Robotis lists
  MX-28T/R at ~72 g, consistent with the standard T/R (non-AT/AR) housing.

**Note on product codes**: the task description lists `902-0067-000` for
"MX-28T", but Robotis's official product-code table
(https://emanual.robotis.com/docs/en/dxl/mx/mx-28-2/, "Info" section) assigns
902-0066-000 to MX-28T and 902-0067-000 to MX-28R. The URDF link names in
this repo use `MX-28T_R_v1...` (see §3) — ambiguous naming from the CAD
export — but electro-mechanically T and R variants are identical (only the
serial-bus PHY differs: TTL vs RS-485), so the spec numbers above apply to
either.

**Operating voltage used by this rig**: not found anywhere in this repo
(`config/real-hardware.toml`, `src/`, or `assets/robots/4-dof/README.md`) —
no battery/PSU voltage is configured or documented. Recommend using the
**12.0 V column** above (Robotis's own "Recommended" operating point) as the
default for any derived constants, and flag this as an assumption to be
confirmed against the actual bench PSU/battery pack voltage. If the real rig
runs on an 11.1 V (3S LiPo) or 14.8 V (4S LiPo) supply, swap to the matching
column — do not silently assume 12 V without checking the physical supply.

## 2. Protocol 2.0 Profile Velocity / Profile Acceleration units

Source: https://emanual.robotis.com/docs/en/dxl/mx/mx-64-2/ and
https://emanual.robotis.com/docs/en/dxl/mx/mx-28-2/ (control table sections,
retrieved 2026-07-23) — identical on both models (standard Dynamixel
Protocol 2.0 control table layout):

- **Profile Velocity** (address 112), velocity-based profile mode: unit =
  **0.229 rev/min per LSB**, range 0–32767. (`0` = infinite velocity / max
  speed.) **Confirmed** — matches the commonly-cited 0.229 rev/min/LSB
  figure exactly.
- **Profile Acceleration** (address 108), velocity-based profile mode: unit =
  **214.577 rev/min² per LSB**, range 0–32767. (`0` = infinite
  acceleration.)
- Both registers also support a time-based profile mode (selected via
  Drive Mode bit 2) where the unit is instead 1 ms per LSB for both
  addresses — not used by `config/real-hardware.toml`'s
  `addr_profile_velocity`/`addr_profile_acceleration` unless Drive Mode is
  explicitly set to time-based (not present in the current config, so the
  rig is using velocity-based profile units above).

Conversion for Rust constants:
```
rev/min per LSB (velocity)      = 0.229
rad/s per LSB (velocity)        = 0.229 * 2*PI / 60 ≈ 0.023980
rev/min^2 per LSB (acceleration) = 214.577
rad/s^2 per LSB (acceleration)   = 214.577 * 2*PI / 60 ≈ 22.4747
```

## 3. Motor-to-joint mapping

### Evidence

**URDF kinematic chain** (`assets/robots/4-dof/urdf/all-4-export.urdf`),
tracing `<joint>` parent/child links from `base_link` to the end effector:

```
Rigid 4  (fixed):    base_link            -> MX-64R_v1__2__1   (mass 0.126 kg)
Revolute 6 (yaw):     MX-64R_v1__2__1      -> FR05-H101_v1__1__1   <- actuator link = MX-64R
Rigid 8  (fixed):    ... -> MX-64R_v1_1    (mass 0.126 kg)
Revolute 9 (shoulder): MX-64R_v1_1          -> FR05-H101_v1_1        <- actuator link = MX-64R
Rigid 12 (fixed):    ... -> MX-28T_R_v1__1__1 (mass 0.072 kg)
Revolute 13 (elbow):  MX-28T_R_v1__1__1     -> FR07-H101_v1_1        <- actuator link = MX-28T
Rigid 17 (fixed):    ... -> MX-28T_R_v1_1   (mass 0.072 kg)
Revolute 18 (wrist):  MX-28T_R_v1_1         -> FR07-H101_v1__1__1    <- actuator link = MX-28T
```

Reasoning: for a serial-chain revolute joint, the URDF's parent link of that
`<joint>` is the physical link that houses the actuator driving it (the
motor casing is rigidly mounted to the upstream structure and its output
shaft drives the downstream link). So the mass-tagged link immediately
preceding each `Revolute` joint identifies which motor drives that joint.
This gives 0.126 kg (MX-64R) driving yaw and shoulder, and 0.072 kg (MX-28T)
driving elbow and wrist — matching the task description's hint about the
`0.126 kg x3` / `0.072 kg` link masses (there is a 3rd 0.126 kg link,
`MX-64R_v1__1__1`, fixed directly to `base_link` via `Rigid 5` with no
downstream `Revolute` joint in this URDF — this is present in the CAD/URDF
export but does not drive any modeled joint; not counted in the mapping
below).

**Joint order confirmation** — `assets/robots/4-dof/README.md` ("실물
Dynamixel 매핑" section) explicitly documents the URDF-joint -> Dynamixel-ID
mapping as the source of truth:

| URDF joint | role (README) | Dynamixel ID | sign (`config/real-hardware.toml`) |
|---|---|---|---|
| Revolute 6  | yaw      | 1 | -1 |
| Revolute 9  | shoulder | 3 | +1 |
| Revolute 13 | elbow    | 4 | +1 |
| Revolute 18 | wrist    | 5 | +1 |

This matches `config/real-hardware.toml`'s `motor_ids = [1, 3, 4, 5]` and
`joint_signs = [-1, 1, 1, 1]` positionally (array index 0..3 = yaw, shoulder,
elbow, wrist — same order as `Arm::competition()`'s `joints` vec in
`src/robot/mod.rs:187-207`, which builds q0..q3 in that same order).

### Final joint-index -> motor-model table

| Joint index (`Arm` chain / `Joints` vector) | Role | Dynamixel ID | sign | Motor model | Link mass (URDF) |
|---|---|---|---|---|---|
| 0 | yaw      | 1 | -1 | **MX-64R** (902-0065-000) | 0.126 kg |
| 1 | shoulder | 3 | +1 | **MX-64R** (902-0065-000) | 0.126 kg |
| 2 | elbow    | 4 | +1 | **MX-28T** (902-0066-000 per Robotis' own table; repo URDF names it ambiguously as `MX-28T_R`) | 0.072 kg |
| 3 | wrist    | 5 | +1 | **MX-28T** (902-0066-000, same note) | 0.072 kg |

## 4. Rust constants reference (for downstream tasks, e.g. #2 and #4)

Suggested constants, each requiring a source comment when implemented:

```rust
// source: https://emanual.robotis.com/docs/en/dxl/mx/mx-64-2/, retrieved 2026-07-23
// Stall torque @ 12.0V (Robotis "Recommended" operating voltage; rig's actual
// bus voltage is not documented in this repo -- confirm before relying on this).
pub const MX64_STALL_TORQUE_NM: f64 = 6.0;
pub const MX64_NO_LOAD_SPEED_RPM: f64 = 63.0;

// source: https://emanual.robotis.com/docs/en/dxl/mx/mx-28-2/, retrieved 2026-07-23
pub const MX28_STALL_TORQUE_NM: f64 = 2.5;
pub const MX28_NO_LOAD_SPEED_RPM: f64 = 55.0;

// source: https://emanual.robotis.com/docs/en/dxl/mx/mx-64-2/ and mx-28-2/
// (Protocol 2.0 control table, addr 112 "Profile Velocity"), retrieved 2026-07-23
pub const PROFILE_VELOCITY_REV_MIN_PER_LSB: f64 = 0.229;
// (addr 108 "Profile Acceleration")
pub const PROFILE_ACCELERATION_REV_MIN2_PER_LSB: f64 = 214.577;
```

No-load speed is an upper bound on joint angular speed under zero external
torque; real max speed under load (e.g. during a swing) will be lower.
Whoever recalibrates `MAX_JOINT_SPEED` (currently `16.0 rad/s` in
`src/constants/arm.rs:10`, task #2) should derate from the no-load rpm above,
not use it directly as an achievable sustained speed.

## 5. Rotor / gearbox reflected inertia (added 2026-07-29 for WP8)

Needed by `src/robot/dynamics.rs`: the RNEA only carries rigid **link** inertia
from URDF/CAD, so the motor's own rotor + gearbox inertia — which the joint sees
amplified by the square of the reduction ratio — is missing from every torque
estimate. `I_reflected = I_rotor · N²`, and with N = 200 (MX-64) that is a
×40 000 amplification, so even a sub-gram·cm² rotor matters.

### 5.1 Gear ratios (datasheet, authoritative)

| Model | Gear ratio | Motor type | Source |
|---|---|---|---|
| MX-28 | **193 : 1** | Coreless (Maxon) | https://emanual.robotis.com/docs/en/dxl/mx/mx-28-2/ (retrieved 2026-07-29) |
| MX-64 | **200 : 1** | Coreless (Maxon) | https://emanual.robotis.com/docs/en/dxl/mx/mx-64-2/ (retrieved 2026-07-23) |
| MX-106 | **225 : 1** | Coreless (Maxon) | https://emanual.robotis.com/docs/en/dxl/mx/mx-106-2/ (retrieved 2026-07-29) — reference point only, not used on this rig |

### 5.2 MX-64 rotor inertia — third-party **identified**, not measured here

**Robotis does not publish rotor inertia** for any MX-series model. The
e-Manual specification tables list stall torque/current, no-load speed, gear
ratio, weight, and resolution — nothing about armature inertia. Confirmed by
re-reading both the MX-64 and MX-28 Protocol 2.0 pages on 2026-07-29.

The best available public number comes from Rhoban's open-source actuator
identification work:

- Paper: M. Duclusaud, G. Passault, V. Padois, O. Ly, *"Extended Friction
  Models for the Physics Simulation of Servo Actuators"*, arXiv:2410.08650
  (v4, 4 Nov 2025). §II-A defines the pendulum test-bench dynamics as
  `τ_m + τ_e(θ) + τ_f = J θ̈` with `J = m l² + J_m`, where `J_m` is the
  **"servo actuator apparent inertia (sometimes referred to as armature)"**
  and explicitly `J_m = N² J_r` when the constructor publishes `J_r`,
  otherwise `J_m` is identified. §VI-A states MX-64 is one of the four
  actuators identified. **`J_m` is therefore referenced to the output
  (joint) shaft** — exactly the quantity we need.
- Identified values: https://github.com/Rhoban/bam →
  `bam/params/mx64/{m1..m6}.json` (retrieved 2026-07-29)

| Model (friction) | `armature` `J_m` [kg·m²] |
|---|---|
| m1 (Coulomb-Viscous) | 0.011951 |
| m2 (Stribeck) | 0.011924 |
| m3 (load-dependent) | 0.011238 |
| m4 (Stribeck load-dep.) | 0.010961 |
| m5 (directional) | 0.011729 |
| m6 (quadratic) | 0.012266 |

Spread across friction models is ±6 %. **Adopted: `J_m` = 1.20e-2 kg·m²**
→ `J_r = J_m / 200² = 3.0e-7 kg·m²` (= 3.0 g·cm², a plausible figure for a
~24 mm coreless rotor).

Independent sanity check that these params are output-referenced: the same
files give `kt` ≈ 1.60–1.66 N·m/A, and the MX-64 e-Manual lists
"6.0 N·m at 12 V, 4.1 A" → 1.46 N·m/A at the output shaft. Same order,
consistent with §IV-B of the paper ("the torque constant `kt` is the product
of the motor torque constant and the reducer ratio").

> **Status: 제3자 식별값 (third-party identified), 실측 아님.** Not measured on
> this rig, and not a manufacturer figure. Uncertainty taken as ±10 %.

### 5.3 MX-28 rotor inertia — **추정치, 실측 필요 (estimate, needs measurement)**

No public identification exists for MX-28 (BAM covers MX-64, MX-106, XL-320,
XL-330, eRob80, Feetech STS3215 — not MX-28). Extrapolated from the two
identified MX-family points, both Coreless(Maxon), using rotor-side stall
torque `T_r = T_stall / N` as the size proxy:

| Model | `T_stall` @12 V | N | `T_r` [N·m] | `J_m` [kg·m²] | `J_r` [kg·m²] |
|---|---|---|---|---|---|
| MX-64 | 6.0 | 200 | 0.03000 | 1.195e-2 | 2.99e-7 |
| MX-106 | 8.4 | 225 | 0.03733 | 2.661e-2 | 5.26e-7 |
| MX-28 | 2.5 | 193 | 0.01295 | *(estimated)* | *(estimated)* |

Three extrapolations:

| Method | exponent | MX-28 `J_r` | MX-28 `J_m` = `J_r·193²` |
|---|---|---|---|
| Two-point MX-64↔MX-106 fit on `T_r` | 2.59 (fitted) | 3.40e-8 | 1.27e-3 |
| Two-point fit on servo weight (72 / 126 / 153 g) | 2.91 (fitted) | 5.85e-8 | 2.18e-3 |
| Geometric similarity `J ∝ T^(5/3)` | 5/3 (assumed) | 7.4e-8 | 2.76e-3 |

**Adopted: geometric mean, `J_r` = 5.4e-8 kg·m² → `J_m` ≈ 2.0e-3 kg·m².**
Plausible range 1.3e-3 – 2.8e-3 kg·m².

Validation of the scaling approach: applying the geometric-similarity law to
predict MX-106 from MX-64 gives `J_r` = 4.32e-7 vs the identified 5.26e-7 —
within 22 %. (The same law applied to XL-330 is off by 8×, but that is a
different technology — cored motor, plastic gears, 5 V — so it is not a
counter-example for the MX family.)

> **Status: 추정치 — 실측 필요.** Follow-up measurement is registered in
> `docs/measure-physics.md`. Direct method: pendulum test-bench per
> arXiv:2410.08650 §V, or simpler — step the joint with a known load inertia
> `m l²` and fit `J` from the acceleration response; `J_m` is the intercept as
> `m l² → 0`.

### 5.4 Values as used in code

`src/defaults/dxl_limits.rs`:

```rust
pub const MX64_ROTOR_INERTIA_KG_M2: f64 = 3.0e-7;  // identified (BAM), ±10%
pub const MX28_ROTOR_INERTIA_KG_M2: f64 = 5.4e-8;  // ESTIMATE, needs measurement
pub const fn reflected_inertia(rotor: f64, gear_ratio: f64) -> f64 { rotor * gear_ratio * gear_ratio }
```

Per-joint (`joint_reflected_inertias_4dof_array`, mapping from §3):

| Joint | Motor | `I_reflected` [kg·m²] | Rigid link `M_ii` | Increase |
|---|---|---|---|---|
| 0 yaw | MX-64 ×2 (dual, mechanically coupled → inertias add) | 2.40e-2 | 3.373e-2 | **+71 %** |
| 1 shoulder | MX-64 | 1.20e-2 | 1.617e-2 | **+74 %** |
| 2 elbow | MX-28 | 2.01e-3 | 1.429e-2 | +14 % |
| 3 wrist | MX-28 | 2.01e-3 | 2.196e-3 | **+92 %** |

(`M_ii` = `JOINT_EFFECTIVE_INERTIA_4DOF`, `src/defaults/sim_motor.rs`.)

Because elbow's link inertia dominates its reflected term, the MX-28 estimate's
±35 % uncertainty barely moves joint 2; only wrist is sensitive to it.

## Sources

- https://emanual.robotis.com/docs/en/dxl/mx/mx-64-2/ (MX-64T/R/AT/AR, Protocol 2.0) — retrieved 2026-07-23
- https://emanual.robotis.com/docs/en/dxl/mx/mx-28-2/ (MX-28T/R/AT/AR, Protocol 2.0) — retrieved 2026-07-23
- `assets/robots/4-dof/urdf/all-4-export.urdf` (repo file, joint/link tree + masses)
- `assets/robots/4-dof/README.md` (repo file, URDF joint <-> Dynamixel ID mapping)
- `config/real-hardware.toml` (repo file, `motor_ids`, `joint_signs`)
- `src/robot/mod.rs` (`Arm::competition()`, joint construction order)
- https://emanual.robotis.com/docs/en/dxl/mx/mx-106-2/ (MX-106, Protocol 2.0) — retrieved 2026-07-29
- https://arxiv.org/abs/2410.08650 — Duclusaud, Passault, Padois, Ly, "Extended Friction Models for the Physics Simulation of Servo Actuators" (v4, 2025-11-04); §II-A `J_m = N²J_r`, §VI-A MX-64 identification
- https://github.com/Rhoban/bam — `bam/params/mx64/*.json`, `bam/params/mx106/m1.json` (identified `armature`) — retrieved 2026-07-29
