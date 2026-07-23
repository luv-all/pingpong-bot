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

## Sources

- https://emanual.robotis.com/docs/en/dxl/mx/mx-64-2/ (MX-64T/R/AT/AR, Protocol 2.0) — retrieved 2026-07-23
- https://emanual.robotis.com/docs/en/dxl/mx/mx-28-2/ (MX-28T/R/AT/AR, Protocol 2.0) — retrieved 2026-07-23
- `assets/robots/4-dof/urdf/all-4-export.urdf` (repo file, joint/link tree + masses)
- `assets/robots/4-dof/README.md` (repo file, URDF joint <-> Dynamixel ID mapping)
- `config/real-hardware.toml` (repo file, `motor_ids`, `joint_signs`)
- `src/robot/mod.rs` (`Arm::competition()`, joint construction order)
