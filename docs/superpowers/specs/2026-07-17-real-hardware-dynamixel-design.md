# Design: RealHardware — Dynamixel 4-DOF (AXL stub)

**Date:** 2026-07-17  
**Status:** approved (user: Dynamixel only; AXL stub)  
**Reference:** `~/Downloads/test-manipulator` (`dynamixel.py`, `config.py`)  
**Out of scope (this slice):** AXL rail driver, UVC cameras, ball detector, full `run_real` vision pipeline

---

## Goal

Replace `RealHardware` `NotImplemented` stubs with a **working Dynamixel 4-axis arm** path that matches the Python manipulator already running on the Windows bench.

**Done when:**

- `cargo run -p pingpong-bot --features real` on Windows opens COM port, enables torque, reads present position.
- `jog_axis` can move one joint or all joints to a target angle (Python `goto` parity).
- `Hardware::command` accepts `SwingTrajectory`, streams goal positions until duration elapses, `is_busy()` gates re-planning (same contract as `SimHardware`).
- `rail_x` is always `0.0`; non-zero `RailMotion` logs a warning and arm joints still execute.
- AXL module exists as explicit stub (`NotImplemented` / no-op) for future work.
- macOS builds without `real` feature unchanged; with `real` on non-Windows, compile-only or dry-run path optional.

---

## Context

| Layer | Python (`test-manipulator`) | Rust (`pingpong-bot`) |
|-------|----------------------------|------------------------|
| Bus | Dynamixel SDK SyncWrite/SyncRead | `rustypot` (plan §3.2) |
| Joints | 4 revolute → IDs `1,3,4,5` | `competition` URDF 4-DOF chain |
| Command | single `set_joint_positions` | `SwingTrajectory` quintic (impact + follow-through) |
| Rail | none | planner may emit `RailMotion` — **ignored** until AXL |
| IK / viz | PyBullet | domain `Arm` + sim viewer (unchanged) |

Python `DynamixelConfig` defaults are the **SSOT for motor mapping** until measured on bench:

- `port = COM8`, `baudrate = 57600`, Protocol 2.0
- `motor_ids = [1, 3, 4, 5]`
- `joint_signs = [-1, 1, 1, 1]`
- `zero_tick = 2048`, `ticks_per_revolution = 4096`
- Goal / torque / present / profile registers per Protocol 2.0 X-series
- `motor_angle_limits_deg` per ID (tick-clamped before write)
- `profile_velocity = 80`, `profile_acceleration = 20`
- comm retries: 5 × 20 ms

---

## Architecture

```
Control @ 100 Hz (pipeline)
    │  plan_best_swing → SwingTrajectory
    ▼
RealHardware::command (non-blocking)
    │  spawn / signal SwingExecutor
    ▼
SwingExecutor thread (≈100–200 Hz)
    │  trajectory.sample_at(t) → Joints
    │  MotorMapping::radians_to_ticks
    ▼
rustypot MX sync_write_goal_position([1,3,4,5], ticks)

RealHardware::read_pose
    │  sync_read_present_position → ticks_to_radians
    ▼
RobotPose { rail_x: 0.0, joints }

RailStub — read 0.0, ignore command (warn if Δrail significant)
```

`Hardware` contract (same as sim):

| Method | Behavior |
|--------|----------|
| `command` | Start trajectory playback; return `Ok(())` immediately if accepted |
| `is_busy` | `true` while executor thread running |
| `read_pose` | Live motor angles + `rail_x = 0.0` |

Control loop already skips planning when `is_busy()` (`pipeline/mod.rs`).

---

## Module layout

```
src/hardware/
  mod.rs           trait Hardware (unchanged)
  sim.rs
  real.rs          RealHardware facade
  dynamixel.rs     #[cfg(windows+real)] bus + mapping + executor
  rail_stub.rs     #[cfg(windows+real)] AXL placeholder
```

`RealHardware` holds:

- `DynamixelBus` — serial port + rustypot controller
- `MotorMapping` — signs, offsets, tick limits (from TOML)
- `SwingExecutor` — `Arc<Mutex<ExecutorState>>` or dedicated thread + channel
- `RailStub`

Public construction:

```rust
RealHardware::new(DynamixelConfig) -> Result<Self, HwError>
```

---

## Configuration (TOML)

Add optional section (required when `mode = "real"`):

```toml
mode = "real"

[hardware.dynamixel]
port = "COM8"
baudrate = 57600
protocol_version = 2.0
motor_ids = [1, 3, 4, 5]
joint_signs = [-1, 1, 1, 1]
joint_offsets_rad = [0.0, 0.0, 0.0, 0.0]
zero_tick = 2048
ticks_per_revolution = 4096
profile_velocity = 80
profile_acceleration = 20
comm_retries = 5
comm_retry_delay_ms = 20
# [[hardware.dynamixel.motor_limits_deg]] — or flat list matching motor_ids
motor_angle_limits_deg = [[90, 220], [135, 225], [92, 230], [120, 220]]

[hardware.rail]
enabled = false   # AXL stub; when false, rail_x fixed 0
```

`RuntimeConfig::validate`: if `mode == Real`, require `hardware.dynamixel` and `motor_ids.len() == arm.dof()` (4 for competition).

Example file: `config/real-hardware.toml` (no cameras yet — for `jog_axis` / pose read smoke).

---

## Motor mapping

Port Python logic verbatim:

```text
adjusted = sign * angle_rad + offset
ticks = round(zero_tick + adjusted * ticks_per_rev / 2π)
clamp to motor_tick_limits[joint_index]
```

Inverse for `read_pose`:

```text
raw = (ticks - zero_tick) * 2π / ticks_per_rev
angle_rad = sign * (raw - offset)
```

**Joint order:** URDF movable joint order for `competition` / `all-4-export` must match `motor_ids` order (Revolute 6→9→13→18 ↔ IDs 1,3,4,5). Document in `config/example.toml` and `assets/robots/4-dof/README.md`.

---

## rustypot integration

- Dependency: `rustypot` (MX series), `serialport` — `#[cfg(all(windows, feature = "real"))]` only in `Cargo.toml`.
- Open port with timeout ≥ 100 ms (Python uses SDK default; match bench).
- On `new()`:
  1. open serial
  2. `sync_write` profile acceleration / velocity (Protocol 2.0 registers 108, 112) — use `DynamixelProtocolHandler::sync_write` if high-level MX controller lacks profile helpers
  3. `enable_torque(true)` on all IDs
- Runtime:
  - `sync_write_goal_position(&motor_ids, &angles_rad_mapped)` — mapping outputs radians in motor frame; rustypot may accept radians directly on MX API — verify and use raw ticks if needed for exact parity with Python
  - `sync_read_present_position` for `read_pose`
- Shutdown (`Drop`): disable torque, close port (best-effort, like Python `close()`).

**Retry policy:** wrap bus ops with transient retry (CRC/timeout), port clear + sleep — mirror Python `_with_comm_retries`.

---

## SwingExecutor

**Why a thread:** `pipeline` Control calls `command` once and polls `is_busy` at 100 Hz; it does not sample the trajectory itself (unlike sim physics stepping `RobotState`).

**Algorithm:**

1. `command(traj)`: if already busy → return `Ok(())` or `HwError::Busy` (match sim: ignore duplicate).
2. Record `t0 = Instant::now()`, store `Arc<SwingTrajectory>`.
3. Spawn executor thread (or reuse single long-lived worker):
   - loop: `t = now - t0`
   - if `t >= duration_secs`: final sample, one last sync_write, exit
   - else: `q = traj.sample_at(t)`, sync_write, sleep `1/stream_hz`
4. `stream_hz`: default **200 Hz** (configurable); must be ≥ control_hz.

**Rail:** if `|traj.rail.end - traj.rail.start| > 1e-4` or rail velocity non-zero, `tracing::warn!` once per command; do not call `RailStub::command`.

**Safety:** clamp sampled angles through mapping tick limits before every write.

---

## AXL stub (`rail_stub.rs`)

```rust
pub struct RailStub;

impl RailStub {
    pub fn read_x(&self) -> f64 { 0.0 }
    pub fn command(&self, _motion: &RailMotion) -> Result<(), HwError> {
        Err(HwError::... NotImplemented) // or Ok if delta ≈ 0
    }
}
```

`RealHardware::read_pose` always sets `rail_x: self.rail.read_x()`.

---

## `run_real` (minimal slice)

Phase A (this design):

- Load `RealHardware` from TOML
- **Not** full camera pipeline yet
- `bail!` replaced with: connect hardware, read pose loop, or delegate to `jog_axis`-style smoke

Phase B (follow-up): wire `RealHardware` + cameras into `pipeline::run` like `run_sim`.

---

## `jog_axis` tool

Replace `todo!()`:

```text
jog_axis --config config/real-hardware.toml --joint 0 --angle-deg 5
jog_axis --goto-x --goto-y --goto-z   # optional: IK + single set_joint_positions (Python goto)
```

Uses same `RealHardware` / `DynamixelBus` as runtime — plan §3.4 “same code path”.

---

## Error mapping

| Condition | `HwError` |
|-----------|-----------|
| COM open fail | `ReadFailed` / new `ConnectFailed` detail |
| Sync write after retries | `CommandFailed` |
| motor_ids.len() ≠ arm DOF | config validation at startup |
| Trajectory while busy | ignore (sim parity) |

---

## Testing

| Test | Where |
|------|--------|
| `radians_to_ticks` / inverse round-trip | unit, `hardware/dynamixel.rs` |
| tick clamp at limits | unit, values from Python defaults |
| `SwingExecutor` samples monotonic time | unit with fake bus (dry-run trait) |
| Windows bench | manual: `jog_axis`, read pose, small swing |
| macOS CI | `cargo check --features real` may use `#[cfg(not(windows))]` stub module or compile-only |

---

## Implementation order

1. `DynamixelConfig` serde + TOML + validation
2. `dynamixel.rs` mapping + bus open/torque/read/write (no executor)
3. `jog_axis` single-position smoke on Windows
4. `SwingExecutor` + `RealHardware` trait impl
5. `rail_stub.rs` + rail warnings
6. `run_real` minimal connect/read loop
7. `config/real-hardware.toml` + README/TODO §4 checkboxes

---

## Open questions (defer)

- Exact rustypot MX controller type vs raw `DynamixelProtocolHandler` for profile registers — decide on first Windows compile.
- Whether duplicate `command` while busy should `Ok(())` or error — **default: Ok ignore** (sim).
- E-stop GPIO / software estop — document only until bench wiring known.

---

## References

- `test-manipulator/src/manipulator/dynamixel.py`
- `test-manipulator/src/manipulator/config.py`
- `plan.md` §3.2 Hardware, `tools/jog_axis`
- `src/hardware/sim.rs` — `is_busy` / `command` semantics
