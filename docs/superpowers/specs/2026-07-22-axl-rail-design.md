# Design: AXL linear rail (`AxlRail` + `jog-rail`)

**Date:** 2026-07-22  
**Status:** approved (user: scope B — jog + `read_pose`)  
**Reference:** `~/Downloads/Interfacing File/.../Linear/LM_interface` (`CLinear_actu.cpp`, `AXL.h` / `AXM.h`, sample `.mot` for field names only)  
**Out of scope (this slice):** swing-synced rail sampling, homing sequence, `.mot` file load, multi-axis rail, E-stop UI, full `run_real` vision

---

## Goal

Replace `RailStub` (`rail_x = 0`) with a Windows-only Ajinextek **AXL** driver so bench tooling and `RealHardware::read_pose` use a real linear position in **meters**.

**Done when:**

- `tools/jog-rail` moves the rail in absolute meters (default) or relative Δm (`--relative` / `--delta-m`).
- Domain / planner units stay meters; pulse conversion lives only inside `AxlRail`.
- `[hardware.rail]` holds all board parameters in TOML (no `AxmMotLoadParaAll` / `.mot`).
- Soft limits apply in **both** the app (clamp) and the board (`AxmSignalSetSoftLimit`).
- `RealHardware::read_pose` returns measured `rail_x` when rail is enabled.
- `Hardware::command` still **warns and ignores** non-zero `RailMotion` (arm-only); swing rail sync is a later slice.
- macOS / builds without board: unit tests for mapping·clamp·TOML; `--dry-run` without DLL calls.

---

## Context

| Layer | Prior state | This slice |
|-------|-------------|------------|
| Rail driver | `rail_stub.rs` always 0 | `AxlRail` via `libloading` → `AXL.dll` |
| Config | Dynamixel-only `[hardware.dynamixel]` | add `[hardware.rail]` |
| Jog | `jog-axis` (joints) | new `jog-rail` binary |
| Swing | warns on non-zero rail | unchanged |
| Units | planner `rail_x` in meters | unchanged; board unit set to meters |

**FFI approach (chosen):** dynamic load with `libloading` under `#[cfg(all(windows, feature = "real"))]`. Rejected: static `AXL.lib` link (fragile CI/mac), C++ `cxx` shim around `LM_interface` (API surface too small to justify).

**Open policy:** no soft-zero on open (`AxmStatusSetActPos/CmdPos` not called). Use whatever command/actual position the board already holds.

**Fixed I/O methods (bench):**

- `pulse_out_method = 4`
- `enc_input_method = 3`

---

## Architecture

```
[hardware.rail] TOML
        │
        ▼
   AxlFfi (libloading)  ──►  AXL.dll  (Windows + feature=real)
        │
        ▼
   AxlRail  (m ↔ pulse, clamp, MovePos / read, soft limit)
        │
   ┌────┴────┐
   ▼         ▼
jog-rail   RealHardware.read_pose  (rail_x measured)
           RealHardware.command    (non-zero rail → warn only)
```

- `enabled = false` / non-Windows / no `real` feature → keep stub behavior (`rail_x = 0`).
- `enabled = true` + open/API failure → **bail** (`HwError`); do not silently fall back to stub.

---

## Configuration `[hardware.rail]`

Added to `config/real-hardware.toml` (and documented in `config/example.toml` / README). Relative paths resolve against the TOML file directory (same as other assets).

| Field | Role |
|-------|------|
| `enabled` | When true, open DLL/board |
| `dll_path` | Path to `AXL.dll` |
| `axis` | Axis index (bench: `0`) |
| `irq_no` | `AxlOpen` argument (sample used `7`) |
| `pulses_per_meter` | Positive integer (or finite value stored as `u32` after validate); board unit via `AxmMotSetMoveUnitPerPulse(1.0, pulses)` so **1 board unit = 1 meter** |
| `x_min_m` / `x_max_m` | Required; `x_min_m < x_max_m`; app clamp + board soft limit |
| `vel` / `accel` / `decel` | `AxmMovePos` profile (and init profile setters as needed) |
| `min_vel` / `max_vel` | `AxmMotSetMinVel` / `AxmMotSetMaxVel` |
| `pulse_out_method` | Default **4** |
| `enc_input_method` | Default **3** |
| `abs_rel_mode` | Board mode after open; default `0` (absolute). Jog relative is still computed in-app as an absolute target |
| `profile_mode` | Default `3` (symmetric S-curve) |
| `accel_unit` | Default `0` (`unit/s²`) |
| Soft limit (`AxmSignalSetSoftLimit`) | `use = ENABLE(1)`, `stop_mode = 0` (emergency/immediate stop per AXL docs — confirm on bench), `selection = 0` (command position). Positions: `dPositivePos = x_max_m`, `dNegativePos = x_min_m` |
| Signal levels (TOML, axis 0) | `inposition_level` default `1`, `alarm_level` default `0`, `neg_end_limit` / `pos_end_limit` default `2` (former `.mot` axis-0). Applied via corresponding `AxmSignalSet*` APIs at open |

**Explicitly not in config / not called:**

- `mot_path`, `AxmMotLoadParaAll`
- Homing velocities / `AxmHome*` sequence
- Second axis (`.mot` axis 1) parameters

**Validation:** when `enabled = true`, require finite positive `pulses_per_meter`, finite `x_min_m < x_max_m`, present `dll_path`, and the motion/IO fields above. `mode = real` does **not** require rail (optional; stub OK).

---

## `AxlRail` API

```text
open(cfg) -> Result<AxlRail>
read_x_m() -> Result<f64>          // actual (fallback cmd) / pulses_per_meter
move_abs_m(x) -> Result<()>        // clamp → AxmMovePos + wait InMotion
move_rel_m(dx) -> Result<()>       // read → abs target → move_abs_m
Drop: servo off + AxlClose (best-effort)
```

**Open sequence:**

1. `libloading` load `dll_path`
2. `AxlOpen(irq_no)` (or no-op if already open — prefer fail if unexpected)
3. Motion module present check (`AxmInfoIsMotionModule`)
4. Apply TOML via `AxmMotSet*` / signal setters (no `.mot`)
5. `AxmSignalSetSoftLimit(..., x_max_m, x_min_m)` in **meters** (board unit = m)
6. `AxmSignalServoOn(axis, ENABLE)`
7. Set abs/rel + profile mode from config

**Move:** blocking wait on `AxmStatusReadInMotion` (same pattern as `CLinear_actu::move_actu`). App clamps target into `[x_min_m, x_max_m]` before command; board soft limit is a second line of defense.

---

## `jog-rail` CLI

New workspace member `tools/jog_rail` (separate from `jog-axis`).

| Flag | Meaning |
|------|---------|
| `--config` | Runtime TOML (default `config/real-hardware.toml`) |
| `--position-m` | Absolute target [m] (default mode; conflicts with relative) |
| `--delta-m` | Relative Δm; implies relative mode (no separate flag required) |
| `--dry-run` | Mapping/clamp/target only; no DLL |
| (implied) | `[hardware.rail] enabled = true` required for live move |

Log `read_x_m` before and after live moves. Enforce max step optionally later; for this slice rely on soft limits + operator care (document small Δ first).

---

## `RealHardware` integration

- Construct `AxlRail` when `hardware.rail.enabled`; else `RailStub`.
- `read_pose`: `rail_x = rail.read_x_m()` (or `0.0` from stub).
- `command`: if `|Δrail|` or rail velocity / follow-through rail significant → `tracing::warn!` once per command; **do not** call `AxlRail::move_*`. Arm `SwingExecutor` unchanged.

---

## Error handling

| Failure | Behavior |
|---------|----------|
| Missing DLL / load error | `HwError` / `anyhow` bail |
| `AxlOpen` / Set / Move non-success | bail with return code (and `AxlGetReturnCodeInfo` if cheap) |
| Target outside limits | clamp (jog may warn); never silently skip enable when user asked for live rail |
| `enabled=false` | stub; no DLL |

---

## Testing

**Unit (no DLL):**

- meters ↔ pulse round-trip
- absolute / relative target clamp into `[x_min_m, x_max_m]`
- TOML validate success / missing-field / bad range
- soft-limit argument packaging from `x_min_m` / `x_max_m`

**Bench (Windows + board):**

1. `jog-rail --dry-run --position-m …`
2. Small `--delta-m --relative`
3. Absolute `--position-m`; out-of-range request clamps
4. `pingpong-bot --features real` smoke: after a jog, `read_pose.rail_x` reflects motion (not stuck at 0)

---

## Docs / TODO

- README: Dynamixel section + `jog-rail` / `[hardware.rail]` / PATH note for `AXL.dll`
- `TODO.md` §4: stub checkbox → jog + measured `read_pose` done; swing rail sync still open
- `docs/phase2.md` / status table: AXL jog ready, swing sync pending

---

## Implementation order (for writing-plans)

1. `RailConfig` + TOML validate + example values in `real-hardware.toml`
2. `AxlFfi` + `AxlRail` (cfg-gated) + unit tests for mapping/clamp
3. Wire `RealHardware` read path; keep command warn-only
4. `tools/jog_rail` + README / TODO updates
5. Bench checklist (manual)

---

## Decisions log (brainstorm)

| Topic | Choice |
|-------|--------|
| Slice scope | B — jog + `read_pose` |
| Units | meters in domain; `pulses_per_meter` in TOML |
| Jog modes | absolute `--position-m` or relative `--delta-m` |
| CLI | separate `jog-rail` |
| Soft zero on open | no |
| Travel limits | required `x_min_m`/`x_max_m`; app + `AxmSignalSetSoftLimit` |
| Swing rail | warn-only this slice |
| FFI | `libloading` |
| `.mot` | not used; all constants in TOML |
| Pulse / encoder methods | 4 / 3 |
