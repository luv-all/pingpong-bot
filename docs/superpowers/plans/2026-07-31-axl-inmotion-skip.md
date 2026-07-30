# AXL InMotion Skip Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 스윙용 `AxmMoveStartPos` 호출 전에 InMotion이면 명령을 무시하고 warn만 남긴다. Dynamixel 스윙은 계속한다.

**Architecture:** `AxlLive::start_move_abs_m`에서 `AxmStatusReadInMotion`을 먼저 읽고, 이동 중이면 `AxmMoveStartPos`를 호출하지 않고 `Ok(())`를 반환한다. `move_abs_m` / `AxmMovePos` 경로는 변경하지 않는다.

**Tech Stack:** Rust, AXL FFI (`axl_live.rs`), `tracing::warn`

## Global Constraints

- 스윙용 `start_move_abs_m` / `command_abs_in_secs`만 변경
- 센터 복귀 `move_abs_m` / `AxmMovePos` 변경 금지
- InMotion이 아닌 AXL 실패는 기존 `Err` 유지
- `real.rs`의 “레일 실패 → 스윙 중단” 분기는 그대로 둔다 (InMotion은 `Ok`라 타지 않음)

---

### Task 1: InMotion soft-skip in `start_move_abs_m`

**Files:**
- Modify: `src/hardware/rail/axl_live.rs` (`start_move_abs_m`)
- Spec: `docs/superpowers/specs/2026-07-31-axl-inmotion-skip-design.md`

**Interfaces:**
- Consumes: `self.ffi.axm_status_read_in_motion`, `check_axl`, existing `start_move_abs_m(config, commanded_m, vel) -> Result<(), HwError>`
- Produces: same signature; InMotion이면 `Ok(())` + warn, Idle이면 기존 move

- [ ] **Step 1: Update `start_move_abs_m`**

```rust
pub(super) fn start_move_abs_m(
    &mut self,
    config: &RailConfig,
    commanded_m: f64,
    vel: f64,
) -> Result<(), HwError> {
    let mut in_motion = 0;
    check_axl("AxmStatusReadInMotion", unsafe {
        (self.ffi.axm_status_read_in_motion)(config.axis, &mut in_motion)
    })?;
    if in_motion != 0 {
        tracing::warn!(
            axis = config.axis,
            commanded_m,
            vel,
            "AXL 레일 InMotion — AxmMoveStartPos 무시"
        );
        return Ok(());
    }

    check_axl("AxmMotSetAbsRelMode", unsafe {
        (self.ffi.axm_mot_set_abs_rel_mode)(config.axis, 0)
    })?;
    check_axl("AxmMoveStartPos", unsafe {
        (self.ffi.axm_move_start_pos)(config.axis, commanded_m, vel, config.accel, config.decel)
    })?;
    return Ok(());
}
```

- [ ] **Step 2: Compile / test dry-run rail path**

Run: `cargo test --lib hardware::rail::axl_rail::tests -- --nocapture`
Expected: PASS (dry-run 경로 회귀 없음)

- [ ] **Step 3: Commit**

```bash
git add src/hardware/rail/axl_live.rs
git commit -m "fix(hardware): ignore AxmMoveStartPos while rail is InMotion"
```
