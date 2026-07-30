# macOS jog rail soft-skip Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** non-Windows에서 live AXL 레일을 soft-skip해 Dynamixel-only `RealHardware` / `jog`가 동작하게 한다.

**Architecture:** `RealHardware::from_bus` live 레일 분기를 `cfg(all(windows, feature = "real"))`로 가드한다. 그 외 OS는 warn 후 `rail=None` (`rail_x=0`). Windows live 실패는 기존 hard error.

**Tech Stack:** Rust, `tracing::warn`, existing `RealHardware` / `AxlRail` / `tools/jog` README.

**Spec:** `docs/superpowers/specs/2026-07-31-mac-jog-rail-skip-design.md`

## Global Constraints

- Windows live AXL open 실패는 soft-skip하지 않는다.
- `RailConfig::default().enabled`는 그대로 `true`.
- live Dynamixel + rail dry-run 혼합은 하지 않는다.
- 변경은 `from_bus`에 집중; jog/`main` 호출부 시그니처 변경 없음.

---

## File map

| File | Role |
|------|------|
| `src/hardware/real.rs` | soft-skip + unit test |
| `tools/jog/README.md` | Mac = Dynamixel only 한 줄 |

---

### Task 1: soft-skip + test + README

**Files:**
- Modify: `src/hardware/real.rs`
- Modify: `tools/jog/README.md`

- [x] **Step 1: Write failing test** in `real.rs` `#[cfg(test)]`:

```rust
#[cfg(not(all(windows, feature = "real")))]
#[test]
fn non_windows_live_rail_soft_skips_to_rail_x_zero() {
    let config = DynamixelConfig {
        stream_hz: 500.0,
        ..DynamixelConfig::default()
    };
    let stream_hz = config.stream_hz;
    let mut bus = DynamixelBus::dry_run(config).expect("dry bus");
    bus.configure_position_mode_max_effort().expect("mode");
    bus.enable_torque(true).expect("torque");
    let arm = Arc::new(
        (*crate::defaults::urdf_4dof()
            .expect("urdf")
            .arm)
            .clone(),
    );
    let mut hardware =
        RealHardware::from_bus(bus, stream_hz, Some(test_rail()), false, arm)
            .expect("hardware");
    assert_eq!(hardware.read_pose().expect("pose").rail_x, 0.0);
}
```

- [x] **Step 2: Run test — expect FAIL** (current `AxlRail::open` Err propagates)

```bash
cargo test -p pingpong-bot --features real non_windows_live_rail_soft_skips --lib
```

- [ ] **Step 3: Implement `from_bus` live branch**

```rust
Some(config) => {
    #[cfg(all(windows, feature = "real"))]
    {
        debug!(/* existing fields */, "레일 Live 개방");
        Some(AxlRail::open(config)?)
    }
    #[cfg(not(all(windows, feature = "real")))]
    {
        warn!(
            dll = %config.dll_path.display(),
            axis = config.axis,
            "AXL 레일은 Windows + feature=real 에서만 지원 — 레일 비활성, Dynamixel만 사용 (rail_x=0)"
        );
        None
    }
}
```

Import `warn` alongside `debug`/`error`.

- [ ] **Step 4: Run tests PASS**

```bash
cargo test -p pingpong-bot --features real real:: --lib
```

- [ ] **Step 5: README** — `tools/jog/README.md` 실행 절에 macOS 실기 = Dynamixel only, 레일 자동 스킵 한 줄.

- [ ] **Step 6: Commit**

```bash
git add src/hardware/real.rs tools/jog/README.md
git commit -S -m "feat(hardware): soft-skip AXL rail on non-Windows live RealHardware"
```
