# control_worker State Machine Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace `control_worker`'s six loose, implicitly-coupled local variables with an explicit `BallControlState` enum, then surface that state live in the `--preview` window instead of terminal logs — behavior stays identical, only the representation and the added visualization change.

**Architecture:** `BallControlState { Idle, Struck { track_seq, return_due_at, measurement } }` replaces `struck_track_seq` / `return_due_at` / `pending_impact_measurement`, which today are three independent `Option`s that happen to always change together by convention only. `CommandLatch` stays untouched (different, orthogonal concern). `PendingVerification`/`consecutive_misses` stay untouched behaviorally but get flagged as currently dead code. A new `RuntimeEvent::ControlState` carries the state to the main thread, which forwards it into `PreviewWindow`, which draws a small two-node diagram (`IDLE`/`STRUCK`) in a fixed corner of the existing camera mosaic.

**Tech Stack:** Rust, crossbeam-channel (existing `RuntimeEvent` channel — no new channel), OpenCV (`opencv` crate, `imgproc::rectangle`/`imgproc::put_text` — no new dependency).

**Spec:** [`docs/superpowers/specs/2026-08-05-control-worker-state-machine-design.md`](../specs/2026-08-05-control-worker-state-machine-design.md)

## Global Constraints

- Scope is `src/real/control_worker.rs`, `src/real/runtime_event.rs`, `src/real/run.rs`, `src/real/preview.rs`, `src/camera/io/preview/*`, `src/camera/facade/preview.rs`, plus three doc files. `estimator_worker.rs` and everything else in `run.rs`/`main_loop` beyond the one new match arm are untouched.
- Behavior must be preserved exactly — this is a representation change, not a logic change. Every existing test in `control_worker.rs` and `sim_child.rs` must keep passing unmodified.
- `PendingVerification` / `verify_due_command` / `log_verification` / `consecutive_misses` and their constants are not revived and not removed this pass — flag with a doc comment only, per the design doc's explicit decision.
- No new external dependencies. No new channels — reuse the existing `event_tx: Sender<RuntimeEvent>`.
- The state panel is fixed-size (not scaled by `overlay_scale`), drawn only when `--preview` is on. The `--sim` kiss3d window is not touched.
- This codebase writes explicit `return` statements at the end of every function (not bare tail expressions) — match that style in all new code.
- Run `cargo test --lib` (or the equivalent scoped command shown per task) after every implementation step, not just at the end.

---

### Task 1: `BallControlState` + `PendingImpactMeasurement` types

**Files:**
- Modify: `src/real/control_worker.rs` (add near the existing `CommandLatch`/`PendingVerification` type definitions, roughly line 60)
- Test: `src/real/control_worker.rs` (existing `#[cfg(test)] mod tests` block)

**Interfaces:**
- Consumes: nothing new.
- Produces: `struct PendingImpactMeasurement { track_seq: u64, rail_commanded_m: f64, joints_commanded: pingpong_bot::robot::Joints }`, `enum BallControlState { Idle, Struck { track_seq: u64, return_due_at: Instant, measurement: PendingImpactMeasurement } }`, `impl BallControlState { fn blocks(&self, track_seq: u64) -> bool }`. Task 2 consumes all of these.

- [ ] **Step 1: Write the failing tests**

Add to the `#[cfg(test)] mod tests` block at the bottom of `src/real/control_worker.rs` (after the existing `due_command_needs_two_stable_readbacks` test):

```rust
    #[test]
    fn idle_blocks_nothing() {
        let state = BallControlState::Idle;
        assert!(!state.blocks(1));
        assert!(!state.blocks(999));
    }

    #[test]
    fn struck_blocks_only_its_own_track() {
        let state = BallControlState::Struck {
            track_seq: 5,
            return_due_at: Instant::now(),
            measurement: PendingImpactMeasurement {
                track_seq: 5,
                rail_commanded_m: 0.30,
                joints_commanded: Joints::from_slice(&[0.0; 4]),
            },
        };
        assert!(state.blocks(5));
        assert!(!state.blocks(6));
    }
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cargo test --lib real::control_worker::tests::idle_blocks_nothing`

Expected: FAIL with "cannot find type `BallControlState` in this scope" (does not compile yet).

- [ ] **Step 3: Implement the types**

Add just above `struct PendingVerification` (currently at line 62 of `src/real/control_worker.rs`):

```rust
/// 임팩트 완주 후 실측 비교용 — 복귀 직전에 로그로 남긴다.
struct PendingImpactMeasurement {
    track_seq: u64,
    rail_commanded_m: f64,
    joints_commanded: pingpong_bot::robot::Joints,
}

/// 현재 공 하나의 처리 상태.
///
/// `Struck`의 세 필드는 항상 함께 만들어지고 함께 사라진다 — 예전에는 별도
/// `Option` 세 개(`struck_track_seq`, `return_due_at`,
/// `pending_impact_measurement`)로 표현해 그 불변식이 관례로만 유지됐다.
enum BallControlState {
    Idle,
    Struck {
        track_seq: u64,
        return_due_at: Instant,
        measurement: PendingImpactMeasurement,
    },
}

impl BallControlState {
    /// 이 상태가 주어진 `track_seq`의 추가 명령을 막는가.
    fn blocks(&self, track_seq: u64) -> bool {
        return matches!(
            self,
            BallControlState::Struck { track_seq: struck, .. } if *struck == track_seq
        );
    }
}
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cargo test --lib real::control_worker::tests::idle_blocks_nothing real::control_worker::tests::struck_blocks_only_its_own_track`

Expected: PASS (2 tests). You will also see a `warning: struct/enum is never constructed/used` — expected at this point, Task 2 wires it in.

- [ ] **Step 5: Commit**

```bash
git add src/real/control_worker.rs
git commit -m "$(cat <<'EOF'
feat(real): add BallControlState replacing three loose strike locals

EOF
)"
```

---

### Task 2: Wire `BallControlState` into the control loop, flag the dead verification path

**Files:**
- Modify: `src/real/control_worker.rs`

**Interfaces:**
- Consumes: `BallControlState`, `PendingImpactMeasurement` from Task 1.
- Produces: the `spawn()` loop now owns `state: BallControlState` instead of `struck_track_seq`/`return_due_at`/`pending_impact_measurement`. No new public interface — this is an internal wiring change.

- [ ] **Step 1: Replace the locals block**

In the `spawn()` closure (around line 115-121), replace:

```rust
        let mut latch = CommandLatch::default();
        let mut last_command: Option<Instant> = None;
        let mut pending_verification: Option<PendingVerification> = None;
        let mut return_due_at: Option<Instant> = None;
        let mut struck_track_seq: Option<u64> = None;
        let mut pending_impact_measurement: Option<(u64, f64, pingpong_bot::robot::Joints)> = None;
        let mut consecutive_misses: u8 = 0;
```

with:

```rust
        let mut latch = CommandLatch::default();
        let mut last_command: Option<Instant> = None;
        let mut pending_verification: Option<PendingVerification> = None;
        let mut state = BallControlState::Idle;
        let mut consecutive_misses: u8 = 0;
```

- [ ] **Step 2: Replace the return-to-center block**

Replace the block starting `if pending_verification.is_none()\n                && return_due_at.is_some_and(...)` (around lines 145-197) with:

```rust
            let due_for_return = match &state {
                BallControlState::Struck { return_due_at, .. } => Instant::now() >= *return_due_at,
                BallControlState::Idle => false,
            };
            if pending_verification.is_none() && due_for_return && !hardware.is_busy() {
                if let BallControlState::Struck { measurement, .. } = &state {
                    match hardware.read_pose() {
                        Ok(measured) => {
                            let joint_errors: Vec<f64> = measurement
                                .joints_commanded
                                .values
                                .iter()
                                .zip(&measured.joints.values)
                                .map(|(commanded, measured)| commanded - measured)
                                .collect();
                            info!(
                                track_seq = measurement.track_seq,
                                rail_commanded_m = f4(measurement.rail_commanded_m),
                                rail_measured_m = f4(measured.rail_x),
                                rail_commanded_minus_measured_m =
                                    f4(measurement.rail_commanded_m - measured.rail_x),
                                joints_commanded = %format!("{:?}", measurement.joints_commanded.values),
                                joints_measured = %format!("{:?}", measured.joints.values),
                                joints_commanded_minus_measured = %format!("{joint_errors:?}"),
                                "동시 임팩트 완료 후 실측"
                            );
                        }
                        Err(error) => warn!(%error, "동시 임팩트 완료 후 포즈 읽기 실패"),
                    }
                }
                if let Err(error) = move_to_center(hardware.as_mut(), &arm) {
                    let reason = format!("제어 후 중앙 복귀 실패: {error}");
                    warn!(%error, "제어 후 중앙 복귀 실패 — 제어를 중단한다");
                    let _ = event_tx.send(RuntimeEvent::Failed {
                        track_seq: latch.track_seq,
                        reason,
                    });
                    break;
                }
                match hardware.read_pose() {
                    Ok(pose) => {
                        if let Some(sim_tx) = &sim_tx {
                            let _ = sim_tx.try_send(SimUpdate {
                                pose: Some(PoseMsg::from(&pose)),
                                ..SimUpdate::default()
                            });
                        }
                        info!(track_seq = latch.track_seq, "제어 후 중앙 복귀 완료");
                    }
                    Err(error) => warn!(%error, "중앙 복귀 후 포즈 읽기 실패"),
                }
                state = BallControlState::Idle;
            }
```

(The `RuntimeEvent::ControlState` emission is added in Task 4 — this step only restores the `Idle` transition itself, so the loop behaves exactly as before in the meantime.)

- [ ] **Step 3: Replace the recv-timeout scheduling block**

Replace (around lines 207-216):

```rust
            if pending_verification.is_none()
                && let Some(due_at) = return_due_at
            {
                let return_wait = if due_at <= now && hardware.is_busy() {
                    BUSY_POLL
                } else {
                    due_at.saturating_duration_since(now)
                };
                timeout = timeout.min(return_wait);
            }
```

with:

```rust
            if pending_verification.is_none()
                && let BallControlState::Struck { return_due_at, .. } = &state
            {
                let return_wait = if *return_due_at <= now && hardware.is_busy() {
                    BUSY_POLL
                } else {
                    return_due_at.saturating_duration_since(now)
                };
                timeout = timeout.min(return_wait);
            }
```

- [ ] **Step 4: Replace the request-skip filter**

Replace (around lines 222-228):

```rust
            if !latch.should_send(request.track_seq, request.stage)
                || struck_track_seq == Some(request.track_seq)
                || request.age_secs() > MAX_REQUEST_AGE_SECS
                || last_command.is_some_and(|at| at.elapsed() < COMMAND_THROTTLE)
            {
                continue;
            }
```

with:

```rust
            if !latch.should_send(request.track_seq, request.stage)
                || state.blocks(request.track_seq)
                || request.age_secs() > MAX_REQUEST_AGE_SECS
                || last_command.is_some_and(|at| at.elapsed() < COMMAND_THROTTLE)
            {
                continue;
            }
```

- [ ] **Step 5: Replace the strike-construction block**

Replace (around lines 307-314):

```rust
            struck_track_seq = Some(request.track_seq);
            return_due_at = Some(issued_at + Duration::from_secs_f64(trajectory.duration_secs));
            pending_impact_measurement = Some((
                request.track_seq,
                applied.rail_m,
                trajectory.follow_through.clone(),
            ));
            pending_verification = None;
```

with:

```rust
            state = BallControlState::Struck {
                track_seq: request.track_seq,
                return_due_at: issued_at + Duration::from_secs_f64(trajectory.duration_secs),
                measurement: PendingImpactMeasurement {
                    track_seq: request.track_seq,
                    rail_commanded_m: applied.rail_m,
                    joints_commanded: trajectory.follow_through.clone(),
                },
            };
            pending_verification = None;
```

- [ ] **Step 6: Flag the dead verification subsystem**

Add this doc comment directly above `struct PendingVerification` (now shifted down by Task 1's insertion, still the first item in that block):

```rust
/// 명령 후 레일·조준축 재측정 상태.
///
/// **현재 실기 루프에서 도달 불가.** `spawn()`의 while 루프는 `pending_verification`을
/// 초기값 `None`과 명령 직후 재설정 `None` 외에는 `Some(...)`으로 대입하지 않는다.
/// 즉 `verify_due_command`의 수렴 판정·타임아웃·`consecutive_misses` 3회 연속
/// 중단 경로는 이 구조체를 직접 구성해 호출하는 유닛 테스트에서만 실행된다.
/// 부활 또는 제거는 별도 결정 사항으로 남아 있다 —
/// `docs/superpowers/specs/2026-08-05-control-worker-state-machine-design.md` 참고.
/// 이번 패스는 동작을 바꾸지 않는다.
```

- [ ] **Step 7: Run the full existing test suite for this file**

Run: `cargo test --lib real::control_worker`

Expected: PASS — all of `startup_initialization_sets_ready_rail_and_all_joints`, `each_prediction_stage_is_sent_only_once_per_ball`, `new_track_resets_latch_before_refined_stage`, `due_command_needs_two_stable_readbacks`, `idle_blocks_nothing`, `struck_blocks_only_its_own_track` pass, no compile errors, no leftover references to `struck_track_seq`/`return_due_at`/`pending_impact_measurement` as locals.

- [ ] **Step 8: Commit**

```bash
git add src/real/control_worker.rs
git commit -m "$(cat <<'EOF'
refactor(real): wire BallControlState into control_worker's loop

Replaces struck_track_seq/return_due_at/pending_impact_measurement —
three locals that only stayed in sync by convention — with one enum.
Flags PendingVerification as currently unreachable in production,
per the state-machine design doc's decision not to revive or remove
it this pass.
EOF
)"
```

---

### Task 3: Record the two findings in the docs

**Files:**
- Modify: `TODO.md` (§2.5, after line 210, before the `---` at line 212)
- Modify: `src/real/README.md` (after line 71, before the `### 제어 괴리 로그` heading at line 73)
- Modify: `docs/two-stage-position-control.md` (after line 85, before the `## 실기·시뮬레이션 공통 경계` heading at line 87)

**Interfaces:** None — documentation only, no code.

- [ ] **Step 1: Add the TODO.md finding**

In `TODO.md`, after the existing second bullet of §2.5 (ending "...사용자 요청으로 지금은 기록만 하고 코드 수정은 보류.") and before the `---`, add:

```markdown
- [ ] **`PendingVerification` 경로가 실기 루프에서 도달 불가 — 2026-08-05 확인, 미해결.**
  `pending_verification`은 선언 시 `None`, 명령 직후 재설정 `None` 외에는 실제
  `spawn()` 루프에서 `Some(...)`으로 대입되지 않는다 — 유닛 테스트가 직접
  구성해 `verify_due_command`를 호출할 때만 그 경로가 실행된다. 즉 아래
  "제어 괴리 로그"·"제어 워커" 섹션이 완료로 적은 재측정 수렴 판정·3회 연속
  실패 시 중단은 현재 실기에서 발동하지 않는다. 부활·제거 결정은 보류.
  `docs/superpowers/specs/2026-08-05-control-worker-state-machine-design.md` 참고.
- [ ] **`struck_track_seq`가 `Refined` 단계 명령을 사실상 막는다 — 2026-08-05 확인, 미해결.**
  명령이 하나 성공하면 단계와 무관하게 그 `track_seq`의 이후 요청을 전부
  건너뛴다. `Provisional`이 거의 즉시 도착하므로 `Refined`(0.25초 관측 후)는
  도착 전에 이미 막힌다 — "공마다 Provisional·Refined를 최대 한 번씩
  보낸다"는 아래 설명과 어긋난다. 의도한 동작인지 확인 필요.
```

- [ ] **Step 2: Add the README.md footnote**

In `src/real/README.md`, after the paragraph "위 3~5번은 `robot::control::DirectController`에 있으며 ... 같은 명령을 `robot::State`의 레일·조준 목표에 적용한다." (ending line 71), add:

```markdown

> **주의 (2026-08-05):** 아래 7~8번(재측정 수렴 판정·3회 연속 실패 시 중단)은
> 현재 실기 `spawn()` 루프에서 발동하지 않는다 — `pending_verification`이
> 유닛 테스트 밖에서는 채워지지 않는다. 상세:
> `docs/superpowers/specs/2026-08-05-control-worker-state-machine-design.md`.
```

- [ ] **Step 3: Add the two-stage-position-control.md footnote**

In `docs/two-stage-position-control.md`, after the paragraph ending "...`timeout`이 3회 연속 나면 레일을 정지하고 조준축을 현재 위치에 홀드한 뒤 제어 워커를 종료한다. 정상 수렴하면 연속 timeout 횟수는 0으로 돌아가며, `superseded`는 timeout으로 세지 않는다." (line 85), add:

```markdown

> **주의 (2026-08-05):** 위 수렴·타임아웃·3회 연속 중단 경로는 현재 실기
> 루프에서 도달 불가하다(`pending_verification`이 채워지지 않음). 문서는
> 의도한 설계를 기록한 것이고, 부활 여부는 별도 결정 사항으로 남아 있다.
```

- [ ] **Step 4: Commit**

```bash
git add TODO.md src/real/README.md docs/two-stage-position-control.md
git commit -m "$(cat <<'EOF'
docs(real): record dead verification path and Refined-blocking finding

EOF
)"
```

---

### Task 4: `ControlStateSnapshot` + `RuntimeEvent::ControlState`

**Files:**
- Modify: `src/real/runtime_event.rs`
- Modify: `src/real/mod.rs` (export)
- Modify: `src/real/control_worker.rs` (emit on both transitions)
- Modify: `src/real/run.rs` (keep the two exhaustive matches compiling)

**Interfaces:**
- Consumes: `BallControlState` fields from Task 2's call sites (read-only, no signature change to `BallControlState` itself).
- Produces: `pub enum ControlStateSnapshot { Idle, Struck { track_seq: u64, return_due_at: Instant, rail_commanded_m: f64, aim_commanded_rad: f64 } }`, `RuntimeEvent::ControlState { state: ControlStateSnapshot }`. Task 6 consumes `ControlStateSnapshot` in `PreviewWindow::set_control_state`.

- [ ] **Step 1: Add `ControlStateSnapshot` and the new `RuntimeEvent` variant**

In `src/real/runtime_event.rs`, add `use std::time::Instant;` to the imports at the top, then add above the `RuntimeEvent` enum:

```rust
/// 프리뷰 상태 패널이 그릴 현재 공 처리 상태 스냅샷.
#[derive(Debug, Clone, Copy)]
pub enum ControlStateSnapshot {
    Idle,
    Struck {
        track_seq: u64,
        return_due_at: Instant,
        rail_commanded_m: f64,
        aim_commanded_rad: f64,
    },
}
```

Then add this variant inside `RuntimeEvent` (after `Commanded`, before `Failed`):

```rust
    /// 현재 공 처리 상태가 바뀌었다 — 프리뷰 상태 패널이 소비한다.
    ControlState { state: ControlStateSnapshot },
```

- [ ] **Step 2: Export it**

In `src/real/mod.rs`, change:

```rust
pub use runtime_event::RuntimeEvent;
```

to:

```rust
pub use runtime_event::{ControlStateSnapshot, RuntimeEvent};
```

- [ ] **Step 3: Make `run.rs`'s exhaustive matches compile**

In `src/real/run.rs`, `main_loop`'s `match &event { ... }` block, add (after the `RuntimeEvent::Commanded { .. } => { ... }` arm):

```rust
                RuntimeEvent::ControlState { .. } => {}
```

In `log_event`, add (after the `RuntimeEvent::Commanded { .. } => info!(...)` arm):

```rust
        RuntimeEvent::ControlState { state } => debug!(?state, "제어 상태 전이"),
```

This requires `debug` to be imported in `run.rs` — change the existing `use tracing::{info, warn};` to `use tracing::{debug, info, warn};`. `ControlStateSnapshot` needs `Debug` (already derived in Step 1) for the `?state` field.

- [ ] **Step 4: Run a build check**

Run: `cargo build --lib`

Expected: builds clean (the two arms above are placeholders — Task 7 replaces the first one with real forwarding).

- [ ] **Step 5: Emit the event from `control_worker.rs`**

In the return-to-center block from Task 2 Step 2, right after the line `state = BallControlState::Idle;`, add:

```rust
                let _ = event_tx.send(RuntimeEvent::ControlState {
                    state: ControlStateSnapshot::Idle,
                });
```

In the strike-construction block from Task 2 Step 5, right after the `state = BallControlState::Struck { ... };` assignment (and after the existing `pending_verification = None;` line), add:

```rust
            let _ = event_tx.send(RuntimeEvent::ControlState {
                state: ControlStateSnapshot::Struck {
                    track_seq: request.track_seq,
                    return_due_at: issued_at + Duration::from_secs_f64(trajectory.duration_secs),
                    rail_commanded_m: applied.rail_m,
                    aim_commanded_rad: applied.aim_rad,
                },
            });
```

Add `ControlStateSnapshot` to the existing `use super::{CommitRequest, PoseMsg, RuntimeEvent, Shutdown, SimUpdate};` import line at the top of `control_worker.rs`, making it `use super::{CommitRequest, ControlStateSnapshot, PoseMsg, RuntimeEvent, Shutdown, SimUpdate};`.

- [ ] **Step 6: Run the tests**

Run: `cargo test --lib real::control_worker`

Expected: PASS, same tests as Task 2 Step 7 — this task adds a fire-and-forget send on an unbounded channel, nothing observable changes for these tests.

- [ ] **Step 7: Commit**

```bash
git add src/real/runtime_event.rs src/real/mod.rs src/real/control_worker.rs src/real/run.rs
git commit -m "$(cat <<'EOF'
feat(real): emit ControlStateSnapshot on Idle/Struck transitions

EOF
)"
```

---

### Task 5: Generic rectangle/text OpenCV primitives

**Files:**
- Modify: `src/camera/io/preview/ops.rs`
- Modify: `src/camera/io/preview/mod.rs`
- Modify: `src/camera/facade/preview.rs`

**Interfaces:**
- Consumes: nothing new (uses existing `opencv::imgproc`, `camera::Pixel`).
- Produces: `pub fn draw_rect_px(img: &mut Mat, top_left: camera::Pixel, width: i32, height: i32, color: Scalar, thickness: i32) -> CvResult<()>`, `pub fn draw_text_at_px(img: &mut Mat, origin: camera::Pixel, text: &str, font_scale: f64, color: Scalar, thickness: i32) -> CvResult<()>`, both re-exported through `camera::Preview`. Task 6 consumes both.

- [ ] **Step 1: Write the failing tests**

Add to the `#[cfg(test)] mod tests` block at the bottom of `src/camera/io/preview/ops.rs` (after `unscale_xy_roundtrips_at_half`):

```rust
    #[test]
    fn draw_rect_px_fills_the_requested_area() {
        let mut img = Mat::zeros(50, 50, opencv::core::CV_8UC3)
            .unwrap()
            .to_mat()
            .unwrap();
        draw_rect_px(
            &mut img,
            camera::Pixel::new(10.0, 10.0),
            20,
            20,
            Scalar::new(10.0, 20.0, 30.0, 0.0),
            -1,
        )
        .unwrap();
        let inside = *img.at_2d::<opencv::core::Vec3b>(20, 20).unwrap();
        assert_eq!(inside, opencv::core::Vec3b::from([10, 20, 30]));
        let outside = *img.at_2d::<opencv::core::Vec3b>(5, 5).unwrap();
        assert_eq!(outside, opencv::core::Vec3b::from([0, 0, 0]));
    }

    #[test]
    fn draw_text_at_px_writes_nonzero_pixels() {
        let mut img = Mat::zeros(50, 100, opencv::core::CV_8UC3)
            .unwrap()
            .to_mat()
            .unwrap();
        draw_text_at_px(
            &mut img,
            camera::Pixel::new(5.0, 30.0),
            "OK",
            1.0,
            Scalar::new(255.0, 255.0, 255.0, 0.0),
            2,
        )
        .unwrap();
        let mut any_lit = false;
        for y in 0..50 {
            for x in 0..100 {
                if *img.at_2d::<opencv::core::Vec3b>(y, x).unwrap() != opencv::core::Vec3b::from([0, 0, 0]) {
                    any_lit = true;
                }
            }
        }
        assert!(any_lit);
    }
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cargo test --lib camera::io::preview::ops::tests::draw_rect_px_fills_the_requested_area`

Expected: FAIL with "cannot find function `draw_rect_px` in this scope".

- [ ] **Step 3: Implement the primitives**

In `src/camera/io/preview/ops.rs`, add after `draw_circle_px`:

```rust
/// 사각형. `thickness < 0`이면 채움 (OpenCV 관례).
pub fn draw_rect_px(
    img: &mut Mat,
    top_left: camera::Pixel,
    width: i32,
    height: i32,
    color: Scalar,
    thickness: i32,
) -> CvResult<()> {
    imgproc::rectangle(
        img,
        opencv::core::Rect::new(
            top_left.x.round() as i32,
            top_left.y.round() as i32,
            width,
            height,
        ),
        color,
        thickness,
        imgproc::LINE_8,
        0,
    )?;
    return Ok(());
}

/// 임의 위치 텍스트 한 줄. 자동 스케일 없음 — 호출측이 `font_scale`을 정한다.
pub fn draw_text_at_px(
    img: &mut Mat,
    origin: camera::Pixel,
    text: &str,
    font_scale: f64,
    color: Scalar,
    thickness: i32,
) -> CvResult<()> {
    imgproc::put_text(
        img,
        text,
        Point::new(origin.x.round() as i32, origin.y.round() as i32),
        imgproc::FONT_HERSHEY_SIMPLEX,
        font_scale,
        color,
        thickness,
        imgproc::LINE_8,
        false,
    )?;
    return Ok(());
}
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cargo test --lib camera::io::preview::ops`

Expected: PASS (3 tests: the existing `unscale_xy_roundtrips_at_half` plus the 2 new ones).

- [ ] **Step 5: Export through `mod.rs` and the `Preview` facade**

In `src/camera/io/preview/mod.rs`, change:

```rust
pub use ops::{draw_cam_label, draw_circle_px, draw_world_velocity, hstack_bgr, unscale_xy};
```

to:

```rust
pub use ops::{
    draw_cam_label, draw_circle_px, draw_rect_px, draw_text_at_px, draw_world_velocity,
    hstack_bgr, unscale_xy,
};
```

In `src/camera/facade/preview.rs`, add `draw_rect_px, draw_text_at_px` to the `use crate::camera::io::{...}` import list, then add to `impl Preview`:

```rust
    pub fn draw_rect_px(
        img: &mut opencv::core::Mat,
        top_left: camera::Pixel,
        width: i32,
        height: i32,
        color: opencv::core::Scalar,
        thickness: i32,
    ) -> opencv::Result<()> {
        return draw_rect_px(img, top_left, width, height, color, thickness);
    }

    pub fn draw_text_at_px(
        img: &mut opencv::core::Mat,
        origin: camera::Pixel,
        text: &str,
        font_scale: f64,
        color: opencv::core::Scalar,
        thickness: i32,
    ) -> opencv::Result<()> {
        return draw_text_at_px(img, origin, text, font_scale, color, thickness);
    }
```

- [ ] **Step 6: Run the full preview module test suite**

Run: `cargo test --lib camera::io::preview`

Expected: PASS, no regressions in sibling tests (`fitted_bgr`, `pixel_pick_mouse`, `text_block`, `world_grid_params`).

- [ ] **Step 7: Commit**

```bash
git add src/camera/io/preview/ops.rs src/camera/io/preview/mod.rs src/camera/facade/preview.rs
git commit -m "$(cat <<'EOF'
feat(camera): add draw_rect_px and draw_text_at_px preview primitives

EOF
)"
```

---

### Task 6: `PreviewWindow` state panel

**Files:**
- Modify: `src/real/preview.rs`

**Interfaces:**
- Consumes: `draw_rect_px`, `draw_text_at_px` from Task 5; `ControlStateSnapshot` from Task 4.
- Produces: `impl PreviewWindow { pub fn set_control_state(&mut self, state: ControlStateSnapshot) }`. Task 7 consumes this method.

- [ ] **Step 1: Write the failing tests**

Add a `#[cfg(test)] mod tests` block at the bottom of `src/real/preview.rs` (the file has none yet):

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use opencv::core::Vec3b;
    use std::time::{Duration, Instant};

    fn idle_pixel(img: &Mat) -> Vec3b {
        return *img.at_2d::<Vec3b>(64, 290).unwrap();
    }

    fn struck_pixel(img: &Mat) -> Vec3b {
        return *img.at_2d::<Vec3b>(64, 390).unwrap();
    }

    #[test]
    fn idle_state_highlights_the_idle_node() {
        let mut img = Mat::zeros(200, 500, opencv::core::CV_8UC3)
            .unwrap()
            .to_mat()
            .unwrap();
        draw_control_state_panel(&mut img, &ControlStateSnapshot::Idle).unwrap();
        assert_eq!(idle_pixel(&img), Vec3b::from([20, 150, 235]));
        assert_eq!(struck_pixel(&img), Vec3b::from([120, 110, 100]));
    }

    #[test]
    fn struck_state_highlights_the_struck_node() {
        let mut img = Mat::zeros(200, 500, opencv::core::CV_8UC3)
            .unwrap()
            .to_mat()
            .unwrap();
        let state = ControlStateSnapshot::Struck {
            track_seq: 7,
            return_due_at: Instant::now() + Duration::from_millis(300),
            rail_commanded_m: 0.30,
            aim_commanded_rad: -0.40,
        };
        draw_control_state_panel(&mut img, &state).unwrap();
        assert_eq!(idle_pixel(&img), Vec3b::from([120, 110, 100]));
        assert_eq!(struck_pixel(&img), Vec3b::from([20, 150, 235]));
    }
}
```

`idle_pixel`/`struck_pixel` sample the center of each node box for a 500-wide test image: with `STATE_PANEL_MARGIN_PX=14`, `STATE_PANEL_W=250` → `panel_x = 500-250-14=236`; `idle_x = 236+14=250`, node center `x=250+40=290`; `struck_x = 250+80+20=350`, node center `x=350+40=390`; both centered at `node_y=14+34=48`, node height 32 → center `y=48+16=64`.

- [ ] **Step 2: Run tests to verify they fail**

Run: `cargo test --lib real::preview::tests::idle_state_highlights_the_idle_node`

Expected: FAIL with "cannot find function `draw_control_state_panel` in this scope".

- [ ] **Step 3: Implement the panel**

Add near the top of `src/real/preview.rs`, after the existing color constants (`STICKY_COLOR`, `MARKER_RADIUS_PX`, etc.):

```rust
/// 상태 패널 — 고정 크기, `--preview` 모자이크 우상단.
const STATE_PANEL_W: i32 = 250;
const STATE_PANEL_H: i32 = 110;
const STATE_PANEL_MARGIN_PX: i32 = 14;
const STATE_NODE_W: i32 = 80;
const STATE_NODE_H: i32 = 32;
const STATE_NODE_Y_OFFSET: i32 = 34;
const STATE_NODE_GAP: i32 = 20;
const STATE_BG_COLOR: Scalar = Scalar::new(18.0, 14.0, 10.0, 0.0);
const STATE_IDLE_COLOR: Scalar = Scalar::new(120.0, 110.0, 100.0, 0.0);
const STATE_ACTIVE_COLOR: Scalar = Scalar::new(20.0, 150.0, 235.0, 0.0);
```

Add `use super::ControlStateSnapshot;` and `use std::time::Instant;` to the imports at the top of the file.

Add this function (module-level, outside `impl PreviewWindow`):

```rust
/// `IDLE`/`STRUCK` 두 노드 다이어그램을 모자이크 우상단에 그린다.
fn draw_control_state_panel(image: &mut Mat, state: &ControlStateSnapshot) -> opencv::Result<()> {
    let panel_x = image.cols() - STATE_PANEL_W - STATE_PANEL_MARGIN_PX;
    let panel_y = STATE_PANEL_MARGIN_PX;
    camera::Preview::draw_rect_px(
        image,
        camera::Pixel::new(f64::from(panel_x), f64::from(panel_y)),
        STATE_PANEL_W,
        STATE_PANEL_H,
        STATE_BG_COLOR,
        -1,
    )?;

    let struck_active = !matches!(state, ControlStateSnapshot::Idle);
    let idle_color = if struck_active {
        STATE_IDLE_COLOR
    } else {
        STATE_ACTIVE_COLOR
    };
    let struck_color = if struck_active {
        STATE_ACTIVE_COLOR
    } else {
        STATE_IDLE_COLOR
    };

    let idle_x = panel_x + 14;
    let node_y = panel_y + STATE_NODE_Y_OFFSET;
    let struck_x = idle_x + STATE_NODE_W + STATE_NODE_GAP;

    camera::Preview::draw_rect_px(
        image,
        camera::Pixel::new(f64::from(idle_x), f64::from(node_y)),
        STATE_NODE_W,
        STATE_NODE_H,
        idle_color,
        -1,
    )?;
    camera::Preview::draw_text_at_px(
        image,
        camera::Pixel::new(f64::from(idle_x + 10), f64::from(node_y + 21)),
        "IDLE",
        0.5,
        Scalar::new(0.0, 0.0, 0.0, 0.0),
        1,
    )?;

    camera::Preview::draw_rect_px(
        image,
        camera::Pixel::new(f64::from(struck_x), f64::from(node_y)),
        STATE_NODE_W,
        STATE_NODE_H,
        struck_color,
        -1,
    )?;
    camera::Preview::draw_text_at_px(
        image,
        camera::Pixel::new(f64::from(struck_x + 4), f64::from(node_y + 21)),
        "STRUCK",
        0.42,
        Scalar::new(0.0, 0.0, 0.0, 0.0),
        1,
    )?;

    if let ControlStateSnapshot::Struck {
        track_seq,
        return_due_at,
        rail_commanded_m,
        aim_commanded_rad,
    } = state
    {
        let remaining = return_due_at
            .saturating_duration_since(Instant::now())
            .as_secs_f64();
        let line1 = format!("track {track_seq}  returns {remaining:.2}s");
        let line2 = format!(
            "rail {:.3}m  aim {:.1}deg",
            rail_commanded_m,
            aim_commanded_rad.to_degrees()
        );
        camera::Preview::draw_text_at_px(
            image,
            camera::Pixel::new(f64::from(panel_x + 14), f64::from(panel_y + 80)),
            &line1,
            0.4,
            STATE_ACTIVE_COLOR,
            1,
        )?;
        camera::Preview::draw_text_at_px(
            image,
            camera::Pixel::new(f64::from(panel_x + 14), f64::from(panel_y + 98)),
            &line2,
            0.4,
            STATE_ACTIVE_COLOR,
            1,
        )?;
    }
    return Ok(());
}
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cargo test --lib real::preview`

Expected: PASS (2 new tests). You will see an `unused function draw_control_state_panel` warning if `render()` isn't calling it yet — resolved in the next step.

- [ ] **Step 5: Add the field, setter, and wire it into `render()`**

Add a field to `PreviewWindow` (in its `struct` definition):

```rust
    /// 최근 제어 상태 — 다음 상태가 올 때까지 남는다.
    control_state: Option<ControlStateSnapshot>,
```

Initialize it in `PreviewWindow::new` (add to the returned `Self { .. }`):

```rust
            control_state: None,
```

Add the setter, next to `set_result`:

```rust
    /// 최근 제어 상태를 화면에 반영한다.
    pub fn set_control_state(&mut self, state: ControlStateSnapshot) {
        self.control_state = Some(state);
    }
```

In `render()`, after the existing `if !self.sticky.is_empty() { ... }` block and before the final `return Ok(...)` line, add:

```rust
        if let Some(state) = &self.control_state {
            draw_control_state_panel(&mut mosaic, state)?;
        }
```

- [ ] **Step 6: Run the full test suite for this file and a build check**

Run: `cargo test --lib real::preview && cargo build --lib`

Expected: PASS, clean build, no unused-function warnings remaining.

- [ ] **Step 7: Commit**

```bash
git add src/real/preview.rs
git commit -m "$(cat <<'EOF'
feat(real): draw a live IDLE/STRUCK state panel in PreviewWindow

EOF
)"
```

---

### Task 7: Forward `ControlState` events into the preview

**Files:**
- Modify: `src/real/run.rs`

**Interfaces:**
- Consumes: `PreviewWindow::set_control_state` from Task 6.
- Produces: nothing new — this completes the wiring started in Task 4.

- [ ] **Step 1: Replace the placeholder arm**

In `main_loop`'s `match &event { ... }` block, replace the placeholder added in Task 4 Step 3:

```rust
                RuntimeEvent::ControlState { .. } => {}
```

with:

```rust
                RuntimeEvent::ControlState { state } => {
                    if let Some(preview) = &mut preview {
                        preview.set_control_state(*state);
                    }
                }
```

- [ ] **Step 2: Build and run the full real-module test suite**

Run: `cargo test --lib real::`

Expected: PASS across `control_worker`, `preview`, `sim_child`, `ball_receding`, `estimator_worker` — no regressions anywhere in `src/real/`.

- [ ] **Step 3: Manual smoke check (no real hardware needed)**

Run: `cargo run --bin pingpong-bot -- --mode real --dry-run --preview --clip fly_07`

Expected: the preview window opens, shows the two-camera mosaic, and the state panel appears top-right showing `IDLE` highlighted while waiting and `STRUCK` highlighted (with `track N` / `returns Xs` / `rail`/`aim` lines) once a ball triggers a command. Close with `q`/ESC.

- [ ] **Step 4: Commit**

```bash
git add src/real/run.rs
git commit -m "$(cat <<'EOF'
feat(real): forward ControlState events into the preview window

EOF
)"
```

---

## Self-review notes

- **Spec coverage:** §"설계"(`BallControlState`/`PendingImpactMeasurement`/`CommandLatch` unchanged) → Tasks 1–2. §"조사 중 발견한 것" → Task 2 Step 6 (code comment) + Task 3 (docs). §"시각화" → Tasks 4–7. §"테스트" → existing tests preserved (Task 2 Step 7, Task 4 Step 6, Task 7 Step 2) plus new tests (Tasks 1, 5, 6). §"문서 갱신" → Task 3. §"범위 제외" → no task touches `estimator_worker.rs`, `run.rs`'s `Outcome`/`LastState`, or `--sim`.
- **Placeholder scan:** no TBD/TODO markers; every step has literal code or an exact shell command.
- **Type consistency checked:** `PendingImpactMeasurement` fields (`track_seq`, `rail_commanded_m`, `joints_commanded`) match between Task 1's definition and Task 2's construction/read sites. `ControlStateSnapshot::Struck` fields (`track_seq`, `return_due_at`, `rail_commanded_m`, `aim_commanded_rad`) match between Task 4's definition, Task 4 Step 5's construction, and Task 6's `draw_control_state_panel` destructuring. `draw_rect_px`/`draw_text_at_px` signatures match between Task 5's definition and Task 6's call sites.
