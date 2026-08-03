# Detector Builder Implementation Plan

> **For agentic workers:** Implement task-by-task. Steps use checkbox syntax.

**Goal:** SimScene-style `Detector::builder()` with `.mask` / `.then(layer)` / `.scorer` / `.roi`; appearance order = `.then` call order.

**Architecture:** `AppearanceLayer` trait + `AppearanceChain` as `CandidateGenerator`; `Detector` bundle replaces `SpatialGate`; `detector_for` declares `ColormaskDetector` / `ContourDetector` and chains them.

**Tech Stack:** Rust, existing fuse/track/OpenCV detectors.

## Global Constraints

- mask always front; ROI is track policy not a stage
- layers are constructed objects passed to `.then`
- no silent fallback without calib/colormask

---

### Task 1: AppearanceLayer + AppearanceChain

**Files:**
- Create: `src/detector/appearance/layer.rs`
- Modify: `src/detector/appearance/mod.rs`, colormask.rs, contour.rs (impls), cascade.rs optional keep

- [ ] Add `AppearanceLayer` trait + `AppearanceChain`
- [ ] Impl for `ColormaskDetector`, `ContourDetector` (gated dilate∩prior for contour)
- [ ] Unit test: color→contour finds blob; order recorded in chain len

### Task 2: Detector + Builder

**Files:**
- Replace/rename: `src/detector/spatial/gate.rs` → Detector bundle + builder (or `src/detector/builder.rs`)
- Modify: spatial/mod.rs, detector/mod.rs, lib.rs

- [ ] `Detector { mask, roi, scorer }` + `BallDetector`
- [ ] `DetectorBuilder` with `.mask` `.then` `.scorer` `.roi` `.build`
- [ ] Remove `SpatialGate` (or type alias deprecated)

### Task 3: defaults + callers

**Files:** `src/defaults/vision.rs`, `tools/detect_full/src/main.rs`, exports

- [ ] `detector_for` uses builder with explicit layer decls
- [ ] detect_full: `detector.roi` instead of `.inner` / Deref
- [ ] `cargo test -p pingpong-bot --lib` + `cargo build -p detect-full`
