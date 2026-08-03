# Mount Feasibility Tune Implementation Plan

> **For agentic workers:** Execute task-by-task. Steps use checkbox syntax.

**Goal:** Find a rail mount that raises swing feasibility (ratio≤1) without relaxing motor limits, then verify with Rapier rally success.

**Architecture:** Offline `mount_search` ranks mounts via `swing_feasibility`; apply winner to `rail_frame`; confirm with `shot_tune`.

**Tech Stack:** Rust, existing `mount-search` / `shot-tune` crates, `defaults::rail_frame`

## Global Constraints

- Do not change Dynamixel max joint speed or end-velocity scale logic
- Prefer smallest hardware move among near-tied feasibility scores

---

### Task 1: Baseline + mount sweep

- [x] Run `cargo run -p mount-search --release -- --json --top-n 10`
- [x] Record current mount (`base_y=-0.02`, `height=0.05`) feasible_count vs top candidates
  - baseline: **0/150** feasible, mean ratio **3.79**
  - chosen: `base_y=-0.10`, height=0.05 → **10/150**, mean **2.48**

### Task 2: Apply rail_frame

- [x] Map winner: `behind_table_end = 0.10`, `above_table = 0.05`
- [x] Update `src/defaults/robot.rs` `rail_frame` (+ comment citing mount_search)
- [x] `shot_tune` default base_y → -0.10; intercept comment updated

### Task 3: shot_tune verify

- [x] Focused band (speed 6.9–7.3 × pitch −5..−3 × height 0.16–0.18, 8 shots/cell, 216 shots)
  - baseline −0.02: success **17/216 (7.9%)**, avg median peak **2.38**
  - new −0.10: success **29/216 (13.4%)**, avg median peak **1.79**
- [x] Summarize A/B results for user
