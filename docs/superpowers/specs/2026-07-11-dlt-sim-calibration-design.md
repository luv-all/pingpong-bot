# Design: Phase 2 Slice 1 — DLT triangulation + sim pinhole calibration

**Date:** 2026-07-11  
**Status:** approved approach A (domain DLT; OpenCV cross-check later)  
**Scope:** milestone 1.2 + 1.4 (partial). Out of scope: ChArUco CLI, OpenCV detector, EKF, TOML config, torque FF.

---

## Goal

Replace the stub in `triangulate_synced` with real multi-view DLT so N camera pixels + calibration → `Point3<World>`.  
Make sim cameras use the **same** pinhole model / extrinsics as domain calibration so projection and triangulation stay consistent.

**Done when:**

- Synthetic test: known 3D point → project with N≥2 cameras → DLT recovers it with error ≪ 5 cm (target: &lt; 1 mm noise-free).
- Sim `CameraView` builds / shares `CameraParams` (no separate ad-hoc FOV math that diverges from DLT).
- Out-of-FOV dummy pixel `(320, 240)` for in-flight balls is removed or replaced with “no observation” (empty frame) so triangulation is not poisoned.

---

## Architecture

```
sim CameraView ──► CameraParams (K, R, t, size)
                         │
BallObservation pixels ──┤
                         ▼
              triangulate_synced (sample_at → DLT)
                         ▼
                   Point3<World>
```

- **domain** owns: `CameraParams`, projection matrix `P = K [R|t]`, DLT solve, `triangulate_synced`.
- **infra/sim** owns: placing cameras around the table, converting that pose into `CameraParams`, projecting ball → pixel via domain (or thin wrapper calling the same math).
- **No OpenCV** in this slice. Optional later: `tools/` cross-check with `triangulatePoints`.

---

## Data model

Extend `CameraParams` (keep `camera_id`, `label`):

| Field | Meaning |
|-------|---------|
| `width`, `height` | image size [px] |
| `fx`, `fy`, `cx`, `cy` | pinhole intrinsics |
| `rotation` | 3×3 world→camera rotation (`nalgebra::Matrix3<f64>`) |
| `translation` | 3×1 world→camera translation (`nalgebra::Vector3<f64>`) |

Conventions (lock these):

- World: existing Z-up, table corner origin.
- Camera: OpenCV-style, +Z forward, +X right, +Y down (pixel `v` increases downward).
- `X_cam = R * X_world + t`.
- Pixel: `u = fx * X_cam/Z_cam + cx`, `v = fy * Y_cam/Z_cam + cy`.

Helpers on `CameraParams`:

- `projection_matrix() -> Matrix3x4` — `P = K [R|t]`
- `project_world(Point3<World>) -> Option<PixelPoint>` — behind camera / outside image → `None`

`Calibration::default()` / sim factory: build 3 cameras matching today’s `CameraView::for_camera_index` layout (radius, height, look-at table center), with `fx/fy` derived from `fov_y` and image height.

---

## DLT algorithm

For each camera with pixel `(u, v)` and `P` (rows `p1, p2, p3`):

```
u * p3ᵀ X − p1ᵀ X = 0
v * p3ᵀ X − p2ᵀ X = 0
```

Stack 2N rows into `A` (N≥2), solve `A X = 0` in homogeneous coordinates via SVD (smallest singular vector). Dehomogenize to Euclidean `Point3<World>`.

Errors:

- Fewer than `min_cameras_for_triangulation()` (2) → existing `TriangulationInsufficient`
- Missing calibration for a camera id → new or reuse `ObservationError` (explicit; do not silently skip)
- Degenerate SVD / non-finite → `ObservationError` triangulation failed variant (add if missing)

Keep `sample_at` as-is for time sync before DLT.

---

## Sim integration

1. Add `CameraView::to_params(camera_id) -> CameraParams` (or construct `CameraParams` directly from the same eye/target/fov).
2. `SimCamera` projects with `CameraParams::project_world` (or shared helper) instead of divergent `CameraView::project` math — **one implementation**.
3. If ball is out of view: return `FrameRef::empty()` even when `InFlight` (remove center dummy pixel). Downstream may get fewer cameras; DLT still works with ≥2.
4. Wire a shared `Calibration` into the sim session / app path used for triangulation (today PassThroughEstimator may ignore it — at least triangulation call sites and tests use the real calib). Full estimator swap is slice 2 (EKF).

---

## Testing

| Test | Expectation |
|------|-------------|
| `project_roundtrip_center` | table center near image center for mid camera |
| `dlt_recovers_known_point` | 2–3 synthetic cameras, noise-free, error &lt; 1e-3 m |
| `dlt_needs_two_cameras` | one camera → `TriangulationInsufficient` |
| `triangulate_synced_with_series` | interpolated pixels at `sync_time` still recover point |
| sim smoke (optional) | 3 cameras, static ball, triangulated vs Rapier truth RMSE &lt; 5 cm |

---

## Non-goals (explicit)

- ChArUco / loading calib from disk (1.1, 1.5)
- OpenCV detector (1.3)
- EKF / replacing sim truth-based swing prediction (milestone 2)
- Distortion coefficients (assume zero for sim; real calib can add later)

---

## Implementation order

1. Extend `CameraParams` + helpers + sim layout factory  
2. Implement DLT + unit tests  
3. Point `SimCamera` / `CameraView` at shared projection; drop dummy pixel  
4. Update `Calibration::default` and any broken call sites  
5. Run `cargo test -p pingpong-bot --lib` and `cargo test -p pingpong-bot --features gui`

---

## Follow-ups (later slices)

- Slice 2: EKF + hit-plane; feed triangulated points instead of Rapier truth  
- Optional: OpenCV `triangulatePoints` tool for cross-check  
- Real ChArUco → fill same `CameraParams` fields
