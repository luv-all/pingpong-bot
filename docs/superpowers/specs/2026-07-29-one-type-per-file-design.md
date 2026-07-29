# One type per file — design

Date: 2026-07-29  
Status: approved

## Goal

`src/` + `tools/` 전역에서 **타입·모듈·작명**을 읽기 쉽고 대칭적으로 맞춘다.

- 파일 1개 = **주 타입 1개** (`trait` **또는** `struct`/`enum`) + 그 타입의 `impl`
- 같은 레이어·도메인은 **폴더(모듈)로 묶고**, `mod.rs`는 선언·`pub use`만
- 도메인 접두어는 타입명이 아니라 **모듈 경로**에 둔다 (`ball::Observation`, `robot::State`)
- 과도한 주석 제거 — **단위·비자명 제약**만 유지

## Non-goals

- `tests/` 재구성 (후속)
- 알고리즘·동작 변경
- 새 추상화/trait object 도입
- CLI `Cam*Args` / `Stereo*Args` 접두 제거 (clap·플래그와 맞춤 유지)

## Rules

| 항목 | 규칙 |
|------|------|
| 주 타입 | 파일당 `trait` 또는 `struct`/`enum` 하나 |
| impl | 주 타입과 같은 파일 |
| 연관 `type` / 상수 | 주 타입 파일에 둬도 됨 |
| 자유함수 | 결과 타입 파일, 또는 `ops.rs` |
| 작명 | 모듈이 도메인, 타입은 역할어만 (§3) |
| 주석 | 단위·비자명만 |
| 공개 API | 모듈 경로로 씀 (`camera::Id`). 루트/`use`로 도메인 생략 금지 |
| 호출부 | `use crate::camera;` 후 `camera::Id` — `use crate::camera::Id` / 루트 `Id` 금지 |

## Approach

레이어별 일괄: **파일 분리 + 모듈 이동 + rename**을 같은 패스에서.  
순서: camera(도메인 정리 포함) → detector → estimator/`ball` → planner → robot → hardware → pipeline → sim → tools.

레이어 끝날 때마다:

```bash
cargo check --workspace --all-targets
```

## §3 — 작명·모듈 대칭 (확정)

접두어를 타입에 붙이지 말고 모듈로 올린다.

| 모듈 | 타입 예 |
|------|---------|
| `ball` | `Observation`, `State`, `Ekf`, `Kinematics`, `Handle`, `Visual`, `Snapshot`, `Msg` |
| `robot` | `State`, `Pose`, `Handle`, `Visual`, `Arm`, … |
| `camera` | `Id`, `Params`, `Pixel`, `Frame`, `View`, `Role` |
| `calib` | `Calibration`; `charuco::{BoardSpec, Report, FrameDetect}`; `table::{Landmark, Pnp, PnpResult}` |
| `shooter` | `Settings`, `Layout`, `Handle` |
| `swing` | `Planner`, `Trajectory`, `Feasibility`, … |
| `eval` | `Protocol`, `Shot`, `Zone`, `Mode`, `Report`, … |

호출: `ball::Observation` (구 `BallObservation`), `camera::Id` (구 `CameraId`), `shooter::Settings` (구 `BallShooterSettings`).

**유지:** `Pipeline*`, `Dynamixel*`, `Rail*`, CLI `Cam*`/`Stereo*`.

移行: `pub use ball::Observation as BallObservation` 임시 별칭 가능 → 레이어 정리 후 제거.

## §1 — `camera` (+ calib · ball observation)

```
src/camera/
  mod.rs
  id.rs                 # Id
  params.rs             # Params (구 CameraParams — calib와 공유 시 calib 쪽도 가능)
  pixel.rs              # Pixel (구 PixelPoint)
  frame.rs              # 또는 io/frame.rs
  view.rs / role.rs
  facade/ …
  calib/ …
  io/ …
  tri/ …

src/ball/                 # 신규 도메인 모듈 (관측·상태·추정 facade가 모이는 곳)
  mod.rs
  observation.rs          # Observation
  state.rs                # (estimator에서 이동)
  …
```

`BallObservation` → `ball::Observation`으로 이 단계에서 이동.

## §2 — 레이어 순서

| # | Layer | Focus |
|---|--------|--------|
| 1 | camera + `ball::Observation` 시작 | §1 · §3 |
| 2 | detector | 1타입/파일 |
| 3 | estimator → `ball::{State,Ekf,Kinematics,…}` | §3 |
| 4 | planner / `swing` | |
| 5 | robot | 짧은 이름 + 모듈 |
| 6 | hardware | Dynamixel/Rail 접두 유지 |
| 7 | pipeline | |
| 8 | sim (`shooter`, visuals → `ball`/`robot`) | |
| 9 | tools | 타입 파일 분리 + 새 경로 import |
| — | defaults | 위반·경로만 |
| — | tests/ | 범위 밖 |

## Success criteria

- `src/`·`tools/`에서 한 파일에 주 타입 2개+ 없음
- 도메인 접두어가 타입명에 중복되지 않음 (§3 표)
- `cargo check --workspace --all-targets` 통과
- 임시 별칭은 해당 레이어 패스 종료 시 제거 목표

## Out of scope follow-ups

- `tests/` 동일 규칙
- 문서·TODO 경로 일괄 갱신
- CLI Args 접두 재설계
