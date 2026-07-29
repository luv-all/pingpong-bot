# One type per file + domain modules — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** `src/` + `tools/`에서 파일당 주 타입 1개(+impl)로 쪼개고, 도메인 접두어는 모듈 경로로 올려 `ball::Observation` / `robot::State` / `camera::Id`처럼 대칭적인 공개 API를 만든다.

**Architecture:** 레이어 단위로 (1) 타입을 파일·폴더로 분리 (2) §3 rename·모듈 이동 (3) `lib.rs`/`mod.rs` re-export 정리 (4) `cargo check --workspace --all-targets`로 고정. 移行 중 한 레이어 안에서만 `pub use New as Old` 별칭을 잠깐 쓰고, 그 레이어 호출부 갱신 후 별칭 제거.

**Tech Stack:** Rust workspace, OpenCV(기존 `LIBCLANG_PATH` 관례), clap CLI tools.

**Spec:** `docs/superpowers/specs/2026-07-29-one-type-per-file-design.md`

## Global Constraints

- 파일 1개 = `trait` | `struct` | `enum` 하나 + 그 `impl`
- 도메인 접두어는 모듈에 (`ball`, `robot`, `camera`, `calib`, `shooter`, `swing`, `eval`)
- CLI `Cam*` / `Stereo*` Args, `Dynamixel*`, `Rail*`, `Pipeline*` 접두 유지
- 동작·알고리즘 변경 금지
- 주석: 단위·비자명만
- `tests/` 재구성은 범위 밖 (깨지면 최소 import만 수정)
- 레이어 종료마다: `cargo check --workspace --all-targets` (OpenCV 시 `LIBCLANG_PATH` 설정)
- 호출부는 모듈 경로로 도메인을 남긴다: `camera::Id`, `ball::Observation` (`use …::Id`로 도메인 생략 금지)
- 커밋은 피처 브랜치에서 태스크마다 진행 (사용자 승인됨)

### Rename map (확정)

| Old | New path / name |
|-----|-----------------|
| `BallObservation` | `ball::Observation` |
| `BallState` | `ball::State` |
| `BallEkf` | `ball::Ekf` |
| `BallKinematics` | `ball::Kinematics` |
| `BallHandle` | `ball::Handle` |
| `BallVisual` | `ball::Visual` |
| `BallSnapshot` | `ball::Snapshot` |
| `BallMsg` (있으면) | `ball::Msg` |
| `BallShooterSettings` | `shooter::Settings` |
| `ShooterLayout` | `shooter::Layout` |
| `ShooterHandle` | `shooter::Handle` |
| `CameraId` | `camera::Id` |
| `CameraParams` | `camera::Params` |
| `PixelPoint` | `camera::Pixel` |
| `CameraView` | `camera::View` |
| `CameraRole` | `camera::Role` |
| `RobotState` | `robot::State` (모듈 경로 강조; 타입명 `State`는 `robot` 안에) |
| `RobotPose` | `robot::Pose` |
| `RobotHandle` | `robot::Handle` (gui면 `sim` re-export) |
| `RobotVisual` | `robot::Visual` |
| Charuco / Table 접두 | `calib::charuco::*`, `calib::table::*` 짧은 이름 |
| Swing / Eval facades | `swing::*`, `eval::*` |

루트 `lib.rs`는 점진적으로 평탄 re-export를 줄이고 `pub mod ball;` 등으로 경로를 연다. 한 레이어가 끝날 때까지 임시 `pub use ball::Observation as BallObservation` 허용.

### Verify command

```bash
export LIBCLANG_PATH="$(xcode-select -p)/Toolchains/XcodeDefault.xctoolchain/usr/lib"
export DYLD_LIBRARY_PATH="$LIBCLANG_PATH${DYLD_LIBRARY_PATH:+:$DYLD_LIBRARY_PATH}"
cargo check --workspace --all-targets
cargo fmt --all
```

---

### Task 1: `ball` 모듈 골격 + `Observation`

**Files:**
- Create: `src/ball/mod.rs`, `src/ball/observation.rs`
- Modify: `src/camera/mod.rs` (BallObservation 제거), `src/lib.rs` (`pub mod ball`), 모든 `BallObservation` 사용처

**Interfaces:**
- Produces: `pub struct Observation { pub pixel: camera::Pixel, pub camera_id: camera::Id, pub timestamp: Instant }` — Task 1에서는 아직 `PixelPoint`/`CameraId` 別名을 필드 타입으로 써도 됨. Task 2에서 필드 타입까지 `Pixel`/`Id`로 맞춤.
- Produces: `pingpong_bot::ball::Observation` (+ 임시 `BallObservation` 별칭)

- [ ] **Step 1:** `src/ball/mod.rs` / `observation.rs` 추가, `camera/mod.rs`에서 `BallObservation` 정의·impl 이동 후 `Observation`으로 rename
- [ ] **Step 2:** `lib.rs`에 `pub mod ball;` 및 `pub use ball::Observation;` (+ 원하면 임시 `as BallObservation`)
- [ ] **Step 3:** workspace에서 `BallObservation` → `Observation` (또는 `ball::Observation`)로 import 갱신
- [ ] **Step 4:** Verify command 실행, 실패 시 수정
- [ ] **Step 5:** (사용자 승인 시) commit — `refactor: introduce ball::Observation`

---

### Task 2: `camera` 코어 타입 분리 + §3 rename

**Files:**
- Create under `src/camera/`: `id.rs`, `pixel.rs`, `params.rs` (Params는 calib에서 이동), `view.rs`, `role.rs` (io에서 이동 가능)
- Create: `src/camera/facade/{charuco,table_pnp,triangulate,preview}.rs`
- Split: `src/camera/calib/*`, `src/camera/io/**` — 파일당 1 타입
- Modify: `src/camera/mod.rs` → re-export만

**Interfaces:**
- Produces: `camera::Id`, `camera::Pixel`, `camera::Params`, `camera::View`, `camera::Role`
- Consumes: Task 1 `ball::Observation` (필드 타입을 `Id`/`Pixel`로 갱신)

- [ ] **Step 1:** `CameraId`→`Id`, `PixelPoint`→`Pixel`, `CameraParams`→`Params` 파일 분리·rename; 전역 치환 (`CameraId(` 튜플 구조체 주의)
- [ ] **Step 2:** facade 4종을 `camera/facade/`로 이동, `mod.rs`는 `pub use`만
- [ ] **Step 3:** `calib` / `io` 다타입 파일 분리 (cam_cli/, preview/ 폴더화). Charuco/Table 짧은 이름 적용
- [ ] **Step 4:** Verify + fmt
- [ ] **Step 5:** (승인 시) commit — `refactor: camera one-type-per-file and short names`

---

### Task 3: detector 1타입/파일

**Files:** `src/detector/appearance/colormask.rs`(6) 분리, `builder.rs`, `appearance/layer.rs` 등 위반 파일

- [ ] **Step 1:** 위반 파일 목록 재확인 후 타입당 파일 생성, `mod.rs` re-export
- [ ] **Step 2:** Verify
- [ ] **Step 3:** (승인 시) commit — `refactor: detector one-type-per-file`

---

### Task 4: estimator → `ball::{Ekf,Kinematics,…}`

**Files:**
- Move/rename: `BallEkf`→`ball::Ekf`, `BallKinematics`→`ball::Kinematics`
- Split: `src/estimator/mod.rs`, `measure/*` facades·타입
- Keep: `estimator` 모듈은 파이프라인 `Estimator` 오케스트레이션용으로 남기거나 `ball`로 흡수 — **오케스트레이션 `Estimator`는 `estimator`에 유지**, 공 물리/필터만 `ball`로

**Interfaces:**
- Produces: `ball::Ekf`, `ball::Kinematics`; `PhysicsIdentify`/`TrajAnalysis`는 `ball` 또는 `estimator::measure` — 형제 facade면 `ball`에 모으는 쪽 우선

- [ ] **Step 1:** Ekf/Kinematics 이동·rename, 호출부 갱신
- [ ] **Step 2:** measure/facade 1타입 파일 + 주석 슬림
- [ ] **Step 3:** Verify
- [ ] **Step 4:** (승인 시) commit — `refactor: ball ekf/kinematics domain module`

---

### Task 5: planner → `swing` (+ impact 정리)

**Files:** `src/planner/bang_bang.rs`, `swing/*`, `planner/mod.rs` facades  
- `SwingPlanner` → `swing::Planner` (모듈 `swing` 신설 또는 `planner/swing`을 `pub use`로 `swing`에 노출)

- [ ] **Step 1:** 다타입 파일 분리
- [ ] **Step 2:** `swing::{Planner, Trajectory, Feasibility, …}` rename + 호출부
- [ ] **Step 3:** Verify + commit(승인 시)

---

### Task 6: `robot` 짧은 이름

**Files:** `src/robot/mod.rs`(6), `serial`, `state`, `build`, `urdf`, `dynamics`, `rail`  
- `RobotState`→`State`, `RobotPose`→`Pose` 등 **`robot` 모듈 안**에서 짧게. 루트에서는 `robot::State`로 쓰거나 衝突 시 경로 유지.

- [ ] **Step 1:** 1타입/파일 분리
- [ ] **Step 2:** §3 rename (`State`/`Pose`/…). `Arm` 등은 이미 짧으면 유지
- [ ] **Step 3:** Verify + commit(승인 시)

---

### Task 7: hardware

**Files:** `dynamixel.rs`(7), `rail/*` — 접두 `Dynamixel*`/`Rail*` **유지**, 파일만 분리

- [ ] **Step 1:** 타입당 파일 + 폴더
- [ ] **Step 2:** Verify + commit(승인 시)

---

### Task 8: pipeline

**Files:** `src/pipeline/mod.rs`(5) → `pipeline/{config,error,thread,feed,pipeline}.rs` 등

- [ ] **Step 1:** 분리, `Pipeline*` 접두 유지
- [ ] **Step 2:** Verify + commit(승인 시)

---

### Task 9: sim — `ball`/`shooter`/`eval` + gui 분리

**Files:**
- `BallState`→`ball::State`, `BallShooterSettings`→`shooter::Settings`, `ShooterLayout`→`shooter::Layout`
- GUI: `BallHandle`→`ball::Handle`, `BallVisual`→`ball::Visual`, `RobotHandle`/`RobotVisual` 대칭
- `eval_protocol.rs`(10) → `eval/` 폴더
- `gui/**` 다타입 파일 전부 분리

**Interfaces:**
- Create: `src/shooter/mod.rs` 또는 `src/sim/shooter/`를 `pub mod shooter`로 크레이트 루트에 노출
- Create: `src/eval/mod.rs` (또는 `sim/eval` re-export as `eval`)

- [ ] **Step 1:** `shooter` + `ball::{State,Handle,Visual,Snapshot}` 이동
- [ ] **Step 2:** `eval` 타입 분리·짧은 이름
- [ ] **Step 3:** gui/physics/session 1타입/파일
- [ ] **Step 4:** Verify + commit(승인 시)

---

### Task 10: tools + defaults + 루트 평탄 re-export 축소

**Files:** `tools/*/src/**` 다타입 `main.rs` 분리; `src/lib.rs` 평탄 `pub use` 정리; defaults 위반분

- [ ] **Step 1:** 각 tool에서 Args/상태 타입을 파일로 분리, import를 `ball::`/`camera::` 경로로
- [ ] **Step 2:** `lib.rs`에서 임시 `as OldName` 별칭 제거, 도메인 `pub mod` 중심으로
- [ ] **Step 3:** Verify 전체
- [ ] **Step 4:** 위반 스캔:

```bash
rg -l '^(pub( \(crate\))? )?(struct|enum|trait) ' src tools --type rust | while read f; do
  c=$(rg -c '^(pub( \(crate\))? )?(struct|enum|trait) ' "$f");
  [ "$c" -gt 1 ] && echo "$c $f";
done
```

Expected: 출력 없음 (또는 합의된 예외 0건)

- [ ] **Step 5:** (승인 시) commit — `refactor: tools split and slim root re-exports`

---

## Spec coverage

| Spec | Task |
|------|------|
| 1 type/file + impl | 1–10 |
| 폴더화 · mod re-export | 1–10 |
| §3 ball/robot/camera/… | 1,2,4,5,6,9 |
| CLI/Dynamixel/Rail/Pipeline 접두 유지 | 2,7,8,10 |
| 주석 슬림 | 각 레이어 이동 시 |
| tests/ 제외 | Global |
| check per layer | 각 Task Verify step |

## Placeholder / consistency self-check

- Rename map는 Task 전반에서 동일 (`Observation`, `Id`, `Pixel`, `Params`, …)
- `robot::State` vs `ball::State` — 항상 모듈 경로로 구분
- commit 스텝은 사용자 승인 게이트
