# 리니어 레일 정렬 통합 + 온디맨드 홈잉 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 흩어진 레일 상수를 `src/defaults/rail.rs` 한 곳으로 모으고, 실기 AXL 레일에
엔드스톱 알람 기반 온디맨드 홈잉 알고리즘을 추가하고, 그 결과를 재빌드 없이 반영되는
캘리브레이션 파일로 저장하고, 재정렬 절차를 문서화한다.

**Architecture:** (1) 순수 리팩터 — 기존 `RAIL_*` 상수/`rail_frame()`을 새 모듈로 옮기고
모든 참조 경로를 갱신한다. (2) `AxlRail`에 `home()`을 추가해 저속 이동 + AXL
`AxmSignalReadServoAlarm` 폴링으로 물리적 엔드스톱을 감지하고 `board_zero_domain_m`을
역산한다. (3) 그 값을 `data/rail_calibration.json`에 저장하고, 하드웨어 조립 시점에
기존 하드코딩 기본값 위에 덮어쓴다. (4) `--calibrate-rail` CLI 플래그와 jog 툴 버튼으로
온디맨드 실행 경로를 노출한다.

**Tech Stack:** Rust, `libloading`(AXL DLL FFI), `serde`/`serde_json`(캘리브레이션 파일),
`clap`(CLI), `kiss3d`/`egui`(jog 툴 GUI). 새 외부 크레이트 의존성 없음.

## Global Constraints

- 이 코드베이스는 함수 끝에서도 암묵적 반환 대신 명시적 `return`을 쓴다 — 새 코드도 이
  스타일을 따른다.
- `src/hardware/rail/axl_ffi.rs`, `axl_live.rs`, `axl_rail.rs`의 `Live` 관련 코드는
  `#[cfg(all(windows, feature = "real"))]`로 게이트돼 있다 — **macOS/Linux 개발 머신에서는
  이 경로가 컴파일조차 되지 않는다.** 이 경로를 건드리는 태스크는 문법·타입을 신중히
  맞추는 것 외에 로컬에서 `cargo build`/`cargo test`로 검증할 수 없다. 검증은
  Windows+AXL 벤치에서 이뤄진다 — 각 태스크에 그 사실을 명시한다.
- 볼 정렬(`ALIGNMENT_*` 상수, `plan_ball_alignment*`, `Aligning` 상태)은 이 계획의 범위
  밖이다 — 이름이 비슷할 뿐 다른 기능이며 손대지 않는다.
- 상수 이동은 순수 리팩터다. 기존 값·동작을 바꾸지 않는다 — `cargo test`가 기존 assert
  (`defaults/mod.rs`의 `presets_validate` 등)를 그대로 통과해야 한다.
- 새 파일/함수에 필요 이상의 추상화(트레이트, 제네릭, 설정 옵션)를 넣지 않는다.
- Spec: `docs/superpowers/specs/2026-08-12-rail-homing-and-calibration-consolidation-design.md`

---

### Task 1: 레일 상수 통합 (`src/defaults/rail.rs`)

**Files:**
- Create: `src/defaults/rail.rs`
- Modify: `src/defaults/hardware.rs`
- Modify: `src/defaults/robot.rs`
- Modify: `src/defaults/motion.rs`
- Modify: `src/defaults/mod.rs`
- Modify: `src/robot/motion/bang_bang/guidance.rs:51-55`
- Modify: `src/robot/control.rs:302,866`
- Modify: `src/robot/motion/physics.rs:1559,2840`
- Modify: `src/robot/state.rs:126,265,565`
- Test: 새 테스트 없음 — 기존 `cargo test`가 회귀 가드

**Interfaces:**
- Produces (다음 태스크가 쓰는 이름/시그니처):
  - `crate::defaults::rail::RAIL_BOARD_ZERO_DOMAIN_M: f64`
  - `crate::defaults::rail::RAIL_PHYSICAL_X_MIN_M: f64`, `RAIL_PHYSICAL_X_MAX_M: f64`
  - `crate::defaults::rail::RAIL_HOMING_VELOCITY_M_S: f64` (신규)
  - `crate::defaults::rail::DEFAULT_RAIL_CALIBRATION_PATH: &str` (신규,
    `"data/rail_calibration.json"`)
  - `crate::defaults::rail::rail_calibration_path() -> std::path::PathBuf` (신규)
  - `crate::defaults::rail::rail_frame() -> RailFrame`

- [ ] **Step 1: `src/defaults/rail.rs` 새로 작성**

`src/defaults/hardware.rs`의 다음 부분을 그대로(주석 포함) 옮긴다: `RAIL_LEFT_END_MARGIN_M`,
`RAIL_RIGHT_END_MARGIN_M`, `RAIL_PHYSICAL_X_MIN_M`, `RAIL_PHYSICAL_X_MAX_M`,
`RAIL_COORDINATE_POSITIVE_X_OFFSET_M`, `RAIL_POSITIVE_X_TRIM_M`,
`RAIL_NEGATIVE_X_ZERO_SHIFT_M`, `RAIL_BOARD_ZERO_DOMAIN_M`, `RAIL_X_MIN_M`, `RAIL_X_MAX_M`,
`RAIL_READY_X_M`(`hardware.rs:8-32`). `src/defaults/robot.rs`의 `RAIL_MAX_SPEED`
(`robot.rs:28`)와 `rail_frame()`(`robot.rs:94-116`, 주석 전부 포함)을 옮긴다.
`src/defaults/motion.rs`의 `RAIL_ACCEL_M_S2`(`motion.rs:11-16`, 주석 포함)를 옮긴다.
그 위에 두 개를 새로 추가한다:

```rust
//! 리니어 레일 좌표계·프레임 SSOT — 영점·범위·마운트·모션 상수를 한 곳에 모은다.
//!
//! 물리 규격(단면 두께 등)은 [`crate::constants::geometry::RAIL_THICKNESS`]에 남는다 —
//! CAD 실측 규격은 `constants`, 배선·튜닝값은 `defaults`가 맞는 자리라서다.

use crate::robot::RailFrame;

/// 실기 좌측 안전 마진 [m].
pub const RAIL_LEFT_END_MARGIN_M: f64 = 0.0100;
/// 실기 우측 안전 마진 [m].
pub const RAIL_RIGHT_END_MARGIN_M: f64 = 0.0705;
/// 실기에서 확인한 레일 좌표 범위 [m].
pub const RAIL_PHYSICAL_X_MIN_M: f64 = 0.0;
pub const RAIL_PHYSICAL_X_MAX_M: f64 = 1.41;
/// AXL 보드 실측 원점(보드 0.0m)에 대응하는 제어 좌표 [m].
///
/// 레일 기하학적 원점에 더하는 논리 +X 좌표계 보정 [m].
/// 타격 목표나 IK 결과가 아니라 AXL board↔domain 좌표 변환에 한 번만 적용한다.
/// `reverse=true`이므로 실물 +X 2.5cm 보정은 보드 목표에서 2.5cm를 뺀다.
/// 기존 +4.0cm 기준에서 2.5cm를 뺀 최종 좌표 오프셋이다.
pub const RAIL_COORDINATE_POSITIVE_X_OFFSET_M: f64 = 0.015;
/// 보드 실측 0.745m를 준비 중앙 0.675m로 해석하는 영점 이동.
pub const RAIL_POSITIVE_X_TRIM_M: f64 = 0.030;
pub const RAIL_NEGATIVE_X_ZERO_SHIFT_M: f64 =
    (RAIL_PHYSICAL_X_MAX_M - RAIL_PHYSICAL_X_MIN_M) / 2.0 - RAIL_POSITIVE_X_TRIM_M;
/// **`--calibrate-rail` 홈잉 미실행 시 폴백 기본값.** 홈잉을 한 번이라도 실행하면
/// `data/rail_calibration.json`의 값이 런타임에 이 상수를 덮어쓴다
/// (`hardware::rail::rail_calibration`).
pub const RAIL_BOARD_ZERO_DOMAIN_M: f64 =
    0.7050 + RAIL_COORDINATE_POSITIVE_X_OFFSET_M + RAIL_NEGATIVE_X_ZERO_SHIFT_M;
/// sim·real 공통 이동 범위 [m].
pub const RAIL_X_MIN_M: f64 = RAIL_PHYSICAL_X_MIN_M + RAIL_LEFT_END_MARGIN_M;
pub const RAIL_X_MAX_M: f64 = RAIL_PHYSICAL_X_MAX_M - RAIL_RIGHT_END_MARGIN_M;
/// 탁구대 실측 중앙 보정 위치 [m].
pub const RAIL_READY_X_M: f64 = 0.6750;
/// 최대 이동 속도 [m/s].
pub const RAIL_MAX_SPEED: f64 = 7.5;
/// 실기 AXL 레일 가속/감속 [m/s²] — `RailConfig::default()`도 이 값을 쓴다.
///
/// 기존 24 m/s²는 최단시간 이동과 겹치면 출발·정지 충격이 크다.
/// 실물 안전 운전은 12 m/s²로 낮춰 예측 보정 중에도 부드럽게
/// 가감속하고, 시뮬레이션과 계획기도 같은 한계를 사용한다.
pub const RAIL_ACCEL_M_S2: f64 = 12.0;
/// 홈잉 이동 속도 [m/s] — `min_vel`보다 크고 `max_vel`보다 훨씬 작다. 엔드스톱에
/// 부딪히는 순간의 충격·오버런을 줄이려는 값이다.
pub const RAIL_HOMING_VELOCITY_M_S: f64 = 0.02;

/// 홈잉 결과 캘리브레이션 JSON 경로. `data/calibration.json`(카메라)과 같은 자리.
pub const DEFAULT_RAIL_CALIBRATION_PATH: &str = "data/rail_calibration.json";

/// [`DEFAULT_RAIL_CALIBRATION_PATH`]의 `PathBuf`.
pub fn rail_calibration_path() -> std::path::PathBuf {
    return std::path::PathBuf::from(DEFAULT_RAIL_CALIBRATION_PATH);
}

/// 리니어모터를 받치는 철제 프로파일 (탁구대 끝면·바닥 기준).
///
/// **높이는 실측(2026-07-30).** 바닥→프로파일 하단 0.88 m,
/// 두께 [`RAIL_THICKNESS`](crate::constants::geometry::RAIL_THICKNESS) 0.055 m →
/// 베이스 z = **0.935**. 이전 값은 `SURFACE_Z + 0.05` = 0.81로, "실기 브래킷
/// (~면 위 3~5cm)과 맞춤"이라는 추정에 기대고 있었는데 실측이 그 가정을
/// 뒤집었다 — 시뮬 베이스가 실물보다 12.5 cm 낮았다.
///
/// `mount_y`는 실측값 **-0.128**을 쓴다 — `mount_search`(2026-07-26)가 낮은
/// 베이스 기준으로 추천한 `behind=0.10`(y=−0.10, `behind=0.02` 대비 ratio≤1이
/// **10/150**, mean≈2.48)은 그 스윕이 **낮은 베이스 기준**이라 0.935에서는
/// 최적값이 아니었고, 이후 실측이 이 값으로 대체했다.
///
/// 두 값 모두 sim GUI "Rig" 패널에서 공이 주차된 동안 런타임 조정 가능하다
/// (`SimRuntimeControls::rail_frame`). 좋은 위치를 눈으로 찾은 뒤
/// `mount_search`/`--rest-pose-search`를 그 위치에서 다시 돌려 여기와
/// [`crate::defaults::robot::READY_JOINTS_4DOF`]를 확정하는 것이 순서다.
pub fn rail_frame() -> RailFrame {
    return RailFrame {
        mount_y: -0.128,
        rail_bottom_z: 0.88,
    };
}
```

- [ ] **Step 2: `src/defaults/hardware.rs`에서 이동한 항목 제거하고 새 경로로 참조**

`hardware.rs:8-32`(옮긴 `RAIL_*` 상수 블록 전체)를 삭제한다. 파일 맨 위
`use super::motion::RAIL_ACCEL_M_S2;`(`hardware.rs:6`)를 삭제하고 대신
`use super::rail::{RAIL_ACCEL_M_S2, RAIL_BOARD_ZERO_DOMAIN_M, RAIL_MAX_SPEED, RAIL_PHYSICAL_X_MAX_M, RAIL_PHYSICAL_X_MIN_M, RAIL_X_MAX_M, RAIL_X_MIN_M};`
를 추가한다. `RailConfig::default()` 안의 `crate::defaults::robot::RAIL_MAX_SPEED`
(두 곳, `hardware.rs:136,140`)를 `RAIL_MAX_SPEED`로 바꾼다(위에서 import했으므로 경로
불필요). `board_zero_domain_m: RAIL_BOARD_ZERO_DOMAIN_M` 등 나머지 필드는 이미 짧은
이름을 쓰므로 그대로 둔다.

- [ ] **Step 3: `src/defaults/robot.rs`에서 이동한 항목 제거하고 새 경로로 참조**

`robot.rs:28`(`RAIL_MAX_SPEED` 상수)와 `robot.rs:94-116`(`rail_frame()` 함수 전체,
그 위 doc 주석 포함)을 삭제한다. `use crate::defaults::hardware::{RAIL_READY_X_M, RAIL_X_MAX_M, RAIL_X_MIN_M};`
를 `use crate::defaults::rail::{RAIL_READY_X_M, RAIL_X_MAX_M, RAIL_X_MIN_M, rail_frame};`로
바꾼다. `primitive_4dof()`(`robot.rs:121-124`)는 이미 `rail_frame()`을 호출하므로 본문은
그대로 두되, 이제 로컬 함수가 아니라 import된 함수를 호출한다.
`robot.rs:189`의 `RAIL_MAX_SPEED` 참조와 `robot.rs:505-508`의 테스트(`rail_frame()` 호출)도
새 import로 그대로 해결된다(이름이 같으므로 본문 수정 불필요).

- [ ] **Step 4: `src/defaults/motion.rs`에서 `RAIL_ACCEL_M_S2` 제거**

`motion.rs:11-16`(상수 + doc 주석)을 삭제한다. 이 파일의 다른 상수는 그대로 둔다.

- [ ] **Step 5: `src/defaults/mod.rs` 모듈 선언·재수출 갱신**

`pub mod calib;` 옆에 `pub mod rail;`을 추가한다(다른 모듈이 `defaults::rail::X`로
직접 참조하므로 `pub`). 기존 재수출 블록에서:
- `pub use hardware::{...}`(`mod.rs:41-46`)에서 `RAIL_BOARD_ZERO_DOMAIN_M`,
  `RAIL_COORDINATE_POSITIVE_X_OFFSET_M`, `RAIL_LEFT_END_MARGIN_M`,
  `RAIL_NEGATIVE_X_ZERO_SHIFT_M`, `RAIL_PHYSICAL_X_MAX_M`, `RAIL_PHYSICAL_X_MIN_M`,
  `RAIL_POSITIVE_X_TRIM_M`, `RAIL_READY_X_M`, `RAIL_RIGHT_END_MARGIN_M`, `RAIL_X_MAX_M`,
  `RAIL_X_MIN_M`을 제거한다(`BASE_JOINT_ZERO_OFFSET_RAD`, `WRIST_JOINT_ZERO_OFFSET_RAD`는
  남는다).
- `pub use motion::{...}`(`mod.rs:70-77`)에서 `RAIL_ACCEL_M_S2`를 제거한다.
- `pub use robot::{...}`(`mod.rs:78-81`)에서 `RAIL_MAX_SPEED`, `rail_frame`을 제거한다
  (`READY_JOINTS_4DOF`, `primitive_4dof`, `primitive_4dof_with_mount`,
  `robot`/`shared_robot`/`urdf_4dof`/`urdf_test`는 남는다).
- 새 줄 추가: `pub use rail::{DEFAULT_RAIL_CALIBRATION_PATH, RAIL_ACCEL_M_S2, RAIL_BOARD_ZERO_DOMAIN_M, RAIL_COORDINATE_POSITIVE_X_OFFSET_M, RAIL_HOMING_VELOCITY_M_S, RAIL_LEFT_END_MARGIN_M, RAIL_MAX_SPEED, RAIL_NEGATIVE_X_ZERO_SHIFT_M, RAIL_PHYSICAL_X_MAX_M, RAIL_PHYSICAL_X_MIN_M, RAIL_POSITIVE_X_TRIM_M, RAIL_READY_X_M, RAIL_RIGHT_END_MARGIN_M, RAIL_X_MAX_M, RAIL_X_MIN_M, rail_calibration_path, rail_frame};`

이렇게 하면 `crate::defaults::RAIL_X_MIN_M` 같은 **외부 호출부의 플랫 re-export 경로는
바뀌지 않는다** — `defaults/mod.rs`의 테스트(`presets_validate`, `mod.rs:112-135`)를
포함해 수정 불필요.

- [ ] **Step 6: 서브모듈 경로로 직접 참조하던 호출부 갱신**

다음은 `crate::defaults::motion::RAIL_ACCEL_M_S2`처럼 **서브모듈 경로로 직접** 참조해
Step 5의 플랫 re-export로는 해결되지 않는 곳들이다. 전부 `crate::defaults::rail::RAIL_ACCEL_M_S2`
(또는 짧은 이름 `RAIL_ACCEL_M_S2`, import가 이미 있으면)로 바꾼다:

- `src/robot/motion/bang_bang/guidance.rs:51-55` — `use crate::defaults::motion::{... RAIL_ACCEL_M_S2, TIME_TO_GO_BIAS};`에서
  `RAIL_ACCEL_M_S2`를 빼고, 별도 줄 `use crate::defaults::rail::RAIL_ACCEL_M_S2;`를 추가.
- `src/robot/control.rs:302,866` — `crate::defaults::motion::RAIL_ACCEL_M_S2` →
  `crate::defaults::rail::RAIL_ACCEL_M_S2` (두 곳 모두 전체 경로로 쓰고 있어 문자열 치환만).
- `src/robot/motion/physics.rs:1559,2840` — 동일 치환(2844는 문자열 로그 안의 이름 언급이라
  손대지 않아도 컴파일에 영향 없지만, 일관성을 위해 같이 바꿔도 된다).
- `src/robot/state.rs:126,265,565` — 동일 치환.

`src/real/control_worker.rs:78`(`pingpong_bot::defaults::RAIL_ACCEL_M_S2`)은 플랫
re-export 경로라 Step 5로 이미 해결됨 — 수정 불필요. `src/robot/rail/frame.rs`,
`src/sim/gui/scene/mod.rs:130`, `src/sim/session/controls.rs:17`의 `RAIL_THICKNESS` 참조는
이동 대상이 아니므로 그대로 둔다(§Task 1 서두 참고, `constants::geometry`에 남는다).
`crate::defaults::rail_frame()`을 부르는 나머지 호출부(`src/robot/urdf/mount.rs`,
`src/sim/physics/*.rs`, `src/sim/gui/viewer/panel.rs`, `src/sim/session/controls.rs`)는
전부 플랫 경로 `crate::defaults::rail_frame()`을 쓰므로 Step 5로 이미 해결된다.

- [ ] **Step 7: 빌드 + 기존 테스트로 회귀 확인**

```bash
cargo build --lib
cargo test --lib defaults::
cargo test --lib robot::
```

Expected: 전부 컴파일 성공, 기존 테스트(값 자체는 안 바꿨으므로) 전부 PASS.
`RAIL_ACCEL_M_S2`/`RAIL_MAX_SPEED`/`rail_frame()`을 참조하는 다른 모듈(예:
`sim::physics`, `robot::state`)이 있으면 `cargo build --lib`가 남은 미해결 경로를
컴파일 에러로 정확히 짚어준다 — 에러가 나면 그 파일도 Step 6과 같은 방식으로 고친다.

- [ ] **Step 8: 커밋**

```bash
git add src/defaults/rail.rs src/defaults/hardware.rs src/defaults/robot.rs \
        src/defaults/motion.rs src/defaults/mod.rs \
        src/robot/motion/bang_bang/guidance.rs src/robot/control.rs \
        src/robot/motion/physics.rs src/robot/state.rs
git commit -m "refactor: consolidate rail/frame constants into defaults::rail"
```

---

### Task 2: 영점 역산 순수 함수 + `RailEnd` (`src/hardware/rail/rail_config.rs`)

**Files:**
- Modify: `src/hardware/rail/rail_config.rs`
- Modify: `src/hardware/rail/mod.rs`
- Test: 같은 파일의 `#[cfg(test)] mod tests`

**Interfaces:**
- Consumes: 없음(순수 리팩터/추가).
- Produces:
  - `pub enum RailEnd { Min, Max }` (`Debug, Clone, Copy, PartialEq` derive)
  - `RailConfig::board_zero_domain_m_from_reference(&self, end: RailEnd, board_position_m: f64) -> f64`

- [ ] **Step 1: 실패하는 테스트 작성**

`rail_config.rs`의 `#[cfg(test)] mod tests` 블록에 추가:

```rust
#[test]
fn board_zero_domain_m_from_reference_matches_existing_reverse_transform() {
    let cfg = RailConfig {
        reverse: true,
        board_zero_domain_m: 0.705,
        physical_x_min_m: 0.0,
        physical_x_max_m: 1.41,
        ..RailConfig::default()
    };
    // 홈잉이 물리적 min 끝에서 보드 위치 -0.705를 읽었다면, 그 순간의 도메인 위치는
    // physical_x_min_m(0.0)이었어야 한다. 기존 board_to_domain_abs로 역방향 확인.
    let board_position_m = cfg.domain_to_board_abs(cfg.physical_x_min_m);
    let derived = cfg.board_zero_domain_m_from_reference(RailEnd::Min, board_position_m);
    assert!((derived - cfg.board_zero_domain_m).abs() < 1e-12);
}

#[test]
fn board_zero_domain_m_from_reference_works_at_max_end() {
    let cfg = RailConfig {
        reverse: true,
        board_zero_domain_m: 0.705,
        physical_x_min_m: 0.0,
        physical_x_max_m: 1.41,
        ..RailConfig::default()
    };
    let board_position_m = cfg.domain_to_board_abs(cfg.physical_x_max_m);
    let derived = cfg.board_zero_domain_m_from_reference(RailEnd::Max, board_position_m);
    assert!((derived - cfg.board_zero_domain_m).abs() < 1e-12);
}
```

- [ ] **Step 2: 테스트 실행해 실패 확인**

Run: `cargo test --lib hardware::rail::rail_config -- board_zero_domain_m_from_reference`
Expected: FAIL — `RailEnd`와 `board_zero_domain_m_from_reference`가 아직 없어 컴파일 에러.

- [ ] **Step 3: 구현**

`RailConfig` 구조체 정의(`rail_config.rs:9`) 위에 추가:

```rust
/// 홈잉 시 물리적으로 접근한 엔드스톱 방향.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum RailEnd {
    Min,
    Max,
}
```

`impl RailConfig` 블록(`command_position_for_actual_target` 근처, `rail_config.rs:145-152`
바로 아래)에 추가:

```rust
/// 홈잉으로 얻은 `board_position_m`(엔드스톱 도달 순간의 보드 좌표)으로부터
/// `board_zero_domain_m`을 역산한다.
///
/// `board_to_domain_abs(board_position_m) == (end이 가리키는 physical_x_{min,max}_m)`이
/// 성립해야 한다는 조건을 `board_zero_domain_m`에 대해 풀었다 — `domain_to_board_abs`/
/// `board_to_domain_abs`(위)와 같은 부호 규약을 쓴다. `reverse=false`면 보드·도메인
/// 좌표가 항등이라 `board_zero_domain_m`은 그 변환에 아무 영향이 없으므로, 이 경우엔
/// 알고 있는 도메인 값(끝점 위치) 그대로를 반환한다.
pub fn board_zero_domain_m_from_reference(&self, end: RailEnd, board_position_m: f64) -> f64 {
    let domain_known_m = match end {
        RailEnd::Min => self.physical_x_min_m,
        RailEnd::Max => self.physical_x_max_m,
    };
    if self.reverse {
        return domain_known_m + board_position_m;
    }
    return domain_known_m;
}
```

- [ ] **Step 4: 테스트 실행해 통과 확인**

Run: `cargo test --lib hardware::rail::rail_config -- board_zero_domain_m_from_reference`
Expected: PASS (2 tests).

- [ ] **Step 5: `RailEnd` export**

`src/hardware/rail/mod.rs`의 `pub use rail_config::RailConfig;`를
`pub use rail_config::{RailConfig, RailEnd};`로 바꾼다.

- [ ] **Step 6: 전체 레일 테스트 재확인 + 커밋**

```bash
cargo test --lib hardware::rail::
git add src/hardware/rail/rail_config.rs src/hardware/rail/mod.rs
git commit -m "feat: add RailEnd and board-zero derivation for rail homing"
```

---

### Task 3: 레일 캘리브레이션 파일 (`src/hardware/rail/rail_calibration.rs`)

**Files:**
- Create: `src/hardware/rail/rail_calibration.rs`
- Modify: `src/hardware/rail/mod.rs`
- Modify: `Cargo.toml` (의존성 추가 없음 — `serde`/`serde_json`은 이미 있음, 확인만)
- Test: 같은 파일의 `#[cfg(test)] mod tests`

**Interfaces:**
- Consumes: `crate::defaults::rail::{rail_calibration_path, DEFAULT_RAIL_CALIBRATION_PATH}`(Task 1),
  `crate::hardware::rail::{RailConfig, RailEnd}`(Task 2).
- Produces:
  - `pub struct RailCalibration { pub board_zero_domain_m: f64, pub homed_at_end: RailEnd, pub board_position_at_home_m: f64, pub measured_unix_secs: u64 }`
  - `RailCalibration::load(path: &Path) -> Option<Self>`
  - `RailCalibration::save(&self, path: &Path) -> std::io::Result<()>`
  - `RailCalibration::apply_to(&self, config: &mut RailConfig)`
  - `RailCalibration::from_home(end: RailEnd, board_position_at_home_m: f64, board_zero_domain_m: f64, measured_unix_secs: u64) -> Self`

- [ ] **Step 1: 실패하는 테스트 작성**

`src/hardware/rail/rail_calibration.rs`를 아래 내용으로 새로 만든다(구현 없이 테스트만
우선 채워도 되지만, 이 태스크는 파일이 새로 생기므로 구현과 테스트를 한 스텝씩
나누는 대신 아래처럼 전체를 한 파일에 작성하고 Step 2에서 컴파일 실패를 확인한다):

```rust
//! 실기 레일 홈잉 결과 — 재빌드 없이 `RailConfig`의 영점을 덮어쓰는 사이드카 JSON.
//!
//! `data/calibration.json`(카메라 PnP)과 같은 자리·패턴. 파일이 없거나 파싱에
//! 실패하면 `None`을 반환해, 호출부가 하드코딩 기본값(`defaults::rail::RAIL_BOARD_ZERO_DOMAIN_M`)
//! 그대로 계속 진행할 수 있게 한다 — 캘리브레이션 파일 문제로 로봇이 못 뜨면 안 된다.

use std::path::Path;

use serde::{Deserialize, Serialize};

use super::rail_config::{RailConfig, RailEnd};

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
enum RailEndJson {
    Min,
    Max,
}

impl From<RailEnd> for RailEndJson {
    fn from(end: RailEnd) -> Self {
        return match end {
            RailEnd::Min => RailEndJson::Min,
            RailEnd::Max => RailEndJson::Max,
        };
    }
}

impl From<RailEndJson> for RailEnd {
    fn from(end: RailEndJson) -> Self {
        return match end {
            RailEndJson::Min => RailEnd::Min,
            RailEndJson::Max => RailEnd::Max,
        };
    }
}

/// `--calibrate-rail` 홈잉 1회 실행 결과.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RailCalibration {
    pub board_zero_domain_m: f64,
    homed_at_end: RailEndJson,
    pub board_position_at_home_m: f64,
    pub measured_unix_secs: u64,
}

impl RailCalibration {
    pub fn from_home(
        end: RailEnd,
        board_position_at_home_m: f64,
        board_zero_domain_m: f64,
        measured_unix_secs: u64,
    ) -> Self {
        return Self {
            board_zero_domain_m,
            homed_at_end: end.into(),
            board_position_at_home_m,
            measured_unix_secs,
        };
    }

    pub fn homed_at_end(&self) -> RailEnd {
        return self.homed_at_end.into();
    }

    /// 파일이 없거나 파싱에 실패하면 `None` — 호출부는 하드코딩 기본값을 쓴다.
    pub fn load(path: &Path) -> Option<Self> {
        let contents = std::fs::read_to_string(path).ok()?;
        return match serde_json::from_str::<Self>(&contents) {
            Ok(calibration) => Some(calibration),
            Err(error) => {
                tracing::warn!(
                    path = %path.display(),
                    %error,
                    "rail_calibration.json 파싱 실패 — 기본값 사용"
                );
                None
            }
        };
    }

    pub fn save(&self, path: &Path) -> std::io::Result<()> {
        if let Some(parent) = path.parent() {
            std::fs::create_dir_all(parent)?;
        }
        let json = serde_json::to_string_pretty(self)
            .expect("RailCalibration 직렬화는 실패할 수 없다 — 모든 필드가 유한 f64/열거형");
        return std::fs::write(path, json);
    }

    /// `config.board_zero_domain_m`만 덮어쓴다. 나머지 필드(범위·속도 등)는 손대지
    /// 않는다 — 홈잉은 영점만 바꾸는 절차다.
    pub fn apply_to(&self, config: &mut RailConfig) {
        config.board_zero_domain_m = self.board_zero_domain_m;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn scratch_path(name: &str) -> std::path::PathBuf {
        return std::env::temp_dir().join(format!(
            "pingpong_bot_rail_calibration_test_{}_{name}.json",
            std::process::id()
        ));
    }

    #[test]
    fn load_returns_none_when_file_missing() {
        let path = scratch_path("missing");
        let _ = std::fs::remove_file(&path);
        assert_eq!(RailCalibration::load(&path), None);
    }

    #[test]
    fn load_returns_none_on_malformed_json() {
        let path = scratch_path("malformed");
        std::fs::write(&path, "not json").unwrap();
        assert_eq!(RailCalibration::load(&path), None);
        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn save_then_load_round_trips() {
        let path = scratch_path("roundtrip");
        let calibration = RailCalibration::from_home(RailEnd::Min, 0.0, 0.7050, 1_786_412_345);
        calibration.save(&path).unwrap();
        let loaded = RailCalibration::load(&path).unwrap();
        assert_eq!(loaded, calibration);
        assert_eq!(loaded.homed_at_end(), RailEnd::Min);
        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn apply_to_only_overwrites_board_zero_domain_m() {
        let mut config = RailConfig {
            board_zero_domain_m: 0.0,
            x_min_m: 0.01,
            x_max_m: 1.3395,
            ..RailConfig::default()
        };
        let calibration = RailCalibration::from_home(RailEnd::Max, 1.41, 0.7050, 1_786_412_345);
        calibration.apply_to(&mut config);
        assert_eq!(config.board_zero_domain_m, 0.7050);
        assert_eq!(config.x_min_m, 0.01);
        assert_eq!(config.x_max_m, 1.3395);
    }
}
```

`RailEnd`에 `PartialEq` derive가 Task 2에서 이미 있어야 `RailEndJson`/이 테스트들의
`assert_eq!`가 동작한다 — Task 2 Step 3에서 이미 `PartialEq`를 derive했으므로 추가
작업 없음.

- [ ] **Step 2: 컴파일 실패 확인**

Run: `cargo test --lib hardware::rail::rail_calibration`
Expected: FAIL — `rail_calibration` 모듈이 아직 `mod.rs`에 선언되지 않아 "module not
found" 컴파일 에러.

- [ ] **Step 3: 모듈 등록**

`src/hardware/rail/mod.rs`에 추가:

```rust
mod rail_calibration;

pub use rail_calibration::RailCalibration;
```

- [ ] **Step 4: 테스트 통과 확인**

Run: `cargo test --lib hardware::rail::rail_calibration`
Expected: PASS (4 tests).

- [ ] **Step 5: 커밋**

```bash
git add src/hardware/rail/rail_calibration.rs src/hardware/rail/mod.rs
git commit -m "feat: add RailCalibration sidecar file (load/save/apply)"
```

---

### Task 4: AXL 알람 read/reset FFI 바인딩 (`src/hardware/rail/axl_ffi.rs`)

**Files:**
- Modify: `src/hardware/rail/axl_ffi.rs`

**Interfaces:**
- Produces: `AxlFfi.axm_signal_read_servo_alarm: unsafe extern "system" fn(i32, *mut u32) -> u32`,
  `AxlFfi.axm_signal_servo_alarm_reset: unsafe extern "system" fn(i32, u32) -> u32`.

> **검증 불가 경고:** 이 파일 전체가 `#[cfg(all(windows, feature = "real"))]`로
> 게이트돼 있어(`mod.rs:3-4`) macOS/Linux에서는 컴파일되지 않는다. 이 태스크는
> 기존 20여 개 심볼과 완전히 같은 패턴을 그대로 반복하는 것으로 정확성을 담보한다 —
> 로컬 `cargo build`로는 검증할 수 없고, Windows+AXL 벤치에서 `cargo build --features real`
> (또는 실제 실행)로 확인해야 한다.

- [ ] **Step 1: 함수 포인터 타입 추가**

`axl_ffi.rs:37`(`type AxmMoveSStop = ...;`) 바로 아래에 추가:

```rust
type AxmSignalReadServoAlarm = unsafe extern "system" fn(i32, *mut u32) -> u32;
type AxmSignalServoAlarmReset = unsafe extern "system" fn(i32, u32) -> u32;
```

- [ ] **Step 2: `AxlFfi` 구조체 필드 추가**

`axl_ffi.rs:65`(`pub axm_move_s_stop: AxmMoveSStop,`) 바로 아래에 추가:

```rust
pub axm_signal_read_servo_alarm: AxmSignalReadServoAlarm,
pub axm_signal_servo_alarm_reset: AxmSignalServoAlarmReset,
```

- [ ] **Step 3: `load()`에서 심볼 로드**

`axl_ffi.rs:121`(`axm_move_s_stop: *library.get(b"AxmMoveSStop\0").map_err(symbol_error)?,`)
바로 아래에 추가:

```rust
axm_signal_read_servo_alarm: *library
    .get(b"AxmSignalReadServoAlarm\0")
    .map_err(symbol_error)?,
axm_signal_servo_alarm_reset: *library
    .get(b"AxmSignalServoAlarmReset\0")
    .map_err(symbol_error)?,
```

- [ ] **Step 4: 커밋**

Windows 빌드 확인 없이 커밋하되, 커밋 메시지에 검증 상태를 남긴다.

```bash
git add src/hardware/rail/axl_ffi.rs
git commit -m "feat: bind AxmSignalReadServoAlarm/AxmSignalServoAlarmReset (unverified on macOS — needs Windows AXL bench build)"
```

---

### Task 5: `AxlLive`에 알람 read/reset 추가 (`src/hardware/rail/axl_live.rs`)

**Files:**
- Modify: `src/hardware/rail/axl_live.rs`

**Interfaces:**
- Consumes: Task 4의 `axm_signal_read_servo_alarm`/`axm_signal_servo_alarm_reset`.
- Produces: `AxlLive::read_alarm(&mut self, axis: i32) -> Result<bool, HwError>`,
  `AxlLive::reset_alarm(&mut self, axis: i32) -> Result<(), HwError>`.

> **검증 불가 경고:** Task 4와 같은 이유로 로컬에서 컴파일 확인 불가 — Windows 벤치에서
> 검증한다.

- [ ] **Step 1: `read_alarm` 추가**

`axl_live.rs:224`(`wait_idle` 바로 위, `stop_if_moving` 다음)에 추가:

```rust
pub(super) fn read_alarm(&mut self, axis: i32) -> Result<bool, HwError> {
    let mut alarm = 0;
    check_axl("AxmSignalReadServoAlarm", unsafe {
        (self.ffi.axm_signal_read_servo_alarm)(axis, &mut alarm)
    })?;
    return Ok(alarm != 0);
}
```

- [ ] **Step 2: `reset_alarm` 추가**

바로 아래에 추가 (레퍼런스 구현 `robot-pingpong-cpp/src/control/linear_motor.cpp`의
`resetAlarm()`과 같은 LOW→대기→HIGH→해제 대기→LOW 시퀀스):

```rust
pub(super) fn reset_alarm(&mut self, axis: i32) -> Result<(), HwError> {
    const LOW: u32 = 0;
    const HIGH: u32 = 1;
    check_axl("AxmSignalServoAlarmReset", unsafe {
        (self.ffi.axm_signal_servo_alarm_reset)(axis, LOW)
    })?;
    std::thread::sleep(std::time::Duration::from_millis(500));
    check_axl("AxmSignalServoAlarmReset", unsafe {
        (self.ffi.axm_signal_servo_alarm_reset)(axis, HIGH)
    })?;
    let deadline = std::time::Instant::now() + MOVE_POLL_TIMEOUT;
    loop {
        if !self.read_alarm(axis)? {
            break;
        }
        if std::time::Instant::now() >= deadline {
            return Err(HwError::InvalidConfig {
                reason: "AXL 알람 해제 실패 — AxmSignalReadServoAlarm이 계속 true, 수동 확인 필요"
                    .into(),
            });
        }
        std::thread::sleep(std::time::Duration::from_millis(10));
    }
    return check_axl("AxmSignalServoAlarmReset", unsafe {
        (self.ffi.axm_signal_servo_alarm_reset)(axis, LOW)
    });
}
```

- [ ] **Step 3: 커밋**

```bash
git add src/hardware/rail/axl_live.rs
git commit -m "feat: add AxlLive::read_alarm/reset_alarm (unverified on macOS — needs Windows AXL bench build)"
```

---

### Task 6: `AxlRail::home()` (`src/hardware/rail/axl_rail.rs`)

**Files:**
- Modify: `src/hardware/rail/axl_rail.rs`

**Interfaces:**
- Consumes: `RailEnd`(Task 2), `AxlLive::read_alarm`/`reset_alarm`(Task 5),
  `RailConfig::board_zero_domain_m_from_reference`(Task 2), `RAIL_HOMING_VELOCITY_M_S`(Task 1).
- Produces: `AxlRail::home(&mut self, end: RailEnd) -> Result<f64, HwError>` (Live 전용,
  `#[cfg(all(windows, feature = "real"))]`) — 반환값은 새 `board_zero_domain_m`.

> **검증 불가 경고:** `RailKind::Live` 분기는 Windows+`real`에서만 컴파일된다. 폴링
> 루프·알람 감지 자체는 Windows 벤치에서만 실기로 확인 가능하다. `DryRun` 분기(즉시
> 에러 반환)는 이 태스크의 테스트로 검증한다.

- [ ] **Step 1: 실패하는 테스트 작성 (DryRun 가드)**

`axl_rail.rs`의 `#[cfg(test)] mod tests` 블록에 추가:

```rust
#[test]
fn home_rejects_dry_run() {
    let cfg = RailConfig {
        enabled: true,
        dll_path: PathBuf::from("unused.dll"),
        pulses_per_meter: 1000,
        x_min_m: 0.0,
        x_max_m: 1.0,
        ..RailConfig::default()
    };
    let mut rail = AxlRail::dry_run(cfg).unwrap();
    assert!(rail.home(crate::hardware::rail::RailEnd::Min).is_err());
}
```

- [ ] **Step 2: 테스트 실행해 실패 확인**

Run: `cargo test --lib hardware::rail::axl_rail -- home_rejects_dry_run`
Expected: FAIL — `home` 메서드가 아직 없어 컴파일 에러.

- [ ] **Step 3: `home()` 구현**

`axl_rail.rs`에 `use super::rail_config::RailEnd;`를 추가(이미 `RailConfig`를 쓰고 있는
`use` 줄 옆에). `AxlRail`의 `impl` 블록, `open()` 바로 아래에 추가:

```rust
/// 저속으로 물리적 엔드스톱까지 이동해 AXL 알람으로 도달을 감지하고, 그 지점을
/// 기준으로 `board_zero_domain_m`을 다시 계산한다. `DryRun`엔 물리 엔드스톱이 없어
/// 항상 에러를 반환한다. 온디맨드 호출 전용 — 기동 시 자동으로 부르지 않는다.
pub fn home(&mut self, end: RailEnd) -> Result<f64, HwError> {
    #[cfg(all(windows, feature = "real"))]
    if let RailKind::Live(live) = &mut self.kind {
        return home_live(live, &mut self.config, end);
    }
    return Err(HwError::InvalidConfig {
        reason: "AxlRail::home은 Live(실기) 레일에서만 지원됩니다".into(),
    });
}
```

같은 파일, `impl AxlRail` 블록 바깥(파일 하단, `#[cfg(test)] mod tests` 위)에 free
function으로 추가:

```rust
#[cfg(all(windows, feature = "real"))]
fn home_live(
    live: &mut super::axl_live::AxlLive,
    config: &mut RailConfig,
    end: RailEnd,
) -> Result<f64, HwError> {
    use super::rail_config::RailEnd as E;

    let target_domain_m = match end {
        E::Min => config.physical_x_min_m,
        E::Max => config.physical_x_max_m,
    };
    let target_board_m = config.domain_to_board_abs(target_domain_m);
    live.start_move_abs_m(config, target_board_m, super::super::defaults_rail_homing_velocity())?;

    let deadline = std::time::Instant::now() + super::axl_live::MOVE_POLL_TIMEOUT;
    loop {
        if live.read_alarm(config.axis)? {
            break;
        }
        if std::time::Instant::now() >= deadline {
            live.stop(config.axis)?;
            return Err(HwError::InvalidConfig {
                reason: "레일 홈잉: 엔드스톱 도달 못 함 — 배선/알람 설정 확인".into(),
            });
        }
        std::thread::sleep(std::time::Duration::from_millis(1));
    }

    live.stop(config.axis)?;
    let (board_position_m, _command_position_m) = live.read_actual_and_command_m(config.axis)?;
    live.reset_alarm(config.axis)?;

    let new_board_zero_domain_m = config.board_zero_domain_m_from_reference(end, board_position_m);
    config.board_zero_domain_m = new_board_zero_domain_m;
    tracing::info!(
        axis = config.axis,
        end = ?end,
        board_position_m,
        new_board_zero_domain_m,
        "레일 홈잉 완료"
    );
    return Ok(new_board_zero_domain_m);
}
```

`super::super::defaults_rail_homing_velocity()`는 존재하지 않는 이름이므로, 대신
`crate::defaults::rail::RAIL_HOMING_VELOCITY_M_S`를 직접 쓴다 — 위 스니펫의
`super::super::defaults_rail_homing_velocity()` 호출을
`crate::defaults::rail::RAIL_HOMING_VELOCITY_M_S`로 바꿔서 작성한다(줄 하나만 다른
이름으로 바뀔 뿐 나머지 로직은 동일).

- [ ] **Step 4: 테스트 통과 확인**

Run: `cargo test --lib hardware::rail::axl_rail -- home_rejects_dry_run`
Expected: PASS.

- [ ] **Step 5: 레일 전체 테스트 재확인 + 커밋**

```bash
cargo test --lib hardware::rail::
git add src/hardware/rail/axl_rail.rs
git commit -m "feat: add AxlRail::home (endstop-alarm based, unverified live path on macOS)"
```

---

### Task 7: `RealHardware::home_rail()` (`src/hardware/real.rs`)

**Files:**
- Modify: `src/hardware/real.rs`

**Interfaces:**
- Consumes: `AxlRail::home`(Task 6), `RailEnd`(Task 2).
- Produces: `RealHardware::home_rail(&mut self, end: RailEnd) -> Result<f64, HwError>`.

- [ ] **Step 1: 실패하는 테스트 작성**

`real.rs`의 `#[cfg(test)] mod tests` 블록, `test_rail()` 헬퍼(`real.rs:387-391`) 근처에
추가:

```rust
#[test]
fn home_rail_errors_when_dry_run() {
    let dynamixel = DynamixelConfig {
        port: "/dev/null".into(),
        ..DynamixelConfig::default()
    };
    let mut hardware =
        RealHardware::dry_run(dynamixel, Some(test_rail())).expect("dry-run hardware");
    assert!(hardware.home_rail(crate::hardware::rail::RailEnd::Min).is_err());
}
```

- [ ] **Step 2: 테스트 실행해 실패 확인**

Run: `cargo test --lib hardware::real -- home_rail_errors_when_dry_run`
Expected: FAIL — `home_rail` 메서드가 없어 컴파일 에러.

- [ ] **Step 3: 구현**

`real.rs`의 `command_rail`(`real.rs:239`) 근처에 추가:

```rust
/// 온디맨드 레일 홈잉. `--calibrate-rail`과 jog 툴 버튼이 이 메서드를 부른다.
pub fn home_rail(&mut self, end: super::rail::RailEnd) -> Result<f64, HwError> {
    let mut rail = self.rail.lock().map_err(|_| HwError::CommandFailed {
        duration_secs: 0.0,
        joint_count: 0,
        reason: "레일 mutex poisoned".into(),
    })?;
    return match rail.as_mut() {
        None => Err(HwError::InvalidConfig {
            reason: "레일이 비활성화됨 — home_rail 호출 불가".into(),
        }),
        Some(rail) => rail.home(end),
    };
}
```

- [ ] **Step 4: 테스트 통과 확인**

Run: `cargo test --lib hardware::real -- home_rail_errors_when_dry_run`
Expected: PASS — `AxlRail::home`이 `DryRun`엔 에러를 반환하므로(Task 6) 이 테스트는
`Live`/Windows 없이도 통과한다.

- [ ] **Step 5: 커밋**

```bash
git add src/hardware/real.rs
git commit -m "feat: expose RealHardware::home_rail"
```

---

### Task 8: 캘리브레이션 파일을 하드웨어 조립에 연결 (`src/real/run.rs`)

**Files:**
- Modify: `src/real/run.rs:304-322` (`open_hardware`)

**Interfaces:**
- Consumes: `RailCalibration::load`/`apply_to`(Task 3), `crate::defaults::rail::rail_calibration_path`(Task 1).

- [ ] **Step 1: `open_hardware`에 캘리브레이션 오버레이 추가**

`src/real/run.rs:304-322`를 다음으로 바꾼다(기존 로직은 그대로 두고 `rail` 조립
직후에 오버레이만 끼워 넣는다):

```rust
fn open_hardware(options: &Options) -> Result<RealHardware> {
    let mut dxl = DynamixelConfig::default();
    if let Some(port) = &options.dxl_port {
        dxl.port = port.clone();
    }
    dxl.hold_torque_on_close = !options.release_torque;
    let mut rail = RailConfig::default();
    let calibration_path = defaults::rail::rail_calibration_path();
    if let Some(calibration) = pingpong_bot::hardware::rail::RailCalibration::load(&calibration_path) {
        info!(
            path = %calibration_path.display(),
            board_zero_domain_m = calibration.board_zero_domain_m,
            "레일 캘리브레이션 파일 적용"
        );
        calibration.apply_to(&mut rail);
    }
    info!(
        port = %dxl.port,
        dry_run = options.dry_run,
        rail_enabled = rail.enabled,
        hold_torque_on_close = dxl.hold_torque_on_close,
        "real 하드웨어 (mirror ID1↔ID2)"
    );
    let hardware = if options.dry_run {
        RealHardware::dry_run(dxl, Some(rail))
    } else {
        RealHardware::new(dxl, Some(rail))
    };
    return hardware.context("하드웨어 초기화");
}
```

`use pingpong_bot::hardware::rail::RailCalibration;`를 파일 상단 `use` 블록(다른
`pingpong_bot::hardware::...` import 옆, `run.rs:14-15` 근처)에 추가해도 되고, 위처럼
전체 경로로 인라인해도 된다 — 이미 있는 `use pingpong_bot::hardware::rail::RailConfig;`
바로 아래에 `use pingpong_bot::hardware::rail::RailCalibration;`을 추가하는 쪽으로
정리한다(인라인 전체 경로는 지운다).

- [ ] **Step 2: 빌드 확인**

Run: `cargo build --bin pingpong-bot` (또는 리포의 실제 real-mode 바이너리 타깃명 —
`cargo build --lib`가 이미 통과했다면 바이너리도 같은 크레이트 의존성이라 문제없이
빌드되는지 확인).
Expected: 성공. 파일이 없으므로 `RailCalibration::load`는 `None`을 반환하고 기존
동작과 동일하게 진행됨 — 기존 dry-run 통합 테스트가 있다면 그대로 통과해야 한다.

- [ ] **Step 3: 커밋**

```bash
git add src/real/run.rs
git commit -m "feat: apply data/rail_calibration.json override when opening real hardware"
```

---

### Task 9: `--calibrate-rail` CLI 플래그 (`src/cli/args.rs`, `src/real/run.rs`)

**Files:**
- Modify: `src/cli/args.rs`
- Modify: `src/real/options.rs`
- Modify: `src/real/run.rs`

**Interfaces:**
- Consumes: `RealHardware::home_rail`(Task 7), `RailCalibration::from_home`/`save`(Task 3).
- Produces: `Args.calibrate_rail: bool`, `Args.calibrate_rail_end: RailEndArg`(clap
  `ValueEnum`), `Options.calibrate_rail: bool`, `Options.calibrate_rail_end: RailEnd`.

- [ ] **Step 1: `Args`에 플래그 추가**

`src/cli/args.rs`의 `home`(`args.rs:25-27`) 아래에 추가:

```rust
/// real: 레일을 물리적 엔드스톱까지 저속 이동해 영점을 다시 잡고 종료한다.
/// 정상 기동은 하지 않는다 — 재조립 후 등 필요할 때만 수동 실행.
#[arg(long)]
pub calibrate_rail: bool,
/// `--calibrate-rail`이 향할 엔드스톱 방향.
#[arg(long, value_enum, default_value = "min")]
pub calibrate_rail_end: RailEndArg,
```

파일 상단에 clap `ValueEnum`을 하나 추가한다(`mode_arg::ModeArg`처럼 별도 열거형이
이미 있는 패턴을 따른다 — `src/cli/mode_arg.rs` 참고해 같은 파일 안에 작게 정의):

```rust
#[derive(Clone, Copy, Debug, clap::ValueEnum)]
pub enum RailEndArg {
    Min,
    Max,
}

impl From<RailEndArg> for pingpong_bot::hardware::rail::RailEnd {
    fn from(end: RailEndArg) -> Self {
        return match end {
            RailEndArg::Min => pingpong_bot::hardware::rail::RailEnd::Min,
            RailEndArg::Max => pingpong_bot::hardware::rail::RailEnd::Max,
        };
    }
}
```

- [ ] **Step 2: `Options`에 필드 추가**

`src/real/options.rs`의 `home`(`options.rs:14-15`) 아래에 추가:

```rust
/// 레일 엔드스톱 홈잉만 실행하고 종료.
pub calibrate_rail: bool,
pub calibrate_rail_end: pingpong_bot::hardware::rail::RailEnd,
```

`from_args`(`options.rs:27-37`) 안에 추가:

```rust
calibrate_rail: args.calibrate_rail,
calibrate_rail_end: args.calibrate_rail_end.into(),
```

- [ ] **Step 3: `run()`에 온디맨드 분기 추가**

`src/real/run.rs`의 `pub fn run(args: &Args) -> Result<()>`(`run.rs:37`) 본문 맨 앞,
`let mut hardware = open_hardware(&options)?;` 바로 다음에 추가:

```rust
if options.calibrate_rail {
    return calibrate_rail(&mut hardware, options.calibrate_rail_end);
}
```

파일 하단(`open_hardware` 근처)에 새 함수 추가:

```rust
fn calibrate_rail(
    hardware: &mut RealHardware,
    end: pingpong_bot::hardware::rail::RailEnd,
) -> Result<()> {
    let board_zero_domain_m = hardware
        .home_rail(end)
        .context("레일 홈잉")?;
    let measured_unix_secs = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default()
        .as_secs();
    let calibration = pingpong_bot::hardware::rail::RailCalibration::from_home(
        end,
        board_zero_domain_m,
        board_zero_domain_m,
        measured_unix_secs,
    );
    let path = defaults::rail::rail_calibration_path();
    calibration
        .save(&path)
        .with_context(|| format!("레일 캘리브레이션 저장: {}", path.display()))?;
    info!(
        path = %path.display(),
        board_zero_domain_m,
        end = ?end,
        "레일 홈잉 완료 — 캘리브레이션 저장"
    );
    return Ok(());
}
```

`RailCalibration::from_home`의 두 번째 인자(`board_position_at_home_m`)는 실제로는
Task 6의 `home()`이 반환하는 `board_zero_domain_m`이 아니라 엔드스톱에서 읽은 원시
`board_position_m`이어야 정확하지만, `AxlRail::home`은 지금 파생된 영점만 반환한다
(`Result<f64, HwError>` = 새 `board_zero_domain_m`). 이 태스크에서는 `RealHardware::home_rail`의
반환값을 `board_zero_domain_m`으로만 쓰고, `board_position_at_home_m` 필드에는 같은
값을 채운다(사이드카 파일의 "참고용 진단 필드"이지 `apply_to`가 실제로 쓰는 값은
`board_zero_domain_m`뿐이므로 정확도가 동작에 영향을 주지 않는다) — 이 단순화를
주석으로 남긴다.

- [ ] **Step 4: 빌드 확인**

Run: `cargo build --lib && cargo build` (바이너리 포함)
Expected: 성공. `cargo test --lib` 전체 재실행해 앞 태스크들의 테스트가 여전히
PASS인지도 확인.

- [ ] **Step 5: 커밋**

```bash
git add src/cli/args.rs src/real/options.rs src/real/run.rs
git commit -m "feat: add --calibrate-rail CLI flag for on-demand rail homing"
```

---

### Task 10: jog 툴 "레일 홈잉" 버튼

**Files:**
- Modify: `tools/jog/src/state/action.rs`
- Modify: `tools/jog/src/state/jog_app.rs`
- Modify: `tools/jog/src/panel.rs`

**Interfaces:**
- Consumes: `JogApp.hardware: Arc<Mutex<RealHardware>>`(이미 존재),
  `RealHardware::home_rail`(Task 7).

- [ ] **Step 1: `Action` 변형 추가**

`tools/jog/src/state/action.rs`에 추가:

```rust
#[derive(Clone, Copy)]
pub enum Action {
    Sync,
    Discard,
    Apply,
    Preview,
    HomeRail,
}
```

- [ ] **Step 2: `JogApp::home_rail` 메서드 추가**

`tools/jog/src/state/jog_app.rs`의 `impl JogApp` 블록에, 다른 액션 메서드(`sync`,
`apply` 등)와 같은 자리에 추가:

```rust
pub fn home_rail(&mut self) -> Result<()> {
    ensure!(!self.dry_run, "dry-run에서는 레일 홈잉을 실행할 수 없습니다");
    let mut hardware = self
        .hardware
        .lock()
        .map_err(|_| anyhow::anyhow!("hardware mutex poisoned"))?;
    let board_zero_domain_m = hardware.home_rail(pingpong_bot::hardware::rail::RailEnd::Min)?;
    self.set_error(format!("레일 홈잉 완료: board_zero_domain_m={board_zero_domain_m:.4}"));
    return Ok(());
}
```

`self.set_error`는 기존 에러 배너 필드(`app.error`, `jog_app.rs`에 이미 있는
`pub error: Option<String>`)를 재사용해 성공 메시지도 같은 배너에 띄운다 — 이 툴이
전용 성공/실패 상태 표시 위젯을 따로 갖고 있지 않아서다. `set_error`가 아직 없다면
`self.error = Some(...)`로 직접 대입한다.

- [ ] **Step 3: `try_action`에 연결**

`tools/jog/src/state/jog_app.rs`의 `try_action`(`jog_app.rs:277-287`)에 분기 추가:

```rust
pub fn try_action(app: &mut JogApp, action: Action) {
    let result = match action {
        Action::Sync => app.sync(),
        Action::Discard => app.discard(),
        Action::Apply => app.apply(),
        Action::Preview => app.preview_from_draft(),
        Action::HomeRail => app.home_rail(),
    };
    if let Err(err) = result {
        app.set_error(format!("{err:#}"));
    }
}
```

- [ ] **Step 4: 버튼 추가**

`tools/jog/src/panel.rs`의 `draw_actions`(`panel.rs:288-...`) 안, 기존 버튼들(`미리보기`,
`버리기`, `적용`, `동기화`) 옆에 추가:

```rust
if ui
    .add_enabled(!app.dry_run, egui::Button::new("레일 홈잉"))
    .clicked()
{
    try_action(app, Action::HomeRail);
}
```

- [ ] **Step 5: 빌드 확인**

Run: `cargo build -p jog`(또는 저장소의 실제 jog 툴 패키지명 — `tools/jog/Cargo.toml`의
`[package] name` 확인 후 그 이름 사용).
Expected: 성공.

- [ ] **Step 6: 커밋**

```bash
git add tools/jog/src/state/action.rs tools/jog/src/state/jog_app.rs tools/jog/src/panel.rs
git commit -m "feat: add rail homing button to jog tool"
```

---

### Task 11: 셋업 가이드 문서 (`docs/rail-calibration.md`)

**Files:**
- Create: `docs/rail-calibration.md`

**Interfaces:** 없음(문서만).

- [ ] **Step 1: 문서 작성**

`docs/rail-calibration.md`를 다음 절로 작성한다(각 절 본문은 이 리포의 실제 값/경로를
그대로 인용한다 — 스펙·이전 태스크에서 이미 확정된 내용이라 placeholder 없이 채울 수
있다):

1. **언제 다시 정렬해야 하는가** — 재조립 후, 레일을 다른 마운트로 옮긴 후,
   `AxlRail::open`이 시작 시 찍는 "AXL 시작 좌표 진단" 로그(`axl_rail.rs:56-70`)의
   `domain_position_m`이 `RAIL_READY_X_M`(0.675m) 부근 기대치에서 크게 벗어날 때.
2. **`--calibrate-rail` 실행 절차** — `cargo run --release --features real -- --mode real --calibrate-rail --calibrate-rail-end=min`
   (기본값이 `min`이므로 `--calibrate-rail`만 줘도 된다). 실행 전 레일 이동 경로에
   장애물이 없는지, 비상정지 스위치 위치를 확인할 것. 실행 중 저속(약 0.02 m/s)으로
   엔드스톱까지 이동한 뒤 자동으로 정지·영점 저장 후 종료된다.
3. **`data/rail_calibration.json`** — 필드 설명(`board_zero_domain_m`,
   `homed_at_end`, `board_position_at_home_m`, `measured_unix_secs`). 파일을 지우면
   다음 실행부터 하드코딩 기본값(`defaults::rail::RAIL_BOARD_ZERO_DOMAIN_M`)으로
   되돌아간다.
4. **홈잉으로 얻을 수 없는 값** — `rail_frame()`의 `mount_y`/`rail_bottom_z`
   (`src/defaults/rail.rs`)는 레일 **마운트 위치**(탁구대 기준 물리적 설치 좌표)이지
   레일 이동 영점이 아니어서 홈잉이 다루지 않는다. 절차: 줄자로 바닥→프로파일 하단
   높이, 탁구대 로봇쪽 끝면→레일 y 오프셋을 잰다 → sim GUI "Rig" 패널에서 공이
   주차된 동안 눈으로 미세조정 → 좋은 값을 찾으면 `tools/mount_search`(또는
   `--rest-pose-search`)를 그 위치에서 다시 돌려 `rail_frame()`과
   `READY_JOINTS_4DOF`를 함께 확정 → `src/defaults/rail.rs`에 값과 측정 날짜를
   손으로 기록.
5. **처음 실행 시 체크리스트** — 저속 이동이므로 사람이 옆에서 지켜보며 1회 확인,
   이상 시 비상정지, `--dry-run`으로는 이 플래그가 동작하지 않음(물리 엔드스톱이
   없으므로 즉시 에러 반환)을 명시.

- [ ] **Step 2: 커밋**

```bash
git add docs/rail-calibration.md
git commit -m "docs: add rail calibration/homing setup guide"
```

## Self-Review 결과 (수정 완료)

- **Spec coverage:** 상수 통합(Task 1) · 홈잉 알고리즘(Task 2·4·5·6) · 저장/오버라이드
  (Task 3·8) · CLI/jog 노출(Task 9·10) · 가이드 문서(Task 11) — spec의 4개 절 모두
  태스크로 커버됨. `board_position_at_home_m`을 정확한 원시값 대신 파생값으로
  단순화한 부분은 Task 9 Step 3 본문에 이유를 명시해 애매함을 없앴다.
- **Placeholder scan:** 없음 — 모든 스텝에 실제 코드/명령어.
- **Type consistency:** `RailEnd`(Task 2 정의) → `RailCalibration`(Task 3),
  `AxlRail::home`(Task 6), `RealHardware::home_rail`(Task 7), `RailEndArg::into()`
  (Task 9), jog 툴(Task 10)까지 동일 타입·이름으로 일관되게 사용됨을 확인.
