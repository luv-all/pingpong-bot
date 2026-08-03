# jog 슈터 기반 스윙 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** jog의 스윙 커맨드 입력을 "도달점 + 입사속도"에서 "슈터 파라미터"로 바꿔, 물리적으로 불가능한 조합이 입력될 수 없게 한다.

**Architecture:** 슈터 기하를 발사구=회전 피벗으로 재정의해 조준각과 무관하게 발사 위치를 고정한다. jog는 `launch::Settings` → `Kinematics::predict_to`로 도달점·입사속도를 얻어 기존 임팩트 역산(`rally_return` → `required_racket_velocity` → `velocities_for_racket_velocity`)에 넣는다. 슈터 패널 위젯과 3D Visual은 공용 모듈로 추출해 메인 sim과 jog가 공유한다.

**Tech Stack:** Rust 2024, nalgebra, rapier3d(glam `Vector`/`Rotation`), kiss3d + egui, anyhow

## Global Constraints

- 설계 문서: `docs/superpowers/specs/2026-07-30-jog-shooter-swing-design.md` (아직 커밋 안 됨 — Task 1 커밋에 함께 넣는다)
- 코드베이스 규약: 함수는 명시적 `return`으로 끝낸다. 주석·문서는 한국어.
- 커밋 메시지: 제목은 영어, 본문은 한국어. co-author 트레일러 금지.
- 브랜치: `main`에 직접 커밋한다 (별도 브랜치 없음).
- 실측 슈터 발사구 좌표(SSOT): `x = table::WIDTH_X * 0.5`, `y = table::LENGTH_Y - 0.275`, `z = table::SURFACE_Z + 0.225`. 기본 `pitch_deg = 15.0`, `speed_mps = 7.5`.
- 빌드/테스트: `cargo test` (기본 feature = `gui`). jog는 `cargo test -p jog`.
- 커밋 시 1Password SSH 에이전트가 잠겨 있으면 `error: 1Password: agent returned an error`가 난다. 그때는 에이전트 잠금 해제를 요청하고, 서명을 끄지 않는다.

---

### Task 1: 슈터 기하·기본값을 실측값으로 재정의

**Files:**
- Modify: `src/sim/launch/layout.rs:8-21`
- Modify: `src/defaults/sim.rs:60-79`
- Test: `src/sim/launch/settings.rs:404-585` (기존 `mod tests`)

**Interfaces:**
- Consumes: 없음 (첫 태스크)
- Produces: `launch::Layout::{MOUNT_X, MOUNT_Y, BODY_HEIGHT, BARREL_FORWARD_M, VISUAL_SIZE_X, VISUAL_SIZE_Y, VISUAL_SIZE_Z}` (모두 `f64` 연관 상수), `launch::Settings::default()`가 발사구를 실측 좌표에 고정. `Settings::muzzle_position() -> rapier3d::prelude::Vector`(glam `Vec3`, f32)는 시그니처 변경 없음.

- [ ] **Step 1: 실패하는 테스트를 쓴다**

`src/sim/launch/settings.rs`의 `mod tests` 안에 추가한다. 파일 상단 `use` 는 이미 `super::*`, `rand::SeedableRng`가 있고, `Vector3`·`defaults`·`HitPlane`·`estimator`는 파일 본문에서 이미 import되어 있으므로 테스트에서 그대로 쓸 수 있다.

```rust
    /// 실측 슈터: 발사구가 (W/2, L−0.275, 면+0.225)에 있다.
    #[test]
    fn default_muzzle_matches_measured_mount() {
        let m = Settings::default().muzzle_position();
        assert!((m.x - (table::WIDTH_X * 0.5) as f32).abs() < 1e-5, "x={}", m.x);
        assert!((m.y - (table::LENGTH_Y - 0.275) as f32).abs() < 1e-5, "y={}", m.y);
        assert!(
            (m.z - (table::SURFACE_Z + 0.225) as f32).abs() < 1e-5,
            "z={}",
            m.z
        );
    }

    /// 발사구가 조준 회전의 피벗이므로 yaw/pitch/roll을 바꿔도 안 움직인다.
    #[test]
    fn default_muzzle_is_independent_of_aim() {
        let base = Settings::default().muzzle_position();
        for (yaw, pitch, roll) in [
            (0.0, 0.0, 0.0),
            (20.0, -25.0, 30.0),
            (-15.0, 25.0, -40.0),
        ] {
            let s = Settings {
                yaw_deg: yaw,
                pitch_deg: pitch,
                roll_deg: roll,
                ..Default::default()
            };
            let m = s.muzzle_position();
            assert!(
                (m - base).length() < 1e-5,
                "yaw={yaw} pitch={pitch} roll={roll} -> {m:?}"
            );
        }
    }

    /// 기본 샷이 접수 평면에 라켓이 들어갈 만한 높이로 도달한다.
    #[test]
    fn default_shot_reaches_hit_plane_high_enough() {
        let s = Settings::default();
        let muzzle = s.muzzle_position();
        let vel = s.launch_velocity();
        let omega = s.launch_angular_velocity();
        let pred = estimator::Kinematics::predict_to(
            Vector3::new(f64::from(muzzle.x), f64::from(muzzle.y), f64::from(muzzle.z)),
            Vector3::new(f64::from(vel.x), f64::from(vel.y), f64::from(vel.z)),
            Vector3::new(f64::from(omega.x), f64::from(omega.y), f64::from(omega.z)),
            HitPlane {
                y: table::DEFAULT_HIT_PLANE_Y,
            },
            &defaults::PhysicsParams::default(),
        )
        .expect("기본 샷은 접수 평면에 도달해야 한다");
        let above = pred.impact_position.coords.z - table::SURFACE_Z;
        assert!(
            above > 0.15,
            "도달 높이 면+{above:.3} m — 라켓이 들어갈 수 없다"
        );
    }
```

같은 `mod tests`에서 **기존 테스트 2개를 새 기하에 맞게 교체**한다.

`visual_body_sits_outside_table_end`를 통째로 다음으로 바꾼다 (본체가 이제 테이블 위에 얹히므로 "테이블 끝 밖" 단언은 더 이상 성립하지 않는다):

```rust
    #[test]
    fn visual_body_clears_table_surface() {
        let s = Settings::default();
        let visual = s.visual_position();
        let bottom = visual.z - (layout::Layout::VISUAL_SIZE_Z * 0.5) as f32;
        assert!(
            bottom > table::SURFACE_Z as f32,
            "본체 바닥 z={bottom} 이 테이블 면 {} 아래",
            table::SURFACE_Z
        );
    }
```

`default_aims_toward_robot_with_slight_drop`을 통째로 다음으로 바꾼다:

```rust
    #[test]
    fn default_aims_toward_robot_and_upward() {
        let dir = Settings::default().aim_direction();
        assert!(dir.y < 0.0, "로봇 쪽(−y)을 봐야 한다: {dir:?}");
        assert!(dir.z > 0.0, "기본 pitch는 +15°(위)다: {dir:?}");
        assert!(dir.x.abs() < 1e-5, "좌우 치우침 없음: {dir:?}");
    }
```

- [ ] **Step 2: 테스트가 실패하는지 확인한다**

Run: `cargo test -p pingpong-bot --lib sim::launch::settings 2>&1 | tail -30`
Expected: FAIL — `default_muzzle_matches_measured_mount`에서 `y=2.7358`(기대 2.465), `default_aims_toward_robot_and_upward`에서 `dir.z > 0` 실패.

- [ ] **Step 3: 레이아웃 상수를 바꾼다**

`src/sim/launch/layout.rs`의 `impl Layout` 블록 전체를 다음으로 교체한다:

```rust
impl Layout {
    /// 로봇은 y≈0, 슈터는 테이블 +y 쪽(상대편).
    pub const MOUNT_X: f64 = table::WIDTH_X * 0.5;
    /// 마운트 기준 발사구 전방 돌출 [m].
    ///
    /// 0 — 발사구를 조준 회전의 피벗과 일치시킨다. 그래야 yaw/pitch/roll을
    /// 어떻게 두든 발사 위치가 실측 좌표에 고정된다. 돌출을 주면 pitch가
    /// 발사구를 들어올려 "기본 발사 위치"가 조준각에 따라 달라진다.
    pub const BARREL_FORWARD_M: f64 = 0.0;
    /// 뷰어 직육면체 전체 크기 [m] (충돌 없음 — 표시 전용)
    pub const VISUAL_SIZE_X: f64 = 0.10;
    pub const VISUAL_SIZE_Y: f64 = 0.18;
    pub const VISUAL_SIZE_Z: f64 = 0.14;
    /// 슈터 마운트 y [m] — 실측: 테이블 끝선에서 27.5 cm 안쪽.
    pub const MOUNT_Y: f64 = table::LENGTH_Y - 0.275;
    /// 슈터 마운트 높이 [m] (테이블 면 → 발사구). 실측 22.5 cm의 두 배 —
    /// `mount_position()`이 `BODY_HEIGHT * 0.5`를 쓴다.
    pub const BODY_HEIGHT: f64 = 0.45;
}
```

- [ ] **Step 4: 기본 설정값을 바꾼다**

`src/defaults/sim.rs`의 `impl Default for launch::Settings`를 다음으로 교체한다:

```rust
impl Default for launch::Settings {
    fn default() -> Self {
        // 실측 슈터: 발사구 (WIDTH_X/2, LENGTH_Y−0.275, 면+0.225), 위로 15°.
        // 오프셋이 모두 0이어야 muzzle == mount가 되어 실측 좌표와 일치한다.
        //
        // speed 7.5: 6.0이면 접수 평면 도달 높이가 면+8 cm라 라켓이 들어갈 수
        // 없다. 예측기 실측 — 6.0 → 면+0.08, 7.0 → 면+0.28, 8.0 → 면+0.41
        // (y=0.20 기준). 7.5면 이전 기본값(면+0.31)과 비슷한 높이가 된다.
        return Self {
            speed_mps: 7.5,
            yaw_deg: 0.0,
            pitch_deg: 15.0,
            roll_deg: 0.0,
            pos_offset_x_m: 0.0,
            pos_offset_y_m: 0.0,
            pos_offset_z_m: 0.0,
            lateral_offset_m: 0.0,
            height_offset_m: 0.0,
            topspin_rad_s: 0.0,
            sidespin_rad_s: 0.0,
            drill_spin_rad_s: 0.0,
        };
    }
}
```

- [ ] **Step 5: 대상 테스트가 통과하는지 확인한다**

Run: `cargo test -p pingpong-bot --lib sim::launch::settings 2>&1 | tail -30`
Expected: PASS — `default_muzzle_matches_measured_mount`, `default_muzzle_is_independent_of_aim`, `default_shot_reaches_hit_plane_high_enough`, `visual_body_clears_table_surface`, `default_aims_toward_robot_and_upward`, `visual_front_face_matches_muzzle`, `default_shot_clears_rapier_net` 모두 통과.

- [ ] **Step 6: 전체 회귀를 돌리고 파급을 실측한다**

Run: `cargo test 2>&1 | tail -60`

`RANDOM_SHOT_*` 범위(`src/defaults/sim.rs:6-46`)와 eval 상수는 **옛 기하 기준으로 튜닝된 값**이다. 새 마운트는 0.5 m 가까워졌고 `BARREL_FORWARD_M`이 0이 됐으므로, 다음이 깨질 수 있다:

- `randomized_varies_aim_height_spin` / `sample_without_gate_often_clips_net_but_randomized_does_not` (`src/sim/launch/settings.rs`)
- `diag_random_shot_speed_reachability` (`src/sim/physics/world.rs` 테스트)
- `tests/incoming_net_gate_matches_sim.rs`, `tests/diag_wp*.rs`

**깨지면 임의로 상수를 흔들지 말고**, 실패 목록과 실제 수치를 그대로 보고한 뒤 재튜닝 여부를 사람에게 확인받는다. 전부 통과하면 다음 단계로 간다.

- [ ] **Step 7: 커밋**

```bash
git add src/sim/launch/layout.rs src/defaults/sim.rs src/sim/launch/settings.rs \
        docs/superpowers/specs/2026-07-30-jog-shooter-swing-design.md \
        docs/superpowers/plans/2026-07-30-jog-shooter-swing.md
git commit -F - <<'EOF'
feat(sim): pin shooter muzzle to measured mount geometry

발사구를 조준 회전의 피벗과 일치시켜(BARREL_FORWARD_M=0) yaw/pitch/roll과
무관하게 발사 위치가 실측 좌표에 고정되게 했다.

- 발사구 = (WIDTH_X/2, LENGTH_Y-0.275, 면+0.225)
- 기본 pitch -1° -> +15°, speed 6.0 -> 7.5 m/s
  (6.0은 접수 평면 도달 높이가 면+8cm라 라켓이 들어갈 수 없다)
- 본체가 테이블 위에 얹히므로 visual_body_sits_outside_table_end를
  visual_body_clears_table_surface로 교체
EOF
```

---

### Task 2: 슈터 egui 위젯을 공용 모듈로 추출

**Files:**
- Create: `src/sim/gui/shooter/ui.rs`
- Modify: `src/sim/gui/shooter/mod.rs`
- Modify: `src/sim/gui/viewer/panel.rs:83-158`

**Interfaces:**
- Consumes: `launch::Settings` (Task 1에서 기본값 확정)
- Produces:
  - `pub struct shooter::ui::Buttons { pub shoot: bool, pub random: bool, pub park: bool }` — `Debug + Default + Clone + Copy`
  - `pub fn shooter::ui::draw(ui: &mut egui::Ui, settings: &mut launch::Settings) -> Buttons`
  - `pub use ui::Buttons as ShooterButtons;`는 만들지 않는다 — 호출부는 `shooter::ui::draw` 경로를 그대로 쓴다.

- [ ] **Step 1: 공용 위젯 파일을 만든다**

Create `src/sim/gui/shooter/ui.rs`:

```rust
//! 슈터 파라미터 egui 위젯 — 메인 sim 패널과 jog가 공유한다.

use kiss3d::egui;

use crate::sim::launch;

/// 위젯 안 버튼 클릭 결과.
#[derive(Debug, Default, Clone, Copy)]
pub struct Buttons {
    pub shoot: bool,
    pub random: bool,
    pub park: bool,
}

/// 슈터 파라미터 전체 + Shoot / Random / Park.
///
/// 슬라이더 범위가 여기 한 곳에만 있다 — 호출부가 늘어도 범위는 갈라지지 않는다.
pub fn draw(ui: &mut egui::Ui, settings: &mut launch::Settings) -> Buttons {
    let mut buttons = Buttons::default();

    ui.collapsing("Position", |ui| {
        ui.add(egui::Slider::new(&mut settings.pos_offset_x_m, -0.8..=0.8).text("x [m]"));
        ui.add(egui::Slider::new(&mut settings.pos_offset_y_m, -0.6..=0.8).text("y [m]"));
        ui.add(egui::Slider::new(&mut settings.pos_offset_z_m, -0.3..=0.5).text("z [m]"));
        let m = settings.mount_position();
        ui.monospace(format!("mount {:.2} {:.2} {:.2}", m.x, m.y, m.z));
    });
    ui.collapsing("Aim", |ui| {
        ui.add(egui::Slider::new(&mut settings.yaw_deg, -25.0..=25.0).text("yaw [deg]"));
        ui.add(egui::Slider::new(&mut settings.pitch_deg, -25.0..=25.0).text("pitch [deg]"));
        ui.add(egui::Slider::new(&mut settings.roll_deg, -45.0..=45.0).text("roll [deg]"));
    });
    ui.collapsing("Muzzle", |ui| {
        ui.add(egui::Slider::new(&mut settings.lateral_offset_m, -0.5..=0.5).text("lateral [m]"));
        ui.add(egui::Slider::new(&mut settings.height_offset_m, -0.2..=0.4).text("height [m]"));
    });
    ui.collapsing("Speed / spin", |ui| {
        ui.add(egui::Slider::new(&mut settings.speed_mps, 3.0..=15.0).text("speed [m/s]"));
        ui.add(egui::Slider::new(&mut settings.topspin_rad_s, -80.0..=80.0).text("topspin"));
        ui.add(egui::Slider::new(&mut settings.sidespin_rad_s, -80.0..=80.0).text("sidespin"));
        ui.add(egui::Slider::new(&mut settings.drill_spin_rad_s, -80.0..=80.0).text("drill"));
    });
    ui.horizontal(|ui| {
        if ui.button("Shoot").clicked() {
            buttons.shoot = true;
        }
        if ui.button("Random").clicked() {
            buttons.random = true;
        }
        if ui.button("Park").clicked() {
            buttons.park = true;
        }
    });

    return buttons;
}
```

- [ ] **Step 2: 모듈을 공개한다**

`src/sim/gui/shooter/mod.rs`를 다음으로 교체한다:

```rust
//! sim GUI — 슈터 settings R/W · egui 위젯 · 본체 비주얼.

#[cfg(feature = "gui")]
pub mod handle;
#[cfg(feature = "gui")]
pub mod ui;

#[cfg(feature = "gui")]
pub use handle::Handle;
```

- [ ] **Step 3: 메인 sim 패널이 공용 위젯을 쓰게 한다**

`src/sim/gui/viewer/panel.rs`에서 `egui::Window::new("Shooter")`의 `.show(ctx, |ui| { ... })` 클로저 본문(현재 89-157행, `ui.collapsing("Position", ...)`부터 마지막 `ui.horizontal(...)`까지) 전체를 다음으로 교체한다. `shoot` / `random_shoot` / `park`는 68-70행에 이미 선언되어 있다.

```rust
        .show(ctx, |ui| {
            let buttons = crate::sim::gui::shooter::ui::draw(ui, &mut ui_state.shooter);
            shoot |= buttons.shoot;
            random_shoot |= buttons.random;
            park |= buttons.park;
        });
```

- [ ] **Step 4: 빌드·회귀 확인**

Run: `cargo build 2>&1 | tail -20 && cargo test 2>&1 | tail -20`
Expected: 경고 없이 빌드, 테스트 전부 통과. 슈터 창의 동작은 리팩터 전과 같다 (순수 추출).

- [ ] **Step 5: 커밋**

```bash
git add src/sim/gui/shooter/ui.rs src/sim/gui/shooter/mod.rs src/sim/gui/viewer/panel.rs
git commit -F - <<'EOF'
refactor(sim): extract shared shooter egui widget

Shooter 창 본문을 sim::gui::shooter::ui::draw로 추출했다. 슬라이더 범위가
한 곳에만 있으므로 jog가 같은 패널을 띄워도 범위가 갈라지지 않는다.
동작 변화 없음 — 순수 추출.
EOF
```

---

### Task 3: 슈터 3D Visual 공용화 + 경량 호스트 렌더

**Files:**
- Create: `src/sim/gui/shooter/visual.rs`
- Modify: `src/sim/gui/shooter/mod.rs`
- Modify: `src/sim/gui/viewer/scene_dynamics.rs:37`(struct 필드), `:173-179`(spawn), `:406-410`(sync)
- Modify: `src/sim/gui/host/run.rs:81`, `:113-119` 부근

**Interfaces:**
- Consumes: `launch::Layout::VISUAL_SIZE_*`, `launch::Settings::{visual_position, orientation}`, `shooter::Handle::settings`
- Produces:
  - `pub struct shooter::Visual`
  - `pub fn shooter::Visual::spawn(scene: &mut SceneNode3d) -> Visual`
  - `pub fn shooter::Visual::set_from_settings(&mut self, settings: &launch::Settings)`
  - `pub fn shooter::Visual::set_pose(&mut self, position: Vector, rotation: Rotation)` — `Vector`/`Rotation`은 `rapier3d::prelude`
  - `pub use visual::Visual;`

- [ ] **Step 1: 공용 Visual을 만든다**

Create `src/sim/gui/shooter/visual.rs`:

```rust
//! 슈터 본체 비주얼 (직육면체, 충돌 없음 — 표시 전용).

use kiss3d::prelude::*;
use rapier3d::prelude::{Rotation, Vector};

use crate::sim::launch;

/// 슈터 본체 직육면체. 발사구가 전면에 오도록 조준축 뒤로 물려 그린다.
pub struct Visual {
    node: SceneNode3d,
}

impl Visual {
    pub fn spawn(scene: &mut SceneNode3d) -> Self {
        let node = scene
            .add_cube(
                launch::Layout::VISUAL_SIZE_X as f32,
                launch::Layout::VISUAL_SIZE_Y as f32,
                launch::Layout::VISUAL_SIZE_Z as f32,
            )
            .set_color(Color::new(0.45, 0.45, 0.5, 1.0));
        return Self { node };
    }

    /// 물리 월드가 준 본체 자세 그대로 (`SimWorld::shooter_pose`).
    pub fn set_pose(&mut self, position: Vector, rotation: Rotation) {
        self.node
            .set_position(Vec3::new(position.x, position.y, position.z))
            .set_rotation(Quat::from_xyzw(
                rotation.x, rotation.y, rotation.z, rotation.w,
            ));
    }

    /// 설정에서 직접 — 월드 없이 그릴 때. `SimWorld::sync_shooter_pose`와 같은 SSOT.
    pub fn set_from_settings(&mut self, settings: &launch::Settings) {
        self.set_pose(settings.visual_position(), settings.orientation());
    }

    pub fn node_mut(&mut self) -> &mut SceneNode3d {
        return &mut self.node;
    }
}
```

- [ ] **Step 2: 모듈을 공개한다**

`src/sim/gui/shooter/mod.rs`를 다음으로 교체한다:

```rust
//! sim GUI — 슈터 settings R/W · egui 위젯 · 본체 비주얼.

#[cfg(feature = "gui")]
pub mod handle;
#[cfg(feature = "gui")]
pub mod ui;
#[cfg(feature = "gui")]
pub mod visual;

#[cfg(feature = "gui")]
pub use handle::Handle;
#[cfg(feature = "gui")]
pub use visual::Visual;
```

- [ ] **Step 3: 메인 뷰어가 공용 Visual을 쓰게 한다**

`src/sim/gui/viewer/scene_dynamics.rs`에서:

1. `struct SceneDynamics`의 `shooter: SceneNode3d,`를 `shooter: shooter::Visual,`로 바꾸고, 파일 상단 `use` 에 `use crate::sim::gui::shooter;`를 추가한다.
2. spawn 부분(173-179행)을 다음으로 바꾼다:

```rust
    let shooter = shooter::Visual::spawn(scene);
```

3. sync 부분(406-410행)을 다음으로 바꾼다:

```rust
    let (sh_pos, sh_rot) = world.shooter_pose();
    nodes.shooter.set_pose(sh_pos, sh_rot);
```

`to_vec3` / `to_quat`가 다른 곳에서도 쓰이면 그대로 두고, 이 변경으로 미사용이 되면 삭제한다 (`cargo build`의 dead_code 경고로 확인).

- [ ] **Step 4: 경량 호스트가 슈터를 그리게 한다**

`src/sim/gui/host/run.rs`에서:

1. 상단 `use` 에 `use crate::sim::gui::shooter;`를 추가한다.
2. 81행 `let _ = &options.layers.shooter;`를 다음으로 교체한다:

```rust
    let mut shooter_visual = options
        .layers
        .shooter
        .as_ref()
        .map(|_| shooter::Visual::spawn(&mut scene));
```

3. 렌더 루프 안, robot 동기화 블록(113-119행) 바로 다음에 추가한다:

```rust
        if let (Some(handle), Some(visual)) = (&options.layers.shooter, &mut shooter_visual) {
            visual.set_from_settings(&handle.settings());
        }
```

- [ ] **Step 5: 빌드·회귀 확인**

Run: `cargo build 2>&1 | tail -20 && cargo test 2>&1 | tail -20`
Expected: 빌드 성공, 테스트 전부 통과.

- [ ] **Step 6: 커밋**

```bash
git add src/sim/gui/shooter/visual.rs src/sim/gui/shooter/mod.rs \
        src/sim/gui/viewer/scene_dynamics.rs src/sim/gui/host/run.rs
git commit -F - <<'EOF'
feat(sim): render shooter in the lightweight scene host

슈터 본체 큐보이드를 sim::gui::shooter::Visual로 추출하고, 경량 호스트가
무시하던 layers.shooter를 실제로 그리도록 했다. jog처럼 ui_hook 경로를 쓰는
씬에서도 슈터가 보인다.
EOF
```

---

### Task 4: jog 슈터 예측 헬퍼

**Files:**
- Create: `tools/jog/src/plan/shooter.rs`
- Modify: `tools/jog/src/plan/mod.rs:3-15` (모듈 선언·재수출)

**Interfaces:**
- Consumes: `launch::Settings`, `Kinematics::predict_to`, `HitPlane`, `Prediction`
- Produces: `pub fn plan::shooter::predict(settings: &launch::Settings, hit_plane_y: f64) -> anyhow::Result<Prediction>` — 실패 시 사람이 읽는 한국어 사유가 담긴 `Err`

- [ ] **Step 1: 실패하는 테스트를 쓴다**

Create `tools/jog/src/plan/shooter.rs` — 지금은 테스트만 넣고 함수는 비워 두지 않는다. 대신 아래 Step 3에서 본문을 채운다. 우선 파일 전체를 다음으로 만든다 (`predict`는 `todo!()`):

```rust
//! 슈터 설정 → 접수 평면 도달 예측.

use anyhow::{Result, ensure};
use nalgebra::Vector3;
use pingpong_bot::defaults::PhysicsParams;
use pingpong_bot::estimator::{HitPlane, Kinematics, Prediction};
use pingpong_bot::sim::launch;

/// 슈터 설정으로 발사한 공이 `hit_plane_y` 평면에 도달하는 지점·속도.
///
/// 실제 파이프라인과 같은 예측기를 쓴다 — 테이블 바운스를 포함해 적분하고,
/// 네트 미달·테이블 구름·리드 시간 밖이면 실패한다.
pub fn predict(settings: &launch::Settings, hit_plane_y: f64) -> Result<Prediction> {
    todo!("Step 3")
}

#[cfg(test)]
mod tests {
    use super::*;
    use pingpong_bot::constants::table;

    #[test]
    fn default_shooter_reaches_default_hit_plane() {
        let pred = predict(&launch::Settings::default(), table::DEFAULT_HIT_PLANE_Y)
            .expect("기본 슈터는 접수 평면에 도달해야 한다");
        assert!(pred.time_to_impact_secs > 0.0);
        assert!(pred.incoming_velocity.y < 0.0, "로봇 쪽으로 와야 한다");
        assert!(
            pred.impact_position.coords.z > table::SURFACE_Z + 0.15,
            "도달 높이 {}",
            pred.impact_position.coords.z - table::SURFACE_Z
        );
    }

    #[test]
    fn downward_slow_shot_is_unreachable() {
        let settings = launch::Settings {
            pitch_deg: -25.0,
            speed_mps: 3.0,
            ..Default::default()
        };
        let err = predict(&settings, table::DEFAULT_HIT_PLANE_Y).unwrap_err();
        assert!(
            format!("{err:#}").contains("도달"),
            "사유가 사람이 읽을 수 있어야 함: {err:#}"
        );
    }

    #[test]
    fn non_finite_plane_is_rejected() {
        let err = predict(&launch::Settings::default(), f64::NAN).unwrap_err();
        assert!(format!("{err:#}").contains("접수 평면"), "{err:#}");
    }
}
```

`tools/jog/src/plan/mod.rs` 상단의 모듈 선언에 추가한다:

```rust
mod draft;
mod kind;
pub mod shooter;
```

새 의존성은 필요 없다 — `muzzle_position()` 등이 돌려주는 rapier `Vector`는 타입 이름을 쓰지 않고 필드(`.x/.y/.z`)만 읽어 변환한다.

- [ ] **Step 2: 테스트가 실패하는지 확인한다**

Run: `cargo test -p jog plan::shooter 2>&1 | tail -20`
Expected: FAIL — `not yet implemented` 패닉 3건.

- [ ] **Step 3: 구현한다**

`predict`와 헬퍼를 다음으로 채운다:

```rust
pub fn predict(settings: &launch::Settings, hit_plane_y: f64) -> Result<Prediction> {
    ensure!(hit_plane_y.is_finite(), "접수 평면 y가 유한해야 합니다");
    let m = settings.muzzle_position();
    let v = settings.launch_velocity();
    let w = settings.launch_angular_velocity();
    return Kinematics::predict_to(
        Vector3::new(f64::from(m.x), f64::from(m.y), f64::from(m.z)),
        Vector3::new(f64::from(v.x), f64::from(v.y), f64::from(v.z)),
        Vector3::new(f64::from(w.x), f64::from(w.y), f64::from(w.z)),
        HitPlane { y: hit_plane_y },
        &PhysicsParams::default(),
    )
    .ok_or_else(|| {
        anyhow::anyhow!(
            "이 슈터 설정으로는 y={hit_plane_y:.3} m 평면에 도달하는 공이 없습니다 \
             (네트 미달 · 너무 낮음 · 리드 시간 밖)"
        )
    });
}
```

- [ ] **Step 4: 테스트가 통과하는지 확인한다**

Run: `cargo test -p jog plan::shooter 2>&1 | tail -20`
Expected: PASS 3/3.

- [ ] **Step 5: 커밋**

```bash
git add tools/jog/src/plan/shooter.rs tools/jog/src/plan/mod.rs
git commit -F - <<'EOF'
feat(jog): predict hit-plane arrival from shooter settings

슈터 설정을 실제 파이프라인과 같은 예측기(Kinematics::predict_to)에 넣어
도달점·입사속도를 얻는 헬퍼. 도달 불가는 사람이 읽는 사유로 실패한다.
EOF
```

---

### Task 5: jog 모션 종류를 슈터 기반 Swing 하나로 통합

**Files:**
- Modify: `tools/jog/src/plan/kind.rs` (전체 교체)
- Modify: `tools/jog/src/plan/draft.rs` (전체 교체)
- Modify: `tools/jog/src/plan/mod.rs` (`compose` · `reach_ok` · `swing_traj` · `swing_ball_traj` · `ball_aim_target`)

**Interfaces:**
- Consumes: `plan::shooter::predict` (Task 4)
- Produces:
  - `pub enum Kind { Joint, Angles, RailAbs, Ik, Pose, Swing }`
  - `Draft { kind, joint_index, joint_deg, angles_deg, rail_x, reach_dxyz, tilt_pitch_deg, tilt_yaw_deg, shooter: launch::Settings, hit_plane_y: f64 }`
  - `pub struct plan::SwingPreview { pub prediction: Prediction, pub ik_ok: bool }`
  - `pub fn plan::swing_preview(arm: &Arm, start: &robot::Pose, draft: &Draft) -> Result<SwingPreview>`
  - `pub fn plan::compose(...)`, `pub fn plan::reach_ok(...)` — 시그니처 불변

- [ ] **Step 1: 실패하는 테스트를 쓴다**

`tools/jog/src/plan/mod.rs`의 `mod tests`에 추가한다:

```rust
    #[test]
    fn default_shooter_swing_has_a_solution() {
        let built = defaults::robot().expect("robot");
        let start = robot::Pose::new(0.0, built.arm.default_joints.clone());
        let mut draft = Draft::default();
        draft.kind = Kind::Swing;
        let preview = swing_preview(&built.arm, &start, &draft).expect("예측은 성공해야 한다");
        assert!(preview.ik_ok, "기본 슈터 공은 IK가 풀려야 한다");
        compose(&built.arm, &start, &draft, 1.0, 90.0).expect("스윙 궤적이 만들어져야 한다");
    }

    #[test]
    fn unreachable_shooter_swing_reports_reason() {
        let built = defaults::robot().expect("robot");
        let start = robot::Pose::new(0.0, built.arm.default_joints.clone());
        let mut draft = Draft::default();
        draft.kind = Kind::Swing;
        draft.shooter.pitch_deg = -25.0;
        draft.shooter.speed_mps = 3.0;
        let err = compose(&built.arm, &start, &draft, 1.0, 90.0).unwrap_err();
        assert!(format!("{err:#}").contains("도달"), "{err:#}");
        assert!(!reach_ok(&built.arm, &start, &draft));
    }
```

- [ ] **Step 2: 테스트가 실패하는지 확인한다**

Run: `cargo test -p jog 2>&1 | tail -20`
Expected: FAIL — `swing_preview` 미정의, `Draft`에 `shooter` 필드 없음 (컴파일 에러).

- [ ] **Step 3: Kind를 정리한다**

`tools/jog/src/plan/kind.rs` 전체를 교체한다:

```rust
//! 조그 모션 종류.

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Kind {
    Joint,
    Angles,
    RailAbs,
    Ik,
    Pose,
    /// 슈터가 쏜 공의 예측 도달점으로 임팩트 스윙.
    Swing,
}

impl Kind {
    pub fn label(self) -> &'static str {
        return match self {
            Self::Joint => "관절 하나",
            Self::Angles => "관절 전부",
            Self::RailAbs => "레일 절대 위치",
            Self::Ik => "라켓 조금 옮기기",
            Self::Pose => "라켓 옮기기+기울이기",
            Self::Swing => "스윙 (슈터 공)",
        };
    }
}
```

- [ ] **Step 4: Draft를 교체한다**

`tools/jog/src/plan/draft.rs` 전체를 교체한다:

```rust
//! egui 입력 초안.

use pingpong_bot::constants::table;
use pingpong_bot::sim::launch;

use super::kind::Kind;

/// egui 입력 초안.
#[derive(Debug, Clone)]
pub struct Draft {
    pub kind: Kind,
    pub joint_index: usize,
    pub joint_deg: f64,
    pub angles_deg: [f64; 4],
    pub rail_x: f64,
    /// IK / Pose 공통: 현재 라켓 위치 대비 Δ(좌우, 전후, 높이) [m].
    pub reach_dxyz: [f64; 3],
    /// Pose / Swing: 기준 법선 대비 기울기 [deg].
    pub tilt_pitch_deg: f64,
    pub tilt_yaw_deg: f64,
    /// Swing: 슈터 설정 (sim controls와 동기화된다).
    pub shooter: launch::Settings,
    /// Swing: 공을 맞을 접수 평면 y [m].
    pub hit_plane_y: f64,
}

impl Default for Draft {
    fn default() -> Self {
        return Self {
            kind: Kind::Joint,
            joint_index: 0,
            joint_deg: 0.0,
            angles_deg: [0.0; 4],
            rail_x: 0.0,
            reach_dxyz: [0.0; 3],
            tilt_pitch_deg: 0.0,
            tilt_yaw_deg: 0.0,
            shooter: launch::Settings::default(),
            hit_plane_y: table::DEFAULT_HIT_PLANE_Y,
        };
    }
}
```

- [ ] **Step 5: compose / reach_ok / 스윙 계산을 교체한다**

`tools/jog/src/plan/mod.rs`에서:

1. `use` 에 `use pingpong_bot::estimator::Prediction;`를 추가하고, `pub use draft::Draft;` 아래에 재수출은 추가하지 않는다 (`SwingPreview`는 이 파일에서 직접 정의).

2. `compose`의 `match`에서 `Kind::Swing => swing_traj(arm, start, draft, duration_secs, max_delta_deg),`는 그대로 두고, `Kind::AimBall`·`Kind::SwingBall` arm 두 개를 삭제한다.

3. `reach_ok`의 `Kind::Swing` arm을 다음으로 바꾸고, `Kind::AimBall | Kind::SwingBall` arm은 삭제한다:

```rust
        Kind::Swing => swing_preview(arm, start, draft).is_ok_and(|p| p.ik_ok),
```

4. 기존 `swing_ball_traj`와 `swing_traj` 함수 두 개를 삭제하고, `ball_aim_target` 함수도 삭제한다. 대신 다음을 넣는다:

```rust
/// 슈터 공 스윙의 미리보기 정보 — 패널 표시와 `reach_ok`가 함께 쓴다.
pub struct SwingPreview {
    pub prediction: Prediction,
    /// 도달점·법선으로 임팩트 포즈 IK가 풀리는가.
    pub ik_ok: bool,
}

/// 슈터 설정 → 도달 예측 → 임팩트 포즈 IK 가능 여부.
///
/// 예측 자체가 실패하면 `Err` — 그 사유를 패널에 그대로 띄운다.
pub fn swing_preview(arm: &Arm, start: &robot::Pose, draft: &Draft) -> Result<SwingPreview> {
    let prediction = shooter::predict(&draft.shooter, draft.hit_plane_y)?;
    let normal = swing_normal(&prediction, draft)?;
    let ik_ok = arm
        .inverse_pose_with_rail(prediction.impact_position, normal, start)
        .is_ok();
    return Ok(SwingPreview { prediction, ik_ok });
}

/// 라켓 기준 법선 = 입사 반대 방향 + 사용자 기울기.
fn swing_normal(prediction: &Prediction, draft: &Draft) -> Result<Vector3<f64>> {
    let v_in = prediction.incoming_velocity;
    ensure!(v_in.norm() > 1e-3, "입사 속도가 너무 작습니다");
    return tilt_normal(-v_in.normalize(), draft.tilt_pitch_deg, draft.tilt_yaw_deg);
}

fn swing_traj(
    arm: &Arm,
    start: &robot::Pose,
    draft: &Draft,
    duration_secs: f64,
    max_delta_deg: f64,
) -> Result<motion::Trajectory> {
    let prediction = shooter::predict(&draft.shooter, draft.hit_plane_y)?;
    let target = prediction.impact_position;
    let v_in = prediction.incoming_velocity;
    let aim_normal = swing_normal(&prediction, draft)?;

    let impact = arm
        .inverse_pose_with_rail(target, aim_normal, start)
        .context("스윙 임팩트 포즈 IK")?;
    let racket = arm
        .forward_kinematics_with_rail(impact.rail_x, &impact.joints)
        .context("임팩트 FK")?;
    let normal = racket.normal.normalize();

    let v_out = Impact::rally_return(target, v_in);
    let e = ImpactParams::default().racket_effective_restitution;
    let v_r = Impact::required_racket_velocity(v_in, v_out, normal, e).context("라켓 속도 역산")?;

    let (rail_impact_vel, joint_impact_vel) = arm
        .velocities_for_racket_velocity(&impact, v_r)
        .context("라켓 속도 → 관절·레일 속도")?;

    ensure_max_delta(&start.joints, &impact.joints, max_delta_deg)?;
    return build_follow_through_swing(
        start,
        &impact,
        joint_impact_vel,
        rail_impact_vel,
        duration_secs,
    );
}
```

5. `reach_pose_target`은 `Kind::Pose`만 쓰므로 그대로 둔다. `current_racket`도 유지된다.
6. `use` 정리: `Point3`가 `ball_aim_target` 삭제로 미사용이 되면 `point3` 헬퍼와 함께 삭제한다. `vec3`도 미사용이면 삭제한다. `cargo build`의 경고로 확인한다.

- [ ] **Step 6: 테스트가 통과하는지 확인한다**

Run: `cargo test -p jog 2>&1 | tail -30`
Expected: 컴파일 에러는 `tools/jog/src/panel.rs`·`state/jog_app.rs`에서만 남는다 (Task 6에서 고친다). 그 두 파일을 아직 안 고쳤으므로 이 단계에서는 `cargo check -p jog --lib`가 아니라 **Task 6까지 마친 뒤 한 번에** 테스트한다. 여기서는 `plan` 모듈만 확인한다:

Run: `cargo build -p jog 2>&1 | rg "^error" | head -20`
Expected: `panel.rs` / `jog_app.rs`의 `Kind::AimBall` · `draft.arrival_xyz` · `draft.swing_speed` 관련 에러만 남는다.

- [ ] **Step 7: 커밋하지 않는다**

이 태스크만으로는 `jog`가 빌드되지 않는다. Task 6과 함께 커밋한다.

---

### Task 6: jog 패널·배선·문서

**Files:**
- Modify: `tools/jog/src/panel.rs` (`draw` · `draw_motion` · `draw_arrival` 삭제 · `draw_swing` 추가)
- Modify: `tools/jog/src/state/jog_app.rs` (`shooter` 핸들 · 고스트 동기화)
- Modify: `tools/jog/src/main.rs:107-132`
- Modify: `tools/jog/README.md`

**Interfaces:**
- Consumes: `plan::{Kind, Draft, SwingPreview, swing_preview, reach_ok}`, `sim::gui::shooter::{Handle, ui}`
- Produces: 없음 (최종 소비자)

- [ ] **Step 1: JogApp에 슈터 핸들과 고스트 동기화를 넣는다**

`tools/jog/src/state/jog_app.rs`에서:

1. `use pingpong_bot::sim::gui::ball;` 아래에 `use pingpong_bot::sim::gui::shooter;`를 추가하고, `use crate::plan::{self, Draft, Kind};`는 `use crate::plan::{self, Draft, Kind, SwingPreview};`로 바꾼다.
2. `pub ball: Option<ball::Handle>,` 아래에 필드를 추가한다:

```rust
    pub shooter: Option<shooter::Handle>,
```

3. `JogApp::new`의 반환 구조체에 `shooter: None,`을 추가한다.
4. `attach_ball` 아래에 추가한다:

```rust
    pub fn attach_shooter(&mut self, shooter: shooter::Handle) {
        self.shooter = Some(shooter);
    }

    /// 패널의 슈터 값을 sim controls로 밀어 넣는다 (월드 슈터 자세·비주얼 갱신).
    pub fn push_shooter(&self) {
        if let Some(handle) = &self.shooter {
            handle.set_settings(self.draft.shooter.clone());
        }
    }
```

5. `sync_arrival_ghost`를 통째로 다음으로 바꾼다:

```rust
    /// 예측 도달점을 홀로그램 공에 반영. Swing 이외 모션이거나 예측 실패면 숨김.
    pub fn sync_ball_ghost(&self, preview: Option<&SwingPreview>) {
        let Some(ball) = &self.ball else {
            return;
        };
        let Some(preview) = preview.filter(|_| self.draft.kind == Kind::Swing) else {
            ball.set_position(None);
            ball.set_velocity(None);
            return;
        };
        let v = preview.prediction.incoming_velocity;
        ball.set_position(Some(preview.prediction.impact_position));
        ball.set_velocity(Some([v.x, v.y, v.z]));
    }
```

6. `fill_draft_from_pose`는 그대로 둔다 (`arrival_xyz` 참조가 없다).

- [ ] **Step 2: 패널을 고친다**

`tools/jog/src/panel.rs`에서:

1. `use` 를 다음으로 바꾼다:

```rust
use kiss3d::egui::{self, Color32, RichText};
use pingpong_bot::robot::motion::InterceptWindow;
use pingpong_bot::sim::gui::shooter;

use crate::plan::{Kind, REACH_DELTA_M, SwingPreview, joint_label, reach_ok, swing_preview};
use crate::state::{Action, JogApp, try_action};
```

(`pingpong_bot::constants::table` import는 `draw_arrival` 삭제로 미사용이 되면 지운다.)

2. `draw` 를 다음으로 바꾼다 — 예측을 프레임당 한 번만 계산해 고스트·표시가 공유하게 한다:

```rust
pub fn draw(ctx: &egui::Context, app: &mut JogApp) {
    ensure_korean_fonts(ctx);

    let preview = if app.draft.kind == Kind::Swing {
        app.synced_pose
            .as_ref()
            .and_then(|pose| swing_preview(&app.arm, pose, &app.draft).ok())
    } else {
        None
    };
    app.sync_ball_ghost(preview.as_ref());

    draw_shooter_window(ctx, app);

    egui::Window::new("Jog")
        .default_pos(egui::pos2(12.0, 12.0))
        .default_width(400.0)
        .resizable(true)
        .show(ctx, |ui| {
            draw_header(ui, app);
            ui.separator();
            draw_status(ui, app);
            ui.separator();
            draw_params(ui, app);
            ui.separator();
            draw_motion(ui, app, preview.as_ref());
            ui.separator();
            draw_actions(ui, app, preview.as_ref());
            if let Some(err) = &app.error {
                ui.add_space(4.0);
                ui.colored_label(Color32::from_rgb(220, 90, 80), err);
            }
        });
}

/// 메인 sim과 같은 슈터 위젯. 값이 바뀌면 곧바로 sim controls로 민다.
fn draw_shooter_window(ctx: &egui::Context, app: &mut JogApp) {
    egui::Window::new("슈터")
        .default_pos(egui::pos2(440.0, 12.0))
        .default_width(280.0)
        .resizable(true)
        .show(ctx, |ui| {
            let buttons = shooter::ui::draw(ui, &mut app.draft.shooter);
            if buttons.random {
                app.draft.shooter = app
                    .draft
                    .shooter
                    .randomized(&mut rand::thread_rng());
            }
            app.push_shooter();
            if let Some(handle) = &app.shooter {
                if buttons.shoot {
                    handle.request_shoot();
                }
                if buttons.park {
                    handle.request_park();
                }
            }
        });
}
```

3. `draw_motion` 시그니처를 `fn draw_motion(ui: &mut egui::Ui, app: &mut JogApp, preview: Option<&SwingPreview>)`로 바꾸고, ComboBox의 `for kind in [...]` 배열을 다음으로 줄인다:

```rust
            for kind in [
                Kind::Joint,
                Kind::Angles,
                Kind::RailAbs,
                Kind::Ik,
                Kind::Pose,
                Kind::Swing,
            ] {
```

`match app.draft.kind` 에서 `Kind::Swing` arm을 다음으로 바꾸고, `Kind::AimBall`·`Kind::SwingBall` arm은 삭제한다:

```rust
        Kind::Swing => {
            draw_swing(ui, app, preview);
        }
```

4. `draw_arrival` 함수를 통째로 삭제하고 다음을 넣는다:

```rust
fn draw_swing(ui: &mut egui::Ui, app: &mut JogApp, preview: Option<&SwingPreview>) {
    let hit = InterceptWindow::default();
    ui.label("공을 맞을 깊이 (접수 평면 y) [m]");
    ranged(
        ui,
        "y",
        &mut app.draft.hit_plane_y,
        hit.y_min,
        hit.y_max,
        0.005,
    );

    ui.label("면 기울기 [°]");
    ranged(ui, "pitch", &mut app.draft.tilt_pitch_deg, -30.0, 30.0, 0.5);
    ranged(ui, "yaw", &mut app.draft.tilt_yaw_deg, -30.0, 30.0, 0.5);

    ui.separator();
    if app.synced_pose.is_none() {
        ui.label("동기화하면 예측 결과가 표시됩니다");
        return;
    }
    let Some(preview) = preview else {
        ui.colored_label(
            Color32::from_rgb(220, 90, 80),
            "이 슈터 설정으로는 접수 평면에 도달하는 공이 없습니다",
        );
        ui.label(
            RichText::new("네트 미달 · 너무 낮음 · 리드 시간 밖 — 속도나 pitch를 올려보세요")
                .weak()
                .small(),
        );
        return;
    };

    let p = preview.prediction.impact_position.coords;
    let v = preview.prediction.incoming_velocity;
    ui.label(format!("도달점 = ({:.3}, {:.3}, {:.3}) m", p.x, p.y, p.z));
    ui.label(format!("입사 속도 = ({:.2}, {:.2}, {:.2}) m/s", v.x, v.y, v.z));
    ui.label(format!(
        "리드 시간 = {:.3} s",
        preview.prediction.time_to_impact_secs
    ));
    if preview.ik_ok {
        ui.colored_label(Color32::from_rgb(90, 190, 120), "IK 가능");
    } else {
        ui.colored_label(
            Color32::from_rgb(220, 90, 80),
            "IK 불가 — 깊이·기울기나 슈터 조준을 바꿔보세요",
        );
    }
}
```

5. `draw_reach`의 `reach_ok` 호출은 그대로 둔다 (Ik / Pose에서만 쓰인다).

6. 도달 불가·IK 불가면 미리보기를 막는다. `draw_actions`의 시그니처와 첫 버튼을 다음으로 바꾼다 (나머지 버튼 3개는 그대로):

```rust
fn draw_actions(ui: &mut egui::Ui, app: &mut JogApp, preview: Option<&SwingPreview>) {
    // 슈터 공이 도달 불가이거나 임팩트 IK가 안 풀리면 미리보기를 막는다.
    let swing_ready = app.draft.kind != Kind::Swing || preview.is_some_and(|p| p.ik_ok);
    ui.horizontal(|ui| {
        if ui
            .add_enabled(
                app.phase.can_preview() && swing_ready,
                egui::Button::new("미리보기"),
            )
            .clicked()
        {
            try_action(app, Action::Preview);
        }
```

- [ ] **Step 3: jog가 rand를 쓰도록 의존성을 추가한다**

`tools/jog/Cargo.toml`의 `[dependencies]`에 추가한다 (루트 `Cargo.toml:44`와 같은 버전 — `randomized`가 `rand::Rng`를 받으므로 major가 같아야 한다):

```toml
rand = "0.8"
```

- [ ] **Step 4: main.rs를 배선한다**

`tools/jog/src/main.rs`에서 `SimScene::builder()` 체인에 `.with_shooter(...)`를 추가하고, 씬에서 핸들을 꺼내 앱에 붙인다.

빌더 체인을 다음으로 바꾼다:

```rust
    let world = session.world();
    let scene = SimScene::builder()
        .title(if args.dry_run { "jog (dry-run)" } else { "jog" })
        .with_robot(Arc::clone(&world))
        .with_shooter(Arc::clone(&controls), Some(Arc::clone(&world)))
        .with_ball()
        .ghost_ball(true)
        .urdf(robot.urdf.clone())
        .with_ui_hook(ui_hook)
        .build();
```

핸들을 꺼내는 블록을 다음으로 바꾼다:

```rust
    let robot_handle = scene
        .robot()
        .cloned()
        .ok_or_else(|| anyhow::anyhow!("robot layer missing"))?;
    let ball_handle = scene
        .ball()
        .cloned()
        .ok_or_else(|| anyhow::anyhow!("ball layer missing"))?;
    let shooter_handle = scene
        .shooter()
        .cloned()
        .ok_or_else(|| anyhow::anyhow!("shooter layer missing"))?;
    {
        let mut app = app.lock().expect("jog app");
        app.attach_robot(robot_handle);
        app.attach_ball(ball_handle);
        app.attach_shooter(shooter_handle);
        app.push_shooter();
        if let Err(err) = app.sync() {
            anyhow::bail!("boot sync 실패: {err:#}");
        }
    }
```

`SimScene::shooter()`는 `src/sim/gui/host/scene.rs:52`에 이미 있다 — 새로 만들 필요 없다.

- [ ] **Step 5: 빌드와 테스트를 돌린다**

Run: `cargo test -p jog 2>&1 | tail -30 && cargo build -p jog 2>&1 | tail -20`
Expected: PASS — `joint_preview_respects_maxdelta`, `zero_reach_delta_is_reachable`, `default_shooter_swing_has_a_solution`, `unreachable_shooter_swing_reports_reason`, `plan::shooter` 3건.

Run: `cargo test 2>&1 | tail -20`
Expected: 워크스페이스 전체 통과.

- [ ] **Step 6: README를 갱신한다**

`tools/jog/README.md`에서:

1. "모션 (패널)" 표의 `swing` 행을 다음으로 바꾸고, 없는 종류는 지운다:

```markdown
| `swing` | 슈터가 쏜 공의 예측 도달점으로 임팩트 스윙 |
```

2. "### 스윙 세기" 절 전체를 다음으로 바꾼다:

```markdown
### 스윙 (슈터 공)

**슈터** 창에서 위치·조준각·속도·스핀을 정하면, `Kinematics::predict_to`가
접수 평면 도달점과 입사 속도를 예측한다 — 실제 파이프라인과 같은 예측기다.
도달점·입사속도를 직접 넣지 않으므로 물리적으로 불가능한 조합이 들어올 수 없다.

Jog 창에서는 **공을 맞을 깊이**(접수 평면 y)와 **면 기울기**만 정한다.
임팩트 라켓 속도는 `rally_return` → `required_racket_velocity`로 역산된다.

도달 불가(네트 미달·너무 낮음·리드 시간 밖)면 사유가 표시되고 미리보기가 막힌다.
**Random**은 네트 통과가 검증된 샷만 뽑으므로 "올만한 공"이 자동으로 나온다.
```

3. 상단 설명 문단의 "공 추적·`plan_swing` 같은 planner는 **쓰지 않는다**." 뒤에 한 문장을 덧붙인다:

```markdown
스윙 입력은 시뮬 슈터 파라미터로 주고, 도달점·입사 속도는 탄도 예측에서 얻는다.
```

- [ ] **Step 7: 커밋**

```bash
git add tools/jog/src/plan/kind.rs tools/jog/src/plan/draft.rs tools/jog/src/plan/mod.rs \
        tools/jog/src/panel.rs tools/jog/src/state/jog_app.rs tools/jog/src/main.rs \
        tools/jog/Cargo.toml tools/jog/README.md
git commit -F - <<'EOF'
feat(jog): drive the swing command from shooter parameters

도달점과 입사 속도를 각각 슬라이더로 받던 탓에 물리적으로 불가능한 조합이
대부분이라 해가 거의 없었다. 이제 슈터 파라미터만 주면 도달점·입사 속도는
탄도 예측에서 나온다.

- Swing/AimBall/SwingBall 3개를 슈터 기반 Swing 하나로 통합
- 메인 sim과 같은 슈터 위젯을 별도 창으로 띄우고 Random 지원
- 도달점·입사속도·리드 시간·IK 가능 여부를 패널에 표시
- 홀로그램 공이 예측 도달점과 입사 속도를 보여준다
EOF
```

---

## 완료 확인

- [ ] `cargo test` 워크스페이스 전체 통과
- [ ] `cargo run -p jog -- --dry-run` 으로 창이 뜨고, 슈터 본체가 보이며, 슈터 슬라이더를 움직이면 본체 자세와 홀로그램 공 도달점이 따라 움직인다
- [ ] 기본 상태에서 모션을 `스윙 (슈터 공)`으로 두면 "IK 가능"이 뜨고 미리보기가 재생된다
- [ ] 슈터 pitch를 −25°, speed를 3으로 내리면 "도달하는 공이 없습니다"가 뜨고 미리보기 버튼이 눌러도 에러가 난다
