//! 발사 파라미터.

use super::layout;
use crate::constants::{ball, table};
use crate::defaults;
use crate::estimator;
use crate::estimator::HitPlane;
use nalgebra::Vector3;
use rand::Rng;
use rapier3d::prelude::{
    BroadPhaseBvh, CCDSolver, ColliderBuilder, ColliderSet, ImpulseJointSet, IntegrationParameters,
    IslandManager, MassProperties, MultibodyJointSet, NarrowPhase, PhysicsPipeline,
    RigidBodyBuilder, RigidBodySet, Rotation, Vector,
};

use crate::sim::physics::arm_bodies::{
    NET_HALF_THICKNESS_M, ball_collision_groups, static_collision_groups,
};

pub use crate::defaults::sim::{
    RANDOM_SHOT_FIXED_MUZZLE_HEIGHT_Z_M, RANDOM_SHOT_FIXED_MUZZLE_INSET_Y_M,
    RANDOM_SHOT_FIXED_PITCH_DEG, RANDOM_SHOT_FIXED_ROLL_DEG, RANDOM_SHOT_FIXED_YAW_DEGS,
    RANDOM_SHOT_HEIGHT_MAX_M, RANDOM_SHOT_HEIGHT_MIN_M, RANDOM_SHOT_LATERAL_MAX_M,
    RANDOM_SHOT_LATERAL_MIN_M, RANDOM_SHOT_NET_GATE_MAX_TRIES, RANDOM_SHOT_PITCH_MAX_DEG,
    RANDOM_SHOT_PITCH_MIN_DEG, RANDOM_SHOT_ROLL_MAX_DEG, RANDOM_SHOT_ROLL_MIN_DEG,
    RANDOM_SHOT_SIDESPIN_MAX, RANDOM_SHOT_SIDESPIN_MIN, RANDOM_SHOT_SPEED_MAX_MPS,
    RANDOM_SHOT_SPEED_MIN_MPS, RANDOM_SHOT_TARGET_PADDING_M, RANDOM_SHOT_TOPSPIN_MAX,
    RANDOM_SHOT_TOPSPIN_MIN,
};

/// GUI·런타임에서 조절하는 발사 파라미터.
#[derive(Debug, Clone, PartialEq)]
pub struct Settings {
    /// 초기 속도 크기 [m/s]
    pub speed_mps: f64,
    /// yaw [deg] — Z축 기준 좌우 조준 (0=로봇 정면, +x=우측)
    pub yaw_deg: f64,
    /// pitch [deg] — 위아래 조준 (0=수평, +=위, -=아래)
    pub pitch_deg: f64,
    /// roll [deg] — 발사축 기준 롤 (스핀 축·발사구 위치 회전)
    pub roll_deg: f64,
    /// 마운트 월드 오프셋 [m] — 기본 설치점(`layout::Layout::MOUNT_*`) 기준
    pub pos_offset_x_m: f64,
    pub pos_offset_y_m: f64,
    pub pos_offset_z_m: f64,
    /// 발사구 좌우 오프셋 [m] — 슈터 로컬 right
    pub lateral_offset_m: f64,
    /// 발사구 높이 오프셋 [m] — 슈터 로컬 up (본체 중심 기준)
    pub height_offset_m: f64,
    /// topspin [rad/s] — 슈터 로컬 right 축 (+=topspin)
    pub topspin_rad_s: f64,
    /// sidespin [rad/s] — 슈터 로컬 up 축
    pub sidespin_rad_s: f64,
    /// drill spin [rad/s] — 슈터 로컬 forward 축 (총구 축 회전)
    pub drill_spin_rad_s: f64,
}

impl Settings {
    /// 슈터 마운트 기준점 (월드) — 탄도·오프셋의 원점.
    pub fn mount_position(&self) -> Vector {
        return Vector::new(
            (layout::Layout::MOUNT_X + self.pos_offset_x_m) as f32,
            (layout::Layout::MOUNT_Y + self.pos_offset_y_m) as f32,
            (table::SURFACE_Z + layout::Layout::BODY_HEIGHT * 0.5 + self.pos_offset_z_m) as f32,
        );
    }

    /// 조준 방향 단위벡터 (월드). yaw=0, pitch=0 이면 -y.
    pub fn aim_direction(&self) -> Vector {
        let yaw = self.yaw_deg.to_radians() as f32;
        let pitch = self.pitch_deg.to_radians() as f32;
        let x = pitch.cos() * yaw.sin();
        let y = -pitch.cos() * yaw.cos();
        let z = pitch.sin();
        return Vector::new(x, y, z).normalize();
    }

    /// 슈터 로컬 (forward, right, up) — roll 반영.
    pub fn local_basis(&self) -> (Vector, Vector, Vector) {
        let forward = self.aim_direction();
        let world_up = Vector::new(0.0, 0.0, 1.0);
        let mut right = world_up.cross(forward);
        if right.length_squared() < 1e-8 {
            right = Vector::new(1.0, 0.0, 0.0);
        } else {
            right = right.normalize();
        }
        let mut up = forward.cross(right);
        up = up.normalize();

        let roll = self.roll_deg.to_radians() as f32;
        let (sin, cos) = roll.sin_cos();
        let right_r = right * cos + up * sin;
        let up_r = up * cos - right * sin;
        return (forward, right_r, up_r);
    }

    /// 슈터 본체 회전 (조준 + roll).
    pub fn orientation(&self) -> Rotation {
        let forward = self.aim_direction();
        let aim = Rotation::from_rotation_arc(Vector::new(0.0, -1.0, 0.0), forward);
        let roll = self.roll_deg.to_radians() as f32;
        let roll_q = Rotation::from_axis_angle(forward, roll);
        return roll_q * aim;
    }

    /// 발사구 위치 — 슈터 로컬 오프셋을 월드로 변환 (탄도 SSOT).
    pub fn muzzle_position(&self) -> Vector {
        let (forward, right, up) = self.local_basis();
        let local = forward * (layout::Layout::BARREL_FORWARD_M as f32)
            + up * self.height_offset_m as f32
            + right * self.lateral_offset_m as f32;
        return self.mount_position() + local;
    }

    /// 발사구(공 시작점)의 월드 절대 XYZ를 설정한다.
    ///
    /// 내부적으로는 마운트 오프셋을 쓰지만 GUI·시나리오에서는
    /// 직접 측정할 수 있는 공 중심 좌표를 입력하는 편이 안전하다.
    /// yaw/pitch/roll·발사구 로컬 오프셋은 그대로 유지한다.
    pub fn set_muzzle_position(&mut self, target: Vector) {
        let current = self.muzzle_position();
        self.pos_offset_x_m += f64::from(target.x - current.x);
        self.pos_offset_y_m += f64::from(target.y - current.y);
        self.pos_offset_z_m += f64::from(target.z - current.z);
    }

    /// [`Self::set_muzzle_position`]의 설정 파일/CLI 친화형 버전.
    pub fn set_muzzle_xyz(&mut self, x_m: f64, y_m: f64, z_m: f64) {
        self.set_muzzle_position(Vector::new(x_m as f32, y_m as f32, z_m as f32));
    }

    /// 뷰어 직육면체 중심 — 발사구가 전면에 오도록 조준축 뒤로 반 길이.
    pub fn visual_position(&self) -> Vector {
        let (forward, _, _) = self.local_basis();
        let half_depth = (layout::Layout::VISUAL_SIZE_Y * 0.5) as f32;
        return self.muzzle_position() - forward * half_depth;
    }

    /// 조준 방향 × 속도.
    pub fn launch_velocity(&self) -> Vector {
        return self.aim_direction() * self.speed_mps as f32;
    }

    /// 슈터 로컬 스핀 축을 월드 각속도로 변환.
    pub fn launch_angular_velocity(&self) -> Vector {
        let (forward, right, up) = self.local_basis();
        return right * self.topspin_rad_s as f32
            + up * self.sidespin_rad_s as f32
            + forward * self.drill_spin_rad_s as f32;
    }

    /// 좌우 발사 위치(`lateral_offset_m`)에서 로봇쪽 테이블 가장자리(y=0)의
    /// padding 안쪽 구간 전체를 조준하는 데 필요한 yaw 범위 [deg] — `(최소, 최대)`.
    ///
    /// 발사 위치가 정해지면 "테이블 위 어딘가를 조준한다"는 조건만으로 yaw
    /// 범위가 기하학적으로 결정된다: 좌우 padding을 둔 반대편 가장자리
    /// 양 끝을 잇는 선까지의 각도. 이 범위 안에서 뽑으면 좌우 위치가 다른
    /// 두 샷이 진짜로 다른 궤적(다른 각도)이 된다 — `lateral_offset_m`만
    /// 바꾸는 평행이동과 달리.
    pub(crate) fn yaw_range_for_lateral_deg(lateral_offset_m: f64) -> (f64, f64) {
        return Self::yaw_range_for_mount_deg(
            layout::Layout::MOUNT_X + lateral_offset_m,
            layout::Layout::MOUNT_Y,
        );
    }

    /// 마운트 (x,y)에서 로봇쪽 테이블 padding 안쪽을 조준하는 yaw 범위 [deg].
    pub(crate) fn yaw_range_for_mount_deg(mount_x: f64, mount_y: f64) -> (f64, f64) {
        let yaw_deg_for_target_x = |target_x: f64| -> f64 {
            let dx = target_x - mount_x;
            let dy = 0.0 - mount_y;
            return dx.atan2(-dy).to_degrees();
        };
        let yaw_left = yaw_deg_for_target_x(RANDOM_SHOT_TARGET_PADDING_M);
        let yaw_right = yaw_deg_for_target_x(table::WIDTH_X - RANDOM_SHOT_TARGET_PADDING_M);
        return (yaw_left.min(yaw_right), yaw_left.max(yaw_right));
    }

    /// 좌우·yaw·속도만 안전 범위 안에서 랜덤화한다.
    ///
    /// 접수·리치 회귀 테스트용 — 높이·스핀·pitch/roll은 호출 시점 값을 유지한다.
    pub fn randomized_aim(&self, rng: &mut impl Rng) -> Self {
        let lateral_offset_m = rng.gen_range(RANDOM_SHOT_LATERAL_MIN_M..=RANDOM_SHOT_LATERAL_MAX_M);
        let mount_x = layout::Layout::MOUNT_X + self.pos_offset_x_m + lateral_offset_m;
        let mount_y = layout::Layout::MOUNT_Y + self.pos_offset_y_m;
        let (yaw_min, yaw_max) = Self::yaw_range_for_mount_deg(mount_x, mount_y);
        let yaw_deg = rng.gen_range(yaw_min..=yaw_max);
        let speed_mps = rng.gen_range(RANDOM_SHOT_SPEED_MIN_MPS..=RANDOM_SHOT_SPEED_MAX_MPS);
        return Self {
            lateral_offset_m,
            yaw_deg,
            speed_mps,
            ..self.clone()
        };
    }

    /// 발사 직후 탄도가 네트 게이트·hit-plane에 도달하는지 (ballistics + 스핀).
    ///
    /// Rapier와 같은 `PhysicsParams`(drag/magnus)로 적분한다. 바운스 후 궤적은
    /// Rapier와 어긋날 수 있으므로, 샘플 채택은 [`Self::clears_incoming_rapier_net`]도
    /// 같이 본다.
    pub fn clears_incoming_net_gate(&self) -> bool {
        let muzzle = self.muzzle_position();
        let vel = self.launch_velocity();
        let omega = self.launch_angular_velocity();
        let position = Vector3::new(
            f64::from(muzzle.x),
            f64::from(muzzle.y),
            f64::from(muzzle.z),
        );
        let velocity = Vector3::new(f64::from(vel.x), f64::from(vel.y), f64::from(vel.z));
        let spin = Vector3::new(f64::from(omega.x), f64::from(omega.y), f64::from(omega.z));
        let plane = HitPlane {
            y: table::DEFAULT_HIT_PLANE_Y,
        };
        return crate::estimator::Kinematics::predict_to(
            position,
            velocity,
            spin,
            plane,
            &defaults::PhysicsParams::default(),
        )
        .is_some();
    }

    /// 테이블·네트·공만 있는 가벼운 Rapier로, 수신 탄도가 **네트 collider에
    /// 닿지 않고** 네트 y를 넘어가는지 확인한다.
    ///
    /// ballistics 게이트만으로는 바운스 후 높이 오차로 “네트 위 튕김” 샘플이
    /// 통과했다. GUI Random / Eval 샘플 SSOT.
    pub fn clears_incoming_rapier_net(&self) -> bool {
        return !contacts_incoming_rapier_net(self);
    }

    fn sample_randomized_params(&self, rng: &mut impl Rng) -> Self {
        let yaw_deg =
            RANDOM_SHOT_FIXED_YAW_DEGS[rng.gen_range(0..RANDOM_SHOT_FIXED_YAW_DEGS.len())];
        let mut shot = self.with_fixed_random_geometry(yaw_deg);
        shot.speed_mps = rng.gen_range(RANDOM_SHOT_SPEED_MIN_MPS..=RANDOM_SHOT_SPEED_MAX_MPS);
        shot.topspin_rad_s = rng.gen_range(RANDOM_SHOT_TOPSPIN_MIN..=RANDOM_SHOT_TOPSPIN_MAX);
        shot.sidespin_rad_s = rng.gen_range(RANDOM_SHOT_SIDESPIN_MIN..=RANDOM_SHOT_SIDESPIN_MAX);
        return shot;
    }

    /// GUI Random용 고정 발사구·자세 — 발사구를 `(W/2, L−inset, 면+height)`에
    /// 두고 pitch/roll을 고정한다. yaw만 호출자가 고른다.
    ///
    /// `BARREL_FORWARD_M` 때문에 조준각이 바뀌면 발사구가 움직이므로, 마운트
    /// 오프셋으로 보정해서 발사구 절대 좌표를 유지한다.
    fn with_fixed_random_geometry(&self, yaw_deg: f64) -> Self {
        let mut shot = self.clone();
        shot.yaw_deg = yaw_deg;
        shot.pitch_deg = RANDOM_SHOT_FIXED_PITCH_DEG;
        shot.roll_deg = RANDOM_SHOT_FIXED_ROLL_DEG;
        shot.lateral_offset_m = 0.0;
        shot.height_offset_m = 0.0;

        let muzzle_x = table::WIDTH_X * 0.5;
        let muzzle_y = table::LENGTH_Y - RANDOM_SHOT_FIXED_MUZZLE_INSET_Y_M;
        let muzzle_z = table::SURFACE_Z + RANDOM_SHOT_FIXED_MUZZLE_HEIGHT_Z_M;
        // height=lateral=0 → muzzle = mount + barrel·forward
        let forward = shot.aim_direction();
        let barrel = layout::Layout::BARREL_FORWARD_M as f32;
        let mount_x = muzzle_x as f32 - forward.x * barrel;
        let mount_y = muzzle_y as f32 - forward.y * barrel;
        let mount_z = muzzle_z as f32 - forward.z * barrel;

        shot.pos_offset_x_m = f64::from(mount_x) - layout::Layout::MOUNT_X;
        shot.pos_offset_y_m = f64::from(mount_y) - layout::Layout::MOUNT_Y;
        shot.pos_offset_z_m =
            f64::from(mount_z) - (table::SURFACE_Z + layout::Layout::BODY_HEIGHT * 0.5);
        return shot;
    }

    /// 위치·자세는 실측 고정값, yaw∈{−10,0,10}, 속도·스핀만 랜덤.
    ///
    /// ballistics 네트 게이트 **그리고** Rapier 네트 비접촉을 통과한 샘플만 반환.
    /// drill spin은 호출 시점 값 그대로 유지된다.
    pub fn randomized(&self, rng: &mut impl Rng) -> Self {
        for _ in 0..RANDOM_SHOT_NET_GATE_MAX_TRIES {
            let shot = self.sample_randomized_params(rng);
            if shot.clears_incoming_net_gate() && shot.clears_incoming_rapier_net() {
                return shot;
            }
        }
        // 최후: 고정 기하 + 이산 yaw, 속도·스핀은 Rapier 통과가 확인된 기본값.
        let defaults = Self::default();
        let yaw_deg =
            RANDOM_SHOT_FIXED_YAW_DEGS[rng.gen_range(0..RANDOM_SHOT_FIXED_YAW_DEGS.len())];
        let mut shot = self.with_fixed_random_geometry(yaw_deg);
        shot.topspin_rad_s = defaults.topspin_rad_s;
        shot.sidespin_rad_s = defaults.sidespin_rad_s;
        shot.speed_mps = defaults.speed_mps;
        return shot;
    }

    /// 현재 GUI에서 정한 공 발사 절대 위치는 유지하고 나머지 샷을 랜덤화한다.
    pub fn randomized_at_current_muzzle(&self, rng: &mut impl Rng) -> Self {
        let muzzle = self.muzzle_position();
        for _ in 0..RANDOM_SHOT_NET_GATE_MAX_TRIES {
            let mut shot = self.sample_randomized_params(rng);
            shot.set_muzzle_position(muzzle);
            if shot.clears_incoming_net_gate() && shot.clears_incoming_rapier_net() {
                return shot;
            }
        }

        // 사용자가 네트보다 낮은 등 불가능한 위치를 골라도 위치 설정 자체를
        // 몰래 되돌리지 않는다. 발사 후 결과를 그대로 시뮬에서 확인하게 한다.
        let mut shot = self.randomized(rng);
        shot.set_muzzle_position(muzzle);
        return shot;
    }
}

/// 공에 항력·Magnus 외력을 건다 (중력은 Rapier gravity).
///
/// `SimWorld::apply_ball_aero_forces`와 같은 식 — 게이트 미니월드와 본 시뮬의
/// 탄도를 맞추기 위한 것이다.
fn apply_aero_force(
    body: &mut rapier3d::prelude::RigidBody,
    physics: &crate::defaults::PhysicsParams,
) {
    body.reset_forces(true);
    let lin = body.linvel();
    let ang = body.angvel();
    let velocity = Vector3::new(f64::from(lin.x), f64::from(lin.y), f64::from(lin.z));
    let omega = Vector3::new(f64::from(ang.x), f64::from(ang.y), f64::from(ang.z));
    let mass = f64::from(body.mass());
    if mass <= 1e-12 {
        return;
    }
    let force =
        estimator::Kinematics::aero_accel(velocity, omega, physics.drag, physics.magnus) * mass;
    body.add_force(
        Vector::new(force.x as f32, force.y as f32, force.z as f32),
        true,
    );
}

/// 테이블+네트+공만으로 수신 탄도의 네트 접촉 여부 (팔/라켓 없음).
fn contacts_incoming_rapier_net(settings: &Settings) -> bool {
    let physics = defaults::PhysicsParams::default();
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut impulse_joints = ImpulseJointSet::new();
    let mut multibody_joints = MultibodyJointSet::new();
    let mut islands = IslandManager::new();
    let mut broad = BroadPhaseBvh::new();
    let mut narrow = NarrowPhase::new();
    let mut ccd = CCDSolver::new();
    let mut pipeline = PhysicsPipeline::new();
    let mut integration = IntegrationParameters::default();
    // SimWorld 회귀·GUI 스텝과 동일 — default dt(≈1/60)면 바운스 후 네트 판정이 어긋난다.
    integration.dt = 1.0 / 1000.0;
    integration.num_solver_iterations = 12;
    let gravity = Vector::new(0.0, 0.0, -9.81);

    let table_cx = (table::WIDTH_X * 0.5) as f32;
    let table_cy = (table::LENGTH_Y * 0.5) as f32;
    let table_body = RigidBodyBuilder::fixed()
        .translation(Vector::new(
            table_cx,
            table_cy,
            (table::SURFACE_Z - table::HALF_THICKNESS) as f32,
        ))
        .build();
    let table_handle = bodies.insert(table_body);
    colliders.insert_with_parent(
        ColliderBuilder::cuboid(
            (table::WIDTH_X * 0.5) as f32,
            (table::LENGTH_Y * 0.5) as f32,
            table::HALF_THICKNESS as f32,
        )
        .collision_groups(static_collision_groups())
        .restitution(physics.restitution as f32)
        .friction(physics.friction as f32)
        .build(),
        table_handle,
        &mut bodies,
    );

    let net_body = RigidBodyBuilder::fixed()
        .translation(Vector::new(
            table_cx,
            table_cy,
            (table::SURFACE_Z + table::NET_HEIGHT * 0.5) as f32,
        ))
        .build();
    let net_handle = bodies.insert(net_body);
    let net_collider = colliders.insert_with_parent(
        // 본 시뮬과 동일: soft 실체 네트 (관통 없음).
        crate::sim::physics::arm_bodies::net_collider_builder(&physics).build(),
        net_handle,
        &mut bodies,
    );

    let muzzle = settings.muzzle_position();
    let linvel = settings.launch_velocity();
    let angvel = settings.launch_angular_velocity();
    let ball_body = RigidBodyBuilder::dynamic()
        .translation(muzzle)
        .linvel(linvel)
        .angvel(angvel)
        .ccd_enabled(true)
        .angular_damping(ball::ANGULAR_DAMPING as f32)
        .build();
    let ball_handle = bodies.insert(ball_body);
    let ball_collider = colliders.insert_with_parent(
        ColliderBuilder::ball(ball::RADIUS as f32)
            .collision_groups(ball_collision_groups())
            .restitution(physics.restitution as f32)
            .friction(physics.ball_friction as f32)
            .mass_properties(MassProperties::new(
                Vector::ZERO,
                ball::MASS as f32,
                Vector::splat(ball::SHELL_INERTIA as f32),
            ))
            .build(),
        ball_handle,
        &mut bodies,
    );

    let net_y = (table::LENGTH_Y * 0.5) as f32;
    let mut previous_y = muzzle.y;
    for _ in 0..4_000 {
        // SimWorld::apply_ball_aero_forces와 동일 — 항력·Magnus 없이 적분하면
        // 공이 덜 처져 "네트를 넘는다"고 오판한다.
        apply_aero_force(&mut bodies[ball_handle], &physics);
        pipeline.step(
            gravity,
            &integration,
            &mut islands,
            &mut broad,
            &mut narrow,
            &mut bodies,
            &mut colliders,
            &mut impulse_joints,
            &mut multibody_joints,
            &mut ccd,
            &(),
            &(),
        );
        if narrow
            .contact_pair(ball_collider, net_collider)
            .is_some_and(|pair| pair.has_any_active_contact())
        {
            return true;
        }
        let y = bodies[ball_handle].translation().y;
        // 공 중심이 네트 평면을 지나도 아직 접촉할 수 있다 — 네트 상단을
        // 스치는 공은 중심 y가 평면보다 작아진 뒤에 접촉이 잡힌다(관측 y≈1.359,
        // 평면 1.37). 네트 영향권을 완전히 벗어난 뒤에야 통과로 판정한다.
        let net_clear_y = net_y - (NET_HALF_THICKNESS_M + ball::RADIUS as f32);
        if previous_y > net_clear_y && y <= net_clear_y {
            return false;
        }
        let v = bodies[ball_handle].linvel();
        let speed = (v.x * v.x + v.y * v.y + v.z * v.z).sqrt();
        // 테이블에 붙어 멈춘 경우 등.
        if speed < 0.05 && y > net_y {
            return false;
        }
        previous_y = y;
    }
    return false;
}

#[cfg(test)]
mod tests {
    use super::*;
    use rand::SeedableRng;

    #[test]
    fn visual_body_sits_outside_table_end() {
        let s = Settings::default();
        let visual = s.visual_position();
        // 본체 중심은 테이블 끝 밖(+y).
        assert!(
            visual.y > table::LENGTH_Y as f32,
            "visual center should be past table end, y={}",
            visual.y
        );
        let muzzle = s.muzzle_position();
        // 발사구는 끝선 근처(테이블 위/경계).
        assert!(
            (muzzle.y - table::LENGTH_Y as f32).abs() < 0.05,
            "muzzle should be near table end, y={}",
            muzzle.y
        );
    }

    #[test]
    fn visual_front_face_matches_muzzle() {
        let s = Settings::default();
        let (forward, _, _) = s.local_basis();
        let front = s.visual_position() + forward * (layout::Layout::VISUAL_SIZE_Y * 0.5) as f32;
        let muzzle = s.muzzle_position();
        assert!(
            (front - muzzle).length_squared() < 1e-10,
            "front={front:?} muzzle={muzzle:?}"
        );
    }

    #[test]
    fn default_aims_toward_robot_with_slight_drop() {
        let s = Settings::default();
        let dir = s.aim_direction();
        assert!(dir.y < 0.0);
        assert!(dir.z < 0.0);
        assert!(dir.x.abs() < 1e-5);
    }

    #[test]
    fn yaw_deflects_toward_plus_x() {
        let mut s = Settings::default();
        s.yaw_deg = 10.0;
        s.pitch_deg = 0.0;
        let dir = s.aim_direction();
        assert!(dir.x > 0.0);
        assert!(dir.y < 0.0);
    }

    #[test]
    fn launch_velocity_matches_speed_and_aim() {
        let s = Settings {
            speed_mps: 10.0,
            ..Default::default()
        };
        let v = s.launch_velocity();
        assert!((v.length() - 10.0).abs() < 1e-4);
        assert!(v.y < 0.0);
    }

    #[test]
    fn absolute_muzzle_position_moves_ball_start_without_changing_aim() {
        let mut settings = Settings::default();
        let before_aim = settings.aim_direction();
        let target = Vector::new(0.31, 2.42, 1.18);
        settings.set_muzzle_position(target);
        assert!((settings.muzzle_position() - target).length() < 1e-5);
        assert!((settings.aim_direction() - before_aim).length() < 1e-7);
    }

    #[test]
    fn topspin_is_around_local_right() {
        let s = Settings {
            topspin_rad_s: 30.0,
            ..Default::default()
        };
        let w = s.launch_angular_velocity();
        assert!(w.length() > 0.0);
    }

    #[test]
    fn yaw_range_shifts_toward_the_farther_edge() {
        // 발사 위치가 오른쪽(+x)으로 치우칠수록: 가까운 오른쪽 padding 가장자리는
        // 거의 정면(yaw_max가 0에 가까워짐)이고, 먼 왼쪽 padding 가장자리는 더
        // 비스듬한 각도(yaw_min이 더 음수)가 필요하다. 왼쪽으로 치우치면 반대.
        let (left_min, left_max) = Settings::yaw_range_for_lateral_deg(-0.5);
        let (center_min, center_max) = Settings::yaw_range_for_lateral_deg(0.0);
        let (right_min, right_max) = Settings::yaw_range_for_lateral_deg(0.5);

        assert!(right_min < center_min && center_min < left_min);
        assert!(right_max < center_max && center_max < left_max);
        // 중앙 발사에서는 좌우 padding이 같으니 범위도 원점 대칭이어야 한다.
        assert!((center_min + center_max).abs() < 1e-6);
    }

    #[test]
    fn randomized_pins_muzzle_pose_and_varies_speed_spin() {
        let base = Settings {
            pitch_deg: -7.0,
            roll_deg: 12.0,
            height_offset_m: 0.05,
            topspin_rad_s: 3.0,
            sidespin_rad_s: -2.0,
            drill_spin_rad_s: 1.0,
            pos_offset_x_m: 0.1,
            pos_offset_y_m: -0.1,
            pos_offset_z_m: 0.05,
            ..Default::default()
        };
        let mut rng = rand::thread_rng();
        let mut seen_yaw = [false; 3];
        for _ in 0..60 {
            let shot = base.randomized(&mut rng);
            assert!(
                (RANDOM_SHOT_SPEED_MIN_MPS..=RANDOM_SHOT_SPEED_MAX_MPS).contains(&shot.speed_mps)
            );
            assert!(
                (RANDOM_SHOT_TOPSPIN_MIN..=RANDOM_SHOT_TOPSPIN_MAX).contains(&shot.topspin_rad_s)
            );
            assert!(
                (RANDOM_SHOT_SIDESPIN_MIN..=RANDOM_SHOT_SIDESPIN_MAX)
                    .contains(&shot.sidespin_rad_s)
            );
            assert!((shot.pitch_deg - RANDOM_SHOT_FIXED_PITCH_DEG).abs() < 1e-12);
            assert!((shot.roll_deg - RANDOM_SHOT_FIXED_ROLL_DEG).abs() < 1e-12);
            assert!((shot.lateral_offset_m).abs() < 1e-12);
            assert!((shot.height_offset_m).abs() < 1e-12);
            assert!(
                RANDOM_SHOT_FIXED_YAW_DEGS
                    .iter()
                    .any(|&y| (shot.yaw_deg - y).abs() < 1e-12),
                "yaw must be one of {:?}, got {}",
                RANDOM_SHOT_FIXED_YAW_DEGS,
                shot.yaw_deg
            );
            for (i, &y) in RANDOM_SHOT_FIXED_YAW_DEGS.iter().enumerate() {
                if (shot.yaw_deg - y).abs() < 1e-12 {
                    seen_yaw[i] = true;
                }
            }

            let muzzle = shot.muzzle_position();
            assert!(
                (f64::from(muzzle.x) - table::WIDTH_X * 0.5).abs() < 1e-4,
                "x={}",
                muzzle.x
            );
            assert!(
                (f64::from(muzzle.y) - (table::LENGTH_Y - RANDOM_SHOT_FIXED_MUZZLE_INSET_Y_M))
                    .abs()
                    < 1e-4,
                "y={}",
                muzzle.y
            );
            assert!(
                (f64::from(muzzle.z) - (table::SURFACE_Z + RANDOM_SHOT_FIXED_MUZZLE_HEIGHT_Z_M))
                    .abs()
                    < 1e-4,
                "z={}",
                muzzle.z
            );

            assert_eq!(shot.drill_spin_rad_s, base.drill_spin_rad_s);
            assert!(
                shot.clears_incoming_net_gate(),
                "randomized는 네트 게이트를 통과하는 샷만 반환해야 함: {shot:?}"
            );
            assert!(
                shot.clears_incoming_rapier_net(),
                "randomized는 Rapier 네트 비접촉 샷만 반환해야 함: {shot:?}"
            );
        }
        assert!(
            seen_yaw.iter().all(|&v| v),
            "60회면 세 yaw를 모두 봐야 함: {seen_yaw:?}"
        );
    }

    #[test]
    fn sample_without_gate_often_clips_net_but_randomized_does_not() {
        let base = Settings::default();
        let mut rng = rand::rngs::StdRng::seed_from_u64(42);
        let mut raw_clips = 0;
        for _ in 0..80 {
            let raw = base.sample_randomized_params(&mut rng);
            // 의도적으로 낮은 높이·pitch로 내려 미달을 만든다.
            let mut low = raw;
            low.height_offset_m = -0.20;
            low.pitch_deg = -5.0;
            // 마운트 보정을 깨서 낮은 탄도로 만든다.
            low.pos_offset_z_m -= 0.30;
            if !low.clears_incoming_net_gate() || !low.clears_incoming_rapier_net() {
                raw_clips += 1;
            }
        }
        assert!(
            raw_clips > 5,
            "전제: 필터 없는 낮은 샘플 중 네트 미달이 있어야 함 (clips={raw_clips})"
        );

        let mut rng = rand::rngs::StdRng::seed_from_u64(42);
        for _ in 0..40 {
            let shot = base.randomized(&mut rng);
            assert!(
                shot.clears_incoming_net_gate() && shot.clears_incoming_rapier_net(),
                "필터 후 샷이 네트 미달/접촉: {shot:?}"
            );
        }
    }

    #[test]
    fn default_shot_clears_rapier_net() {
        let shot = Settings::default();
        assert!(shot.clears_incoming_net_gate(), "default ballistics");
        assert!(shot.clears_incoming_rapier_net(), "default rapier net");
    }

    #[test]
    fn gui_random_keeps_user_configured_launch_position() {
        let mut base = Settings::default();
        base.set_muzzle_xyz(0.41, 2.38, 1.22);
        let mut rng = rand::rngs::StdRng::seed_from_u64(73);
        let shot = base.randomized_at_current_muzzle(&mut rng);
        let muzzle = shot.muzzle_position();
        assert!((f64::from(muzzle.x) - 0.41).abs() < 1e-5);
        assert!((f64::from(muzzle.y) - 2.38).abs() < 1e-5);
        assert!((f64::from(muzzle.z) - 1.22).abs() < 1e-5);
    }
}
