//! Rapier3d 시뮬레이션 월드.
//!
//! 탁구대·로봇(-x) · 슈터(+x) · 공. 공은 슈터에 주차되어 있다가
//! GUI 트리거로 발사되고, 로봇이 라켓으로 받는다.

use std::sync::Arc;

use pingpong_domain::{
    Arm, RobotState,
    constants::{ball, table},
};
use rapier3d::prelude::*;

use super::rapier_convert::racket_pose_to_rapier;
use super::shooter::{BallShooterSettings, BallState, ShooterLayout};

/// 한 물리 스텝 입력 — `controls` 뮤텍스를 물리 연산 동안 잡지 않기 위함.
pub struct SimStepInput<'a> {
    /// 현재 슈터 설정
    pub shooter: &'a BallShooterSettings,
    /// 이번 스텝에 발사
    pub shoot: bool,
    /// 이번 스텝에 주차
    pub park: bool,
}

/// Rapier 물리 월드 — 탁구대, 슈터, 공, 키네마틱 라켓.
pub struct SimWorld {
    /// 적분 스텝 설정
    pub integration_parameters: IntegrationParameters,
    /// 물리 파이프라인
    pub physics_pipeline: PhysicsPipeline,
    /// 섬(island) 관리
    pub island_manager: IslandManager,
    /// broad-phase 충돌 검사
    pub broad_phase: BroadPhaseBvh,
    /// narrow-phase 충돌 검사
    pub narrow_phase: NarrowPhase,
    /// 강체 집합
    pub rigid_body_set: RigidBodySet,
    /// 콜라이더 집합
    pub collider_set: ColliderSet,
    /// 임펄스 조인트
    pub impulse_joint_set: ImpulseJointSet,
    /// 멀티바디 조인트
    pub multibody_joint_set: MultibodyJointSet,
    /// 연속 충돌 검출(CCD) 솔버
    pub ccd_solver: CCDSolver,
    /// 중력 벡터
    pub gravity: Vector,
    /// 공 강체 핸들
    pub ball_handle: RigidBodyHandle,
    /// 라켓 강체 핸들
    pub racket_handle: RigidBodyHandle,
    /// 슈터 본체 (고정)
    pub shooter_handle: RigidBodyHandle,
    /// 불변 로봇 기구 모델
    pub arm: Arc<Arm>,
    /// URDF 기반 FK·뷰어 (선택)
    pub urdf: Option<Arc<crate::urdf::UrdfRobot>>,
    /// 런타임 관절 상태
    pub robot: RobotState,
    /// sim 경과 시간 [s]
    pub sim_time: f64,
    /// 공 주차/비행
    pub ball_state: BallState,
    /// 마지막 발사 설정 (상태 표시용)
    pub last_shooter_settings: BallShooterSettings,
}

impl SimWorld {
    /// 탁구대·슈터·주차된 공·로봇 라켓을 배치한다.
    pub fn new(arm: Arc<Arm>, urdf: Option<Arc<crate::urdf::UrdfRobot>>) -> Self {
        let mut integration_parameters = IntegrationParameters::default();
        integration_parameters.dt = 1.0 / 1000.0;

        let mut rigid_body_set = RigidBodySet::new();
        let mut collider_set = ColliderSet::new();

        let robot = if let Some(ref model) = urdf {
            RobotState::new(model.default_joints())
        } else {
            arm.initial_state()
        };
        let initial_pose = if let Some(ref model) = urdf {
            model
                .end_effector_pose_in_sim(robot.joints().values.as_slice())
                .expect("URDF 초기 FK")
        } else {
            robot.racket_pose(&arm).expect("초기 FK")
        };
        let (racket_pos, racket_rot) = racket_pose_to_rapier(&initial_pose);

        let table_z = (table::SURFACE_Z - table::HALF_THICKNESS) as f32;
        let table_cx = (table::WIDTH_X * 0.5) as f32;
        let table_cy = (table::LENGTH_Y * 0.5) as f32;
        let table_body = RigidBodyBuilder::fixed()
            .translation(Vector::new(table_cx, table_cy, table_z))
            .build();
        let table_handle = rigid_body_set.insert(table_body);
        let table_collider = ColliderBuilder::cuboid(
            (table::WIDTH_X * 0.5) as f32,
            (table::LENGTH_Y * 0.5) as f32,
            table::HALF_THICKNESS as f32,
        )
        .restitution(0.85)
        .friction(0.4)
        .build();
        collider_set.insert_with_parent(table_collider, table_handle, &mut rigid_body_set);

        let net_body = RigidBodyBuilder::fixed()
            .translation(Vector::new(
                table_cx,
                table_cy,
                (table::SURFACE_Z + table::NET_HEIGHT * 0.5) as f32,
            ))
            .build();
        let net_handle = rigid_body_set.insert(net_body);
        let net_collider = ColliderBuilder::cuboid(
            (table::WIDTH_X * 0.5) as f32,
            0.005,
            (table::NET_HEIGHT * 0.5) as f32,
        )
        .restitution(0.3)
        .build();
        collider_set.insert_with_parent(net_collider, net_handle, &mut rigid_body_set);

        // 슈터 기계 (+y, 로봇 반대편)
        let shooter_z = (table::SURFACE_Z + ShooterLayout::BODY_HEIGHT * 0.5) as f32;
        let shooter_body = RigidBodyBuilder::fixed()
            .translation(Vector::new(
                ShooterLayout::MOUNT_X as f32,
                ShooterLayout::MOUNT_Y as f32,
                shooter_z,
            ))
            .build();
        let shooter_handle = rigid_body_set.insert(shooter_body);
        let shooter_collider = ColliderBuilder::cuboid(0.12, 0.25, 0.18)
            .restitution(0.2)
            .friction(0.6)
            .build();
        collider_set.insert_with_parent(shooter_collider, shooter_handle, &mut rigid_body_set);

        let default_shooter = BallShooterSettings::default();

        let racket_body = RigidBodyBuilder::kinematic_position_based()
            .pose(Pose::from_parts(racket_pos, racket_rot))
            .build();
        let racket_handle = rigid_body_set.insert(racket_body);
        let racket_collider = ColliderBuilder::cuboid(0.08, 0.09, 0.006)
            .restitution(0.75)
            .friction(0.5)
            .build();
        collider_set.insert_with_parent(racket_collider, racket_handle, &mut rigid_body_set);

        let muzzle = default_shooter.muzzle_position();
        let ball_body = RigidBodyBuilder::fixed().translation(muzzle).build();
        let ball_handle = rigid_body_set.insert(ball_body);
        let ball_collider = ColliderBuilder::ball(ball::RADIUS as f32)
            .restitution(0.88)
            .friction(0.2)
            .density(0.25)
            .build();
        collider_set.insert_with_parent(ball_collider, ball_handle, &mut rigid_body_set);

        let mut world = Self {
            integration_parameters,
            physics_pipeline: PhysicsPipeline::new(),
            island_manager: IslandManager::new(),
            broad_phase: BroadPhaseBvh::new(),
            narrow_phase: NarrowPhase::new(),
            rigid_body_set,
            collider_set,
            impulse_joint_set: ImpulseJointSet::new(),
            multibody_joint_set: MultibodyJointSet::new(),
            ccd_solver: CCDSolver::new(),
            gravity: Vector::new(0.0, 0.0, -9.81),
            ball_handle,
            racket_handle,
            shooter_handle,
            arm,
            urdf,
            robot,
            sim_time: 0.0,
            ball_state: BallState::Parked,
            last_shooter_settings: default_shooter.clone(),
        };
        world.sync_shooter_pose(&default_shooter);
        return world;
    }

    /// 물리 1스텝: GUI 요청 처리 → 관절 추종 → Rapier 적분 → 이탈 공 회수.
    pub fn step(&mut self, dt: f64, input: Option<SimStepInput<'_>>) {
        if let Some(input) = input {
            if input.park {
                self.park_ball(input.shooter);
            }
            if input.shoot {
                self.shoot_ball(input.shooter);
            }
            self.sync_shooter_pose(input.shooter);
            if self.ball_state == BallState::Parked {
                self.sync_parked_ball(input.shooter);
            }
        }

        self.robot.step_toward_targets(&self.arm, dt);
        self.sync_racket_kinematic();

        self.physics_pipeline.step(
            self.gravity,
            &self.integration_parameters,
            &mut self.island_manager,
            &mut self.broad_phase,
            &mut self.narrow_phase,
            &mut self.rigid_body_set,
            &mut self.collider_set,
            &mut self.impulse_joint_set,
            &mut self.multibody_joint_set,
            &mut self.ccd_solver,
            &(),
            &(),
        );

        self.sim_time += dt;

        if self.ball_state == BallState::InFlight {
            self.park_if_out_of_play();
        }
    }

    /// 슈터 본체 위치·회전을 설정에 맞춘다.
    pub fn sync_shooter_pose(&mut self, settings: &BallShooterSettings) {
        let pos = settings.mount_position();
        let rot = settings.orientation();
        if let Some(body) = self.rigid_body_set.get_mut(self.shooter_handle) {
            body.set_translation(pos, true);
            body.set_rotation(rot, true);
        }
    }

    /// 주차 중인 공을 발사구에 붙인다.
    fn sync_parked_ball(&mut self, settings: &BallShooterSettings) {
        let muzzle = settings.muzzle_position();
        if let Some(body) = self.rigid_body_set.get_mut(self.ball_handle) {
            body.set_translation(muzzle, true);
        }
    }

    /// 슈터에서 공을 발사한다.
    pub fn shoot_ball(&mut self, settings: &BallShooterSettings) {
        self.sync_shooter_pose(settings);
        self.last_shooter_settings = settings.clone();
        let muzzle = settings.muzzle_position();
        let linvel = settings.launch_velocity();
        let angvel = settings.launch_angular_velocity();

        if let Some(body) = self.rigid_body_set.get_mut(self.ball_handle) {
            body.set_body_type(RigidBodyType::Dynamic, true);
            body.set_translation(muzzle, true);
            body.set_linvel(linvel, true);
            body.set_angvel(angvel, true);
            body.enable_ccd(true);
        }
        self.ball_state = BallState::InFlight;
    }

    /// 공을 슈터 발사구에 주차한다.
    pub fn park_ball(&mut self, settings: &BallShooterSettings) {
        self.last_shooter_settings = settings.clone();
        self.sync_shooter_pose(settings);
        let muzzle = settings.muzzle_position();
        if let Some(body) = self.rigid_body_set.get_mut(self.ball_handle) {
            body.set_body_type(RigidBodyType::Fixed, true);
            body.set_translation(muzzle, true);
            body.set_linvel(Vector::ZERO, true);
            body.set_angvel(Vector::ZERO, true);
        }
        self.ball_state = BallState::Parked;
    }

    /// 테이블 밖·바닥으로 떨어진 공을 슈터로 회수한다.
    fn park_if_out_of_play(&mut self) {
        let pos = self.rigid_body_set[self.ball_handle].translation();
        let out = pos.x < -0.15
            || pos.x > (table::WIDTH_X + 0.15) as f32
            || pos.y < -0.15
            || pos.y > (table::LENGTH_Y + 0.15) as f32
            || pos.z < 0.35;

        if out {
            let settings = self.last_shooter_settings.clone();
            self.park_ball(&settings);
        }
    }

    /// 공 중심 위치 (Rapier 좌표).
    pub fn ball_position(&self) -> Vector {
        return self.rigid_body_set[self.ball_handle].translation();
    }

    /// 공 선속도.
    pub fn ball_velocity(&self) -> Vector {
        return self.rigid_body_set[self.ball_handle].linvel();
    }

    /// 슈터 본체 위치·회전 (kiss3d 동기화용).
    pub fn shooter_pose(&self) -> (Vector, Rotation) {
        let body = &self.rigid_body_set[self.shooter_handle];
        return (body.translation(), *body.rotation());
    }

    /// 라켓 강체 위치·회전.
    pub fn racket_pose(&self) -> (Vector, Rotation) {
        let body = &self.rigid_body_set[self.racket_handle];
        return (body.translation(), *body.rotation());
    }

    /// 불변 arm 모델.
    pub fn arm(&self) -> &Arm {
        return &self.arm;
    }

    /// 읽기 전용 로봇 상태.
    pub fn robot(&self) -> &RobotState {
        return &self.robot;
    }

    /// 변경 가능한 로봇 상태.
    pub fn robot_mut(&mut self) -> &mut RobotState {
        return &mut self.robot;
    }

    /// URDF 모델 (있으면 FK·뷰어에 사용).
    pub fn urdf(&self) -> Option<&crate::urdf::UrdfRobot> {
        return self.urdf.as_deref();
    }

    /// FK 결과로 키네마틱 라켓 위치를 갱신한다.
    fn sync_racket_kinematic(&mut self) {
        let pose = if let Some(model) = self.urdf.as_ref() {
            model.end_effector_pose_in_sim(self.robot.joints().values.as_slice())
        } else {
            self.robot.racket_pose(&self.arm)
        };
        let Some(pose) = pose else {
            return;
        };
        let (pos, rot) = racket_pose_to_rapier(&pose);
        if let Some(body) = self.rigid_body_set.get_mut(self.racket_handle) {
            body.set_next_kinematic_position(Pose::from_parts(pos, rot));
        }
    }
}

#[cfg(test)]
mod tests {
    use std::sync::Arc;

    use super::*;

    use pingpong_domain::{Arm, constants::table};

    fn test_arm() -> Arc<Arm> {
        return Arc::new(
            Arm::builder()
                .base_xyz(table::WIDTH_X * 0.15, 0.02, table::SURFACE_Z)
                .link(0.35)
                .revolute_at(-1.2, 1.2, 0.0)
                .link(0.30)
                .revolute_at(-0.2, 1.4, 0.6)
                .link(0.15)
                .revolute_at(-1.5, 0.5, -0.4)
                .max_joint_speed(2.5)
                .build()
                .expect("테스트용 3DOF arm"),
        );
    }

    #[test]
    fn ball_stays_parked_until_shoot() {
        let arm = test_arm();
        let mut world = SimWorld::new(arm, None);
        let y0 = world.ball_position().y;
        for _ in 0..200 {
            world.step(1.0 / 1000.0, None);
        }
        assert_eq!(world.ball_state, BallState::Parked);
        assert!((world.ball_position().y - y0).abs() < 1e-4);
    }

    #[test]
    fn shoot_sends_ball_toward_robot_side() {
        let arm = test_arm();
        let mut world = SimWorld::new(arm, None);
        let settings = BallShooterSettings::default();
        world.shoot_ball(&settings);
        let y0 = world.ball_position().y;
        for _ in 0..300 {
            world.step(1.0 / 1000.0, None);
        }
        assert_eq!(world.ball_state, BallState::InFlight);
        assert!(world.ball_position().y < y0);
    }
}
