//! Rapier3d 시뮬레이션 월드.
//!
//! 탁구대·로봇(-x) · 슈터(+x) · 공. 공은 슈터에 주차되어 있다가
//! GUI 트리거로 발사되고, 로봇이 라켓으로 받는다.

use std::sync::Arc;

use pingpong_domain::{
    Arm, DomainError, HitPlane, Prediction, RobotPose, RobotState, Target,
    ball_past_midcourt_for_commit, constants::{ball, table}, in_swing_commit_window, plan_swing,
};
use rapier3d::prelude::*;
use tracing::{debug, warn};

use super::ball_script::{BallAction, BallEvent, BallScript, BallVec3};
use crate::estimator::predict_impact;
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
    pub urdf: Option<Arc<crate::robot::urdf::UrdfRobot>>,
    /// 제어 관절 → URDF actuated 매핑 (`None`이면 앞쪽 truncate)
    control_to_urdf: Option<Vec<Option<usize>>>,
    /// 런타임 관절 상태
    pub robot: RobotState,
    /// sim 경과 시간 [s]
    pub sim_time: f64,
    /// 공 주차/비행
    pub ball_state: BallState,
    /// 마지막 발사 설정 (상태 표시용)
    pub last_shooter_settings: BallShooterSettings,
    /// `sim_time`에 실행할 공 동역학 이벤트
    pending_ball_events: Vec<BallEvent>,
    /// 디버그 — 마지막 hit plane 예측 (뷰어 마커용)
    debug_prediction: Option<Prediction>,
    /// 접수 평면 — 물리 스텝에서 즉시 스윙 계획에 사용
    hit_plane: HitPlane,
    /// true면 Rapier 진실 상태로 자동 스윙 (sim 기본).
    /// false면 카메라→DLT→EKF→control이 타격 (`--ekf-swing`).
    oracle_auto_swing: bool,
    /// 이번 비행에서 스윙을 이미 commit했는지 (재계획·팔 떨림 방지)
    swing_committed: bool,
}

impl SimWorld {
    /// 탁구대·슈터·주차된 공·로봇 라켓을 배치한다.
    ///
    /// 제어·Rapier 라켓은 항상 `arm` SSOT. URDF는 뷰어 mesh용이며
    /// `set_control_to_urdf`로 관절 매핑을 줄 수 있다.
    pub fn new(arm: Arc<Arm>, urdf: Option<Arc<crate::robot::urdf::UrdfRobot>>) -> Self {
        let mut integration_parameters = IntegrationParameters::default();
        integration_parameters.dt = 1.0 / 1000.0;

        let mut rigid_body_set = RigidBodySet::new();
        let mut collider_set = ColliderSet::new();

        // 제어 DOF = Arm. URDF default(예: 3축)로 초기화하면 plan_swing과 어긋난다.
        let robot = arm.initial_state();
        let initial_pose = robot.racket_pose(&arm).expect("초기 FK");
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
        .restitution(ball::TABLE_BOUNCE_RESTITUTION as f32)
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
        let racket_collider = ColliderBuilder::cuboid(0.06, 0.07, 0.005)
            .restitution(ball::RESTITUTION as f32)
            .friction(0.5)
            .build();
        collider_set.insert_with_parent(racket_collider, racket_handle, &mut rigid_body_set);

        let muzzle = default_shooter.muzzle_position();
        let ball_body = RigidBodyBuilder::fixed().translation(muzzle).build();
        let ball_handle = rigid_body_set.insert(ball_body);
        let ball_collider = ColliderBuilder::ball(ball::RADIUS as f32)
            .restitution(ball::TABLE_BOUNCE_RESTITUTION as f32)
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
            control_to_urdf: None,
            robot,
            sim_time: 0.0,
            ball_state: BallState::Parked,
            last_shooter_settings: default_shooter.clone(),
            pending_ball_events: Vec::new(),
            debug_prediction: None,
            hit_plane: HitPlane {
                y: table::DEFAULT_HIT_PLANE_Y,
            },
            oracle_auto_swing: true,
            swing_committed: false,
        };
        world.sync_shooter_pose(&default_shooter);
        return world;
    }

    /// 제어→URDF 관절 매핑 (뷰어 FK). `None`이면 truncate fallback.
    pub fn set_control_to_urdf(&mut self, map: Option<Vec<Option<usize>>>) {
        self.control_to_urdf = map;
    }

    /// 뷰어용 URDF 관절각 (제어 `Joints` + 매핑).
    pub fn urdf_joint_values(&self) -> Option<Vec<f64>> {
        let urdf = self.urdf.as_ref()?;
        let sources = self.control_to_urdf.as_deref();
        return Some(crate::robot::urdf::map_control_joints_or_truncate(
            self.robot.joints().values.as_slice(),
            urdf.joint_count(),
            sources,
            0.0,
        ));
    }

    /// Rapier 진실 상태 자동 스윙 on/off.
    pub fn set_oracle_auto_swing(&mut self, enabled: bool) {
        self.oracle_auto_swing = enabled;
    }

    /// 진실 상태 기반 자동 스윙(오라클) 여부.
    pub fn oracle_auto_swing(&self) -> bool {
        return self.oracle_auto_swing;
    }

    /// 이번 공에 스윙을 이미 commit했는지.
    pub fn swing_committed(&self) -> bool {
        return self.swing_committed;
    }

    /// control/oracle이 스윙을 commit했음을 표시한다.
    pub fn mark_swing_committed(&mut self) {
        self.swing_committed = true;
    }

    /// 물리 1스텝: GUI 요청 처리 → 공 스크립트 → 관절 추종 → Rapier 적분.
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

        self.tick_ball_events();

        self.robot.step_toward_targets(&self.arm, dt);
        self.try_auto_swing();
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

    /// 접수 평면을 설정한다 (CLI·파이프라인과 동기화).
    pub fn set_hit_plane(&mut self, plane: HitPlane) {
        self.hit_plane = plane;
    }

    /// 공 비행 중 commit 창에 들어올 때 한 번만 `plan_swing`.
    ///
    /// 발사 직후(긴 lead)에는 대기하고, 네트 통과 후
    /// `time_to_impact ∈ [MIN_SWING, COMMIT_MAX]`일 때 시작한다.
    /// sim 기본은 오라클(진실 상태). `--ekf-swing`이면 control 경로가 타격.
    fn try_auto_swing(&mut self) {
        if !self.oracle_auto_swing {
            // 디버그 마커만 갱신
            if self.ball_state == BallState::InFlight {
                if let Some(prediction) = predict_impact(self, self.hit_plane) {
                    self.set_debug_prediction(Some(prediction));
                }
            }
            return;
        }
        if self.ball_state != BallState::InFlight {
            return;
        }
        if self.swing_committed || self.robot.is_swinging() {
            return;
        }
        let Some(prediction) = predict_impact(self, self.hit_plane) else {
            return;
        };
        self.set_debug_prediction(Some(prediction));

        // 상대 코트에 있으면 아직 이름 — 바운스·탄도 안정화 대기
        let ball_y = f64::from(self.ball_position().y);
        if !ball_past_midcourt_for_commit(ball_y) {
            return;
        }
        if !in_swing_commit_window(prediction.time_to_impact_secs) {
            return;
        }

        let start = RobotPose::new(self.robot.rail_x(), self.robot.joints().clone());
        let target = Target { prediction };
        let trajectory = match plan_swing(&self.arm, target, &start) {
            Ok(trajectory) => trajectory,
            Err(DomainError::InfeasibleSwing(ref err)) => {
                debug!(%err, "plan_swing 불가 — 이번 공 스킵");
                return;
            }
            Err(other) => {
                warn!(%other, "sim 자동 스윙 계획 실패");
                return;
            }
        };
        debug!(
            duration_secs = trajectory.duration_secs,
            rail_end = trajectory.rail.end,
            end_vel = ?trajectory.end_velocity,
            "sim plan_swing commit"
        );
        self.robot.replace_swing(trajectory);
        self.swing_committed = true;
    }

    /// 디버그용 hit plane 예측 (없으면 `None`).
    pub fn debug_prediction(&self) -> Option<&Prediction> {
        return self.debug_prediction.as_ref();
    }

    /// 디버그용 hit plane 예측을 갱신한다.
    pub fn set_debug_prediction(&mut self, prediction: Option<Prediction>) {
        self.debug_prediction = prediction;
    }

    /// 슈터에서 공을 발사한다.
    pub fn shoot_ball(&mut self, settings: &BallShooterSettings) {
        self.sync_shooter_pose(settings);
        self.last_shooter_settings = settings.clone();
        let muzzle = settings.muzzle_position();
        let linvel = settings.launch_velocity();
        let angvel = settings.launch_angular_velocity();
        self.launch_ball_at(
            BallVec3::new(muzzle.x, muzzle.y, muzzle.z),
            BallVec3::new(linvel.x, linvel.y, linvel.z),
            BallVec3::new(angvel.x, angvel.y, angvel.z),
        );
    }

    /// 위치·속도로 공을 dynamic 비행 상태로 만든다.
    pub fn launch_ball_at(
        &mut self,
        position: BallVec3,
        linear_velocity: BallVec3,
        angular_velocity: BallVec3,
    ) {
        if let Some(body) = self.rigid_body_set.get_mut(self.ball_handle) {
            body.set_body_type(RigidBodyType::Dynamic, true);
            body.set_translation(position.to_rapier(), true);
            body.set_linvel(linear_velocity.to_rapier(), true);
            body.set_angvel(angular_velocity.to_rapier(), true);
            body.enable_ccd(true);
        }
        self.ball_state = BallState::InFlight;
        self.robot.cancel_swing();
        self.swing_committed = false;
        self.try_auto_swing();
    }

    /// 선형 임펄스 [N·s]를 적용한다 (dynamic일 때만).
    pub fn apply_ball_impulse(&mut self, impulse: BallVec3) {
        if let Some(body) = self.rigid_body_set.get_mut(self.ball_handle) {
            if body.body_type() != RigidBodyType::Dynamic {
                return;
            }
            body.apply_impulse(impulse.to_rapier(), true);
        }
    }

    /// 공 동역학 이벤트를 큐에 넣는다 (`sim_time` 도달 시 실행).
    pub fn enqueue_ball_events(&mut self, script: BallScript) {
        for event in script.events() {
            self.pending_ball_events.push(event.clone());
        }
        self.pending_ball_events.sort_by(|a, b| {
            a.at_time
                .partial_cmp(&b.at_time)
                .unwrap_or(std::cmp::Ordering::Equal)
        });
    }

    /// 대기 중인 공 이벤트 수.
    pub fn pending_ball_event_count(&self) -> usize {
        return self.pending_ball_events.len();
    }

    fn tick_ball_events(&mut self) {
        while let Some(event) = self.pending_ball_events.first() {
            if event.at_time > self.sim_time {
                break;
            }
            let event = self.pending_ball_events.remove(0);
            self.apply_ball_action(event.action);
        }
    }

    fn apply_ball_action(&mut self, action: BallAction) {
        match action {
            BallAction::Launch {
                position,
                linear_velocity,
                angular_velocity,
            } => self.launch_ball_at(position, linear_velocity, angular_velocity),
            BallAction::Impulse { impulse } => self.apply_ball_impulse(impulse),
            BallAction::SetVelocity {
                linear_velocity,
                angular_velocity,
            } => {
                if let Some(body) = self.rigid_body_set.get_mut(self.ball_handle) {
                    body.set_body_type(RigidBodyType::Dynamic, true);
                    body.set_linvel(linear_velocity.to_rapier(), true);
                    body.set_angvel(angular_velocity.to_rapier(), true);
                    body.enable_ccd(true);
                }
                self.ball_state = BallState::InFlight;
            }
            BallAction::Teleport { position } => {
                if let Some(body) = self.rigid_body_set.get_mut(self.ball_handle) {
                    body.set_translation(position.to_rapier(), true);
                }
            }
            BallAction::Park { position } => {
                if let Some(body) = self.rigid_body_set.get_mut(self.ball_handle) {
                    if let Some(pos) = position {
                        body.set_translation(pos.to_rapier(), true);
                    }
                    body.set_body_type(RigidBodyType::Fixed, true);
                    body.set_linvel(Vector::ZERO, true);
                    body.set_angvel(Vector::ZERO, true);
                }
                self.ball_state = BallState::Parked;
            }
        }
    }

    /// 공을 슈터 발사구에 주차한다.
    pub fn park_ball(&mut self, settings: &BallShooterSettings) {
        self.debug_prediction = None;
        self.robot.cancel_swing();
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
    pub fn urdf(&self) -> Option<&crate::robot::urdf::UrdfRobot> {
        return self.urdf.as_deref();
    }

    /// 리니어 레일 x를 반영한 sim 마운트 (URDF FK·뷰어).
    pub fn effective_sim_mount(&self) -> crate::robot::urdf::SimRobotMount {
        if let Some(rail) = self.arm.rail.as_ref() {
            return crate::robot::urdf::SimRobotMount {
                position: [self.robot.rail_x(), rail.mount_y, rail.mount_z],
                rpy: [0.0, 0.0, 0.0],
            };
        }
        if let Some(urdf) = self.urdf.as_ref() {
            return urdf.mount;
        }
        return crate::robot::urdf::SimRobotMount {
            position: [self.arm.base.v.x, self.arm.base.v.y, self.arm.base.v.z],
            rpy: [0.0, 0.0, 0.0],
        };
    }

    /// FK 결과로 키네마틱 라켓 위치를 갱신한다.
    ///
    /// Rapier 충돌은 **제어 IK와 동일한 `Arm` FK**만 사용한다.
    /// URDF mesh FK는 링크 길이가 달라 공과 맞지 않는다 (mesh는 뷰어 전용).
    fn sync_racket_kinematic(&mut self) {
        let Some(pose) = self.robot.racket_pose(&self.arm) else {
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

    use crate::sim::BallShooterSettings;

    use pingpong_domain::{Arm, RobotPose, Target, constants::table};

    fn test_arm() -> Arc<Arm> {
        return Arc::new(Arm::competition().expect("테스트용 4DOF arm"));
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

    #[test]
    fn contact_swing_reaches_impact_fk_at_duration() {
        let arm = test_arm();
        let mut world = SimWorld::new(arm.clone(), None);
        world.set_oracle_auto_swing(true);
        world.shoot_ball(&BallShooterSettings::default());

        let mut min_dist = f64::MAX;
        for _ in 0..600 {
            world.step(1.0 / 1000.0, None);
            let ee = world.robot().racket_pose(&arm).expect("FK").position.v;
            let ball = world.ball_position();
            let dx = f64::from(ball.x) - ee.x;
            let dy = f64::from(ball.y) - ee.y;
            let dz = f64::from(ball.z) - ee.z;
            min_dist = min_dist.min((dx * dx + dy * dy + dz * dz).sqrt());
        }

        assert!(
            min_dist < 0.12,
            "비행 중 라켓·공 최소 거리 {min_dist:.3}m — 접촉 근처여야 함"
        );
    }

    #[test]
    fn auto_swing_on_shoot_moves_rail() {
        let arm = test_arm();
        assert!(arm.rail.is_some(), "테스트 arm은 리니어 포함");
        let mut world = SimWorld::new(arm, None);
        world.set_oracle_auto_swing(true);
        let settings = BallShooterSettings::default();
        assert_eq!(world.robot().rail_x(), 0.0, "대기 위치 x=0");
        world.shoot_ball(&settings);
        assert!(
            !world.robot().is_swinging(),
            "발사 직후는 commit 창 밖 — 스윙 대기"
        );
        let mut started = false;
        for _ in 0..800 {
            world.step(1.0 / 1000.0, None);
            if world.robot().is_swinging() || world.swing_committed() {
                started = true;
                break;
            }
        }
        assert!(started, "네트 통과 후 commit 창에서 스윙이 시작되어야 함");
        for _ in 0..500 {
            world.step(1.0 / 1000.0, None);
        }
        let rail_after = world.robot().rail_x();
        assert!(
            rail_after > 0.2,
            "레일이 impact x 방향으로 이동해야 함 (after={rail_after})"
        );
    }

    #[test]
    fn auto_swing_plans_with_strike_velocity() {
        use pingpong_domain::plan_swing;

        let arm = test_arm();
        let world = SimWorld::new(arm.clone(), None);
        let rail_x = world.robot().rail_x();
        let reachable = arm
            .forward_kinematics_with_rail(rail_x, world.robot().joints())
            .expect("FK")
            .position
            .v;
        // 도달 가능한 접수점 + 슈터 입사 속도
        let impact = pingpong_domain::Point3::new(
            reachable.x,
            table::DEFAULT_HIT_PLANE_Y.min(reachable.y.max(0.15)),
            reachable.z,
        );
        let start = RobotPose::new(rail_x, world.robot().joints().clone());
        let traj = plan_swing(
            &arm,
            Target {
                prediction: pingpong_domain::Prediction {
                    time_to_impact_secs: 0.25,
                    impact_position: impact,
                    incoming_velocity: nalgebra::Vector3::new(0.0, -7.5, -0.5),
                },
            },
            &start,
        )
        .expect("속도 포함 스윙");
        assert!(
            traj.end_velocity.iter().any(|v| v.abs() > 0.05),
            "로프트 타격 끝속도가 살아 있어야 함: {:?}",
            traj.end_velocity
        );
    }

    #[test]
    fn quintic_swing_moves_robot_joints() {
        use pingpong_domain::{HitPlane, Target, plan_swing};

        let arm = test_arm();
        let mut world = SimWorld::new(arm.clone(), None);
        let settings = BallShooterSettings::default();
        world.shoot_ball(&settings);

        let hit_plane = HitPlane {
            y: table::DEFAULT_HIT_PLANE_Y,
        };
        let pos = world.ball_position();
        let vel = world.ball_velocity();
        let vy = f64::from(vel.y);
        let t = ((hit_plane.y - f64::from(pos.y)) / vy).max(0.15);
        let impact_x = f64::from(pos.x) + f64::from(vel.x) * t;
        let reachable = arm
            .forward_kinematics_with_rail(world.robot().rail_x(), world.robot().joints())
            .expect("FK");
        let impact = pingpong_domain::Point3::<pingpong_domain::World>::new(
            impact_x,
            hit_plane.y,
            reachable.position.v.z,
        );
        let start = RobotPose::new(world.robot().rail_x(), world.robot().joints().clone());
        let trajectory = plan_swing(
            &arm,
            Target {
                prediction: pingpong_domain::Prediction {
                    time_to_impact_secs: t,
                    impact_position: impact,
                    incoming_velocity: nalgebra::Vector3::new(
                        f64::from(vel.x),
                        f64::from(vel.y),
                        f64::from(vel.z),
                    ),
                },
            },
            &start,
        )
        .expect("스윙 계획");
        let rail_end = trajectory.rail.end;
        let duration = trajectory.duration_secs;
        world.robot_mut().begin_swing(trajectory);

        let j0: Vec<f64> = world.robot().joints().values.clone();
        let dt = 1.0 / 1000.0;
        let steps = ((duration / dt).ceil() as usize).saturating_add(100);
        for _ in 0..steps {
            world.step(dt, None);
        }
        let j1: Vec<f64> = world.robot().joints().values.clone();
        let r1 = world.robot().rail_x();
        assert_ne!(j0, j1, "스윙 후 관절각이 변해야 함");
        assert!((r1 - rail_end).abs() < 0.05, "레일이 접수 x로 이동해야 함");
    }

    #[test]
    fn effective_sim_mount_follows_rail_x() {
        let arm = Arc::new(Arm::competition().expect("arm"));
        let mut world = SimWorld::new(arm, None);
        let x = 0.42;
        let joints = world.robot().joints().clone();
        *world.robot_mut() = RobotState::new(joints, x);
        let mount = world.effective_sim_mount();
        assert!((mount.position[0] - x).abs() < 1e-9);
    }

    #[test]
    fn urdf_joint_values_use_control_map() {
        use std::path::PathBuf;
        let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../assets/robots/urdf-test/urdf-test_description/urdf/urdf-test.urdf");
        if !path.exists() {
            return;
        }
        let urdf = Arc::new(
            crate::robot::UrdfRobot::from_file(&path, Some("pingpong_paddle_v5_1")).expect("urdf"),
        );
        let arm = Arc::new(Arm::competition().expect("arm"));
        let mut world = SimWorld::new(arm, Some(urdf));
        world.set_control_to_urdf(Some(vec![Some(0), Some(1), Some(2)]));
        let rail = world.robot().rail_x();
        *world.robot_mut() = RobotState::new(
            pingpong_domain::Joints {
                values: vec![0.11, 0.22, 0.33, 0.44],
            },
            rail,
        );
        let q = world.urdf_joint_values().expect("mapped");
        assert_eq!(q, vec![0.11, 0.22, 0.33]);
    }
}
