//! Rapier3d 시뮬레이션 월드.
//!
//! 탁구대·로봇(-x) · 슈터(+x) · 공. 공은 슈터에 주차되어 있다가
//! GUI 트리거로 발사되고, 로봇이 라켓으로 받는다.

use std::sync::Arc;

use crate::{
    Arm, DomainError, InterceptWindow, PhysicsParams, Prediction, RobotPose, RobotState,
    ball_past_midcourt_for_commit,
    constants::{ball, table},
    plan_best_swing,
};
use rapier3d::prelude::*;
use tracing::{debug, warn};

use super::ball_script::{BallAction, BallEvent, BallScript, BallVec3};
use super::rapier_convert::racket_pose_to_rapier;
use super::shooter::{BallShooterSettings, BallState, ShooterLayout};
use crate::sim::estimator::predict_impact;

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
    /// 테이블·공 반발 등 (config `[physics]`)
    pub physics: PhysicsParams,
    /// URDF 기반 FK·뷰어 (선택)
    pub urdf: Option<Arc<crate::robot::urdf::UrdfRobot>>,
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
    /// 동적으로 탐색할 접수 y 구간.
    intercept: InterceptWindow,
    /// true면 Rapier ground truth로 자동 스윙 (sim 기본).
    /// false면 카메라→DLT→EKF→control이 타격.
    use_ground_truth: bool,
    /// 이번 비행에서 스윙을 이미 commit했는지 (재계획·팔 떨림 방지)
    swing_committed: bool,
    /// 마지막으로 `plan_best_swing`을 실제로 시도한 `sim_time`.
    ///
    /// 실패해도 `swing_committed`는 세팅되지 않으므로, 시도 자체를 스로틀링
    /// 하지 않으면 공이 커밋 창 안에 있는 한 매 물리 스텝(1kHz)마다 IK·충돌
    /// 샘플링 전체를 다시 도는 성능 버그가 된다(`try_auto_swing` 참고). 반대로
    /// 한 번 실패했다고 완전히 포기해버리면(과거 구현) 첫 시도의 예측이
    /// 일시적으로 부정확했을 뿐인 경우(공이 아직 접근 중이라 목표가 곧
    /// 도달권 안으로 들어옴)를 놓친다 — 그래서 완전 포기가 아니라 재시도
    /// 빈도만 `SWING_RETRY_THROTTLE_SECS`로 제한한다.
    last_swing_attempt_at: f64,
    /// 이번 비행이 발사된 `sim_time` — `park_if_out_of_play`의 최대 비행
    /// 시간 안전장치(`MAX_BALL_FLIGHT_SECS`)가 기준으로 삼는다.
    flight_started_at: f64,
}

impl SimWorld {
    /// 탁구대·슈터·주차된 공·로봇 라켓을 배치한다.
    ///
    /// 제어·Rapier 라켓·URDF 뷰어는 같은 관절 순서와 기구학을 사용한다.
    pub fn new(arm: Arc<Arm>, urdf: Option<Arc<crate::robot::urdf::UrdfRobot>>) -> Self {
        return Self::with_physics(arm, urdf, PhysicsParams::default());
    }

    /// config `[physics]` 반발 등을 Rapier collider에 반영한다.
    pub fn with_physics(
        arm: Arc<Arm>,
        urdf: Option<Arc<crate::robot::urdf::UrdfRobot>>,
        physics: PhysicsParams,
    ) -> Self {
        let mut integration_parameters = IntegrationParameters::default();
        integration_parameters.dt = 1.0 / 1000.0;
        integration_parameters.num_solver_iterations = 12;

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
        .restitution(physics.restitution as f32)
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
        let racket_collider = ColliderBuilder::cuboid(
            crate::constants::geometry::RACKET_HALF_X as f32,
            crate::constants::geometry::RACKET_HALF_Y as f32,
            crate::constants::geometry::RACKET_HALF_Z as f32,
        )
        .restitution(physics.restitution as f32)
        .friction(0.5)
        .build();
        collider_set.insert_with_parent(racket_collider, racket_handle, &mut rigid_body_set);

        let muzzle = default_shooter.muzzle_position();
        let ball_body = RigidBodyBuilder::fixed().translation(muzzle).build();
        let ball_handle = rigid_body_set.insert(ball_body);
        let ball_collider = ColliderBuilder::ball(ball::RADIUS as f32)
            .restitution(physics.restitution as f32)
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
            physics,
            urdf,
            robot,
            sim_time: 0.0,
            ball_state: BallState::Parked,
            last_shooter_settings: default_shooter.clone(),
            pending_ball_events: Vec::new(),
            debug_prediction: None,
            intercept: InterceptWindow {
                y_min: 0.20,
                y_max: 0.55,
                sample_step: 0.05,
            },
            use_ground_truth: true,
            swing_committed: false,
            last_swing_attempt_at: f64::NEG_INFINITY,
            flight_started_at: 0.0,
        };
        world.sync_shooter_pose(&default_shooter);
        return world;
    }

    /// 뷰어용 URDF 관절각. 제어 모델과 축 순서가 정확히 같아야 한다.
    pub fn urdf_joint_values(&self) -> Option<Vec<f64>> {
        let urdf = self.urdf.as_ref()?;
        let values = &self.robot.joints().values;
        if values.len() != urdf.joint_count() {
            return None;
        }
        return Some(values.clone());
    }

    /// Rapier ground truth 자동 스윙 on/off.
    pub fn set_use_ground_truth(&mut self, enabled: bool) {
        self.use_ground_truth = enabled;
    }

    /// ground truth 기반 자동 스윙 여부.
    pub fn use_ground_truth(&self) -> bool {
        return self.use_ground_truth;
    }

    /// 이번 공에 스윙을 이미 commit했는지.
    pub fn swing_committed(&self) -> bool {
        return self.swing_committed;
    }

    /// control/ground truth 경로가 스윙을 commit했음을 표시한다.
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

    /// 동적 인터셉트 구간을 설정한다.
    pub fn set_intercept_window(&mut self, intercept: InterceptWindow) {
        self.intercept = intercept;
    }

    /// 공 비행 중 commit 창에 들어올 때 한 번만 `plan_swing`.
    ///
    /// 발사 직후(긴 lead)에는 대기하고, 네트 통과 후
    /// `time_to_impact ∈ [MIN_SWING, COMMIT_MAX]`일 때 시작한다.
    /// `use_ground_truth`가 true면 정확한 sim 상태, false면 control 경로가 타격.
    fn try_auto_swing(&mut self) {
        if !self.use_ground_truth {
            // 디버그 마커만 갱신
            if self.ball_state == BallState::InFlight {
                if let Some(prediction) = self
                    .intercept
                    .hit_planes()
                    .into_iter()
                    .find_map(|plane| predict_impact(self, plane))
                {
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
        let predictions: Vec<Prediction> = self
            .intercept
            .hit_planes()
            .into_iter()
            .filter_map(|plane| predict_impact(self, plane))
            .collect();
        if predictions.is_empty() {
            return;
        }

        // 상대 코트에 있으면 아직 이름 — 바운스·탄도 안정화 대기
        let ball_y = f64::from(self.ball_position().y);
        if !ball_past_midcourt_for_commit(ball_y) {
            return;
        }
        // 여기부터는 실제 `plan_best_swing`(IK·충돌 샘플링)을 도는 비싼
        // 구간이라 재시도 빈도를 제한한다. 완전히 포기하지는 않는다 — 첫
        // 시도의 예측이 일시적으로 부정확했을 뿐(공이 아직 접근 중이라
        // 목표가 곧 도달권 안으로 들어오는 경우)일 수 있어서, 한 번
        // 실패했다고 이번 비행 내내 포기하면 실제로 접수 가능한 공을
        // 놓친다(4-dof 카탈로그 로봇으로 실측 확인됨).
        const SWING_RETRY_THROTTLE_SECS: f64 = 0.02;
        if self.sim_time - self.last_swing_attempt_at < SWING_RETRY_THROTTLE_SECS {
            return;
        }
        self.last_swing_attempt_at = self.sim_time;
        let start = RobotPose::new(self.robot.rail_x(), self.robot.joints().clone());
        let planned = match plan_best_swing(&self.arm, &predictions, &start) {
            Ok(planned) => planned,
            Err(DomainError::InfeasibleSwing(ref err)) => {
                debug!(%err, "plan_swing 불가 — 이번 시도 스킵, 다음 재시도 대기");
                return;
            }
            Err(other) => {
                warn!(%other, "sim 자동 스윙 계획 실패 — 다음 재시도 대기");
                return;
            }
        };
        self.set_debug_prediction(Some(planned.prediction));
        let trajectory = planned.trajectory;
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
        self.last_swing_attempt_at = f64::NEG_INFINITY;
        self.flight_started_at = self.sim_time;
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

    /// 테이블 밖·바닥으로 떨어졌거나, 테이블 위에서 멈춰버린 공을 슈터로 회수한다.
    fn park_if_out_of_play(&mut self) {
        let body = &self.rigid_body_set[self.ball_handle];
        let pos = body.translation();
        let out = pos.x < -0.15
            || pos.x > (table::WIDTH_X + 0.15) as f32
            || pos.y < -0.15
            || pos.y > (table::LENGTH_Y + 0.15) as f32
            || pos.z < 0.35;
        // 라켓에 맞고 되돌아온 공이 테이블 위에 그대로 안착하는 경우 위 `out`
        // 조건에 걸리지 않아 `ball_state`가 영원히 InFlight로 남는다 — 그러면
        // `try_auto_swing`이 (실패한 뒤에도) 매 물리 스텝 재시도하는 성능
        // 문제로 이어진다(Random Shoot를 반복하면 멈추는 현상의 원인).
        let resting = body.linvel().length_squared() < (0.01 * 0.01)
            && pos.z < (table::SURFACE_Z + 0.05) as f32;
        // 위 두 조건이 못 잡는 경우(예: 반발이 커서 오래 통통 튀며 안 멈추는
        // 경우)에 대비한 최종 안전장치 — 비행이 이만큼 길어지면 속도·위치와
        // 무관하게 무조건 회수한다.
        const MAX_BALL_FLIGHT_SECS: f64 = 4.0;
        let timed_out = self.sim_time - self.flight_started_at > MAX_BALL_FLIGHT_SECS;

        if out || resting || timed_out {
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
                rpy: self
                    .urdf
                    .as_ref()
                    .map_or([0.0, 0.0, 0.0], |urdf| urdf.mount.rpy),
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
    /// URDF 로봇도 부팅 시 같은 `Arm` 직렬 체인으로 변환된다.
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

    use crate::{Arm, RobotPose, constants::table};

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
        world.set_use_ground_truth(true);
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
    fn ground_truth_rally_contacts_racket_clears_net_and_bounces_near_center() {
        let arm = test_arm();
        let mut world = SimWorld::new(arm, None);
        world.set_use_ground_truth(true);

        let collider_for_body = |body_handle| {
            world
                .collider_set
                .iter()
                .find_map(|(handle, collider)| {
                    (collider.parent() == Some(body_handle)).then_some(handle)
                })
                .expect("body collider")
        };
        let ball_collider = collider_for_body(world.ball_handle);
        let racket_collider = collider_for_body(world.racket_handle);
        let table_collider = world
            .collider_set
            .iter()
            .find_map(|(handle, collider)| {
                let cuboid = collider.shape().as_cuboid()?;
                ((f64::from(cuboid.half_extents.x) - table::WIDTH_X * 0.5).abs() < 1e-5
                    && (f64::from(cuboid.half_extents.y) - table::LENGTH_Y * 0.5).abs() < 1e-5)
                    .then_some(handle)
            })
            .expect("table collider");

        world.shoot_ball(&BallShooterSettings::default());
        let mut racket_contact = false;
        let mut returned = false;
        let mut net_clearance = None;
        let mut bounce = None;
        let mut contact_state = None;
        let mut max_return_y = f32::NEG_INFINITY;
        let net_y = (table::LENGTH_Y * 0.5) as f32;
        let mut previous_y = world.ball_position().y;

        for _ in 0..4_000 {
            world.step(1.0 / 1000.0, None);
            let position = world.ball_position();
            let velocity = world.ball_velocity();

            let racket_pair = world
                .narrow_phase
                .contact_pair(ball_collider, racket_collider);
            if racket_pair.is_some_and(ContactPair::has_any_active_contact) {
                racket_contact = true;
                if contact_state.is_none() {
                    contact_state = Some((position, velocity));
                }
            }
            if racket_contact && velocity.y > 0.0 {
                returned = true;
                max_return_y = max_return_y.max(position.y);
            }
            if returned && previous_y < net_y && position.y >= net_y {
                net_clearance = Some(position.z);
            }
            if net_clearance.is_some()
                && world
                    .narrow_phase
                    .contact_pair(ball_collider, table_collider)
                    .is_some_and(ContactPair::has_any_active_contact)
            {
                bounce = Some(position);
                break;
            }
            previous_y = position.y;
        }

        assert!(racket_contact, "공–라켓 활성 접촉이 있어야 함");
        assert!(returned, "라켓 접촉 뒤 공의 vy가 +여야 함");
        let net_z = net_clearance.unwrap_or_else(|| {
            panic!("리턴 공이 네트를 통과해야 함: contact={contact_state:?}, max_y={max_return_y}")
        });
        assert!(
            f64::from(net_z)
                >= table::SURFACE_Z + table::NET_HEIGHT + crate::constants::BALL_RADIUS,
            "네트 통과 높이 부족: {net_z}"
        );
        let bounce = bounce.expect("네트 통과 뒤 상대 코트 첫 테이블 접촉");
        let target_x = (table::WIDTH_X * 0.5) as f32;
        let target_y = (table::LENGTH_Y * 0.75) as f32;
        assert!(
            (bounce.x - target_x).abs() <= 0.20 && (bounce.y - target_y).abs() <= 0.20,
            "bounce={bounce:?}, target=({target_x}, {target_y}), contact={contact_state:?}"
        );
    }

    /// 진단용 — `catalog::find_robot("4-dof")`가 실제 카탈로그 경로(URDF +
    /// `RobotBuilder`)로 만드는 팔·마운트 그대로 로드한다. `competition()`
    /// 처럼 손으로 만든 것이 아니라 `main.rs::load_robot`과 동일 경로.
    fn fourdof_robot() -> (Arc<Arm>, Option<Arc<crate::robot::urdf::UrdfRobot>>) {
        let entry = crate::robot::catalog::find_robot("4-dof").expect("4-dof 카탈로그 항목");
        let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join(entry.urdf_rel.expect("4-dof는 URDF 필수"));
        let built = crate::robot::RobotBuilder::new()
            .urdf(&path)
            .ee_link_opt(entry.ee_link)
            .mount_preset(crate::robot::MountPreset::Rep103AtTableEnd)
            .max_joint_speed(entry.max_joint_speed)
            .build()
            .expect("4-dof RobotBuilder 빌드");
        return (built.arm, built.urdf);
    }

    /// 기본 슈터 샷이 네트 위를 여유 있게 지나가는지 회귀 검증한다.
    ///
    /// `pitch_deg=-4.0`이던 예전 기본값은 첫 바운스 뒤 네트를 -0.7cm 차로
    /// 스쳤다(`predict_hit_plane`은 네트를 모델링하지 않아 이 경계 샷에서
    /// 예측이 커밋 직후 곧바로 틀어짐 — 팔 기하학과 무관하게 발생하는 결함).
    /// `pitch_deg=-2.0`으로 옮긴 지금은 여유 있게 넘어간다.
    #[test]
    fn default_shot_clears_net_with_margin() {
        let arm = test_arm();
        let mut world = SimWorld::new(arm, None);
        world.set_use_ground_truth(false); // 스윙 없이 순수 탄도만 본다

        let net_collider = world
            .collider_set
            .iter()
            .find_map(|(handle, collider)| {
                let cuboid = collider.shape().as_cuboid()?;
                ((f64::from(cuboid.half_extents.y) - 0.005).abs() < 1e-6).then_some(handle)
            })
            .expect("net collider");
        let ball_collider = world
            .collider_set
            .iter()
            .find_map(|(handle, collider)| {
                (collider.parent() == Some(world.ball_handle)).then_some(handle)
            })
            .expect("ball collider");

        let net_top_z = table::SURFACE_Z + crate::constants::table::NET_HEIGHT;
        world.shoot_ball(&BallShooterSettings::default());

        let net_y = (table::LENGTH_Y * 0.5) as f32;
        let mut previous_y = world.ball_position().y;
        for _ in 0..3_000 {
            world.step(1.0 / 1000.0, None);
            let pos = world.ball_position();
            assert!(
                !world
                    .narrow_phase
                    .contact_pair(ball_collider, net_collider)
                    .is_some_and(ContactPair::has_any_active_contact),
                "기본 샷이 네트에 맞음: y={:.4} z={:.4} (net_top={:.4})",
                pos.y,
                pos.z,
                net_top_z
            );
            if previous_y > net_y && pos.y <= net_y {
                assert!(
                    f64::from(pos.z) > net_top_z,
                    "네트 통과 높이 여유 없음: z={:.4} net_top={:.4}",
                    pos.z,
                    net_top_z
                );
                return;
            }
            previous_y = pos.y;
        }
        panic!("공이 네트 y를 지나가지 않음 — 샷이 테이블 위에서 멈췄거나 이탈함");
    }

    /// `competition()` primitive는 이미 랠리 통합 테스트가 있지만
    /// (`ground_truth_rally_contacts_racket_clears_net_and_bounces_near_center`),
    /// 카탈로그 `"4-dof"` URDF 로봇(`main.rs::load_robot`과 동일 경로로 조립)은
    /// 한 번도 같은 방식으로 검증된 적이 없었다.
    #[test]
    fn fourdof_ground_truth_rally_contacts_racket_and_returns() {
        let (arm, urdf) = fourdof_robot();
        let mut world = SimWorld::new(arm.clone(), urdf);
        world.set_use_ground_truth(true);

        let collider_for_body = |body_handle| {
            world
                .collider_set
                .iter()
                .find_map(|(handle, collider)| {
                    (collider.parent() == Some(body_handle)).then_some(handle)
                })
                .expect("body collider")
        };
        let ball_collider = collider_for_body(world.ball_handle);
        let racket_collider = collider_for_body(world.racket_handle);

        world.shoot_ball(&BallShooterSettings::default());

        let mut racket_contact = false;
        let mut returned = false;
        let mut min_dist = f64::MAX;

        for _ in 0..4_000 {
            world.step(1.0 / 1000.0, None);

            let ee = world.robot().racket_pose(&arm).expect("FK").position.v;
            let ball = world.ball_position();
            let dx = f64::from(ball.x) - ee.x;
            let dy = f64::from(ball.y) - ee.y;
            let dz = f64::from(ball.z) - ee.z;
            min_dist = min_dist.min((dx * dx + dy * dy + dz * dz).sqrt());

            if world
                .narrow_phase
                .contact_pair(ball_collider, racket_collider)
                .is_some_and(ContactPair::has_any_active_contact)
            {
                racket_contact = true;
            }
            if racket_contact && world.ball_velocity().y > 0.0 {
                returned = true;
                break;
            }
        }

        assert!(
            racket_contact,
            "4-dof 라켓·공 접촉 없음 — min_dist={min_dist:.4}"
        );
        assert!(returned, "라켓 접촉 뒤 공의 vy가 +여야 함");
    }

    #[test]
    fn auto_swing_on_shoot_moves_rail() {
        let arm = test_arm();
        assert!(arm.rail.is_some(), "테스트 arm은 리니어 포함");
        let mut world = SimWorld::new(arm, None);
        world.set_use_ground_truth(true);
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

    /// 실물 로봇은 모터 토크 한계 때문에 레일 한쪽 끝→반대쪽 끝처럼 급한
    /// 이동을 못 만든다 — 매 스윙 뒤 항상 테이블 폭 중앙(레일 `default_x`,
    /// 관절 `default_joints`)으로 복귀시켜 다음 스윙의 시작 조건을 일정하게
    /// 유지해야 한다. `home_x`(레일 원점, x=0)는 부팅 시 대기 위치일 뿐 여기서
    /// 말하는 중앙이 아니다. 스윙이 끝난 뒤 다음 공을 쏘지 않아도 로봇이
    /// 저절로 복귀하는지 검증한다.
    #[test]
    fn robot_returns_to_center_after_swing_without_next_shot() {
        let arm = test_arm();
        let center_rail_x = arm.rail.as_ref().expect("테스트 arm은 리니어 포함").default_x();
        let center_joints = arm.default_joints.clone();

        let mut world = SimWorld::new(arm, None);
        world.set_use_ground_truth(true);
        world.shoot_ball(&BallShooterSettings::default());

        let mut swing_started = false;
        for _ in 0..800 {
            world.step(1.0 / 1000.0, None);
            if world.robot().is_swinging() {
                swing_started = true;
                break;
            }
        }
        assert!(swing_started, "스윙이 시작되어야 함");

        // 타격 스윙이 끝나면 로봇이 곧바로 복귀 궤적을 이어서 시작하므로
        // (`RobotState::step_toward_targets`), `is_swinging()`은 타격+팔로스루
        // +복귀 전체를 하나의 연속 동작으로 본다 — "다 끝났다"는 신호는
        // `is_swinging()`이 다시 false가 되는 순간 하나뿐이고, 그 시점에는
        // 이미 중앙 복귀까지 끝나 있어야 한다.
        let mut swing_ended = false;
        for _ in 0..6_000 {
            world.step(1.0 / 1000.0, None);
            if !world.robot().is_swinging() {
                swing_ended = true;
                break;
            }
        }
        assert!(swing_ended, "타격+복귀가 끝나야 함");

        let rail_x = world.robot().rail_x();
        let joints_close = world
            .robot()
            .joints()
            .values
            .iter()
            .zip(center_joints.values.iter())
            .all(|(actual, center)| (actual - center).abs() < 1e-2);
        assert!(
            (rail_x - center_rail_x).abs() < 1e-2 && joints_close,
            "스윙 뒤 다음 발사 없이도 로봇이 저절로 중앙(rail={center_rail_x})으로 복귀해야 함 \
             (실제 rail={rail_x}, joints={:?}, center={:?})",
            world.robot().joints().values,
            center_joints.values,
        );
    }

    #[test]
    fn auto_swing_plans_with_strike_velocity() {
        use crate::plan_swing;

        let arm = test_arm();
        let world = SimWorld::new(arm.clone(), None);
        let rail_x = world.robot().rail_x();
        // 기본 슈터가 첫 바운스 뒤 만드는 동적 y=0.30 후보.
        let impact = crate::Point3::new(table::WIDTH_X * 0.5, 0.30, 1.05);
        let start = RobotPose::new(rail_x, world.robot().joints().clone());
        let traj = plan_swing(
            &arm,
            crate::Prediction {
                time_to_impact_secs: 0.28,
                impact_position: impact,
                incoming_velocity: nalgebra::Vector3::new(0.0, -4.22, 0.37),
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
        use crate::{HitPlane, plan_swing};

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
        let impact = crate::Point3::new(impact_x, hit_plane.y, reachable.position.v.z);
        let start = RobotPose::new(world.robot().rail_x(), world.robot().joints().clone());
        let trajectory = plan_swing(
            &arm,
            crate::Prediction {
                time_to_impact_secs: t,
                impact_position: impact,
                incoming_velocity: nalgebra::Vector3::new(
                    f64::from(vel.x),
                    f64::from(vel.y),
                    f64::from(vel.z),
                ),
            },
            &start,
        )
        .expect("스윙 계획");
        let rail_end = trajectory.rail.end;
        let duration = trajectory.duration_secs;
        world.robot_mut().begin_swing(trajectory);

        let j0: Vec<f64> = world.robot().joints().values.clone();
        let dt = 1.0 / 1000.0;
        // 스윙이 끝나자마자 로봇이 자동으로 홈 복귀 궤적을 이어서 시작하므로
        // (실물 로봇처럼 항상 중앙 정렬), 여유 버퍼를 크게 두면 레일이 이미
        // 복귀 방향으로 움직이기 시작한 뒤 값을 재게 된다 — 스윙 완료 직후만
        // 확인하도록 버퍼를 작게 둔다.
        let steps = ((duration / dt).ceil() as usize).saturating_add(5);
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
    fn urdf_joint_values_are_the_control_joint_values() {
        use std::path::PathBuf;
        let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("assets/robots/urdf-test/urdf-test_description/urdf/urdf-test.urdf");
        assert!(
            path.exists(),
            "URDF 테스트 자산이 없습니다: {}",
            path.display()
        );
        let built = crate::robot::RobotBuilder::new()
            .urdf(path)
            .ee_link("pingpong_paddle_v5_1")
            .mount_xyz_rpy(
                [0.0, 0.02, crate::constants::table::SURFACE_Z],
                [0.1, -0.2, 0.3],
            )
            .build()
            .expect("robot");
        let mut world = SimWorld::new(built.arm, built.urdf);
        let rail = world.robot().rail_x();
        *world.robot_mut() = RobotState::new(
            crate::Joints {
                values: vec![0.11, 0.22, 0.33],
            },
            rail,
        );
        let q = world.urdf_joint_values().expect("same joints");
        assert_eq!(q, vec![0.11, 0.22, 0.33]);
        assert_eq!(world.effective_sim_mount().rpy, [0.1, -0.2, 0.3]);
    }

    /// GUI "Random Shoot"가 쓰는 `lateral_offset_m ∈ [-0.5, 0.5]` 전체 범위에서
    /// 첫 바운스가 항상 테이블 폭 안(여유 있게)에 떨어지는지 검증한다.
    ///
    /// `yaw_deg`로 좌우를 바꾸는 방법도 시도했지만, 경험적 스윕에서 일부 각도
    /// (±10~15°)가 네트를 비스듬히 맞고 튕겨 테이블 밖으로 나가는 걸 확인했다
    /// (공 자유비행 자체가 각도에 비선형적으로 반응). `lateral_offset_m`은
    /// 궤적 모양은 그대로 두고 시작 x만 평행이동하므로 이 문제가 없다.
    #[test]
    fn random_shot_lateral_range_stays_within_table() {
        const LATERAL_RANGE_M: f64 = 0.5;
        const EDGE_MARGIN_M: f64 = 0.1;

        for lateral in [-0.5_f64, -0.25, 0.0, 0.25, 0.5] {
            assert!(lateral.abs() <= LATERAL_RANGE_M);
            let arm = test_arm();
            let mut world = SimWorld::new(arm, None);
            world.set_use_ground_truth(false);
            let table_collider = world
                .collider_set
                .iter()
                .find_map(|(handle, collider)| {
                    let cuboid = collider.shape().as_cuboid()?;
                    ((f64::from(cuboid.half_extents.x) - table::WIDTH_X * 0.5).abs() < 1e-5
                        && (f64::from(cuboid.half_extents.y) - table::LENGTH_Y * 0.5).abs()
                            < 1e-5)
                        .then_some(handle)
                })
                .expect("table collider");
            let ball_collider = world
                .collider_set
                .iter()
                .find_map(|(handle, collider)| {
                    (collider.parent() == Some(world.ball_handle)).then_some(handle)
                })
                .expect("ball collider");

            let settings = BallShooterSettings {
                lateral_offset_m: lateral,
                ..BallShooterSettings::default()
            };
            world.shoot_ball(&settings);
            let mut bounce_x = None;
            for _ in 0..5_000 {
                world.step(1.0 / 1000.0, None);
                if world
                    .narrow_phase
                    .contact_pair(ball_collider, table_collider)
                    .is_some_and(ContactPair::has_any_active_contact)
                {
                    bounce_x = Some(f64::from(world.ball_position().x));
                    break;
                }
            }
            let bounce_x = bounce_x
                .unwrap_or_else(|| panic!("lateral={lateral:+.2} — 공이 테이블에 안 떨어짐"));
            assert!(
                bounce_x > EDGE_MARGIN_M && bounce_x < table::WIDTH_X - EDGE_MARGIN_M,
                "lateral={lateral:+.2} — 첫 바운스 x={bounce_x:.3}가 테이블 폭 여유 범위 밖 \
                 (x∈[{EDGE_MARGIN_M:.2},{:.2}] 기대)",
                table::WIDTH_X - EDGE_MARGIN_M
            );
        }
    }

    /// `BallShooterSettings::randomized`가 뽑을 수 있는 (lateral, yaw, speed) 공간의
    /// 코너(각 lateral의 yaw_min/yaw_max × speed_min/speed_max)를 모두 스윕해서,
    /// 어떤 랜덤 샷도 네트를 맞지 않고 라켓 접수·리턴까지 이어짐을 검증한다.
    ///
    /// `randomized`는 발사 위치(`lateral_offset_m`)에 따라 기하학적으로 유효한
    /// yaw 범위를 계산해 그 안에서 뽑는다(`yaw_range_for_lateral_deg`) — 이 범위의
    /// 양 끝이 이 테스트가 실제로 검증하는 "가장 비스듬한" 샷이다.
    #[test]
    fn repeated_random_shoot_never_stalls_and_always_reparks() {
        use rand::SeedableRng;
        let mut rng = rand::rngs::StdRng::seed_from_u64(42);

        let arm = test_arm();
        let mut world = SimWorld::new(arm, None);
        world.set_use_ground_truth(true);

        // 재시도 폭주 버그(수정 전)는 실패한 스윙 계획을 매 틱마다 다시 돌려서
        // "느린 스텝"이 한 비행 내내 수백~수천 번 반복됐다. 수정 후에는 스윙이
        // 끝나는 순간 `plan_return_to_center`가 딱 한 번(그 자체는 몇 ms 걸릴
        // 수 있음) 도는 것만 허용한다 — 라운드당 느린 스텝 "개수"가 적어야
        // 폭주가 아니라 한 번짜리 계획 비용임을 보증한다.
        const SLOW_STEP_THRESHOLD: std::time::Duration = std::time::Duration::from_millis(2);
        const MAX_SLOW_STEPS_PER_ROUND: usize = 3;

        let mut worst_step = std::time::Duration::ZERO;
        for round in 0..30 {
            let settings = BallShooterSettings::default().randomized(&mut rng);
            world.shoot_ball(&settings);

            let mut reparked = false;
            let mut slow_steps = 0;
            for _ in 0..6_000 {
                let t0 = std::time::Instant::now();
                world.step(1.0 / 1000.0, None);
                let dt = t0.elapsed();
                if dt > worst_step {
                    worst_step = dt;
                }
                if dt > SLOW_STEP_THRESHOLD {
                    slow_steps += 1;
                }
                if world.ball_state == BallState::Parked {
                    reparked = true;
                    break;
                }
            }
            assert!(
                reparked,
                "round {round} — 6000 스텝 안에 공이 다시 Parked로 안 돌아옴 (settings={settings:?})"
            );
            assert!(
                slow_steps <= MAX_SLOW_STEPS_PER_ROUND,
                "round {round} — {SLOW_STEP_THRESHOLD:?} 넘는 스텝이 {slow_steps}개 — 재시도 폭주 의심"
            );
        }

        assert!(
            worst_step < std::time::Duration::from_millis(15),
            "반복 Random Shoot 중 스텝 하나가 너무 오래 걸림: {worst_step:?}"
        );
    }

    #[test]
    fn random_shot_grid_still_swings_when_robot_starts_from_center() {
        // 실제 GUI 재현: 첫 샷이 끝나면 로봇이 (레일 0이 아니라) 테이블
        // 중앙(`default_x()`)으로 복귀해 있다. 이후 Random Shoot이 쏘는
        // 격자 코너들이, 로봇이 그 중앙 위치에서 시작해도 여전히 스윙·
        // 접수되는지 확인한다 (사용자 재현: 첫 샷 이후 Random Shoot을
        // 누르면 공만 날아가고 로봇팔이 안 움직이는 문제).
        for lateral in [-0.5_f64, -0.25, 0.0, 0.25, 0.5] {
            let (yaw_min, yaw_max) = BallShooterSettings::yaw_range_for_lateral_deg(lateral);
            for yaw in [yaw_min, yaw_max] {
                for speed in [
                    crate::sim::shooter::RANDOM_SHOT_SPEED_MIN_MPS,
                    crate::sim::shooter::RANDOM_SHOT_SPEED_MAX_MPS,
                ] {
                    let settings = BallShooterSettings {
                        lateral_offset_m: lateral,
                        yaw_deg: yaw,
                        speed_mps: speed,
                        ..BallShooterSettings::default()
                    };

                    let arm = test_arm();
                    let center_rail_x = arm.rail.as_ref().expect("리니어").default_x();
                    let center_joints = arm.default_joints.clone();
                    let mut world = SimWorld::new(arm.clone(), None);
                    world.set_use_ground_truth(true);
                    *world.robot_mut() = RobotState::new(center_joints, center_rail_x);

                    let collider_for_body = |world: &SimWorld, body_handle| {
                        world
                            .collider_set
                            .iter()
                            .find_map(|(handle, collider)| {
                                (collider.parent() == Some(body_handle)).then_some(handle)
                            })
                            .expect("body collider")
                    };
                    let ball_collider = collider_for_body(&world, world.ball_handle);
                    let racket_collider = collider_for_body(&world, world.racket_handle);

                    world.shoot_ball(&settings);

                    let mut racket_contact = false;
                    let mut returned = false;
                    for _ in 0..5_000 {
                        world.step(1.0 / 1000.0, None);
                        if world
                            .narrow_phase
                            .contact_pair(ball_collider, racket_collider)
                            .is_some_and(ContactPair::has_any_active_contact)
                        {
                            racket_contact = true;
                        }
                        if racket_contact && world.ball_velocity().y > 0.0 {
                            returned = true;
                            break;
                        }
                    }

                    assert!(
                        racket_contact,
                        "lateral={lateral:+.2} yaw={yaw:+.2} speed={speed:.2} — \
                         로봇이 중앙에서 시작할 때 라켓 접수 없음"
                    );
                    assert!(
                        returned,
                        "lateral={lateral:+.2} yaw={yaw:+.2} speed={speed:.2} — \
                         로봇이 중앙에서 시작할 때 라켓 접수 뒤 리턴 안 됨"
                    );
                }
            }
        }
    }

    #[test]
    fn repeated_full_random_shots_each_get_racket_contact() {
        // 이전 스트레스 테스트(`repeated_random_shoot_never_stalls_and_always_reparks`)는
        // 공이 결국 회수(re-park)되는지만 확인해서, "로봇이 아예 안 치고
        // 공만 지나가도" 통과해버린다 — 정확히 사용자가 재현한 증상(공은
        // 날아가는데 로봇팔이 안 움직임)을 못 잡는다. 매 라운드 실제로
        // 라켓 접수가 일어나는지까지 확인한다. 같은 `SimWorld` 인스턴스를
        // 계속 재사용해서(GUI에서 Shoot을 반복 누르는 것과 동일), 각 샷이
        // "이전 샷이 완전히 끝난(로봇이 중앙 복귀 완료) 뒤" 시작되게 한다.
        use rand::SeedableRng;
        let mut rng = rand::rngs::StdRng::seed_from_u64(123);

        let arm = test_arm();
        let mut world = SimWorld::new(arm, None);
        world.set_use_ground_truth(true);

        for round in 0..15 {
            let settings = BallShooterSettings::default().randomized(&mut rng);
            world.shoot_ball(&settings);

            let ball_collider = world
                .collider_set
                .iter()
                .find_map(|(handle, collider)| {
                    (collider.parent() == Some(world.ball_handle)).then_some(handle)
                })
                .expect("ball collider");
            let racket_collider = world
                .collider_set
                .iter()
                .find_map(|(handle, collider)| {
                    (collider.parent() == Some(world.racket_handle)).then_some(handle)
                })
                .expect("racket collider");

            let mut racket_contact = false;
            let mut fully_settled = false;
            for _ in 0..8_000 {
                world.step(1.0 / 1000.0, None);
                if world
                    .narrow_phase
                    .contact_pair(ball_collider, racket_collider)
                    .is_some_and(ContactPair::has_any_active_contact)
                {
                    racket_contact = true;
                }
                // "이전 샷이 완전히 끝난 뒤"까지 기다린다 — 공이 회수되고
                // 로봇도 스윙 중이 아님(중앙 복귀까지 끝).
                if world.ball_state == BallState::Parked && !world.robot().is_swinging() {
                    fully_settled = true;
                    break;
                }
            }
            assert!(
                fully_settled,
                "round {round} — 다음 라운드 전에 공 회수·로봇 복귀가 끝나지 않음                  (settings={settings:?})"
            );
            assert!(
                racket_contact,
                "round {round} — 공은 날아갔는데 라켓 접수가 없었음 (로봇팔이 안 움직임)                  (settings={settings:?})"
            );
        }
    }


    /// `random_shot_grid_clears_net_and_returns`는 `Arm::competition()`(손으로
    /// 만든 테스트용 팔)만 검증한다 — 실제 GUI가 쓰는 카탈로그 "4-dof" 로봇
    /// (`fourdof_robot`, URDF + `Rep103AtTableEnd`)은 리치가 달라 같은 범위가
    /// 안전하지 않을 수 있다(실측으로 확인됨 — 아래 좌우 위치·yaw 촘촘한
    /// 격자에서 5.6 m/s부터 임팩트 지점이 도달 범위 밖으로 나가는 조합이
    /// 나왔고, 그래서 `RANDOM_SHOT_SPEED_MAX_MPS`를 5.5로 낮췄다). 코너만이
    /// 아니라 각 좌우 위치에서 유효한 yaw 범위를 0/25/50/75/100%로 나눠
    /// 스윕해야 한다 — 코너만 봐서는 못 잡는 실패(중간값에서만 실패)가
    /// 실제로 있었다.
    #[test]
    fn random_shot_fine_grid_clears_net_and_returns_for_fourdof_robot() {
        for lateral in [-0.5_f64, -0.25, 0.0, 0.25, 0.5] {
            let (yaw_min, yaw_max) = BallShooterSettings::yaw_range_for_lateral_deg(lateral);
            for frac in [0.0_f64, 0.25, 0.5, 0.75, 1.0] {
                let yaw = yaw_min + (yaw_max - yaw_min) * frac;
                for speed in [
                    crate::sim::shooter::RANDOM_SHOT_SPEED_MIN_MPS,
                    crate::sim::shooter::RANDOM_SHOT_SPEED_MAX_MPS,
                ] {
                    let settings = BallShooterSettings {
                        lateral_offset_m: lateral,
                        yaw_deg: yaw,
                        speed_mps: speed,
                        ..BallShooterSettings::default()
                    };
                    let (arm, urdf) = fourdof_robot();
                    let mut world = SimWorld::new(arm, urdf);
                    world.set_use_ground_truth(true);

                    let ball_collider = world
                        .collider_set
                        .iter()
                        .find_map(|(handle, collider)| {
                            (collider.parent() == Some(world.ball_handle)).then_some(handle)
                        })
                        .expect("ball collider");
                    let racket_collider = world
                        .collider_set
                        .iter()
                        .find_map(|(handle, collider)| {
                            (collider.parent() == Some(world.racket_handle)).then_some(handle)
                        })
                        .expect("racket collider");

                    world.shoot_ball(&settings);

                    let mut racket_contact = false;
                    let mut returned = false;
                    for _ in 0..5_000 {
                        world.step(1.0 / 1000.0, None);
                        if world
                            .narrow_phase
                            .contact_pair(ball_collider, racket_collider)
                            .is_some_and(ContactPair::has_any_active_contact)
                        {
                            racket_contact = true;
                        }
                        if racket_contact && world.ball_velocity().y > 0.0 {
                            returned = true;
                            break;
                        }
                    }

                    assert!(
                        racket_contact,
                        "lateral={lateral:+.2} yaw={yaw:+.2} speed={speed:.2} — \
                         4-dof 로봇이 라켓 접수 못 함"
                    );
                    assert!(
                        returned,
                        "lateral={lateral:+.2} yaw={yaw:+.2} speed={speed:.2} — \
                         4-dof 로봇이 접수 뒤 리턴 못 함"
                    );
                }
            }
        }
    }

    #[test]
    fn plain_shoot_then_random_shoot_gets_racket_contact_broad_sweep() {
        // 사용자가 정확히 재현한 순서: 평범한 Shoot(중앙→중앙, 기본 조준)을
        // 먼저 완전히 끝낸 뒤, Random Shoot을 누른다. 여러 랜덤 시드로
        // 넓게 스윕해서 실패하는 조합이 있는지 찾는다. 사용자가 실제로
        // 돌리는 건 `Arm::competition()`이 아니라 카탈로그 "4-dof" 로봇
        // (`main.rs::load_robot`과 동일 경로)이므로 그걸로 재현한다.
        use rand::SeedableRng;

        for seed in 0..200_u64 {
            let (arm, urdf) = fourdof_robot();
            let mut world = SimWorld::new(arm, urdf);
            world.set_use_ground_truth(true);

            // 1구: 평범한 Shoot.
            world.shoot_ball(&BallShooterSettings::default());
            let mut settled = false;
            for _ in 0..8_000 {
                world.step(1.0 / 1000.0, None);
                if world.ball_state == BallState::Parked && !world.robot().is_swinging() {
                    settled = true;
                    break;
                }
            }
            assert!(settled, "seed={seed} — 1구(평범한 Shoot) 후 정착 안 됨");

            // 2구: Random Shoot.
            let mut rng = rand::rngs::StdRng::seed_from_u64(seed);
            let settings = BallShooterSettings::default().randomized(&mut rng);
            world.shoot_ball(&settings);

            let ball_collider = world
                .collider_set
                .iter()
                .find_map(|(handle, collider)| {
                    (collider.parent() == Some(world.ball_handle)).then_some(handle)
                })
                .expect("ball collider");
            let racket_collider = world
                .collider_set
                .iter()
                .find_map(|(handle, collider)| {
                    (collider.parent() == Some(world.racket_handle)).then_some(handle)
                })
                .expect("racket collider");

            let mut racket_contact = false;
            for _ in 0..8_000 {
                world.step(1.0 / 1000.0, None);
                if world
                    .narrow_phase
                    .contact_pair(ball_collider, racket_collider)
                    .is_some_and(ContactPair::has_any_active_contact)
                {
                    racket_contact = true;
                    break;
                }
                if world.ball_state == BallState::Parked {
                    break;
                }
            }
            assert!(
                racket_contact,
                "seed={seed} — 평범한 Shoot 뒤 Random Shoot(settings={settings:?})에서 \
                 라켓 접수 없음 (로봇팔이 안 움직인 것으로 보임)"
            );
        }
    }

    #[test]
    fn interrupting_swing_with_new_shot_does_not_permanently_break_robot() {
        use rand::SeedableRng;
        let mut rng = rand::rngs::StdRng::seed_from_u64(7);

        // 스윙 도중(타격이든, 그 뒤 자동 복귀든) Shoot/Random Shoot으로 새
        // 공을 쏘는 상황(Shoot 하다 Random Shoot 하면 로봇이 멈춘다는 사용자
        // 재현)을 다양한 끼어들기 시점으로 재현한다. 매 라운드 끝에 방해
        // 없는 평범한 샷을 하나 더 쏴서, 그게 정상적으로 접수되는지로
        // 로봇이 영구적으로 고착됐는지 확인한다.
        for interrupt_after_commit_ms in [10_u64, 50, 120, 250, 400, 600] {
            let arm = test_arm();
            let mut world = SimWorld::new(arm, None);
            world.set_use_ground_truth(true);

            world.shoot_ball(&BallShooterSettings::default().randomized(&mut rng));
            let mut committed = false;
            for _ in 0..800 {
                world.step(1.0 / 1000.0, None);
                if world.swing_committed() {
                    committed = true;
                    break;
                }
            }
            assert!(
                committed,
                "interrupt_after_commit_ms={interrupt_after_commit_ms} — 1구 스윙이 commit 안 됨"
            );
            for _ in 0..interrupt_after_commit_ms {
                world.step(1.0 / 1000.0, None);
            }

            // 2구: 1구의 타격·팔로스루·자동 복귀 중 어느 시점이든 끊고 새로
            // (역시 랜덤) 쏜다.
            world.shoot_ball(&BallShooterSettings::default().randomized(&mut rng));
            for _ in 0..6_000 {
                world.step(1.0 / 1000.0, None);
                if world.ball_state == BallState::Parked {
                    break;
                }
            }

            // 3구: 방해 없이 평범하게 쏜다 — 앞선 끼어들기로 로봇이
            // 영구적으로 망가지지 않았다면 이번엔 정상적으로 접수해야 한다.
            world.shoot_ball(&BallShooterSettings::default());
            let mut racket_contact = false;
            for _ in 0..5_000 {
                world.step(1.0 / 1000.0, None);
                if world.robot().is_swinging() {
                    racket_contact = true;
                    break;
                }
            }
            assert!(
                racket_contact,
                "interrupt_after_commit_ms={interrupt_after_commit_ms} — 끼어들기 이후 \
                 3구(방해 없음)는 스윙이 시작돼야 하는데 안 됨 — 로봇이 고착된 것으로 \
                 보임 (rail={}, joints={:?})",
                world.robot().rail_x(),
                world.robot().joints().values,
            );
        }
    }

    #[test]
    fn random_shot_grid_clears_net_and_returns() {
        for lateral in [-0.5_f64, -0.25, 0.0, 0.25, 0.5] {
            let (yaw_min, yaw_max) = BallShooterSettings::yaw_range_for_lateral_deg(lateral);
            for yaw in [yaw_min, yaw_max] {
                for speed in [
                    crate::sim::shooter::RANDOM_SHOT_SPEED_MIN_MPS,
                    crate::sim::shooter::RANDOM_SHOT_SPEED_MAX_MPS,
                ] {
                    let settings = BallShooterSettings {
                        lateral_offset_m: lateral,
                        yaw_deg: yaw,
                        speed_mps: speed,
                        ..BallShooterSettings::default()
                    };

                    let arm = test_arm();
                    let mut world = SimWorld::new(arm.clone(), None);
                    world.set_use_ground_truth(true);

                    let collider_for_body = |world: &SimWorld, body_handle| {
                        world
                            .collider_set
                            .iter()
                            .find_map(|(handle, collider)| {
                                (collider.parent() == Some(body_handle)).then_some(handle)
                            })
                            .expect("body collider")
                    };
                    let ball_collider = collider_for_body(&world, world.ball_handle);
                    let racket_collider = collider_for_body(&world, world.racket_handle);
                    let net_collider = world
                        .collider_set
                        .iter()
                        .find_map(|(handle, collider)| {
                            let cuboid = collider.shape().as_cuboid()?;
                            ((f64::from(cuboid.half_extents.y) - 0.005).abs() < 1e-6)
                                .then_some(handle)
                        })
                        .expect("net collider");

                    world.shoot_ball(&settings);

                    let mut racket_contact = false;
                    let mut returned = false;
                    for _ in 0..5_000 {
                        world.step(1.0 / 1000.0, None);

                        assert!(
                            !world
                                .narrow_phase
                                .contact_pair(ball_collider, net_collider)
                                .is_some_and(ContactPair::has_any_active_contact),
                            "lateral={lateral:+.2} yaw={yaw:+.2} speed={speed:.2} — \
                             네트에 맞음"
                        );

                        if world
                            .narrow_phase
                            .contact_pair(ball_collider, racket_collider)
                            .is_some_and(ContactPair::has_any_active_contact)
                        {
                            racket_contact = true;
                        }
                        if racket_contact && world.ball_velocity().y > 0.0 {
                            returned = true;
                            break;
                        }
                    }

                    assert!(
                        racket_contact,
                        "lateral={lateral:+.2} yaw={yaw:+.2} speed={speed:.2} — \
                         라켓 접수 없음"
                    );
                    assert!(
                        returned,
                        "lateral={lateral:+.2} yaw={yaw:+.2} speed={speed:.2} — \
                         라켓 접수 뒤 리턴 안 됨"
                    );
                }
            }
        }
    }
}
