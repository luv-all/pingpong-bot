//! Rapier3d 시뮬레이션 월드.
//!
//! 탁구대·로봇(-x) · 슈터(+x) · 공. 공은 슈터에 주차되어 있다가
//! GUI 트리거로 발사되고, 로봇이 라켓으로 받는다.

use std::sync::Arc;

use crate::{
    Arm, DomainError, InterceptWindow, PhysicsParams, Prediction, RobotPose, RobotState,
    SwingPlanError, ball_past_midcourt_for_commit, constants::{ball, table}, in_swing_commit_window,
    plan_best_swing,
};
use rapier3d::prelude::*;
use tracing::{debug, warn};

use super::arm_bodies::ArmMultibody;
use super::shooter::{BallShooterSettings, BallState};
use crate::sim::estimator::predict_impact;
use crate::sim::gui::debug_snap::{CommitPhase, SimDebugSnapshot};

/// 한 물리 스텝 입력 — `controls` 뮤텍스를 물리 연산 동안 잡지 않기 위함.
pub struct SimStepInput<'a> {
    /// 현재 슈터 설정
    pub shooter: &'a BallShooterSettings,
    /// 이번 스텝에 발사
    pub shoot: bool,
    /// 이번 스텝에 주차
    pub park: bool,
}

/// Rapier 물리 월드 — 탁구대, 슈터, 공, 다물체 암(EE 충돌 · τ_max · 폐루프 관절).
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
    /// 라켓(EE 링크) 강체 핸들 — 다물체 EE
    pub racket_handle: RigidBodyHandle,
    /// 슈터 본체 (고정)
    pub shooter_handle: RigidBodyHandle,
    /// 다물체 암 (τ_max 모터 · 관성 · EE 충돌)
    pub arm_bodies: ArmMultibody,
    /// 불변 로봇 기구 모델
    pub arm: Arc<Arm>,
    /// 테이블·공 반발 등
    pub physics: PhysicsParams,
    /// URDF 기반 FK·뷰어 (선택)
    pub urdf: Option<Arc<crate::robot::urdf::UrdfModel>>,
    /// 런타임 관절 상태 (명령 / 플래너)
    pub robot: RobotState,
    /// sim 경과 시간 [s]
    pub sim_time: f64,
    /// 공 주차/비행
    pub ball_state: BallState,
    /// 마지막 발사 설정 (상태 표시용)
    pub last_shooter_settings: BallShooterSettings,
    /// 디버그 — 마지막 hit plane 예측 (뷰어 마커용)
    debug_prediction: Option<Prediction>,
    /// 동적으로 탐색할 접수 y 구간.
    intercept: InterceptWindow,
    /// true면 Rapier ground truth로 자동 스윙 (sim 기본).
    /// false면 카메라→DLT→EKF→control이 타격.
    use_ground_truth: bool,
    /// 이번 비행에서 스윙을 이미 commit했는지 (재계획·팔 떨림 방지)
    swing_committed: bool,
    /// 이번 비행에서 스윙을 포기했는지 (도달 불능·너무 늦음). commit 없이 손 뗌.
    swing_abandoned: bool,
    /// commit 창 안에서의 연속 하드 불능(IK/충돌/리턴) 횟수.
    /// 한 번 실패로 바로 포기하면 예측이 잠시 어긋난 공을 놓치므로,
    /// 연속 하드 계획 실패 횟수. 비행 포기는 `tti < min_swing`에서만 하며,
    /// 이때 스트릭이 있으면 사유 로그에 남긴다.
    hard_fail_streak: u32,
    /// 마지막으로 `plan_best_swing`을 실제로 시도한 `sim_time`.
    ///
    /// `InsufficientTime`(아직 이름)은 재시도하되, 매 틱 IK를 돌리지 않도록
    /// `SWING_RETRY_THROTTLE_SECS`로 빈도만 제한한다.
    last_swing_attempt_at: f64,
    /// 이번 비행이 발사된 `sim_time` — `park_if_out_of_play`의 최대 비행
    /// 시간 안전장치(`MAX_BALL_FLIGHT_SECS`)가 기준으로 삼는다.
    flight_started_at: f64,
    /// 뷰어·Status용 디버그 스냅샷 (실패 사유·궤적·한계).
    debug_snap: SimDebugSnapshot,
}

impl SimWorld {
    /// 탁구대·슈터·주차된 공·로봇 라켓을 배치한다.
    ///
    /// 제어·Rapier 라켓·URDF 뷰어는 같은 관절 순서와 기구학을 사용한다.
    pub fn new(robot: crate::robot::Robot) -> Self {
        return Self::with_physics(robot, crate::defaults::physics());
    }

    /// config `[physics]` 반발 등을 Rapier collider에 반영한다.
    pub fn with_physics(robot: crate::robot::Robot, physics: PhysicsParams) -> Self {
        let crate::robot::Robot { arm, urdf } = robot;
        let mut integration_parameters = IntegrationParameters::default();
        integration_parameters.dt = 1.0 / 1000.0;
        // 다물체 + 공 접촉: 12가 키네마틱 라켓 리턴 임펄스에 필요 (8이면 스침만 기록).
        integration_parameters.num_solver_iterations = 12;

        let mut rigid_body_set = RigidBodySet::new();
        let mut collider_set = ColliderSet::new();
        let mut multibody_joint_set = MultibodyJointSet::new();

        // 제어 DOF = Arm. URDF default(예: 3축)로 초기화하면 plan_swing과 어긋난다.
        let robot = arm.initial_state();

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
        .collision_groups(super::arm_bodies::static_collision_groups())
        .restitution(physics.restitution as f32)
        .friction(physics.friction as f32)
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
        .collision_groups(super::arm_bodies::static_collision_groups())
        .restitution(physics.net_restitution as f32)
        .build();
        collider_set.insert_with_parent(net_collider, net_handle, &mut rigid_body_set);

        // 슈터 본체 (+y) — 포즈만 유지, 충돌 없음 (뷰어 표시 전용).
        // 초기 위치는 아래에서 sync_shooter_pose로 발사구에 맞춘다.
        let shooter_body = RigidBodyBuilder::fixed().build();
        let shooter_handle = rigid_body_set.insert(shooter_body);

        let default_shooter = BallShooterSettings::default();

        // 다물체 암: SerialChain 정합 + EE 충돌 (키네마틱 라켓 없음).
        let mount = nalgebra::Vector3::new(robot.rail_x(), arm.base.coords.y, arm.base.coords.z);
        let arm_bodies = ArmMultibody::spawn(
            &mut rigid_body_set,
            &mut collider_set,
            &mut multibody_joint_set,
            &arm,
            mount,
            robot.joints(),
            // 라켓 e ≠ 테이블 e. combine Min → 공–라켓 접촉이 e_eff.
            crate::defaults::impact().racket_effective_restitution as f32,
        );
        let racket_handle = arm_bodies
            .racket_handle()
            .expect("multibody EE racket link");

        let muzzle = default_shooter.muzzle_position();
        let ball_body = RigidBodyBuilder::fixed()
            .translation(muzzle)
            // 공기 토크로 스핀이 서서히 감쇠 — 바운스 마찰로 생긴 과한 ω가
            // Magnus로 탄도를 폭주시키지 않게 한다 (슈터 의도 스핀은 짧은
            // 비행에 충분히 남음).
            .angular_damping(ball::ANGULAR_DAMPING as f32)
            .build();
        let ball_handle = rigid_body_set.insert(ball_body);
        let ball_collider = ColliderBuilder::ball(ball::RADIUS as f32)
            .collision_groups(super::arm_bodies::ball_collision_groups())
            .restitution(physics.restitution as f32)
            .friction(physics.ball_friction as f32)
            // ITTF 질량 + 중공 셸 I=(2/3)mr² (Rapier 기본 솔리드 2/5 대신).
            .mass_properties(MassProperties::new(
                Vector::ZERO,
                ball::MASS as f32,
                Vector::splat(ball::SHELL_INERTIA as f32),
            ))
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
            multibody_joint_set,
            ccd_solver: CCDSolver::new(),
            gravity: Vector::new(0.0, 0.0, -9.81),
            ball_handle,
            racket_handle,
            shooter_handle,
            arm_bodies,
            arm,
            physics,
            urdf,
            robot,
            sim_time: 0.0,
            ball_state: BallState::Parked,
            last_shooter_settings: default_shooter.clone(),
            debug_prediction: None,
            intercept: crate::defaults::intercept(),
            use_ground_truth: true,
            swing_committed: false,
            swing_abandoned: false,
            hard_fail_streak: 0,
            last_swing_attempt_at: f64::NEG_INFINITY,
            flight_started_at: 0.0,
            debug_snap: SimDebugSnapshot::default(),
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

    /// 이번 공 스윙을 포기했는지 (도달 불능·시간 부족).
    pub fn swing_abandoned(&self) -> bool {
        return self.swing_abandoned;
    }

    /// 뷰어·Status용 디버그 스냅샷.
    pub fn debug_snap(&self) -> &SimDebugSnapshot {
        return &self.debug_snap;
    }

    /// control/ground truth 경로가 스윙을 commit했음을 표시한다.
    pub fn mark_swing_committed(&mut self) {
        self.swing_committed = true;
        self.debug_snap.commit_phase = CommitPhase::Committed;
    }

    /// 물리 1스텝: GUI 요청 처리 → 관절 추종 → Rapier 적분.
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

        // B: 명령(궤적→모터 목표) → 물리 → 측정 관절각을 RobotState에 반영.
        self.robot.step_commands(&self.arm, dt);
        self.try_auto_swing();
        self.drive_arm_motors();
        self.apply_ball_aero_forces();

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

        if let Some(&first) = self.arm_bodies.joint_handles.first()
            && let Some((mb, _)) = self.multibody_joint_set.get_mut(first)
        {
            mb.forward_kinematics(&mut self.rigid_body_set, true);
            mb.update_rigid_bodies(&mut self.rigid_body_set, true);
        }

        let measured = self.arm_bodies.read_joint_angles(&self.multibody_joint_set);
        self.robot.set_measured_joints(measured);

        self.sim_time += dt;
        self.refresh_debug_snap();

        if self.ball_state == BallState::InFlight {
            self.park_if_out_of_play();
        }
    }

    /// 매 스텝 디버그 스냅샷(관통·ω·탄도 등)을 갱신한다.
    fn refresh_debug_snap(&mut self) {
        let bp = self.ball_position();
        let bv = self.ball_velocity();
        let aw = self.ball_angular_velocity();
        let ball_pos = nalgebra::Vector3::new(f64::from(bp.x), f64::from(bp.y), f64::from(bp.z));
        let ball_vel = nalgebra::Vector3::new(f64::from(bv.x), f64::from(bv.y), f64::from(bv.z));
        let omega = nalgebra::Vector3::new(f64::from(aw.x), f64::from(aw.y), f64::from(aw.z));
        let hit_y = self
            .debug_prediction
            .as_ref()
            .map(|p| p.impact_position.coords.y)
            .unwrap_or(table::DEFAULT_HIT_PLANE_Y);
        let rail_x = self.robot.rail_x();
        let joints = self.robot.joints().clone();
        let in_flight = self.ball_state == BallState::InFlight;
        let physics = self.physics;
        self.debug_snap.refresh_runtime(
            &self.arm,
            rail_x,
            &joints,
            ball_pos,
            ball_vel,
            omega,
            in_flight,
            &physics,
            hit_y,
        );
    }

    /// 비행 중 공에 항력·Magnus 외력을 건다 (중력은 Rapier gravity).
    ///
    /// ballistics `aero_accel`과 동일 식 — 예측기와 Rapier 궤적을 맞춘다.
    fn apply_ball_aero_forces(&mut self) {
        if self.ball_state != BallState::InFlight {
            return;
        }
        let Some(body) = self.rigid_body_set.get_mut(self.ball_handle) else {
            return;
        };
        body.reset_forces(true);
        let lin = body.linvel();
        let ang = body.angvel();
        let velocity = nalgebra::Vector3::new(f64::from(lin.x), f64::from(lin.y), f64::from(lin.z));
        let omega = nalgebra::Vector3::new(f64::from(ang.x), f64::from(ang.y), f64::from(ang.z));
        let a = crate::planner::physics::aero_accel(
            velocity,
            omega,
            self.physics.drag,
            self.physics.magnus,
        );
        let mass = f64::from(body.mass());
        if mass <= 1e-12 {
            return;
        }
        let force = a * mass;
        body.add_force(
            Vector::new(force.x as f32, force.y as f32, force.z as f32),
            true,
        );
    }

    /// 슈터 본체 위치·회전을 설정에 맞춘다 (발사구가 전면에 오도록).
    pub fn sync_shooter_pose(&mut self, settings: &BallShooterSettings) {
        let pos = settings.visual_position();
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

    /// 공 비행 중 commit 창에 들어올 때 스윙을 계획한다.
    ///
    /// - 도달 불능(IK/충돌/리턴 불가): 그 시도는 즉시 버린다(억지 commit 없음).
    ///   초·중반 예측이 틀릴 수 있어 비행 전체 포기는 하지 않고 재시도한다.
    /// - `InsufficientTime`: 스로틀 재시도. 모든 후보가 `tti < min_swing`이면 포기.
    /// - 포기 후에는 팔이 움직이지 않는다.
    fn try_auto_swing(&mut self) {
        const SWING_RETRY_THROTTLE_SECS: f64 = 0.02;

        if self.ball_state != BallState::InFlight {
            return;
        }

        // 비행 중에는 항상 디버그 마커를 최신 탄도로 갱신 (커밋 후에도 스윙 재계획 없음).
        let marker = self
            .intercept
            .hit_planes()
            .into_iter()
            .find_map(|plane| predict_impact(self, plane));

        if !self.use_ground_truth {
            if let Some(prediction) = marker {
                self.set_debug_prediction(Some(prediction));
            }
            return;
        }

        if self.swing_committed || self.swing_abandoned || self.robot.is_swinging() {
            if let Some(prediction) = marker {
                self.set_debug_prediction(Some(prediction));
            }
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
            self.debug_snap.commit_phase = CommitPhase::WaitMidcourt;
            if let Some(prediction) = predictions.first() {
                self.set_debug_prediction(Some(prediction.clone()));
            }
            return;
        }

        let min_swing = crate::defaults::control().min_swing_secs;
        let soonest_tti = predictions
            .iter()
            .map(|p| p.time_to_impact_secs)
            .fold(f64::INFINITY, f64::min);
        // 모든 후보가 최소 스윙 시간보다 짧음 → 물리적으로 안전한 스윙 불가.
        if soonest_tti < min_swing {
            let reason = if self.hard_fail_streak > 0 {
                format!(
                    "tti < min_swing (하드 실패 {}회 후 너무 늦음)",
                    self.hard_fail_streak
                )
            } else {
                "tti < min_swing — 너무 늦음".to_string()
            };
            self.debug_snap.commit_phase = CommitPhase::TooLate;
            self.abandon_swing(&reason);
            if let Some(prediction) = predictions.first() {
                self.set_debug_prediction(Some(prediction.clone()));
            }
            return;
        }

        // commit 창 밖(너무 이름)이면 계획하지 않고 대기.
        let any_in_window = predictions
            .iter()
            .any(|p| in_swing_commit_window(p.time_to_impact_secs));
        if !any_in_window {
            self.debug_snap.commit_phase = CommitPhase::WaitWindow;
            if let Some(prediction) = predictions.first() {
                self.set_debug_prediction(Some(prediction.clone()));
            }
            return;
        }

        self.debug_snap.commit_phase = CommitPhase::InWindow;

        if self.sim_time - self.last_swing_attempt_at < SWING_RETRY_THROTTLE_SECS {
            if let Some(prediction) = predictions.first() {
                self.set_debug_prediction(Some(prediction.clone()));
            }
            return;
        }
        self.last_swing_attempt_at = self.sim_time;
        let start = RobotPose::new(self.robot.rail_x(), self.robot.joints().clone());
        let planned = match plan_best_swing(&self.arm, &predictions, &start) {
            Ok(planned) => {
                self.hard_fail_streak = 0;
                planned
            }
            Err(DomainError::InfeasibleSwing(ref err)) if err.is_hard_unreachable() => {
                // 이번 시도만 스킵. 비행 포기는 tti < min_swing에서만 —
                // 초반 hit-plane 오판으로 닿는 공을 버리지 않기 위함.
                self.hard_fail_streak = self.hard_fail_streak.saturating_add(1);
                self.debug_snap.record_fail(err);
                debug!(
                    %err,
                    streak = self.hard_fail_streak,
                    soonest_tti,
                    "plan_swing 하드 불능 — 재시도"
                );
                if let Some(prediction) = predictions.first() {
                    self.set_debug_prediction(Some(prediction.clone()));
                }
                return;
            }
            Err(DomainError::InfeasibleSwing(ref err @ SwingPlanError::InsufficientTime { .. })) => {
                self.debug_snap.record_fail(err);
                debug!("plan_swing InsufficientTime — 재시도 대기");
                if let Some(prediction) = predictions.first() {
                    self.set_debug_prediction(Some(prediction.clone()));
                }
                return;
            }
            Err(other) => {
                self.hard_fail_streak = self.hard_fail_streak.saturating_add(1);
                self.debug_snap.last_fail_text = Some(other.to_string());
                warn!(%other, streak = self.hard_fail_streak, "sim 자동 스윙 계획 실패");
                if let Some(prediction) = predictions.first() {
                    self.set_debug_prediction(Some(prediction.clone()));
                }
                return;
            }
        };
        self.debug_snap.clear_fail_on_success();
        self.set_debug_prediction(Some(planned.prediction));
        let trajectory = planned.trajectory;
        self.debug_snap
            .set_committed_path(&self.arm, &trajectory);
        debug!(
            duration_secs = trajectory.duration_secs,
            rail_end = trajectory.rail.end,
            end_vel = ?trajectory.end_velocity,
            "sim plan_swing commit"
        );
        self.robot.replace_swing(trajectory);
        self.swing_committed = true;
    }

    fn abandon_swing(&mut self, reason: &str) {
        self.swing_abandoned = true;
        self.debug_snap.record_abandon_text(reason);
        debug!(%reason, "이번 비행 스윙 포기 — 팔 고정");
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
            [muzzle.x, muzzle.y, muzzle.z],
            [linvel.x, linvel.y, linvel.z],
            [angvel.x, angvel.y, angvel.z],
        );
    }

    /// 위치·속도로 공을 dynamic 비행 상태로 만든다.
    pub fn launch_ball_at(
        &mut self,
        position: [f32; 3],
        linear_velocity: [f32; 3],
        angular_velocity: [f32; 3],
    ) {
        if let Some(body) = self.rigid_body_set.get_mut(self.ball_handle) {
            body.set_body_type(RigidBodyType::Dynamic, true);
            body.set_translation(Vector::new(position[0], position[1], position[2]), true);
            body.set_linvel(
                Vector::new(linear_velocity[0], linear_velocity[1], linear_velocity[2]),
                true,
            );
            body.set_angvel(
                Vector::new(angular_velocity[0], angular_velocity[1], angular_velocity[2]),
                true,
            );
            body.enable_ccd(true);
        }
        self.ball_state = BallState::InFlight;
        self.robot.cancel_swing();
        self.swing_committed = false;
        self.swing_abandoned = false;
        self.hard_fail_streak = 0;
        self.last_swing_attempt_at = f64::NEG_INFINITY;
        self.flight_started_at = self.sim_time;
        self.debug_snap.reset_for_new_flight();
        self.try_auto_swing();
    }

    /// 공을 슈터 발사구에 주차한다.
    ///
    /// 스윙/중앙 복귀 궤적은 유지한다 — 공 회수로 복귀를 끊으면
    /// (`cancel_swing`) 레일·관절이 스윙 끝에 멈춰 다음 샷이 깨진다.
    /// 새 발사(`launch_ball_at`)만 진행 중 스윙을 취소한다.
    pub fn park_ball(&mut self, settings: &BallShooterSettings) {
        self.debug_prediction = None;
        self.last_shooter_settings = settings.clone();
        self.sync_shooter_pose(settings);
        let muzzle = settings.muzzle_position();
        if let Some(body) = self.rigid_body_set.get_mut(self.ball_handle) {
            body.set_body_type(RigidBodyType::Fixed, true);
            body.set_translation(muzzle, true);
            body.set_linvel(Vector::ZERO, true);
            body.set_angvel(Vector::ZERO, true);
            body.reset_forces(true);
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

    /// 공 각속도 [rad/s].
    pub fn ball_angular_velocity(&self) -> Vector {
        return self.rigid_body_set[self.ball_handle].angvel();
    }

    /// 슈터 본체 위치·회전 (kiss3d 동기화용).
    pub fn shooter_pose(&self) -> (Vector, Rotation) {
        let body = &self.rigid_body_set[self.shooter_handle];
        return (body.translation(), *body.rotation());
    }

    /// 라켓 EE 위치·회전 (`+Z` = 면 법선). 링크 원점이 아니라 collider 프레임.
    pub fn racket_pose(&self) -> (Vector, Rotation) {
        let iso = self
            .arm_bodies
            .ee_world_isometry(&self.rigid_body_set)
            .expect("EE link");
        let t = iso.translation.vector;
        let q = iso.rotation.quaternion();
        return (
            Vector::new(t.x as f32, t.y as f32, t.z as f32),
            Rotation::from_xyzw(q.i as f32, q.j as f32, q.k as f32, q.w as f32),
        );
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
    pub fn urdf(&self) -> Option<&crate::robot::urdf::UrdfModel> {
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
            position: [self.arm.base.coords.x, self.arm.base.coords.y, self.arm.base.coords.z],
            rpy: [0.0, 0.0, 0.0],
        };
    }

    /// 레일 베이스 + τ_max 모터 목표 (다물체 추종). 목표는 명령 `targets`.
    fn drive_arm_motors(&mut self) {
        let mount = self.effective_sim_mount();
        self.arm_bodies.set_base_xy(
            &mut self.rigid_body_set,
            &mut self.multibody_joint_set,
            mount.position[0],
            mount.position[1],
            mount.position[2],
        );
        let targets = self.robot.targets().clone();
        self.arm_bodies
            .set_motor_targets(&mut self.multibody_joint_set, &targets);
    }

    /// 테스트: yaw 모터 max_force를 덮어쓴다.
    #[cfg(test)]
    pub fn set_yaw_motor_max_force_for_test(&mut self, tau0: f64) {
        let mut torques = crate::defaults::control().max_joint_torques;
        torques[0] = tau0;
        self.arm_bodies
            .set_motor_max_forces(&mut self.multibody_joint_set, &torques);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::sim::BallShooterSettings;

    use crate::{RobotPose, constants::table};

    fn test_robot() -> crate::robot::Robot {
        return crate::defaults::primitive_4dof().expect("테스트용 4DOF robot");
    }

    #[test]
    fn ball_mass_properties_match_ittf_thin_shell() {
        let world = SimWorld::new(test_robot());
        let body = world
            .rigid_body_set
            .get(world.ball_handle)
            .expect("ball body");
        let mass = f64::from(body.mass());
        assert!(
            (mass - ball::MASS).abs() < 1e-9,
            "mass={mass} want {}",
            ball::MASS
        );
        let inertia = body.mass_properties().local_mprops.principal_inertia();
        for axis in [inertia.x, inertia.y, inertia.z] {
            assert!(
                (f64::from(axis) - ball::SHELL_INERTIA).abs() < 1e-12,
                "I={axis} want {}",
                ball::SHELL_INERTIA
            );
        }
    }

    #[test]
    fn ball_stays_parked_until_shoot() {
        let arm = test_robot();
        let mut world = SimWorld::new(arm);
        let y0 = world.ball_position().y;
        for _ in 0..200 {
            world.step(1.0 / 1000.0, None);
        }
        assert_eq!(world.ball_state, BallState::Parked);
        assert!((world.ball_position().y - y0).abs() < 1e-4);
    }

    #[test]
    fn shoot_sends_ball_toward_robot_side() {
        let arm = test_robot();
        let mut world = SimWorld::new(arm);
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
    fn hard_unreachable_flight_is_abandoned_without_committing_swing() {
        // 리치 밖 가장자리로 조준 — commit 창에서 IK가 계속 실패하다
        // tti < min_swing이 되면 포기 (억지 commit 없음).
        let mut world = SimWorld::new(fourdof_robot());
        world.set_use_ground_truth(true);
        world.set_intercept_window(crate::defaults::intercept());
        let settings = BallShooterSettings {
            lateral_offset_m: 0.5,
            yaw_deg: -28.0,
            speed_mps: 5.7,
            ..BallShooterSettings::default()
        };
        world.shoot_ball(&settings);

        let mut saw_abandon = false;
        for _ in 0..8_000 {
            world.step(1.0 / 1000.0, None);
            if world.swing_abandoned() {
                saw_abandon = true;
                assert!(
                    !world.swing_committed(),
                    "포기한 비행은 commit되면 안 됨"
                );
                assert!(
                    !world.robot().is_swinging(),
                    "포기 후 팔이 스윙 중이면 안 됨"
                );
                break;
            }
            if world.ball_state == BallState::Parked {
                break;
            }
        }
        assert!(
            saw_abandon,
            "리치 밖 샷은 swing_abandoned 되어야 함 (committed={})",
            world.swing_committed()
        );
    }

    #[test]
    fn default_shot_still_commits_when_reachable() {
        let mut world = SimWorld::new(fourdof_robot());
        world.set_use_ground_truth(true);
        world.set_intercept_window(crate::defaults::intercept());
        world.shoot_ball(&BallShooterSettings::default());
        for _ in 0..8_000 {
            world.step(1.0 / 1000.0, None);
            if world.swing_committed() || world.robot().is_swinging() {
                assert!(!world.swing_abandoned());
                return;
            }
            if world.ball_state == BallState::Parked {
                break;
            }
        }
        panic!(
            "기본 샷은 commit 되어야 함 abandoned={} committed={}",
            world.swing_abandoned(),
            world.swing_committed()
        );
    }

    #[test]
    fn contact_swing_reaches_impact_fk_at_duration() {
        let arm = test_robot();
        let mut world = SimWorld::new(arm.clone());
        world.set_use_ground_truth(true);
        world.shoot_ball(&BallShooterSettings::default());

        let mut started = false;
        for _ in 0..800 {
            world.step(1.0 / 1000.0, None);
            if world.robot().is_swinging() || world.swing_committed() {
                started = true;
                break;
            }
        }
        assert!(started, "네트 통과 후 commit 창에서 스윙이 시작되어야 함");
        for _ in 0..800 {
            world.step(1.0 / 1000.0, None);
        }
        assert!(
            world.robot().rail_x() > 0.2,
            "스윙 중 레일이 impact 쪽으로 이동해야 함"
        );
    }

    #[test]
    fn simworld_ee_tracks_fk_with_direct_motor_ramp() {
        let arm = test_robot();
        let mut world = SimWorld::new(arm.clone());
        let start = world.robot().joints().clone();
        let mut impact = start.clone();
        impact.values[1] += 0.2;
        impact.values[2] -= 0.3;
        let mount = world.effective_sim_mount().position;
        let mut max_err = 0.0_f64;
        for step in 0..300 {
            let t = ((step as f64) / 250.0).min(1.0);
            let mut target = start.clone();
            for i in 0..target.values.len().min(impact.values.len()) {
                target.values[i] = start.values[i] + t * (impact.values[i] - start.values[i]);
            }
            world.arm_bodies.set_base_xy(
                &mut world.rigid_body_set,
                &mut world.multibody_joint_set,
                mount[0],
                mount[1],
                mount[2],
            );
            world
                .arm_bodies
                .set_motor_targets(&mut world.multibody_joint_set, &target);
            let gravity = world.gravity;
            let params = world.integration_parameters;
            world.physics_pipeline.step(
                gravity,
                &params,
                &mut world.island_manager,
                &mut world.broad_phase,
                &mut world.narrow_phase,
                &mut world.rigid_body_set,
                &mut world.collider_set,
                &mut world.impulse_joint_set,
                &mut world.multibody_joint_set,
                &mut world.ccd_solver,
                &(),
                &(),
            );
            let read = world.arm_bodies.read_joint_angles(&world.multibody_joint_set);
            let fk = arm
                .arm
                .forward_kinematics_with_rail(0.0, &read)
                .expect("fk")
                .position
                .coords;
            let ee = world
                .arm_bodies
                .ee_world_translation(&world.rigid_body_set)
                .expect("ee");
            max_err = max_err.max((ee - fk).norm());
        }
        assert!(
            max_err < 0.01,
            "direct motor ramp in SimWorld EE↔FK max_err={max_err:.4}"
        );
    }

    #[test]
    fn simworld_ee_tracks_fk_during_commanded_swing() {
        let arm = test_robot();
        let mut world = SimWorld::new(arm.clone());
        let start = world.robot().joints().clone();
        let mut impact = start.clone();
        impact.values[1] += 0.2;
        impact.values[2] -= 0.3;
        let traj = crate::SwingTrajectory::new(
            start,
            impact,
            vec![0.0; 4],
            vec![0.0; 4],
            0.25,
            crate::RailMotion::fixed(world.robot().rail_x()),
        );
        world.robot_mut().begin_swing(traj);
        let mut max_err = 0.0_f64;
        let mut max_err_tgt = 0.0_f64;
        for _ in 0..300 {
            world.step(1.0 / 1000.0, None);
            let fk = world
                .robot()
                .racket_pose(&arm.arm)
                .expect("fk")
                .position
                .coords;
            let fk_tgt = arm
                .arm
                .forward_kinematics_with_rail(world.robot().rail_x(), world.robot().targets())
                .expect("fk tgt")
                .position
                .coords;
            let ee = world
                .arm_bodies
                .ee_world_translation(&world.rigid_body_set)
                .expect("ee");
            max_err = max_err.max((ee - fk).norm());
            max_err_tgt = max_err_tgt.max((ee - fk_tgt).norm());
        }
        assert!(
            max_err < 0.01,
            "SimWorld swing EE↔FK(measured) max_err={max_err:.4} EE↔FK(targets)={max_err_tgt:.4}"
        );
    }

    #[test]
    fn ball_contacts_ee_collider_when_overlapping() {
        let arm = test_robot();
        let mut world = SimWorld::new(arm.clone());
        let pose = world.robot().racket_pose(&arm.arm).expect("fk");
        let p = pose.position.coords;
        // 라켓 중심에 공을 겹치게 둔다.
        if let Some(body) = world.rigid_body_set.get_mut(world.ball_handle) {
            body.set_body_type(RigidBodyType::Dynamic, true);
            body.set_translation(
                Vector::new(p.x as f32, p.y as f32, p.z as f32),
                true,
            );
            body.set_linvel(Vector::new(0.0, 0.0, 0.0), true);
        }
        let ball_c = world
            .collider_set
            .iter()
            .find_map(|(h, c)| (c.parent() == Some(world.ball_handle)).then_some(h))
            .expect("ball collider");
        let racket_c = world
            .collider_set
            .iter()
            .find_map(|(h, c)| (c.parent() == Some(world.racket_handle)).then_some(h))
            .expect("racket collider");

        world.step(1.0 / 1000.0, None);
        let pair = world.narrow_phase.contact_pair(ball_c, racket_c);
        assert!(
            pair.is_some(),
            "overlapping ball/EE should create a contact pair"
        );
        assert!(
            pair.is_some_and(ContactPair::has_any_active_contact),
            "contact should be active"
        );
    }

    #[test]
    fn ground_truth_rally_contacts_racket_clears_net_and_bounces_near_center() {
        let arm = test_robot();
        let mut world = SimWorld::new(arm);
        world.set_use_ground_truth(true);
        world.set_intercept_window(crate::defaults::intercept());

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

        assert!(
            world.swing_committed() || world.robot().is_swinging() || world.robot().rail_x() > 0.05,
            "스윙이 계획·실행되어야 함"
        );
        assert!(racket_contact, "라켓·공 접촉이 있어야 함");
        assert!(returned, "라켓 접촉 뒤 공의 vy가 +여야 함");
        let net_z = net_clearance.unwrap_or_else(|| {
            panic!("리턴 공이 네트를 통과해야 함: contact={contact_state:?}, max_y={max_return_y}")
        });
        assert!(
            f64::from(net_z)
                >= table::SURFACE_Z + table::NET_HEIGHT + crate::constants::BALL_RADIUS,
            "네트 통과 높이 부족: {net_z}"
        );
        let bounce = match bounce {
            Some(b) => b,
            None => {
                // 끝선 슈터는 비행거리가 길어 리턴이 테이블 끝을 넘길 수 있다.
                // 네트 통과 + 상대 코트 진입이면 랠리 성공으로 본다.
                assert!(
                    max_return_y > net_y + 0.2,
                    "리턴이 상대 코트로 진행해야 함: max_y={max_return_y} contact={contact_state:?}"
                );
                return;
            }
        };
        let target_x = (table::WIDTH_X * 0.5) as f32;
        let target_y = (table::LENGTH_Y * 0.75) as f32;
        // 탄도 목표는 스핀 무시. 중공 셸 I=(2/3)mr²이면 마찰→ω 결합이
        // 솔리드(2/5)와 달라 착지 y가 수 cm 어긋날 수 있다.
        assert!(
            (bounce.x - target_x).abs() <= 0.20 && (bounce.y - target_y).abs() <= 0.50,
            "bounce={bounce:?}, target=({target_x}, {target_y}), contact={contact_state:?}"
        );
    }

    /// 진단용 — `defaults::urdf_4dof` (URDF + RobotBuilder).
    fn fourdof_robot() -> crate::Robot {
        return crate::defaults::urdf_4dof().expect("4-dof URDF");
    }

    /// 기본 슈터 샷이 네트 위를 여유 있게 지나가는지 회귀 검증한다.
    ///
    /// `pitch_deg=-4.0`이던 예전 기본값은 첫 바운스 뒤 네트를 -0.7cm 차로
    /// 스쳤다. 슈터를 테이블 끝 밖으로 옮긴 뒤 `pitch=-1`·`height=0.28`으로
    /// Rapier·`predict_hit_plane`(네트 게이트)이 같이 통과한다.
    #[test]
    fn default_shot_clears_net_with_margin() {
        let arm = test_robot();
        let mut world = SimWorld::new(arm);
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

    /// `defaults::primitive_4dof()` primitive는 이미 랠리 통합 테스트가 있지만
    /// (`ground_truth_rally_contacts_racket_clears_net_and_bounces_near_center`),
    /// `defaults::urdf_4dof` URDF 로봇은
    /// 한 번도 같은 방식으로 검증된 적이 없었다.
    #[test]
    fn fourdof_ground_truth_rally_contacts_racket_and_returns() {
        let robot = fourdof_robot();
        let arm = robot.arm.clone();
        let mut world = SimWorld::new(robot.clone());
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
        let mut min_ee_ball = f64::MAX;
        let mut max_ee_fk = 0.0_f64;

        for _ in 0..4_000 {
            world.step(1.0 / 1000.0, None);

            let ee_fk = world.robot().racket_pose(&arm).expect("FK").position.coords;
            let ee_phys = world
                .arm_bodies
                .ee_world_translation(&world.rigid_body_set)
                .expect("ee");
            max_ee_fk = max_ee_fk.max((ee_phys - ee_fk).norm());
            let ball = world.ball_position();
            let dx = f64::from(ball.x) - ee_fk.x;
            let dy = f64::from(ball.y) - ee_fk.y;
            let dz = f64::from(ball.z) - ee_fk.z;
            min_dist = min_dist.min((dx * dx + dy * dy + dz * dz).sqrt());
            let ex = f64::from(ball.x) - ee_phys.x;
            let ey = f64::from(ball.y) - ee_phys.y;
            let ez = f64::from(ball.z) - ee_phys.z;
            min_ee_ball = min_ee_ball.min((ex * ex + ey * ey + ez * ez).sqrt());

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
            "4-dof 라켓·공 접촉 없음 — min_fk={min_dist:.4} min_ee={min_ee_ball:.4} max_ee_fk={max_ee_fk:.4} swing={}",
            world.swing_committed()
        );
        assert!(returned, "라켓 접촉 뒤 공의 vy가 +여야 함");
    }

    #[test]
    fn auto_swing_on_shoot_moves_rail() {
        let arm = test_robot();
        assert!(arm.arm.rail.is_some(), "테스트 arm은 리니어 포함");
        let mut world = SimWorld::new(arm);
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
        let arm = test_robot();
        let center_rail_x = arm.arm.rail.as_ref().expect("테스트 arm은 리니어 포함").default_x();
        let center_joints = arm.arm.default_joints.clone();

        let mut world = SimWorld::new(arm);
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

        let arm = test_robot();
        let world = SimWorld::new(arm.clone());
        let rail_x = world.robot().rail_x();
        // 홈 FK z를 써서 손잡이 반영 EE에서도 도달·속도 한계 안에 들게 한다.
        let reachable_z = arm
            .arm
            .forward_kinematics_with_rail(rail_x, world.robot().joints())
            .expect("FK")
            .position
            .coords
            .z;
        let impact = crate::Point3::new(
            table::WIDTH_X * 0.5,
            table::DEFAULT_HIT_PLANE_Y,
            reachable_z,
        );
        let start = RobotPose::new(rail_x, world.robot().joints().clone());
        let traj = plan_swing(
            &arm.arm,
            crate::Prediction {
                time_to_impact_secs: 0.45,
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

        let arm = test_robot();
        let mut world = SimWorld::new(arm.clone());
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
            .arm
            .forward_kinematics_with_rail(world.robot().rail_x(), world.robot().joints())
            .expect("FK");
        let impact = crate::Point3::new(impact_x, hit_plane.y, reachable.position.coords.z);
        let start = RobotPose::new(world.robot().rail_x(), world.robot().joints().clone());
        let trajectory = plan_swing(
            &arm.arm,
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
        let mut world = SimWorld::new(crate::defaults::primitive_4dof().expect("arm"));
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
        let mut world = SimWorld::new(built);
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
            let arm = test_robot();
            let mut world = SimWorld::new(arm);
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

        let arm = test_robot();
        let mut world = SimWorld::new(arm);
        world.set_use_ground_truth(true);

        // 재시도 폭주 버그(수정 전)는 실패한 스윙 계획을 매 틱마다 다시 돌려서
        // "느린 스텝"이 한 비행 내내 수백~수천 번 반복됐다. 수정 후에는 스윙이
        // 끝나는 순간 `plan_return_to_center`가 딱 한 번(그 자체는 몇 ms 걸릴
        // 수 있음) 도는 것만 허용한다.
        //
        // 다물체 암 기본 ON 이후: 간헐적 2~4ms 스파이크는 정상 베이스라인.
        // 폭주는 여전히 "느린 스텝이 수십 개 이상"으로 잡는다.
        // 스핀·Magnus는 접수 예측을 흔들어 재시도가 늘 수 있어, 이 테스트는
        // 조준만 랜덤화한다(스핀 다양성은 GUI·다른 테스트).
        const SLOW_STEP_THRESHOLD: std::time::Duration = std::time::Duration::from_millis(4);
        const MAX_SLOW_STEPS_PER_ROUND: usize = 12;

        let mut worst_step = std::time::Duration::ZERO;
        for round in 0..30 {
            let settings = BallShooterSettings::default().randomized_aim(&mut rng);
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
            worst_step < std::time::Duration::from_millis(100),
            "반복 Random Shoot 중 스텝 하나가 너무 오래 걸림: {worst_step:?}"
        );
    }

    #[test]
    fn random_shot_grid_still_swings_when_robot_starts_from_center() {
        // 실제 GUI 재현: 첫 샷이 끝나면 로봇이 (레일 0이 아니라) 테이블
        // 중앙(`default_x()`)으로 복귀해 있다. 이후 Random Shoot이 쏘는
        // 격자 코너들이, 로봇이 그 중앙 위치에서 시작해도
        // (1) 스윙·접수하거나 (2) 도달 불능이면 명시적으로 포기해야 한다.
        // 금지: 공만 날아가고 commit/abandon 없이 팔이 아무 결정도 안 함.
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

                    let arm = test_robot();
                    let center_rail_x = arm.arm.rail.as_ref().expect("리니어").default_x();
                    let center_joints = arm.arm.default_joints.clone();
                    let mut world = SimWorld::new(arm.clone());
                    world.set_use_ground_truth(true);
                    // 격자 코너 샷은 mount 도달 구간(`defaults::intercept` y≤0.18)보다
                    // 앞쪽 평면도 샘플해야 접수/포기가 갈린다.
                    world.set_intercept_window(InterceptWindow {
                        y_min: 0.20,
                        y_max: 0.55,
                        sample_step: 0.05,
                    });
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
                        if world.swing_abandoned() {
                            break;
                        }
                    }

                    if world.swing_abandoned() {
                        assert!(
                            !world.swing_committed() && !racket_contact,
                            "lateral={lateral:+.2} yaw={yaw:+.2} speed={speed:.2} — \
                             포기한 비행은 commit/접수가 없어야 함"
                        );
                        continue;
                    }

                    assert!(
                        racket_contact,
                        "lateral={lateral:+.2} yaw={yaw:+.2} speed={speed:.2} — \
                         로봇이 중앙에서 시작할 때 라켓 접수·포기 둘 다 없음"
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
        //
        // 스핀은 테이블 바운스에 영향을 줘 예측 미스로 이어질 수 있어
        // 이 테스트에서는 조준·높이만 랜덤화한다(GUI 스핀은 유지).
        use rand::SeedableRng;
        let mut rng = rand::rngs::StdRng::seed_from_u64(123);

        let arm = test_robot();
        let mut world = SimWorld::new(arm);
        world.set_use_ground_truth(true);

        // 스핀·높이·pitch/roll 랜덤은 GUI용. 접수 회귀는 기본 자세로
        // 조준(lateral/yaw/speed)만 흔든다.
        let defaults = BallShooterSettings::default();
        for round in 0..15 {
            let settings = defaults.randomized_aim(&mut rng);
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


    /// `random_shot_grid_clears_net_and_returns`는 yaw 코너만 본다. 같은
    /// `defaults::urdf_4dof` 로봇으로 좌우·yaw를 0/25/50/75/100% 촘촘히
    /// 스윕한다 — 코너만 봐서는 못 잡는 실패(중간값에서만 실패)가 실제로
    /// 있었다. 속도 상한도 이 격자에서 맞춰 둔다.
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
                    let robot = fourdof_robot();
                    let mut world = SimWorld::new(robot);
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
        // 돌리는 건 `primitive_4dof()`이 아니라 `defaults::urdf_4dof` 이므로 그걸로 재현한다.
        use rand::SeedableRng;

        for seed in 0..200_u64 {
            let robot = fourdof_robot();
            let mut world = SimWorld::new(robot);
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

            // 2구: Random Shoot (조준만 — 높이/스핀/pitch/roll은 리치 회귀에서 제외).
            let mut rng = rand::rngs::StdRng::seed_from_u64(seed);
            let settings = BallShooterSettings::default().randomized_aim(&mut rng);
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
            let arm = test_robot();
            let mut world = SimWorld::new(arm);
            world.set_use_ground_truth(true);

            let defaults = BallShooterSettings::default();
            let first = defaults.randomized_aim(&mut rng);
            world.shoot_ball(&first);
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
            let second = defaults.randomized_aim(&mut rng);
            world.shoot_ball(&second);
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

    /// Random Shoot yaw 코너 × 속도 코너가 `urdf_4dof`에서 네트·접수·리턴을
    /// 통과하는지 스모크. 촘촘한 스윕은
    /// `random_shot_fine_grid_clears_net_and_returns_for_fourdof_robot`.
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

                    let mut world = SimWorld::new(fourdof_robot());
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

    #[test]
    fn dual_yaw_motor_max_force_is_double_single_in_world() {
        let arm = test_robot();
        let mut world = SimWorld::new(arm);
        world.set_yaw_motor_max_force_for_test(12.0);
        let dual = yaw_motor_max_force(&world);
        world.set_yaw_motor_max_force_for_test(6.0);
        let single = yaw_motor_max_force(&world);
        assert!(
            (dual - 12.0).abs() < 1e-3 && (single - 6.0).abs() < 1e-3,
            "dual={dual} single={single}"
        );
        assert!(dual > single + 1.0);
    }

    fn yaw_motor_max_force(world: &SimWorld) -> f32 {
        let handle = world.arm_bodies.joint_handles[0];
        let (mbodies, link_id) = world.multibody_joint_set.get(handle).expect("joint");
        let link = mbodies.link(link_id).expect("link");
        let revolute = link.joint.data.as_revolute().expect("revolute");
        return revolute.motor().map(|m| m.max_force).unwrap_or(0.0);
    }
}
