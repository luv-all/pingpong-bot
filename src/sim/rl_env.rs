//! Headless residual-torque 강화학습 환경.
//!
//! 기존 플래너가 공까지의 위치 궤적을 만들고 정책은 스윙 재생 중에만 4축
//! signed torque를 보정한다. 물리는 1 kHz, 정책은 기본 100 Hz로 동작한다.

use rapier3d::prelude::{ColliderHandle, RigidBodyHandle};
use serde::{Deserialize, Serialize};

use crate::constants::{BALL_RADIUS, table};
use crate::robot::Robot;
use crate::sim::{BallShooterSettings, BallState, SimWorld};

const PHYSICS_DT: f64 = 1.0 / 1000.0;
const DEFAULT_ACTION_REPEAT: usize = 10;
const MAX_CONTROL_STEPS: usize = 450;

/// 정책에 전달하는 관측. 모든 길이는 로봇 관절 수를 제외하면 고정이다.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TorqueObservation {
    pub ball_position: [f64; 3],
    pub ball_velocity: [f64; 3],
    pub joint_position: Vec<f64>,
    pub joint_velocity: Vec<f64>,
    /// 기존 플래너/궤적이 현재 시각에 요구하는 관절 위치.
    pub joint_target: Vec<f64>,
    pub racket_position: [f64; 3],
    pub racket_velocity: [f64; 3],
    pub time_to_impact_secs: f64,
    pub policy_active: f64,
    pub contact: f64,
}

impl TorqueObservation {
    /// Python/Gym에서 바로 쓸 1차원 벡터.
    pub fn flattened(&self) -> Vec<f64> {
        let mut values = Vec::with_capacity(17 + self.joint_position.len() * 3);
        values.extend(self.ball_position);
        values.extend(self.ball_velocity);
        values.extend(self.joint_position.iter().copied());
        values.extend(self.joint_velocity.iter().copied());
        values.extend(self.joint_target.iter().copied());
        values.extend(self.racket_position);
        values.extend(self.racket_velocity);
        values.push(self.time_to_impact_secs);
        values.push(self.policy_active);
        values.push(self.contact);
        return values;
    }
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct TorqueEpisodeInfo {
    pub incoming_valid: bool,
    pub committed: bool,
    pub contact: bool,
    pub returned: bool,
    pub cleared_net: bool,
    pub returned_in: bool,
    pub bounced_own_half: bool,
    pub net_fault: bool,
    pub peak_outgoing_y_mps: f64,
    pub ball_velocity_before_contact: [f64; 3],
    pub ball_velocity_after_contact: [f64; 3],
    pub bounce_xy: Option<[f64; 2]>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TorqueStep {
    pub observation: TorqueObservation,
    pub reward: f64,
    pub terminated: bool,
    pub truncated: bool,
    pub info: TorqueEpisodeInfo,
}

/// 한 인스턴스가 한 Rapier 월드를 소유하는 episodic 환경.
pub struct TorqueResidualEnv {
    robot: Robot,
    world: SimWorld,
    ball_collider: ColliderHandle,
    racket_collider: ColliderHandle,
    table_collider: ColliderHandle,
    previous_ball_y: f32,
    previous_ball_velocity: [f64; 3],
    previous_joint_position: Vec<f64>,
    previous_racket_position: [f64; 3],
    previous_action: Vec<f64>,
    incoming_crossed_net: bool,
    contact_active: bool,
    control_steps: usize,
    done: bool,
    info: TorqueEpisodeInfo,
}

impl TorqueResidualEnv {
    pub fn new(robot: Robot) -> Self {
        let world = SimWorld::new(robot.clone());
        let (ball_collider, racket_collider, table_collider) = colliders(&world);
        let joint_position = world.robot().joints().values.clone();
        let action_size = world.arm().joint_count();
        let racket_position = position_array(world.racket_pose().0);
        return Self {
            robot,
            world,
            ball_collider,
            racket_collider,
            table_collider,
            previous_ball_y: 0.0,
            previous_ball_velocity: [0.0; 3],
            previous_joint_position: joint_position,
            previous_racket_position: racket_position,
            previous_action: vec![0.0; action_size],
            incoming_crossed_net: false,
            contact_active: false,
            control_steps: 0,
            done: true,
            info: TorqueEpisodeInfo::default(),
        };
    }

    pub fn action_size(&self) -> usize {
        return self.world.arm().joint_count();
    }

    pub fn observation_size(&self) -> usize {
        return self.observe().flattened().len();
    }

    pub fn reset(&mut self, settings: &BallShooterSettings) -> TorqueObservation {
        self.world = SimWorld::new(self.robot.clone());
        self.world.set_use_ground_truth(true);
        (self.ball_collider, self.racket_collider, self.table_collider) =
            colliders(&self.world);
        self.world.shoot_ball(settings);
        self.previous_ball_y = self.world.ball_position().y;
        self.previous_ball_velocity = velocity_array(self.world.ball_velocity());
        self.previous_joint_position = self.world.robot().joints().values.clone();
        self.previous_racket_position = position_array(self.world.racket_pose().0);
        self.previous_action.fill(0.0);
        self.incoming_crossed_net = false;
        self.contact_active = false;
        self.control_steps = 0;
        self.done = false;
        self.info = TorqueEpisodeInfo::default();
        // 발사 직후부터 commit 전까지는 정책 액션이 의도적으로 무시된다.
        // 이 긴 구간을 replay buffer에 넣으면 SAC 샘플 대부분이
        // `(action 무관, reward=0)`이 되므로, 환경 내부에서 0토크로 건너뛰고
        // 실제로 힘을 배분할 수 있는 스윙 창부터 에피소드를 노출한다.
        let zeros = vec![0.0; self.action_size()];
        let mut observation = self.observe();
        for _ in 0..350 {
            let warmup = self.step_default(&zeros);
            observation = warmup.observation;
            if observation.policy_active > 0.5 || warmup.terminated || warmup.truncated {
                break;
            }
        }
        self.control_steps = 0;
        return observation;
    }

    /// 정규화된 관절 토크 액션을 적용하고 물리를 `action_repeat`회 적분한다.
    pub fn step(&mut self, action: &[f64], action_repeat: usize) -> TorqueStep {
        if self.done {
            return TorqueStep {
                observation: self.observe(),
                reward: 0.0,
                terminated: true,
                truncated: false,
                info: self.info.clone(),
            };
        }

        let repeat = action_repeat.clamp(1, 50);
        let policy_active = self.world.robot().is_swinging() && !self.info.contact;
        if self.info.contact {
            self.world.clear_torque_residual();
        } else {
            self.world.set_normalized_torque_residual(action);
        }
        let action_cost: f64 = action
            .iter()
            .map(|value| {
                let finite = if value.is_finite() { *value } else { 0.0 };
                finite.clamp(-1.0, 1.0).powi(2)
            })
            .sum();
        let action_delta_cost: f64 = (0..self.action_size())
            .map(|index| {
                let current = normalized_action(action, index);
                let previous = self.previous_action.get(index).copied().unwrap_or(0.0);
                (current - previous).powi(2)
            })
            .sum();
        let mut reward = if policy_active {
            -0.0005 * action_cost - 0.002 * action_delta_cost
        } else {
            0.0
        };
        for index in 0..self.previous_action.len() {
            self.previous_action[index] = normalized_action(action, index);
        }
        let mut terminated = false;
        let mut executed_substeps = 0usize;

        // 접촉 전에는 action_repeat만 전진한다. 이 액션 구간에서 접촉하면
        // residual을 끄고 착지까지 내부 진행해 결과 보상을 해당 액션에 직접
        // 귀속시킨다. 접촉 뒤 무의미한 policy step도 replay에 쌓이지 않는다.
        for substep in 0..2_500 {
            self.world.step(PHYSICS_DT, None);
            executed_substeps += 1;
            self.info.committed |= self.world.swing_committed();

            let position = self.world.ball_position();
            let velocity = self.world.ball_velocity();
            let net_y = (table::LENGTH_Y * 0.5) as f32;
            let net_top_z = (table::SURFACE_Z + table::NET_HEIGHT + BALL_RADIUS) as f32;
            let on_table = self
                .world
                .narrow_phase
                .contact_pair(self.ball_collider, self.table_collider)
                .is_some_and(|pair| pair.has_any_active_contact());

            if !self.info.contact {
                if self.previous_ball_y > net_y && position.y <= net_y {
                    self.incoming_crossed_net = position.z > net_top_z;
                }
                if self.incoming_crossed_net
                    && position.y > 0.0
                    && position.y < net_y
                    && on_table
                {
                    self.info.incoming_valid = true;
                }
            }

            let touching_racket = self
                .world
                .narrow_phase
                .contact_pair(self.ball_collider, self.racket_collider)
                .is_some_and(|pair| pair.has_any_active_contact());
            if touching_racket && !self.contact_active {
                if !self.info.contact {
                    self.info.ball_velocity_before_contact = self.previous_ball_velocity;
                    reward += 2.0;
                }
                self.info.contact = true;
                self.world.clear_torque_residual();
            }
            self.contact_active = touching_racket;

            if self.info.contact && velocity.y > 0.0 {
                let first_return = !self.info.returned;
                self.info.returned = true;
                self.info.peak_outgoing_y_mps =
                    self.info.peak_outgoing_y_mps.max(f64::from(velocity.y));
                self.info.ball_velocity_after_contact = velocity_array(velocity);
                if first_return {
                    // 공이 실제로 얻은 상대 방향 속도가 힘 전달의 직접 보상.
                    reward += 3.0 + (f64::from(velocity.y) / 3.0).clamp(0.0, 2.0);
                }
            }

            if self.info.returned && self.previous_ball_y < net_y && position.y >= net_y {
                self.info.cleared_net = position.z > net_top_z;
                if self.info.cleared_net {
                    reward += 4.0;
                } else {
                    self.info.net_fault = true;
                    reward -= 5.0;
                    terminated = true;
                }
            }

            if self.info.contact && on_table && position.y < net_y {
                self.info.bounced_own_half = true;
                reward -= 8.0;
                terminated = true;
            } else if self.info.cleared_net && on_table && position.y >= net_y {
                let x = f64::from(position.x);
                let y = f64::from(position.y);
                self.info.bounce_xy = Some([x, y]);
                self.info.returned_in = y < table::LENGTH_Y;
                if self.info.returned_in {
                    let target_x = table::WIDTH_X * 0.5;
                    let target_y = table::LENGTH_Y * 0.75;
                    let error = (x - target_x).hypot(y - target_y);
                    reward += 10.0 - 3.0 * error;
                } else {
                    reward -= 4.0;
                }
                terminated = true;
            }

            self.previous_ball_y = position.y;
            self.previous_ball_velocity = velocity_array(velocity);
            if terminated || self.world.ball_state == BallState::Parked {
                terminated = true;
                break;
            }
            if substep + 1 >= repeat && !self.info.contact {
                break;
            }
        }

        self.control_steps += 1;
        let truncated = self.control_steps >= MAX_CONTROL_STEPS;
        if (terminated || truncated) && !self.info.contact {
            reward -= 4.0;
        } else if (terminated || truncated) && !self.info.returned {
            reward -= 3.0;
        }
        self.done = terminated || truncated;
        if self.done {
            self.world.clear_torque_residual();
        }

        let observation =
            self.observe_and_advance_history(executed_substeps as f64 * PHYSICS_DT);
        return TorqueStep {
            observation,
            reward,
            terminated,
            truncated,
            info: self.info.clone(),
        };
    }

    pub fn step_default(&mut self, action: &[f64]) -> TorqueStep {
        return self.step(action, DEFAULT_ACTION_REPEAT);
    }

    pub fn world(&self) -> &SimWorld {
        return &self.world;
    }

    fn observe(&self) -> TorqueObservation {
        let ball_position = position_array(self.world.ball_position());
        let ball_velocity = velocity_array(self.world.ball_velocity());
        let joint_position = self.world.robot().joints().values.clone();
        let joint_velocity = vec![0.0; joint_position.len()];
        let joint_target = self.world.robot().targets().values.clone();
        let racket_position = position_array(self.world.racket_pose().0);
        let time_to_impact_secs = self
            .world
            .debug_prediction()
            .map_or(1.0, |prediction| prediction.time_to_impact_secs)
            .clamp(0.0, 1.0);
        return TorqueObservation {
            ball_position,
            ball_velocity,
            joint_position,
            joint_velocity,
            joint_target,
            racket_position,
            racket_velocity: [0.0; 3],
            time_to_impact_secs,
            policy_active: if self.world.robot().is_swinging()
                && self.world.ball_state == BallState::InFlight
                && !self.info.contact
            {
                1.0
            } else {
                0.0
            },
            contact: if self.info.contact { 1.0 } else { 0.0 },
        };
    }

    fn observe_and_advance_history(&mut self, dt: f64) -> TorqueObservation {
        let mut observation = self.observe();
        if dt > f64::EPSILON {
            observation.joint_velocity = observation
                .joint_position
                .iter()
                .zip(self.previous_joint_position.iter())
                .map(|(current, previous)| (current - previous) / dt)
                .collect();
            for axis in 0..3 {
                observation.racket_velocity[axis] =
                    (observation.racket_position[axis] - self.previous_racket_position[axis]) / dt;
            }
        }
        self.previous_joint_position = observation.joint_position.clone();
        self.previous_racket_position = observation.racket_position;
        return observation;
    }
}

fn colliders(world: &SimWorld) -> (ColliderHandle, ColliderHandle, ColliderHandle) {
    let ball = collider_for_parent(world, world.ball_handle);
    let racket = collider_for_parent(world, world.racket_handle);
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
    return (ball, racket, table_collider);
}

fn collider_for_parent(world: &SimWorld, parent: RigidBodyHandle) -> ColliderHandle {
    return world
        .collider_set
        .iter()
        .find_map(|(handle, collider)| (collider.parent() == Some(parent)).then_some(handle))
        .expect("parent collider");
}

fn position_array(value: rapier3d::prelude::Vector) -> [f64; 3] {
    return [f64::from(value.x), f64::from(value.y), f64::from(value.z)];
}

fn velocity_array(value: rapier3d::prelude::Vector) -> [f64; 3] {
    return [f64::from(value.x), f64::from(value.y), f64::from(value.z)];
}

fn normalized_action(action: &[f64], index: usize) -> f64 {
    let value = action.get(index).copied().unwrap_or(0.0);
    return if value.is_finite() {
        value.clamp(-1.0, 1.0)
    } else {
        0.0
    };
}

#[cfg(test)]
mod tests {
    use super::*;

    fn test_env() -> TorqueResidualEnv {
        return TorqueResidualEnv::new(crate::defaults::primitive_4dof().expect("robot"));
    }

    #[test]
    fn reset_and_step_shapes_are_stable() {
        let mut env = test_env();
        let observation = env.reset(&BallShooterSettings::default());
        assert_eq!(env.action_size(), 4);
        assert_eq!(observation.joint_position.len(), 4);
        assert_eq!(observation.flattened().len(), env.observation_size());
        let step = env.step_default(&[0.0; 4]);
        assert_eq!(step.observation.joint_velocity.len(), 4);
        assert!(step.reward.is_finite());
    }

    #[test]
    fn torque_action_is_bounded_by_joint_limits() {
        let mut env = test_env();
        env.reset(&BallShooterSettings::default());
        // 스윙 전 액션은 무시된다.
        env.world.set_normalized_torque_residual(&[99.0, -99.0, f64::NAN, 0.5]);
        assert!(env.world.motor_torque_residual().iter().all(|value| *value == 0.0));
    }
}
