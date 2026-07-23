//! 시뮬 암 링크 바디 — Rapier multibody + 관절 모터 effort 상한.
//!
//! `motor_max_force` = [`crate::defaults`] `max_joint_torques` (defaults SSOT, yaw 듀얼=12).
//! 볼 충돌체는 기본적으로 붙이지 않는다 — `SimWorld`는 FK 키네마틱 라켓을 쓴다.
//! (`attach_racket_collider=true`는 EE 실험·단위 테스트용.)

use nalgebra::Vector3;
use rapier3d::prelude::*;

use crate::constants::geometry::{RACKET_HALF_X, RACKET_HALF_Y, RACKET_HALF_Z};
use crate::robot::{Arm, Joints};
use crate::defaults;

/// 위치 추종 게인 — τ 여유 있을 때 명령각에 가깝게, 포화 시에만 지연.
const MOTOR_STIFFNESS: f32 = 1200.0;
const MOTOR_DAMPING: f32 = 80.0;

/// 다물체 암 핸들.
pub struct ArmMultibody {
    pub link_handles: Vec<RigidBodyHandle>,
    pub joint_handles: Vec<MultibodyJointHandle>,
    pub racket_link_index: usize,
    base_handle: RigidBodyHandle,
}

impl ArmMultibody {
    /// 마운트에 키네마틱 베이스 + revolute 체인 (+ EE 라켓 collider).
    pub fn spawn(
        bodies: &mut RigidBodySet,
        colliders: &mut ColliderSet,
        joints: &mut MultibodyJointSet,
        arm: &Arm,
        mount: Vector3<f64>,
        initial: &Joints,
        restitution: f32,
        attach_racket_collider: bool,
    ) -> Self {
        let torques = defaults::control().max_joint_torques;
        let n = arm.joint_count().min(initial.values.len());
        let base = bodies.insert(
            RigidBodyBuilder::kinematic_position_based()
                .translation(Vec3::new(mount.x as f32, mount.y as f32, mount.z as f32))
                .build(),
        );

        let mut link_handles = Vec::with_capacity(n);
        let mut joint_handles = Vec::with_capacity(n);
        let mut parent = base;
        let mut cursor = mount;

        for index in 0..n {
            let joint_def = &arm.chain.joints[index];
            let origin = joint_def.origin.translation.vector;
            cursor += origin;
            let axis = joint_def.axis.into_inner();
            let mass = link_mass(index, n);
            let is_ee = index + 1 == n;
            let child = bodies.insert(
                RigidBodyBuilder::dynamic()
                    .translation(Vec3::new(cursor.x as f32, cursor.y as f32, cursor.z as f32))
                    .additional_mass(mass as f32)
                    // 중간 링크 CCD는 비용만 큼 — EE(+공)만으로 충분.
                    .ccd_enabled(is_ee)
                    .build(),
            );
            let tau = torques.get(index).copied().unwrap_or(6.0) as f32;
            let revolute = RevoluteJointBuilder::new(Vec3::new(
                axis.x as f32,
                axis.y as f32,
                axis.z as f32,
            ))
            .local_anchor1(Vec3::new(origin.x as f32, origin.y as f32, origin.z as f32))
            .local_anchor2(Vec3::ZERO)
            .motor_position(initial.values[index] as f32, MOTOR_STIFFNESS, MOTOR_DAMPING)
            .motor_max_force(tau);
            let handle = joints
                .insert(parent, child, revolute, true)
                .expect("multibody joint");
            link_handles.push(child);
            joint_handles.push(handle);
            parent = child;
        }

        // 관절 좌표를 initial로 맞춘다 (spawn 시 각 dof=0이라 default q2≠0이면 EE가 빗나감).
        Self::apply_initial_angles(joints, bodies, &joint_handles, initial);

        let racket_link_index = link_handles.len().saturating_sub(1);
        if attach_racket_collider {
            if let Some(&ee) = link_handles.last() {
                let racket = ColliderBuilder::cuboid(
                    RACKET_HALF_X as f32,
                    RACKET_HALF_Y as f32,
                    RACKET_HALF_Z as f32,
                )
                .restitution(restitution)
                .friction(0.5)
                .density(0.4)
                .build();
                colliders.insert_with_parent(racket, ee, bodies);
            }
        } else {
            let _ = restitution;
        }

        return Self {
            link_handles,
            joint_handles,
            racket_link_index,
            base_handle: base,
        };
    }

    pub fn base_handle(&self) -> RigidBodyHandle {
        return self.base_handle;
    }

    pub fn racket_handle(&self) -> Option<RigidBodyHandle> {
        return self.link_handles.get(self.racket_link_index).copied();
    }

    pub fn joint_count(&self) -> usize {
        return self.joint_handles.len();
    }

    /// 레일 x에 맞춰 베이스를 옮긴다 (키네마틱).
    pub fn set_base_xy(&self, bodies: &mut RigidBodySet, x: f64, y: f64, z: f64) {
        if let Some(body) = bodies.get_mut(self.base_handle) {
            body.set_next_kinematic_translation(Vec3::new(x as f32, y as f32, z as f32));
        }
    }

    /// 목표 관절각으로 모터 위치 제어. effort 상한은 spawn 시 τ_max.
    pub fn set_motor_targets(&self, joints: &mut MultibodyJointSet, targets: &Joints) {
        let n = self.joint_handles.len().min(targets.values.len());
        for i in 0..n {
            let handle = self.joint_handles[i];
            let Some((mb, link_id)) = joints.get_mut(handle) else {
                continue;
            };
            let Some(link) = mb.link_mut(link_id) else {
                continue;
            };
            if let Some(revolute) = link.joint.data.as_revolute_mut() {
                revolute.set_motor_position(
                    targets.values[i] as f32,
                    MOTOR_STIFFNESS,
                    MOTOR_DAMPING,
                );
            }
        }
    }

    /// 관절별 motor_max_force를 덮어쓴다 (듀얼 vs 단일 회귀용).
    pub fn set_motor_max_forces(&self, joints: &mut MultibodyJointSet, torques: &[f64]) {
        let n = self.joint_handles.len().min(torques.len());
        for i in 0..n {
            let handle = self.joint_handles[i];
            let Some((mb, link_id)) = joints.get_mut(handle) else {
                continue;
            };
            let Some(link) = mb.link_mut(link_id) else {
                continue;
            };
            if let Some(revolute) = link.joint.data.as_revolute_mut() {
                revolute.set_motor_max_force(torques[i] as f32);
            }
        }
    }

    fn apply_initial_angles(
        joints: &mut MultibodyJointSet,
        bodies: &mut RigidBodySet,
        joint_handles: &[MultibodyJointHandle],
        initial: &Joints,
    ) {
        let Some(&first) = joint_handles.first() else {
            return;
        };
        let link_ids: Vec<usize> = joint_handles
            .iter()
            .filter_map(|&h| joints.get(h).map(|(_, id)| id))
            .collect();
        let Some((mb, _)) = joints.get_mut(first) else {
            return;
        };
        let mut disp = vec![0.0_f32; mb.ndofs()];
        for (index, &link_id) in link_ids.iter().enumerate() {
            let Some(link) = mb.link(link_id) else {
                continue;
            };
            let aid = link.assembly_id();
            if aid < disp.len() {
                disp[aid] = initial.values.get(index).copied().unwrap_or(0.0) as f32;
            }
        }
        mb.apply_displacements(&disp);
        mb.forward_kinematics(bodies, true);
        mb.update_rigid_bodies(bodies, true);
    }
}

fn link_mass(index: usize, count: usize) -> f64 {
    // 가볍게: τ 예산 안에서 추종 가능 + 적분 비용↓. EE는 약간 더 무겁게(타격 관성).
    if index + 1 == count {
        return 0.08;
    }
    return 0.04;
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::defaults::arm;
    use crate::Point3;

    #[test]
    fn spawns_four_joints_dual_yaw_torque_from_entry() {
        let arm = arm().expect("arm");
        let mut bodies = RigidBodySet::new();
        let mut colliders = ColliderSet::new();
        let mut joints = MultibodyJointSet::new();
        let mb = ArmMultibody::spawn(
            &mut bodies,
            &mut colliders,
            &mut joints,
            &arm,
            Point3::from(crate::defaults::rail_frame().mount_xyz0()).coords,
            &arm.default_joints,
            0.85,
            true,
        );
        assert_eq!(mb.joint_count(), 4);
        assert_eq!(defaults::control().max_joint_torques[0], 12.0);
        assert_eq!(defaults::control().max_joint_torques[1], 6.0);
    }

    #[test]
    fn dual_yaw_motor_force_exceeds_single() {
        let arm = arm().expect("arm");
        let mut bodies = RigidBodySet::new();
        let mut colliders = ColliderSet::new();
        let mut joints = MultibodyJointSet::new();
        let mb = ArmMultibody::spawn(
            &mut bodies,
            &mut colliders,
            &mut joints,
            &arm,
            Point3::from(crate::defaults::rail_frame().mount_xyz0()).coords,
            &arm.default_joints,
            0.85,
            true,
        );
        mb.set_motor_max_forces(&mut joints, &[12.0, 6.0, 6.0, 6.0]);
        let dual = read_yaw_max_force(&joints, mb.joint_handles[0]);
        mb.set_motor_max_forces(&mut joints, &[6.0, 6.0, 6.0, 6.0]);
        let single = read_yaw_max_force(&joints, mb.joint_handles[0]);
        assert!(dual > single + 1.0, "dual={dual} single={single}");
    }

    fn read_yaw_max_force(joints: &MultibodyJointSet, handle: MultibodyJointHandle) -> f32 {
        let (mb, link_id) = joints.get(handle).expect("joint");
        let link = mb.link(link_id).expect("link");
        let revolute = link.joint.data.as_revolute().expect("revolute");
        return revolute.motor().map(|m| m.max_force).unwrap_or(0.0);
    }
}
