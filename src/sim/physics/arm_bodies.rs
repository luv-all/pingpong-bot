//! 시뮬 암 링크 바디 — Rapier multibody + 관절 모터 effort 상한.
//!
//! `SerialChain`과 1:1 프레임 (`mount_rotation` · origin 회전 · 축 · `ee_transform`).
//! EE collider는 FK 면축 리맵(+Y→Rapier +Z)을 포함. 볼 충돌 SSOT.

use nalgebra::{Isometry3, Translation3, Unit, UnitQuaternion, Vector3};
use rapier3d::prelude::*;

use crate::constants::geometry::{RACKET_HALF_X, RACKET_HALF_Y, RACKET_HALF_Z};
use crate::defaults;
use crate::robot::{Arm, Joints};

/// 위치 추종 게인 — τ 여유 있을 때 명령각에 가깝게, 포화 시에만 지연.
const MOTOR_STIFFNESS: f32 = 5_000.0;
const MOTOR_DAMPING: f32 = 200.0;

/// 라켓은 공만, 테이블/네트와는 비충돌 (관절 구속 깨짐 방지).
fn racket_collision_groups() -> InteractionGroups {
    return InteractionGroups::new(Group::GROUP_2, Group::GROUP_1, InteractionTestMode::And);
}

/// 공은 라켓·테이블 등과 충돌.
pub fn ball_collision_groups() -> InteractionGroups {
    return InteractionGroups::new(
        Group::GROUP_1,
        Group::GROUP_2 | Group::GROUP_3,
        InteractionTestMode::And,
    );
}

/// 테이블·네트·슈터.
pub fn static_collision_groups() -> InteractionGroups {
    return InteractionGroups::new(Group::GROUP_3, Group::GROUP_1, InteractionTestMode::And);
}

/// CAD 라켓 +Y 법선 → Rapier 큐브 +Z 법선 (`racket_pose_from_isometry`와 동일).
fn link_from_racket() -> UnitQuaternion<f64> {
    return UnitQuaternion::from_axis_angle(
        &Unit::new_normalize(Vector3::new(0.0, 1.0, 1.0)),
        std::f64::consts::PI,
    );
}

fn na_iso_to_rapier_pose(iso: &Isometry3<f64>) -> Pose {
    let t = iso.translation.vector;
    let q = iso.rotation.quaternion();
    return Pose::from_parts(
        Vec3::new(t.x as f32, t.y as f32, t.z as f32),
        Rotation::from_xyzw(q.i as f32, q.j as f32, q.k as f32, q.w as f32),
    );
}

fn vec3_f32(v: Vector3<f64>) -> Vec3 {
    // IEEE -0.0 이 Rapier revolute 기저 방향을 뒤집어 q=0 자세가 거울상이 된다.
    return Vec3::new((v.x + 0.0) as f32, (v.y + 0.0) as f32, (v.z + 0.0) as f32);
}

/// 다물체 암 핸들.
pub struct ArmMultibody {
    pub link_handles: Vec<RigidBodyHandle>,
    pub joint_handles: Vec<MultibodyJointHandle>,
    pub racket_link_index: usize,
    base_handle: RigidBodyHandle,
    mount_rotation: UnitQuaternion<f64>,
    /// 마지막 링크 기준 EE(+면축) 고정 변환 — FK `ee_transform * link_from_racket`.
    ee_from_link: Isometry3<f64>,
}

impl ArmMultibody {
    /// 마운트에 키네마틱 베이스 + revolute 체인 + EE 라켓 collider.
    pub fn spawn(
        bodies: &mut RigidBodySet,
        colliders: &mut ColliderSet,
        joints: &mut MultibodyJointSet,
        arm: &Arm,
        mount: Vector3<f64>,
        initial: &Joints,
        restitution: f32,
    ) -> Self {
        let torques = defaults::control().max_joint_torques;
        let n = arm.joint_count().min(initial.values.len());
        let mount_iso = arm.chain.mount_isometry(mount);
        let base = bodies.insert(
            RigidBodyBuilder::kinematic_position_based()
                .pose(na_iso_to_rapier_pose(&mount_iso))
                .build(),
        );

        let mut link_handles = Vec::with_capacity(n);
        let mut joint_handles = Vec::with_capacity(n);
        let mut parent = base;
        // q=0 누적: mount * origin0 * R0(0) * origin1 * …
        let mut parent_iso = mount_iso;

        for index in 0..n {
            let joint_def = &arm.chain.joints[index];
            let origin = &joint_def.origin;
            let axis_local = joint_def.axis.into_inner();
            let child_iso = parent_iso * origin;

            let mass = link_mass(index, n);
            let is_ee = index + 1 == n;
            let child = bodies.insert(
                RigidBodyBuilder::dynamic()
                    .pose(na_iso_to_rapier_pose(&child_iso))
                    .additional_mass(mass as f32)
                    .ccd_enabled(is_ee)
                    .build(),
            );

            // parent local: axis after origin rot; child local: joint axis.
            let axis1 = origin.rotation * axis_local;
            let axis2 = axis_local;
            let tau = torques.get(index).copied().unwrap_or(6.0) as f32;
            let mut revolute = RevoluteJointBuilder::new(vec3_f32(axis2))
                .local_anchor1(vec3_f32(origin.translation.vector))
                .local_anchor2(Vec3::ZERO)
                .motor_position(initial.values[index] as f32, MOTOR_STIFFNESS, MOTOR_DAMPING)
                .motor_max_force(tau)
                .build();
            revolute.data.set_local_axis1(vec3_f32(axis1));
            revolute.data.set_local_axis2(vec3_f32(axis2));
            let handle = joints
                .insert(parent, child, revolute, true)
                .expect("multibody joint");
            link_handles.push(child);
            joint_handles.push(handle);
            parent = child;
            parent_iso = child_iso;
        }

        Self::apply_initial_angles(joints, bodies, &joint_handles, initial);

        // 중력은 공·탁구대만 — 링크에 걸리면 모터 추종·FK 정합이 깨진다.
        for &handle in &link_handles {
            if let Some(body) = bodies.get_mut(handle) {
                body.set_gravity_scale(0.0, true);
            }
        }

        let ee_from_link = arm.chain.ee_transform
            * Isometry3::from_parts(Translation3::identity(), link_from_racket());
        let racket_link_index = link_handles.len().saturating_sub(1);
        if let Some(&ee) = link_handles.last() {
            let racket = ColliderBuilder::cuboid(
                RACKET_HALF_X as f32,
                RACKET_HALF_Y as f32,
                RACKET_HALF_Z as f32,
            )
            .position(na_iso_to_rapier_pose(&ee_from_link))
            .collision_groups(racket_collision_groups())
            .restitution(restitution)
            // 공 e(테이블용)보다 작을 때 Average면 중간값이 됨 → Min으로 e_eff 유지.
            .restitution_combine_rule(CoefficientCombineRule::Min)
            .friction(defaults::impact().racket_friction as f32)
            // density>0 이면 COM이 ee_from_link 쪽으로 밀려 body.position()≠링크원점 → FK 불일치.
            .mass(0.0)
            .build();
            colliders.insert_with_parent(racket, ee, bodies);
        }

        return Self {
            link_handles,
            joint_handles,
            racket_link_index,
            base_handle: base,
            mount_rotation: arm.chain.mount_rotation,
            ee_from_link,
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

    /// EE(라켓) 월드 위치 — 링크 포즈 × `ee_from_link` (FK 라켓 중심과 비교용).
    pub fn ee_world_translation(&self, bodies: &RigidBodySet) -> Option<Vector3<f64>> {
        return self
            .ee_world_isometry(bodies)
            .map(|iso| iso.translation.vector);
    }

    /// EE 월드 자세 — Rapier 라켓 계약 (`+Z` = 면 법선). 뷰어·`racket_pose` SSOT.
    pub fn ee_world_isometry(&self, bodies: &RigidBodySet) -> Option<Isometry3<f64>> {
        let handle = self.racket_handle()?;
        let body = bodies.get(handle)?;
        let pos = body.position();
        let r = pos.rotation;
        let link_iso = Isometry3::from_parts(
            Translation3::new(
                pos.translation.x as f64,
                pos.translation.y as f64,
                pos.translation.z as f64,
            ),
            UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
                r.w as f64, r.x as f64, r.y as f64, r.z as f64,
            )),
        );
        return Some(link_iso * self.ee_from_link);
    }

    /// 레일 x에 맞춰 베이스를 옮긴다 (키네마틱, mount_rotation 유지).
    ///
    /// 멀티바디 루트에서는 `set_next_kinematic_*`가 무시되는 경우가 있어
    /// `set_translation`/`set_rotation`으로 직접 맞춘 뒤 FK로 자식을 갱신한다.
    pub fn set_base_xy(
        &self,
        bodies: &mut RigidBodySet,
        joints: &mut MultibodyJointSet,
        x: f64,
        y: f64,
        z: f64,
    ) {
        let q = self.mount_rotation.quaternion();
        if let Some(body) = bodies.get_mut(self.base_handle) {
            body.set_translation(Vec3::new(x as f32, y as f32, z as f32), true);
            body.set_rotation(
                Rotation::from_xyzw(q.i as f32, q.j as f32, q.k as f32, q.w as f32),
                true,
            );
        }
        if let Some(&first) = self.joint_handles.first()
            && let Some((mb, _)) = joints.get_mut(first)
        {
            mb.forward_kinematics(bodies, true);
            mb.update_rigid_bodies(bodies, true);
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

    /// 다물체 관절각 읽기 (폐루프 측정).
    pub fn read_joint_angles(&self, joints: &MultibodyJointSet) -> Joints {
        let mut values = Vec::with_capacity(self.joint_handles.len());
        for &handle in &self.joint_handles {
            let Some((mb, link_id)) = joints.get(handle) else {
                values.push(0.0);
                continue;
            };
            let Some(link) = mb.link(link_id) else {
                values.push(0.0);
                continue;
            };
            values.push(revolute_angle(&link.joint));
        }
        return Joints::from_slice(&values);
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

fn revolute_angle(joint: &MultibodyJoint) -> f64 {
    let locked_bits = joint.data.locked_axes.bits();
    let locked_ang_bits = locked_bits >> 3;
    let dof_id = (!locked_ang_bits).trailing_zeros() as usize;
    let coords = joint.coords();
    return f64::from(coords[3 + dof_id]);
}

fn link_mass(index: usize, count: usize) -> f64 {
    if index + 1 == count {
        return 0.08;
    }
    return 0.04;
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Point3;
    use crate::defaults::primitive_4dof;

    fn sample_arm() -> Arm {
        return (*primitive_4dof().expect("arm").arm).clone();
    }

    fn racket_e() -> f32 {
        return defaults::impact().racket_effective_restitution as f32;
    }

    fn spawn_test_arm(
        attach_check: bool,
    ) -> (
        Arm,
        RigidBodySet,
        ColliderSet,
        MultibodyJointSet,
        ArmMultibody,
    ) {
        let arm = sample_arm();
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
            racket_e(),
        );
        assert!(attach_check);
        return (arm, bodies, colliders, joints, mb);
    }

    #[test]
    fn spawns_four_joints_dual_yaw_torque_from_entry() {
        let (_arm, _b, _c, _j, mb) = spawn_test_arm(true);
        assert_eq!(mb.joint_count(), 4);
        assert_eq!(defaults::control().max_joint_torques[0], 12.0);
        assert_eq!(defaults::control().max_joint_torques[1], 6.0);
    }

    #[test]
    fn dual_yaw_motor_force_exceeds_single() {
        let (_arm, _b, _c, mut joints, mb) = spawn_test_arm(true);
        mb.set_motor_max_forces(&mut joints, &[12.0, 6.0, 6.0, 6.0]);
        let dual = read_yaw_max_force(&joints, mb.joint_handles[0]);
        mb.set_motor_max_forces(&mut joints, &[6.0, 6.0, 6.0, 6.0]);
        let single = read_yaw_max_force(&joints, mb.joint_handles[0]);
        assert!(dual > single + 1.0, "dual={dual} single={single}");
    }

    #[test]
    fn read_joint_angles_match_initial_displacements() {
        let (arm, _b, _c, joints, mb) = spawn_test_arm(true);
        let read = mb.read_joint_angles(&joints);
        for (a, b) in read.values.iter().zip(&arm.default_joints.values) {
            assert!(
                (a - b).abs() < 1e-3,
                "read={:?} default={:?}",
                read.values,
                arm.default_joints.values
            );
        }
    }

    #[test]
    fn ee_matches_fk_after_nonzero_joint_angles() {
        let arm = sample_arm();
        let mut bodies = RigidBodySet::new();
        let mut colliders = ColliderSet::new();
        let mut joints = MultibodyJointSet::new();
        let mount = Point3::from(crate::defaults::rail_frame().mount_xyz0()).coords;
        let mut q = arm.default_joints.clone();
        q.values[1] = 0.3;
        q.values[2] = -0.5;
        let mb = ArmMultibody::spawn(
            &mut bodies,
            &mut colliders,
            &mut joints,
            &arm,
            mount,
            &q,
            racket_e(),
        );
        let fk = arm
            .forward_kinematics_with_rail(mount.x, &q)
            .expect("fk")
            .position
            .coords;
        let ee = mb.ee_world_translation(&bodies).expect("ee");
        let err = (ee - fk).norm();
        assert!(
            err < 0.002,
            "EE↔FK at nonzero q: err={err:.4} ee={ee:?} fk={fk:?}"
        );
    }

    #[test]
    fn ee_matches_fk_during_ramped_targets_with_base_sync() {
        let arm = sample_arm();
        let mut bodies = RigidBodySet::new();
        let mut colliders = ColliderSet::new();
        let mut joints = MultibodyJointSet::new();
        let mut islands = IslandManager::new();
        let mut broad = DefaultBroadPhase::new();
        let mut narrow = NarrowPhase::new();
        let mut impulse = ImpulseJointSet::new();
        let mut ccd = CCDSolver::new();
        let mut pipeline = PhysicsPipeline::new();
        let gravity = Vec3::new(0.0, 0.0, -9.81);
        let mut params = IntegrationParameters::default();
        params.dt = 1.0 / 1000.0;

        let mount = Point3::from(crate::defaults::rail_frame().mount_xyz0()).coords;
        let mb = ArmMultibody::spawn(
            &mut bodies,
            &mut colliders,
            &mut joints,
            &arm,
            mount,
            &arm.default_joints,
            racket_e(),
        );
        let start = arm.default_joints.clone();
        let mut impact = start.clone();
        impact.values[1] += 0.2;
        impact.values[2] -= 0.3;
        let mut max_err = 0.0_f64;
        for step in 0..300 {
            let t = (step as f64) / 300.0;
            let mut target = start.clone();
            for i in 0..target.values.len().min(impact.values.len()) {
                target.values[i] = start.values[i] + t * (impact.values[i] - start.values[i]);
            }
            mb.set_base_xy(&mut bodies, &mut joints, mount.x, mount.y, mount.z);
            mb.set_motor_targets(&mut joints, &target);
            pipeline.step(
                gravity,
                &params,
                &mut islands,
                &mut broad,
                &mut narrow,
                &mut bodies,
                &mut colliders,
                &mut impulse,
                &mut joints,
                &mut ccd,
                &(),
                &(),
            );
            if let Some(&first) = mb.joint_handles.first()
                && let Some((m, _)) = joints.get_mut(first)
            {
                m.forward_kinematics(&mut bodies, true);
                m.update_rigid_bodies(&mut bodies, true);
            }
            let read = mb.read_joint_angles(&joints);
            let fk = arm
                .forward_kinematics_with_rail(mount.x, &read)
                .expect("fk")
                .position
                .coords;
            let ee = mb.ee_world_translation(&bodies).expect("ee");
            max_err = max_err.max((ee - fk).norm());
        }
        assert!(
            max_err < 0.01,
            "ramped+base_sync EE↔FK max_err={max_err:.4}"
        );
    }

    #[test]
    fn ee_matches_fk_after_motor_tracking() {
        let arm = sample_arm();
        let mut bodies = RigidBodySet::new();
        let mut colliders = ColliderSet::new();
        let mut joints = MultibodyJointSet::new();
        let mut islands = IslandManager::new();
        let mut broad = DefaultBroadPhase::new();
        let mut narrow = NarrowPhase::new();
        let mut impulse = ImpulseJointSet::new();
        let mut ccd = CCDSolver::new();
        let mut pipeline = PhysicsPipeline::new();
        let gravity = Vec3::new(0.0, 0.0, -9.81);
        let params = IntegrationParameters::default();

        let mount = Point3::from(crate::defaults::rail_frame().mount_xyz0()).coords;
        let mb = ArmMultibody::spawn(
            &mut bodies,
            &mut colliders,
            &mut joints,
            &arm,
            mount,
            &arm.default_joints,
            racket_e(),
        );
        let mut target = arm.default_joints.clone();
        target.values[1] = 0.35;
        target.values[2] = -0.4;
        mb.set_motor_targets(&mut joints, &target);

        for _ in 0..500 {
            pipeline.step(
                gravity,
                &params,
                &mut islands,
                &mut broad,
                &mut narrow,
                &mut bodies,
                &mut colliders,
                &mut impulse,
                &mut joints,
                &mut ccd,
                &(),
                &(),
            );
        }
        let read = mb.read_joint_angles(&joints);
        let fk = arm
            .forward_kinematics_with_rail(mount.x, &read)
            .expect("fk")
            .position
            .coords;
        let ee = mb.ee_world_translation(&bodies).expect("ee");
        let err = (ee - fk).norm();
        assert!(
            err < 0.005,
            "after motor track EE↔FK(read) err={err:.4}; read={:?} target={:?} ee={ee:?} fk={fk:?}",
            read.values,
            target.values
        );
    }

    #[test]
    fn ee_matches_fk_within_2mm_at_default_pose() {
        let (arm, bodies, _c, _j, mb) = spawn_test_arm(true);
        let mount = Point3::from(crate::defaults::rail_frame().mount_xyz0()).coords;
        let fk = arm
            .forward_kinematics_with_rail(mount.x, &arm.default_joints)
            .expect("fk");
        let ee = mb.ee_world_translation(&bodies).expect("ee");
        let err = (ee - fk.position.coords).norm();
        assert!(
            err < 0.002,
            "EE↔FK error {err:.4} m (limit 2mm); ee={ee:?} fk={:?}",
            fk.position.coords
        );
    }

    #[test]
    fn ee_face_normal_matches_fk_toward_opponent() {
        let (arm, bodies, _c, _j, mb) = spawn_test_arm(true);
        let mount = Point3::from(crate::defaults::rail_frame().mount_xyz0()).coords;
        let fk = arm
            .forward_kinematics_with_rail(mount.x, &arm.default_joints)
            .expect("fk");
        let iso = mb.ee_world_isometry(&bodies).expect("ee");
        // Rapier 라켓 계약: local +Z = 면 법선.
        let face = iso.rotation * Vector3::z();
        assert!(
            face.dot(&fk.normal) > 0.99,
            "EE +Z should match FK normal; face={face:?} fk_n={:?}",
            fk.normal
        );
        assert!(face.y > 0.5, "면이 상대(+Y)를 봐야 함: face={face:?}");
    }

    fn read_yaw_max_force(joints: &MultibodyJointSet, handle: MultibodyJointHandle) -> f32 {
        let (mb, link_id) = joints.get(handle).expect("joint");
        let link = mb.link(link_id).expect("link");
        let revolute = link.joint.data.as_revolute().expect("revolute");
        return revolute.motor().map(|m| m.max_force).unwrap_or(0.0);
    }

    #[test]
    fn urdf_4dof_ee_matches_fk_at_default_and_after_ramp() {
        let robot = crate::defaults::urdf_4dof().expect("4dof");
        let arm = robot.arm.as_ref();
        let mut bodies = RigidBodySet::new();
        let mut colliders = ColliderSet::new();
        let mut joints = MultibodyJointSet::new();
        let mount = Point3::from(crate::defaults::rail_frame().mount_xyz0()).coords;
        let mb = ArmMultibody::spawn(
            &mut bodies,
            &mut colliders,
            &mut joints,
            arm,
            mount,
            &arm.default_joints,
            racket_e(),
        );
        let fk0 = arm
            .forward_kinematics_with_rail(mount.x, &arm.default_joints)
            .expect("fk")
            .position
            .coords;
        let ee0 = mb.ee_world_translation(&bodies).expect("ee");
        let err0 = (ee0 - fk0).norm();
        assert!(
            err0 < 0.002,
            "URDF 4dof rest EE↔FK err={err0:.4} ee={ee0:?} fk={fk0:?}"
        );

        let mut islands = IslandManager::new();
        let mut broad = DefaultBroadPhase::new();
        let mut narrow = NarrowPhase::new();
        let mut impulse = ImpulseJointSet::new();
        let mut ccd = CCDSolver::new();
        let mut pipeline = PhysicsPipeline::new();
        let gravity = Vec3::new(0.0, 0.0, -9.81);
        let mut params = IntegrationParameters::default();
        params.dt = 1.0 / 1000.0;
        let start = arm.default_joints.clone();
        let mut impact = start.clone();
        impact.values[1] += 0.2;
        impact.values[2] -= 0.3;
        let mut max_err = 0.0_f64;
        for step in 0..300 {
            let t = ((step as f64) / 250.0).min(1.0);
            let mut target = start.clone();
            for i in 0..target.values.len().min(impact.values.len()) {
                target.values[i] = start.values[i] + t * (impact.values[i] - start.values[i]);
            }
            let rail_x = 0.05 * t;
            mb.set_base_xy(&mut bodies, &mut joints, rail_x, mount.y, mount.z);
            mb.set_motor_targets(&mut joints, &target);
            pipeline.step(
                gravity,
                &params,
                &mut islands,
                &mut broad,
                &mut narrow,
                &mut bodies,
                &mut colliders,
                &mut impulse,
                &mut joints,
                &mut ccd,
                &(),
                &(),
            );
            let read = mb.read_joint_angles(&joints);
            let fk = arm
                .forward_kinematics_with_rail(rail_x, &read)
                .expect("fk")
                .position
                .coords;
            let ee = mb.ee_world_translation(&bodies).expect("ee");
            max_err = max_err.max((ee - fk).norm());
        }
        assert!(
            max_err < 0.02,
            "URDF 4dof ramp+rail EE↔FK max_err={max_err:.4}"
        );
    }
}
