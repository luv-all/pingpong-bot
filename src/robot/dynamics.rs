//! 직렬 매니퓰레이터 역동역학 (RNEA).
//!
//! \(\tau = M(q)\ddot q + C(q,\dot q)\dot q + g(q)\). URDF 합산 관성이 있는 `Arm`만.

use nalgebra::{Matrix3, Vector3};

use crate::constants::physics::G;
use crate::robot::Arm;

/// 링크 프레임(revolute child)에서의 질량 분포. COM·관성은 그 프레임 기준.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LinkInertia {
    pub mass: f64,
    /// 링크 원점 → COM [m]
    pub com: Vector3<f64>,
    /// COM에서 링크 축 정렬 관성 [kg·m²]
    pub inertia_com: Matrix3<f64>,
}

impl LinkInertia {
    /// `other`를 `other_in_self` (other → self)로 옮겨 합친다.
    pub fn combine(&self, other: &Self, other_in_self: &nalgebra::Isometry3<f64>) -> Self {
        let r = other_in_self.rotation.to_rotation_matrix();
        let other_com_in_self = other_in_self * other.com;
        let mass = self.mass + other.mass;
        let com = if mass <= f64::EPSILON {
            Vector3::zeros()
        } else {
            (self.mass * self.com + other.mass * other_com_in_self) / mass
        };
        let i_other_at_ocom = r * other.inertia_com * r.transpose();
        let d_self = self.com - com;
        let d_other = other_com_in_self - com;
        let inertia_com = self.inertia_com
            + self.mass * steiner(d_self)
            + i_other_at_ocom
            + other.mass * steiner(d_other);
        return Self {
            mass,
            com,
            inertia_com,
        };
    }
}

fn steiner(d: Vector3<f64>) -> Matrix3<f64> {
    let d2 = d.norm_squared();
    return Matrix3::identity() * d2 - d * d.transpose();
}

/// \(q,\dot q,\ddot q\)에서 관절 토크 [N·m]. 관성 없거나 길이 불일치면 `None`.
pub fn required_torque(arm: &Arm, q: &[f64], qd: &[f64], qdd: &[f64]) -> Option<Vec<f64>> {
    let inertias = arm.inertias.as_ref()?;
    let n = arm.joint_count();
    if q.len() != n || qd.len() != n || qdd.len() != n || inertias.len() != n {
        return None;
    }
    let mount = arm.base.coords;
    let (_, frames) = arm.chain.forward_with_joint_frames(mount, q)?;
    let link_poses = link_world_poses(arm, mount, q)?;

    let mut omega = vec![Vector3::zeros(); n];
    let mut alpha = vec![Vector3::zeros(); n];
    let mut a_com = vec![Vector3::zeros(); n];
    let mut a_origin = vec![Vector3::zeros(); n];

    for i in 0..n {
        let axis = frames[i].1;
        let joint_pos = frames[i].0;
        let (omega_prev, alpha_prev, a_prev, p_prev) = if i == 0 {
            (Vector3::zeros(), Vector3::zeros(), -G, mount)
        } else {
            (
                omega[i - 1],
                alpha[i - 1],
                a_origin[i - 1],
                link_poses[i - 1].translation.vector,
            )
        };
        omega[i] = omega_prev + qd[i] * axis;
        alpha[i] = alpha_prev + qdd[i] * axis + omega_prev.cross(&(qd[i] * axis));

        let link_origin = link_poses[i].translation.vector;
        let r_joint = joint_pos - p_prev;
        let a_joint =
            a_prev + alpha_prev.cross(&r_joint) + omega_prev.cross(&omega_prev.cross(&r_joint));
        let r_lo = link_origin - joint_pos;
        a_origin[i] =
            a_joint + alpha[i].cross(&r_lo) + omega[i].cross(&omega[i].cross(&r_lo));
        let c = link_poses[i] * inertias[i].com - link_origin;
        a_com[i] = a_origin[i] + alpha[i].cross(&c) + omega[i].cross(&omega[i].cross(&c));
    }

    let mut f_child = Vector3::zeros();
    let mut n_child = Vector3::zeros();
    let mut child_origin = arm
        .chain
        .forward_with_joint_frames(mount, q)?
        .0
        .translation
        .vector;
    let mut tau = vec![0.0; n];
    for i in (0..n).rev() {
        let link_origin = link_poses[i].translation.vector;
        let r = link_poses[i].rotation.to_rotation_matrix();
        let i_world = r * inertias[i].inertia_com * r.transpose();
        let com_w = link_poses[i] * inertias[i].com;
        let f_net = inertias[i].mass * a_com[i];
        let n_spin = i_world * alpha[i] + omega[i].cross(&(i_world * omega[i]));
        let f_i = f_net + f_child;
        let n_i = n_spin
            + (com_w - link_origin).cross(&f_net)
            + n_child
            + (child_origin - link_origin).cross(&f_child);
        tau[i] = n_i.dot(&frames[i].1);
        f_child = f_i;
        n_child = n_i;
        child_origin = link_origin;
    }
    return Some(tau);
}

fn link_world_poses(
    arm: &Arm,
    mount: Vector3<f64>,
    q: &[f64],
) -> Option<Vec<nalgebra::Isometry3<f64>>> {
    if q.len() != arm.chain.joints.len() {
        return None;
    }
    let mut transform = arm.chain.mount_isometry(mount);
    let mut poses = Vec::with_capacity(q.len());
    for (joint, &angle) in arm.chain.joints.iter().zip(q) {
        transform *= joint.origin;
        transform *= nalgebra::Isometry3::from_parts(
            nalgebra::Translation3::identity(),
            nalgebra::UnitQuaternion::from_axis_angle(&joint.axis, angle),
        );
        poses.push(transform);
    }
    return Some(poses);
}

/// \(|\tau_i| \le limits[i]` (limits 짧으면 마지막 값 반복).
pub fn is_feasible(tau: &[f64], limits: &[f64]) -> bool {
    if limits.is_empty() {
        return false;
    }
    return tau.iter().enumerate().all(|(i, &t)| {
        let lim = limits.get(i).copied().unwrap_or(*limits.last().unwrap());
        return t.abs() <= lim + 1e-9;
    });
}

#[cfg(test)]
mod tests {
    use super::*;

    fn urdf_arm() -> Arm {
        return (*crate::defaults::urdf_4dof().expect("urdf").arm).clone();
    }

    #[test]
    fn gravity_only_torques_are_finite_at_default() {
        let arm = urdf_arm();
        let q = arm.default_joints.values.clone();
        let zd = vec![0.0; q.len()];
        let tau = required_torque(&arm, &q, &zd, &zd).expect("tau");
        assert_eq!(tau.len(), 4);
        assert!(tau.iter().all(|t| t.is_finite()));
        let norm: f64 = tau.iter().map(|t| t * t).sum::<f64>().sqrt();
        assert!(norm > 1e-4, "expected gravity torque, got {tau:?}");
    }

    #[test]
    fn mass_matrix_column_matches_rnea_finite_difference() {
        let arm = urdf_arm();
        let q = vec![0.1, 0.4, -0.2, 0.15];
        let zero = vec![0.0; 4];
        let tau0 = required_torque(&arm, &q, &zero, &zero).expect("g");
        let eps = 1e-3;
        for col in 0..4 {
            let mut qdd = zero.clone();
            qdd[col] = eps;
            let tau = required_torque(&arm, &q, &zero, &qdd).expect("tau");
            let m_ii = (tau[col] - tau0[col]) / eps;
            assert!(m_ii > 0.0, "M[{col},{col}] should be > 0, got {m_ii}");
        }
    }

    #[test]
    fn is_feasible_respects_limits() {
        assert!(is_feasible(&[1.0, 2.0], &[3.0, 3.0]));
        assert!(!is_feasible(&[1.0, 4.0], &[3.0, 3.0]));
    }

    #[test]
    fn urdf_arm_has_inertias() {
        let arm = urdf_arm();
        assert_eq!(arm.inertias.as_ref().map(|v| v.len()), Some(4));
    }
}
