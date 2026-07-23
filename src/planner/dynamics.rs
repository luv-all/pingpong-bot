//! 재귀 Newton-Euler 역동역학 (inverse dynamics).
//!
//! 직렬 revolute 체인에서 주어진 관절 상태(각·각속도·각가속도)를 실현하는 데
//! 필요한 관절 토크 [N*m]를 계산한다. 표준 재귀 Newton-Euler 공식
//! (Craig, "Introduction to Robotics: Mechanics and Control")을 월드 좌표계에서
//! 직접 구현한다.
//!
//! 각 관절이 움직이는 강체 관성은 `Arm::aggregated_inertials`를 쓴다 —
//! actuated child link 하나가 아니라 그 뒤 fixed joint로 붙은 하위 링크(모터
//! 몸체, 브래킷, 패들 등)까지 평행축 정리로 합친 값이라야 실제 부하를 반영한다.
//!
//! 중력은 base 링크에 `-G`의 가상 선가속도를 주는 표준 트릭으로 접는다:
//! 그러면 각 링크 질량중심 가속도에 중력 반작용이 포함돼 별도 중력 항 없이
//! 정적 중력 토크가 자연히 나온다.

use nalgebra::{DMatrix, DVector, Isometry3, Matrix3, Translation3, UnitQuaternion, Vector3};

use crate::constants::physics::G;
use crate::robot::{Arm, Joints};

/// Newton-Euler 재귀에 필요한 per-joint 스크래치 버퍼.
///
/// 궤적을 여러 시점에서 반복 평가할 때(토크 이용률 샘플링) 매 호출마다 힙
/// 할당하지 않도록 버퍼를 한 번 만들어 재사용한다. `resize`로 관절 개수에 맞춘다.
#[derive(Debug, Default, Clone)]
pub struct RneaScratch {
    origin: Vec<Vector3<f64>>,        // 관절 축 원점 (월드)
    axis: Vec<Vector3<f64>>,          // 관절 축 단위벡터 (월드)
    com_world: Vec<Vector3<f64>>,     // 합성 강체 질량중심 (월드)
    inertia_world: Vec<Matrix3<f64>>, // 질량중심 기준 관성 (월드축)
    force: Vec<Vector3<f64>>,         // F_i = m a_c
    moment: Vec<Vector3<f64>>,        // N_i (질량중심 기준)
}

impl RneaScratch {
    pub fn new() -> Self {
        return Self::default();
    }

    fn resize(&mut self, n: usize) {
        self.origin.resize(n, Vector3::zeros());
        self.axis.resize(n, Vector3::zeros());
        self.com_world.resize(n, Vector3::zeros());
        self.inertia_world.resize(n, Matrix3::zeros());
        self.force.resize(n, Vector3::zeros());
        self.moment.resize(n, Vector3::zeros());
    }
}

/// 주어진 관절 상태를 실현하는 데 필요한 관절 토크 [N*m]를 반환한다.
///
/// `joint_velocities`/`joint_accelerations`는 관절 각속도 [rad/s]·각가속도
/// [rad/s^2]로 `joints`와 같은 길이여야 한다. 길이가 안 맞거나 체인이 비면
/// `joints` 길이만큼의 0 벡터를 반환한다(호출부의 방어적 폴백).
///
/// 반환값 부호는 관절 축 방향 기준(모터가 내야 하는 토크). 한계 판정에는
/// 절댓값을 쓴다.
pub fn required_joint_torques(
    arm: &Arm,
    joints: &Joints,
    joint_velocities: &[f64],
    joint_accelerations: &[f64],
) -> Vec<f64> {
    let mut scratch = RneaScratch::new();
    let mut torque = vec![0.0; joints.values.len()];
    required_joint_torques_into(
        arm,
        joints,
        joint_velocities,
        joint_accelerations,
        &mut scratch,
        &mut torque,
    );
    return torque;
}

/// [`required_joint_torques`]의 버퍼 재사용 버전. `scratch`와 `torque` 버퍼를
/// 호출부가 소유해 반복 호출 시 힙 할당을 피한다. `torque`는 `joints`와 같은
/// 길이로 맞춰지고 결과로 덮어써진다.
pub fn required_joint_torques_into(
    arm: &Arm,
    joints: &Joints,
    joint_velocities: &[f64],
    joint_accelerations: &[f64],
    scratch: &mut RneaScratch,
    torque: &mut Vec<f64>,
) {
    let chain = &arm.chain;
    let n = chain.joints.len();
    torque.clear();
    torque.resize(joints.values.len(), 0.0);
    if joints.values.len() != n
        || joint_velocities.len() != n
        || joint_accelerations.len() != n
        || arm.aggregated_inertials.len() != n
    {
        return;
    }
    scratch.resize(n);

    // --- 전방 패스: 관절 프레임(월드)과 링크 질량중심 위치/관성 ---
    // 마운트 병진은 결과에 영향 없다(중력 균일, 동역학은 병진 불변). 마운트
    // 회전만 arm→world 방향으로 반영한다.
    let mut transform = chain.mount_isometry(arm.base.v);
    for i in 0..n {
        let joint = &chain.joints[i];
        transform *= joint.origin;
        scratch.origin[i] = transform.translation.vector;
        scratch.axis[i] = (transform.rotation * joint.axis.into_inner()).normalize();
        // 관절 회전 적용 → 링크 프레임(child link 로컬 좌표계).
        transform *= Isometry3::from_parts(
            Translation3::identity(),
            UnitQuaternion::from_axis_angle(&joint.axis, joints.values[i]),
        );
        let rotation = transform.rotation.to_rotation_matrix();
        let body = &arm.aggregated_inertials[i];
        scratch.com_world[i] = transform.translation.vector + rotation * body.com.v;
        scratch.inertia_world[i] = rotation * body.inertia * rotation.transpose();
    }

    // --- 전방 패스: 각속도/각가속도/질량중심 선가속도 → 관성력/관성모멘트 ---
    let mut omega = Vector3::zeros();
    let mut omega_dot = Vector3::zeros();
    // base 링크에 -G 선가속도를 줘 중력을 접는다.
    let mut accel_origin = -G;
    let mut prev_origin = scratch.origin[0];

    for i in 0..n {
        let r = scratch.origin[i] - prev_origin;
        accel_origin = accel_origin + omega_dot.cross(&r) + omega.cross(&omega.cross(&r));

        let axis_rate = scratch.axis[i] * joint_velocities[i];
        let axis_accel = scratch.axis[i] * joint_accelerations[i];
        omega_dot = omega_dot + axis_accel + omega.cross(&axis_rate);
        omega += axis_rate;

        let rc = scratch.com_world[i] - scratch.origin[i];
        let accel_com = accel_origin + omega_dot.cross(&rc) + omega.cross(&omega.cross(&rc));

        let mass = arm.aggregated_inertials[i].mass;
        scratch.force[i] = mass * accel_com;
        let iw = scratch.inertia_world[i];
        scratch.moment[i] = iw * omega_dot + omega.cross(&(iw * omega));

        prev_origin = scratch.origin[i];
    }

    // --- 후방 패스: 말단 → base 로 힘/모멘트 전파, 축 투영으로 관절 토크 ---
    let mut force_next = Vector3::zeros();
    let mut moment_next = Vector3::zeros();
    let mut origin_next: Option<Vector3<f64>> = None;

    for i in (0..n).rev() {
        let force_i = force_next + scratch.force[i];
        let mut moment_i = moment_next
            + scratch.moment[i]
            + (scratch.com_world[i] - scratch.origin[i]).cross(&scratch.force[i]);
        if let Some(origin_next) = origin_next {
            moment_i += (origin_next - scratch.origin[i]).cross(&force_next);
        }
        torque[i] = moment_i.dot(&scratch.axis[i]);

        force_next = force_i;
        moment_next = moment_i;
        origin_next = Some(scratch.origin[i]);
    }
}

/// 중력·코리올리·원심력 항만 (가속 0에서의 필요 토크) [N*m].
///
/// `forward_dynamics`가 `required_joint_torques`를 뒤집는 데 쓴다.
pub fn bias_torques(arm: &Arm, joints: &Joints, joint_velocities: &[f64]) -> Vec<f64> {
    let n = joints.values.len();
    return required_joint_torques(arm, joints, joint_velocities, &vec![0.0; n]);
}

/// 관성 행렬 M(q) [N*m / (rad/s^2)], 관절 `n x n`.
///
/// RNEA를 질량 행렬 계산에 재사용하는 표준 트릭: 속도 0에서 단위 가속도
/// 열벡터마다 RNEA를 한 번씩 돌려 중력 성분을 빼면 그 열이 나온다
/// (`tau(q,0,e_j) - tau(q,0,0) = M(q) e_j = M(:,j)`). 관절 n개면 RNEA n+1회.
pub fn mass_matrix(arm: &Arm, joints: &Joints) -> DMatrix<f64> {
    let n = joints.values.len();
    let zero = vec![0.0; n];
    let bias = required_joint_torques(arm, joints, &zero, &zero);
    let mut m = DMatrix::zeros(n, n);
    for j in 0..n {
        let mut unit_accel = zero.clone();
        unit_accel[j] = 1.0;
        let tau = required_joint_torques(arm, joints, &zero, &unit_accel);
        for i in 0..n {
            m[(i, j)] = tau[i] - bias[i];
        }
    }
    return m;
}

/// 정방향 동역학: 명령 토크로 실제 나오는 관절 각가속도 [rad/s^2].
///
/// `q̈ = M(q)^-1 (τ - bias(q,q̇))`. `M(q)`가 특이(자유도 손실 등)면 `None`.
/// quintic 같은 사전 정의 궤적 모양 없이, 토크 명령을 그대로 강체 동역학에
/// 적분해 "순수하게 토크로 낼 수 있는 움직임"을 시뮬레이션할 때 쓴다
/// (`tools/swing_bench` 참고).
pub fn forward_dynamics(
    arm: &Arm,
    joints: &Joints,
    joint_velocities: &[f64],
    joint_torques: &[f64],
) -> Option<Vec<f64>> {
    let n = joints.values.len();
    if joint_velocities.len() != n || joint_torques.len() != n {
        return None;
    }
    let bias = bias_torques(arm, joints, joint_velocities);
    let m = mass_matrix(arm, joints);
    let rhs = DVector::from_iterator(n, (0..n).map(|i| joint_torques[i] - bias[i]));
    let accel = m.lu().solve(&rhs)?;
    return Some(accel.iter().copied().collect());
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Point3;
    use crate::robot::{Arm, JointLimit, Joints, LinkInertial, SerialChain, SerialJoint};

    const G_MAG: f64 = 9.81;

    /// 단일 관절 테스트용 arm을 만든다. 축은 월드 x, 강체는 축에서 y로 `d`,
    /// 질량 `mass`인 점질량 근사.
    fn single_link_arm(mass: f64, d: f64) -> Arm {
        let chain = SerialChain::new(
            UnitQuaternion::identity(),
            vec![
                SerialJoint::new(Isometry3::identity(), Vector3::new(1.0, 0.0, 0.0))
                    .expect("axis"),
            ],
            Isometry3::identity(),
        )
        .expect("chain");
        let body = LinkInertial {
            mass,
            com: Point3::new(0.0, d, 0.0),
            inertia: Matrix3::zeros(),
        };
        return Arm::builder()
            .base_xyz(0.0, 0.0, 0.0)
            .serial_chain(
                chain,
                vec![None],
                vec![body],
                Joints::from_slice(&[0.0]),
            )
            .aggregated_inertials(vec![body])
            .joint_torque_limits(vec![f64::INFINITY])
            .max_joint_speed(10.0)
            .build()
            .expect("single-link arm");
    }

    #[test]
    fn single_link_static_gravity_torque_matches_closed_form() {
        let mass = 0.2;
        let d = 0.15;
        let arm = single_link_arm(mass, d);
        let tau = required_joint_torques(
            &arm,
            &Joints::from_slice(&[0.0]),
            &[0.0],
            &[0.0],
        );
        // 축(x)에 대한 중력 모멘트 크기 = m * g * d.
        let expected = mass * G_MAG * d;
        assert!(
            (tau[0].abs() - expected).abs() < 1e-9,
            "tau={} expected≈{expected}",
            tau[0]
        );
    }

    #[test]
    fn aggregated_shoulder_body_includes_fixed_motor_mass() {
        // 합성 강체 질량이 원본 child link(브래킷)만의 질량보다 유의미하게 커야
        // 한다 — fixed joint로 붙은 모터 몸체 질량이 포함됐는지 확인.
        let arm = Arm::competition().expect("competition arm");
        let bracket_only = arm.link_inertials[1].mass;
        let aggregated = arm.aggregated_inertials[1].mass;
        assert!(
            aggregated > bracket_only + 0.05,
            "합성 질량 {aggregated} 가 브래킷 단독 {bracket_only} 보다 커야 (모터 몸체 포함)"
        );
        // §URDF 하드 계산과 일치: 0.052 + 0.027 + 0.0114 + 0.072 ≈ 0.1624 kg.
        assert!(
            (aggregated - 0.162435).abs() < 1e-4,
            "shoulder 합성 질량 {aggregated} ≈ 0.162435 기대"
        );
    }

    #[test]
    fn two_link_static_sanity_gravity_only() {
        // 정적(속도·가속 0) competition arm: 수직 축(shoulder)은 중력 토크 0,
        // 수평 축(yaw/elbow/wrist)은 유한한 중력 토크를 낸다.
        let arm = Arm::competition().expect("competition arm");
        let n = arm.joint_count();
        // yaw=0에서 평가한다. shoulder 축이 정확히 월드 z(수직)가 되는 것은
        // yaw가 0일 때뿐이라, 휴지 자세(`READY_JOINTS_4DOF`, yaw≈0.12 rad)를
        // 그대로 쓰면 축이 살짝 기울어 중력 모멘트가 정확히 0이 아니다
        // (실측 9.1e-6 N*m — stall 토크의 3e-6 수준으로 물리적으로는 무시할
        // 값이지만, 이 테스트가 검증하려는 명제는 "수직 축에는 중력
        // 모멘트가 없다"이므로 축을 수직으로 두고 봐야 의미가 있다).
        let mut joints = arm.default_joints.clone();
        joints.values[0] = 0.0;
        let tau = required_joint_torques(&arm, &joints, &vec![0.0; n], &vec![0.0; n]);
        // shoulder 축은 월드 z(수직)라 중력 모멘트가 0에 가깝다.
        assert!(
            tau[1].abs() < 1e-6,
            "수직 shoulder 축은 정적 중력 토크 ~0 이어야: {}",
            tau[1]
        );
        // 나머지 수평 축은 유한한 정적 토크, 그러나 모터 stall 한참 아래.
        for i in [0usize, 2, 3] {
            assert!(
                tau[i].abs() > 1e-3 && tau[i].abs() < 3.0,
                "joint {i} 정적 토크 {} 가 물리적으로 타당해야",
                tau[i]
            );
        }
    }

    #[test]
    fn zero_inertia_free_arm_needs_zero_torque_without_gravity() {
        // 질량 0 강체는 어떤 상태에서도 토크 0.
        let chain = SerialChain::new(
            UnitQuaternion::identity(),
            vec![
                SerialJoint::new(
                    Isometry3::translation(0.0, 0.0, 0.1),
                    Vector3::new(0.0, 0.0, 1.0),
                )
                .expect("axis"),
            ],
            Isometry3::identity(),
        )
        .expect("chain");
        let body = LinkInertial {
            mass: 0.0,
            com: Point3::new(0.0, 0.0, 0.0),
            inertia: Matrix3::zeros(),
        };
        let arm = Arm::builder()
            .base_xyz(0.0, 0.0, 0.0)
            .serial_chain(chain, vec![Some(JointLimit::new(-3.0, 3.0))], vec![body], Joints::from_slice(&[0.0]))
            .aggregated_inertials(vec![body])
            .joint_torque_limits(vec![f64::INFINITY])
            .max_joint_speed(10.0)
            .build()
            .expect("massless arm");
        let tau = required_joint_torques(&arm, &Joints::from_slice(&[0.3]), &[2.0], &[5.0]);
        assert!(tau[0].abs() < 1e-12, "질량 0 → 토크 0: {}", tau[0]);
    }

    #[test]
    fn mass_matrix_is_symmetric() {
        // 강체 사슬의 관성 행렬은 물리적으로 대칭이어야 한다.
        let arm = Arm::competition().expect("competition arm");
        let m = mass_matrix(&arm, &arm.default_joints);
        let asym = (&m - m.transpose()).abs().max();
        assert!(asym < 1e-9, "M(q)가 비대칭: max|M - M^T|={asym}");
    }

    #[test]
    fn forward_dynamics_inverts_required_joint_torques() {
        // q̈ 하나를 정하고 필요 토크를 역동역학으로 구한 뒤, 그 토크를 다시
        // 정동역학에 넣으면 같은 q̈가 나와야 한다(선형계의 왕복 일관성).
        let arm = Arm::competition().expect("competition arm");
        let n = arm.joint_count();
        let joints = arm.default_joints.clone();
        let velocities = vec![0.3, -0.2, 0.15, -0.4];
        let accelerations = vec![1.0, -0.5, 2.0, 0.75];
        assert_eq!(velocities.len(), n);
        assert_eq!(accelerations.len(), n);

        let tau = required_joint_torques(&arm, &joints, &velocities, &accelerations);
        let recovered =
            forward_dynamics(&arm, &joints, &velocities, &tau).expect("M(q)가 특이가 아니어야");
        for i in 0..n {
            assert!(
                (recovered[i] - accelerations[i]).abs() < 1e-6,
                "joint {i}: recovered={} expected={}",
                recovered[i],
                accelerations[i]
            );
        }
    }

    #[test]
    fn forward_dynamics_static_hold_needs_zero_accel_when_torque_matches_gravity() {
        // 정지 상태에서 중력 상쇄 토크만 주면 가속도는 0이어야 한다.
        let arm = Arm::competition().expect("competition arm");
        let n = arm.joint_count();
        let joints = arm.default_joints.clone();
        let zero = vec![0.0; n];
        let gravity_torque = bias_torques(&arm, &joints, &zero);
        let accel =
            forward_dynamics(&arm, &joints, &zero, &gravity_torque).expect("M(q) invertible");
        for a in accel {
            assert!(a.abs() < 1e-9, "중력 상쇄 토크인데 가속도 {a} != 0");
        }
    }
}
