//! j0~j3 전체 모션 계획 — overshoot 목표로 가는 j0/j1/j2와 backswing-스냅
//! j3를 하나의 시간축에 묶는다. 하드웨어 접근이 없어 실기 없이도
//! `pingpong_bot::defaults::robot()`의 소프트웨어 팔 모델로 테스트할 수 있다.

use pingpong_bot::Point3;
use pingpong_bot::robot::motion::quadratic_segment::QuadraticSegment;
use pingpong_bot::robot::motion::ramp_cruise_segment::RampCruiseSegment;
use pingpong_bot::robot::{Arm, IkSearch, Joints, Pose};

use crate::geometry::overshoot_target;
use crate::wrist_motion::WristMotion;

/// j0·j2 인덱스 — 이 팔에서 토크가 가장 큰 두 관절, 생산 코드의
/// `POWER_SWEEP_JOINT_INDICES`와 같은 관례.
const POWER_JOINT_INDICES: [usize; 2] = [0, 2];
/// 어깨(j1) 인덱스 — IK가 요구하는 만큼만 따라가는 수동 관절.
const PASSIVE_JOINT_INDEX: usize = 1;

#[derive(Debug, Clone, Copy, PartialEq)]
enum JointProfile {
    RampCruise(RampCruiseSegment),
    Quadratic(QuadraticSegment),
}

impl JointProfile {
    fn sample(&self, t: f64) -> (f64, f64, f64) {
        return match self {
            JointProfile::RampCruise(segment) => segment.sample(t),
            JointProfile::Quadratic(segment) => segment.sample(t),
        };
    }
}

pub struct SwingPlan {
    /// `PASSIVE_JOINT_INDEX`를 포함해 j0~j2 전부(손목 제외) 담는다 — 인덱스로
    /// 직접 접근한다.
    arm_profiles: Vec<JointProfile>,
    wrist_index: usize,
    wrist: WristMotion,
    overshoot_joints: Joints,
    total_duration_secs: f64,
}

impl SwingPlan {
    #[allow(clippy::too_many_arguments)]
    pub fn build(
        arm: &Arm,
        current: &Joints,
        target: Point3,
        overshoot_m: f64,
        total_duration_secs: f64,
        impact_time_secs: f64,
        wrist_cocked_rad: f64,
        backswing_secs: f64,
        ramp_secs: f64,
        snap_velocity_margin: f64,
    ) -> Result<Self, String> {
        if impact_time_secs >= total_duration_secs {
            return Err(format!(
                "impact_time_secs ({impact_time_secs:.4}) must be less than total_duration_secs \
                 ({total_duration_secs:.4}) -- the overshoot target must be reached after the \
                 real contact instant"
            ));
        }
        let rail_x = arm.rail.as_ref().map_or(0.0, |rail| rail.default_x());
        let (overshoot_position, push_direction) = overshoot_target(target, overshoot_m);
        let hint = Pose::new(rail_x, current.clone());
        let (overshoot_pose, _) = arm
            .inverse_pose_at_fixed_rail_best_normal(
                rail_x,
                overshoot_position,
                push_direction,
                &hint,
                IkSearch::Global,
            )
            .map_err(|error| format!("overshoot target unreachable: {error:?}"))?;
        let overshoot_joints = overshoot_pose.joints;

        let ramp_accel = arm.max_joint_speed / ramp_secs.max(f64::EPSILON);
        let mut arm_profiles = Vec::with_capacity(3);
        for index in 0..3 {
            if POWER_JOINT_INDICES.contains(&index) {
                let segment = RampCruiseSegment::new(
                    current.values[index],
                    overshoot_joints.values[index],
                    total_duration_secs,
                    ramp_accel,
                )
                .ok_or_else(|| {
                    format!(
                        "j{index} cannot reach its overshoot target ({:.4} -> {:.4}) within \
                         total_duration_secs={total_duration_secs:.4} even at the joint speed \
                         ceiling -- shorten overshoot_m or lengthen total_duration_secs",
                        current.values[index], overshoot_joints.values[index]
                    )
                })?;
                arm_profiles.push(JointProfile::RampCruise(segment));
            } else {
                debug_assert_eq!(index, PASSIVE_JOINT_INDEX);
                let segment = QuadraticSegment::new(
                    current.values[index],
                    0.0,
                    overshoot_joints.values[index],
                    total_duration_secs,
                );
                arm_profiles.push(JointProfile::Quadratic(segment));
            }
        }

        let wrist_index = arm
            .wrist_joint_index()
            .ok_or_else(|| "arm has no wrist joint".to_string())?;
        let limit = arm
            .joint_limit(wrist_index)
            .ok_or_else(|| "wrist joint has no configured limit".to_string())?;
        let wrist = WristMotion::try_new(
            current.values[wrist_index],
            wrist_cocked_rad,
            limit,
            backswing_secs,
            impact_time_secs,
            total_duration_secs,
            arm.max_joint_speed,
            snap_velocity_margin,
        )?;

        return Ok(Self {
            arm_profiles,
            wrist_index,
            wrist,
            overshoot_joints,
            total_duration_secs,
        });
    }

    pub fn overshoot_joints(&self) -> &Joints {
        return &self.overshoot_joints;
    }

    pub fn wrist_snap_target_angle(&self) -> f64 {
        return self.wrist.snap_target_angle();
    }

    pub fn wrist_peak_speed(&self, samples: usize) -> f64 {
        return self.wrist.peak_speed(samples);
    }

    pub fn total_duration_secs(&self) -> f64 {
        return self.total_duration_secs;
    }

    /// `t`[s]에서 4관절 전체 각도를 샘플한다.
    pub fn sample(&self, t: f64) -> Joints {
        let mut values = vec![0.0; 4];
        for index in 0..3 {
            values[index] = self.arm_profiles[index].sample(t).0;
        }
        values[self.wrist_index] = self.wrist.sample(t).0;
        return Joints { values };
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use pingpong_bot::constants::table;

    fn sample_target() -> Point3 {
        return Point3::new(table::WIDTH_X * 0.5, 0.3, 0.95);
    }

    #[test]
    fn build_succeeds_for_a_reasonable_center_table_target() {
        let active = pingpong_bot::defaults::robot().expect("active robot");
        let arm = &active.arm;
        let current = arm.default_joints.clone();
        let plan = SwingPlan::build(
            arm,
            &current,
            sample_target(),
            0.05,
            2.0,
            1.5,
            -0.6,
            0.3,
            0.060,
            0.85,
        );
        assert!(
            plan.is_ok(),
            "expected feasible plan, got err: {:?}",
            plan.err()
        );
    }

    #[test]
    fn build_rejects_when_impact_time_is_not_before_total_duration() {
        let active = pingpong_bot::defaults::robot().expect("active robot");
        let arm = &active.arm;
        let current = arm.default_joints.clone();
        let plan = SwingPlan::build(
            arm,
            &current,
            sample_target(),
            0.05,
            0.20,
            0.20,
            -0.2,
            0.05,
            0.060,
            0.85,
        );
        assert!(plan.is_err());
    }

    #[test]
    fn build_rejects_when_ramp_cruise_is_infeasible() {
        let active = pingpong_bot::defaults::robot().expect("active robot");
        let arm = &active.arm;
        let current = arm.default_joints.clone();
        // 아주 짧은 총 시간 + 큰 overshoot로 j0/j2 도달 불가능을 강제한다.
        let plan = SwingPlan::build(
            arm,
            &current,
            sample_target(),
            0.50,
            0.02,
            0.01,
            -0.2,
            0.005,
            0.060,
            0.85,
        );
        assert!(plan.is_err());
    }

    #[test]
    fn build_rejects_when_wrist_snap_does_not_fit() {
        let active = pingpong_bot::defaults::robot().expect("active robot");
        let arm = &active.arm;
        let current = arm.default_joints.clone();
        let wrist_limit = arm.joint_limit(3).expect("wrist limit");
        let plan = SwingPlan::build(
            arm,
            &current,
            sample_target(),
            0.05,
            0.30,
            0.11,
            wrist_limit.min * 0.9,
            0.10,
            0.060,
            0.85,
        );
        assert!(plan.is_err());
    }

    #[test]
    fn sample_at_zero_matches_current_joints() {
        let active = pingpong_bot::defaults::robot().expect("active robot");
        let arm = &active.arm;
        let current = arm.default_joints.clone();
        let plan = SwingPlan::build(
            arm,
            &current,
            sample_target(),
            0.05,
            2.0,
            1.5,
            -0.6,
            0.3,
            0.060,
            0.85,
        )
        .expect("feasible plan");
        let sampled = plan.sample(0.0);
        for index in 0..4 {
            assert!(
                (sampled.values[index] - current.values[index]).abs() < 1e-6,
                "joint {index} mismatch at t=0: {} vs {}",
                sampled.values[index],
                current.values[index]
            );
        }
    }

    #[test]
    fn sample_at_total_duration_reaches_overshoot_joints_for_arm_joints() {
        let active = pingpong_bot::defaults::robot().expect("active robot");
        let arm = &active.arm;
        let current = arm.default_joints.clone();
        let plan = SwingPlan::build(
            arm,
            &current,
            sample_target(),
            0.05,
            2.0,
            1.5,
            -0.6,
            0.3,
            0.060,
            0.85,
        )
        .expect("feasible plan");
        let sampled = plan.sample(2.0);
        let overshoot = plan.overshoot_joints();
        for index in 0..3 {
            assert!(
                (sampled.values[index] - overshoot.values[index]).abs() < 1e-6,
                "joint {index} should reach its overshoot target at total_duration_secs"
            );
        }
    }

    #[test]
    fn sample_wrist_reaches_snap_target_at_impact_time() {
        let active = pingpong_bot::defaults::robot().expect("active robot");
        let arm = &active.arm;
        let current = arm.default_joints.clone();
        let plan = SwingPlan::build(
            arm,
            &current,
            sample_target(),
            0.05,
            2.0,
            1.5,
            -0.6,
            0.3,
            0.060,
            0.85,
        )
        .expect("feasible plan");
        let sampled = plan.sample(1.5);
        assert!(
            (sampled.values[3] - plan.wrist_snap_target_angle()).abs() < 1e-6,
            "wrist should reach its snap target exactly at impact_time_secs"
        );
    }
}
