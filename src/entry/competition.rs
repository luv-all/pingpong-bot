//! competition 배선 SSOT — 숫자·DSL·하드웨어 선언은 여기만.

use nalgebra::{Isometry3, UnitQuaternion, Vector3};

use crate::constants::table;
use crate::detector::{
    ColorSpace, ColormaskConfig, ColormaskDetector, ContourDetector, RoiTrack, Scorer, ScorerParams,
    fuse, track,
};
use crate::generators;
use crate::hardware::dynamixel::{DynamixelConfig, MirrorSlave};
use crate::physics_config::PhysicsParams;
use crate::planner::InterceptWindow;
use crate::robot::{Arm, ArmBuildError, JointLimit, Joints, SerialChain, SerialJoint};
use crate::tunables::{ControlParams, EstimatorParams, ImpactParams, Tunables, install};

/// competition 팔 — `Arm::builder` 조립. 타입 프리셋 메서드 없음.
pub fn competition_arm() -> Result<Arm, ArmBuildError> {
    const BASE_Y: f64 = 0.02;
    const MAX_JOINT_SPEED: f64 = 16.0;
    const RAIL_MAX_SPEED: f64 = 12.0;

    let joints = vec![
        SerialJoint::new(
            Isometry3::translation(-0.02575, 0.028, 0.0601),
            Vector3::new(-1.0, 0.0, 0.0),
        )
        .expect("4-dof q0 axis"),
        SerialJoint::new(
            Isometry3::translation(0.0255, 0.0, 0.0825),
            Vector3::new(0.0, 0.0, -1.0),
        )
        .expect("4-dof q1 axis"),
        SerialJoint::new(
            Isometry3::translation(0.0, 0.025, 0.1398),
            Vector3::new(-1.0, 0.0, 0.0),
        )
        .expect("4-dof q2 axis"),
        SerialJoint::new(
            Isometry3::translation(0.0, 0.1518, 0.0),
            Vector3::new(-1.0, 0.0, 0.0),
        )
        .expect("4-dof q3 axis"),
    ];
    let chain = SerialChain::new(
        UnitQuaternion::identity(),
        joints,
        Isometry3::translation(0.0, 0.0513, -0.034),
    )
    .expect("4-dof serial chain");
    return Arm::builder()
        .base_xyz(0.0, BASE_Y, table::SURFACE_Z)
        .linear_rail(
            BASE_Y,
            table::SURFACE_Z,
            0.0,
            table::WIDTH_X,
            RAIL_MAX_SPEED,
        )
        .serial_chain(
            chain,
            vec![
                None,
                Some(JointLimit::new(-0.523599, 0.523599)),
                Some(JointLimit::new(-2.007129, 1.48353)),
                Some(JointLimit::new(-2.094395, 2.094395)),
            ],
            Joints::from_slice(&[0.0, 0.0, -0.2617995, 0.0]),
        )
        .max_joint_speed(MAX_JOINT_SPEED)
        .build();
}

pub fn competition_physics() -> PhysicsParams {
    return PhysicsParams {
        restitution: 0.85,
        friction: 0.15,
        drag: 0.0,
    };
}

pub fn competition_tunables() -> Tunables {
    return Tunables {
        control: ControlParams {
            min_swing_secs: 0.08,
            swing_commit_max_secs: 0.35,
            swing_follow_through_secs: 0.06,
            swing_commit_max_ball_y_frac: 0.55,
            ekf_meas_jump_m: 0.6,
            max_joint_accel: 400.0,
            // yaw 듀얼 MX-64 stall≈6 → 12; 나머지 단일. I는 α≈τ/I가 스윙 가능하도록.
            max_joint_torques: [12.0, 6.0, 6.0, 6.0],
            joint_inertia: 0.015,
            racket_open_pitch: 0.45,
        },
        impact: ImpactParams {
            net_clearance: 0.08,
            rally_time_to_bounce: 0.55,
            racket_effective_restitution: 0.42,
            max_return_speed: 6.0,
        },
        estimator: EstimatorParams {
            min_lead: 0.05,
            max_lead: 1.2,
            integrate_dt: 0.001,
            min_approach_speed_y: 0.8,
            min_strike_clearance: 0.05,
            q_pos: 1.0e-4,
            q_vel: 1.0e-2,
            r_meas: 0.0009,
        },
    };
}

pub fn install_competition_tunables() {
    install(competition_tunables());
}

pub fn competition_intercept() -> InterceptWindow {
    return InterceptWindow {
        y_min: 0.20,
        y_max: 0.55,
        sample_step: 0.05,
    };
}

const ROI_HALF_PX: i32 = 80;
const MOTION_WEIGHT: f64 = 0.5;

fn competition_scorer() -> Scorer {
    let params = ScorerParams {
        min_area_px: 20.0,
        max_area_px: 20_000.0,
        min_circularity: 0.55,
    };
    return Scorer::from(&params).with_motion_weight(MOTION_WEIGHT);
}

fn competition_colormask() -> ColormaskDetector {
    let cfg = ColormaskConfig {
        space: ColorSpace::Ycrcb,
        c0_min: 0,
        c0_max: 255,
        c1_min: 133,
        c1_max: 173,
        c2_min: 77,
        c2_max: 127,
    };
    return ColormaskDetector::new(cfg);
}

/// `fuse(generators![…], scorer)` — TOML 브릿지 없음.
pub fn competition_detector() -> RoiTrack {
    let scorer = competition_scorer();
    let fuse_det = fuse(
        generators![
            competition_colormask(),
            ContourDetector::from(&ScorerParams {
                min_area_px: 20.0,
                max_area_px: 20_000.0,
                min_circularity: 0.55,
            }),
        ],
        scorer,
    )
    .with_motion_weight(MOTION_WEIGHT);
    return track(fuse_det, ROI_HALF_PX);
}

/// 벤치 4-dof Dynamixel + yaw 미러(ID1↔ID2). 포트는 호출측이 덮어쓴다.
pub fn competition_dynamixel() -> DynamixelConfig {
    return DynamixelConfig {
        port: "COM8".to_owned(),
        baudrate: 57_600,
        protocol_version: 2.0,
        motor_ids: vec![1, 3, 4, 5],
        ticks_per_revolution: 4096,
        zero_tick: 2048,
        addr_goal_position: 116,
        addr_torque_enable: 64,
        addr_present_position: 132,
        addr_profile_acceleration: 108,
        addr_profile_velocity: 112,
        profile_acceleration: 20,
        profile_velocity: 80,
        comm_retries: 5,
        comm_retry_delay_ms: 20,
        stream_hz: 200.0,
        joint_signs: vec![-1, 1, 1, 1],
        joint_offsets_rad: vec![0.0; 4],
        motor_angle_limits_deg: vec![
            [90.0, 220.0],
            [135.0, 225.0],
            [92.0, 230.0],
            [120.0, 220.0],
        ],
        mirror_slaves: vec![MirrorSlave {
            master_id: 1,
            slave_id: 2,
        }],
    };
}
