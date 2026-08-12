use crate::robot::Joints;

use super::{
    DynamixelBus, DynamixelConfig, DynamixelConfigError, MotorMapping,
    dynamixel_profile_velocity_to_rad_s,
};

fn bench_config() -> DynamixelConfig {
    return DynamixelConfig::default();
}

#[test]
fn profile_velocity_to_rad_s_matches_hand_computed_value() {
    // config/real-hardware.toml의 profile_velocity = 80.
    // 80 LSB * 0.229 rev/min/LSB = 18.32 rev/min
    // 18.32 rev/min * 2*PI rad/rev / 60 s/min ≈ 1.918466 rad/s
    // source: https://emanual.robotis.com/docs/en/dxl/mx/mx-64-2/ (Profile Velocity
    // unit, Protocol 2.0 control table, addr 112), retrieved 2026-07-23.
    let rad_s = dynamixel_profile_velocity_to_rad_s(80);
    assert!((rad_s - 1.918_466).abs() < 1e-4);

    // 0 LSB -> 0 rad/s (register `0` also means "infinite velocity" on real hardware,
    // but the pure unit conversion itself must still be 0).
    assert!((dynamixel_profile_velocity_to_rad_s(0) - 0.0).abs() < 1e-12);
}

#[test]
fn motor_mapping_matches_python_reference() {
    let mapping = MotorMapping::new(bench_config()).expect("valid mapping");

    assert_eq!(mapping.radians_to_ticks(0, 0.0), 2503);
    assert_eq!(
        mapping.radians_to_ticks(0, std::f64::consts::FRAC_PI_2),
        1536
    );
    assert_eq!(
        mapping.radians_to_ticks(1, std::f64::consts::FRAC_PI_2),
        1536
    );
}

#[test]
fn motor_mapping_round_trips_and_clamps_to_motor_limits() {
    let mapping = MotorMapping::new(bench_config()).expect("valid mapping");

    let ticks = mapping.radians_to_ticks(2, -0.4);
    let restored = mapping.ticks_to_radians(2, ticks);
    assert!((restored - -0.4).abs() < 0.002);

    assert_eq!(mapping.radians_to_ticks(0, 100.0), 1024);
    assert_eq!(mapping.radians_to_ticks(0, -100.0), 2503);
}

#[test]
fn base_zero_offset_moves_ready_pair_forty_five_degrees_backward() {
    let calibrated = MotorMapping::new(bench_config()).expect("calibrated mapping");
    let mut zero_offset_config = bench_config();
    zero_offset_config.joint_offsets_rad[0] = 0.0;
    let zero_offset = MotorMapping::new(zero_offset_config).expect("zero-offset mapping");
    let ready_base = crate::defaults::READY_JOINTS_4DOF[0];

    let calibrated_master = calibrated.radians_to_ticks(0, ready_base);
    let old_master = zero_offset.radians_to_ticks(0, ready_base);
    let calibrated_slave = calibrated.config().mirror_tick(calibrated_master);

    assert_eq!(calibrated_master - old_master, 512, "45° = 512tick");
    assert_eq!(calibrated_master, 2217);
    assert_eq!(calibrated_slave, 1929);
    assert!(
        (calibrated.ticks_to_radians(0, calibrated_master) - ready_base).abs() < 0.002,
        "보정 후에도 논리 관절각 round-trip은 유지돼야 함"
    );
}

#[test]
fn wrist_zero_offset_rotates_id5_eight_degrees_toward_bench_alignment() {
    let calibrated = MotorMapping::new(bench_config()).expect("calibrated mapping");
    let mut zero_offset_config = bench_config();
    zero_offset_config.joint_offsets_rad[3] = 0.0;
    let zero_offset = MotorMapping::new(zero_offset_config).expect("zero-offset mapping");
    let ready_wrist = crate::defaults::READY_JOINTS_4DOF[3];

    let calibrated_tick = calibrated.radians_to_ticks(3, ready_wrist);
    let old_tick = zero_offset.radians_to_ticks(3, ready_wrist);
    assert_eq!(old_tick - calibrated_tick, 92, "8° 보정 tick 반올림");
    assert!(
        (calibrated.ticks_to_radians(3, calibrated_tick) - ready_wrist).abs() < 0.002,
        "보정 후에도 논리 관절각 round-trip은 유지돼야 함"
    );
}

#[test]
fn dry_run_limit_escape_holds_outside_start_and_only_moves_inward() {
    let mut config = bench_config();
    config.joint_offsets_rad[0] = 0.0;
    let mut bus = DynamixelBus::dry_run(config).expect("dry-run bus");
    let start = Joints::from_slice(&[100.0_f64.to_radians(), 0.0, -0.2, -0.4]);
    bus.arm_limit_escape_from(&start).expect("arm escape");

    bus.write_joints(&start).expect("hold outside start");
    let start_master_tick = bus.mapping.radians_to_raw_ticks(0, start.values[0]);
    let start_slave_tick = bus.mapping.config().mirror_tick(start_master_tick);
    let goals = bus.last_bus_goals().expect("paired MX-64 goals");
    assert!(goals.contains(&(1, start_master_tick)));
    assert!(goals.contains(&(2, start_slave_tick)));
    let held = bus.read_joints().expect("held pose");
    assert!((held.values[0].to_degrees() - 100.0).abs() < 0.1);

    let inward = Joints::from_slice(&[95.0_f64.to_radians(), 0.0, -0.2, -0.4]);
    bus.write_joints(&inward).expect("move inward");
    let moved = bus.read_joints().expect("inward pose");
    assert!((moved.values[0].to_degrees() - 95.0).abs() < 0.1);

    let outward_again = Joints::from_slice(&[101.0_f64.to_radians(), 0.0, -0.2, -0.4]);
    bus.write_joints(&outward_again)
        .expect("block outward reversal");
    let blocked = bus.read_joints().expect("blocked pose");
    assert!((blocked.values[0].to_degrees() - 95.0).abs() < 0.1);

    let boundary = Joints::from_slice(&[90.0_f64.to_radians(), 0.0, -0.2, -0.4]);
    bus.write_joints(&boundary).expect("enter normal range");
    let normal = bus.read_joints().expect("normal pose");
    assert!((normal.values[0].to_degrees() - 90.0).abs() < 0.1);

    bus.write_joints(&start).expect("normal limit restored");
    let reclamped = bus.read_joints().expect("reclamped pose");
    assert!((reclamped.values[0].to_degrees() - 90.0).abs() < 0.1);
}

#[test]
fn motor_mapping_rejects_mismatched_vector_lengths() {
    let mut config = bench_config();
    config.joint_signs.pop();

    let error = MotorMapping::new(config).unwrap_err();
    assert!(matches!(
        error,
        DynamixelConfigError::VectorLength {
            name: "joint_signs",
            ..
        }
    ));
}

#[test]
fn dry_run_bus_round_trips_last_written_joints() {
    let mut bus = DynamixelBus::dry_run(bench_config()).expect("dry-run bus");
    let target = Joints::from_slice(&[0.2, 0.1, -0.3, 0.2]);

    bus.enable_torque(true).expect("torque");
    bus.write_joints(&target).expect("write");
    let actual = bus.read_joints().expect("read");

    for (expected, actual) in target.values.iter().zip(actual.values) {
        assert!((expected - actual).abs() < 0.002);
    }
}

#[test]
fn torque_enable_holds_every_bus_id_before_motion() {
    let mut bus = DynamixelBus::dry_run(bench_config()).expect("dry-run bus");

    bus.enable_torque(true).expect("torque");

    let goals = bus.last_bus_goals().expect("hold goals");
    let ids: Vec<u8> = goals.iter().map(|(id, _)| *id).collect();
    assert_eq!(ids, vec![1, 3, 4, 5, 2]);
    assert_eq!(goals.len(), bus.mapping.config().bus_ids().len());
}

#[test]
fn dry_run_mirrors_slave_goal_around_zero_tick() {
    let mut bus = DynamixelBus::dry_run(bench_config()).expect("dry-run bus");
    // joint0 sign=-1 → URDF +angle decreases ticks from 2048.
    // Pick ticks via mapping: master absolute ~2276 (200°) → 이론 대칭
    // 1820 (160°) + 실물 조립 영점 50tick = slave 1870.
    let ticks_per_rev = bus.mapping.config().ticks_per_revolution;
    let master_200 = (200.0 * f64::from(ticks_per_rev) / 360.0).round() as i32;
    let expected_slave = bus.mapping.config().mirror_tick(master_200);

    // Drive joint0 so radians_to_ticks yields master_200 (within clamp).
    let angle = bus.mapping.ticks_to_radians(0, master_200);
    bus.write_joints(&Joints::from_slice(&[angle, 0.0, -0.26, 0.0]))
        .expect("write");
    let goals = bus.last_bus_goals().expect("dry-run goals");
    assert!(
        goals
            .iter()
            .any(|(id, tick)| *id == 1 && *tick == master_200)
    );
    assert!(
        goals
            .iter()
            .any(|(id, tick)| *id == 2 && *tick == expected_slave),
        "goals={goals:?} expected slave {expected_slave}"
    );
}

#[test]
fn dry_run_single_joint_write_does_not_command_other_motors() {
    let mut bus = DynamixelBus::dry_run(bench_config()).expect("dry-run bus");
    let before = bus.read_joints().expect("before");

    bus.write_joint(3, -0.25).expect("wrist write");

    let goals = bus.last_bus_goals().expect("dry-run goals");
    assert_eq!(goals.len(), 1);
    assert_eq!(goals[0].0, 5, "라켓 손목축 ID 5만 명령해야 함");
    let after = bus.read_joints().expect("after");
    for index in 0..3 {
        assert!((after.values[index] - before.values[index]).abs() < 0.002);
    }
    assert!((after.values[3] - -0.25).abs() < 0.002);
}

#[test]
fn dry_run_configures_position_mode_with_max_effort() {
    let mut bus = DynamixelBus::dry_run(bench_config()).expect("dry-run bus");
    bus.configure_position_mode_max_effort().expect("configure");
    assert_eq!(bus.last_operating_mode(), Some(3));
    let pwm = bus.last_pwm_limits().expect("pwm");
    assert!(pwm.iter().all(|(_, v)| *v == 885));
    assert_eq!(pwm.len(), 5); // motor_ids 4 + mirror slave
    let current = bus.last_current_limits().expect("current");
    assert_eq!(current, &[(1, 1941), (2, 1941), (3, 1941)]);
}

#[test]
fn mirror_tick_formula() {
    let cfg = bench_config();
    assert_eq!(cfg.mirror_tick(2048), 2098);
    let t200 = (200.0_f64 * 4096.0 / 360.0).round() as i32;
    let t160 = (160.0_f64 * 4096.0 / 360.0).round() as i32;
    assert_eq!(cfg.mirror_tick(t200), t160 + 50);
}
