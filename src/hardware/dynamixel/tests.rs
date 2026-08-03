use crate::robot::Joints;

use super::{
    DynamixelBus, DynamixelConfig, DynamixelConfigError, MotorMapping,
    dynamixel_profile_velocity_to_rad_s,
};

fn bench_config() -> DynamixelConfig {
    return DynamixelConfig::default();
}

#[test]
fn profile_velocity_to_rad_s_matches_real_default() {
    // 실기 내부 프로파일을 플래너의 관절 속도 상한과 맞춘다.
    // source: https://emanual.robotis.com/docs/en/dxl/mx/mx-64-2/ (Profile Velocity
    // unit, Protocol 2.0 control table, addr 112), retrieved 2026-07-23.
    let raw = DynamixelConfig::default().profile_velocity;
    assert_eq!(DynamixelConfig::default().profile_acceleration, 60);
    let rad_s = dynamixel_profile_velocity_to_rad_s(raw);
    assert_eq!(raw, 192);
    assert!(
        (rad_s - crate::defaults::DYNAMIXEL_MAX_JOINT_SPEED_RAD_S).abs() < 0.02,
        "실기 프로파일 {rad_s} rad/s != 플래너 {} rad/s",
        crate::defaults::DYNAMIXEL_MAX_JOINT_SPEED_RAD_S,
    );

    // 0 LSB -> 0 rad/s (register `0` also means "infinite velocity" on real hardware,
    // but the pure unit conversion itself must still be 0).
    assert!((dynamixel_profile_velocity_to_rad_s(0) - 0.0).abs() < 1e-12);
}

#[test]
fn motor_mapping_matches_python_reference() {
    let mapping = MotorMapping::new(bench_config()).expect("valid mapping");

    assert_eq!(mapping.radians_to_ticks(0, 0.0), 2048);
    assert_eq!(
        mapping.radians_to_ticks(0, std::f64::consts::FRAC_PI_2),
        1024
    );
    assert_eq!(
        mapping.radians_to_ticks(1, std::f64::consts::FRAC_PI_2),
        2560
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
    let target = Joints::from_slice(&[-0.2, 0.1, -0.3, 0.2]);

    bus.enable_torque(true).expect("torque");
    bus.write_joints(&target).expect("write");
    let actual = bus.read_joints().expect("read");

    for (expected, actual) in target.values.iter().zip(actual.values) {
        assert!((expected - actual).abs() < 0.002);
    }
}

#[test]
fn dry_run_lock_current_position_enables_torque_without_moving() {
    let mut bus = DynamixelBus::dry_run(bench_config()).expect("dry-run bus");
    let before = bus.read_joints().expect("before");

    bus.lock_current_position().expect("torque lock");

    assert!(bus.torque_is_locked());
    assert_eq!(bus.read_joints().expect("after"), before);
}

#[test]
fn dry_run_lock_at_joints_sets_goal_and_enables_torque_without_readback() {
    let mut bus = DynamixelBus::dry_run(bench_config()).expect("dry-run bus");
    let goal = Joints::from_slice(&[0.2, -0.1, 0.3, -0.4]);

    bus.lock_at_joints(&goal).expect("goal torque lock");

    assert!(bus.torque_is_locked());
    let cached = bus.cached_joints().expect("cached goal");
    for (expected, actual) in goal.values.iter().zip(cached.values) {
        assert!((expected - actual).abs() < 0.002);
    }
}

#[test]
fn dry_run_mirrors_slave_goal_around_zero_tick() {
    let mut bus = DynamixelBus::dry_run(bench_config()).expect("dry-run bus");
    // joint0 sign=-1 → URDF +angle decreases ticks from 2048.
    // Pick ticks via mapping: want master absolute ~2276 (200°) → slave 1820 (160°).
    let zero = bus.mapping.config().zero_tick;
    let ticks_per_rev = bus.mapping.config().ticks_per_revolution;
    let master_200 = (200.0 * f64::from(ticks_per_rev) / 360.0).round() as i32;
    let expected_slave = 2 * zero - master_200;

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
    assert_eq!(cfg.mirror_tick(2048), 2048);
    let t200 = (200.0_f64 * 4096.0 / 360.0).round() as i32;
    let t160 = (160.0_f64 * 4096.0 / 360.0).round() as i32;
    assert_eq!(cfg.mirror_tick(t200), t160);
}
