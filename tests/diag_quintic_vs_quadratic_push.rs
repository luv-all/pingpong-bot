//! 실험: 2026-08-13 재측정된 `READY_JOINTS_4DOF`(새 홈 자세)에서
//! 시작할 때, 기존 quintic 다관절 푸시([`plan_fixed_joint_swing`])와 새
//! 등가속(quadratic) 버전([`plan_fixed_joint_swing_quadratic`])의 실현가능성을
//! 비교한다.
//!
//! 배경: 사용자 관찰 — 새 홈 자세(j2 elbow가 상단 한계 8.7° 이내로 접힘)에서
//! 기존 스쿱/푸시 스윙이 잘 안 먹는다. quintic은 `a0=0`(ease-in)으로 시작해
//! 목표 임팩트 속도를 맞추려고 도중에 quadratic보다 더 큰 첨두 가속도를
//! 요구하는데, 이게 한계 가까이 접힌 관절에서 실현가능성을 깎아먹는지가
//! 이 실험의 질문이다.
//!
//! 실행:
//!   cargo test --release --test diag_quintic_vs_quadratic_push -- --ignored --nocapture

use pingpong_bot::Point3;
use pingpong_bot::constants::table;
use pingpong_bot::defaults;
use pingpong_bot::defaults::READY_JOINTS_4DOF;
use pingpong_bot::robot;
use pingpong_bot::robot::motion::physics::{
    plan_ball_alignment, plan_fixed_joint_swing, plan_fixed_joint_swing_quadratic,
};

#[derive(Debug, Default)]
struct Tally {
    attempts: usize,
    succeeded: usize,
    peak_joint_speed_sum: f64,
    peak_joint_accel_sum: f64,
}

impl Tally {
    fn record_success(&mut self, peak_joint_speed: f64, peak_joint_accel: f64) {
        self.succeeded += 1;
        self.peak_joint_speed_sum += peak_joint_speed;
        self.peak_joint_accel_sum += peak_joint_accel;
    }

    fn report(&self, label: &str) {
        let rate = 100.0 * self.succeeded as f64 / self.attempts.max(1) as f64;
        if self.succeeded == 0 {
            println!(
                "{label}: {}/{} succeeded ({rate:.0}%)",
                self.succeeded, self.attempts
            );
            return;
        }
        let avg_speed = self.peak_joint_speed_sum / self.succeeded as f64;
        let avg_accel = self.peak_joint_accel_sum / self.succeeded as f64;
        println!(
            "{label}: {}/{} succeeded ({rate:.0}%), avg peak joint speed={avg_speed:.3} rad/s, avg peak joint accel={avg_accel:.3} rad/s^2",
            self.succeeded, self.attempts
        );
    }
}

#[test]
#[ignore]
fn quintic_vs_quadratic_push_from_new_home_pose() {
    let active = defaults::robot().expect("active robot");
    let arm = &active.arm;

    // 케이스 1: 정렬 이동 없이 홈 자세에서 바로 스윙 — 사용자가 관찰한 가장
    // 나쁜 경우(팔이 정렬로 더 펴지기 전, elbow가 한계 가까이 접힌 채로).
    let home = robot::Pose::new(
        arm.rail.as_ref().map_or(0.0, |rail| rail.default_x()),
        robot::Joints::from_slice(&READY_JOINTS_4DOF),
    );
    println!("\n--- 케이스 1: 새 홈 자세에서 바로 스윙 (정렬 없음) ---");
    match plan_fixed_joint_swing(arm, &home) {
        Ok(planned) => println!(
            "quintic: 성공, peak joint speed={:.3} rad/s, peak joint accel={:.3} rad/s^2",
            planned.trajectory.peak_joint_speed(),
            planned.trajectory.peak_joint_acceleration()
        ),
        Err(error) => println!("quintic: 실패 — {error:?}"),
    }
    match plan_fixed_joint_swing_quadratic(arm, &home) {
        Ok(planned) => println!(
            "quadratic: 성공, peak joint speed={:.3} rad/s, peak joint accel={:.3} rad/s^2",
            planned.trajectory.peak_joint_speed(),
            planned.trajectory.peak_joint_acceleration()
        ),
        Err(error) => println!("quadratic: 실패 — {error:?}"),
    }

    // 케이스 2: 테이블 전역 대표 타점으로 정렬한 뒤 스윙 — 실제 커밋 경로에
    // 더 가까운 시나리오. 좌/중앙/우 × 근/원 6개 지점.
    println!("\n--- 케이스 2: 테이블 전역 대표 타점 정렬 후 스윙 ---");
    let mut quintic_tally = Tally::default();
    let mut quadratic_tally = Tally::default();
    let y_positions = [0.15, 0.45, 0.75]; // 마운트 기준 y [m], 근~원
    let x_fractions = [0.15, 0.5, 0.85]; // 테이블 폭 비율, 좌/중/우
    for &y in &y_positions {
        for &x_fraction in &x_fractions {
            let ball = Point3::new(table::WIDTH_X * x_fraction, y, 0.95);
            let alignment = match plan_ball_alignment(arm, &home, ball) {
                Ok(alignment) => alignment,
                Err(_) => continue,
            };
            let aligned = robot::Pose::new(
                alignment.follow_through_rail_x,
                alignment.follow_through.clone(),
            );

            quintic_tally.attempts += 1;
            if let Ok(planned) = plan_fixed_joint_swing(arm, &aligned) {
                quintic_tally.record_success(
                    planned.trajectory.peak_joint_speed(),
                    planned.trajectory.peak_joint_acceleration(),
                );
            }

            quadratic_tally.attempts += 1;
            if let Ok(planned) = plan_fixed_joint_swing_quadratic(arm, &aligned) {
                quadratic_tally.record_success(
                    planned.trajectory.peak_joint_speed(),
                    planned.trajectory.peak_joint_acceleration(),
                );
            }
        }
    }
    quintic_tally.report("quintic (기존)");
    quadratic_tally.report("quadratic (신규)");
}
