//! 휴리스틱·측정 가능 튜닝값.
//!
//! 앱 숫자는 `crate::entry`에서 조립해 [`install`]한다. 타입에 프리셋을 심지 않는다.
//! 규격·치수(ITTF, CAD, G, 공 반지름)는 `constants/`에 남긴다.

use std::cell::RefCell;

use anyhow::{Result, ensure};
use serde::Deserialize;

thread_local! {
    static INSTALLED: RefCell<Option<Tunables>> = const { RefCell::new(None) };
}

/// 런타임 설정에서 읽은 튜닝을 현재 스레드에 설치한다.
pub fn install(tunables: Tunables) {
    INSTALLED.with(|slot| {
        *slot.borrow_mut() = Some(tunables);
    });
}

/// 설치된 튜닝. 미설치면 panic(앱) / test에서는 entry competition 값.
pub fn current() -> Tunables {
    return INSTALLED.with(|slot| {
        if let Some(t) = slot.borrow().clone() {
            return t;
        }
        #[cfg(test)]
        {
            return crate::entry::competition_tunables();
        }
        #[cfg(not(test))]
        {
            panic!("tunables::install(...) required before tunables::current()");
        }
    });
}

/// 런타임 튜닝 묶음 — control · impact · estimator.
#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct Tunables {
    pub control: ControlParams,
    pub impact: ImpactParams,
    pub estimator: EstimatorParams,
}

#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
pub struct ControlParams {
    pub min_swing_secs: f64,
    pub swing_commit_max_secs: f64,
    pub swing_follow_through_secs: f64,
    pub swing_commit_max_ball_y_frac: f64,
    pub ekf_meas_jump_m: f64,
    pub max_joint_accel: f64,
    /// 관절별 토크 상한 [N·m]. yaw 듀얼 MX-64면 `[12, 6, 6, 6]`.
    pub max_joint_torques: [f64; 4],
    pub joint_inertia: f64,
    pub racket_open_pitch: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
pub struct ImpactParams {
    pub net_clearance: f64,
    pub rally_time_to_bounce: f64,
    pub racket_effective_restitution: f64,
    pub max_return_speed: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
pub struct EstimatorParams {
    pub min_lead: f64,
    pub max_lead: f64,
    pub integrate_dt: f64,
    pub min_approach_speed_y: f64,
    pub min_strike_clearance: f64,
    pub q_pos: f64,
    pub q_vel: f64,
    pub r_meas: f64,
}

impl Tunables {
    pub fn validate(&self) -> Result<()> {
        self.control.validate()?;
        self.impact.validate()?;
        self.estimator.validate()?;
        return Ok(());
    }
}

impl ControlParams {
    pub fn validate(&self) -> Result<()> {
        ensure!(self.min_swing_secs > 0.0, "min_swing_secs > 0");
        ensure!(
            self.swing_commit_max_secs >= self.min_swing_secs,
            "swing_commit_max_secs >= min_swing_secs"
        );
        ensure!(self.swing_follow_through_secs >= 0.0, "follow_through >= 0");
        ensure!(
            (0.0..=1.0).contains(&self.swing_commit_max_ball_y_frac),
            "swing_commit_max_ball_y_frac in 0..=1"
        );
        ensure!(self.ekf_meas_jump_m > 0.0, "ekf_meas_jump_m > 0");
        ensure!(self.max_joint_accel > 0.0, "max_joint_accel > 0");
        ensure!(
            self.max_joint_torques.iter().all(|&t| t > 0.0),
            "max_joint_torques > 0"
        );
        ensure!(self.joint_inertia > 0.0, "joint_inertia > 0");
        return Ok(());
    }
}

impl ImpactParams {
    pub fn validate(&self) -> Result<()> {
        ensure!(self.net_clearance >= 0.0, "net_clearance >= 0");
        ensure!(self.rally_time_to_bounce > 0.0, "rally_time_to_bounce > 0");
        ensure!(
            self.racket_effective_restitution > 0.0,
            "racket_effective_restitution > 0"
        );
        ensure!(self.max_return_speed > 0.0, "max_return_speed > 0");
        return Ok(());
    }
}

impl EstimatorParams {
    pub fn validate(&self) -> Result<()> {
        ensure!(self.min_lead > 0.0, "min_lead > 0");
        ensure!(self.max_lead >= self.min_lead, "max_lead >= min_lead");
        ensure!(self.integrate_dt > 0.0, "integrate_dt > 0");
        ensure!(self.min_approach_speed_y > 0.0, "min_approach_speed_y > 0");
        ensure!(self.min_strike_clearance >= 0.0, "min_strike_clearance >= 0");
        ensure!(self.q_pos >= 0.0, "q_pos >= 0");
        ensure!(self.q_vel >= 0.0, "q_vel >= 0");
        ensure!(self.r_meas > 0.0, "r_meas > 0");
        return Ok(());
    }
}

#[cfg(test)]
mod tests {
    use crate::entry::{competition_tunables, install_competition_tunables};

    use super::*;

    #[test]
    fn competition_tunables_validate() {
        let t = competition_tunables();
        t.validate().unwrap();
        assert!((t.control.min_swing_secs - 0.08).abs() < 1e-12);
        assert!((t.control.max_joint_torques[0] - 12.0).abs() < 1e-12);
        assert!((t.impact.max_return_speed - 6.0).abs() < 1e-12);
    }

    #[test]
    fn install_then_current() {
        install_competition_tunables();
        assert_eq!(current().control.max_joint_torques[0], 12.0);
    }
}
