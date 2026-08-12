//! 삼각측량 궤적의 이벤트 분석 공개 진입점.

use super::{BounceEvent, RollEvent, TrajPoint, traj_measure};

pub struct TrajAnalysis;

impl TrajAnalysis {
    pub fn detect_bounces(traj: &[TrajPoint]) -> Vec<BounceEvent> {
        return traj_measure::detect_bounces(traj);
    }

    pub fn detect_rolls(traj: &[TrajPoint]) -> Vec<RollEvent> {
        return traj_measure::detect_rolls(traj);
    }

    pub fn mean_bounce_e(events: &[BounceEvent]) -> Option<f64> {
        return traj_measure::mean_bounce_e(events);
    }

    pub fn mean_roll_mu(events: &[RollEvent]) -> Option<f64> {
        return traj_measure::mean_roll_mu(events);
    }

    /// 시간-위치 표본 창의 최소자승 기울기(=속도) — 인접 2점차보다 잡음에 강하다.
    pub fn windowed_velocity(points: &[TrajPoint]) -> Option<nalgebra::Vector3<f64>> {
        return traj_measure::windowed_velocity(points);
    }
}
