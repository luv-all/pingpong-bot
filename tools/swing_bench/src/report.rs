//! 벤치 결과 리포트.

use serde::Serialize;

#[derive(Debug, Serialize)]
pub struct Report {
    pub robot: String,
    /// 목표 라켓 속도가 관절/레일 속도 한계를 넘어 실행 전에 잘렸는지.
    pub target_speed_clamped: bool,
    pub feasible: bool,
    pub achieved_time_secs: f64,
    /// 위치만 허용오차 안에 처음 들어온 시각 [s].
    pub position_reached_time_secs: Option<f64>,
    pub max_time_secs: f64,
    pub time_budget_secs: Option<f64>,
    pub within_time_budget: Option<bool>,
    pub position_error: f64,
    /// 종료 시점 실제 라켓 속도 크기 [m/s].
    pub achieved_racket_speed_m_s: f64,
    /// 목표 라켓 속도 크기 [m/s].
    pub target_racket_speed_m_s: f64,
    /// 종료 시점 라켓 속도 방향과 목표 방향의 각도차 [deg].
    pub racket_direction_error_deg: f64,
    pub peak_joint_torque_utilization: Vec<f64>,
    pub peak_joint_speed_rad_s: Vec<f64>,
    pub peak_joint_speed_ratio_to_cap: Vec<f64>,
    pub peak_rail_speed_m_s: f64,
    pub peak_rail_speed_ratio_to_cap: f64,
}
