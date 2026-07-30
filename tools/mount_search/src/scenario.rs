//! 대표 랠리 시나리오.

use nalgebra::Vector3;
use pingpong_bot::Point3;
use pingpong_bot::constants::table;

pub struct Scenario {
    pub impact: Point3,
    pub incoming_velocity: Vector3<f64>,
}

/// 테이블 폭 × 입사 높이 × 속도 × 하강각. 슈터 기하에 의존하지 않음.
pub fn build_scenarios() -> Vec<Scenario> {
    [0.15, 0.35, 0.5, 0.65, 0.85]
        .into_iter()
        .flat_map(|x_frac| {
            let impact_x = table::WIDTH_X * x_frac;
            [0.10, 0.15, 0.20, 0.25, 0.30]
                .into_iter()
                .flat_map(move |z_offset| {
                    let impact_z = table::SURFACE_Z + z_offset;
                    [7.0, 8.5, 10.0].into_iter().flat_map(move |speed| {
                        [0.10, 0.30].into_iter().map(move |descend_frac| Scenario {
                            impact: Point3::new(impact_x, table::DEFAULT_HIT_PLANE_Y, impact_z),
                            incoming_velocity: Vector3::new(0.0, -speed, -speed * descend_frac),
                        })
                    })
                })
        })
        .collect()
}
