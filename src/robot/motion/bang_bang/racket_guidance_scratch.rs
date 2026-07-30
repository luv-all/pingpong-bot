//! [`super::guidance::step_racket_guidance`] 스크래치 버퍼.

use nalgebra::DMatrix;

use crate::robot::dynamics::{MassMatrixScratch, RneaScratch};

/// [`step_racket_guidance`]가 매 스텝 재사용하는 스크래치 버퍼 모음 — 호출부
/// (`plan_bang_bang_for`, `tools/swing_bench`)가 루프 밖에서 한 번만 만들어
/// 반복 재사용한다. 안 그러면 스텝마다 `mass_matrix`(RNEA n+1회) +
/// `bias_torques`(RNEA 1회)가 각자 새 버퍼를 할당한다.
pub struct RacketGuidanceScratch {
    pub(crate) rnea: RneaScratch,
    pub(crate) mass_matrix: MassMatrixScratch,
    pub(crate) mass: DMatrix<f64>,
    pub(crate) bias_zero_accel: Vec<f64>,
    pub(crate) bias: Vec<f64>,
}

impl RacketGuidanceScratch {
    pub fn new(joint_count: usize) -> Self {
        return Self {
            rnea: RneaScratch::new(),
            mass_matrix: MassMatrixScratch::new(),
            mass: DMatrix::zeros(joint_count, joint_count),
            bias_zero_accel: vec![0.0; joint_count],
            bias: vec![0.0; joint_count],
        };
    }
}
