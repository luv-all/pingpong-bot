//! Newton-Euler 재귀 스크래치 버퍼.

use nalgebra::{Matrix3, Vector3};

/// Newton-Euler 재귀에 필요한 per-joint 스크래치 버퍼.
///
/// 궤적을 여러 시점에서 반복 평가할 때(토크 이용률 샘플링) 매 호출마다 힙
/// 할당하지 않도록 버퍼를 한 번 만들어 재사용한다. `resize`로 관절 개수에 맞춘다.
#[derive(Debug, Default, Clone)]
pub struct RneaScratch {
    pub(super) origin: Vec<Vector3<f64>>,    // 관절 축 원점 (월드)
    pub(super) axis: Vec<Vector3<f64>>,      // 관절 축 단위벡터 (월드)
    pub(super) com_world: Vec<Vector3<f64>>, // 합성 강체 질량중심 (월드)
    pub(super) inertia_world: Vec<Matrix3<f64>>, // 질량중심 기준 관성 (월드축)
    pub(super) force: Vec<Vector3<f64>>,     // F_i = m a_c
    pub(super) moment: Vec<Vector3<f64>>,    // N_i (질량중심 기준)
}

impl RneaScratch {
    pub fn new() -> Self {
        return Self::default();
    }

    pub(super) fn resize(&mut self, n: usize) {
        self.origin.resize(n, Vector3::zeros());
        self.axis.resize(n, Vector3::zeros());
        self.com_world.resize(n, Vector3::zeros());
        self.inertia_world.resize(n, Matrix3::zeros());
        self.force.resize(n, Vector3::zeros());
        self.moment.resize(n, Vector3::zeros());
    }
}
