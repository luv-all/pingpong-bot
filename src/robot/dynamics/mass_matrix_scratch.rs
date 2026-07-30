//! 질량 행렬 계산용 스크래치 버퍼.

/// [`mass_matrix_into`]가 매 스텝 재계산할 때 쓰는 스크래치 버퍼 모음 —
/// 영가속도/단위가속도/토크 임시 벡터를 매 호출 새로 할당하지 않도록 호출부가
/// 소유해 재사용한다(`RneaScratch`와 별개로, `mass_matrix`가 RNEA를 n+1회
/// 도는 데 필요한 관절 개수짜리 작은 벡터들).
#[derive(Debug, Default, Clone)]
pub struct MassMatrixScratch {
    pub(super) zero_accel: Vec<f64>,
    pub(super) unit_accel: Vec<f64>,
    pub(super) bias: Vec<f64>,
    pub(super) tau: Vec<f64>,
}

impl MassMatrixScratch {
    pub fn new() -> Self {
        return Self::default();
    }

    pub(super) fn resize(&mut self, n: usize) {
        self.zero_accel.clear();
        self.zero_accel.resize(n, 0.0);
        self.unit_accel.clear();
        self.unit_accel.resize(n, 0.0);
        self.bias.resize(n, 0.0);
        self.tau.resize(n, 0.0);
    }
}
