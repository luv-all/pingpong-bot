//! PnP 결과 (+ 재투영 RMSE).

use crate::camera;

/// PnP 결과 (+ 재투영 RMSE).
#[derive(Debug, Clone)]
pub struct PnpResult {
    pub params: camera::Params,
    /// 선택 해의 재투영 RMSE [px]
    pub reproj_rmse: f64,
    /// IPPE가 낸 후보 수
    pub candidates: usize,
}
