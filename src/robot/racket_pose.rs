//! 월드 좌표계 라켓 자세.

use nalgebra::Vector3;

use crate::Point3;

/// 월드 좌표계 라켓 자세 - sim/real 동일 표현.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RacketPose {
    /// 라켓 중심 위치 (월드)
    pub position: Point3,
    /// 라켓 면 법선 (단위 벡터)
    pub normal: Vector3<f64>,
    /// Hamilton 단위 쿼터니언 (w, x, y, z) - 어댑터가 SDK 회전으로 변환
    pub orientation: [f64; 4],
}
