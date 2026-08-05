//! 첫 상태를 세울 때의 튜너블. 쓰는 곳은 [`crate::vision::seed`].

/// 재투영이 이보다 벌어지면 두 캠이 다른 걸 잡은 것 [px].
pub const MAX_REPROJECTION_PX: f64 = 14.0;
