//! ChArUco 보드 규격.

/// ChArUco 보드 규격 (CLI에서 덮어쓸 수 있음).
#[derive(Debug, Clone, Copy)]
pub struct BoardSpec {
    pub squares_x: i32,
    pub squares_y: i32,
    pub square_length_m: f32,
    pub marker_length_m: f32,
}
