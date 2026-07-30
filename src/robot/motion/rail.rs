//! 스윙 quintic에 딸린 리니어 X 이동.

/// quintic 스윙에 딸린 리니어 X 이동.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Rail {
    pub start: f64,
    pub end: f64,
    pub start_velocity: f64,
    pub end_velocity: f64,
}

impl Rail {
    pub const fn fixed(x: f64) -> Self {
        return Self {
            start: x,
            end: x,
            start_velocity: 0.0,
            end_velocity: 0.0,
        };
    }
}

impl Default for Rail {
    fn default() -> Self {
        return Self::fixed(0.0);
    }
}
