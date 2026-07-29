//! revolute 관절 1축 허용 범위 [rad].

/// revolute 관절 1축 허용 범위 [rad].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct JointLimit {
    /// 최소 각도 [rad]
    pub min: f64,
    /// 최대 각도 [rad]
    pub max: f64,
}

impl JointLimit {
    /// [min, max] 범위를 만든다.
    pub const fn new(min: f64, max: f64) -> Self {
        return Self { min, max };
    }

    /// 각도가 허용 범위 안인지 확인한다.
    pub fn contains(self, angle: f64) -> bool {
        return angle >= self.min && angle <= self.max;
    }
}
