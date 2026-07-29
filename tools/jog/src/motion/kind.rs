//! 조그 모션 종류.

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MotionKind {
    Joint,
    Angles,
    RailAbs,
    Ik,
    Pose,
    Swing,
    /// 공 도달점으로 관절·레일만 이동 (스윙 없음).
    AimBall,
    /// 공 도달점 + 입사 속도 → 임팩트 역산 스윙.
    SwingBall,
}

impl MotionKind {
    pub fn label(self) -> &'static str {
        return match self {
            Self::Joint => "관절 하나",
            Self::Angles => "관절 전부",
            Self::RailAbs => "레일 절대 위치",
            Self::Ik => "라켓 조금 옮기기",
            Self::Pose => "라켓 옮기기+기울이기",
            Self::Swing => "스윙 (속도 직접)",
            Self::AimBall => "공 도달점 조준",
            Self::SwingBall => "공 도달점 스윙",
        };
    }
}
