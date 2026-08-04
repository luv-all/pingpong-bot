//! 조그 모션 종류.

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Kind {
    Joint,
    Angles,
    RailAbs,
    Ik,
    Pose,
    /// 슈터가 쏜 공의 예측 도달점으로 임팩트 스윙.
    Swing,
    /// 고정 스윙 딕셔너리(IK 없음) — 현재 레일 x에서 START → END 관절각.
    FixedSwing,
}

impl Kind {
    pub fn label(self) -> &'static str {
        return match self {
            Self::Joint => "관절 하나",
            Self::Angles => "관절 전부",
            Self::RailAbs => "레일 절대 위치",
            Self::Ik => "라켓 조금 옮기기",
            Self::Pose => "라켓 옮기기+기울이기",
            Self::Swing => "스윙 (슈터 공)",
            Self::FixedSwing => "고정 스윙 딕셔너리 (IK 없음)",
        };
    }
}
