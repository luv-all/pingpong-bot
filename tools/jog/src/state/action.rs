//! 패널 버튼 액션.

#[derive(Clone, Copy)]
pub enum Action {
    Sync,
    Discard,
    Apply,
    Preview,
    HomeRail,
}
