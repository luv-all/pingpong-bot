/// 프리뷰 키 입력.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PreviewAction {
    /// 키 없음 (timeout)
    Continue,
    /// `q` / ESC
    Quit,
    /// 그 외 키 (Space=32, 's'=115, 화살표=waitKeyEx 풀코드 등).
    Key(i32),
}
