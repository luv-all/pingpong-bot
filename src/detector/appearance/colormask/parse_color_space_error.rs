/// [`super::ColorSpace`] 파싱 실패.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ParseColorSpaceError;

impl std::fmt::Display for ParseColorSpaceError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        return f.write_str("expected ycrcb|hsv");
    }
}

impl std::error::Error for ParseColorSpaceError {}
