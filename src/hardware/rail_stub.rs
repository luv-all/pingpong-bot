//! AXL 리니어 레일 자리. 실물 드라이버가 붙기 전에는 x=0으로 고정한다.

#[derive(Debug, Default)]
pub struct RailStub;

impl RailStub {
    pub const fn read_x(&self) -> f64 {
        return 0.0;
    }
}
