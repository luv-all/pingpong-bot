/// `AxmSignalSetSoftLimit` 인자 (미터 단위).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SoftLimitArgs {
    pub use_: u32,
    pub stop_mode: u32,
    pub selection: u32,
    pub positive_m: f64,
    pub negative_m: f64,
}
