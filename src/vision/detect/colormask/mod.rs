//! 색 마스크 파라미터와 그 JSON 저장소.
//!
//! `tune-colormask`가 쓰고 [`super::ColorBox`]가 읽는다. 검출 자체는 여기 없다 —
//! 여긴 "무슨 색인가"의 SSOT일 뿐이다.

mod cam;
mod color_space;
mod params;
mod parse_color_space_error;
mod set;

pub use cam::{ColormaskBgr, ColormaskCam};
pub use color_space::ColorSpace;
pub use params::ColormaskParams;
pub use parse_color_space_error::ParseColorSpaceError;
pub use set::{ColormaskSet, load_colormask_set, load_colormask_set_or_empty, save_colormask_set};
