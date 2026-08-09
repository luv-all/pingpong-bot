//! YCrCb / HSV 색 마스크로 공 검출.

mod cam;
mod color_space;
mod detector;
mod params;
mod parse_color_space_error;
mod set;

pub use cam::{ColormaskBgr, ColormaskCam};
pub use color_space::ColorSpace;
pub use detector::ColormaskDetector;
pub use params::ColormaskParams;
pub use parse_color_space_error::ParseColorSpaceError;
pub use set::{ColormaskSet, load_colormask_set, load_colormask_set_or_empty, save_colormask_set};
