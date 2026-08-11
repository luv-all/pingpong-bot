//! [`Layer`](super::Layer) 구현들. 조립 순서는 [`super::Detector`]가 정한다.

pub mod background;
mod color;
mod color_box;
mod spatial;

pub use background::Background;
pub use color::ColorPlane;
pub use color_box::ColorBox;
pub use spatial::Spatial;
