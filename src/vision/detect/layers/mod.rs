//! [`Layer`](super::Layer) 구현들. 조립 순서는 [`super::Detector`]가 정한다.

pub mod background;
mod color;
mod color_box;

pub use background::Background;
pub use color::ColorPlane;
pub use color_box::ColorBox;
