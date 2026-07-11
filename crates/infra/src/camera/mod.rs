//! 가상·합성 카메라 어댑터.

mod projection;
mod sim_camera;
mod synthetic;

pub use sim_camera::SimCamera;
pub use synthetic::SyntheticCamera;
