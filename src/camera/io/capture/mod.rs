//! 프레임 소스 (sim 힌트 / OpenCV VideoCapture / 파일).

mod capture_backend;
mod exposure_readout;
mod frame;
mod frame_source;
mod hint_source;
mod image_dir_source;
mod open_cv_capture;

pub use capture_backend::CaptureBackend;
pub use exposure_readout::ExposureReadout;
pub use frame::Frame;
pub use frame_source::FrameSource;
pub use hint_source::HintSource;
pub use image_dir_source::ImageDirSource;
pub use open_cv_capture::OpenCvCapture;
