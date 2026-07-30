//! 런타임 파이프라인 오케스트레이션.
//!
//! 스레드·채널 오케스트레이션 (plan §4).

mod config;
mod error;
mod feed;
mod pipeline;
mod thread;

pub use config::PipelineConfig;
pub use error::PipelineError;
pub use feed::CameraFeed;
pub use pipeline::{Pipeline, run};
pub use thread::PipelineThread;
