//! 본 바이너리 CLI 인자 (라이브러리 아님 — `main.rs` 전용).

pub mod args;
pub mod mode_arg;

pub use args::Args;
pub use mode_arg::ModeArg;
