//! 리니어 레일 — 설정 + AXL 드라이버.

mod axl;
#[cfg(all(windows, feature = "real"))]
#[allow(dead_code)]
mod axl_ffi;
mod config;

pub use axl::AxlRail;
pub use config::{RailConfig, RailConfigError, SoftLimitArgs};
