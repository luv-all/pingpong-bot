//! Sync / Apply / Discard 상태머신 + 조그 앱 상태.

mod action;
mod jog_app;
mod phase;

pub use action::Action;
pub use jog_app::{JogApp, try_action};
