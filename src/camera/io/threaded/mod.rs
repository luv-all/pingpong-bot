//! 백그라운드 grab + 최신 프레임 (hinguri Camera::update 알맹이).

mod latest_slot;
mod threaded_capture;

pub use threaded_capture::ThreadedCapture;
