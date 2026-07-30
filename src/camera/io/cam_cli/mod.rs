//! 카메라 CLI — clap 스키마·캡처 오케스트레이션.
//!
//! 앱 프리셋 [`Default`]는 [`crate::defaults::calib`]에 있다.

mod cam_cli_args;
mod cam_stream_args;
mod mono_offline_args;
mod resolved_cam;
mod stereo_cam_cli_args;
mod stereo_offline_args;
mod stereo_pair_cli_args;
mod stream_preset;

pub use cam_cli_args::CamCliArgs;
pub use cam_stream_args::CamStreamArgs;
pub use mono_offline_args::MonoOfflineArgs;
pub use resolved_cam::ResolvedCam;
pub use stereo_cam_cli_args::StereoCamCliArgs;
pub use stereo_offline_args::StereoOfflineArgs;
pub use stereo_pair_cli_args::StereoPairCliArgs;
pub use stream_preset::StreamPreset;

pub(crate) fn parse_fourcc(value: &str) -> Result<[u8; 4], String> {
    let bytes = value.as_bytes();
    if bytes.len() != 4 {
        return Err(format!("FOURCC는 정확히 4글자여야 함: {value}"));
    }
    return Ok([bytes[0], bytes[1], bytes[2], bytes[3]]);
}
