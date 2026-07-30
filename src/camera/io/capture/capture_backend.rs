use opencv::videoio;

/// OpenCV `VideoCapture` API 백엔드.
///
/// Windows에서 DSHOW는 MJPG 협상이 자주 실패하고 YUY2(~10fps)에 갇힌다.
/// 듀얼 UVC는 [`CaptureBackend::recommended`]가 **MSMF**를 고른다 (hinguri 실측 경로).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum CaptureBackend {
    /// `CAP_ANY` — OS 기본 선택.
    #[default]
    Any,
    /// DirectShow (Windows). MJPG 실패 시 YUY2로 떨어지기 쉬움.
    DShow,
    /// Media Foundation (Windows). 듀얼·고FPS에 유리.
    Msmf,
    /// V4L2 (Linux).
    V4l2,
    /// AVFoundation (macOS).
    AvFoundation,
}

impl CaptureBackend {
    /// 호스트에서 고FPS UVC에 안전한 기본값.
    pub fn recommended() -> Self {
        if cfg!(target_os = "windows") {
            return Self::Msmf;
        }
        return Self::Any;
    }

    pub fn parse(s: &str) -> Result<Self, String> {
        return match s.trim().to_ascii_lowercase().as_str() {
            "any" | "default" | "" => Ok(Self::Any),
            "dshow" | "directshow" => Ok(Self::DShow),
            "msmf" | "mediafoundation" => Ok(Self::Msmf),
            "v4l" | "v4l2" => Ok(Self::V4l2),
            "avfoundation" | "avf" => Ok(Self::AvFoundation),
            "recommended" | "auto" => Ok(Self::recommended()),
            other => Err(format!(
                "unknown capture backend '{other}' (any|dshow|msmf|v4l2|avfoundation|recommended)"
            )),
        };
    }

    pub fn as_str(self) -> &'static str {
        return match self {
            Self::Any => "any",
            Self::DShow => "dshow",
            Self::Msmf => "msmf",
            Self::V4l2 => "v4l2",
            Self::AvFoundation => "avfoundation",
        };
    }

    pub fn api_pref(self) -> i32 {
        return match self {
            Self::Any => videoio::CAP_ANY,
            Self::DShow => videoio::CAP_DSHOW,
            Self::Msmf => videoio::CAP_MSMF,
            Self::V4l2 => videoio::CAP_V4L2,
            Self::AvFoundation => videoio::CAP_AVFOUNDATION,
        };
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    #[cfg(target_os = "windows")]
    fn windows_recommended_is_msmf() {
        assert_eq!(CaptureBackend::recommended(), CaptureBackend::Msmf);
    }

    #[test]
    #[cfg(not(target_os = "windows"))]
    fn non_windows_recommended_is_any() {
        assert_eq!(CaptureBackend::recommended(), CaptureBackend::Any);
    }
}
