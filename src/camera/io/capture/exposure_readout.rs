/// [`super::OpenCvCapture::exposure_readout`] 결과.
#[derive(Debug, Clone)]
pub struct ExposureReadout {
    pub auto: Option<f64>,
    pub exposure: Option<f64>,
    pub gain: Option<f64>,
    pub backend: String,
}

impl ExposureReadout {
    pub fn summary_line(&self) -> String {
        let ae = self
            .auto
            .map(|v| format!("{v:.2}"))
            .unwrap_or_else(|| "-".into());
        let exp = self
            .exposure
            .map(|v| format!("{v:.2}"))
            .unwrap_or_else(|| "-".into());
        let gain = self
            .gain
            .map(|v| format!("{v:.1}"))
            .unwrap_or_else(|| "-".into());
        return format!("ae {ae} exp {exp} gain {gain}");
    }

    /// OpenCV macOS 백엔드는 width/height/fps 외 UVC 컨트롤을 거의 무시한다.
    pub fn likely_unsupported(&self) -> bool {
        let b = self.backend.to_ascii_lowercase();
        return b.contains("avfoundation") || b.contains("avf");
    }
}
