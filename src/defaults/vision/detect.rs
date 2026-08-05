//! 검출 캐스케이드 튜너블. 쓰는 곳은 [`crate::vision::detect`].

/// 배경 모델이 기억하는 프레임 수. 정상 상태 학습률은 `1/HISTORY`다.
pub const BACKGROUND_HISTORY: i32 = 500;
/// 마할라노비스 제곱 임계 — OpenCV 기본값.
pub const BACKGROUND_VAR_THRESHOLD: f64 = 16.0;
/// 배경 모델을 돌릴 배율. 공 반지름이 3~18 px라 절반에서도 남는다.
pub const BACKGROUND_SCALE: f64 = 0.5;
/// 음수는 자동. MOG2 구현은 `1 / min(2·nframes, BACKGROUND_HISTORY)`를 쓴다. 초반이 커서
/// 모델이 빨리 서고, `2·nframes ≥ BACKGROUND_HISTORY`부터 `1/BACKGROUND_HISTORY`로 고정된다.
pub const BACKGROUND_LEARNING_RATE: f64 = -1.0;

/// 원형도 하한. 순위에 안 쓰고 걸러내기만 하므로 느슨하게 잡는다.
///
/// 완벽한 원이어도 래스터화만으로 떨어진다 — 실측 r=3 px 에서 0.67, r=20 px 에서 0.87
/// (`pick_tests.rs`). 모션 블러가 여기서 더 깎는다.
pub const MIN_CIRCULARITY: f64 = 0.35;
