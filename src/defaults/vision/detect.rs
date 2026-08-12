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
///
/// 2026-08-13 실측: 0.4·0.5 다 시도해 봄(서브 토스가 `FloorEdgeMask` Y컷에 잘리는 문제
/// 완화용 — 배경 차분이 못 거르는 피부색을 원형도로 대신 거르자는 생각이었다). 세
/// 값(0.4/0.5/0.5+Y여유) 다 fly_45~53 0.2s 리드 오차 중앙값이 1.3→2.4~3.1cm로
/// 악화됐다 — 실제 공 검출이 이미 0.35 문턱 가까이서 도는 클립이 있어서(모션블러·
/// 원거리), 문턱을 올리면 팔·몸통보다 진짜 공을 더 많이 잃는다. 되돌림 — 이 문턱으로
/// 저 문제를 풀 수는 없다.
pub const MIN_CIRCULARITY: f64 = 0.35;
