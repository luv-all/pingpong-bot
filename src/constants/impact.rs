//! 임팩트/로프트 리턴 목표.

/// 네트 위 여유 높이 [m].
pub const NET_CLEARANCE: f64 = 0.08;

/// 임팩트에서 상대 코트 중앙 바운드까지 목표 비행 시간 [s].
pub const RALLY_TIME_TO_BOUNCE: f64 = 0.55;

/// 라켓 명령(필요 라켓속도) 역산에 쓰는 유효 반발계수.
///
/// 실측 물리 반발계수(ITTF 공-테이블 규격 0.89~0.92, `config/*.toml`
/// `[physics] restitution`=0.85)와 다르다 — 시도해봤다(2026-07-23,
/// swing-bench/bang-bang 수렴 조사 중). 0.85로 맞추면
/// `ground_truth_rally_contacts_racket_clears_net_and_bounces_near_center`가
/// 깨진다(bounce y=1.76 vs target 2.055 — 미달). 즉 실제 Rapier 충돌
/// (움직이는 kinematic 라켓 vs dynamic 공, 압축성 접촉·CCD·서브스텝)은
/// 단순 반발계수 e 공식대로 운동량을 전달하지 않아서, 이 값은 "라켓
/// 고무의 실제 반발계수"가 아니라 그 차이를 흡수하는 경험적 캘리브레이션
/// 상수다. `0.42`는 기존 리턴 정확도 테스트로 검증된 값이라 되돌린다 —
/// 바꾸려면 물리 문헌이 아니라 실제 Rapier 시뮬 결과로 재캘리브레이션
/// 해야 한다.
pub const RACKET_EFFECTIVE_RESTITUTION: f64 = 0.42;

/// 리턴 속도 상한 [m/s].
pub const MAX_RETURN_SPEED: f64 = 6.0;
