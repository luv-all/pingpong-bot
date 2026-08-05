//! [`super`] 단위 테스트.

use super::*;

fn state(t_ms: u64, y: f64) -> State {
    return State {
        t: Duration::from_millis(t_ms),
        position: Point3::new(0.5, y, 1.0),
        velocity: Vector3::new(0.0, -1.0, 0.0),
        sigma_position: Vector3::repeat(0.02),
        sigma_velocity: Vector3::repeat(0.3),
        spin: None,
    };
}

/// y가 1 m/s로 줄어드는 직선, 100 ms 간격.
fn straight() -> Track {
    return Track(
        (0..=10u64)
            .map(|i| state(i * 100, 1.0 - i as f64 * 0.1))
            .collect(),
    );
}

#[test]
fn at_time_interpolates_between_samples() {
    let mid = straight()
        .at_time(Duration::from_millis(150))
        .expect("궤적 안");
    assert!((mid.position.y - 0.85).abs() < 1e-9, "y={}", mid.position.y);
}

#[test]
fn at_time_refuses_outside_the_track() {
    assert!(straight().at_time(Duration::from_millis(1500)).is_none());
    assert!(Track::default().at_time(Duration::ZERO).is_none());
}

#[test]
fn at_plane_finds_the_crossing() {
    let hit = straight().at_plane(0.55).expect("평면을 지난다");
    assert!((hit.position.y - 0.55).abs() < 1e-9, "y={}", hit.position.y);
    assert!((hit.velocity.y + 1.0).abs() < 1e-9);
}

#[test]
fn at_plane_is_none_when_never_crossed() {
    assert!(straight().at_plane(-5.0).is_none());
}

#[test]
fn interpolation_mixes_sigma_too() {
    let mut track = straight();
    track.0[1].sigma_position = Vector3::repeat(0.10);
    let mid = track.at_time(Duration::from_millis(50)).expect("궤적 안");
    assert!(mid.sigma_position.x > 0.02 && mid.sigma_position.x < 0.10);
}

/// 같은 질의가 관측 쪽에도 그대로 돈다.
#[test]
fn both_tracks_answer_the_same_queries() {
    let trajectory = Trajectory {
        seq: 0,
        origin: Instant::now(),
        measured: straight(),
        predicted: straight(),
    };
    assert_eq!(
        trajectory.measured.at_plane(0.55),
        trajectory.predicted.at_plane(0.55)
    );
    // Deref 로 슬라이스 API 도 그대로 쓴다.
    assert_eq!(trajectory.measured.len(), 11);
    assert!(trajectory.measured.last().is_some());
}
