use super::*;
use pingpong_bot::Vector3;

/// y가 1 m/s로 줄어드는 직선 궤적, 0.1 s 간격.
fn straight() -> Track {
    return Track(
        (0..=10)
            .map(|i| {
                let t = f64::from(i) * 0.1;
                State {
                    t: Duration::from_secs_f64(t),
                    position: Point3::new(0.0, 1.0 - t, 1.0),
                    velocity: Vector3::new(0.0, -1.0, 0.0),
                    ..State::default()
                }
            })
            .collect(),
    );
}

#[test]
fn convergence_error_measures_the_gap_to_truth() {
    // 실제는 예측보다 5 cm 옆으로 갔다.
    let observed = vec![Observed {
        frame: 20,
        t: 0.2,
        point: Point3::new(0.05, 0.8, 1.0),
        reprojection_px: 1.0,
    }];
    let error = convergence_error(&straight(), &observed, 0.0, 0.2, 100.0).expect("짝 성립");
    assert!((error - 0.05).abs() < 1e-9, "error={error}");
}

/// 짝이 없으면 **0이 아니라 없음**이어야 한다 — 못 잰 걸 잘 맞춘 걸로 읽으면 안 된다.
#[test]
fn convergence_error_is_none_without_a_matching_observation() {
    let observed = vec![Observed {
        frame: 90,
        t: 0.9,
        point: Point3::new(0.0, 0.1, 1.0),
        reprojection_px: 1.0,
    }];
    assert!(convergence_error(&straight(), &observed, 0.0, 0.2, 100.0).is_none());
}

fn at_y(y: f64) -> Point3 {
    return Point3::new(0.5, y, 1.0);
}

#[test]
fn clip_keeps_a_track_that_never_reaches_the_mount() {
    let limit = draw_limit_y();
    let track = vec![at_y(limit + 1.0), at_y(limit + 0.5), at_y(limit + 0.2)];
    assert_eq!(clip_to_mount(&track).len(), 3);
}

#[test]
fn clip_ends_exactly_at_the_mount() {
    let limit = draw_limit_y();
    let track = vec![at_y(limit + 0.4), at_y(limit + 0.2), at_y(limit - 0.3)];
    let clipped = clip_to_mount(&track);
    let last = clipped.last().expect("경계점이 남는다");
    assert!((last.y - limit).abs() < 1e-9, "y={} != {limit}", last.y);
    // 마운트 뒤 점은 하나도 안 남는다.
    assert!(clipped.iter().all(|p| p.y >= limit - 1e-9));
}

#[test]
fn clip_drops_a_track_entirely_behind_the_mount() {
    let limit = draw_limit_y();
    let track = vec![at_y(limit - 0.1), at_y(limit - 0.5)];
    assert!(clip_to_mount(&track).len() < 2, "그릴 선분이 없어야 한다");
}

/// 평면 통과는 이제 계약이 준다. 이 툴이 쓰는 그대로 확인한다.
#[test]
fn the_track_reports_where_it_crosses_a_plane() {
    let track = straight();
    let hit = track.at_plane(0.5).expect("평면을 지난다");
    assert!((hit.position.y - 0.5).abs() < 1e-9, "y={}", hit.position.y);
    assert!((hit.velocity.y + 1.0).abs() < 1e-9);
    assert!(track.at_plane(-5.0).is_none(), "안 지나는 평면");
}
