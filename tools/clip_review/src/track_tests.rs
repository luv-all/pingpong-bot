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

fn reviewed_with(predicted: Track, measured: Track, fps: f64) -> Reviewed {
    return Reviewed {
        frames: vec![FrameState::default(); 11],
        observed: Vec::new(),
        contract: Some(Contract {
            frame: 0,
            t: 0.0,
            at_trigger: Trajectory {
                seq: 0,
                origin: Instant::now(),
                measured: measured.clone(),
                predicted: predicted.clone(),
            },
            latest: Trajectory {
                seq: 0,
                origin: Instant::now(),
                measured,
                predicted,
            },
        }),
        fps,
    };
}

/// 재생 위치까지만 잘라야 한다. measured 는 클립 끝까지 자란 상태로 들고 있으므로
/// 안 자르면 아직 안 온 구간이 화면에 미리 보인다.
#[test]
fn the_measured_track_is_cut_at_the_playback_frame() {
    // 0.1 s 간격 11개, fps 10 이면 프레임 i 의 시각이 곧 i 번째 표본의 t 다.
    let reviewed = reviewed_with(Track::default(), straight(), 10.0);
    assert_eq!(reviewed.measured_to(0).len(), 1);
    assert_eq!(reviewed.measured_to(4).len(), 5);
    assert_eq!(reviewed.measured_to(10).len(), 11);
}

/// 예측은 트리거에 얼렸으므로 재생 위치와 무관하다.
#[test]
fn the_predicted_track_does_not_move_with_playback() {
    let reviewed = reviewed_with(straight(), Track::default(), 10.0);
    assert_eq!(reviewed.predicted().len(), 11);
}

/// 계약이 없으면 그릴 것도 없다 — 빈 것과 "0 이라고 말하는 것"은 다르다.
#[test]
fn without_a_contract_there_is_nothing_to_draw() {
    let mut reviewed = reviewed_with(straight(), straight(), 10.0);
    reviewed.contract = None;
    assert!(reviewed.measured_to(5).is_empty());
    assert!(reviewed.predicted().is_empty());
}
