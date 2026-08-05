use super::*;
use crate::constants::table;

#[test]
fn three_views_recover_a_known_point() {
    let calibration = Calibration::sim(3);
    let truth = Point3::new(
        table::WIDTH_X * 0.5,
        table::LENGTH_Y * 0.4,
        table::SURFACE_Z + 0.2,
    );
    let ids = [camera::Id::new(0), camera::Id::new(1), camera::Id::new(2)];
    let recovered = Triangulate::projections(&calibration, &ids, truth).expect("DLT");
    let error = (recovered.coords - truth.coords).norm();
    assert!(error < 1e-3, "noise-free 오차 {error} m");
}

#[test]
fn two_views_recover_a_known_point() {
    let calibration = Calibration::sim(3);
    let truth = Point3::new(0.6, 1.0, 0.9);
    let ids = [camera::Id::new(0), camera::Id::new(2)];
    let recovered = Triangulate::projections(&calibration, &ids, truth).expect("2-view");
    assert!((recovered.coords - truth.coords).norm() < 1e-3);
}

#[test]
fn a_single_view_cannot_triangulate() {
    let calibration = Calibration::sim(3);
    assert!(
        Triangulate::pixels(
            &[(camera::Id::new(0), camera::Pixel::new(1.0, 1.0))],
            &calibration
        )
        .is_none()
    );
}

/// 동차 좌표 `w`가 0에 붙으면 무한대가 나온다. 나누기 전에 걸러야 한다.
#[test]
fn a_degenerate_homogeneous_coordinate_is_rejected() {
    assert!(dehomogenise(1.0, 2.0, 3.0, 0.0).is_none());
    assert!(dehomogenise(1.0, 2.0, 3.0, f64::NAN).is_none());
    assert!(dehomogenise(2.0, 4.0, 6.0, 2.0).is_some());
}
