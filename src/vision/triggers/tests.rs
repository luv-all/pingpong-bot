//! [`super`] 단위 테스트.

use std::time::Duration;

use super::*;
use crate::vision::contract::State;
use crate::vision::trigger::Trigger;
use crate::{Point3, Vector3};

fn state(y: f64, vy: f64, vz: f64, sigma_v: f64) -> State {
    return State {
        t: Duration::ZERO,
        position: Point3::new(0.5, y, 1.0),
        velocity: Vector3::new(0.0, vy, vz),
        sigma_position: Vector3::repeat(0.02),
        sigma_velocity: Vector3::repeat(sigma_v),
        spin: None,
    };
}

#[test]
fn plane_fires_only_toward_the_robot() {
    let t = PlaneCrossing { y: 1.0 };
    assert!(t.ready(&[state(0.9, -4.0, 0.0, 0.1)]), "넘었고 다가온다");
    assert!(!t.ready(&[state(1.1, -4.0, 0.0, 0.1)]), "아직 안 넘었다");
    assert!(!t.ready(&[state(0.9, 4.0, 0.0, 0.1)]), "넘었지만 멀어진다");
}

#[test]
fn sigma_needs_every_axis() {
    let t = SigmaThreshold {
        position: Vector3::repeat(0.05),
        velocity: Vector3::repeat(0.2),
    };
    assert!(t.ready(&[state(0.5, -4.0, 0.0, 0.1)]));

    // 한 축만 커도 안 된다 — 그게 스칼라로 재면 가려지는 경우다.
    let mut wide = state(0.5, -4.0, 0.0, 0.1);
    wide.sigma_velocity.x = 0.9;
    assert!(!t.ready(&[wide]));
}

#[test]
fn bounce_needs_a_sign_flip_in_history() {
    let t = FirstBounce;
    assert!(!t.ready(&[state(0.5, -4.0, -2.0, 0.1)]), "한 점으론 모른다");
    assert!(t.ready(&[state(0.6, -4.0, -2.0, 0.1), state(0.5, -4.0, 1.5, 0.1)]));
}

#[test]
fn empty_track_never_fires() {
    assert!(!PlaneCrossing { y: 1.0 }.ready(&[]));
    assert!(!FirstBounce.ready(&[]));
}

#[test]
fn any_fires_on_the_first_condition_met() {
    let late = state(1.1, -4.0, 0.0, 0.9); // 평면 전, σ도 넓다
    let combo = Any(vec![
        Box::new(SigmaThreshold {
            position: Vector3::repeat(0.05),
            velocity: Vector3::repeat(0.2),
        }),
        Box::new(PlaneCrossing { y: 1.0 }),
    ]);
    assert!(!combo.ready(&[late]));

    // σ가 좁아지면 평면 전이라도 발동한다.
    let mut tight = late;
    tight.sigma_velocity = Vector3::repeat(0.1);
    assert!(combo.ready(&[tight]));
}

#[test]
fn all_needs_every_condition() {
    let combo = All(vec![
        Box::new(PlaneCrossing { y: 1.0 }),
        Box::new(FirstBounce),
    ]);
    assert!(!combo.ready(&[state(0.9, -4.0, -2.0, 0.1)]), "바운스 전");
    assert!(combo.ready(&[state(1.0, -4.0, -2.0, 0.1), state(0.9, -4.0, 1.5, 0.1)]));
}

/// 빈 `All`은 참이 되면 안 된다 — 아무 조건도 안 걸었는데 즉시 발동한다.
#[test]
fn empty_all_does_not_fire() {
    assert!(!All(Vec::new()).ready(&[state(0.5, -4.0, 0.0, 0.1)]));
}
