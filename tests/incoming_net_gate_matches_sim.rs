//! 수신 네트 게이트(`clears_incoming_rapier_net`) ↔ 본 시뮬 정합.
//!
//! 게이트는 테이블·네트·공만 있는 가벼운 Rapier로 "수신 탄도가 네트에 닿지
//! 않는가"를 판정한다. 이 판정이 `SimWorld` 실제 거동과 어긋나면 eval 은
//! 네트에 걸리는 서브를 로봇에게 먹이고, 로봇은 예측 높이에서 헛스윙한다
//! (예측 z≈1.05 vs 실제 도착 z=0.78).

use pingpong_bot::constants::table;
use pingpong_bot::defaults;
use pingpong_bot::sim::eval;
use pingpong_bot::sim::launch;
use pingpong_bot::sim::physics;
use pingpong_bot::sim::physics::SimWorld;

/// 공이 라켓에 닿아 있는가.
fn ball_touches_racket(world: &SimWorld) -> bool {
    let Some(ball) = world
        .collider_set
        .iter()
        .find_map(|(h, c)| (c.parent() == Some(world.ball_handle)).then_some(h))
    else {
        return false;
    };
    let Some(racket) = world
        .collider_set
        .iter()
        .find_map(|(h, c)| (c.parent() == Some(world.racket_handle)).then_some(h))
    else {
        return false;
    };
    return world
        .narrow_phase
        .contact_pair(ball, racket)
        .is_some_and(|p| p.has_any_active_contact());
}

/// 본 시뮬에서 **수신** 공이 네트에 접촉하는가.
///
/// 라켓이 한 번 닿은 뒤는 모두 "리턴"이라 관심 밖이다. 약한 리턴이 자기
/// 코트에 튀어 네트를 맞는 경우가 있어, 속도 부호만으로는 갈라지지 않는다.
fn incoming_touches_net_in_sim(settings: &launch::Settings) -> bool {
    const DT: f64 = 1.0 / 1000.0;
    let mut world = SimWorld::with_physics(
        defaults::robot().expect("robot"),
        defaults::PhysicsParams::default(),
    );
    world.set_use_ground_truth(true);
    world.shoot_ball(settings);

    let mut previous_y = f64::from(world.ball_position().y);
    for _ in 0..4_000 {
        world.step(DT, None);
        if ball_touches_racket(&world) {
            return false;
        }
        if world.ball_intersects_net() {
            return true;
        }
        let y = f64::from(world.ball_position().y);
        // 히트 평면을 지나면 그 뒤 네트 접촉은 "리턴" 쪽이라 관심 밖.
        if previous_y > table::DEFAULT_HIT_PLANE_Y && y <= table::DEFAULT_HIT_PLANE_Y {
            return false;
        }
        previous_y = y;
        if world.ball_state == physics::BallState::Parked {
            return false;
        }
    }
    return false;
}

/// 본 시뮬에서 수신 공이 네트 평면을 지날 때의 상단 여유고(m).
fn incoming_net_clearance_in_sim(settings: &launch::Settings) -> Option<f64> {
    const DT: f64 = 1.0 / 1000.0;
    let net_y = table::LENGTH_Y * 0.5;
    let net_top = table::SURFACE_Z + table::NET_HEIGHT;
    let mut world = SimWorld::with_physics(
        defaults::robot().expect("robot"),
        defaults::PhysicsParams::default(),
    );
    world.set_use_ground_truth(true);
    world.shoot_ball(settings);

    let mut previous = (
        f64::from(world.ball_position().y),
        f64::from(world.ball_position().z),
    );
    for _ in 0..4_000 {
        world.step(DT, None);
        let y = f64::from(world.ball_position().y);
        let z = f64::from(world.ball_position().z);
        if previous.0 > net_y && y <= net_y {
            let frac = (previous.0 - net_y) / (previous.0 - y);
            let z_at = previous.1 + (z - previous.1) * frac;
            return Some(z_at - pingpong_bot::constants::ball::RADIUS - net_top);
        }
        previous = (y, z);
    }
    return None;
}

#[test]
#[ignore = "진단 전용"]
fn diag_default_shot() {
    let mut shot = launch::Settings::default();
    for lift in [0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0] {
        shot.pitch_deg = launch::Settings::default().pitch_deg + lift;
        println!(
            "pitch={:+.2} gate_clear={:<5} sim_touch={:<5} clearance={}",
            shot.pitch_deg,
            shot.clears_incoming_rapier_net(),
            incoming_touches_net_in_sim(&shot),
            incoming_net_clearance_in_sim(&shot)
                .map(|c| format!("{:+.4} m", c))
                .unwrap_or_else(|| "-".into()),
        );
    }
}

#[test]
#[ignore = "진단 전용"]
fn diag_net_clearance() {
    let launch = pingpong_bot::sim::eval::LaunchParams::default();
    for (i, (zone, index_in_zone)) in eval::Protocol::shot_schedule(eval::Mode::Block)
        .into_iter()
        .enumerate()
    {
        let settings = eval::Protocol::settings_for_zone_shot(&launch, zone, index_in_zone);
        println!(
            "#{:<3} {:<6} pitch={:+.2} gate_clear={:<5} sim_touch={:<5} clearance={}",
            i + 1,
            zone.label(),
            settings.pitch_deg,
            settings.clears_incoming_rapier_net(),
            incoming_touches_net_in_sim(&settings),
            incoming_net_clearance_in_sim(&settings)
                .map(|c| format!("{:+.4} m", c))
                .unwrap_or_else(|| "-".into()),
        );
    }
}

/// 게이트가 통과시킨 샷은 본 시뮬에서도 네트에 닿지 않아야 한다.
#[test]
fn incoming_net_gate_agrees_with_sim() {
    let launch = pingpong_bot::sim::eval::LaunchParams::default();
    let mut disagreements = Vec::new();

    for (i, (zone, index_in_zone)) in eval::Protocol::shot_schedule(eval::Mode::Block)
        .into_iter()
        .enumerate()
    {
        let settings = eval::Protocol::settings_for_zone_shot(&launch, zone, index_in_zone);
        let gate_says_clear = settings.clears_incoming_rapier_net();
        let sim_touches_net = incoming_touches_net_in_sim(&settings);
        if gate_says_clear && sim_touches_net {
            disagreements.push(format!(
                "#{:<3} {:<6} yaw={:+.2} pitch={:+.2} — 게이트는 통과라는데 시뮬에서 네트 접촉",
                i + 1,
                zone.label(),
                settings.yaw_deg,
                settings.pitch_deg,
            ));
        }
    }

    assert!(
        disagreements.is_empty(),
        "게이트가 네트에 걸리는 서브를 통과시킴 ({}/30):\n{}",
        disagreements.len(),
        disagreements.join("\n")
    );
}
