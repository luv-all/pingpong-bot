//! 평가 프로토콜 실행·공개 API.

use std::sync::{Arc, Mutex};

use rand::Rng;

use crate::defaults::PhysicsParams;
use crate::robot::Robot;
use crate::shooter;
use crate::sim::physics::world::SimWorld;

use super::{
    EVAL_NET_PASSTHROUGH_RETRIES, EVAL_PITCH_JITTER_DEG, EVAL_SPEED_JITTER_MPS,
    EVAL_YAW_JITTER_DEG, Flags, LaunchParams, LiveObserver, Mode, Progress, Report, SHOTS_PER_ZONE,
    Shot, TOTAL_SHOTS, Zone, ZoneScore,
};

/// `(zone, index_in_zone)` 30발 스케줄.
pub(crate) fn shot_schedule(mode: Mode) -> Vec<(Zone, usize)> {
    match mode {
        Mode::Block => {
            let mut out = Vec::with_capacity(TOTAL_SHOTS);
            for zone in Zone::BLOCK_ORDER {
                for index_in_zone in 0..SHOTS_PER_ZONE {
                    out.push((zone, index_in_zone));
                }
            }
            return out;
        }
        Mode::Alternating => {
            // 왼→중앙→오→중앙→왼→… 존당 10발이 찰 때까지.
            let pattern = [Zone::Left, Zone::Center, Zone::Right, Zone::Center];
            let mut zone_counts = [0_usize; 3];
            let mut out = Vec::with_capacity(TOTAL_SHOTS);
            let mut cursor = 0_usize;
            while out.len() < TOTAL_SHOTS {
                let zone = pattern[cursor % pattern.len()];
                cursor += 1;
                let zi = zone.zone_index();
                if zone_counts[zi] >= SHOTS_PER_ZONE {
                    continue;
                }
                let index_in_zone = zone_counts[zi];
                zone_counts[zi] += 1;
                out.push((zone, index_in_zone));
            }
            return out;
        }
    }
}
/// Eval 발사 파라미터를 존 템플릿으로 바꾼다 (지터 없음).
///
/// 스핀/롤=0. 좌·우는 `side_yaw_deg` 대칭, 중앙은 yaw=0.
/// pitch/height는 슈터 기본값에서 시작해 네트 게이트만 맞춘다.
pub(crate) fn settings_for_zone(launch: &LaunchParams, zone: Zone) -> shooter::Settings {
    return settings_for_zone_shot(launch, zone, 0);
}

/// 존 안 n번째 샷 (지터 없음). `index_in_zone`은 스케줄 호환용으로 유지.
pub(crate) fn settings_for_zone_shot(
    launch: &LaunchParams,
    zone: Zone,
    index_in_zone: usize,
) -> shooter::Settings {
    let _ = index_in_zone;
    return build_zone_shot::<rand::rngs::StdRng>(launch, zone, None);
}

/// 존 샷 + 미약 지터 (speed / yaw / pitch).
pub(crate) fn settings_for_zone_shot_jittered<R: Rng + ?Sized>(
    launch: &LaunchParams,
    zone: Zone,
    index_in_zone: usize,
    rng: &mut R,
) -> shooter::Settings {
    let _ = index_in_zone;
    return build_zone_shot(launch, zone, Some(rng));
}

fn build_zone_shot<R: Rng + ?Sized>(
    launch: &LaunchParams,
    zone: Zone,
    mut rng: Option<&mut R>,
) -> shooter::Settings {
    let mut shot = shooter::Settings::default();
    shot.roll_deg = 0.0;
    shot.topspin_rad_s = 0.0;
    shot.sidespin_rad_s = 0.0;
    shot.drill_spin_rad_s = 0.0;
    shot.lateral_offset_m = 0.0;

    shot.speed_mps = launch.speed_mps.max(0.1);
    if let Some(r) = rng.as_mut() {
        shot.speed_mps =
            (shot.speed_mps + r.gen_range(-EVAL_SPEED_JITTER_MPS..=EVAL_SPEED_JITTER_MPS)).max(0.1);
    }

    let mut yaw = zone.yaw_deg(launch.side_yaw_deg);
    if zone != Zone::Center
        && let Some(r) = rng.as_mut()
    {
        yaw += r.gen_range(-EVAL_YAW_JITTER_DEG..=EVAL_YAW_JITTER_DEG);
    }
    shot.yaw_deg = yaw;

    // ballistics + Rapier 네트 비접촉까지 pitch를 올린다.
    lift_pitch_for_net_gate(&mut shot);

    if let Some(r) = rng.as_mut() {
        let pitch_before = shot.pitch_deg;
        shot.pitch_deg += r.gen_range(-EVAL_PITCH_JITTER_DEG..=EVAL_PITCH_JITTER_DEG);
        if !shot.clears_incoming_net_gate() || !shot.clears_incoming_rapier_net() {
            shot.pitch_deg = pitch_before;
        }
    }
    return shot;
}

fn lift_pitch_for_net_gate(shot: &mut shooter::Settings) {
    if shot.clears_incoming_net_gate() && shot.clears_incoming_rapier_net() {
        return;
    }
    let base_pitch = shot.pitch_deg;
    // 5 m/s 대역은 낙하가 커서 yaw가 있으면 +4° 근처까지 필요할 수 있다.
    for lift in [
        0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0, 5.5, 6.0, 7.0, 8.0, 9.0, 10.0,
    ] {
        shot.pitch_deg = base_pitch + lift;
        if shot.clears_incoming_net_gate() && shot.clears_incoming_rapier_net() {
            return;
        }
    }
}

/// 헤드리스 한 발 — Rapier로 끝까지 돌리고 플래그·네트 투과 여부를 반환한다.
pub(crate) fn run_eval_shot(
    robot: &Robot,
    physics: PhysicsParams,
    settings: &shooter::Settings,
) -> (Flags, bool) {
    const MAX_STEPS: usize = 4_000;
    const DT: f64 = 1.0 / 1000.0;

    let mut world = SimWorld::with_physics(robot.clone(), physics);
    world.set_use_ground_truth(true);
    world.shoot_ball(settings);

    let mut observer = LiveObserver::new(&world);
    for _ in 0..MAX_STEPS {
        world.step(DT, None);
        if observer.observe(&world) {
            break;
        }
    }
    return (observer.flags, observer.net_passthrough);
}
/// 30발 프로토콜 실행. `progress`가 있으면 매 발 후 갱신.
pub(crate) fn run_eval_protocol(
    robot: &Robot,
    physics: PhysicsParams,
    launch: &LaunchParams,
    mode: Mode,
    progress: Option<Arc<Mutex<Progress>>>,
) -> Report {
    let mut shots = Vec::with_capacity(TOTAL_SHOTS);
    let mut by_zone = [ZoneScore::default(); 3];
    let mut counts = [0_u32; 4];
    let mut total = 0_u32;
    let mut done = 0_usize;

    let mut rng = rand::thread_rng();
    for (zone, index_in_zone) in shot_schedule(mode) {
        let mut settings = settings_for_zone_shot_jittered(launch, zone, index_in_zone, &mut rng);
        let mut flags = Flags::default();
        let mut accepted = false;
        for attempt in 0..=EVAL_NET_PASSTHROUGH_RETRIES {
            let (shot_flags, passthrough) = run_eval_shot(robot, physics, &settings);
            if !passthrough {
                flags = shot_flags;
                accepted = true;
                break;
            }
            tracing::warn!(
                zone = zone.label(),
                index_in_zone,
                attempt,
                "eval: 네트 투과(물리 이상) — 다른 상태로 재시도"
            );
            if attempt < EVAL_NET_PASSTHROUGH_RETRIES {
                settings = settings_for_zone_shot_jittered(launch, zone, index_in_zone, &mut rng);
            }
        }
        if !accepted {
            tracing::error!(
                zone = zone.label(),
                index_in_zone,
                retries = EVAL_NET_PASSTHROUGH_RETRIES,
                "eval: 네트 투과 재시도 소진 — 0점으로 기록"
            );
            flags = Flags::default();
        }
        let points = flags.score();
        total += u32::from(points);
        counts[points as usize] += 1;
        let zi = zone.zone_index();
        by_zone[zi].total += u32::from(points);
        by_zone[zi].counts[points as usize] += 1;
        shots.push(Shot {
            zone,
            index_in_zone,
            flags,
            points,
            settings,
        });
        done += 1;
        if let Some(p) = &progress {
            let mut g = p.lock().expect("eval progress");
            g.done = done;
        }
    }

    let report = Report {
        mode,
        shots,
        by_zone,
        total,
        counts,
    };
    if let Some(p) = &progress {
        let mut g = p.lock().expect("eval progress");
        g.done = TOTAL_SHOTS;
        g.report = Some(report.clone());
    }
    return report;
}

/// 평가 프로토콜 공개 진입점.
pub struct Protocol;

impl Protocol {
    pub fn shot_schedule(mode: Mode) -> Vec<(Zone, usize)> {
        return shot_schedule(mode);
    }

    pub fn settings_for_zone(launch: &LaunchParams, zone: Zone) -> shooter::Settings {
        return settings_for_zone(launch, zone);
    }

    pub fn settings_for_zone_shot(
        launch: &LaunchParams,
        zone: Zone,
        index_in_zone: usize,
    ) -> shooter::Settings {
        return settings_for_zone_shot(launch, zone, index_in_zone);
    }

    pub fn settings_for_zone_shot_jittered<R: Rng + ?Sized>(
        launch: &LaunchParams,
        zone: Zone,
        index_in_zone: usize,
        rng: &mut R,
    ) -> shooter::Settings {
        return settings_for_zone_shot_jittered(launch, zone, index_in_zone, rng);
    }

    pub fn run_shot(
        robot: &Robot,
        physics: PhysicsParams,
        settings: &shooter::Settings,
    ) -> (Flags, bool) {
        return run_eval_shot(robot, physics, settings);
    }

    pub fn run(
        robot: &Robot,
        physics: PhysicsParams,
        launch: &LaunchParams,
        mode: Mode,
        progress: Option<Arc<Mutex<Progress>>>,
    ) -> Report {
        return run_eval_protocol(robot, physics, launch, mode, progress);
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    use crate::eval::live_observer::net_plane_passthrough;

    #[test]
    fn net_plane_passthrough_detects_low_crossing() {
        let net_y = 1.37_f32;
        let top = 0.93_f32;
        assert!(net_plane_passthrough(1.40, 1.30, 0.80, net_y, top));
        assert!(net_plane_passthrough(1.30, 1.40, 0.80, net_y, top));
        assert!(!net_plane_passthrough(1.40, 1.30, 0.95, net_y, top));
        assert!(!net_plane_passthrough(1.50, 1.45, 0.80, net_y, top));
    }

    #[test]
    fn score_rubric_matches_july31() {
        assert_eq!(Flags::default().score(), 0);
        assert_eq!(
            Flags {
                contact: true,
                ..Default::default()
            }
            .score(),
            1
        );
        assert_eq!(
            Flags {
                contact: true,
                cleared_net: true,
                ..Default::default()
            }
            .score(),
            2
        );
        assert_eq!(
            Flags {
                contact: true,
                cleared_net: true,
                returned_in: true,
                ..Default::default()
            }
            .score(),
            3
        );
    }

    /// 자기 코트에 바운스한 뒤 넘어간 리턴은 탁구 규칙상 반칙 — 3점 조건을
    /// 다 채웠어도 접촉만 인정해 1점이다.
    #[test]
    fn own_half_bounce_is_a_foul_worth_one_point() {
        let flags = Flags {
            contact: true,
            cleared_net: true,
            returned_in: true,
            bounced_own_half: true,
            double_hit: false,
        };
        assert_eq!(flags.score(), 1, "자기 코트 바운스는 1점: {flags:?}");
        assert!(flags.is_foul());
    }

    /// 더블히트도 같은 취급.
    #[test]
    fn double_hit_is_a_foul_worth_one_point() {
        let flags = Flags {
            contact: true,
            cleared_net: true,
            returned_in: true,
            bounced_own_half: false,
            double_hit: true,
        };
        assert_eq!(flags.score(), 1, "더블히트는 1점: {flags:?}");
        assert!(flags.is_foul());
    }

    /// 반칙이라도 접촉 자체가 없으면 0점이다 (강등이 0점을 1점으로 올리지 않는다).
    #[test]
    fn foul_without_contact_is_still_zero() {
        let flags = Flags {
            bounced_own_half: true,
            ..Default::default()
        };
        assert_eq!(flags.score(), 0);
    }

    #[test]
    fn pass_requires_more_than_45() {
        let mut report = Report {
            mode: Mode::Block,
            shots: vec![],
            by_zone: [ZoneScore::default(); 3],
            total: 45,
            counts: [0; 4],
        };
        assert!(!report.passed());
        report.total = 46;
        assert!(report.passed());
    }

    #[test]
    fn schedules_have_ten_per_zone() {
        for mode in [Mode::Block, Mode::Alternating] {
            let sched = shot_schedule(mode);
            assert_eq!(sched.len(), TOTAL_SHOTS);
            let mut counts = [0_usize; 3];
            for (zone, idx) in &sched {
                counts[zone.zone_index()] += 1;
                assert!(*idx < SHOTS_PER_ZONE);
            }
            assert_eq!(counts, [10, 10, 10], "{mode:?}");
        }
    }

    #[test]
    fn alternating_starts_left_center_right_center() {
        let sched = shot_schedule(Mode::Alternating);
        assert_eq!(
            sched[..4].iter().map(|(z, _)| *z).collect::<Vec<_>>(),
            vec![Zone::Left, Zone::Center, Zone::Right, Zone::Center]
        );
    }

    #[test]
    fn block_is_left_then_center_then_right() {
        let sched = shot_schedule(Mode::Block);
        assert!(sched[..10].iter().all(|(z, _)| *z == Zone::Left));
        assert!(sched[10..20].iter().all(|(z, _)| *z == Zone::Center));
        assert!(sched[20..].iter().all(|(z, _)| *z == Zone::Right));
    }

    #[test]
    fn zone_shot_jitter_moves_speed_yaw_pitch_but_keeps_zone_side() {
        use rand::SeedableRng;

        let launch = LaunchParams::default();
        let clean = settings_for_zone_shot(&launch, Zone::Left, 3);
        let mut rng = rand::rngs::StdRng::seed_from_u64(7);
        let jittered = settings_for_zone_shot_jittered(&launch, Zone::Left, 3, &mut rng);

        assert!(clean.yaw_deg < 0.0, "left zone yaw negative");
        assert!(jittered.yaw_deg < 0.0, "left zone stays leftish");
        assert!((jittered.speed_mps - clean.speed_mps).abs() <= EVAL_SPEED_JITTER_MPS + 1e-12);
        assert!((jittered.pitch_deg - clean.pitch_deg).abs() <= EVAL_PITCH_JITTER_DEG + 1e-12);
        assert!((jittered.yaw_deg - clean.yaw_deg).abs() <= EVAL_YAW_JITTER_DEG + 1e-12);
        assert!(
            (jittered.yaw_deg - clean.yaw_deg).abs() > 1e-9
                || (jittered.speed_mps - clean.speed_mps).abs() > 1e-9
                || (jittered.pitch_deg - clean.pitch_deg).abs() > 1e-9,
            "expected non-zero jitter"
        );
    }

    #[test]
    fn side_yaw_is_symmetric() {
        let launch = LaunchParams {
            speed_mps: 7.0,
            side_yaw_deg: 12.0,
        };
        let left = settings_for_zone(&launch, Zone::Left);
        let right = settings_for_zone(&launch, Zone::Right);
        let center = settings_for_zone(&launch, Zone::Center);
        assert!((left.yaw_deg + right.yaw_deg).abs() < 1e-12);
        assert!((center.yaw_deg).abs() < 1e-12);
        assert!((left.speed_mps - 7.0).abs() < 1e-12);
    }
}

#[cfg(test)]
mod smoke {
    use super::*;
    use crate::defaults;
    use crate::eval;

    #[test]
    fn protocol_runs_and_prints_score() {
        let robot = defaults::robot().expect("robot");
        let report = run_eval_protocol(
            &robot,
            defaults::PhysicsParams::default(),
            &LaunchParams::default(),
            Mode::Block,
            None,
        );
        eprintln!(
            "EVAL total={}/{} pass={} counts={:?} zones={:?}",
            report.total,
            eval::MAX_SCORE,
            report.passed(),
            report.counts,
            report.by_zone.map(|z| z.total),
        );
        assert_eq!(report.shots.len(), TOTAL_SHOTS);
        // 존 조준이 맞으면 전부 0점이면 안 된다 (네트 미달 버그 회귀).
        assert!(
            report.total > 0,
            "expected some points from zone-aimed protocol"
        );
    }
}
