//! 평가 프로토콜 — 좌/중/우 각 10발, 0~3점.

use std::sync::{Arc, Mutex};

use rand::Rng;

use crate::PhysicsParams;
use crate::constants::{BALL_RADIUS, table};
use crate::robot::Robot;
use crate::sim::physics::shooter::{BallShooterSettings, BallState};
use crate::sim::physics::world::SimWorld;
use rapier3d::prelude::RigidBodyHandle;

/// 속도 지터 [m/s].
const EVAL_SPEED_JITTER_MPS: f64 = 0.15;
/// 좌·우 yaw 지터 [deg].
const EVAL_YAW_JITTER_DEG: f64 = 0.5;
/// pitch 지터 [deg] — 네트 lift 이후 적용.
const EVAL_PITCH_JITTER_DEG: f64 = 0.5;
/// 네트 CCD 투과(물리 버그) 시 다른 지터 상태로 재시도하는 상한.
const EVAL_NET_PASSTHROUGH_RETRIES: usize = 12;

/// 존당 발사 수.
pub const SHOTS_PER_ZONE: usize = 10;
/// 전체 발사 수 (좌+중+우).
pub const TOTAL_SHOTS: usize = SHOTS_PER_ZONE * 3;
/// 만점.
pub const MAX_SCORE: u32 = (TOTAL_SHOTS * 3) as u32;
/// 통과: 45점을 **넘겨야** 함 (>45).
pub const PASS_SCORE_EXCLUSIVE: u32 = 45;

/// 평가 존 — 로봇이 테이블(+y)을 바라볼 때 기준.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EvalZone {
    /// 로봇 기준 오른쪽 (+x)
    Right,
    Center,
    Left,
}

/// 평가 발사 순서.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum EvalMode {
    /// 왼쪽 10 → 중앙 10 → 오른쪽 10.
    #[default]
    Block,
    /// 왼→중앙→오→중앙→왼→… (존당 10발될 때까지).
    Alternating,
}

impl EvalMode {
    pub fn label(self) -> &'static str {
        return match self {
            Self::Block => "Block (L×10→C×10→R×10)",
            Self::Alternating => "Alternating (L→C→R→C→…)",
        };
    }

    pub fn short_label(self) -> &'static str {
        return match self {
            Self::Block => "Block",
            Self::Alternating => "Alt",
        };
    }
}

impl EvalZone {
    pub const ALL: [Self; 3] = [Self::Right, Self::Center, Self::Left];

    /// 블록 모드 발사 순서: 왼 → 중 → 오.
    pub const BLOCK_ORDER: [Self; 3] = [Self::Left, Self::Center, Self::Right];

    pub fn label(self) -> &'static str {
        return match self {
            Self::Right => "Right",
            Self::Center => "Center",
            Self::Left => "Left",
        };
    }

    fn zone_index(self) -> usize {
        return match self {
            Self::Right => 0,
            Self::Center => 1,
            Self::Left => 2,
        };
    }

    /// 슈터 `lateral_offset_m` [m] — 존 표시·레거시용. 발사 yaw는 [`EvalLaunchParams`].
    pub fn lateral_m(self) -> f64 {
        return match self {
            Self::Right => 0.35,
            Self::Center => 0.0,
            Self::Left => -0.35,
        };
    }

    /// 좌·우 대칭 yaw [deg]. Right=+, Left=−, Center=0.
    pub fn yaw_deg(self, side_yaw_deg: f64) -> f64 {
        return match self {
            Self::Right => side_yaw_deg.abs(),
            Self::Left => -side_yaw_deg.abs(),
            Self::Center => 0.0,
        };
    }
}

/// Eval 전용 발사 설정 — Shooter 패널과 분리 (실기 리모컨 대응).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EvalLaunchParams {
    /// 발사 속도 [m/s].
    pub speed_mps: f64,
    /// 좌·우 존 yaw 절대값 [deg] (중앙은 0).
    pub side_yaw_deg: f64,
}

impl Default for EvalLaunchParams {
    fn default() -> Self {
        return Self {
            speed_mps: 6.0,
            // 테이블 1/3 바깥쪽을 겨냥하는 대략값 (±).
            side_yaw_deg: 10.0,
        };
    }
}

/// 한 발 관측 (shot_tune과 동일 계열).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct ShotFlags {
    pub contact: bool,
    pub cleared_net: bool,
    pub returned_in: bool,
    /// 리턴이 로봇 반쪽(`y < net_y`) 상면에 닿음 — 탁구 규칙상 반칙.
    ///
    /// 정상 리턴은 라켓에서 곧바로 상대 코트로 가야 한다. 자기 코트를
    /// 거쳐 넘어가는 건 약한 스윙의 대표 증상이고 실기에서는 실점이다.
    pub bounced_own_half: bool,
    /// 라켓이 같은 공을 두 번 침 — 반칙.
    pub double_hit: bool,
}

impl ShotFlags {
    /// 0 미타격 · 1 접촉(또는 반칙) · 2 네트 통과 · 3 상대 코트 착지.
    ///
    /// 반칙은 접촉만 인정해 1점으로 강등한다 — 네트를 넘겼든 상대 코트에
    /// 들어갔든 무효다.
    pub fn score(self) -> u8 {
        if !self.contact {
            return 0;
        }
        if self.bounced_own_half || self.double_hit {
            return 1;
        }
        if self.returned_in {
            return 3;
        }
        if self.cleared_net {
            return 2;
        }
        return 1;
    }

    /// 반칙으로 강등됐는지 — 패널·로그에서 "3점 조건을 다 채웠는데 반칙"을
    /// 구분해 보여주기 위한 것.
    pub fn is_foul(self) -> bool {
        return self.bounced_own_half || self.double_hit;
    }
}

/// 한 발 결과.
#[derive(Debug, Clone, PartialEq)]
pub struct EvalShot {
    pub zone: EvalZone,
    pub index_in_zone: usize,
    pub flags: ShotFlags,
    pub points: u8,
    /// 발사 당시 설정 — GUI에서 같은 시나리오를 다시 실행할 때 사용.
    pub settings: BallShooterSettings,
}

/// 존별 집계.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct ZoneScore {
    pub total: u32,
    pub counts: [u32; 4],
}

/// 전체 리포트.
#[derive(Debug, Clone, PartialEq)]
pub struct EvalReport {
    pub mode: EvalMode,
    pub shots: Vec<EvalShot>,
    pub by_zone: [ZoneScore; 3],
    pub total: u32,
    pub counts: [u32; 4],
}

impl EvalReport {
    pub fn passed(&self) -> bool {
        return self.total > PASS_SCORE_EXCLUSIVE;
    }

    pub fn zone_score(&self, zone: EvalZone) -> ZoneScore {
        return self.by_zone[zone.zone_index()];
    }
}

/// `(zone, index_in_zone)` 30발 스케줄.
pub fn shot_schedule(mode: EvalMode) -> Vec<(EvalZone, usize)> {
    match mode {
        EvalMode::Block => {
            let mut out = Vec::with_capacity(TOTAL_SHOTS);
            for zone in EvalZone::BLOCK_ORDER {
                for index_in_zone in 0..SHOTS_PER_ZONE {
                    out.push((zone, index_in_zone));
                }
            }
            return out;
        }
        EvalMode::Alternating => {
            // 왼→중앙→오→중앙→왼→… 존당 10발이 찰 때까지.
            let pattern = [
                EvalZone::Left,
                EvalZone::Center,
                EvalZone::Right,
                EvalZone::Center,
            ];
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

/// 백그라운드 진행 상태.
#[derive(Debug, Clone)]
pub struct EvalProgress {
    pub done: usize,
    pub total: usize,
    pub report: Option<EvalReport>,
    pub error: Option<String>,
}

impl Default for EvalProgress {
    fn default() -> Self {
        return Self {
            done: 0,
            total: TOTAL_SHOTS,
            report: None,
            error: None,
        };
    }
}

/// Eval 발사 파라미터를 존 템플릿으로 바꾼다 (지터 없음).
///
/// 스핀/롤=0. 좌·우는 `side_yaw_deg` 대칭, 중앙은 yaw=0.
/// pitch/height는 슈터 기본값에서 시작해 네트 게이트만 맞춘다.
pub fn settings_for_zone(launch: &EvalLaunchParams, zone: EvalZone) -> BallShooterSettings {
    return settings_for_zone_shot(launch, zone, 0);
}

/// 존 안 n번째 샷 (지터 없음). `index_in_zone`은 스케줄 호환용으로 유지.
pub fn settings_for_zone_shot(
    launch: &EvalLaunchParams,
    zone: EvalZone,
    index_in_zone: usize,
) -> BallShooterSettings {
    let _ = index_in_zone;
    return build_zone_shot::<rand::rngs::StdRng>(launch, zone, None);
}

/// 존 샷 + 미약 지터 (speed / yaw / pitch).
pub fn settings_for_zone_shot_jittered<R: Rng + ?Sized>(
    launch: &EvalLaunchParams,
    zone: EvalZone,
    index_in_zone: usize,
    rng: &mut R,
) -> BallShooterSettings {
    let _ = index_in_zone;
    return build_zone_shot(launch, zone, Some(rng));
}

fn build_zone_shot<R: Rng + ?Sized>(
    launch: &EvalLaunchParams,
    zone: EvalZone,
    mut rng: Option<&mut R>,
) -> BallShooterSettings {
    let mut shot = BallShooterSettings::default();
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
    if zone != EvalZone::Center
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

fn lift_pitch_for_net_gate(shot: &mut BallShooterSettings) {
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
pub fn run_eval_shot(
    robot: &Robot,
    physics: PhysicsParams,
    settings: &BallShooterSettings,
) -> (ShotFlags, bool) {
    const MAX_STEPS: usize = 4_000;
    const DT: f64 = 1.0 / 1000.0;

    let mut world = SimWorld::with_physics(robot.clone(), physics);
    world.set_use_ground_truth(true);
    world.shoot_ball(settings);

    let mut observer = LiveShotObserver::new(&world);
    for _ in 0..MAX_STEPS {
        world.step(DT, None);
        if observer.observe(&world) {
            break;
        }
    }
    return (observer.flags, observer.net_passthrough);
}

/// 라이브 시뮬에서 한 발의 0~3점 플래그를 누적 관찰한다.
#[derive(Debug, Clone)]
pub struct LiveShotObserver {
    pub flags: ShotFlags,
    /// 네트 실체를 관통함 (CCD 터널 등) — 채점 무효, 재시도 대상.
    pub net_passthrough: bool,
    previous_y: f32,
    saw_flight: bool,
    saw_net_contact: bool,
    /// 라켓 접촉 **이후**의 네트 접촉만 — 리턴이 네트를 스치고 넘어가는
    /// 건 랠리 중 유효타라, `saw_net_contact`(들어오는 공 포함)와 구분한다.
    net_contact_after_hit: bool,
    /// 직전 스텝에 라켓과 접촉 중이었는지 — 더블히트 상승 에지 판정용.
    racket_contact_active: bool,
    /// 라켓에서 떨어진 뒤 경과 스텝 — 접촉 채터링을 새 히트로 오인하지
    /// 않도록 재접촉에 최소 간격을 요구한다.
    steps_since_release: u32,
    finished: bool,
}

/// 접촉이 끊긴 뒤 이만큼 지나서 다시 닿아야 별개의 히트로 센다 (1 kHz 기준 30 ms).
/// Rapier narrow-phase 접촉은 임팩트 한 번에도 몇 스텝 깜빡일 수 있다.
const RACKET_REHIT_MIN_STEPS: u32 = 30;

impl LiveShotObserver {
    pub fn new(world: &SimWorld) -> Self {
        return Self {
            flags: ShotFlags::default(),
            net_passthrough: false,
            previous_y: world.ball_position().y,
            saw_flight: world.ball_state == BallState::InFlight,
            saw_net_contact: false,
            net_contact_after_hit: false,
            racket_contact_active: false,
            steps_since_release: u32::MAX,
            finished: false,
        };
    }

    pub fn finished(&self) -> bool {
        return self.finished;
    }

    pub fn points(&self) -> u8 {
        return self.flags.score();
    }

    /// 한 물리 스텝 후 호출. 종료되면 `true`.
    pub fn observe(&mut self, world: &SimWorld) -> bool {
        if self.finished {
            return true;
        }
        if world.ball_state == BallState::InFlight {
            self.saw_flight = true;
        }

        let position = world.ball_position();
        let velocity = world.ball_velocity();
        let net_y = (table::LENGTH_Y * 0.5) as f32;
        let net_top_z = (table::SURFACE_Z + table::NET_HEIGHT + BALL_RADIUS) as f32;

        if world.ball_intersects_net() {
            self.saw_net_contact = true;
        }

        // 클리어 높이 미만으로 네트 평면을 넘겼는데 접촉이 없으면 = 투과(물리 이상).
        if net_plane_passthrough(self.previous_y, position.y, position.z, net_y, net_top_z)
            && !self.saw_net_contact
        {
            self.net_passthrough = true;
            self.finished = true;
            return true;
        }

        // 라켓 접촉 — 상승 에지로 히트를 센다. 접촉이 끊겼다 최소 간격 뒤에
        // 다시 닿으면 더블히트(반칙)다.
        let touching_racket = ball_contacts_parent(world, world.racket_handle);
        if touching_racket && !self.racket_contact_active {
            if self.flags.contact && self.steps_since_release >= RACKET_REHIT_MIN_STEPS {
                self.flags.double_hit = true;
            }
            self.flags.contact = true;
        }
        if !touching_racket && self.racket_contact_active {
            self.steps_since_release = 0;
        } else if !touching_racket {
            self.steps_since_release = self.steps_since_release.saturating_add(1);
        }
        self.racket_contact_active = touching_racket;

        // 라켓 접촉 이후의 네트 접촉만 기록 — 리턴이 네트를 스치고 넘어가는
        // 건 랠리 중 유효타다.
        if self.flags.contact && world.ball_intersects_net() {
            self.net_contact_after_hit = true;
        }

        // 리턴이 자기 코트(로봇 반쪽) 상면에 닿으면 반칙. `flags.contact`
        // 게이트가 들어오는 공의 정상 바운스를 배제한다. 테이블 상면은
        // z = SURFACE_Z 이므로 공 중심이 그보다 위여야 윗면 접촉이다
        // (옆면을 맞으면 중심 z ≤ SURFACE_Z).
        if self.flags.contact
            && !self.flags.bounced_own_half
            && position.y < net_y
            && f64::from(position.z) > table::SURFACE_Z
            && ball_contacts_table(world)
        {
            self.flags.bounced_own_half = true;
        }

        let returned = self.flags.contact && velocity.y > 0.0;
        if returned && self.previous_y < net_y && position.y >= net_y {
            // 네트를 스치고 넘어간 경우도 통과로 인정한다.
            self.flags.cleared_net = position.z > net_top_z || self.net_contact_after_hit;
        }
        if self.flags.cleared_net
            && !self.flags.returned_in
            && position.y > net_y
            // 끝선(edge)에 걸치는 착지도 인(in)이다.
            && f64::from(position.y) <= table::LENGTH_Y + BALL_RADIUS
            // 상면 착지만 인정 — 옆면을 맞고 나가는 건 아웃.
            && f64::from(position.z) > table::SURFACE_Z
            && ball_contacts_table(world)
        {
            self.flags.returned_in = true;
            self.finished = true;
            return true;
        }
        if self.saw_flight && world.ball_state == BallState::Parked {
            self.finished = true;
            return true;
        }
        self.previous_y = position.y;
        return false;
    }
}

/// 네트 상단 클리어 높이 미만으로 미드코트(`net_y`)를 횡단했는지.
fn net_plane_passthrough(prev_y: f32, y: f32, z: f32, net_y: f32, net_top_z: f32) -> bool {
    if z >= net_top_z {
        return false;
    }
    let crossed = (prev_y < net_y && y >= net_y) || (prev_y > net_y && y <= net_y);
    return crossed;
}

fn ball_contacts_parent(world: &SimWorld, parent: RigidBodyHandle) -> bool {
    let Some(ball_collider) = world.collider_set.iter().find_map(|(handle, collider)| {
        (collider.parent() == Some(world.ball_handle)).then_some(handle)
    }) else {
        return false;
    };
    let Some(other) = world
        .collider_set
        .iter()
        .find_map(|(handle, collider)| (collider.parent() == Some(parent)).then_some(handle))
    else {
        return false;
    };
    return world
        .narrow_phase
        .contact_pair(ball_collider, other)
        .is_some_and(|pair| pair.has_any_active_contact());
}

fn ball_contacts_table(world: &SimWorld) -> bool {
    let Some(ball_collider) = world.collider_set.iter().find_map(|(handle, collider)| {
        (collider.parent() == Some(world.ball_handle)).then_some(handle)
    }) else {
        return false;
    };
    let Some(table_collider) = world.collider_set.iter().find_map(|(handle, collider)| {
        let cuboid = collider.shape().as_cuboid()?;
        ((f64::from(cuboid.half_extents.x) - table::WIDTH_X * 0.5).abs() < 1e-5
            && (f64::from(cuboid.half_extents.y) - table::LENGTH_Y * 0.5).abs() < 1e-5)
            .then_some(handle)
    }) else {
        return false;
    };
    return world
        .narrow_phase
        .contact_pair(ball_collider, table_collider)
        .is_some_and(|pair| pair.has_any_active_contact());
}

/// 30발 프로토콜 실행. `progress`가 있으면 매 발 후 갱신.
pub fn run_eval_protocol(
    robot: &Robot,
    physics: PhysicsParams,
    launch: &EvalLaunchParams,
    mode: EvalMode,
    progress: Option<Arc<Mutex<EvalProgress>>>,
) -> EvalReport {
    let mut shots = Vec::with_capacity(TOTAL_SHOTS);
    let mut by_zone = [ZoneScore::default(); 3];
    let mut counts = [0_u32; 4];
    let mut total = 0_u32;
    let mut done = 0_usize;

    let mut rng = rand::thread_rng();
    for (zone, index_in_zone) in shot_schedule(mode) {
        let mut settings = settings_for_zone_shot_jittered(launch, zone, index_in_zone, &mut rng);
        let mut flags = ShotFlags::default();
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
            flags = ShotFlags::default();
        }
        let points = flags.score();
        total += u32::from(points);
        counts[points as usize] += 1;
        let zi = zone.zone_index();
        by_zone[zi].total += u32::from(points);
        by_zone[zi].counts[points as usize] += 1;
        shots.push(EvalShot {
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

    let report = EvalReport {
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

#[cfg(test)]
mod tests {
    use super::*;

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
        assert_eq!(ShotFlags::default().score(), 0);
        assert_eq!(
            ShotFlags {
                contact: true,
                ..Default::default()
            }
            .score(),
            1
        );
        assert_eq!(
            ShotFlags {
                contact: true,
                cleared_net: true,
                ..Default::default()
            }
            .score(),
            2
        );
        assert_eq!(
            ShotFlags {
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
        let flags = ShotFlags {
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
        let flags = ShotFlags {
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
        let flags = ShotFlags {
            bounced_own_half: true,
            ..Default::default()
        };
        assert_eq!(flags.score(), 0);
    }

    #[test]
    fn pass_requires_more_than_45() {
        let mut report = EvalReport {
            mode: EvalMode::Block,
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
        for mode in [EvalMode::Block, EvalMode::Alternating] {
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
        let sched = shot_schedule(EvalMode::Alternating);
        assert_eq!(
            sched[..4].iter().map(|(z, _)| *z).collect::<Vec<_>>(),
            vec![
                EvalZone::Left,
                EvalZone::Center,
                EvalZone::Right,
                EvalZone::Center
            ]
        );
    }

    #[test]
    fn block_is_left_then_center_then_right() {
        let sched = shot_schedule(EvalMode::Block);
        assert!(sched[..10].iter().all(|(z, _)| *z == EvalZone::Left));
        assert!(sched[10..20].iter().all(|(z, _)| *z == EvalZone::Center));
        assert!(sched[20..].iter().all(|(z, _)| *z == EvalZone::Right));
    }

    #[test]
    fn zone_shot_jitter_moves_speed_yaw_pitch_but_keeps_zone_side() {
        use rand::SeedableRng;

        let launch = EvalLaunchParams::default();
        let clean = settings_for_zone_shot(&launch, EvalZone::Left, 3);
        let mut rng = rand::rngs::StdRng::seed_from_u64(7);
        let jittered = settings_for_zone_shot_jittered(&launch, EvalZone::Left, 3, &mut rng);

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
        let launch = EvalLaunchParams {
            speed_mps: 7.0,
            side_yaw_deg: 12.0,
        };
        let left = settings_for_zone(&launch, EvalZone::Left);
        let right = settings_for_zone(&launch, EvalZone::Right);
        let center = settings_for_zone(&launch, EvalZone::Center);
        assert!((left.yaw_deg + right.yaw_deg).abs() < 1e-12);
        assert!((center.yaw_deg).abs() < 1e-12);
        assert!((left.speed_mps - 7.0).abs() < 1e-12);
    }
}

#[cfg(test)]
mod smoke {
    use super::*;
    use crate::defaults;

    #[test]
    fn protocol_runs_and_prints_score() {
        let robot = defaults::robot().expect("robot");
        let report = run_eval_protocol(
            &robot,
            defaults::physics(),
            &EvalLaunchParams::default(),
            EvalMode::Block,
            None,
        );
        eprintln!(
            "EVAL total={}/{} pass={} counts={:?} zones={:?}",
            report.total,
            MAX_SCORE,
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
