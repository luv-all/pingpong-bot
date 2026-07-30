//! 라이브 시뮬 한 발 관찰.

use crate::constants::{BALL_RADIUS, table};
use crate::sim::physics::world::SimWorld;
use rapier3d::prelude::RigidBodyHandle;

use super::{Flags, RACKET_REHIT_MIN_STEPS};

/// 라이브 시뮬에서 한 발의 0~3점 플래그를 누적 관찰한다.
#[derive(Debug, Clone)]
pub struct LiveObserver {
    pub flags: Flags,
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

/// 접촉이 끊긴 뒤 다시 닿아야 별개 히트로 센다 — [`RACKET_REHIT_MIN_STEPS`].

impl LiveObserver {
    pub fn new(world: &SimWorld) -> Self {
        return Self {
            flags: Flags::default(),
            net_passthrough: false,
            previous_y: world.ball_position().y,
            saw_flight: world.ball_state == crate::sim::physics::BallState::InFlight,
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
        if world.ball_state == crate::sim::physics::BallState::InFlight {
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
        if self.saw_flight && world.ball_state == crate::sim::physics::BallState::Parked {
            self.finished = true;
            return true;
        }
        self.previous_y = position.y;
        return false;
    }
}

/// 네트 상단 클리어 높이 미만으로 미드코트(`net_y`)를 횡단했는지.
pub(crate) fn net_plane_passthrough(
    prev_y: f32,
    y: f32,
    z: f32,
    net_y: f32,
    net_top_z: f32,
) -> bool {
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
