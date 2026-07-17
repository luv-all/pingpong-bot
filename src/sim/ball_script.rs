//! 탁구공 동역학 스크립트 — sim 시간에 맞춰 위치·속도·임펄스를 적용한다.
//!
//! 슈터 GUI 발사와 별도로, **원하는 시각**에 공 상태를 직접 제어할 때 쓴다.
//! Rapier dynamic body에 `Launch` / `Impulse` / `SetVelocity` 등을 큐잉한다.

use rapier3d::prelude::Vector;

use super::shooter::BallShooterSettings;

/// 월드 좌표 [m] 또는 속도 [m/s] / 임펄스 [N·s].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BallVec3(pub f32, pub f32, pub f32);

impl BallVec3 {
    pub const ZERO: Self = Self(0.0, 0.0, 0.0);

    pub fn new(x: f32, y: f32, z: f32) -> Self {
        return Self(x, y, z);
    }

    pub(crate) fn to_rapier(self) -> Vector {
        return Vector::new(self.0, self.1, self.2);
    }
}

/// sim 시간 `at_time` [s]에 실행할 공 조작.
#[derive(Debug, Clone, PartialEq)]
pub enum BallAction {
    /// 위치·선속도·각속도로 dynamic 비행 시작.
    Launch {
        position: BallVec3,
        linear_velocity: BallVec3,
        angular_velocity: BallVec3,
    },
    /// 선형 임펄스 [N·s] (공이 dynamic일 때).
    Impulse { impulse: BallVec3 },
    /// 속도를 직접 덮어쓴다 (dynamic).
    SetVelocity {
        linear_velocity: BallVec3,
        angular_velocity: BallVec3,
    },
    /// 위치만 순간이동 (dynamic 유지).
    Teleport { position: BallVec3 },
    /// 주차 — `None`이면 현재 위치에 고정.
    Park { position: Option<BallVec3> },
}

/// `sim_time >= at_time` 일 때 한 번 실행되는 이벤트.
#[derive(Debug, Clone, PartialEq)]
pub struct BallEvent {
    /// sim 경과 시간 [s] (0 = 세션 시작 직후)
    pub at_time: f64,
    pub action: BallAction,
}

/// 시간순 공 조작 큐.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct BallScript {
    events: Vec<BallEvent>,
}

impl BallScript {
    pub fn new() -> Self {
        return Self::default();
    }

    /// 이벤트를 추가하고 `at_time` 오름차순으로 정렬한다.
    pub fn push(&mut self, event: BallEvent) {
        self.events.push(event);
        self.events.sort_by(|a, b| {
            a.at_time
                .partial_cmp(&b.at_time)
                .unwrap_or(std::cmp::Ordering::Equal)
        });
    }

    /// `at_time`에 위치·속도로 발사.
    pub fn launch_at(
        &mut self,
        at_time: f64,
        position: BallVec3,
        linear_velocity: BallVec3,
        angular_velocity: BallVec3,
    ) {
        self.push(BallEvent {
            at_time,
            action: BallAction::Launch {
                position,
                linear_velocity,
                angular_velocity,
            },
        });
    }

    /// `at_time`에 선형 임펄스.
    pub fn impulse_at(&mut self, at_time: f64, impulse: BallVec3) {
        self.push(BallEvent {
            at_time,
            action: BallAction::Impulse { impulse },
        });
    }

    /// 슈터 설정과 동일한 조건으로 `at_time`에 발사.
    pub fn launch_from_shooter_at(&mut self, at_time: f64, settings: &BallShooterSettings) {
        let pos = settings.muzzle_position();
        let vel = settings.launch_velocity();
        let ang = settings.launch_angular_velocity();
        self.launch_at(
            at_time,
            BallVec3::new(pos.x, pos.y, pos.z),
            BallVec3::new(vel.x, vel.y, vel.z),
            BallVec3::new(ang.x, ang.y, ang.z),
        );
    }

    pub fn is_empty(&self) -> bool {
        return self.events.is_empty();
    }

    pub fn len(&self) -> usize {
        return self.events.len();
    }

    pub fn clear(&mut self) {
        self.events.clear();
    }

    pub fn events(&self) -> &[BallEvent] {
        return &self.events;
    }

    /// 대기 이벤트를 기존 큐 뒤에 병합한다.
    pub fn extend(&mut self, other: BallScript) {
        for event in other.events {
            self.push(event);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn events_stay_sorted_by_time() {
        let mut script = BallScript::new();
        script.impulse_at(0.5, BallVec3::new(0.0, 1.0, 0.0));
        script.launch_at(
            0.0,
            BallVec3::ZERO,
            BallVec3::new(0.0, -5.0, 0.0),
            BallVec3::ZERO,
        );
        assert!(script.events()[0].at_time < script.events()[1].at_time);
    }
}
