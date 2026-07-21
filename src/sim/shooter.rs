//! 반대편 볼 슈터(발사기) — 로봇(y≈0) 반대(+y)에서 공을 쏴 탁구로봇이 받는 구조.

use crate::constants::table;
use rapier3d::prelude::{Rotation, Vector};

/// 슈터 설치 위치 (월드 좌표, Z-up).
pub struct ShooterLayout;

impl ShooterLayout {
    /// 로봇은 y≈0, 슈터는 테이블 +y 끝(상대편).
    pub const MOUNT_X: f64 = table::WIDTH_X * 0.5;
    /// 슈터 베이스 y [m]
    pub const MOUNT_Y: f64 = table::LENGTH_Y - 0.12;
    /// 슈터 본체 높이 [m] (테이블 면 기준)
    pub const BODY_HEIGHT: f64 = 0.45;
    /// 발사구가 본체 중심에서 조준 방향으로 돌출된 길이 [m]
    pub const BARREL_FORWARD_M: f64 = 0.22;
}

/// GUI·런타임에서 조절하는 발사 파라미터.
#[derive(Debug, Clone, PartialEq)]
pub struct BallShooterSettings {
    /// 초기 속도 크기 [m/s]
    pub speed_mps: f64,
    /// yaw [deg] — Z축 기준 좌우 조준 (0=로봇 정면, +x=우측)
    pub yaw_deg: f64,
    /// pitch [deg] — 위아래 조준 (0=수평, +=위, -=아래)
    pub pitch_deg: f64,
    /// roll [deg] — 발사축 기준 롤 (스핀 축·발사구 위치 회전)
    pub roll_deg: f64,
    /// 발사구 좌우 오프셋 [m] — 슈터 로컬 right
    pub lateral_offset_m: f64,
    /// 발사구 높이 오프셋 [m] — 슈터 로컬 up (본체 중심 기준)
    pub height_offset_m: f64,
    /// topspin [rad/s] — 슈터 로컬 right 축 (+=topspin)
    pub topspin_rad_s: f64,
    /// sidespin [rad/s] — 슈터 로컬 up 축
    pub sidespin_rad_s: f64,
    /// drill spin [rad/s] — 슈터 로컬 forward 축 (총구 축 회전)
    pub drill_spin_rad_s: f64,
}

impl Default for BallShooterSettings {
    fn default() -> Self {
        return Self {
            speed_mps: 5.0,
            yaw_deg: 0.0,
            pitch_deg: -4.0,
            roll_deg: 0.0,
            lateral_offset_m: 0.0,
            height_offset_m: 0.19,
            topspin_rad_s: 0.0,
            sidespin_rad_s: 0.0,
            drill_spin_rad_s: 0.0,
        };
    }
}

impl BallShooterSettings {
    /// 슈터 본체 중심 (월드).
    pub fn mount_position(&self) -> Vector {
        return Vector::new(
            ShooterLayout::MOUNT_X as f32,
            ShooterLayout::MOUNT_Y as f32,
            (table::SURFACE_Z + ShooterLayout::BODY_HEIGHT * 0.5) as f32,
        );
    }

    /// 조준 방향 단위벡터 (월드). yaw=0, pitch=0 이면 -y.
    pub fn aim_direction(&self) -> Vector {
        let yaw = self.yaw_deg.to_radians() as f32;
        let pitch = self.pitch_deg.to_radians() as f32;
        let x = pitch.cos() * yaw.sin();
        let y = -pitch.cos() * yaw.cos();
        let z = pitch.sin();
        return Vector::new(x, y, z).normalize();
    }

    /// 슈터 로컬 (forward, right, up) — roll 반영.
    pub fn local_basis(&self) -> (Vector, Vector, Vector) {
        let forward = self.aim_direction();
        let world_up = Vector::new(0.0, 0.0, 1.0);
        let mut right = world_up.cross(forward);
        if right.length_squared() < 1e-8 {
            right = Vector::new(1.0, 0.0, 0.0);
        } else {
            right = right.normalize();
        }
        let mut up = forward.cross(right);
        up = up.normalize();

        let roll = self.roll_deg.to_radians() as f32;
        let (sin, cos) = roll.sin_cos();
        let right_r = right * cos + up * sin;
        let up_r = up * cos - right * sin;
        return (forward, right_r, up_r);
    }

    /// 슈터 본체 회전 (조준 + roll).
    pub fn orientation(&self) -> Rotation {
        let forward = self.aim_direction();
        let aim = Rotation::from_rotation_arc(Vector::new(0.0, -1.0, 0.0), forward);
        let roll = self.roll_deg.to_radians() as f32;
        let roll_q = Rotation::from_axis_angle(forward, roll);
        return roll_q * aim;
    }

    /// 발사구 위치 — 슈터 로컬 오프셋을 월드로 변환.
    pub fn muzzle_position(&self) -> Vector {
        let (forward, right, up) = self.local_basis();
        let local = forward * (ShooterLayout::BARREL_FORWARD_M as f32)
            + up * self.height_offset_m as f32
            + right * self.lateral_offset_m as f32;
        return self.mount_position() + local;
    }

    /// 조준 방향 × 속도.
    pub fn launch_velocity(&self) -> Vector {
        return self.aim_direction() * self.speed_mps as f32;
    }

    /// 슈터 로컬 스핀 축을 월드 각속도로 변환.
    pub fn launch_angular_velocity(&self) -> Vector {
        let (forward, right, up) = self.local_basis();
        return right * self.topspin_rad_s as f32
            + up * self.sidespin_rad_s as f32
            + forward * self.drill_spin_rad_s as f32;
    }
}

/// 공 비행 상태.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BallState {
    /// 슈터 발사구에 고정 대기
    Parked,
    /// 비행 중
    InFlight,
}

impl std::fmt::Display for BallState {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        return f.write_str(match self {
            Self::Parked => "parked",
            Self::InFlight => "in flight",
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_aims_toward_robot_with_slight_drop() {
        let s = BallShooterSettings::default();
        let dir = s.aim_direction();
        assert!(dir.y < 0.0);
        assert!(dir.z < 0.0);
        assert!(dir.x.abs() < 1e-5);
    }

    #[test]
    fn yaw_deflects_toward_plus_x() {
        let mut s = BallShooterSettings::default();
        s.yaw_deg = 10.0;
        s.pitch_deg = 0.0;
        let dir = s.aim_direction();
        assert!(dir.x > 0.0);
        assert!(dir.y < 0.0);
    }

    #[test]
    fn launch_velocity_matches_speed_and_aim() {
        let s = BallShooterSettings {
            speed_mps: 10.0,
            ..Default::default()
        };
        let v = s.launch_velocity();
        assert!((v.length() - 10.0).abs() < 1e-4);
        assert!(v.y < 0.0);
    }

    #[test]
    fn topspin_is_around_local_right() {
        let s = BallShooterSettings {
            topspin_rad_s: 30.0,
            ..Default::default()
        };
        let w = s.launch_angular_velocity();
        assert!(w.length() > 0.0);
    }
}
