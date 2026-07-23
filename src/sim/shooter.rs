//! 반대편 볼 슈터(발사기) — 로봇(y≈0) 반대(+y)에서 공을 쏴 탁구로봇이 받는 구조.

use crate::constants::table;
use rand::Rng;
use rapier3d::prelude::{Rotation, Vector};

/// `BallShooterSettings::randomized`가 뽑는 좌우 발사 위치(`lateral_offset_m`) 범위 [m].
/// `random_shot_lateral_range_stays_within_table`이 이 전체 범위에서
/// 바운스가 테이블 안에 들어옴을 검증한다.
pub const RANDOM_SHOT_LATERAL_MIN_M: f64 = -0.5;
pub const RANDOM_SHOT_LATERAL_MAX_M: f64 = 0.5;

/// 랜덤 조준 목표를 로봇쪽 테이블 가장자리(y=0) 양 끝에서 이만큼 안쪽으로
/// 제한한다.
///
/// 처음엔 0.15로 잡았는데, 좌우 위치(lateral) × 그 위치에서 유효한 yaw 전체
/// 범위 × 속도를 촘촘히 스윕해보니 각도가 비스듬할수록(각·좌우 극단 조합)
/// "네트를 통과하는 하한 속도"와 "4-dof 로봇 리치 상한 속도" 사이의 여유가
/// 줄어들어, padding=0.15에서는 둘을 동시에 만족하는 속도가 5.5 m/s
/// 딱 한 점뿐이었다(즉 속도 랜덤화 여지가 사실상 없었음). padding을 넓혀
/// 각도를 덜 극단적으로 만들수록 그 여유가 벌어진다(실측: 0.15→[5.5,5.5],
/// 0.25→[5.4,5.5], 0.35→[5.3,5.5], 0.45→[5.2,5.5]). 0.45로 잡아
/// 속도에도 실질적인 랜덤 폭(0.3 m/s)을 확보했다 — 좌우 위치(lateral)
/// 쪽 다양성은 그대로 유지된다.
pub const RANDOM_SHOT_TARGET_PADDING_M: f64 = 0.45;

/// `BallShooterSettings::randomized`가 뽑는 속도 범위 [m/s].
/// (기본 슬라이더 범위 3.0..=15.0보다 훨씬 좁다.)
///
/// `RANDOM_SHOT_TARGET_PADDING_M`과 함께 촘촘한 격자 실측으로 찾은 범위:
/// 하한은 네트 통과(기본 pitch=-2°, 순수 탄도라 로봇 모델과 무관 — 비스듬한
/// 샷일수록 마진이 줄어든다), 상한은 GUI가 실제로 쓰는 카탈로그 "4-dof"
/// 로봇(`fourdof_robot`, URDF + `Rep103AtTableEnd`)의 리치 — `competition_arm()`
/// (손으로 만든 테스트용 팔)만으로는 상한 쪽 문제가 안 보였다(로봇마다 리치가
/// 다름). [5.2, 5.5] 범위는 좌우 위치·yaw 전체 격자에서 두 조건 모두 실측
/// 통과.
pub const RANDOM_SHOT_SPEED_MIN_MPS: f64 = 5.2;
pub const RANDOM_SHOT_SPEED_MAX_MPS: f64 = 5.5;

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
            pitch_deg: -2.0,
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

    /// 좌우 발사 위치(`lateral_offset_m`)에서 로봇쪽 테이블 가장자리(y=0)의
    /// padding 안쪽 구간 전체를 조준하는 데 필요한 yaw 범위 [deg] — `(최소, 최대)`.
    ///
    /// 발사 위치가 정해지면 "테이블 위 어딘가를 조준한다"는 조건만으로 yaw
    /// 범위가 기하학적으로 결정된다: 좌우 padding을 둔 반대편 가장자리
    /// 양 끝을 잇는 선까지의 각도. 이 범위 안에서 뽑으면 좌우 위치가 다른
    /// 두 샷이 진짜로 다른 궤적(다른 각도)이 된다 — `lateral_offset_m`만
    /// 바꾸는 평행이동과 달리.
    pub(crate) fn yaw_range_for_lateral_deg(lateral_offset_m: f64) -> (f64, f64) {
        let mount_x = ShooterLayout::MOUNT_X + lateral_offset_m;
        let mount_y = ShooterLayout::MOUNT_Y;
        let yaw_deg_for_target_x = |target_x: f64| -> f64 {
            let dx = target_x - mount_x;
            let dy = 0.0 - mount_y;
            return dx.atan2(-dy).to_degrees();
        };
        let yaw_left = yaw_deg_for_target_x(RANDOM_SHOT_TARGET_PADDING_M);
        let yaw_right = yaw_deg_for_target_x(table::WIDTH_X - RANDOM_SHOT_TARGET_PADDING_M);
        return (yaw_left.min(yaw_right), yaw_left.max(yaw_right));
    }

    /// `lateral_offset_m`(발사 위치)·`yaw_deg`(그 위치에서 기하학적으로 유효한
    /// 조준 범위)·`speed_mps`를 안전 범위 안에서 랜덤화한 새 설정.
    ///
    /// pitch·roll·height·spin은 호출 시점 값 그대로 유지된다.
    pub fn randomized(&self, rng: &mut impl Rng) -> Self {
        let lateral_offset_m = rng.gen_range(RANDOM_SHOT_LATERAL_MIN_M..=RANDOM_SHOT_LATERAL_MAX_M);
        let (yaw_min, yaw_max) = Self::yaw_range_for_lateral_deg(lateral_offset_m);
        let yaw_deg = rng.gen_range(yaw_min..=yaw_max);
        let speed_mps = rng.gen_range(RANDOM_SHOT_SPEED_MIN_MPS..=RANDOM_SHOT_SPEED_MAX_MPS);
        return Self {
            lateral_offset_m,
            yaw_deg,
            speed_mps,
            ..self.clone()
        };
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

    #[test]
    fn yaw_range_shifts_toward_the_farther_edge() {
        // 발사 위치가 오른쪽(+x)으로 치우칠수록: 가까운 오른쪽 padding 가장자리는
        // 거의 정면(yaw_max가 0에 가까워짐)이고, 먼 왼쪽 padding 가장자리는 더
        // 비스듬한 각도(yaw_min이 더 음수)가 필요하다. 왼쪽으로 치우치면 반대.
        let (left_min, left_max) = BallShooterSettings::yaw_range_for_lateral_deg(-0.5);
        let (center_min, center_max) = BallShooterSettings::yaw_range_for_lateral_deg(0.0);
        let (right_min, right_max) = BallShooterSettings::yaw_range_for_lateral_deg(0.5);

        assert!(right_min < center_min && center_min < left_min);
        assert!(right_max < center_max && center_max < left_max);
        // 중앙 발사에서는 좌우 padding이 같으니 범위도 원점 대칭이어야 한다.
        assert!((center_min + center_max).abs() < 1e-6);
    }

    #[test]
    fn randomized_only_touches_lateral_yaw_speed() {
        let base = BallShooterSettings {
            pitch_deg: -7.0,
            roll_deg: 12.0,
            height_offset_m: 0.05,
            topspin_rad_s: 3.0,
            sidespin_rad_s: -2.0,
            drill_spin_rad_s: 1.0,
            ..Default::default()
        };
        let mut rng = rand::thread_rng();
        for _ in 0..50 {
            let shot = base.randomized(&mut rng);
            assert!((RANDOM_SHOT_LATERAL_MIN_M..=RANDOM_SHOT_LATERAL_MAX_M)
                .contains(&shot.lateral_offset_m));
            assert!((RANDOM_SHOT_SPEED_MIN_MPS..=RANDOM_SHOT_SPEED_MAX_MPS)
                .contains(&shot.speed_mps));
            let (yaw_min, yaw_max) =
                BallShooterSettings::yaw_range_for_lateral_deg(shot.lateral_offset_m);
            assert!(shot.yaw_deg >= yaw_min - 1e-9 && shot.yaw_deg <= yaw_max + 1e-9);

            assert_eq!(shot.pitch_deg, base.pitch_deg);
            assert_eq!(shot.roll_deg, base.roll_deg);
            assert_eq!(shot.height_offset_m, base.height_offset_m);
            assert_eq!(shot.topspin_rad_s, base.topspin_rad_s);
            assert_eq!(shot.sidespin_rad_s, base.sidespin_rad_s);
            assert_eq!(shot.drill_spin_rad_s, base.drill_spin_rad_s);
        }
    }
}
