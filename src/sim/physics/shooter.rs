//! 반대편 볼 슈터(발사기) — 로봇(y≈0) 반대(+y)에서 공을 쏴 탁구로봇이 받는 구조.

use crate::constants::table;
use crate::estimator::ballistics::predict_hit_plane;
use crate::HitPlane;
use nalgebra::Vector3;
use rand::Rng;
use rapier3d::prelude::{Rotation, Vector};

/// GUI `randomized`가 네트 미달 샘플을 버리는 최대 재시도 횟수.
const RANDOM_SHOT_NET_GATE_MAX_TRIES: usize = 48;

/// `BallShooterSettings::randomized`가 뽑는 좌우 발사 위치(`lateral_offset_m`) 범위 [m].
/// `random_shot_lateral_range_stays_within_table`이 이 전체 범위에서
/// 바운스가 테이블 안에 들어옴을 검증한다.
pub const RANDOM_SHOT_LATERAL_MIN_M: f64 = -0.5;
pub const RANDOM_SHOT_LATERAL_MAX_M: f64 = 0.5;

/// 랜덤 조준 목표를 로봇쪽 테이블 가장자리(y=0) 양 끝에서 이만큼 안쪽으로
/// 제한한다.
///
/// padding↓ → yaw 폭↑(가장자리 조준). 0.25 ≈ 조준 폭 1.03m (중앙±0.51).
/// GUI/리치 SSOT는 `urdf_4dof` 격자(`random_shot_fine_grid_*`)다.
pub const RANDOM_SHOT_TARGET_PADDING_M: f64 = 0.25;

/// `BallShooterSettings::randomized`가 뽑는 속도 범위 [m/s].
pub const RANDOM_SHOT_SPEED_MIN_MPS: f64 = 5.6;
pub const RANDOM_SHOT_SPEED_MAX_MPS: f64 = 5.9;

/// 랜덤 발사구 높이 오프셋 [m] (`height_offset_m`, 슈터 로컬 up).
pub const RANDOM_SHOT_HEIGHT_MIN_M: f64 = 0.24;
pub const RANDOM_SHOT_HEIGHT_MAX_M: f64 = 0.32;

/// 랜덤 topspin [rad/s] (+=topspin).
pub const RANDOM_SHOT_TOPSPIN_MIN: f64 = -20.0;
pub const RANDOM_SHOT_TOPSPIN_MAX: f64 = 20.0;
/// 랜덤 sidespin [rad/s].
pub const RANDOM_SHOT_SIDESPIN_MIN: f64 = -15.0;
pub const RANDOM_SHOT_SIDESPIN_MAX: f64 = 15.0;

/// 랜덤 pitch [deg] (+=위). 기본 −1° 근처 — 너무 올리면 네트, 너무 내리면 테이블.
pub const RANDOM_SHOT_PITCH_MIN_DEG: f64 = -3.0;
pub const RANDOM_SHOT_PITCH_MAX_DEG: f64 = 0.5;
/// 랜덤 roll [deg] (발사축 기준).
pub const RANDOM_SHOT_ROLL_MIN_DEG: f64 = -15.0;
pub const RANDOM_SHOT_ROLL_MAX_DEG: f64 = 15.0;

/// 슈터 설치 위치 (월드 좌표, Z-up).
pub struct ShooterLayout;

impl ShooterLayout {
    /// 로봇은 y≈0, 슈터는 테이블 +y 끝(상대편).
    pub const MOUNT_X: f64 = table::WIDTH_X * 0.5;
    /// 마운트 기준 발사구 전방 돌출 [m] (탄도 SSOT)
    pub const BARREL_FORWARD_M: f64 = 0.22;
    /// 뷰어 직육면체 전체 크기 [m] (충돌 없음 — 표시 전용)
    pub const VISUAL_SIZE_X: f64 = 0.10;
    pub const VISUAL_SIZE_Y: f64 = 0.18;
    pub const VISUAL_SIZE_Z: f64 = 0.14;
    /// 슈터 마운트 y [m] — 본체는 테이블 밖, 발사구는 끝선(LENGTH_Y).
    pub const MOUNT_Y: f64 = table::LENGTH_Y + Self::BARREL_FORWARD_M;
    /// 슈터 마운트 기준 높이 [m] (테이블 면 → 중심). 탄도 SSOT.
    pub const BODY_HEIGHT: f64 = 0.45;
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
    /// 마운트 월드 오프셋 [m] — 기본 설치점(`ShooterLayout::MOUNT_*`) 기준
    pub pos_offset_x_m: f64,
    pub pos_offset_y_m: f64,
    pub pos_offset_z_m: f64,
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
            // 끝선 발사(비행거리↑) — 네트 클리어 기준
            speed_mps: 5.6,
            yaw_deg: 0.0,
            pitch_deg: -1.0,
            roll_deg: 0.0,
            pos_offset_x_m: 0.0,
            pos_offset_y_m: 0.0,
            pos_offset_z_m: 0.0,
            lateral_offset_m: 0.0,
            height_offset_m: 0.28,
            topspin_rad_s: 0.0,
            sidespin_rad_s: 0.0,
            drill_spin_rad_s: 0.0,
        };
    }
}

impl BallShooterSettings {
    /// 슈터 마운트 기준점 (월드) — 탄도·오프셋의 원점.
    pub fn mount_position(&self) -> Vector {
        return Vector::new(
            (ShooterLayout::MOUNT_X + self.pos_offset_x_m) as f32,
            (ShooterLayout::MOUNT_Y + self.pos_offset_y_m) as f32,
            (table::SURFACE_Z + ShooterLayout::BODY_HEIGHT * 0.5 + self.pos_offset_z_m) as f32,
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

    /// 발사구 위치 — 슈터 로컬 오프셋을 월드로 변환 (탄도 SSOT).
    pub fn muzzle_position(&self) -> Vector {
        let (forward, right, up) = self.local_basis();
        let local = forward * (ShooterLayout::BARREL_FORWARD_M as f32)
            + up * self.height_offset_m as f32
            + right * self.lateral_offset_m as f32;
        return self.mount_position() + local;
    }

    /// 뷰어 직육면체 중심 — 발사구가 전면에 오도록 조준축 뒤로 반 길이.
    pub fn visual_position(&self) -> Vector {
        let (forward, _, _) = self.local_basis();
        let half_depth = (ShooterLayout::VISUAL_SIZE_Y * 0.5) as f32;
        return self.muzzle_position() - forward * half_depth;
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
        return Self::yaw_range_for_mount_deg(
            ShooterLayout::MOUNT_X + lateral_offset_m,
            ShooterLayout::MOUNT_Y,
        );
    }

    /// 마운트 (x,y)에서 로봇쪽 테이블 padding 안쪽을 조준하는 yaw 범위 [deg].
    pub(crate) fn yaw_range_for_mount_deg(mount_x: f64, mount_y: f64) -> (f64, f64) {
        let yaw_deg_for_target_x = |target_x: f64| -> f64 {
            let dx = target_x - mount_x;
            let dy = 0.0 - mount_y;
            return dx.atan2(-dy).to_degrees();
        };
        let yaw_left = yaw_deg_for_target_x(RANDOM_SHOT_TARGET_PADDING_M);
        let yaw_right = yaw_deg_for_target_x(table::WIDTH_X - RANDOM_SHOT_TARGET_PADDING_M);
        return (yaw_left.min(yaw_right), yaw_left.max(yaw_right));
    }

    /// 좌우·yaw·속도만 안전 범위 안에서 랜덤화한다.
    ///
    /// 접수·리치 회귀 테스트용 — 높이·스핀·pitch/roll은 호출 시점 값을 유지한다.
    pub fn randomized_aim(&self, rng: &mut impl Rng) -> Self {
        let lateral_offset_m = rng.gen_range(RANDOM_SHOT_LATERAL_MIN_M..=RANDOM_SHOT_LATERAL_MAX_M);
        let mount_x = ShooterLayout::MOUNT_X + self.pos_offset_x_m + lateral_offset_m;
        let mount_y = ShooterLayout::MOUNT_Y + self.pos_offset_y_m;
        let (yaw_min, yaw_max) = Self::yaw_range_for_mount_deg(mount_x, mount_y);
        let yaw_deg = rng.gen_range(yaw_min..=yaw_max);
        let speed_mps = rng.gen_range(RANDOM_SHOT_SPEED_MIN_MPS..=RANDOM_SHOT_SPEED_MAX_MPS);
        return Self {
            lateral_offset_m,
            yaw_deg,
            speed_mps,
            ..self.clone()
        };
    }

    /// 발사 직후 탄도가 네트 게이트·hit-plane에 도달하는지 (ballistics + 스핀).
    ///
    /// Rapier와 같은 `PhysicsParams`(drag/magnus)로 적분한다.
    pub fn clears_incoming_net_gate(&self) -> bool {
        let muzzle = self.muzzle_position();
        let vel = self.launch_velocity();
        let omega = self.launch_angular_velocity();
        let position = Vector3::new(f64::from(muzzle.x), f64::from(muzzle.y), f64::from(muzzle.z));
        let velocity = Vector3::new(f64::from(vel.x), f64::from(vel.y), f64::from(vel.z));
        let spin = Vector3::new(f64::from(omega.x), f64::from(omega.y), f64::from(omega.z));
        let plane = HitPlane {
            y: table::DEFAULT_HIT_PLANE_Y,
        };
        return predict_hit_plane(
            position,
            velocity,
            spin,
            plane,
            &crate::defaults::physics(),
        )
        .is_some();
    }

    fn sample_randomized_params(&self, rng: &mut impl Rng) -> Self {
        let mut shot = self.randomized_aim(rng);
        shot.height_offset_m =
            rng.gen_range(RANDOM_SHOT_HEIGHT_MIN_M..=RANDOM_SHOT_HEIGHT_MAX_M);
        shot.topspin_rad_s = rng.gen_range(RANDOM_SHOT_TOPSPIN_MIN..=RANDOM_SHOT_TOPSPIN_MAX);
        shot.sidespin_rad_s =
            rng.gen_range(RANDOM_SHOT_SIDESPIN_MIN..=RANDOM_SHOT_SIDESPIN_MAX);
        shot.pitch_deg = rng.gen_range(RANDOM_SHOT_PITCH_MIN_DEG..=RANDOM_SHOT_PITCH_MAX_DEG);
        shot.roll_deg = rng.gen_range(RANDOM_SHOT_ROLL_MIN_DEG..=RANDOM_SHOT_ROLL_MAX_DEG);
        return shot;
    }

    /// 좌우·높이·yaw·pitch·roll·속도·스핀을 안전 범위 안에서 랜덤화한 새 설정.
    ///
    /// 네트 미달(또는 hit-plane 미도달) 샘플은 ballistics로 버리고 다시 뽑는다.
    /// drill·마운트 `pos_offset_*`는 호출 시점 값 그대로 유지된다.
    /// (GUI는 결과를 슬라이더에 되돌려 슈터 자세가 보이게 유지한다.)
    pub fn randomized(&self, rng: &mut impl Rng) -> Self {
        for _ in 0..RANDOM_SHOT_NET_GATE_MAX_TRIES {
            let shot = self.sample_randomized_params(rng);
            if shot.clears_incoming_net_gate() {
                return shot;
            }
        }
        // 최후: 조준만 랜덤, pitch/높이/스핀은 기본(검증된 네트 통과) 값.
        let defaults = Self::default();
        let mut shot = self.randomized_aim(rng);
        shot.pitch_deg = defaults.pitch_deg;
        shot.roll_deg = defaults.roll_deg;
        shot.height_offset_m = defaults.height_offset_m;
        shot.topspin_rad_s = defaults.topspin_rad_s;
        shot.sidespin_rad_s = defaults.sidespin_rad_s;
        return shot;
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
    use rand::SeedableRng;

    #[test]
    fn visual_body_sits_outside_table_end() {
        let s = BallShooterSettings::default();
        let visual = s.visual_position();
        // 본체 중심은 테이블 끝 밖(+y).
        assert!(
            visual.y > table::LENGTH_Y as f32,
            "visual center should be past table end, y={}",
            visual.y
        );
        let muzzle = s.muzzle_position();
        // 발사구는 끝선 근처(테이블 위/경계).
        assert!(
            (muzzle.y - table::LENGTH_Y as f32).abs() < 0.05,
            "muzzle should be near table end, y={}",
            muzzle.y
        );
    }

    #[test]
    fn visual_front_face_matches_muzzle() {
        let s = BallShooterSettings::default();
        let (forward, _, _) = s.local_basis();
        let front = s.visual_position() + forward * (ShooterLayout::VISUAL_SIZE_Y * 0.5) as f32;
        let muzzle = s.muzzle_position();
        assert!(
            (front - muzzle).length_squared() < 1e-10,
            "front={front:?} muzzle={muzzle:?}"
        );
    }

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
    fn randomized_varies_aim_height_spin() {
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
            assert!((RANDOM_SHOT_HEIGHT_MIN_M..=RANDOM_SHOT_HEIGHT_MAX_M)
                .contains(&shot.height_offset_m));
            assert!((RANDOM_SHOT_TOPSPIN_MIN..=RANDOM_SHOT_TOPSPIN_MAX)
                .contains(&shot.topspin_rad_s));
            assert!((RANDOM_SHOT_SIDESPIN_MIN..=RANDOM_SHOT_SIDESPIN_MAX)
                .contains(&shot.sidespin_rad_s));
            assert!((RANDOM_SHOT_PITCH_MIN_DEG..=RANDOM_SHOT_PITCH_MAX_DEG)
                .contains(&shot.pitch_deg));
            assert!((RANDOM_SHOT_ROLL_MIN_DEG..=RANDOM_SHOT_ROLL_MAX_DEG)
                .contains(&shot.roll_deg));
            let (yaw_min, yaw_max) =
                BallShooterSettings::yaw_range_for_lateral_deg(shot.lateral_offset_m);
            assert!(shot.yaw_deg >= yaw_min - 1e-9 && shot.yaw_deg <= yaw_max + 1e-9);

            assert_eq!(shot.pos_offset_x_m, base.pos_offset_x_m);
            assert_eq!(shot.pos_offset_y_m, base.pos_offset_y_m);
            assert_eq!(shot.pos_offset_z_m, base.pos_offset_z_m);
            assert_eq!(shot.drill_spin_rad_s, base.drill_spin_rad_s);
            assert!(
                shot.clears_incoming_net_gate(),
                "randomized는 네트 게이트를 통과하는 샷만 반환해야 함: {shot:?}"
            );
        }
    }

    #[test]
    fn sample_without_gate_often_clips_net_but_randomized_does_not() {
        let base = BallShooterSettings::default();
        let mut rng = rand::rngs::StdRng::seed_from_u64(42);
        let mut raw_clips = 0;
        for _ in 0..80 {
            if !base.sample_randomized_params(&mut rng).clears_incoming_net_gate() {
                raw_clips += 1;
            }
        }
        assert!(
            raw_clips > 5,
            "전제: 필터 없는 샘플 중 네트 미달이 있어야 함 (clips={raw_clips})"
        );

        let mut rng = rand::rngs::StdRng::seed_from_u64(42);
        for _ in 0..80 {
            let shot = base.randomized(&mut rng);
            assert!(
                shot.clears_incoming_net_gate(),
                "필터 후 샷이 네트 게이트 미달: {shot:?}"
            );
        }
    }
}
