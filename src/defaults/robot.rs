//! 활성 로봇 · URDF 프리셋.
//!
//! 런타임이 쓰는 것은 [`robot`]. 바꾸려면 그 본문만 고친다.
//! 리니어모터 철제 프레임 위치는 [`rail_frame`].
//!
//! 공유·배선은 항상 [`Robot`] (`shared_robot`). FK/IK가 필요하면 `robot.arm`을 본다.

use std::path::PathBuf;
use std::sync::Arc;

use nalgebra::{Isometry3, Matrix3, UnitQuaternion, Vector3};

use crate::Point3;
use crate::constants::geometry;
use crate::constants::table;
use crate::hardware::dynamixel::{DYNAMIXEL_MAX_JOINT_SPEED_RAD_S, joint_torque_limits_4dof};
use crate::robot::{
    Arm, JointLimit, Joints, LinkInertial, MountPreset, RailFrame, Robot, RobotBuildError,
    RobotBuilder, SerialChain, SerialJoint,
};

/// 리니어 레일 최대 속도 [m/s].
///
/// 이전 `12.0`은 근거 없는 리터럴이었다 — 테이블 전폭(1.525 m)을 0.127초에
/// 주파해, rough-to-fine 추종이 예측 방향으로 레일을 미리 옮길 때 렌더 프레임
/// 상 순간이동처럼 보였다(육안 확인, 2026-07-23). 실기
/// `config/real-hardware.toml`의 `[hardware.rail]` `vel`/`max_vel` = 5.0 m/s에
/// 맞춰 재보정 — 전폭 주파 0.305초로, 연속적인 움직임으로 보인다.
pub const RAIL_MAX_SPEED: f64 = 5.0;

/// 4-DOF 휴지(ready) 자세 [rad] — yaw, 어깨, 팔꿈치, 손목 순.
///
/// **왜 관절 한계 중점(예전 값)이 아닌가**: 스윙 커밋 실패의 실제 병목은
/// 임팩트 자세의 도달성이 아니라 *휴지 자세 → 임팩트 자세* 관절공간 이동에
/// 걸리는 시간이다(2026-07-23 2차 조사, `.omc/research/
/// known-regressions-realistic-joint-speed.md` §1). 재보정된 관절속도
/// (~2.88 rad/s, [`DYNAMIXEL_MAX_JOINT_SPEED_RAD_S`]) 아래서 quintic 소요시간은
/// **가장 많이 움직이는 한 관절**이 결정하므로, 휴지 자세는 "중립적으로
/// 보이는" 곳이 아니라 *실제로 마주칠 임팩트 자세들까지의 최악 이동거리가
/// 최소*인 곳이어야 한다.
///
/// 값 산출(`cargo run -p shot-tune --release -- --robot 4-dof --rest-pose-search`):
/// 테이블 폭 전역(x 10~90%) × 접수 창(y 0.20~0.55) × 실현가능 높이 대역
/// (테이블 위 10~30cm) × 대표 입사속도 3종 = 240 시나리오 중 IK 해가 있는
/// 165개의 임팩트 자세를 모아, 관절마다 그 각도 구간의 중점(1D Chebyshev
/// 중심)을 취했다. 비용 `max_시나리오 max_관절 |Δq|`는 두 max가 교환 가능해
/// 관절별로 분리되므로 이 중점이 **정확한** minimax 최적해다(근사가 아님).
///
/// 효과(실측): 최악 Δq 2.00 rad → **1.183 rad**, 필요시간 1.30s → **0.770s**.
/// 이것만으로는 commit 창(0.125~0.175s)에 못 들어와, rough 단계 관절
/// 선추종(`plan_coarse_track` + `RobotState::set_targets`)과 **함께** 쓴다.
pub const READY_JOINTS_4DOF: [f64; 4] = [0.1207, 0.0, 0.1719, -0.6756];

/// 리니어모터를 받치는 철제 프로파일 (탁구대 끝면·윗면 기준).
///
/// `mount_search`(2026-07-26): 현 `behind=0.02`는 ratio≤1이 **0/150**, mean≈3.79.
/// `behind=0.10`(height=0.05)는 **10/150**, mean≈2.48 — 임팩트 끝속도 스케일
/// (`NEAR_SINGULARITY` 2.5) 직전에 들어와 약한 스윙을 줄인다. 더 뒤(0.12)도
/// 비슷하나, 예전 고원(`base_y` −0.10..−0.02)의 바깥쪽 끝을 고름.
/// height 0.05는 실기 브래킷(~면 위 3~5cm)과 맞춤. 슈터는 `shot_tune`으로 재확인.
pub fn rail_frame() -> RailFrame {
    return RailFrame {
        behind_table_end: 0.10,
        above_table: 0.05,
    };
}

/// 경연용 단순 4-dof (URDF 없음) → [`Robot`].
///
/// mesh가 필요하면 [`urdf_4dof`]. 활성 배선은 [`robot`].
pub fn primitive_4dof() -> Result<Robot, RobotBuildError> {
    let frame = rail_frame();
    return primitive_4dof_with_mount(frame.mount_y(), frame.mount_z());
}

/// [`primitive_4dof`]와 같지만 레일 마운트 위치(y·z)를 직접 지정한다 —
/// `tools/mount_search`류 마운트 위치 스윕 전용.
///
/// `mount_y`: 베이스 y [m], 탁구대 로봇쪽 끝(y=0) 기준. 음수면 테이블 바깥.
/// `mount_z`: 베이스 z [m] (월드). 기본 배치는 [`rail_frame`]이 계산한다.
pub fn primitive_4dof_with_mount(mount_y: f64, mount_z: f64) -> Result<Robot, RobotBuildError> {
    let joints = vec![
        SerialJoint::new(
            Isometry3::translation(-0.02575, 0.028, 0.0601),
            Vector3::new(-1.0, 0.0, 0.0),
        )
        .expect("4-dof q0 axis"),
        SerialJoint::new(
            Isometry3::translation(0.0255, 0.0, 0.0825),
            Vector3::new(0.0, 0.0, -1.0),
        )
        .expect("4-dof q1 axis"),
        SerialJoint::new(
            Isometry3::translation(0.0, 0.025, 0.1398),
            Vector3::new(-1.0, 0.0, 0.0),
        )
        .expect("4-dof q2 axis"),
        SerialJoint::new(
            Isometry3::translation(0.0, 0.1518, 0.0),
            Vector3::new(-1.0, 0.0, 0.0),
        )
        .expect("4-dof q3 axis"),
    ];

    let chain = SerialChain::new(
        UnitQuaternion::identity(),
        joints,
        // CAD tip: +Y=면 법선, −Z=손잡이(면 내, 홈 포즈 기준). 타격점은 면 평행 이동.
        Isometry3::translation(
            0.0,
            -geometry::RACKET_HALF_Z,
            -geometry::RACKET_HANDLE_LENGTH,
        ),
    )
    .expect("4-dof serial chain");

    let (link_inertials, aggregated_inertials) = primitive_4dof_inertials();

    let built = Arm::builder()
        .base_xyz(0.0, mount_y, mount_z)
        .linear_rail(mount_y, mount_z, 0.0, table::WIDTH_X, RAIL_MAX_SPEED)
        .serial_chain(
            chain,
            vec![
                None,
                Some(JointLimit::new(-0.523599, 0.523599)),
                Some(JointLimit::new(-2.007129, 1.48353)),
                Some(JointLimit::new(-2.094395, 2.094395)),
            ],
            link_inertials,
            // 휴지 자세는 근거 있는 SSOT 상수 — 산출 근거는 그쪽 주석 참고.
            Joints::from_slice(&READY_JOINTS_4DOF),
        )
        .aggregated_inertials(aggregated_inertials)
        .joint_torque_limits(joint_torque_limits_4dof())
        .max_joint_speed(DYNAMIXEL_MAX_JOINT_SPEED_RAD_S)
        .build()
        .map_err(|e| RobotBuildError::ArmConversion {
            reason: e.to_string(),
        })?;

    return Ok(Robot::from_arm(built));
}

/// `all-4-export.urdf`에서 읽어온 4-dof 링크 관성 — `(원본 child link, 합성 강체)`.
///
/// 첫 값은 각 revolute 관절이 움직이는 child link `<inertial>` 원본
/// (질량 [kg], 질량중심 xyz [m], 관성텐서 ixx/ixy/ixz/iyy/iyz/izz [kg*m^2];
/// 전부 rpy=0). 둘째 값은 그 child link + 다음 revolute 관절까지의 fixed
/// 하위 링크(모터 몸체/브래킷/패들)를 평행축 정리로 합친 "실제로 움직이는
/// 강체"로, Newton-Euler 역동역학([`crate::planner::dynamics`])이 이 쪽을 쓴다.
/// 배치 변환은 URDF의 fixed joint(`Rigid N`) origin을 관절 child link
/// 프레임부터 누적한 값(전부 rpy=0이라 순수 평행이동).
fn primitive_4dof_inertials() -> (Vec<LinkInertial>, Vec<LinkInertial>) {
    let link_inertials = vec![
        // yaw: FR05-H101_v1__1__1
        LinkInertial {
            mass: 0.05198831685263556,
            com: Point3::new(
                0.02550000000002023,
                -1.1796119636642288e-16,
                0.0256313146478562,
            ),
            inertia: Matrix3::new(
                1.3e-05, 0.0, -0.0, //
                0.0, 2.7e-05, 0.0, //
                -0.0, 0.0, 2.6e-05,
            ),
        },
        // shoulder: FR05-H101_v1_1
        LinkInertial {
            mass: 0.05198831685263556,
            com: Point3::new(
                2.024828872279616e-14,
                -1.3530843112619095e-16,
                0.010168685352143825,
            ),
            inertia: Matrix3::new(
                1.3e-05, -0.0, 0.0, //
                -0.0, 2.7e-05, -0.0, //
                0.0, -0.0, 2.6e-05,
            ),
        },
        // elbow: FR07-H101_v1_1
        LinkInertial {
            mass: 0.025998108201265576,
            com: Point3::new(
                -3.365828433557125e-06,
                0.021380623885861517,
                6.089573290068984e-14,
            ),
            inertia: Matrix3::new(
                4e-06, -0.0, 0.0, //
                -0.0, 9e-06, 0.0, //
                0.0, 0.0, 1e-05,
            ),
        },
        // wrist: FR07-H101_v1__1__1
        LinkInertial {
            mass: 0.025998108201265576,
            com: Point3::new(
                -3.3658284336170272e-06,
                0.021380623885861483,
                6.078471059822732e-14,
            ),
            inertia: Matrix3::new(
                4e-06, -0.0, 0.0, //
                -0.0, 9e-06, 0.0, //
                0.0, 0.0, 1e-05,
            ),
        },
    ];
    let aggregated_inertials = vec![
        // yaw child(FR05-H101) + Rigid7→FR05-B101 + Rigid7·Rigid8→MX-64R 몸체.
        LinkInertial::combine(&[
            (Isometry3::identity(), link_inertials[0]),
            (
                Isometry3::translation(0.0255, 0.0, 0.036),
                LinkInertial {
                    mass: 0.01879497598985593,
                    com: Point3::new(
                        -4.5090504680739274e-14,
                        -0.0029557693246398953,
                        0.0016902214716354585,
                    ),
                    inertia: Matrix3::new(
                        2e-06, 0.0, 0.0, //
                        0.0, 4e-06, 0.0, //
                        0.0, 0.0, 5e-06,
                    ),
                },
            ),
            (
                Isometry3::translation(0.0082, 0.004, 0.042),
                LinkInertial {
                    mass: 0.126,
                    com: Point3::new(
                        0.017300000017253583,
                        -0.019207753529397596,
                        0.017451641868345094,
                    ),
                    inertia: Matrix3::new(
                        5.186e-05, 0.0, 0.0, //
                        0.0, 2.948e-05, -1.551e-06, //
                        0.0, -1.551e-06, 4.344e-05,
                    ),
                },
            ),
        ]),
        // shoulder child(FR05-H101) + Rigid10→arm_v9 + Rigid11→FR07-S101 + Rigid12→MX-28T 몸체.
        LinkInertial::combine(&[
            (Isometry3::identity(), link_inertials[1]),
            (
                Isometry3::translation(-0.0235, 0.0, 0.0248),
                LinkInertial {
                    mass: 0.027,
                    com: Point3::new(
                        0.02362282770461404,
                        1.947747047686965e-05,
                        0.05189568925169269,
                    ),
                    inertia: Matrix3::new(
                        2.666e-05, 0.0, 1.11e-07, //
                        0.0, 3.21e-05, 0.0, //
                        1.11e-07, 0.0, 1.077e-05,
                    ),
                },
            ),
            (
                Isometry3::translation(0.0, -0.008, 0.1188),
                LinkInertial {
                    mass: 0.011446844551351427,
                    com: Point3::new(
                        -2.0825301118992945e-14,
                        0.008467333868896306,
                        0.002214506791986759,
                    ),
                    inertia: Matrix3::new(
                        1e-06, 0.0, 0.0, //
                        0.0, 2e-06, 0.0, //
                        0.0, 0.0, 2e-06,
                    ),
                },
            ),
            (
                Isometry3::translation(-0.015, -0.0045, 0.1248),
                LinkInertial {
                    mass: 0.072,
                    com: Point3::new(
                        0.015031845145486198,
                        0.017984471617669542,
                        0.014999999976589629,
                    ),
                    inertia: Matrix3::new(
                        1.717e-05, 2.12e-07, 0.0, //
                        2.12e-07, 1.251e-05, 0.0, //
                        0.0, 0.0, 2.035e-05,
                    ),
                },
            ),
        ]),
        // elbow child(FR07-H101) + Rigid19→arm2_v2 + Rigid16→FR07-S101 + Rigid17→MX-28T 몸체.
        LinkInertial::combine(&[
            (Isometry3::identity(), link_inertials[2]),
            (
                Isometry3::translation(0.007778, 0.03, 0.007778),
                LinkInertial {
                    mass: 0.0217,
                    com: Point3::new(
                        -0.007777999999573574,
                        0.03999999999999991,
                        -0.007778000000000118,
                    ),
                    inertia: Matrix3::new(
                        1.841e-05, 0.0, 0.0, //
                        0.0, 4.818e-06, 0.0, //
                        0.0, 0.0, 1.841e-05,
                    ),
                },
            ),
            (
                Isometry3::translation(0.0, 0.11, 0.0),
                LinkInertial {
                    mass: 0.011446844551351427,
                    com: Point3::new(
                        2.0689287956454638e-14,
                        0.00221450679198662,
                        0.000467333868896469,
                    ),
                    inertia: Matrix3::new(
                        1e-06, 0.0, 0.0, //
                        0.0, 2e-06, 0.0, //
                        0.0, 0.0, 2e-06,
                    ),
                },
            ),
            (
                Isometry3::translation(-0.015, 0.116, -0.0085),
                LinkInertial {
                    mass: 0.072,
                    com: Point3::new(
                        0.015031845145486136,
                        0.024284471617669556,
                        0.008499999976589567,
                    ),
                    inertia: Matrix3::new(
                        1.717e-05, 2.12e-07, 0.0, //
                        2.12e-07, 1.251e-05, 0.0, //
                        0.0, 0.0, 2.035e-05,
                    ),
                },
            ),
        ]),
        // wrist child(FR07-H101) + Rigid14→racket_joint + Rigid14·Rigid15→pingpong_paddle.
        LinkInertial::combine(&[
            (Isometry3::identity(), link_inertials[3]),
            (
                Isometry3::translation(-0.007778, 0.03, -0.007778),
                LinkInertial {
                    mass: 0.0265,
                    com: Point3::new(
                        0.007777999999983611,
                        0.01501142035517336,
                        -0.0015451576233940778,
                    ),
                    inertia: Matrix3::new(
                        8.635e-06, 0.0, 0.0, //
                        0.0, 1.349e-05, 0.0, //
                        0.0, 0.0, 1.053e-05,
                    ),
                },
            ),
            (
                Isometry3::translation(0.0, 0.0513, -0.034),
                LinkInertial {
                    mass: 0.1729,
                    com: Point3::new(
                        6.7342507438505894e-15,
                        -0.006399999999999961,
                        -0.046816094811444026,
                    ),
                    inertia: Matrix3::new(
                        0.0006375, 0.0, 0.0, //
                        0.0, 0.0008405, 0.0, //
                        0.0, 0.0, 0.0002094,
                    ),
                },
            ),
        ]),
    ];
    return (link_inertials, aggregated_inertials);
}

/// `robot()`을 `Arc`로 (파이프라인·테스트용).
pub fn shared_robot() -> Arc<Robot> {
    return Arc::new(robot().expect("defaults::robot"));
}

/// `assets/robots/4-dof` URDF 프리셋 (진단·비교용).
pub fn urdf_4dof() -> Result<Robot, RobotBuildError> {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("assets/robots/4-dof/urdf/all-4-export.urdf");
    return RobotBuilder::new()
        .urdf(&path)
        .ee_link_opt(Some("pingpong_paddle_v5_1"))
        .mount_preset(MountPreset::Rep103AtTableEnd)
        .max_joint_speed(DYNAMIXEL_MAX_JOINT_SPEED_RAD_S)
        .build();
}

/// `assets/robots/urdf-test` 프리셋.
pub fn urdf_test() -> Result<Robot, RobotBuildError> {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("assets/robots/urdf-test/urdf-test_description/urdf/urdf-test.urdf");
    return RobotBuilder::new()
        .urdf(&path)
        .ee_link_opt(Some("pingpong_paddle_v5_1"))
        .mount_preset(MountPreset::Rep103AtTableEnd)
        .max_joint_speed(DYNAMIXEL_MAX_JOINT_SPEED_RAD_S)
        .build();
}

/// **지금 쓰는 로봇.** 바꾸려면 이 함수 본문만 고친다 (`urdf_4dof` 등).
pub fn robot() -> Result<Robot, RobotBuildError> {
    return urdf_4dof();
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::constants::table;

    #[test]
    fn rail_frame_mounts_behind_and_above_table() {
        let frame = rail_frame();
        assert!((frame.mount_y() - (-0.10)).abs() < 1e-12);
        assert!((frame.mount_z() - (table::SURFACE_Z + 0.05)).abs() < 1e-12);
        assert_eq!(frame.mount_xyz0(), [0.0, -0.10, table::SURFACE_Z + 0.05]);
    }

    #[test]
    fn primitive_follows_rail_frame() {
        let robot = primitive_4dof().expect("primitive_4dof");
        let arm = robot.arm.as_ref();
        let frame = rail_frame();
        assert!((arm.base.coords.y - frame.mount_y()).abs() < 1e-12);
        assert!((arm.base.coords.z - frame.mount_z()).abs() < 1e-12);
        let rail = arm.rail.expect("rail");
        assert!((rail.mount_y - frame.mount_y()).abs() < 1e-12);
        assert!((rail.mount_z - frame.mount_z()).abs() < 1e-12);
    }
}
