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
use crate::defaults::dxl_limits::{
    DYNAMIXEL_MAX_JOINT_SPEED_RAD_S, joint_reflected_inertias_4dof, joint_torque_limits_4dof,
};
use crate::defaults::hardware::{RAIL_READY_X_M, RAIL_X_MAX_M, RAIL_X_MIN_M};
use crate::robot::{
    Arm, JointLimit, Joints, LinkInertial, MountPreset, RailFrame, Robot, RobotBuildError,
    RobotBuilder, SerialChain, SerialJoint,
};

/// 리니어 레일 최대 속도 [m/s].
///
/// 발사기 실기 시험 기준값 7.5 m/s. 계획기와 AXL 설정이 공유한다.
/// 기존의 1.5배 오버드라이브는 제거해 레일 보정 때 갑자기 튀는 동작을 줄인다.
pub const RAIL_MAX_SPEED: f64 = 7.5;

/// 4-DOF 휴지(ready) 자세 [rad] — yaw, 어깨, 팔꿈치, 손목 순.
///
/// **왜 관절 한계 중점이 아닌가**: 스윙 커밋 실패의 실제 병목은 임팩트
/// 자세의 도달성이 아니라 *휴지 자세 → 임팩트 자세* 관절공간 이동에
/// 걸리는 시간이다(2026-07-23 2차 조사, `.omc/research/
/// known-regressions-realistic-joint-speed.md` §1). 재보정된 관절속도
/// (~5.18 rad/s, [`DYNAMIXEL_MAX_JOINT_SPEED_RAD_S`]) 아래서 quintic 소요시간은
/// **가장 많이 움직이는 한 관절**이 결정하므로, 휴지 자세는 "중립적으로
/// 보이는" 곳이 아니라 *실제로 마주칠 임팩트 자세들까지의 최악 이동거리가
/// 최소*인 곳이어야 한다.
///
/// **2026-07-30 재계산 (`diag_windup_rest_pose_search`,
/// `src/robot/motion/physics.rs`).** 사용자 관찰: GUI에서 스윙마다 라켓이
/// "뒤로 당겨지는" 동작이 반복된다 — 팔로스루가 임팩트 방향으로 더 나아간
/// 뒤 `plan_return_to_center`가 중립 휴지 자세로 역방향 복귀하기 때문으로
/// 보인다. 가설: 휴지 자세를 미리 "당겨진(backswing)" 자세로 잡으면
/// 나아질까 — **직접 검증한 결과 반대였다.** 임팩트 관절각에서 명령
/// 관절속도 방향으로 시간 `T_w`만큼 되감아 만든 windup 자세들의 Chebyshev
/// 중심은 `T_w`가 커질수록 최악 Δq가 **단조 증가**했다(T_w=0→0.28s에서
/// 1.105→1.924 rad) — 240개 대표 임팩트가 테이블 전역에 걸쳐 서로 다른
/// 방향을 향해, 단일 방향으로 되감으면 표본이 중심에서 더 퍼진다. 그래서
/// 백스윙 오프셋은 **채택하지 않는다**(`T_w=0`, 즉 임팩트 자세 자체의
/// Chebyshev 중심이 최선).
///
/// 다만 재계산 자체(`T_w=0`)는 값을 바꿀 근거가 됐다: 예전 값은
/// `plan_coarse_track`(위치 3제약 단일 IK)로 냈는데, 실제 스윙 목표는
/// `solve_impact_target`(다중 시드 조작성 최적화, `best_impact_candidate`)이
/// 낸다 — 계산 방식이 실제 커밋 경로와 어긋나 있었다. 실제 경로와 일치하는
/// 방식으로 같은 240 시나리오(118개 IK 해)를 다시 돌리자 **최악 Δq가
/// 1.280→1.105 rad로 자연히 줄었다**(재계산 자체의 개선, 백스윙과 무관).
/// GUI의 "당겨지는" 동작은 별도 원인(팔로스루→복귀 반전)으로 보이며
/// 후속 조사 대상.
///
/// 예전 값 산출(`cargo run -p shot-tune --release -- --robot 4-dof
/// --rest-pose-search`, `plan_coarse_track` 기준, 165/240 해결):
/// 최악 Δq 2.00 rad → 1.183 rad, 필요시간 1.30s → 0.770s.
///
/// 이것만으로는 commit 창에 다 못 들어와, rough 단계 관절 선추종
/// (`plan_coarse_track` + `RobotState::slew_targets_toward`)과 **함께** 쓴다.
///
/// # 2026-07-30 재산출 필요 — 미착수 (스윙 튜닝 담당 몫)
///
/// [`rail_frame`]에 실측을 반영해 베이스 z가 0.81→0.935로 올라갔다. 아래 값은
/// **낮은 베이스에서 뽑은 것**이라 현재 마운트에서는 최적이 아니다. 새 마운트에서
/// `diag_windup_rest_pose_search`를 돌려 확인한 수치:
///
/// | | 최악 Δq | 필요시간 |
/// |---|---|---|
/// | 아래 값(낮은 베이스 산출) | 1.282 rad | 0.835s |
/// | 같은 방식으로 재산출 `[0.8612, 0.0, 0.1889, -1.2076]` | 0.767 rad | 0.499s |
///
/// 도달성 자체도 나빠졌다(IK 해 118/240 → 91/240) — 베이스를 올린 대가다.
///
/// 값을 바꾸면 딸려오는 것들: [`JOINT_EFFECTIVE_INERTIA_4DOF`]
/// (crate::defaults::JOINT_EFFECTIVE_INERTIA_4DOF) 재측정(휴지 자세가 mass matrix
/// 대각을 바꾼다), `robot::tests::default_arm_produces_racket_pose`(재산출 값에서는
/// 라켓이 베이스보다 아래로 내려온다 — 임팩트 대역에 가까워지므로 정상이지만
/// 그 단정문이 실패한다), 그리고 `mount_search`로 `mount_y` 재스윕.
/// **2026-08-05 실기 동기화.** 라켓 최하단 15.5 cm·장축 8°가
/// 실측과 일치한 두 번의 안정 도달값을 평균했다. 예전 계획기 최적화값을
/// 계속 명령하면 실물은 항상 약 1–2.5° 다른 자세에 멈춰 sim·real
/// 시작 자세가 어긋났다. 이 값은 기본 자세에서 그 괴리를 없앤다.
pub const READY_JOINTS_4DOF: [f64; 4] = [0.5269, -0.0023, -0.1641, -0.6849];

/// 타격 후 리니어 레일 x를 유지한 채 관절만 사진의 높은 아치로 접는 복귀 자세.
/// 상완은 위로 세우고 팔꿈치 이후는 아래로 접으며, 라켓 장축은 READY와 같은
/// 실측 8°를 유지한다. 활성 URDF 기준 최고 관절은 마운트보다 약 27 cm 높고
/// 라켓 중심은 마운트와 거의 같은 높이다.
pub const POST_HIT_TUCKED_JOINTS_4DOF: [f64; 4] = [0.05, 0.0, 1.2, -1.5721];

/// 리니어모터를 받치는 철제 프로파일 (탁구대 끝면·바닥 기준).
///
/// **높이는 실측(2026-07-30).** 바닥→프로파일 하단 0.88 m,
/// 두께 [`RAIL_THICKNESS`](crate::constants::geometry::RAIL_THICKNESS) 0.055 m →
/// 베이스 z = **0.935**. 이전 값은 `SURFACE_Z + 0.05` = 0.81로, "실기 브래킷
/// (~면 위 3~5cm)과 맞춤"이라는 추정에 기대고 있었는데 실측이 그 가정을
/// 뒤집었다 — 시뮬 베이스가 실물보다 12.5 cm 낮았다.
///
/// `mount_y`는 실측값 **-0.128**을 쓴다 — `mount_search`(2026-07-26)가 낮은
/// 베이스 기준으로 추천한 `behind=0.10`(y=−0.10, `behind=0.02` 대비 ratio≤1이
/// **10/150**, mean≈2.48)은 그 스윕이 **낮은 베이스 기준**이라 0.935에서는
/// 최적값이 아니었고, 이후 실측이 이 값으로 대체했다.
///
/// 두 값 모두 sim GUI "Rig" 패널에서 공이 주차된 동안 런타임 조정 가능하다
/// (`SimRuntimeControls::rail_frame`). 좋은 위치를 눈으로 찾은 뒤
/// `mount_search`/`--rest-pose-search`를 그 위치에서 다시 돌려 여기와
/// [`READY_JOINTS_4DOF`]를 확정하는 것이 순서다.
pub fn rail_frame() -> RailFrame {
    return RailFrame {
        mount_y: -0.128,
        rail_bottom_z: 0.88,
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

    // primitive도 실측 피치를 적용해 URDF 활성 로봇과 같은 라켓 축을 쓴다.
    // 다만 primitive EE는 이미 손목→블레이드 중심 0.10 m를 포함하므로,
    // URDF 원점용 0.1111 m 추가 이동은 중복 적용하지 않는다.
    let link_from_racket = UnitQuaternion::from_axis_angle(
        &nalgebra::Unit::new_normalize(Vector3::new(0.0, 1.0, 1.0)),
        std::f64::consts::PI,
    );
    let mount_pitch = UnitQuaternion::from_axis_angle(
        &nalgebra::Unit::new_normalize(Vector3::x()),
        geometry::RACKET_MOUNT_PITCH_CORRECTION_RAD,
    );
    let link_from_racket_iso =
        Isometry3::from_parts(nalgebra::Translation3::identity(), link_from_racket);
    let ee_transform = link_from_racket_iso
        * Isometry3::from_parts(nalgebra::Translation3::identity(), mount_pitch)
        * Isometry3::translation(
            0.0,
            -geometry::RACKET_HANDLE_LENGTH,
            -geometry::RACKET_HALF_Z,
        )
        * link_from_racket_iso.inverse();
    let chain = SerialChain::new(UnitQuaternion::identity(), joints, ee_transform)
        .expect("4-dof serial chain");

    let (link_inertials, aggregated_inertials) = primitive_4dof_inertials();

    let built = Arm::builder()
        .base_xyz(0.0, mount_y, mount_z)
        .linear_rail(
            mount_y,
            mount_z,
            RAIL_X_MIN_M,
            RAIL_X_MAX_M,
            RAIL_READY_X_M,
            RAIL_MAX_SPEED,
        )
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
        // URDF/CAD 관성에는 모터 회전자가 안 들어 있다 — 감속기 뒤에서 본
        // 반사관성(`I_rotor·N²`)을 별도로 채운다(WP8).
        .joint_reflected_inertias(joint_reflected_inertias_4dof())
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
/// 강체"로, Newton-Euler 역동역학([`crate::robot::dynamics`])이 이 쪽을 쓴다.
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

    /// 2026-08-05 버니어 실측(손잡이 끝이 로봇 쪽으로 8°)을 라켓
    /// 장착 변환의 회귀 기준으로 고정한다.
    #[test]
    fn ready_racket_mount_matches_bench_geometry() {
        let robot = urdf_4dof().expect("4-dof");
        let arm = robot.arm.as_ref();
        let rail_x = arm
            .rail
            .as_ref()
            .map_or(RAIL_READY_X_M, |rail| rail.default_x());
        let pose = arm
            .forward_kinematics_with_rail(rail_x, &arm.default_joints)
            .expect("ready FK");
        let [w, x, y, z] = pose.orientation;
        let rotation = UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(w, x, y, z));
        let axis_x = rotation * Vector3::x();
        let blade_axis = rotation * Vector3::y();
        let axis_normal = rotation * Vector3::z();
        let angle_deg = blade_axis.z.abs().acos().to_degrees();
        let half_height = axis_x.z.abs() * geometry::RACKET_HALF_X
            + blade_axis.z.abs() * geometry::RACKET_HALF_Y
            + axis_normal.z.abs() * geometry::RACKET_HALF_Z;
        let lowest = pose.position.z - crate::constants::table::SURFACE_Z - half_height;

        assert!(
            (angle_deg - 8.0).abs() < 0.15,
            "bench angle=8deg, model={angle_deg:.3}deg"
        );
        assert!(
            blade_axis.y < 0.0 && blade_axis.z > 0.0,
            "손잡이 쪽(+blade Y)은 로봇(-world Y)·위(+Z)로 기울어야 함: {blade_axis:?}"
        );
        assert!(
            pose.normal.y > 0.0 && pose.normal.z >= 0.0,
            "라켓 면은 상대 탁구대(+Y)를 보고 아래로 기울지 않아야 함: {:?}",
            pose.normal
        );
        assert!(
            (lowest - 0.155).abs() < 0.003,
            "bench lowest=0.155m, model={lowest:.4}m"
        );
    }

    #[test]
    fn post_hit_pose_forms_high_arch_without_table_collision() {
        let robot = urdf_4dof().expect("4-dof");
        let arm = robot.arm.as_ref();
        let rail_x = arm
            .rail
            .as_ref()
            .map_or(RAIL_READY_X_M, |rail| rail.default_x());
        let joints = Joints::from_slice(&POST_HIT_TUCKED_JOINTS_4DOF);
        let chain = arm.chain_points(rail_x, &joints).expect("post-hit chain");
        let mount_z = chain.first().expect("mount").z;
        let peak_z = chain.iter().map(|point| point.z).fold(mount_z, f64::max);
        let mount = chain.first().expect("mount");
        let racket = chain.last().expect("racket");
        let racket_z = racket.z;

        assert!(peak_z - mount_z > 0.25, "팔꿈치가 충분히 높아야 함");
        assert!(
            peak_z - racket_z > 0.25,
            "높은 아치 뒤 라켓이 아래로 접혀야 함"
        );
        assert!(
            (racket_z - mount_z).abs() < 0.02,
            "라켓 중심은 마운트 높이 근처"
        );
        assert!(
            (racket.x - mount.x).hypot(racket.y - mount.y) < 0.20,
            "라켓이 몸통 가까이 접혀야 함"
        );
        assert!(
            crate::robot::collision::table_penetration(arm, rail_x, &joints) <= 1e-4,
            "복귀 자세가 테이블을 관통하면 안 됨"
        );
    }
}
