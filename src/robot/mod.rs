//! 로봇 팔 기구학.
//!
//! `Arm`은 sim/real이 같이 쓰는 불변 기하 모델이다. 부팅 때 한 번 만들고
//! `Arc<Arm>`으로 넘긴다. FK/IK랑 스윙 계획은 이 타입만 본다.
//!
//! 조립은 `ArmBuilder`, 런타임 추종은 `RobotState`.

pub mod builder;
pub mod catalog;
mod loader;
pub mod rail;
pub mod serial;
pub mod state;
pub mod urdf;

#[cfg(test)]
mod tests;

use nalgebra::{DMatrix, DVector, Isometry3, Matrix3, UnitQuaternion, Vector3};

pub use builder::{ArmBuildError, ArmBuilder};
pub use catalog::{
    DEFAULT_ROBOT_ID, ROBOTS, RobotEntry, find_robot, robot_ids_csv, shared_competition_arm,
};
pub use loader::{MountPreset, RobotBuildError, RobotBuilder, SimRobot};
pub use serial::{SerialChain, SerialChainError, SerialJoint};
pub use state::RobotState;
pub use urdf::{UrdfGeometry, UrdfLinkVisual, UrdfLoadError, UrdfRobot};

use self::rail::LinearRail;
use crate::error::SwingPlanError;
use crate::geometry::Point3;

/// revolute 관절각 [rad].
#[derive(Debug, Clone, PartialEq)]
pub struct Joints {
    pub values: Vec<f64>,
}

impl Joints {
    pub fn from_slice(values: &[f64]) -> Self {
        return Self {
            values: values.to_vec(),
        };
    }
}

/// 레일 x + 팔 관절각 스냅샷 (`plan_swing` 입력).
#[derive(Debug, Clone, PartialEq)]
pub struct RobotPose {
    pub rail_x: f64,
    pub joints: Joints,
}

impl RobotPose {
    pub fn new(rail_x: f64, joints: Joints) -> Self {
        return Self { rail_x, joints };
    }
}

/// revolute 관절 1축 허용 범위 [rad].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct JointLimit {
    /// 최소 각도 [rad]
    pub min: f64,
    /// 최대 각도 [rad]
    pub max: f64,
}

impl JointLimit {
    /// [min, max] 범위를 만든다.
    pub const fn new(min: f64, max: f64) -> Self {
        return Self { min, max };
    }

    /// 각도가 허용 범위 안인지 확인한다.
    pub fn contains(self, angle: f64) -> bool {
        return angle >= self.min && angle <= self.max;
    }
}

/// 링크 관성 - URDF `<inertial>` 원본 (질량/질량중심/관성텐서).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LinkInertial {
    /// 질량 [kg]
    pub mass: f64,
    /// 질량중심 - 링크 로컬(URDF origin) 좌표계 [m]
    pub com: Point3,
    /// 질량중심 기준 관성 텐서 [kg*m^2]
    pub inertia: Matrix3<f64>,
}

impl LinkInertial {
    /// 공통 기준 프레임에 배치된 여러 강체를 하나의 등가 강체로 합성한다
    /// (평행축 정리). `bodies`의 각 원소는 `(배치 변환, 로컬 관성)`으로,
    /// 배치 변환은 그 강체의 로컬 프레임을 공통 기준 프레임에 놓는 `Isometry3`다.
    ///
    /// fixed joint로 붙은 하위 링크(모터 몸체 등)를 actuated child link와 합쳐
    /// 관절이 실제로 움직이는 강체의 질량/질량중심/관성텐서를 구할 때 쓴다.
    /// 반환 관성텐서는 합성 질량중심 기준, 공통 기준 프레임 축으로 표현된다.
    pub fn combine(bodies: &[(Isometry3<f64>, LinkInertial)]) -> LinkInertial {
        let total_mass: f64 = bodies.iter().map(|(_, body)| body.mass).sum();
        if total_mass <= 0.0 {
            return LinkInertial {
                mass: 0.0,
                com: Point3::new(0.0, 0.0, 0.0),
                inertia: Matrix3::zeros(),
            };
        }
        // 기준 프레임에서의 각 강체 질량중심 위치.
        let placed_com = |placement: &Isometry3<f64>, body: &LinkInertial| {
            placement.rotation * body.com.v + placement.translation.vector
        };
        let mut com = Vector3::zeros();
        for (placement, body) in bodies {
            com += body.mass * placed_com(placement, body);
        }
        com /= total_mass;

        let mut inertia = Matrix3::zeros();
        for (placement, body) in bodies {
            // 로컬 관성텐서를 기준 프레임 축으로 회전: R * I * Rᵀ.
            let rotation = placement.rotation.to_rotation_matrix();
            let rotated = rotation * body.inertia * rotation.transpose();
            // 평행축 정리로 합성 질량중심 기준으로 이동.
            let d = placed_com(placement, body) - com;
            let translated =
                body.mass * (Matrix3::identity() * d.dot(&d) - d * d.transpose());
            inertia += rotated + translated;
        }
        return LinkInertial {
            mass: total_mass,
            com: Point3::from(com),
            inertia,
        };
    }
}

/// 로봇 팔 불변 모델. sim/real/plan_swing이 같은 `Arm`을 참조한다.
#[derive(Debug, Clone, PartialEq)]
pub struct Arm {
    /// 베이스 원점 (월드 좌표) [m] - 리니어 레일이 있으면 y/z 마운트 기준, x는 무시
    pub base: Point3,
    /// X축 리니어 레일 (있으면 베이스 x는 `rail_x`로 이동)
    pub rail: Option<LinearRail>,
    /// revolute 축 순서대로의 링크 길이 [m] - `limits`/`default_joints`와 같은 길이
    pub link_lengths: Vec<f64>,
    /// 축별 관절 한계. `None`은 URDF continuous 관절.
    pub limits: Vec<Option<JointLimit>>,
    /// 축별 링크 관성 (질량/질량중심/텐서). `limits`/`link_lengths`와 같은 길이,
    /// revolute 관절이 움직이는 child link 기준 (fixed 하위 링크 미포함, URDF 원본).
    pub link_inertials: Vec<LinkInertial>,
    /// 축별 "합성" 강체 관성 - actuated child link + 다음 revolute 관절까지의
    /// fixed 하위 링크(모터 몸체 등)를 평행축 정리로 합친 값. Newton-Euler
    /// 역동역학이 실제로 관절이 움직이는 강체 질량으로 쓴다. `link_inertials`와
    /// 같은 길이, 각 관절 child link 로컬 프레임 기준.
    pub aggregated_inertials: Vec<LinkInertial>,
    /// 축별 관절 토크 한계 [N*m] - 모터 연속 토크 안전 한계.
    /// `limits`/`link_inertials`와 같은 길이. `f64::INFINITY`는 무제한.
    pub joint_torque_limits: Vec<f64>,
    /// 부팅 시 초기 관절각
    pub default_joints: Joints,
    /// 관절 추종 최대 각속도 [rad/s]
    pub max_joint_speed: f64,
    /// FK/IK 구현. URDF 로봇은 원본 고정 변환과 축을 보존한 직렬 체인을 쓴다.
    pub(crate) chain: SerialChain,
}

/// 월드 좌표계 라켓 자세 - sim/real 동일 표현.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RacketPose {
    /// 라켓 중심 위치 (월드)
    pub position: Point3,
    /// 라켓 면 법선 (단위 벡터)
    pub normal: Vector3<f64>,
    /// Hamilton 단위 쿼터니언 (w, x, y, z) - 어댑터가 SDK 회전으로 변환
    pub orientation: [f64; 4],
}

impl Arm {
    /// 빈 `ArmBuilder`를 반환한다.
    pub fn builder() -> ArmBuilder {
        return ArmBuilder::new();
    }

    /// URDF 등에서 보존한 일반 revolute 직렬 체인으로 팔을 만든다.
    #[allow(clippy::too_many_arguments)]
    pub fn from_serial_chain(
        base: Point3,
        rail: Option<LinearRail>,
        chain: SerialChain,
        limits: Vec<Option<JointLimit>>,
        link_inertials: Vec<LinkInertial>,
        aggregated_inertials: Vec<LinkInertial>,
        joint_torque_limits: Vec<f64>,
        default_joints: Joints,
        max_joint_speed: f64,
    ) -> Result<Self, ArmBuildError> {
        let chain_count = chain.joints.len();
        if chain_count != limits.len()
            || chain_count != default_joints.values.len()
            || chain_count != link_inertials.len()
            || chain_count != aggregated_inertials.len()
            || chain_count != joint_torque_limits.len()
        {
            return Err(ArmBuildError::KinematicsJointCountMismatch {
                chain: chain_count,
                limits: limits.len(),
                link_inertials: link_inertials.len(),
                defaults: default_joints.values.len(),
            });
        }
        for (joint_index, limit) in limits.iter().enumerate() {
            let Some(limit) = limit else {
                continue;
            };
            if limit.min > limit.max {
                return Err(ArmBuildError::InvalidJointLimit {
                    joint_index,
                    min: limit.min,
                    max: limit.max,
                });
            }
            let value = default_joints.values[joint_index];
            if !limit.contains(value) {
                return Err(ArmBuildError::DefaultJointOutOfRange {
                    joint_index,
                    value,
                    min: limit.min,
                    max: limit.max,
                });
            }
        }
        if max_joint_speed <= 0.0 {
            return Err(ArmBuildError::NonPositiveMaxJointSpeed {
                value: max_joint_speed,
            });
        }
        let link_lengths = chain
            .joints
            .iter()
            .map(|joint| joint.origin.translation.vector.norm())
            .collect();
        return Ok(Self {
            base,
            rail,
            link_lengths,
            limits,
            link_inertials,
            aggregated_inertials,
            joint_torque_limits,
            default_joints,
            max_joint_speed,
            chain,
        });
    }

    /// 경진용 4DOF URDF 체인을 fixed-link 단위로 합성한 primitive 모델.
    ///
    /// 축·관절 순서·한계·EE offset은 `all-4-export.urdf`와 같고 mesh만 생략한다.
    ///
    /// 앱에서 쓰는 프리셋은 `crate::pipeline::ROBOTS`.
    /// primitive 모델도 URDF 모델과 같은 직렬 체인을 쓴다.
    pub fn competition() -> Result<Self, ArmBuildError> {
        use crate::constants::arm::BASE_Y;
        return Self::competition_with_mount(BASE_Y, 0.0);
    }

    /// [`Arm::competition`]과 같지만 레일 마운트 위치(테이블과의 거리·높이)를
    /// 직접 지정한다 - `tools/mount_search`(마운트 위치 최적화 스윕) 전용.
    ///
    /// `base_y`: 베이스 y [m] - 테이블 로봇쪽 끝 기준(`BASE_Y` 관례와 동일 좌표계).
    /// `height_offset_m`: 테이블 면(`table::SURFACE_Z`) 대비 레일 마운트 높이
    /// 오프셋 [m] - 실기 AXL 레일 브래킷은 테이블 면보다 약 3cm 위에
    /// 설치돼 있다(2026-07-23, 실측 보고). 기본 [`Arm::competition`]은
    /// `0.0`(테이블 면과 같은 높이)을 쓰는데, 이는 실기와 다른 단순화라
    /// 마운트 스윕에서 실측치 중심으로 후보를 넓게 잡아야 한다.
    pub fn competition_with_mount(base_y: f64, height_offset_m: f64) -> Result<Self, ArmBuildError> {
        use crate::constants::arm::RAIL_MAX_SPEED;
        use crate::constants::control::{
            CONTINUOUS_TORQUE_DERATE, MX28_STALL_TORQUE_NM, MX64_STALL_TORQUE_NM,
        };
        use crate::constants::table;
        use crate::hardware::dynamixel::DYNAMIXEL_MAX_JOINT_SPEED_RAD_S;

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
            Isometry3::translation(0.0, 0.0513, -0.034),
        )
        .expect("4-dof serial chain");
        // `all-4-export.urdf`의 각 revolute 관절이 움직이는 child link `<inertial>` 원본값
        // (질량 [kg], 질량중심 xyz [m], 관성텐서 ixx/ixy/ixz/iyy/iyz/izz [kg*m^2]; 전부 rpy=0).
        let link_inertials = vec![
            // yaw: FR05-H101_v1__1__1
            LinkInertial {
                mass: 0.05198831685263556,
                com: Point3::new(0.02550000000002023, -1.1796119636642288e-16, 0.0256313146478562),
                inertia: Matrix3::new(
                    1.3e-05, 0.0, -0.0, //
                    0.0, 2.7e-05, 0.0, //
                    -0.0, 0.0, 2.6e-05,
                ),
            },
            // shoulder: FR05-H101_v1_1
            LinkInertial {
                mass: 0.05198831685263556,
                com: Point3::new(2.024828872279616e-14, -1.3530843112619095e-16, 0.010168685352143825),
                inertia: Matrix3::new(
                    1.3e-05, -0.0, 0.0, //
                    -0.0, 2.7e-05, -0.0, //
                    0.0, -0.0, 2.6e-05,
                ),
            },
            // elbow: FR07-H101_v1_1
            LinkInertial {
                mass: 0.025998108201265576,
                com: Point3::new(-3.365828433557125e-06, 0.021380623885861517, 6.089573290068984e-14),
                inertia: Matrix3::new(
                    4e-06, -0.0, 0.0, //
                    -0.0, 9e-06, 0.0, //
                    0.0, 0.0, 1e-05,
                ),
            },
            // wrist: FR07-H101_v1__1__1
            LinkInertial {
                mass: 0.025998108201265576,
                com: Point3::new(-3.3658284336170272e-06, 0.021380623885861483, 6.078471059822732e-14),
                inertia: Matrix3::new(
                    4e-06, -0.0, 0.0, //
                    -0.0, 9e-06, 0.0, //
                    0.0, 0.0, 1e-05,
                ),
            },
        ];
        // 각 revolute 관절이 실제로 움직이는 "합성" 강체 = actuated child link +
        // 다음 revolute 관절까지의 fixed 하위 링크(모터 몸체/브래킷/패들)를 평행축
        // 정리로 합친 것. 배치 변환은 `all-4-export.urdf`의 fixed joint(Rigid N)
        // origin을 관절 child link 프레임부터 누적한 값(전부 rpy=0이라 순수 평행이동).
        // 원본 질량/질량중심/텐서는 URDF `<inertial>` 그대로다.
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

        // per-joint 연속 토크 안전 한계 [N*m]. 모터 매핑(specs.md §3):
        // joint0=yaw=MX-64R x2(듀얼모터), joint1=shoulder=MX-64R, joint2=elbow=MX-28T,
        // joint3=wrist=MX-28T. stall(12.0V)을 CONTINUOUS_TORQUE_DERATE로 감쇠한
        // 값을 쓴다 — stall은 지속 불가능한 순간 동작점이라 정상 상태 실현
        // 가능성 판정에는 부적절하기 때문(자세한 근거/가정은 constants::control).
        //
        // joint0(yaw)만 모터 2배: URDF에서 `Rigid 4`/`Rigid 5`가 각각
        // `MX-64R_v1__2__1`/`MX-64R_v1__1__1`을 `base_link`에 대칭(±6.625cm)
        // 고정하고, `Revolute 6`(yaw)은 그중 하나(`MX-64R_v1__2__1`)를
        // 부모로 삼는다 — 나머지 한 대(`MX-64R_v1__1__1`)는 어떤 관절도
        // 구동하지 않는 것처럼 보이지만, 실기에서는 이 둘이 기계적으로
        // 결합돼 같은 yaw 축에 토크를 함께 낸다(2026-07-23, 하드웨어 담당자
        // 확인). 소프트웨어 IK/동역학 모델은 이 축을 여전히 관절 1개
        // (`Revolute 6`)로만 다루지만(운동학적 자유도는 늘지 않음), 토크
        // 예산은 모터 1대분이 아니라 2대분이어야 한다.
        let joint_torque_limits = vec![
            2.0 * MX64_STALL_TORQUE_NM * CONTINUOUS_TORQUE_DERATE,
            MX64_STALL_TORQUE_NM * CONTINUOUS_TORQUE_DERATE,
            MX28_STALL_TORQUE_NM * CONTINUOUS_TORQUE_DERATE,
            MX28_STALL_TORQUE_NM * CONTINUOUS_TORQUE_DERATE,
        ];

        let mount_z = table::SURFACE_Z + height_offset_m;
        return Self::builder()
            .base_xyz(0.0, base_y, mount_z)
            .linear_rail(base_y, mount_z, 0.0, table::WIDTH_X, RAIL_MAX_SPEED)
            .serial_chain(
                chain,
                vec![
                    None,
                    Some(JointLimit::new(-0.523599, 0.523599)),
                    Some(JointLimit::new(-2.007129, 1.48353)),
                    Some(JointLimit::new(-2.094395, 2.094395)),
                ],
                link_inertials,
                Joints::from_slice(&[0.0, 0.0, -0.2617995, 0.0]),
            )
            .aggregated_inertials(aggregated_inertials)
            .joint_torque_limits(joint_torque_limits)
            .max_joint_speed(DYNAMIXEL_MAX_JOINT_SPEED_RAD_S)
            .build();
    }

    fn arm_length(&self) -> f64 {
        return self.chain.approximate_reach();
    }

    /// revolute 축(관절) 개수.
    pub fn joint_count(&self) -> usize {
        return self.limits.len();
    }

    pub fn joint_limit(&self, index: usize) -> Option<JointLimit> {
        return self.limits.get(index).copied().flatten();
    }

    /// 라켓 open을 담당하는 관절. 3축 위치 체인은 별도 손목이 없다.
    pub fn wrist_joint_index(&self) -> Option<usize> {
        if self.chain.joints.len() >= 4 {
            return Some(self.chain.joints.len() - 1);
        }
        return None;
    }

    /// `default_joints`로 초기화된 런타임 상태.
    pub fn initial_state(&self) -> RobotState {
        let rail_x = self
            .rail
            .as_ref()
            .map(|rail| rail.home_x())
            .unwrap_or(self.base.v.x);
        return RobotState::new(self.default_joints.clone(), rail_x);
    }

    /// 모든 관절각이 한계 안인지 확인한다.
    pub fn joints_in_limits(&self, joints: &Joints) -> bool {
        if joints.values.len() != self.joint_count() {
            return false;
        }
        return self
            .limits
            .iter()
            .zip(joints.values.iter())
            .all(|(limit, &angle)| limit.is_none_or(|limit| limit.contains(angle)));
    }

    /// 순기구학 - 관절각 -> 라켓 끝점/면 방향.
    ///
    /// 4축: yaw + 2R(어깨/팔꿈치 접힘) + 손목 open.
    pub fn forward_kinematics(&self, joints: &Joints) -> Option<RacketPose> {
        return self.forward_kinematics_at(self.base, joints);
    }

    /// `rail_x`가 주어진 레일 위치에서 FK.
    pub fn forward_kinematics_with_rail(&self, rail_x: f64, joints: &Joints) -> Option<RacketPose> {
        let mount = self.mount_at_rail(rail_x);
        return self.forward_kinematics_at(mount, joints);
    }

    /// 주어진 마운트 원점에서 FK.
    pub fn forward_kinematics_at(&self, mount: Point3, joints: &Joints) -> Option<RacketPose> {
        let (ee, _) = self.chain.forward_with_joint_frames(mount.v, &joints.values)?;
        return Some(racket_pose_from_isometry(ee));
    }

    /// 마운트부터 EE까지의 체인 점 - OBB/뷰어 공용.
    pub fn chain_points(&self, rail_x: f64, joints: &Joints) -> Option<Vec<Vector3<f64>>> {
        let mount = self.mount_at_rail(rail_x).v;
        let (ee, frames) = self.chain.forward_with_joint_frames(mount, &joints.values)?;
        let mut points = Vec::with_capacity(frames.len() + 2);
        points.push(mount);
        for (position, _) in frames {
            if (position - *points.last().expect("mount")).norm_squared() > 1e-16 {
                points.push(position);
            }
        }
        let ee_position = ee.translation.vector;
        if (ee_position - *points.last().expect("mount")).norm_squared() > 1e-16 {
            points.push(ee_position);
        }
        return Some(points);
    }

    /// 손목 open [rad]을 한계 안으로 넣어 새 `Joints`를 만든다.
    pub fn with_wrist_open(&self, joints: &Joints, open: f64) -> Result<Joints, SwingPlanError> {
        if joints.values.len() != self.joint_count() {
            return Err(SwingPlanError::InverseKinematicsNoSolution {
                target_x: 0.0,
                target_y: 0.0,
                target_z: 0.0,
            });
        }
        let Some(wrist_index) = self.wrist_joint_index() else {
            return Ok(joints.clone());
        };
        let requested = -open;
        let clamped = self
            .joint_limit(wrist_index)
            .map_or(requested, |limit| requested.clamp(limit.min, limit.max));
        let mut values = joints.values.clone();
        values[wrist_index] = clamped;
        return Ok(Joints { values });
    }

    /// 리턴 속도 방향에 맞춘 손목 open [rad] (수평/수직 성분).
    pub fn wrist_open_for_return(v_out: Vector3<f64>) -> f64 {
        let horizontal = (v_out.x * v_out.x + v_out.y * v_out.y).sqrt().max(1e-6);
        return v_out.z.atan2(horizontal);
    }

    /// 역기구학 - 라켓 끝을 `target`에 두는 관절각.
    pub fn inverse_kinematics(&self, target: Point3) -> Result<Joints, SwingPlanError> {
        return self.inverse_kinematics_near(target, None);
    }

    /// `hint`에 가까운 IK 해를 고른다 (스윙 연속성용).
    pub fn inverse_kinematics_near(
        &self,
        target: Point3,
        hint: Option<&Joints>,
    ) -> Result<Joints, SwingPlanError> {
        return self.inverse_kinematics_at_mount(self.base, target, hint);
    }

    /// 레일 x에서 IK - X는 레일이 맡고 팔은 Y/Z 평면.
    pub fn inverse_kinematics_with_rail(
        &self,
        rail: &LinearRail,
        rail_x: f64,
        target: Point3,
        hint: Option<&Joints>,
    ) -> Result<Joints, SwingPlanError> {
        return self.inverse_kinematics_at_mount(rail.mount_point(rail_x), target, hint);
    }

    /// 레일과 관절을 함께 움직여 라켓 중심과 면 법선을 맞춘다.
    pub fn inverse_pose_with_rail(
        &self,
        target: Point3,
        target_normal: Vector3<f64>,
        hint: &RobotPose,
    ) -> Result<RobotPose, SwingPlanError> {
        const MAX_ITERS: usize = 250;
        const POSITION_TOLERANCE: f64 = 2e-4;
        const NORMAL_TOLERANCE: f64 = 1e-3;
        const STEP: f64 = 1e-6;
        const DAMPING: f64 = 1e-3;
        const MAX_UPDATE: f64 = 0.2;

        if target_normal.norm_squared() <= f64::EPSILON
            || hint.joints.values.len() != self.joint_count()
        {
            return Err(SwingPlanError::InverseKinematicsNoSolution {
                target_x: target.v.x,
                target_y: target.v.y,
                target_z: target.v.z,
            });
        }
        let target_normal = target_normal.normalize();
        let reference = if target_normal.z.abs() < 0.9 {
            Vector3::z()
        } else {
            Vector3::x()
        };
        let tangent_a = target_normal.cross(&reference).normalize();
        let tangent_b = target_normal.cross(&tangent_a).normalize();
        let has_rail = self.rail.is_some();

        let make_values = |rail_x: f64, joints: &Joints| {
            let mut values = Vec::with_capacity(self.joint_count() + usize::from(has_rail));
            if has_rail {
                values.push(rail_x);
            }
            values.extend_from_slice(&joints.values);
            values
        };
        let decode = |values: &[f64]| {
            let (rail_x, offset) = if has_rail {
                (values[0], 1)
            } else {
                (hint.rail_x, 0)
            };
            (
                rail_x,
                Joints {
                    values: values[offset..].to_vec(),
                },
            )
        };
        let task = |pose: RacketPose| {
            let normal_error = target_normal - pose.normal;
            DVector::from_vec(vec![
                target.v.x - pose.position.v.x,
                target.v.y - pose.position.v.y,
                target.v.z - pose.position.v.z,
                normal_error.dot(&tangent_a),
                normal_error.dot(&tangent_b),
            ])
        };

        let mut seeds = vec![make_values(hint.rail_x, &hint.joints)];
        if let Some(rail) = &self.rail {
            seeds.push(make_values(rail.clamp_x(target.v.x), &self.default_joints));
            seeds.push(make_values(rail.default_x(), &self.default_joints));
        }
        let mut best: Option<(f64, RobotPose)> = None;
        for mut values in seeds {
            for _ in 0..MAX_ITERS {
                let (rail_x, joints) = decode(&values);
                let Some(pose) = self.forward_kinematics_with_rail(rail_x, &joints) else {
                    break;
                };
                let position_error = (target.v - pose.position.v).norm();
                let normal_error = (target_normal - pose.normal).norm();
                let score = position_error + normal_error;
                if best
                    .as_ref()
                    .is_none_or(|(best_score, _)| score < *best_score)
                {
                    best = Some((score, RobotPose::new(rail_x, joints.clone())));
                }
                if position_error <= POSITION_TOLERANCE && normal_error <= NORMAL_TOLERANCE {
                    return Ok(RobotPose::new(rail_x, joints));
                }

                let error = task(pose);
                let mut jacobian = DMatrix::zeros(5, values.len());
                for index in 0..values.len() {
                    let joint_offset = usize::from(has_rail);
                    let difference = if has_rail
                        && index == 0
                        && self.rail.is_some_and(|rail| values[0] + STEP > rail.x_max)
                    {
                        -STEP
                    } else if index >= joint_offset
                        && self
                            .joint_limit(index - joint_offset)
                            .is_some_and(|limit| values[index] + STEP > limit.max)
                    {
                        -STEP
                    } else {
                        STEP
                    };
                    let mut perturbed = values.clone();
                    perturbed[index] += difference;
                    let (perturbed_rail, perturbed_joints) = decode(&perturbed);
                    let Some(perturbed_pose) =
                        self.forward_kinematics_with_rail(perturbed_rail, &perturbed_joints)
                    else {
                        continue;
                    };
                    let derivative = (task(pose) - task(perturbed_pose)) / difference;
                    jacobian.set_column(index, &derivative);
                }
                let regularized = &jacobian * jacobian.transpose()
                    + DMatrix::identity(5, 5) * (DAMPING * DAMPING);
                let Some(inverse) = regularized.try_inverse() else {
                    break;
                };
                let mut delta = jacobian.transpose() * (inverse * error);
                if delta.norm() > MAX_UPDATE {
                    delta *= MAX_UPDATE / delta.norm();
                }
                for (index, value) in values.iter_mut().enumerate() {
                    *value += delta[index];
                }
                let joint_offset = usize::from(has_rail);
                if let Some(rail) = &self.rail {
                    values[0] = rail.clamp_x(values[0]);
                }
                for index in 0..self.joint_count() {
                    if let Some(limit) = self.joint_limit(index) {
                        values[index + joint_offset] =
                            values[index + joint_offset].clamp(limit.min, limit.max);
                    }
                }
            }
        }
        if let Some((_, candidate)) = best {
            let pose = self
                .forward_kinematics_with_rail(candidate.rail_x, &candidate.joints)
                .expect("validated pose IK candidate");
            if (target.v - pose.position.v).norm() <= POSITION_TOLERANCE
                && (target_normal - pose.normal).norm() <= NORMAL_TOLERANCE
            {
                return Ok(candidate);
            }
        }
        return Err(SwingPlanError::InverseKinematicsNoSolution {
            target_x: target.v.x,
            target_y: target.v.y,
            target_z: target.v.z,
        });
    }

    fn inverse_kinematics_at_mount(
        &self,
        mount: Point3,
        target: Point3,
        hint: Option<&Joints>,
    ) -> Result<Joints, SwingPlanError> {
        const MAX_ITERS: usize = 300;
        const TOLERANCE: f64 = 1e-5;
        const DAMPING: f64 = 1e-3;
        const MAX_STEP: f64 = 0.25;

        let mut seeds = Vec::new();
        if let Some(hint) = hint.filter(|joints| joints.values.len() == self.joint_count()) {
            seeds.push(hint.values.clone());
        }
        seeds.push(self.default_joints.values.clone());
        seeds.push(
            self.limits
                .iter()
                .enumerate()
                .map(|(index, limit)| {
                    limit.map_or(self.default_joints.values[index], |limit| {
                        (limit.min + limit.max) * 0.5
                    })
                })
                .collect(),
        );
        for fraction in [0.25, 0.75] {
            seeds.push(
                self.limits
                    .iter()
                    .enumerate()
                    .map(|(index, limit)| {
                        limit.map_or(self.default_joints.values[index], |limit| {
                            limit.min + (limit.max - limit.min) * fraction
                        })
                    })
                    .collect(),
            );
        }

        let mut best_error = f64::INFINITY;
        let mut best = None;
        for mut values in seeds {
            for _ in 0..MAX_ITERS {
                let joints = Joints {
                    values: values.clone(),
                };
                let Some(pose) = self.forward_kinematics_at(mount, &joints) else {
                    break;
                };
                let error = target.v - pose.position.v;
                let error_norm = error.norm();
                if error_norm < best_error {
                    best_error = error_norm;
                    best = Some(values.clone());
                }
                if error_norm <= TOLERANCE {
                    return Ok(joints);
                }

                let Some(jacobian) = self.position_jacobian_at(mount, &joints) else {
                    break;
                };
                let regularized =
                    &jacobian * jacobian.transpose() + Matrix3::identity() * (DAMPING * DAMPING);
                let Some(inverse) = regularized.try_inverse() else {
                    break;
                };
                let mut delta = jacobian.transpose() * (inverse * error);
                let norm = delta.norm();
                if norm > MAX_STEP {
                    delta *= MAX_STEP / norm;
                }
                for (index, value) in values.iter_mut().enumerate() {
                    *value += delta[index];
                    if let Some(limit) = self.joint_limit(index) {
                        *value = (*value).clamp(limit.min, limit.max);
                    }
                }
            }
        }

        if best_error <= TOLERANCE {
            return Ok(Joints {
                values: best.expect("best IK seed"),
            });
        }
        return Err(SwingPlanError::InverseKinematicsNoSolution {
            target_x: target.v.x,
            target_y: target.v.y,
            target_z: target.v.z,
        });
    }

    /// 레일 x에서의 마운트 원점.
    pub fn mount_at_rail(&self, rail_x: f64) -> Point3 {
        if let Some(rail) = &self.rail {
            return rail.mount_point(rail_x);
        }
        return self.base;
    }

    /// 리니어 레일 + 팔 도달 범위로 임팩트점을 보정한다.
    ///
    /// 가능하면 hit-plane y를 유지하고 xz(레일/높이)만 줄인다.
    /// 구면 투영만 하면 y가 로봇 쪽으로 당겨져 타이밍/접촉이 어긋난다.
    pub fn clamp_impact_for_rail(&self, rail: &LinearRail, target: Point3) -> (f64, Point3) {
        let rail_x = rail.clamp_x(target.v.x);
        let mount = rail.mount_point(rail_x);
        return (
            rail_x,
            Self::clamp_preserving_y(mount, target, self.arm_length()),
        );
    }

    /// `y`(접수 깊이)를 우선 보존하며 도달 구 안으로 투영한다.
    fn clamp_preserving_y(mount: Point3, target: Point3, arm_length: f64) -> Point3 {
        let max_reach = (arm_length - 1e-3).max(0.0);
        let rel = target.v - mount.v;
        let distance = rel.norm();
        if distance <= max_reach || distance < f64::EPSILON {
            return target;
        }

        let y_comp = rel.y;
        let lateral_sq = max_reach * max_reach - y_comp * y_comp;
        if lateral_sq > 0.0 {
            let max_lat = lateral_sq.sqrt();
            let lateral = Vector3::new(rel.x, 0.0, rel.z);
            let lat_norm = lateral.norm();
            if lat_norm > 1e-9 {
                let scale = (max_lat / lat_norm).min(1.0);
                return Point3::from(
                    mount.v + Vector3::new(lateral.x * scale, y_comp, lateral.z * scale),
                );
            }
            return Point3::from(mount.v + Vector3::new(0.0, y_comp, 0.0));
        }

        // y 자체만으로도 도달 불능 - 구면 투영 폴백
        return Point3::from(mount.v + rel * (max_reach / distance));
    }

    /// 라켓 위치에 대한 3xN 자코비안 `dp/dq` (마운트 기준).
    fn position_jacobian_at(&self, mount: Point3, joints: &Joints) -> Option<DMatrix<f64>> {
        let (ee, frames) = self.chain.forward_with_joint_frames(mount.v, &joints.values)?;
        let ee_position = ee.translation.vector;
        let mut jacobian = DMatrix::zeros(3, frames.len());
        for (index, (joint_position, joint_axis)) in frames.iter().enumerate() {
            let column = joint_axis.cross(&(ee_position - joint_position));
            jacobian[(0, index)] = column.x;
            jacobian[(1, index)] = column.y;
            jacobian[(2, index)] = column.z;
        }
        return Some(jacobian);
    }

    /// 라켓 면 법선을 유지하는 레일·관절 속도 역산.
    pub fn velocities_for_racket_velocity(
        &self,
        pose: &RobotPose,
        racket_velocity: Vector3<f64>,
    ) -> Result<(f64, Vec<f64>), SwingPlanError> {
        const STEP: f64 = 1e-6;
        if pose.joints.values.len() != self.joint_count() {
            return Err(SwingPlanError::InverseKinematicsNoSolution {
                target_x: racket_velocity.x,
                target_y: racket_velocity.y,
                target_z: racket_velocity.z,
            });
        }
        let current = self
            .forward_kinematics_with_rail(pose.rail_x, &pose.joints)
            .ok_or(SwingPlanError::InverseKinematicsNoSolution {
                target_x: racket_velocity.x,
                target_y: racket_velocity.y,
                target_z: racket_velocity.z,
            })?;
        let reference = if current.normal.z.abs() < 0.9 {
            Vector3::z()
        } else {
            Vector3::x()
        };
        let tangent_a = current.normal.cross(&reference).normalize();
        let tangent_b = current.normal.cross(&tangent_a).normalize();
        let has_rail = self.rail.is_some();
        let mut values = Vec::with_capacity(self.joint_count() + usize::from(has_rail));
        if has_rail {
            values.push(pose.rail_x);
        }
        values.extend_from_slice(&pose.joints.values);
        let output = |racket: RacketPose| {
            DVector::from_vec(vec![
                racket.position.v.x,
                racket.position.v.y,
                racket.position.v.z,
                racket.normal.dot(&tangent_a),
                racket.normal.dot(&tangent_b),
            ])
        };
        let base_output = output(current);
        let mut jacobian = DMatrix::zeros(5, values.len());
        for index in 0..values.len() {
            let joint_offset = usize::from(has_rail);
            let difference = if has_rail
                && index == 0
                && self.rail.is_some_and(|rail| values[0] + STEP > rail.x_max)
            {
                -STEP
            } else if index >= joint_offset
                && self
                    .joint_limit(index - joint_offset)
                    .is_some_and(|limit| values[index] + STEP > limit.max)
            {
                -STEP
            } else {
                STEP
            };
            let mut perturbed = values.clone();
            perturbed[index] += difference;
            let (rail_x, joint_values) = if has_rail {
                (perturbed[0], perturbed[1..].to_vec())
            } else {
                (pose.rail_x, perturbed)
            };
            let perturbed_pose = self
                .forward_kinematics_with_rail(
                    rail_x,
                    &Joints {
                        values: joint_values,
                    },
                )
                .ok_or(SwingPlanError::InverseKinematicsNoSolution {
                    target_x: racket_velocity.x,
                    target_y: racket_velocity.y,
                    target_z: racket_velocity.z,
                })?;
            jacobian.set_column(
                index,
                &((output(perturbed_pose) - &base_output) / difference),
            );
        }
        let target = DVector::from_vec(vec![
            racket_velocity.x,
            racket_velocity.y,
            racket_velocity.z,
            0.0,
            0.0,
        ]);
        let regularized =
            jacobian.transpose() * &jacobian + DMatrix::identity(values.len(), values.len()) * 1e-9;
        let Some(inverse) = regularized.try_inverse() else {
            return Err(SwingPlanError::InverseKinematicsNoSolution {
                target_x: racket_velocity.x,
                target_y: racket_velocity.y,
                target_z: racket_velocity.z,
            });
        };
        let velocities = inverse * jacobian.transpose() * target;
        let (rail_velocity, offset) = if has_rail {
            (velocities[0], 1)
        } else {
            (0.0, 0)
        };
        return Ok((
            rail_velocity,
            velocities.iter().skip(offset).copied().collect(),
        ));
    }

    /// 라켓 위치(x,y,z) 3제약만의 수치미분 자코비안 - `(레일 유무 포함) x 관절수`.
    /// [`linear_velocities_for_racket_velocity`]와 그 조작성 평가(특이값 등)가
    /// 공유하는 빌더.
    fn position_jacobian_fd(&self, pose: &RobotPose) -> Option<DMatrix<f64>> {
        const STEP: f64 = 1e-6;
        if pose.joints.values.len() != self.joint_count() {
            return None;
        }
        let has_rail = self.rail.is_some();
        let mut values = Vec::with_capacity(self.joint_count() + usize::from(has_rail));
        if has_rail {
            values.push(pose.rail_x);
        }
        values.extend_from_slice(&pose.joints.values);
        let base = self.forward_kinematics_with_rail(pose.rail_x, &pose.joints)?;
        let base_pos = DVector::from_vec(vec![base.position.v.x, base.position.v.y, base.position.v.z]);
        let mut jacobian = DMatrix::zeros(3, values.len());
        for index in 0..values.len() {
            let joint_offset = usize::from(has_rail);
            let difference = if has_rail
                && index == 0
                && self.rail.is_some_and(|rail| values[0] + STEP > rail.x_max)
            {
                -STEP
            } else if index >= joint_offset
                && self
                    .joint_limit(index - joint_offset)
                    .is_some_and(|limit| values[index] + STEP > limit.max)
            {
                -STEP
            } else {
                STEP
            };
            let mut perturbed = values.clone();
            perturbed[index] += difference;
            let (rail_x, joint_values) = if has_rail {
                (perturbed[0], perturbed[1..].to_vec())
            } else {
                (pose.rail_x, perturbed)
            };
            let perturbed_pose = self.forward_kinematics_with_rail(
                rail_x,
                &Joints {
                    values: joint_values,
                },
            )?;
            let perturbed_pos = DVector::from_vec(vec![
                perturbed_pose.position.v.x,
                perturbed_pose.position.v.y,
                perturbed_pose.position.v.z,
            ]);
            jacobian.set_column(index, &((perturbed_pos - &base_pos) / difference));
        }
        return Some(jacobian);
    }

    /// 목표 라켓 "선속도"만 내는 관절/레일 속도의 최소노름 해 - 순간 방향
    /// (라켓 법선 회전)은 강제하지 않는다.
    ///
    /// [`velocities_for_racket_velocity`]는 위치 3 + 방향유지 2, 총 5제약을
    /// 걸어 레일+4관절(5 미지수)이 완전결정계가 돼 부하 분산 여지가 없다.
    /// 스윙 임팩트는 그 순간 라켓 자세를 절대 불변으로 유지할 필요는 없고
    /// (실제 스윙도 접촉 순간 라켓이 계속 회전 중이다) 목표 선속도만 내면
    /// 되므로, 위치 3제약만 걸어 남는 2자유도를 관절 부하 분산에 쓴다.
    /// 근거: 2026-07-23 실측 - 방향유지 제거만으로 이 arm의 실측 피크
    /// 관절속도가 17.55→11.25 rad/s로 줄었다(단독으로 한계를 만족시키진
    /// 못했지만, IK 시드 선택과 결합하면 유의미하다).
    pub fn linear_velocities_for_racket_velocity(
        &self,
        pose: &RobotPose,
        racket_velocity: Vector3<f64>,
    ) -> Result<(f64, Vec<f64>), SwingPlanError> {
        let err = || SwingPlanError::InverseKinematicsNoSolution {
            target_x: racket_velocity.x,
            target_y: racket_velocity.y,
            target_z: racket_velocity.z,
        };
        let jacobian = self.position_jacobian_fd(pose).ok_or_else(err)?;
        let jjt = &jacobian * jacobian.transpose() + DMatrix::identity(3, 3) * 1e-9;
        let inverse = jjt.try_inverse().ok_or_else(err)?;
        let target = DVector::from_vec(vec![racket_velocity.x, racket_velocity.y, racket_velocity.z]);
        let velocities = jacobian.transpose() * inverse * target;
        let has_rail = self.rail.is_some();
        let (rail_velocity, offset) = if has_rail { (velocities[0], 1) } else { (0.0, 0) };
        return Ok((
            rail_velocity,
            velocities.iter().skip(offset).copied().collect(),
        ));
    }
}

fn racket_pose_from_isometry(iso: Isometry3<f64>) -> RacketPose {
    // 4-dof CAD 라켓 링크 계약: local +Y=면 법선(얇은 mesh 축), +Z=위.
    // 기구학/Rapier 라켓 계약: local +Z=면 법선, +Y=위.
    let normal = iso.rotation * Vector3::y();
    let link_from_racket = UnitQuaternion::from_axis_angle(
        &nalgebra::Unit::new_normalize(Vector3::new(0.0, 1.0, 1.0)),
        std::f64::consts::PI,
    );
    let orientation = iso.rotation * link_from_racket;
    let q = orientation.quaternion();
    return RacketPose {
        position: Point3::from(iso.translation.vector),
        normal,
        orientation: [q.w, q.i, q.j, q.k],
    };
}
