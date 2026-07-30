//! 링크 관성 - URDF `<inertial>` 원본.

use nalgebra::{Isometry3, Matrix3, Vector3};

use crate::Point3;

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
            placement.rotation * body.com.coords + placement.translation.vector
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
            let translated = body.mass * (Matrix3::identity() * d.dot(&d) - d * d.transpose());
            inertia += rotated + translated;
        }
        return LinkInertial {
            mass: total_mass,
            com: Point3::from(com),
            inertia,
        };
    }
}
