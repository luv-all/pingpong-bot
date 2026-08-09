//! Rapier 공 상태 스냅샷 (위치·속도·각속도).

use nalgebra::Vector3;

#[derive(Debug, Clone, Copy)]
pub struct Snapshot {
    pub position: Vector3<f64>,
    pub velocity: Vector3<f64>,
    pub omega: Vector3<f64>,
}
