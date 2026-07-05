//! `RacketPose`(domain) → Rapier `Pose` 변환.

use pingpong_domain::RacketPose;
use rapier3d::prelude::{Rotation, Vector};

/// domain `RacketPose` → Rapier (위치, 회전).
pub fn racket_pose_to_rapier(pose: &RacketPose) -> (Vector, Rotation) {
    let p = pose.position.v;
    let position = Vector::new(p.x as f32, p.y as f32, p.z as f32);
    let [w, x, y, z] = pose.orientation;
    let rotation = Rotation::from_xyzw(x as f32, y as f32, z as f32, w as f32);
    return (position, rotation);
}
