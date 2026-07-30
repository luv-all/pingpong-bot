//! URDF 링크 하나당 kiss3d 노드.

use kiss3d::prelude::*;

pub(crate) struct UrdfVisualNode {
    pub(crate) link_name: String,
    pub(crate) local_pos: Vec3,
    pub(crate) local_rot: Quat,
    pub(crate) node: SceneNode3d,
    pub(crate) base_color: Color,
}
