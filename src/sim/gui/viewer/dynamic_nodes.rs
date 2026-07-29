//! Primitive 로봇 동적 노드.

use kiss3d::prelude::*;

pub(crate) struct DynamicNodes {
    /// 블레이드 원판 (면 중심 = EE)
    pub(crate) racket: SceneNode3d,
    /// 손목→면 손잡이
    pub(crate) racket_handle: SceneNode3d,
    pub(crate) arm_base: SceneNode3d,
    pub(crate) links: Vec<SceneNode3d>,
    /// `links[i]` 반지름 [m]. `place_link`가 local_scale을 통째로 덮어쓰므로 보관.
    pub(crate) link_radii: Vec<f32>,
    pub(crate) joints: Vec<SceneNode3d>,
    pub(crate) link_color: Color,
    pub(crate) joint_color: Color,
}
