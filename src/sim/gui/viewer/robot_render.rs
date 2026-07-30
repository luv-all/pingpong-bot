//! 로봇 렌더 모드.

use super::dynamic_nodes::DynamicNodes;
use super::urdf_visual_node::UrdfVisualNode;

pub(crate) enum RobotRender {
    Primitive(DynamicNodes),
    Urdf(Vec<UrdfVisualNode>),
}
