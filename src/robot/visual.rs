//! 월드에 맞춰 그리는 로봇 비주얼.

use kiss3d::prelude::*;

use crate::robot::urdf::UrdfModel;
use crate::robot::{PrimitiveNodes, UrdfNodes};
use crate::sim::physics::world::SimWorld;

/// 월드에 맞춰 그리는 로봇 비주얼.
pub enum Visual {
    Primitive(PrimitiveNodes),
    Urdf(UrdfNodes),
}

impl Visual {
    pub fn spawn(scene: &mut SceneNode3d, urdf: Option<&UrdfModel>) -> Self {
        return match urdf {
            Some(model) => Self::Urdf(UrdfNodes::spawn(scene, model)),
            None => Self::Primitive(PrimitiveNodes::spawn(scene)),
        };
    }

    pub fn sync_from_world(&mut self, world: &SimWorld, urdf: Option<&UrdfModel>) {
        match (self, urdf) {
            (Self::Primitive(nodes), _) => nodes.sync_from_world(world),
            (Self::Urdf(nodes), Some(model)) => nodes.sync_from_world(world, model),
            _ => {}
        }
    }
}
