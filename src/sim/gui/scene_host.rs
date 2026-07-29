//! SceneHost facade.

use super::{BallOnlyViewerOptions, SceneHostOptions, TableSceneOptions};

#[cfg(feature = "gui")]
pub struct SceneHost;

#[cfg(feature = "gui")]
impl SceneHost {
    pub fn run(options: SceneHostOptions) -> Result<(), String> {
        return super::host::run_scene_host(options);
    }

    pub fn run_ball_only(options: BallOnlyViewerOptions) -> Result<(), String> {
        return super::host::run_ball_only_viewer(options);
    }

    pub fn build_table_scene(
        scene_root: &mut kiss3d::prelude::SceneNode3d,
        options: &TableSceneOptions,
    ) {
        super::scene::build_table_scene(scene_root, options);
    }
}
