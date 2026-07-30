//! SimViewer facade.

use super::SimViewerOptions;

#[cfg(feature = "gui")]
pub struct SimViewer;

#[cfg(feature = "gui")]
impl SimViewer {
    pub fn run(options: SimViewerOptions) -> Result<(), String> {
        return super::host::run_sim_viewer(options);
    }

    pub fn lock_world_for_frame(
        world: &std::sync::Mutex<crate::sim::physics::world::SimWorld>,
    ) -> Option<std::sync::MutexGuard<'_, crate::sim::physics::world::SimWorld>> {
        return super::viewer::lock_world_for_frame(world);
    }
}
