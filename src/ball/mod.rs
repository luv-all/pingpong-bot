//! 공 도메인: 관측·탄도·추정·시뮬 표시.

pub mod ballistics;
pub mod bounce;
pub mod ekf;
pub mod kinematics;
pub mod measure;
pub mod observation;
pub mod snapshot;
pub mod state;

#[cfg(feature = "gui")]
pub mod handle;
#[cfg(feature = "gui")]
pub mod velocity_visual;
#[cfg(feature = "gui")]
pub mod visual;

pub use ekf::Ekf;
pub use kinematics::Kinematics;
pub use measure::{BounceEvent, PhysicsIdentify, RollEvent, TrajAnalysis, TrajPoint};
pub use observation::Observation;
pub use snapshot::Snapshot;
pub use state::State;

#[cfg(feature = "gui")]
pub use handle::Handle;
#[cfg(feature = "gui")]
pub use velocity_visual::VelocityVisual;
#[cfg(feature = "gui")]
pub use visual::Visual;
