//! sim GUI — 슈터 settings R/W · egui 위젯 · 본체 비주얼.

#[cfg(feature = "gui")]
pub mod handle;
#[cfg(feature = "gui")]
pub mod ui;
#[cfg(feature = "gui")]
pub mod visual;

#[cfg(feature = "gui")]
pub use handle::Handle;
#[cfg(feature = "gui")]
pub use visual::Visual;
