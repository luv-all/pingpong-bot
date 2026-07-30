#[cfg(all(windows, feature = "real"))]
use super::axl_live::AxlLive;

pub(super) enum RailKind {
    DryRun {
        position_m: f64,
    },
    #[cfg(all(windows, feature = "real"))]
    Live(AxlLive),
}
