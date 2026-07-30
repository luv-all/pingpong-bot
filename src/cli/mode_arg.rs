//! CLI 모드.

use clap::ValueEnum;

#[derive(Debug, Clone, Copy, ValueEnum)]
pub enum ModeArg {
    Sim,
    Real,
}
