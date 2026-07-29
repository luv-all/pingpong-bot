use anyhow::{Result, ensure};

use super::ColorSpace;

#[derive(Debug, Clone, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct ColormaskParams {
    pub space: ColorSpace,
    pub c0_min: u8,
    pub c0_max: u8,
    pub c1_min: u8,
    pub c1_max: u8,
    pub c2_min: u8,
    pub c2_max: u8,
}

impl ColormaskParams {
    pub fn validate(&self) -> Result<()> {
        ensure!(self.c0_min <= self.c0_max, "c0_min <= c0_max");
        ensure!(self.c1_min <= self.c1_max, "c1_min <= c1_max");
        ensure!(self.c2_min <= self.c2_max, "c2_min <= c2_max");
        return Ok(());
    }
}
