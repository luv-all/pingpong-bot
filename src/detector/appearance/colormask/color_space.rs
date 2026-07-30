use clap::ValueEnum;

use super::ParseColorSpaceError;

#[derive(
    Debug, Clone, Copy, PartialEq, Eq, Default, ValueEnum, serde::Serialize, serde::Deserialize,
)]
#[value(rename_all = "lower")]
#[serde(rename_all = "lowercase")]
pub enum ColorSpace {
    #[default]
    Ycrcb,
    Hsv,
}

impl std::str::FromStr for ColorSpace {
    type Err = ParseColorSpaceError;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        return match s {
            "ycrcb" | "YCrCb" => Ok(Self::Ycrcb),
            "hsv" | "HSV" => Ok(Self::Hsv),
            _ => Err(ParseColorSpaceError),
        };
    }
}

impl std::fmt::Display for ColorSpace {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        return f.write_str(match self {
            Self::Ycrcb => "ycrcb",
            Self::Hsv => "hsv",
        });
    }
}
