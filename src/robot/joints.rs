//! revolute 관절각 [rad].

/// revolute 관절각 [rad].
#[derive(Debug, Clone, PartialEq)]
pub struct Joints {
    pub values: Vec<f64>,
}

impl Joints {
    pub fn from_slice(values: &[f64]) -> Self {
        return Self {
            values: values.to_vec(),
        };
    }
}
