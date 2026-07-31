//! 관측과 미래 예측이 공유하는 규격화 궤적.

use std::time::Instant;

use nalgebra::{DMatrix, Vector3};
use thiserror::Error;

use crate::Point3;

/// 궤적 행 하나. 위치·속도는 월드 좌표계, 시간은 기준 시각 대비 초다.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TrajectorySample {
    pub position: Point3,
    pub velocity: Vector3<f64>,
    pub time_secs: f64,
}

impl TrajectorySample {
    pub const COLUMN_COUNT: usize = 7;

    pub fn new(position: Point3, velocity: Vector3<f64>, time_secs: f64) -> Self {
        return Self {
            position,
            velocity,
            time_secs,
        };
    }

    /// `[x, y, z, vx, vy, vz, t]` 순서의 행으로 변환한다.
    pub fn to_row(self) -> [f64; Self::COLUMN_COUNT] {
        return [
            self.position.x,
            self.position.y,
            self.position.z,
            self.velocity.x,
            self.velocity.y,
            self.velocity.z,
            self.time_secs,
        ];
    }

    pub fn from_row(row: [f64; Self::COLUMN_COUNT]) -> Result<Self, TrajectoryMatrixError> {
        if !row.iter().all(|value| value.is_finite()) {
            return Err(TrajectoryMatrixError::NonFinite);
        }
        return Ok(Self::new(
            Point3::new(row[0], row[1], row[2]),
            Vector3::new(row[3], row[4], row[5]),
            row[6],
        ));
    }
}

/// 하나의 공에 대한 과거 관측과 미래 예측.
#[derive(Debug, Clone, PartialEq)]
pub struct BallTrajectory {
    pub observed: Vec<TrajectorySample>,
    pub predicted: Vec<TrajectorySample>,
    /// `time_secs == 0`인 가장 최근 EKF 채택 관측 시각.
    pub reference_time: Instant,
}

impl BallTrajectory {
    pub fn new(
        observed: Vec<TrajectorySample>,
        predicted: Vec<TrajectorySample>,
        reference_time: Instant,
    ) -> Result<Self, TrajectoryMatrixError> {
        validate_times(&observed, false)?;
        validate_times(&predicted, true)?;
        return Ok(Self {
            observed,
            predicted,
            reference_time,
        });
    }

    /// 내부 타입을 외부 경계의 `N×7` 행렬로 변환한다.
    pub fn matrices(&self) -> (DMatrix<f64>, DMatrix<f64>) {
        return (
            samples_to_matrix(&self.observed),
            samples_to_matrix(&self.predicted),
        );
    }

    pub fn from_matrices(
        observed: &DMatrix<f64>,
        predicted: &DMatrix<f64>,
        reference_time: Instant,
    ) -> Result<Self, TrajectoryMatrixError> {
        return Self::new(
            samples_from_matrix(observed)?,
            samples_from_matrix(predicted)?,
            reference_time,
        );
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Error)]
pub enum TrajectoryMatrixError {
    #[error("궤적 행렬은 7열이어야 함")]
    WrongColumnCount,
    #[error("궤적에 유한하지 않은 값이 있음")]
    NonFinite,
    #[error("궤적 시간은 오름차순이어야 함")]
    UnsortedTime,
    #[error("관측 시간은 0 이하, 예측 시간은 0 초과여야 함")]
    InvalidTimeDomain,
}

pub fn samples_to_matrix(samples: &[TrajectorySample]) -> DMatrix<f64> {
    let mut matrix = DMatrix::zeros(samples.len(), TrajectorySample::COLUMN_COUNT);
    for (row_index, sample) in samples.iter().enumerate() {
        for (column_index, value) in sample.to_row().into_iter().enumerate() {
            matrix[(row_index, column_index)] = value;
        }
    }
    return matrix;
}

pub fn samples_from_matrix(
    matrix: &DMatrix<f64>,
) -> Result<Vec<TrajectorySample>, TrajectoryMatrixError> {
    if matrix.ncols() != TrajectorySample::COLUMN_COUNT {
        return Err(TrajectoryMatrixError::WrongColumnCount);
    }
    let samples = (0..matrix.nrows())
        .map(|row| {
            TrajectorySample::from_row([
                matrix[(row, 0)],
                matrix[(row, 1)],
                matrix[(row, 2)],
                matrix[(row, 3)],
                matrix[(row, 4)],
                matrix[(row, 5)],
                matrix[(row, 6)],
            ])
        })
        .collect::<Result<Vec<_>, _>>()?;
    if samples
        .windows(2)
        .any(|pair| pair[0].time_secs > pair[1].time_secs)
    {
        return Err(TrajectoryMatrixError::UnsortedTime);
    }
    return Ok(samples);
}

fn validate_times(
    samples: &[TrajectorySample],
    predicted: bool,
) -> Result<(), TrajectoryMatrixError> {
    if samples
        .iter()
        .flat_map(|sample| sample.to_row())
        .any(|value| !value.is_finite())
    {
        return Err(TrajectoryMatrixError::NonFinite);
    }
    if samples.iter().any(|sample| {
        if predicted {
            sample.time_secs <= 0.0
        } else {
            sample.time_secs > 0.0
        }
    }) {
        return Err(TrajectoryMatrixError::InvalidTimeDomain);
    }
    if !predicted && samples.last().is_some_and(|sample| sample.time_secs != 0.0) {
        return Err(TrajectoryMatrixError::InvalidTimeDomain);
    }
    if samples
        .windows(2)
        .any(|pair| pair[0].time_secs > pair[1].time_secs)
    {
        return Err(TrajectoryMatrixError::UnsortedTime);
    }
    return Ok(());
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn matrix_column_order_and_round_trip() {
        let sample = TrajectorySample::new(
            Point3::new(1.0, 2.0, 3.0),
            Vector3::new(4.0, 5.0, 6.0),
            -0.25,
        );
        let matrix = samples_to_matrix(&[sample]);
        assert_eq!(matrix.shape(), (1, 7));
        assert_eq!(
            matrix.row(0).iter().copied().collect::<Vec<_>>(),
            vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0, -0.25]
        );
        assert_eq!(samples_from_matrix(&matrix).unwrap(), vec![sample]);
    }

    #[test]
    fn trajectory_enforces_time_domains() {
        let sample =
            |time_secs| TrajectorySample::new(Point3::origin(), Vector3::zeros(), time_secs);
        assert!(BallTrajectory::new(vec![sample(0.1)], vec![], Instant::now()).is_err());
        assert!(BallTrajectory::new(vec![sample(-0.1)], vec![], Instant::now()).is_err());
        assert!(BallTrajectory::new(vec![], vec![sample(0.0)], Instant::now()).is_err());
    }
}
