use crate::error::DomainError;
use crate::motion;

pub(super) struct Response {
    pub(super) id: u64,
    pub(super) result: Result<motion::bang_bang::PlannedIntercept, DomainError>,
}
