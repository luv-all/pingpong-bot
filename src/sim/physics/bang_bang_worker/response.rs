use crate::error::DomainError;
use crate::swing;

pub(super) struct Response {
    pub(super) id: u64,
    pub(super) result: Result<swing::bang_bang::PlannedIntercept, DomainError>,
}
