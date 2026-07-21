//! monotonic 시각. sim에서는 [`crate::sim::session::SimClockHandle`]이 구현한다.

use std::time::Instant;

pub trait Clock: Send {
    fn now(&self) -> Instant;
}
