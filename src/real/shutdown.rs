//! 채널 파기 브로드캐스트 종료 신호.
//!
//! `AtomicBool` 공유 플래그 대신 쓴다. 메인이 [`ShutdownGuard`]를 들고 있고 워커는
//! [`Shutdown`] 클론을 든다. 가드가 drop되면 모든 클론의 `try_recv`가 `Disconnected`가 되어
//! 전원이 종료한다 — 공유 가변 상태가 없고, 실수로 다시 켤 방법도 없다 (단조 종료).

use crossbeam_channel::{Receiver, Sender, TryRecvError, bounded};

/// 메인 스레드가 쥐는 종료 가드. drop = 전원 종료.
pub struct ShutdownGuard {
    _tx: Sender<()>,
}

/// 워커가 드는 종료 관찰자. 클론해서 스레드마다 하나씩 준다.
#[derive(Clone)]
pub struct Shutdown {
    rx: Receiver<()>,
}

impl Shutdown {
    /// 가드가 drop됐는지.
    pub fn is_down(&self) -> bool {
        return matches!(self.rx.try_recv(), Err(TryRecvError::Disconnected));
    }
}

/// 가드 1개 + 관찰자 1개. 관찰자는 스레드 수만큼 클론한다.
pub fn shutdown_channel() -> (ShutdownGuard, Shutdown) {
    // 용량 1 — 아무것도 보내지 않는다. drop만이 신호다.
    let (tx, rx) = bounded::<()>(1);
    return (ShutdownGuard { _tx: tx }, Shutdown { rx });
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn observers_stay_up_until_the_guard_drops() {
        let (guard, shutdown) = shutdown_channel();
        let cloned = shutdown.clone();
        assert!(!shutdown.is_down());
        assert!(!cloned.is_down());

        drop(guard);

        assert!(shutdown.is_down());
        assert!(cloned.is_down(), "클론도 같이 내려가야 한다");
    }

    #[test]
    fn is_down_is_stable_across_repeated_calls() {
        let (guard, shutdown) = shutdown_channel();
        drop(guard);
        for _ in 0..3 {
            assert!(shutdown.is_down());
        }
    }
}
