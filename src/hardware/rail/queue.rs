//! 레일 명령 큐 — sparse→exact 2단계 제어를 위해 "최신 명령만 유지, 이전
//! 이동이 끝난 뒤에만 다음 명령을 보낸다"를 시스템적으로 보장한다.
//! 설계 문서: docs/superpowers/specs/2026-08-13-rail-command-queue-design.md

use std::marker::PhantomData;
use std::sync::{Arc, Condvar, Mutex};
use std::thread::JoinHandle;

use crate::error::HwError;

use super::AxlRail;

/// `RailQueue`가 백그라운드 워커에서 구동하는 최소 인터페이스.
/// `AxlRail`은 기존 메서드에 위임해 구현한다 — `axl_rail.rs`는 변경하지 않는다.
pub trait RailDriver: Send {
    fn command_abs_in_secs(&mut self, x: f64, duration_secs: f64) -> Result<f64, HwError>;
    fn wait_idle(&mut self) -> Result<(), HwError>;
}

impl RailDriver for AxlRail {
    fn command_abs_in_secs(&mut self, x: f64, duration_secs: f64) -> Result<f64, HwError> {
        return AxlRail::command_abs_in_secs(self, x, duration_secs);
    }

    fn wait_idle(&mut self) -> Result<(), HwError> {
        return AxlRail::wait_idle(self);
    }
}

struct PendingCommand {
    target_m: f64,
    duration_secs: f64,
}

struct QueueState {
    pending: Option<PendingCommand>,
    moving: bool,
    last_error: Option<HwError>,
    shutdown: bool,
}

struct Shared {
    state: Mutex<QueueState>,
    cv: Condvar,
}

/// 최대 1개의 "아직 안 보낸" 명령만 들고 있는 레일 명령 큐.
/// 새 명령은 이전 미전송 명령을 덮어쓴다 — 오래된 중간 목표는 절대 전송되지 않는다.
pub struct RailQueue<R: RailDriver> {
    shared: Arc<Shared>,
    handle: Option<JoinHandle<()>>,
    _driver: PhantomData<R>,
}

impl<R: RailDriver + 'static> RailQueue<R> {
    /// 워커 스레드를 띄우고 `driver` 소유권을 넘긴다.
    pub fn spawn(mut driver: R) -> Self {
        let shared = Arc::new(Shared {
            state: Mutex::new(QueueState {
                pending: None,
                moving: false,
                last_error: None,
                shutdown: false,
            }),
            cv: Condvar::new(),
        });
        let worker_shared = Arc::clone(&shared);
        let handle = std::thread::spawn(move || {
            run_worker(&mut driver, &worker_shared);
        });
        return Self {
            shared,
            handle: Some(handle),
            _driver: PhantomData,
        };
    }

    /// 아직 전송되지 않은 대기 명령을 덮어쓰고 워커를 깨운다. 블로킹하지 않는다.
    pub fn enqueue(&self, target_m: f64, duration_secs: f64) {
        let mut state = self.shared.state.lock().unwrap();
        state.pending = Some(PendingCommand {
            target_m,
            duration_secs,
        });
        self.shared.cv.notify_all();
    }

    /// 지금 실행 중이거나, 아직 전송 안 된 명령이 대기 중이면 `true`.
    pub fn is_moving(&self) -> bool {
        let state = self.shared.state.lock().unwrap();
        return state.moving || state.pending.is_some();
    }

    /// 마지막으로 기록된 에러를 꺼내며 비운다.
    pub fn take_error(&self) -> Option<HwError> {
        let mut state = self.shared.state.lock().unwrap();
        return state.last_error.take();
    }

    /// 큐가 완전히 빌 때까지(실행 중 명령 없음 + 대기 명령 없음) 블로킹한다.
    pub fn wait_idle(&self) {
        let mut state = self.shared.state.lock().unwrap();
        while state.moving || state.pending.is_some() {
            state = self.shared.cv.wait(state).unwrap();
        }
    }
}

impl<R: RailDriver> Drop for RailQueue<R> {
    fn drop(&mut self) {
        {
            let mut state = self.shared.state.lock().unwrap();
            state.shutdown = true;
        }
        self.shared.cv.notify_all();
        if let Some(handle) = self.handle.take() {
            let _ = handle.join();
        }
    }
}

fn run_worker<R: RailDriver>(driver: &mut R, shared: &Arc<Shared>) {
    loop {
        let command = {
            let mut state = shared.state.lock().unwrap();
            loop {
                if state.shutdown {
                    return;
                }
                if let Some(command) = state.pending.take() {
                    break command;
                }
                state = shared.cv.wait(state).unwrap();
            }
        };

        {
            let mut state = shared.state.lock().unwrap();
            state.moving = true;
        }
        shared.cv.notify_all();

        let result = match driver.command_abs_in_secs(command.target_m, command.duration_secs) {
            Ok(_) => driver.wait_idle(),
            Err(error) => Err(error),
        };

        let mut state = shared.state.lock().unwrap();
        if let Err(error) = result {
            state.last_error = Some(error);
        }
        state.moving = false;
        drop(state);
        shared.cv.notify_all();
    }
}

#[cfg(test)]
mod tests {
    use std::sync::mpsc;
    use std::time::Duration;

    use super::*;

    struct MockDriver {
        sent_tx: mpsc::Sender<f64>,
        release_rx: mpsc::Receiver<()>,
        fail_target: Option<f64>,
    }

    impl RailDriver for MockDriver {
        fn command_abs_in_secs(&mut self, x: f64, _duration_secs: f64) -> Result<f64, HwError> {
            self.sent_tx.send(x).unwrap();
            if self.fail_target == Some(x) {
                return Err(HwError::InvalidConfig {
                    reason: "mock failure".into(),
                });
            }
            return Ok(x);
        }

        fn wait_idle(&mut self) -> Result<(), HwError> {
            self.release_rx.recv().unwrap();
            return Ok(());
        }
    }

    /// `release_tx.send(())` unblocks exactly one in-flight `wait_idle()` call.
    fn spawn_mock(
        fail_target: Option<f64>,
    ) -> (RailQueue<MockDriver>, mpsc::Receiver<f64>, mpsc::Sender<()>) {
        let (sent_tx, sent_rx) = mpsc::channel();
        let (release_tx, release_rx) = mpsc::channel();
        let driver = MockDriver {
            sent_tx,
            release_rx,
            fail_target,
        };
        let queue = RailQueue::spawn(driver);
        return (queue, sent_rx, release_tx);
    }

    #[test]
    fn enqueue_then_wait_idle_sends_the_command() {
        let (queue, sent_rx, release_tx) = spawn_mock(None);
        release_tx.send(()).unwrap();
        queue.enqueue(1.0, 0.1);
        queue.wait_idle();
        assert_eq!(sent_rx.recv_timeout(Duration::from_secs(1)).unwrap(), 1.0);
    }

    #[test]
    fn latest_command_wins_while_previous_is_in_flight() {
        let (queue, sent_rx, release_tx) = spawn_mock(None);

        queue.enqueue(1.0, 0.1);
        // Worker picks up 1.0 and blocks inside wait_idle() until released.
        assert_eq!(sent_rx.recv_timeout(Duration::from_secs(1)).unwrap(), 1.0);

        // These land while the worker is still executing 1.0 — only the last
        // one should ever reach the driver.
        queue.enqueue(2.0, 0.1);
        queue.enqueue(3.0, 0.1);

        release_tx.send(()).unwrap(); // finishes 1.0's wait_idle
        release_tx.send(()).unwrap(); // finishes whichever command runs next

        assert_eq!(sent_rx.recv_timeout(Duration::from_secs(1)).unwrap(), 3.0);
        queue.wait_idle();
        assert!(sent_rx.try_recv().is_err(), "2.0 must never have been sent");
    }

    #[test]
    fn is_moving_reflects_executing_and_pending_state() {
        let (queue, sent_rx, release_tx) = spawn_mock(None);

        assert!(!queue.is_moving());

        queue.enqueue(1.0, 0.1);
        assert_eq!(sent_rx.recv_timeout(Duration::from_secs(1)).unwrap(), 1.0);
        assert!(queue.is_moving(), "worker is blocked inside wait_idle() for 1.0");

        queue.enqueue(2.0, 0.1);
        assert!(queue.is_moving(), "2.0 is pending even though not yet sent");

        release_tx.send(()).unwrap(); // finishes 1.0
        release_tx.send(()).unwrap(); // finishes 2.0
        queue.wait_idle();
        assert!(!queue.is_moving());
    }

    #[test]
    fn error_is_recorded_but_queue_keeps_processing() {
        let (queue, sent_rx, release_tx) = spawn_mock(Some(2.0));

        // 2.0 fails inside command_abs_in_secs itself, so no wait_idle() call
        // happens for it and no release token is needed.
        queue.enqueue(2.0, 0.1);
        assert_eq!(sent_rx.recv_timeout(Duration::from_secs(1)).unwrap(), 2.0);

        // Poll until the worker has recorded the error and gone back to idle.
        let deadline = std::time::Instant::now() + Duration::from_secs(1);
        let error = loop {
            if let Some(error) = queue.take_error() {
                break error;
            }
            assert!(std::time::Instant::now() < deadline, "error was never recorded");
            std::thread::sleep(Duration::from_millis(1));
        };
        assert_eq!(
            error,
            HwError::InvalidConfig {
                reason: "mock failure".into(),
            }
        );
        assert!(queue.take_error().is_none(), "take_error must clear the slot");

        // The queue must still accept and run the next command.
        release_tx.send(()).unwrap();
        queue.enqueue(3.0, 0.1);
        queue.wait_idle();
        assert_eq!(sent_rx.recv_timeout(Duration::from_secs(1)).unwrap(), 3.0);
    }

    #[test]
    fn drop_joins_the_worker_thread_without_hanging() {
        let (queue, sent_rx, release_tx) = spawn_mock(None);
        release_tx.send(()).unwrap();
        queue.enqueue(5.0, 0.1);
        queue.wait_idle();
        assert_eq!(sent_rx.recv_timeout(Duration::from_secs(1)).unwrap(), 5.0);

        let (done_tx, done_rx) = mpsc::channel();
        std::thread::spawn(move || {
            drop(queue);
            let _ = done_tx.send(());
        });
        done_rx
            .recv_timeout(Duration::from_secs(2))
            .expect("RailQueue::drop hung instead of joining the worker");
    }
}
