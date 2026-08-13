//! 레일 명령 큐 — sparse→exact 2단계 제어를 위해 "최신 명령만 유지, 이전
//! 이동이 끝난 뒤에만 다음 명령을 보낸다"를 시스템적으로 보장한다. 동시에
//! 이동 중에도 위치 읽기가 그 사이사이에 끼어들 수 있게 한다 — 워커가
//! 완료를 짧은 주기로 폴링하며, 매 폴백마다 대기 중인 읽기 요청을 먼저
//! 처리한다.
//! 설계 문서: docs/superpowers/specs/2026-08-13-rail-command-queue-design.md

use std::marker::PhantomData;
use std::sync::mpsc;
use std::sync::{Arc, Condvar, Mutex};
use std::thread::JoinHandle;
use std::time::Duration;

use crate::error::HwError;

use super::{AxlRail, RailEnd, RailHomeResult};

/// 이동 완료를 폴링하는 주기 — `AxlLive::wait_idle`의 기존 폴링 주기와 같다.
const POLL_INTERVAL: Duration = Duration::from_millis(1);

/// `RailQueue`가 백그라운드 워커에서 구동하는 최소 인터페이스.
/// `AxlRail`은 기존 메서드에 위임해 구현한다 — `axl_rail.rs`는 위임용
/// `is_moving`만 추가하고 나머지 공개 API는 그대로 둔다.
pub trait RailDriver: Send {
    fn command_abs_in_secs(&mut self, x: f64, duration_secs: f64) -> Result<f64, HwError>;
    /// 지금 이동 중인지 논블로킹으로 확인한다. `RailQueue` 워커가 이 값을
    /// 짧은 주기로 폴링하며, 폴 사이사이에 대기 중인 읽기 요청을 처리한다.
    fn is_moving(&mut self) -> Result<bool, HwError>;
    fn read_x_m(&mut self) -> Result<f64, HwError>;
    /// 진행 중인 이동을 즉시 감속 정지한다. `RailQueue::stop`이 호출한다 —
    /// `enqueue`의 "이전 이동을 끝까지 마친 뒤에만 다음 명령" 보장과 달리,
    /// 명시적 중단 요청은 지금 이동을 곧바로 끊는다.
    fn stop(&mut self) -> Result<(), HwError>;
    /// 물리적 엔드스톱까지 저속 이동해 원점을 다시 잡는다. 드물게, 명시적으로만
    /// 호출된다 — 워커를 최대 홈잉 타임아웃만큼 통째로 점유한다(다른 큐
    /// 요청은 그동안 대기), 기존 `home_rail`이 뮤텍스를 같은 방식으로 쓰던 것과 같다.
    fn home(&mut self, end: RailEnd) -> Result<RailHomeResult, HwError>;
}

impl RailDriver for AxlRail {
    fn command_abs_in_secs(&mut self, x: f64, duration_secs: f64) -> Result<f64, HwError> {
        return AxlRail::command_abs_in_secs(self, x, duration_secs);
    }

    fn is_moving(&mut self) -> Result<bool, HwError> {
        return AxlRail::is_moving(self);
    }

    fn read_x_m(&mut self) -> Result<f64, HwError> {
        return AxlRail::read_x_m(self);
    }

    fn stop(&mut self) -> Result<(), HwError> {
        return AxlRail::stop(self);
    }

    fn home(&mut self, end: RailEnd) -> Result<RailHomeResult, HwError> {
        return AxlRail::home(self, end);
    }
}

struct PendingCommand {
    target_m: f64,
    duration_secs: f64,
}

/// 대기 중인 읽기 요청 — 워커가 처리하고 결과를 `response_tx`로 돌려준다.
struct ReadRequest {
    response_tx: mpsc::Sender<Result<f64, HwError>>,
}

/// 대기 중인 홈잉 요청 — 워커가 처리하고 결과를 `response_tx`로 돌려준다.
struct HomeRequest {
    end: RailEnd,
    response_tx: mpsc::Sender<Result<RailHomeResult, HwError>>,
}

struct QueueState {
    pending: Option<PendingCommand>,
    pending_read: Option<ReadRequest>,
    pending_home: Option<HomeRequest>,
    stop_requested: bool,
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
/// 위치 읽기는 이동 중에도 워커의 폴링 사이사이에 끼어들어 처리된다.
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
                pending_read: None,
                pending_home: None,
                stop_requested: false,
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

    /// 지금 위치를 읽는다. 이동 중이어도 워커의 폴링 사이사이(≤ [`POLL_INTERVAL`])에
    /// 끼어들어 처리되므로, 진행 중인 이동이 끝날 때까지 블로킹하지 않는다.
    pub fn read_x_m(&self) -> Result<f64, HwError> {
        let (response_tx, response_rx) = mpsc::channel();
        {
            let mut state = self.shared.state.lock().unwrap();
            state.pending_read = Some(ReadRequest { response_tx });
        }
        self.shared.cv.notify_all();
        return response_rx.recv().unwrap_or_else(|_| {
            Err(HwError::ReadFailed {
                reason: "RailQueue 워커가 응답 없이 종료됨".into(),
            })
        });
    }

    /// 물리적 엔드스톱까지 저속 이동해 원점을 다시 잡는다. 드물게, 명시적으로만
    /// 호출한다 — 홈잉이 끝날 때까지(최대 몇 분) 블로킹하며, 그동안 다른 모든
    /// 큐 요청(이동·읽기·정지)도 함께 대기한다. 기존 `home_rail`이 레일
    /// 뮤텍스를 홈잉 내내 붙들던 것과 같은 배타성이다.
    pub fn home(&self, end: RailEnd) -> Result<RailHomeResult, HwError> {
        let (response_tx, response_rx) = mpsc::channel();
        {
            let mut state = self.shared.state.lock().unwrap();
            state.pending_home = Some(HomeRequest { end, response_tx });
        }
        self.shared.cv.notify_all();
        return response_rx.recv().unwrap_or_else(|_| {
            Err(HwError::ReadFailed {
                reason: "RailQueue 워커가 응답 없이 종료됨".into(),
            })
        });
    }

    /// 진행 중인 이동을 즉시 정지시키고, 아직 전송되지 않은 대기 명령도 지운다.
    /// `enqueue`와 달리 지금 이동을 끝까지 기다리지 않고 곧바로 끊는다 —
    /// 명시적 중단(비상 정지 등)에만 쓴다.
    pub fn stop(&self) {
        let mut state = self.shared.state.lock().unwrap();
        state.pending = None;
        state.stop_requested = true;
        drop(state);
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

/// 대기 중인 읽기 요청이 있으면 처리하고 `true`를 반환한다. 없으면 `false`.
fn service_pending_read<R: RailDriver>(driver: &mut R, shared: &Arc<Shared>) -> bool {
    let request = {
        let mut state = shared.state.lock().unwrap();
        state.pending_read.take()
    };
    return match request {
        Some(request) => {
            let _ = request.response_tx.send(driver.read_x_m());
            true
        }
        None => false,
    };
}

/// 중단 요청이 있으면 처리(대기 명령 폐기 + 즉시 정지)하고 `true`를 반환한다.
fn service_pending_stop<R: RailDriver>(driver: &mut R, shared: &Arc<Shared>) -> bool {
    let requested = {
        let mut state = shared.state.lock().unwrap();
        if !state.stop_requested {
            return false;
        }
        state.stop_requested = false;
        state.pending = None;
        true
    };
    if requested && let Err(error) = driver.stop() {
        let mut state = shared.state.lock().unwrap();
        state.last_error = Some(error);
    }
    return requested;
}

/// 대기 중인 홈잉 요청이 있으면 처리하고 `true`를 반환한다. 없으면 `false`.
/// 홈잉 자체가 배타적·블로킹 동작이라 다른 요청과 인터리빙하지 않는다 —
/// 완료까지 통째로 워커를 점유한다.
fn service_pending_home<R: RailDriver>(driver: &mut R, shared: &Arc<Shared>) -> bool {
    let request = {
        let mut state = shared.state.lock().unwrap();
        state.pending_home.take()
    };
    return match request {
        Some(request) => {
            let _ = request.response_tx.send(driver.home(request.end));
            true
        }
        None => false,
    };
}

fn run_worker<R: RailDriver>(driver: &mut R, shared: &Arc<Shared>) {
    loop {
        wait_for_work(shared);
        if service_pending_stop(driver, shared) {
            continue;
        }
        if service_pending_read(driver, shared) {
            continue;
        }
        if service_pending_home(driver, shared) {
            continue;
        }
        let command = {
            let mut state = shared.state.lock().unwrap();
            state.pending.take()
        };
        match command {
            Some(command) => run_command(driver, shared, command),
            None => {
                if shared.state.lock().unwrap().shutdown {
                    return;
                }
            }
        }
    }
}

/// 처리할 일(대기 명령, 대기 읽기, 대기 홈잉, 중단 요청, 종료 신호) 중 하나가
/// 생길 때까지 블로킹한다.
fn wait_for_work(shared: &Arc<Shared>) {
    let mut state = shared.state.lock().unwrap();
    while state.pending.is_none()
        && state.pending_read.is_none()
        && state.pending_home.is_none()
        && !state.stop_requested
        && !state.shutdown
    {
        state = shared.cv.wait(state).unwrap();
    }
}

fn run_command<R: RailDriver>(driver: &mut R, shared: &Arc<Shared>, command: PendingCommand) {
    {
        let mut state = shared.state.lock().unwrap();
        state.moving = true;
    }
    shared.cv.notify_all();

    let result = match driver.command_abs_in_secs(command.target_m, command.duration_secs) {
        Ok(_) => poll_until_idle(driver, shared),
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

/// 이동이 끝날 때까지 논블로킹 `is_moving` 폴을 반복하며, 매 폴 사이에 대기 중인
/// 읽기·중단 요청을 처리한다 — 이동 중에도 위치 읽기가 최대 [`POLL_INTERVAL`] 안에
/// 응답하고, 중단 요청이 오면 이동을 끝까지 기다리지 않고 즉시 끊는다.
fn poll_until_idle<R: RailDriver>(driver: &mut R, shared: &Arc<Shared>) -> Result<(), HwError> {
    loop {
        service_pending_read(driver, shared);
        if service_pending_stop(driver, shared) {
            return Ok(());
        }
        if !driver.is_moving()? {
            return Ok(());
        }
        std::thread::sleep(POLL_INTERVAL);
    }
}

#[cfg(test)]
mod tests {
    use std::sync::mpsc;
    use std::time::{Duration, Instant};

    use super::*;

    struct MockDriver {
        sent_tx: mpsc::Sender<f64>,
        /// `release_tx.send(())`가 도착하면 그다음 `is_moving()` 폴부터 `false`를
        /// 반환한다 — 진짜 `AxmStatusReadInMotion`처럼 매 호출이 즉시 반환된다.
        release_rx: mpsc::Receiver<()>,
        released: bool,
        fail_target: Option<f64>,
        read_value_m: f64,
        stop_tx: mpsc::Sender<()>,
    }

    impl RailDriver for MockDriver {
        fn command_abs_in_secs(&mut self, x: f64, _duration_secs: f64) -> Result<f64, HwError> {
            self.released = false;
            self.sent_tx.send(x).unwrap();
            if self.fail_target == Some(x) {
                return Err(HwError::InvalidConfig {
                    reason: "mock failure".into(),
                });
            }
            self.read_value_m = x;
            return Ok(x);
        }

        fn is_moving(&mut self) -> Result<bool, HwError> {
            if !self.released && self.release_rx.try_recv().is_ok() {
                self.released = true;
            }
            return Ok(!self.released);
        }

        fn read_x_m(&mut self) -> Result<f64, HwError> {
            return Ok(self.read_value_m);
        }

        fn stop(&mut self) -> Result<(), HwError> {
            let _ = self.stop_tx.send(());
            return Ok(());
        }

        fn home(&mut self, end: RailEnd) -> Result<RailHomeResult, HwError> {
            let board_position_m = match end {
                RailEnd::Min => -1.0,
                RailEnd::Max => 1.0,
            };
            return Ok(RailHomeResult {
                board_position_m,
                board_zero_domain_m: 0.0,
            });
        }
    }

    /// `release_tx.send(())`가 도착하면 그 시점 이후의 `is_moving()` 폴부터 정지로 본다.
    /// 반환하는 `mpsc::Receiver<()>`는 `MockDriver::stop`이 실제로 호출될 때마다 신호를 받는다.
    fn spawn_mock(
        fail_target: Option<f64>,
    ) -> (
        RailQueue<MockDriver>,
        mpsc::Receiver<f64>,
        mpsc::Sender<()>,
        mpsc::Receiver<()>,
    ) {
        let (sent_tx, sent_rx) = mpsc::channel();
        let (release_tx, release_rx) = mpsc::channel();
        let (stop_tx, stop_rx) = mpsc::channel();
        let driver = MockDriver {
            sent_tx,
            release_rx,
            released: false,
            fail_target,
            read_value_m: 0.0,
            stop_tx,
        };
        let queue = RailQueue::spawn(driver);
        return (queue, sent_rx, release_tx, stop_rx);
    }

    #[test]
    fn enqueue_then_wait_idle_sends_the_command() {
        let (queue, sent_rx, release_tx, _stop_rx) = spawn_mock(None);
        release_tx.send(()).unwrap();
        queue.enqueue(1.0, 0.1);
        queue.wait_idle();
        assert_eq!(sent_rx.recv_timeout(Duration::from_secs(1)).unwrap(), 1.0);
    }

    #[test]
    fn latest_command_wins_while_previous_is_in_flight() {
        let (queue, sent_rx, release_tx, _stop_rx) = spawn_mock(None);

        queue.enqueue(1.0, 0.1);
        // Worker picks up 1.0 and polls is_moving() == true until released.
        assert_eq!(sent_rx.recv_timeout(Duration::from_secs(1)).unwrap(), 1.0);

        // These land while the worker is still executing 1.0 — only the last
        // one should ever reach the driver.
        queue.enqueue(2.0, 0.1);
        queue.enqueue(3.0, 0.1);

        release_tx.send(()).unwrap(); // finishes 1.0
        assert_eq!(sent_rx.recv_timeout(Duration::from_secs(1)).unwrap(), 3.0);
        release_tx.send(()).unwrap(); // finishes 3.0

        queue.wait_idle();
        assert!(sent_rx.try_recv().is_err(), "2.0 must never have been sent");
    }

    #[test]
    fn is_moving_reflects_executing_and_pending_state() {
        let (queue, sent_rx, release_tx, _stop_rx) = spawn_mock(None);

        assert!(!queue.is_moving());

        queue.enqueue(1.0, 0.1);
        assert_eq!(sent_rx.recv_timeout(Duration::from_secs(1)).unwrap(), 1.0);
        assert!(queue.is_moving(), "worker is polling is_moving() for 1.0");

        queue.enqueue(2.0, 0.1);
        assert!(queue.is_moving(), "2.0 is pending even though not yet sent");

        release_tx.send(()).unwrap(); // finishes 1.0
        assert_eq!(sent_rx.recv_timeout(Duration::from_secs(1)).unwrap(), 2.0);
        release_tx.send(()).unwrap(); // finishes 2.0
        queue.wait_idle();
        assert!(!queue.is_moving());
    }

    #[test]
    fn error_is_recorded_but_queue_keeps_processing() {
        let (queue, sent_rx, release_tx, _stop_rx) = spawn_mock(Some(2.0));

        // 2.0 fails inside command_abs_in_secs itself, so no is_moving() poll
        // happens for it and no release token is needed.
        queue.enqueue(2.0, 0.1);
        assert_eq!(sent_rx.recv_timeout(Duration::from_secs(1)).unwrap(), 2.0);

        // Poll until the worker has recorded the error and gone back to idle.
        let deadline = Instant::now() + Duration::from_secs(1);
        let error = loop {
            if let Some(error) = queue.take_error() {
                break error;
            }
            assert!(Instant::now() < deadline, "error was never recorded");
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
        let (queue, sent_rx, release_tx, _stop_rx) = spawn_mock(None);
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

    #[test]
    fn read_x_m_returns_promptly_even_while_a_command_is_in_flight() {
        let (queue, sent_rx, release_tx, _stop_rx) = spawn_mock(None);
        queue.enqueue(1.0, 0.1);
        assert_eq!(sent_rx.recv_timeout(Duration::from_secs(1)).unwrap(), 1.0);
        assert!(queue.is_moving(), "worker should be polling is_moving() now");

        let started = Instant::now();
        let value = queue.read_x_m().unwrap();
        // Must return within a couple of poll intervals, not wait for the
        // move to be released — proves reads never queue behind a move.
        assert!(started.elapsed() < Duration::from_millis(50));
        assert_eq!(value, 1.0);

        release_tx.send(()).unwrap();
        queue.wait_idle();
    }

    #[test]
    fn read_x_m_works_while_fully_idle() {
        let (queue, _sent_rx, _release_tx, _stop_rx) = spawn_mock(None);
        assert_eq!(queue.read_x_m().unwrap(), 0.0);
    }

    #[test]
    fn stop_interrupts_an_in_flight_move_without_waiting_for_it() {
        let (queue, sent_rx, _release_tx, stop_rx) = spawn_mock(None);
        queue.enqueue(1.0, 0.1);
        assert_eq!(sent_rx.recv_timeout(Duration::from_secs(1)).unwrap(), 1.0);
        assert!(queue.is_moving());

        let started = Instant::now();
        queue.stop();
        queue.wait_idle();
        // No release token was ever sent for 1.0's is_moving() poll — the
        // only way this returns is stop() cutting the move short.
        assert!(started.elapsed() < Duration::from_millis(50));
        stop_rx
            .recv_timeout(Duration::from_secs(1))
            .expect("driver.stop() must have been called");
        assert!(!queue.is_moving());
    }

    #[test]
    fn stop_discards_a_not_yet_sent_pending_command() {
        let (queue, sent_rx, _release_tx, stop_rx) = spawn_mock(None);
        queue.enqueue(1.0, 0.1);
        assert_eq!(sent_rx.recv_timeout(Duration::from_secs(1)).unwrap(), 1.0);

        queue.enqueue(2.0, 0.1); // pending, never sent
        queue.stop();
        stop_rx
            .recv_timeout(Duration::from_secs(1))
            .expect("driver.stop() must have been called");
        queue.wait_idle();
        assert!(sent_rx.try_recv().is_err(), "2.0 must never have been sent");
    }

    #[test]
    fn home_returns_the_drivers_result() {
        let (queue, _sent_rx, _release_tx, _stop_rx) = spawn_mock(None);
        let result = queue.home(RailEnd::Max).unwrap();
        assert_eq!(result.board_position_m, 1.0);
    }
}
