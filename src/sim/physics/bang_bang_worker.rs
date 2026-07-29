//! bang-bang 스윙 계획을 물리 스레드 밖에서 돌리는 백그라운드 워커.
//!
//! `plan_bang_bang_swing`은 최대 ~350스텝의 RNEA/자코비안 계산을 돌아
//! 실제로 수십~수백 ms가 걸릴 수 있다(`.omc/progress.txt`). 물리 스레드가
//! 이 계산을 `SimWorld` 락 아래에서 동기적으로 기다리면, 같은 `step()` 호출
//! 안에서 스윙 계획이 Rapier 적분보다 먼저 실행되므로 그동안 공의 물리도
//! 함께 멈춘다 — 시뮬레이션 시계 전체가 실제 시간보다 그만큼 뒤처지고,
//! 사용자에게는 "팔이 늦게 움직인다"(공 도착 시점과 스윙 시작이 실제
//! 시계 기준으로 어긋난다)로 보인다.
//!
//! 이 워커는 계산을 전용 스레드로 옮긴다 — 물리 스레드는 요청만 보내고
//! (블로킹 없이) 결과를 매 틱 논블로킹으로 폴링한다. 계산이 진행되는
//! 동안에도 공은 매 틱 정상적으로 전진한다. 결과가 도착하면 호출부가
//! "요청 시각 대비 지금까지 흐른 sim 시간"만큼 재생 시작 지점을 앞으로
//! 당겨(`robot::State::replace_bang_bang_swing_at`) 보정한다.

use crate::robot;
use std::sync::Arc;
use std::thread;

use crossbeam_channel::{Receiver, Sender, unbounded};

use crate::error::DomainError;
use crate::swing;
use crate::{Arm, Prediction};

struct Request {
    id: u64,
    arm: Arc<Arm>,
    predictions: Vec<Prediction>,
    start: robot::Pose,
}

struct Response {
    id: u64,
    result: Result<swing::bang_bang::PlannedIntercept, DomainError>,
}

/// 진행 중인 요청 하나의 메타데이터 — 응답이 오면 이 시각 기준으로
/// 재생 시작 지점을 보정한다.
struct Inflight {
    id: u64,
    requested_at_sim_time: f64,
}

/// bang-bang 계획 전용 백그라운드 워커 — 요청 1개씩 순차 처리.
pub struct BangBangWorker {
    tx: Sender<Request>,
    rx: Receiver<Response>,
    next_id: u64,
    inflight: Option<Inflight>,
}

impl BangBangWorker {
    pub fn new() -> Self {
        let (req_tx, req_rx) = unbounded::<Request>();
        let (res_tx, res_rx) = unbounded::<Response>();
        thread::spawn(move || {
            for request in req_rx.iter() {
                let result = swing::Planner::plan_bang_bang(
                    &request.arm,
                    &request.predictions,
                    &request.start,
                );
                // 수신측(SimWorld)이 이미 drop돼 채널이 끊겼으면 조용히 종료.
                if res_tx
                    .send(Response {
                        id: request.id,
                        result,
                    })
                    .is_err()
                {
                    break;
                }
            }
        });
        return Self {
            tx: req_tx,
            rx: res_rx,
            next_id: 0,
            inflight: None,
        };
    }

    /// 계산 중인 요청이 있는지.
    pub fn is_busy(&self) -> bool {
        return self.inflight.is_some();
    }

    /// 이미 계산 중이면 아무것도 안 하고 `false`. 아니면 새 요청을 보내고
    /// `true` — 호출부는 이번 틱에 결과를 기다리지 않고 곧바로 리턴해야
    /// 한다(그래야 물리 스텝·공 적분이 막히지 않는다).
    pub fn submit(
        &mut self,
        sim_time: f64,
        arm: Arc<Arm>,
        predictions: Vec<Prediction>,
        start: robot::Pose,
    ) -> bool {
        if self.inflight.is_some() {
            return false;
        }
        self.next_id += 1;
        let id = self.next_id;
        if self
            .tx
            .send(Request {
                id,
                arm,
                predictions,
                start,
            })
            .is_err()
        {
            // 워커 스레드가 죽었으면(패닉 등) 조용히 무시 — 다음 틱에 재시도.
            return false;
        }
        self.inflight = Some(Inflight {
            id,
            requested_at_sim_time: sim_time,
        });
        return true;
    }

    /// 지금 추적 중인 요청과 일치하는 응답이 도착했으면 소비해 반환한다 —
    /// `(요청 시각, 결과)`. 호출부가 `sim_time - 요청 시각`으로 재생 시작
    /// 지점을 보정한다. `cancel_inflight` 이후 늦게 도착한 오래된 응답은
    /// 추적 중인 id와 안 맞으므로 조용히 버려진다.
    pub fn poll(
        &mut self,
    ) -> Option<(f64, Result<swing::bang_bang::PlannedIntercept, DomainError>)> {
        let mut latest = None;
        // 이론상 한 번에 응답이 하나만 있어야 하지만, 방어적으로 채널을
        // 전부 드레인하며 지금 추적 중인 id와 일치하는 마지막 것만 취한다.
        while let Ok(response) = self.rx.try_recv() {
            if self
                .inflight
                .as_ref()
                .is_some_and(|inflight| inflight.id == response.id)
            {
                latest = Some(response);
            }
        }
        let response = latest?;
        let requested_at = self
            .inflight
            .take()
            .expect("응답 id가 일치했으므로 inflight 존재")
            .requested_at_sim_time;
        return Some((requested_at, response.result));
    }

    /// 진행 중인 요청 추적을 버린다(응답이 와도 무시) — 새 공이 발사돼
    /// 이전 계획이 더는 의미 없을 때 호출한다(`shoot_ball`).
    pub fn cancel_inflight(&mut self) {
        self.inflight = None;
    }
}

#[cfg(test)]
mod tests {
    use std::time::{Duration, Instant};

    use super::*;

    fn sample_request_pieces() -> (Arc<Arm>, Vec<Prediction>, robot::Pose) {
        let robot = crate::defaults::primitive_4dof().expect("4dof robot");
        let arm = robot.arm;
        let start = arm.initial_state();
        let start_pose = robot::Pose::new(start.rail_x(), start.joints().clone());
        let prediction = Prediction {
            time_to_impact_secs: 0.3,
            impact_position: crate::Point3::new(
                crate::constants::table::WIDTH_X * 0.5,
                0.30,
                0.932,
            ),
            incoming_velocity: nalgebra::Vector3::new(0.0, -6.01, 1.51),
        };
        return (arm, vec![prediction], start_pose);
    }

    #[test]
    fn submit_then_busy_until_polled() {
        let mut worker = BangBangWorker::new();
        let (arm, predictions, start) = sample_request_pieces();

        assert!(!worker.is_busy());
        let sent = worker.submit(0.0, Arc::clone(&arm), predictions.clone(), start.clone());
        assert!(sent, "첫 요청은 보내져야 함");
        assert!(worker.is_busy());

        // 이미 계산 중일 때 새 요청을 보내면 무시돼야 한다(false 반환,
        // 기존 inflight 그대로 유지) — 이게 없으면 매 틱 새 스레드/요청이
        // 쌓여 워커의 "요청 1개씩만 처리" 전제가 깨진다.
        let sent_again = worker.submit(0.001, Arc::clone(&arm), predictions, start);
        assert!(!sent_again, "이미 계산 중이면 새 요청을 보내지 않아야 함");

        // 결과가 올 때까지 논블로킹으로 폴링 — 실제 배경 스레드라 시간이
        // 걸리므로 타임아웃을 두고 스핀한다(플레이키 방지용 여유 5초).
        let deadline = Instant::now() + Duration::from_secs(5);
        let mut resolved = None;
        while Instant::now() < deadline {
            if let Some(result) = worker.poll() {
                resolved = Some(result);
                break;
            }
            std::thread::sleep(Duration::from_millis(1));
        }
        let (requested_at, _result) = resolved.expect("5초 안에 결과가 와야 함");
        assert_eq!(requested_at, 0.0, "요청 시각이 그대로 보존돼야 함");
        assert!(
            !worker.is_busy(),
            "poll 이후에는 더 이상 busy가 아니어야 함"
        );
        assert!(
            worker.poll().is_none(),
            "한 번 소비한 응답을 다시 반환하면 안 됨"
        );
    }

    #[test]
    fn cancel_inflight_drops_stale_response() {
        let mut worker = BangBangWorker::new();
        let (arm, predictions, start) = sample_request_pieces();

        worker.submit(0.0, arm, predictions, start);
        assert!(worker.is_busy());
        worker.cancel_inflight();
        assert!(!worker.is_busy());

        // 취소 후에도 배경 스레드는 계산을 끝까지 돌려 응답을 보낼 수
        // 있지만, 추적 중인 id가 없으므로(inflight=None) poll은 그 응답을
        // 절대 반환하면 안 된다 — 새 공이 발사됐는데 옛 공 기준 계획이
        // 뒤늦게 커밋되는 사고를 막는 장치.
        std::thread::sleep(Duration::from_millis(300));
        assert!(
            worker.poll().is_none(),
            "취소된 요청의 응답은 항상 버려져야 함"
        );
    }
}
