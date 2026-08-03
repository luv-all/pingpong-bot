//! 클립 하나의 성적표.
//!
//! 바꾼 게 나아졌는지는 눈이 아니라 이 숫자로 판정한다. 재생 없이도 나오므로 스윕에
//! 그대로 쓸 수 있고, 재생 중에는 HUD 한 줄로 같은 값을 본다.
//!
//! **정답은 필터 밖에서 온다** — 생 삼각측량이 접수 평면을 지난 지점이다. 예측을 적합
//! 자신의 궤적과 견주면 적합이 통째로 밀려도 안 보인다.

use pingpong_bot::Vector3;
use pingpong_bot::constants::table;

use crate::track::Reviewed;

/// 예측 오차를 잴 리드타임 [s] — 실제 도달까지 남은 시간.
pub const LEADS_SECS: [f64; 4] = [0.4, 0.3, 0.2, 0.1];

pub struct Score {
    pub frames: usize,
    /// 캠별 검출 프레임 수.
    pub detected: [usize; 2],
    /// 두 캠이 같이 잡아 삼각측량된 프레임 수.
    pub both: usize,
    /// 트랙을 몇 번 갈아엎었나. 0이 정상이다.
    pub track_switches: u64,
    /// 트리거 (프레임, 시각, 그때 적합이 쓰던 관측 수).
    pub trigger: Option<(usize, f64, usize)>,
    /// 적합 잔차 [px], 캠별 (p50, p95).
    pub residual: [Option<(f64, f64)>; 2],
    /// 리드타임별 예측 오차 [m] — [`LEADS_SECS`] 순서.
    pub lead_error: Vec<Option<f64>>,
    /// 얼린 예측이 그 뒤 실제 궤적에서 얼마나 벗어났나 — (RMSE, 축별 RMSE, 짝 수) [m].
    ///
    /// 평면 하나에서 재는 타점 오차는 평면을 어디 두느냐에 흔들린다. 겹치는 구간 전체를
    /// 재면 그 자의성이 없어지고, 축별로 나누면 어느 방향이 나쁜지가 바로 보인다.
    pub predicted_rmse: Option<(f64, Vector3, usize)>,
    /// 예측을 시간축으로 얼마나 밀면 RMSE 가 최소가 되나 [s], 그리고 그때의 RMSE [m].
    ///
    /// 궤적 모양은 맞는데 그 위 위치가 밀린 경우를 가른다. 이 값이 크고 밀었을 때 RMSE 가
    /// 확 줄면 공간 오차가 아니라 **시간 오차**다 — 속도를 잘못 봤거나 타임스탬프가 밀렸다.
    pub best_shift: Option<(f64, f64)>,
    /// 생 궤적의 속력 [m/s] — (앞 구간, 뒷 구간).
    ///
    /// 공이 실제로 감속하는가. `PhysicsParams::default().drag` 가 0 이라 예측은 절대
    /// 감속하지 않는다. 입력이 감속하는데 예측이 안 하면 오차가 **시간에 따라 자란다** —
    /// 시간축으로 밀어도 안 줄어드는 오차의 정체가 그것이다.
    pub raw_speed: Option<(f64, f64)>,
    /// 접수 평면 타점 오차 [m]. 제어가 실제로 쓰는 한 점이라 같이 둔다.
    pub impact_error: Option<f64>,
}

impl Score {
    pub fn of(reviewed: &Reviewed) -> Self {
        let mut detected = [0usize; 2];
        let mut residual: [Vec<f64>; 2] = [Vec::new(), Vec::new()];
        for state in &reviewed.frames {
            for slot in 0..2 {
                if state.pixels[slot].is_some() {
                    detected[slot] += 1;
                }
                if let Some(px) = state.residual_px[slot] {
                    residual[slot].push(px);
                }
            }
        }

        let trigger = reviewed.contract.as_ref().map(|contract| {
            let sightings = reviewed
                .frames
                .get(contract.frame)
                .map_or(0, |state| state.sightings);
            return (contract.frame, contract.t, sightings);
        });

        let plane = table::DEFAULT_HIT_PLANE_Y;
        let impact_error = reviewed
            .contract
            .as_ref()
            .and_then(|contract| contract.at_trigger.predicted.at_plane(plane))
            .zip(reviewed.observed_crossing_y(plane))
            .map(|(guess, truth)| (guess.position - truth).norm());

        // 실제 도달 시각·지점이 정답이다. 리드는 거기서 거꾸로 센다 — "지금 커밋하면
        // 얼마나 틀리나"와 같은 질문이다.
        let impact = reviewed
            .observed
            .windows(2)
            .find(|w| w[0].point.y >= plane && w[1].point.y < plane)
            .map(|w| w[1]);
        let truth = reviewed.observed_crossing_y(plane);
        let lead_error = LEADS_SECS
            .iter()
            .map(|lead| {
                let (impact, truth) = (impact?, truth?);
                let at = impact.t - lead;
                // 프레임 반 칸 안쪽에서 가장 가까운 프레임의 예측만 짝으로 인정한다.
                let tolerance = 0.5 / reviewed.fps;
                return reviewed
                    .frames
                    .iter()
                    .enumerate()
                    .filter(|(index, _)| (reviewed.time_of(*index) - at).abs() <= tolerance)
                    .find_map(|(_, state)| state.predicted_impact)
                    .map(|guess| (guess - truth).norm());
            })
            .collect();

        // 얼린 예측 대 실제 궤적, 겹치는 구간 전체.
        let predicted_rmse = reviewed.contract.as_ref().and_then(|contract| {
            let mut squared = Vector3::zeros();
            let mut count = 0usize;
            for observed in &reviewed.observed {
                let Some(guess) = contract
                    .at_trigger
                    .predicted
                    .at_time(std::time::Duration::from_secs_f64(observed.t.max(0.0)))
                else {
                    continue;
                };
                let gap = guess.position - observed.point;
                squared += gap.component_mul(&gap);
                count += 1;
            }
            if count == 0 {
                return None;
            }
            let per_axis = (squared / count as f64).map(f64::sqrt);
            return Some((per_axis.norm(), per_axis, count));
        });

        // 시간축으로 밀어 보며 RMSE 가 최소가 되는 지점. ±100 ms 를 2 ms 로 훑는다.
        let best_shift = reviewed.contract.as_ref().and_then(|contract| {
            let mut best: Option<(f64, f64)> = None;
            for step in -50..=50 {
                let shift = f64::from(step) * 0.002;
                let Some((rmse, _, _)) = rmse_at(&contract.at_trigger.predicted, reviewed, shift)
                else {
                    continue;
                };
                if best.is_none_or(|(_, low)| rmse < low) {
                    best = Some((shift, rmse));
                }
            }
            return best;
        });

        let raw_speed = speed_early_late(reviewed);

        return Self {
            frames: reviewed.len(),
            detected,
            both: reviewed.observed.len(),
            track_switches: reviewed.frames.last().map_or(0, |state| state.seq),
            trigger,
            residual: [percentiles(&mut residual[0]), percentiles(&mut residual[1])],
            lead_error,
            predicted_rmse,
            best_shift,
            raw_speed,
            impact_error,
        };
    }

    /// 재생 중 HUD 한 줄. 지금 프레임의 값이 아니라 클립 전체의 성적이다 —
    /// 바꾼 게 나아졌는지는 프레임 하나로 못 본다.
    pub fn hud_line(&self) -> String {
        let resid = match self.residual[0].zip(self.residual[1]) {
            Some((left, right)) => format!("resid {:.1}/{:.1}px", left.0, right.0),
            None => "resid --".to_owned(),
        };
        let impact = self
            .impact_error
            .map_or("--".to_owned(), |e| format!("{:.0}cm", e * 100.0));
        return format!(
            "{resid}  fit {}  tracks {}  MISS {impact}",
            self.trigger
                .map_or("--".to_owned(), |(_, _, n)| n.to_string()),
            self.track_switches + 1
        );
    }

    /// 성적표. pass 1이 끝나자마자 한 번 찍는다.
    pub fn print(&self, clip: &str) {
        println!("── {clip} 성적표 ──────────────────────────────");
        println!(
            "  검출        cam0 {}/{} ({:.0}%)   cam1 {}/{} ({:.0}%)   동시 {} ({:.0}%)",
            self.detected[0],
            self.frames,
            rate(self.detected[0], self.frames),
            self.detected[1],
            self.frames,
            rate(self.detected[1], self.frames),
            self.both,
            rate(self.both, self.frames)
        );
        for (slot, stats) in self.residual.iter().enumerate() {
            match stats {
                Some((p50, p95)) => {
                    println!("  적합 잔차   cam{slot} p50 {p50:.1} px  p95 {p95:.1} px")
                }
                None => println!("  적합 잔차   cam{slot} --"),
            }
        }
        match self.trigger {
            Some((frame, t, sightings)) => {
                println!("  트리거      f{frame} t={t:.3}s   그때 관측 {sightings}개")
            }
            None => println!("  트리거      끝내 안 걸림"),
        }
        // 공 하나가 지나가면 트랙 하나가 서고 화면을 떠나면 죽는다. 클립에 공이 하나면
        // 1개가 정상이고, 그보다 많으면 비행 중에 놓쳐 다시 세운 것이다.
        println!(
            "  트랙        {}개 (공 하나면 1개가 정상)",
            self.track_switches + 1
        );
        print!("  예측 오차  ");
        for (lead, error) in LEADS_SECS.iter().zip(&self.lead_error) {
            print!(
                " {lead:.1}s {:>7}",
                error.map_or("--".to_owned(), |e| format!("{:.1}cm", e * 100.0))
            );
        }
        println!();
        match self.predicted_rmse {
            Some((all, axes, count)) => println!(
                "  궤적 RMSE   {:.1} cm   (x {:.1}  y {:.1}  z {:.1}) — 겹치는 {count}점",
                all * 100.0,
                axes.x * 100.0,
                axes.y * 100.0,
                axes.z * 100.0
            ),
            None => println!("  궤적 RMSE   -- (예측과 실제가 겹치는 구간이 없다)"),
        }
        if let (Some((shift, best)), Some((now, _, _))) = (self.best_shift, self.predicted_rmse)
            && best < now * 0.8
        {
            println!(
                "              예측을 {:+.0} ms 밀면 {:.1} cm — 공간이 아니라 시간이 밀려 있다",
                shift * 1000.0,
                best * 100.0
            );
        }
        if let Some((early, late)) = self.raw_speed {
            let decay = if early > 1e-6 {
                100.0 * (early - late) / early
            } else {
                0.0
            };
            println!(
                "  생 속력     앞 {early:.2} -> 뒤 {late:.2} m/s ({decay:+.0}%) — 예측은 drag=0 이라 안 준다"
            );
        }
        println!(
            "  타점 오차   {}   (y={:.2} 평면 — 평면 위치에 흔들리므로 참고용)",
            self.impact_error
                .map_or("--".to_owned(), |e| format!("{:.1} cm", e * 100.0)),
            table::DEFAULT_HIT_PLANE_Y
        );
        println!("────────────────────────────────────────────");
    }
}

/// 생 궤적의 앞 1/3 과 뒤 1/3 에서 잰 속력 [m/s].
///
/// 표본이 튀므로 구간 양 끝의 차분으로 낸다 — 인접 프레임 차분은 잡음이 지배한다.
fn speed_early_late(reviewed: &Reviewed) -> Option<(f64, f64)> {
    let points = &reviewed.observed;
    if points.len() < 6 {
        return None;
    }
    let third = points.len() / 3;
    let span = |from: usize, to: usize| -> Option<f64> {
        let (a, b) = (points.get(from)?, points.get(to)?);
        let dt = b.t - a.t;
        return (dt > 1e-6).then(|| (b.point - a.point).norm() / dt);
    };
    return Some((
        span(0, third)?,
        span(points.len() - 1 - third, points.len() - 1)?,
    ));
}

/// 예측을 `shift` [s] 만큼 민 뒤의 (RMSE, 축별 RMSE, 짝 수) [m].
fn rmse_at(
    predicted: &pingpong_bot::vision::Track,
    reviewed: &Reviewed,
    shift: f64,
) -> Option<(f64, Vector3, usize)> {
    let mut squared = Vector3::zeros();
    let mut count = 0usize;
    for observed in &reviewed.observed {
        let at = observed.t + shift;
        if at < 0.0 {
            continue;
        }
        let Some(guess) = predicted.at_time(std::time::Duration::from_secs_f64(at)) else {
            continue;
        };
        let gap = guess.position - observed.point;
        squared += gap.component_mul(&gap);
        count += 1;
    }
    if count == 0 {
        return None;
    }
    let per_axis = (squared / count as f64).map(f64::sqrt);
    return Some((per_axis.norm(), per_axis, count));
}

fn rate(part: usize, whole: usize) -> f64 {
    if whole == 0 {
        return 0.0;
    }
    return 100.0 * part as f64 / whole as f64;
}

/// (p50, p95). 표본이 없으면 `None`.
fn percentiles(samples: &mut [f64]) -> Option<(f64, f64)> {
    if samples.is_empty() {
        return None;
    }
    samples.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let at = |q: f64| samples[((samples.len() - 1) as f64 * q).round() as usize];
    return Some((at(0.50), at(0.95)));
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn percentiles_pick_the_right_samples() {
        let mut samples: Vec<f64> = (0..=100).map(f64::from).collect();
        let (p50, p95) = percentiles(&mut samples).expect("표본");
        assert!((p50 - 50.0).abs() < 1e-9, "p50={p50}");
        assert!((p95 - 95.0).abs() < 1e-9, "p95={p95}");
        assert!(percentiles(&mut []).is_none());
    }

    /// 표본이 하나뿐이어도 터지지 않아야 한다 — 검출이 거의 안 되는 클립이 있다.
    #[test]
    fn a_single_sample_is_both_percentiles() {
        assert_eq!(percentiles(&mut [3.5]), Some((3.5, 3.5)));
    }
}
