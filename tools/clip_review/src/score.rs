//! 클립 하나의 성적표.
//!
//! 바꾼 게 나아졌는지는 눈이 아니라 이 숫자로 판정한다. 재생 없이도 나오므로 스윕에
//! 그대로 쓸 수 있고, 재생 중에는 HUD 한 줄로 같은 값을 본다.
//!
//! **정답은 필터 밖에서 온다** — 생 삼각측량이 접수 평면을 지난 지점이다. 예측을 적합
//! 자신의 궤적과 견주면 적합이 통째로 밀려도 안 보인다.

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
    /// 접수 평면 타점 오차 [m].
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

        return Self {
            frames: reviewed.len(),
            detected,
            both: reviewed.observed.len(),
            track_switches: reviewed.frames.last().map_or(0, |state| state.seq),
            trigger,
            residual: [percentiles(&mut residual[0]), percentiles(&mut residual[1])],
            lead_error,
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
        println!(
            "  트랙        {}개 (갈아엎기 {}회 — 0이 정상)",
            self.track_switches + 1,
            self.track_switches
        );
        print!("  예측 오차  ");
        for (lead, error) in LEADS_SECS.iter().zip(&self.lead_error) {
            print!(
                " {lead:.1}s {:>7}",
                error.map_or("--".to_owned(), |e| format!("{:.1}cm", e * 100.0))
            );
        }
        println!();
        println!(
            "  타점 오차   {}   (y={:.2} 평면, 생 삼각측량 기준)",
            self.impact_error
                .map_or("--".to_owned(), |e| format!("{:.1} cm", e * 100.0)),
            table::DEFAULT_HIT_PLANE_Y
        );
        println!("────────────────────────────────────────────");
    }
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
