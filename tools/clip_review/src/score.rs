//! 클립 하나의 성적표.
//!
//! 바꾼 게 나아졌는지는 눈이 아니라 이 숫자로 판정한다. 재생 없이도 나오므로 스윕에
//! 그대로 쓸 수 있고, 재생 중에는 HUD 한 줄로 같은 값을 본다.
//!
//! **정답은 필터 밖에서 온다** — 생 삼각측량이 접수 평면을 지난 지점이다. 예측을 적합
//! 자신의 궤적과 견주면 적합이 통째로 밀려도 안 보인다.

use pingpong_bot::Vector3;
use pingpong_bot::constants::table;
use pingpong_bot::physics::TrajPoint;
use pingpong_bot::robot::motion::InterceptWindow;

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
    /// 로봇이 칠 수 있는 각 평면에서 **라켓이 얼마나 빗나가나** — `(y, 면내 오차, dx, dz)` [m].
    ///
    /// RMSE 는 궤적 전체의 값이라 "그래서 몇 cm 빗맞나"를 안 알려 준다. 접수 창의 평면마다
    /// 재면 그게 곧 라켓을 놓을 자리의 오차다. y 는 평면이 고정하므로 면내 `(x, z)` 만 센다.
    pub plane_error: Vec<(f64, Option<(f64, f64, f64)>)>,
    /// (예측 바운스 수, 실제 바운스 수) — 겹치는 구간 안에서만.
    ///
    /// `plane_error`·`impact_error`는 평면 **한 점**의 (dx, dz)만 본다 — 예측이 튐을
    /// 한 번 덜/더 겪어 궤적 모양 자체가 틀렸어도, 그 평면에서 우연히 자리가 비슷하면
    /// 오차가 작게 나온다(실측 fly_47: 실제 2바운스, 예측 1바운스인데 접수 평면에서
    /// 우연히 가까움). 개수가 다르면 위치가 맞아도 **물리 국면을 잘못 봤다는 뜻** — 이건
    /// 그 자체로 하드 신호로 남긴다.
    pub bounce_counts: Option<(usize, usize)>,
    /// 리드타임과 타점 오차를 잰 기준 평면 [m]. 클립마다 다를 수 있다.
    pub reference_plane: f64,
    /// 실제 공이 처음 튄 시각 [s]. 트리거보다 **뒤**면 커밋 순간에는 튐 정보가 아직
    /// 없다는 뜻이라, 스핀이든 반사계수든 그 시점엔 원리적으로 못 푼다.
    pub first_bounce_t: Option<f64>,
    /// 커밋 순간 바운스에서 스핀이 풀렸나. `None`이면 무회전(`ASSUMED_SPIN`=0)으로
    /// 굴린 예측이라, 그 샷의 튐 반사는 원리적으로 못 맞힌다 — y·z 오차의 주범이다.
    pub solved_spin: Option<Vector3>,
    /// 공이 [`Self::reference_plane`]에 **실제로** 도달한 시각 [s].
    ///
    /// 제어가 실제로 쓸 수 있는 시간은 `impact_t - trigger.1`이다 — 트리거를 얼마나
    /// 일찍 걸었나(리드타임)를 절대값으로 재는 유일한 기준이라, 스윕에서 후보끼리
    /// 상대 비교만 하지 않고 "그래서 몇 ms를 벌어 줬나"를 바로 볼 수 있다.
    pub impact_t: Option<f64>,
    /// 그 평면의 타점 오차 [m]. 제어가 실제로 쓰는 한 점이라 같이 둔다.
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
        //
        // 기준 평면은 **공이 실제로 지나는 것 중 가장 깊은 것**을 쓴다. 라켓이 접수 창
        // 안쪽에서 공을 치면 그보다 로봇 쪽 평면엔 공이 영영 안 온다 (실측 fly_10 은
        // y=0.14 에서 맞아 0.08 을 안 지난다). 고정 평면을 쓰면 그런 클립은 지표가 통째로
        // 비어 버린다.
        let plane = reference_plane(reviewed).unwrap_or(plane);
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

        let bounce_counts = reviewed
            .contract
            .as_ref()
            .and_then(|contract| count_bounces(reviewed, &contract.at_trigger.predicted));

        // 로봇이 실제로 칠 수 있는 평면들. 여기서 빗나간 거리가 곧 라켓 오차다.
        let plane_error = InterceptWindow::default()
            .hit_planes()
            .into_iter()
            .map(|plane| {
                let gap = reviewed
                    .contract
                    .as_ref()
                    .and_then(|contract| contract.at_trigger.predicted.at_plane(plane.y))
                    .zip(reviewed.observed_crossing_y(plane.y))
                    .map(|(guess, truth)| {
                        let (dx, dz) = (guess.position.x - truth.x, guess.position.z - truth.z);
                        return (dx.hypot(dz), dx, dz);
                    });
                return (plane.y, gap);
            })
            .collect();

        return Self {
            frames: reviewed.len(),
            detected,
            both: reviewed.observed.len(),
            track_switches: reviewed.frames.last().map_or(0, |state| state.seq),
            trigger,
            residual: [percentiles(&mut residual[0]), percentiles(&mut residual[1])],
            reference_plane: reference_plane(reviewed).unwrap_or(plane),
            impact_t: impact.map(|observed| observed.t),
            first_bounce_t: first_touchdown_t(reviewed),
            solved_spin: reviewed
                .contract
                .as_ref()
                .and_then(|contract| reviewed.frames.get(contract.frame))
                .and_then(|state| state.solved_spin),
            lead_error,
            plane_error,
            bounce_counts,
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
        // 위치가 맞아도 튐 횟수가 다르면 조용히 안 넘긴다 — 화면 두 줄 예산 안에서도 이건
        // 그냥 숫자가 아니라 "다른 물리를 맞았다"는 경보라 예외로 넣는다.
        let bounce = match self.bounce_counts {
            Some((p, o)) if p != o => format!("  BOUNCE 예측{p}≠실제{o}"),
            _ => String::new(),
        };
        return format!(
            "{resid}  fit {}  tracks {}  MISS {impact}{bounce}",
            self.trigger
                .map_or("--".to_owned(), |(_, _, n)| n.to_string()),
            self.track_switches + 1
        );
    }

    /// 성적표를 찍는다.
    pub fn print(&self, clip: &str) {
        println!("{}", self.render(clip));
    }

    /// 성적표 한 덩이. `--all` 은 이걸 파일로도 남긴다.
    pub fn render(&self, clip: &str) -> String {
        let mut out = Vec::new();
        out.push(format!("── {clip} 성적표 ──────────────────────────────"));
        out.push(format!(
            "  검출        cam0 {}/{} ({:.0}%)   cam1 {}/{} ({:.0}%)   동시 {} ({:.0}%)",
            self.detected[0],
            self.frames,
            rate(self.detected[0], self.frames),
            self.detected[1],
            self.frames,
            rate(self.detected[1], self.frames),
            self.both,
            rate(self.both, self.frames)
        ));
        for (slot, stats) in self.residual.iter().enumerate() {
            out.push(match stats {
                Some((p50, p95)) => {
                    format!("  적합 잔차   cam{slot} p50 {p50:.1} px  p95 {p95:.1} px")
                }
                None => format!("  적합 잔차   cam{slot} --"),
            });
        }
        out.push(match self.trigger {
            Some((frame, t, sightings)) => {
                format!("  트리거      f{frame} t={t:.3}s   그때 관측 {sightings}개")
            }
            None => "  트리거      끝내 안 걸림".to_owned(),
        });
        // 공 하나가 지나가면 트랙 하나가 서고 화면을 떠나면 죽는다. 클립에 공이 하나면
        // 1개가 정상이고, 그보다 많으면 비행 중에 놓쳐 다시 세운 것이다.
        out.push(format!(
            "  트랙        {}개 (공 하나면 1개가 정상)",
            self.track_switches + 1
        ));

        let leads: Vec<String> = LEADS_SECS
            .iter()
            .zip(&self.lead_error)
            .map(|(lead, error)| {
                format!(
                    "{lead:.1}s {:>7}",
                    error.map_or("--".to_owned(), |e| format!("{:.1}cm", e * 100.0))
                )
            })
            .collect();
        out.push(format!("  예측 오차   {}", leads.join(" ")));
        out.push(match (self.first_bounce_t, self.trigger) {
            (Some(bounce), Some((_, fired, _))) if bounce > fired => format!(
                "  튐 시각      t={bounce:.3}s — 트리거({fired:.3}s)보다 {:.0}ms 뒤. 커밋 순간엔 튐 정보가 없다",
                (bounce - fired) * 1000.0
            ),
            (Some(bounce), Some((_, fired, _))) => format!(
                "  튐 시각      t={bounce:.3}s — 트리거({fired:.3}s)보다 {:.0}ms 앞. 튐이 관측 안에 있다",
                (fired - bounce) * 1000.0
            ),
            _ => "  튐 시각      생 궤적에서 튐을 못 찾았다".to_owned(),
        });
        out.push(match self.solved_spin {
            Some(spin) => format!(
                "  스핀        바운스에서 풀림 ({:.0}, {:.0}, {:.0}) rad/s",
                spin.x, spin.y, spin.z
            ),
            None => "  스핀        안 풀림 — 무회전으로 굴렸다. 튐 반사가 y·z로 어긋난다".to_owned(),
        });

        out.push(match self.predicted_rmse {
            Some((all, axes, count)) => format!(
                "  궤적 RMSE   {:.1} cm   (x {:.1}  y {:.1}  z {:.1}) — 겹치는 {count}점",
                all * 100.0,
                axes.x * 100.0,
                axes.y * 100.0,
                axes.z * 100.0
            ),
            None => "  궤적 RMSE   -- (예측과 실제가 겹치는 구간이 없다)".to_owned(),
        });
        if let (Some((shift, best)), Some((now, _, _))) = (self.best_shift, self.predicted_rmse)
            && best < now * 0.8
        {
            out.push(format!(
                "              예측을 {:+.0} ms 밀면 {:.1} cm — 공간이 아니라 시간이 밀려 있다",
                shift * 1000.0,
                best * 100.0
            ));
        }
        if let Some((early, late)) = self.raw_speed {
            let decay = if early > 1e-6 {
                100.0 * (early - late) / early
            } else {
                0.0
            };
            out.push(format!(
                "  생 속력     앞 {early:.2} -> 뒤 {late:.2} m/s ({decay:+.0}%) — 예측은 drag=0 이라 안 준다"
            ));
        }
        out.push(format!(
            "  타점 오차   {}   (y={:.2} — 공이 실제로 지나는 가장 깊은 접수 평면)",
            self.impact_error
                .map_or("--".to_owned(), |e| format!("{:.1} cm", e * 100.0)),
            self.reference_plane
        ));
        out.push(match self.bounce_counts {
            Some((p, o)) if p == o => format!("  바운스 횟수 예측 {p} = 실제 {o}"),
            Some((p, o)) => format!(
                "  바운스 횟수 예측 {p} ≠ 실제 {o} — 평면 오차가 작아도 믿지 마라. \
                 튐 국면 자체를 놓쳤다"
            ),
            None => "  바운스 횟수 -- (예측 없음)".to_owned(),
        });
        out.push("  접수 창에서 라켓이 빗나가는 거리 (면내 x·z):".to_owned());
        for (y, gap) in &self.plane_error {
            out.push(match gap {
                Some((all, dx, dz)) => format!(
                    "    y={y:.2}  {:>5.1} cm   (dx {:+5.1}  dz {:+5.1})",
                    all * 100.0,
                    dx * 100.0,
                    dz * 100.0
                ),
                None => format!("    y={y:.2}     --   (예측이나 실제가 이 평면을 안 지남)"),
            });
        }
        out.push("────────────────────────────────────────────".to_owned());
        return out.join("\n");
    }
}

/// 공이 실제로 지나는 접수 평면 중 가장 로봇에 가까운 것 [m].
///
/// 라켓이 접수 창 안쪽에서 치면 그보다 깊은 평면엔 공이 안 온다. 그런 클립도 지표가
/// 나오도록 실제로 지나는 평면을 고른다.
fn reference_plane(reviewed: &Reviewed) -> Option<f64> {
    return InterceptWindow::default()
        .hit_planes()
        .into_iter()
        .map(|plane| plane.y)
        .filter(|y| reviewed.observed_crossing_y(*y).is_some())
        .fold(None, |best: Option<f64>, y| {
            Some(best.map_or(y, |b| b.min(y)))
        });
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

/// (예측 바운스 수, 실제 바운스 수) — 예측 궤적이 덮는 시간 구간 안의 실제 관측만 센다.
/// 예측 뒤(로봇을 이미 지난 자리)에서 실제가 몇 번 더 튀든 그건 이 예측의 책임이 아니다.
///
/// `TrajAnalysis::detect_bounces`는 안 쓴다 — 그건 e·μ **측정**용이라 신뢰도 필터가
/// 빡빡해서(창 회귀 vin_z·접선속도·e 범위) 애매한 바운스를 조용히 버린다. 여기서는
/// "튕겼는가"만 알면 되고, 놓치는 쪽이 훨씬 나쁘다. 그래서 원래의 느슨한 감지 조건만 쓴다.
///
/// **한계**: `predicted`는 `PREDICT_UNTIL_Y`(로봇 마운트)에서 끊긴다 — 실제가 그 뒤에서
/// 튀는 건 이 함수의 창 밖이라 여기 안 잡힌다(실측 fly_47: 예측이 마운트 도달 전에 끝나서
/// 둘 다 0으로 나옴, 진짜 갈라짐은 `predicted_rmse`가 잡음 — 20.4→23.3cm). 이 필드는
/// "겹치는 구간 안에서 튐 국면 자체가 갈렸나"만 보는 보조 신호고, 전체 궤적 오차의
/// 최종 판정은 여전히 `predicted_rmse`다.
fn count_bounces(reviewed: &Reviewed, predicted: &pingpong_bot::vision::Track) -> Option<(usize, usize)> {
    let (first, last) = (predicted.first()?.t.as_secs_f64(), predicted.last()?.t.as_secs_f64());
    let predicted_points: Vec<TrajPoint> = predicted
        .iter()
        .map(|s| TrajPoint {
            t: s.t.as_secs_f64(),
            pos: s.position,
            pixels: Vec::new(),
        })
        .collect();
    let observed_points: Vec<TrajPoint> = reviewed
        .observed
        .iter()
        .filter(|o| o.t >= first && o.t <= last)
        .map(|o| TrajPoint {
            t: o.t,
            pos: o.point,
            pixels: Vec::new(),
        })
        .collect();
    return Some((
        count_touchdowns(&predicted_points),
        count_touchdowns(&observed_points),
    ));
}

/// 테이블 근처에서 vz 부호가 뒤집히는 횟수 — `detect_bounces`의 감지 조건과 같지만
/// 신뢰도 재확인 없이 다 센다. 목적이 다르면(측정 vs 있었나 없었나) 필터도 다르다.
/// 생 궤적에서 처음 튄 시각 [s]. [`count_touchdowns`]와 같은 판정을 쓴다 — 개수와
/// 시각이 다른 규칙으로 갈리면 안 된다.
fn first_touchdown_t(reviewed: &Reviewed) -> Option<f64> {
    let points: Vec<TrajPoint> = reviewed
        .observed
        .iter()
        .map(|o| TrajPoint {
            t: o.t,
            pos: o.point,
            pixels: Vec::new(),
        })
        .collect();
    return touchdown_times(&points).first().copied();
}

/// 튄 순간들의 시각 [s].
fn touchdown_times(points: &[TrajPoint]) -> Vec<f64> {
    let floor = table::SURFACE_Z + 0.10;
    let mut out = Vec::new();
    let mut i = 1usize;
    while i + 1 < points.len() {
        let dt_in = points[i].t - points[i - 1].t;
        let dt_out = points[i + 1].t - points[i].t;
        if dt_in > 1e-4 && dt_out > 1e-4 {
            let vin_z = (points[i].pos.z - points[i - 1].pos.z) / dt_in;
            let vout_z = (points[i + 1].pos.z - points[i].pos.z) / dt_out;
            if vin_z < -0.25 && vout_z > 0.15 && points[i].pos.z < floor {
                out.push(points[i].t);
                i += 3;
                continue;
            }
        }
        i += 1;
    }
    return out;
}

fn count_touchdowns(points: &[TrajPoint]) -> usize {
    return touchdown_times(points).len();
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
