//! 추적 전 첫 상태를 세운다. 삼각측량 1회이고, 여기서만 다른 카메라를 기다린다.
//!
//! 카메라가 동기되어 있지 않으므로 두 시선의 시각이 다르다. 시드 시각은 중간으로 잡고,
//! 어긋난 만큼은 호출자가 공분산으로 흡수한다.

use std::time::Duration;

use crate::Point3;
use crate::camera;
use crate::camera::Triangulate;
use crate::constants::table;
use crate::defaults::vision::seed::MAX_REPROJECTION_PX;

use super::detect::Candidate;

/// 시드를 받아 줄 플레이 부피 여유 [m].
const VOLUME_MARGIN: f64 = 0.5;

type View<'a> = (&'a camera::Params, Candidate, Duration);

/// 모든 쌍을 삼각측량하고 물리 게이트(비행 부피)를 태운 뒤, 나머지 카메라 중 재투영이
/// 맞는 개수로 고른다. 카메라 수에 의존하지 않는다.
///
/// 실패하면 다음 프레임에 다시 시도한다. 나쁜 시드보다 늦은 시드가 낫다.
pub fn seed_state(views: &[View<'_>]) -> Option<Point3> {
    let mut best: Option<(usize, f64, Point3)> = None;
    for a in 0..views.len() {
        for b in (a + 1)..views.len() {
            let pair = [
                (views[a].0.projection_matrix(), views[a].1.pixel),
                (views[b].0.projection_matrix(), views[b].1.pixel),
            ];
            let Some(point) = Triangulate::views(&pair) else {
                continue;
            };
            if outside_volume(point) {
                continue;
            }
            let (votes, worst) = agreement(views, point);
            if votes < 2 {
                continue;
            }
            let better = best.is_none_or(|(v, w, _)| votes > v || (votes == v && worst < w));
            if better {
                best = Some((votes, worst, point));
            }
        }
    }
    return best.map(|(_, _, point)| point);
}

/// 몇 대가 이 점에 동의하나, 그리고 그중 최악 재투영 오차 [px].
fn agreement(views: &[View<'_>], point: Point3) -> (usize, f64) {
    let mut votes = 0;
    let mut worst = 0.0_f64;
    for (params, candidate, _) in views {
        let Some(projected) = params.project_world_unclipped(point) else {
            continue;
        };
        let error = (projected - candidate.pixel).norm();
        if error <= MAX_REPROJECTION_PX {
            votes += 1;
            worst = worst.max(error);
        }
    }
    return (votes, worst);
}

/// 뷰들이 얼마나 어긋났나. 시드 위치 공분산을 부풀릴 근거다.
pub fn skew(views: &[View<'_>]) -> Duration {
    let times = || views.iter().map(|(_, _, t)| *t);
    let (Some(newest), Some(oldest)) = (times().max(), times().min()) else {
        return Duration::ZERO;
    };
    return newest - oldest;
}

/// 시드 시각. 어긋난 두 시선의 중간이라 오차가 절반이 된다.
pub fn midpoint(views: &[View<'_>]) -> Duration {
    let times = || views.iter().map(|(_, _, t)| *t);
    let (Some(newest), Some(oldest)) = (times().max(), times().min()) else {
        return Duration::ZERO;
    };
    return oldest + (newest - oldest) / 2;
}

fn outside_volume(p: Point3) -> bool {
    return p.y < -VOLUME_MARGIN
        || p.y > table::LENGTH_Y + VOLUME_MARGIN
        || p.x < -VOLUME_MARGIN
        || p.x > table::WIDTH_X + VOLUME_MARGIN
        || p.z < table::SURFACE_Z - VOLUME_MARGIN
        || p.z > table::SURFACE_Z + 2.0;
}
