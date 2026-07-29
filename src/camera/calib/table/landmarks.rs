//! 탁구대 랜드마크 목록·메시.

use crate::Point3;
use crate::constants::table;

use super::Landmark;

pub use crate::constants::camera::TABLE_LANDMARK_COUNT;
pub use crate::defaults::calib::MAX_REPROJ_RMSE_PX;

/// 팀 규약 8점 (순서 고정 — 클릭도 이 순서).
///
/// 1–4 꼭짓점, 5–7 내부(중앙선 Y=L/4, L/2, 3L/4), 8 로봇쪽 변 중점.
pub fn table_landmarks() -> [Landmark; TABLE_LANDMARK_COUNT] {
    let z = table::SURFACE_Z;
    let w = table::WIDTH_X;
    let l = table::LENGTH_Y;
    return [
        Landmark {
            id: "c00",
            prompt: "1/8 robot-left corner (0,0)",
            world: Point3::new(0.0, 0.0, z),
        },
        Landmark {
            id: "c10",
            prompt: "2/8 robot-right corner (W,0)",
            world: Point3::new(w, 0.0, z),
        },
        Landmark {
            id: "c11",
            prompt: "3/8 far-right corner (W,L)",
            world: Point3::new(w, l, z),
        },
        Landmark {
            id: "c01",
            prompt: "4/8 far-left corner (0,L)",
            world: Point3::new(0.0, l, z),
        },
        Landmark {
            id: "center",
            prompt: "5/8 table center (W/2,L/2)",
            world: Point3::new(w * 0.5, l * 0.5, z),
        },
        Landmark {
            id: "inner_robot",
            prompt: "6/8 inner robot-half (W/2,L/4)",
            world: Point3::new(w * 0.5, l * 0.25, z),
        },
        Landmark {
            id: "inner_far",
            prompt: "7/8 inner far-half (W/2,3L/4)",
            world: Point3::new(w * 0.5, l * 0.75, z),
        },
        Landmark {
            id: "mid_robot",
            prompt: "8/8 robot-side mid-edge (W/2,0)",
            world: Point3::new(w * 0.5, 0.0, z),
        },
    ];
}

/// 화면에 그릴 메시 선분 (랜드마크 인덱스 쌍).
/// 양 끝점이 모두 준비됐을 때만 그린다 (클릭·재투영 공통).
///
/// 8점 모두 연결:
/// - 0..=3 둘레
/// - 4(center)↔4꼭짓점 스포크
/// - 중앙선 7–5–4–6
/// - 로봇변 7↔0,1 / 원쪽 6↔2,3
pub fn table_landmark_mesh_edges() -> &'static [(usize, usize)] {
    return &[
        // perimeter
        (0, 1),
        (1, 2),
        (2, 3),
        (3, 0),
        // center spokes to corners
        (4, 0),
        (4, 1),
        (4, 2),
        (4, 3),
        // centerline: mid_robot - inner_robot - center - inner_far
        (7, 5),
        (5, 4),
        (4, 6),
        // robot mid-edge to near corners
        (7, 0),
        (7, 1),
        // far inner to far corners (대칭)
        (6, 2),
        (6, 3),
    ];
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn eight_landmarks_on_table_surface() {
        let marks = table_landmarks();
        assert_eq!(marks.len(), TABLE_LANDMARK_COUNT);
        for m in &marks {
            assert!((m.world.z - table::SURFACE_Z).abs() < 1e-12);
        }
        assert_eq!(marks[0].world, Point3::new(0.0, 0.0, table::SURFACE_Z));
        assert_eq!(
            marks[4].world,
            Point3::new(
                table::WIDTH_X * 0.5,
                table::LENGTH_Y * 0.5,
                table::SURFACE_Z
            )
        );
        assert_eq!(
            marks[5].world,
            Point3::new(
                table::WIDTH_X * 0.5,
                table::LENGTH_Y * 0.25,
                table::SURFACE_Z
            )
        );
        assert_eq!(
            marks[6].world,
            Point3::new(
                table::WIDTH_X * 0.5,
                table::LENGTH_Y * 0.75,
                table::SURFACE_Z
            )
        );
        assert_eq!(
            marks[7].world,
            Point3::new(table::WIDTH_X * 0.5, 0.0, table::SURFACE_Z)
        );
    }
}
