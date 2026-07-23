//! 탁구대 규격 랜드마크 (solvePnP 외참용 SSOT).
//!
//! 원점 = 로봇 쪽 꼭짓점(바닥 아님, **테이블 면** `SURFACE_Z`).  
//! +X 너비, +Y 길이, +Z up — [`crate::constants::table`].

use crate::Point3;
use crate::constants::table;

/// 권장 랜드마크 개수 (4 corners + center + robot-side mid).
pub const TABLE_LANDMARK_COUNT: usize = 6;

/// 재투영 RMSE 합격 상한 [px]. 플랜: ≤ 2~3.
pub const MAX_REPROJ_RMSE_PX: f64 = 3.0;

/// 고정 월드 랜드마크 하나.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TableLandmark {
    /// 짧은 영문 id (로그·UI)
    pub id: &'static str,
    /// 클릭 안내 문구 (ASCII — Hershey)
    pub prompt: &'static str,
    /// 월드 좌표 [m]
    pub world: Point3,
}

/// 팀 규약 6점 (순서 고정 — 클릭도 이 순서).
///
/// 1–4 꼭짓점, 5 중앙, 6 로봇쪽 변 중점.
pub fn table_landmarks() -> [TableLandmark; TABLE_LANDMARK_COUNT] {
    let z = table::SURFACE_Z;
    let w = table::WIDTH_X;
    let l = table::LENGTH_Y;
    return [
        TableLandmark {
            id: "c00",
            prompt: "1/6 robot-left corner (0,0)",
            world: Point3::new(0.0, 0.0, z),
        },
        TableLandmark {
            id: "c10",
            prompt: "2/6 robot-right corner (W,0)",
            world: Point3::new(w, 0.0, z),
        },
        TableLandmark {
            id: "c11",
            prompt: "3/6 far-right corner (W,L)",
            world: Point3::new(w, l, z),
        },
        TableLandmark {
            id: "c01",
            prompt: "4/6 far-left corner (0,L)",
            world: Point3::new(0.0, l, z),
        },
        TableLandmark {
            id: "center",
            prompt: "5/6 table center (W/2,L/2)",
            world: Point3::new(w * 0.5, l * 0.5, z),
        },
        TableLandmark {
            id: "mid_robot",
            prompt: "6/6 robot-side mid-edge (W/2,0)",
            world: Point3::new(w * 0.5, 0.0, z),
        },
    ];
}

/// 화면에 그릴 메시 선분 (랜드마크 인덱스 쌍).
/// 양 끝점이 모두 클릭됐을 때만 그린다.
///
/// - 0..=3: 탁구대 둘레 사각형
/// - 4(center)↔꼭짓점: 대각 메시
/// - 5(mid_robot)↔로봇쪽 두 꼭짓점·중앙
pub fn table_landmark_mesh_edges() -> &'static [(usize, usize)] {
    return &[
        // perimeter
        (0, 1),
        (1, 2),
        (2, 3),
        (3, 0),
        // center spokes
        (4, 0),
        (4, 1),
        (4, 2),
        (4, 3),
        // robot mid-edge
        (5, 0),
        (5, 1),
        (5, 4),
    ];
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn six_landmarks_on_table_surface() {
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
            Point3::new(table::WIDTH_X * 0.5, 0.0, table::SURFACE_Z)
        );
    }
}
