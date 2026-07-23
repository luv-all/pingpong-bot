//! 다중 뷰 삼각측량 — OpenCV `triangulatePoints` + DLT 폴백.

mod opencv_tri;
mod triangulate;

pub use triangulate::{
    dlt_triangulate, sample_at, triangulate_projections, triangulate_synced, triangulate_views,
};
