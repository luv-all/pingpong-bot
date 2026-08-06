//! CLI 바이너리용 tracing subscriber 초기화.
//!
//! Windows PowerShell 등에서 `RUST_LOG=… cargo …` 문법이 깨지므로
//! `--debug` 플래그만 쓴다.
//!
//! `real_mode`가 `true`면 `target: "latency"` 이벤트를 콘솔과 별개로
//! `logs/latency-<유닉스 초>.jsonl`에도 JSON Lines로 남긴다 — 실기 파이프라인
//! 구간별 소요 시간 진단용(`docs/superpowers/specs/2026-08-06-real-latency-instrumentation-design.md`).
//! 파일을 열지 못해도 콘솔 로그·실기 제어는 그대로 동작한다.

use std::fs::{self, File};
use std::sync::Arc;
use std::time::{SystemTime, UNIX_EPOCH};

use tracing_subscriber::EnvFilter;
use tracing_subscriber::Layer;
use tracing_subscriber::filter::Targets;
use tracing_subscriber::layer::SubscriberExt;
use tracing_subscriber::util::SubscriberInitExt;

/// tracing subscriber를 한 번 초기화한다.
///
/// - `debug == true` → `debug_crates`를 `=debug`로
/// - 아니면 기본 `info`
/// - `real_mode == true` → `target: "latency"` 이벤트를 파일 레이어로도 미러링
pub fn init_tracing(debug: bool, debug_crates: &[&str], real_mode: bool) {
    let filter = if debug {
        let directives = debug_crates
            .iter()
            .map(|name| format!("{name}=debug"))
            .collect::<Vec<_>>()
            .join(",");
        EnvFilter::new(if directives.is_empty() {
            "debug".to_owned()
        } else {
            directives
        })
    } else {
        EnvFilter::new("info")
    };
    let stdout_layer = tracing_subscriber::fmt::layer().with_filter(filter);

    let latency_layer = if real_mode {
        open_latency_file("logs").map(|file| {
            tracing_subscriber::fmt::layer()
                .json()
                .with_writer(file)
                .with_filter(Targets::new().with_target("latency", tracing::Level::INFO))
        })
    } else {
        None
    };

    tracing_subscriber::registry()
        .with(stdout_layer)
        .with(latency_layer)
        .init();
}

/// `<base_dir>/latency-<유닉스 초>.jsonl`을 새로 연다. 실패하면 `eprintln!`으로
/// 한 번만 경고하고 `None`을 돌려준다 — 이 시점엔 아직 tracing subscriber가 없어
/// 로그 매크로를 쓸 수 없고, 계측 실패가 실기 제어를 막아서도 안 된다.
fn open_latency_file(base_dir: &str) -> Option<Arc<File>> {
    if let Err(error) = fs::create_dir_all(base_dir) {
        eprintln!("경고: {base_dir} 디렉터리 생성 실패 — 레이턴시 파일 로그 없이 계속: {error}");
        return None;
    }
    let unix_secs = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0, |elapsed| elapsed.as_secs());
    let path = format!("{base_dir}/latency-{unix_secs}.jsonl");
    return match File::create(&path) {
        Ok(file) => {
            println!("레이턴시 진단 로그: {path}");
            Some(Arc::new(file))
        }
        Err(error) => {
            eprintln!("경고: 레이턴시 로그 파일 생성 실패({path}) — 파일 로그 없이 계속: {error}");
            None
        }
    };
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn open_latency_file_creates_directory_and_unique_jsonl() {
        let base =
            std::env::temp_dir().join(format!("pingpong_latency_test_{}", std::process::id()));
        let base_str = base.to_str().expect("temp 경로는 유효한 UTF-8").to_owned();
        let _ = std::fs::remove_dir_all(&base);

        let file = open_latency_file(&base_str);
        assert!(file.is_some(), "파일을 열 수 있어야 한다");
        drop(file);

        let entries: Vec<_> = std::fs::read_dir(&base)
            .expect("디렉터리가 생성돼 있어야 한다")
            .filter_map(|entry| entry.ok())
            .collect();
        assert_eq!(entries.len(), 1, "파일이 정확히 하나 생성돼야 한다");
        let name = entries[0].file_name();
        let name = name.to_string_lossy();
        assert!(
            name.starts_with("latency-") && name.ends_with(".jsonl"),
            "예상치 못한 파일명: {name}"
        );

        std::fs::remove_dir_all(&base).expect("정리");
    }
}
