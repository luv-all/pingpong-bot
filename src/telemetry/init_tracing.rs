//! CLI 바이너리용 tracing subscriber 초기화.
//!
//! Windows PowerShell 등에서 `RUST_LOG=… cargo …` 문법이 깨지므로
//! `--debug` 플래그만 쓴다.

use tracing_subscriber::EnvFilter;

/// tracing subscriber를 한 번 초기화한다.
///
/// - `debug == true` → `debug_crates`를 `=debug`로
/// - 아니면 기본 `info`
pub fn init_tracing(debug: bool, debug_crates: &[&str]) {
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
    tracing_subscriber::fmt().with_env_filter(filter).init();
}
