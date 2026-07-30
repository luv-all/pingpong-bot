//! 로그·HUD 숫자 포맷.
//!
//! tracing은 `f64`를 그대로 찍어 `0.5062136600022615` 같은 줄이 나온다. 벤치에서 읽을 값은
//! 소수점 2자리면 충분하다 — 로그에 싣는 실수는 전부 이걸 통과시킨다.

/// 소수점 2자리 문자열.
pub fn f2(value: f64) -> String {
    return format!("{value:.2}");
}

/// 슬라이스를 소수점 2자리로 (`[0.51, 0.00, -0.21, -0.69]`).
pub fn f2_slice(values: &[f64]) -> String {
    let body = values
        .iter()
        .map(|value| f2(*value))
        .collect::<Vec<_>>()
        .join(", ");
    return format!("[{body}]");
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rounds_to_two_places() {
        assert_eq!(f2(0.5062136600022615), "0.51");
        assert_eq!(f2(-0.6918253353364242), "-0.69");
        assert_eq!(f2(0.0), "0.00");
    }

    #[test]
    fn formats_joint_slices() {
        assert_eq!(f2_slice(&[0.5067, 0.0, -0.2054]), "[0.51, 0.00, -0.21]");
        assert_eq!(f2_slice(&[]), "[]");
    }
}
