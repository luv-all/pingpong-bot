//! [`super`] 단위 테스트.

use super::*;

/// 채널마다 범위가 겹치지만 `g - b` 방향으로는 완전히 갈리는 두 무리.
fn diagonal_split() -> (Vec<[u8; 3]>, Vec<[u8; 3]>) {
    let positive = vec![[100, 150, 0], [150, 200, 0], [200, 250, 0]];
    let negative = vec![[150, 100, 0], [200, 150, 0], [250, 200, 0]];
    return (positive, negative);
}

#[test]
fn separates_what_an_axis_aligned_box_cannot() {
    let (positive, negative) = diagonal_split();

    // 양성을 감싸는 최소 AABB 는 음성을 통과시킨다.
    let lo = |i: usize| positive.iter().map(|p| p[i]).min().unwrap();
    let hi = |i: usize| positive.iter().map(|p| p[i]).max().unwrap();
    let in_box = |p: &[u8; 3]| (0..3).all(|i| p[i] >= lo(i) && p[i] <= hi(i));
    let leaked = negative.iter().filter(|n| in_box(n)).count();
    assert!(leaked > 0, "이 픽스처는 상자가 새야 의미가 있다");

    // 판별면은 안 샌다.
    let gate = ColorPlane::fit(&positive, &negative, 1.0).expect("fit");
    assert!(
        positive.iter().all(|p| gate.keep(*p)),
        "양성을 다 남겨야 한다"
    );
    assert!(
        negative.iter().all(|n| !gate.keep(*n)),
        "음성을 다 걸러야 한다"
    );
}

/// 재현율을 낮추면 임계가 올라가 양성 일부를 버린다.
#[test]
fn keep_ratio_trades_recall_for_tightness() {
    let (positive, negative) = diagonal_split();
    let loose = ColorPlane::fit(&positive, &negative, 1.0).expect("fit");
    let tight = ColorPlane::fit(&positive, &negative, 0.5).expect("fit");

    let kept = |g: &ColorPlane| positive.iter().filter(|p| g.keep(**p)).count();
    assert_eq!(kept(&loose), 3);
    assert!(kept(&tight) < 3, "0.5 면 절반 언저리만 남아야 한다");
}

#[test]
fn refuses_degenerate_input() {
    let one = vec![[100, 100, 100]];
    assert!(ColorPlane::fit(&one, &one, 1.0).is_err(), "표본 부족");

    let same = vec![[100, 100, 100], [100, 100, 100], [100, 100, 100]];
    assert!(
        ColorPlane::fit(&same, &same, 1.0).is_err(),
        "두 클래스가 같으면 판별 방향이 없다"
    );
}
