//! 한 발 관측 플래그.

/// 한 발 관측 (shot_tune과 동일 계열).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Flags {
    pub contact: bool,
    pub cleared_net: bool,
    pub returned_in: bool,
    /// 리턴이 로봇 반쪽(`y < net_y`) 상면에 닿음 — 탁구 규칙상 반칙.
    ///
    /// 정상 리턴은 라켓에서 곧바로 상대 코트로 가야 한다. 자기 코트를
    /// 거쳐 넘어가는 건 약한 스윙의 대표 증상이고 실기에서는 실점이다.
    pub bounced_own_half: bool,
    /// 라켓이 같은 공을 두 번 침 — 반칙.
    pub double_hit: bool,
}

impl Flags {
    /// 0 미타격 · 1 접촉(또는 반칙) · 2 네트 통과 · 3 상대 코트 착지.
    ///
    /// 반칙은 접촉만 인정해 1점으로 강등한다 — 네트를 넘겼든 상대 코트에
    /// 들어갔든 무효다.
    pub fn score(self) -> u8 {
        if !self.contact {
            return 0;
        }
        if self.bounced_own_half || self.double_hit {
            return 1;
        }
        if self.returned_in {
            return 3;
        }
        if self.cleared_net {
            return 2;
        }
        return 1;
    }

    /// 반칙으로 강등됐는지 — 패널·로그에서 "3점 조건을 다 채웠는데 반칙"을
    /// 구분해 보여주기 위한 것.
    pub fn is_foul(self) -> bool {
        return self.bounced_own_half || self.double_hit;
    }
}
