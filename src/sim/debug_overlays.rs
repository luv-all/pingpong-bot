//! 시뮬 View 창 디버그 오버레이 토글.

/// View 창 체크박스로 켜고 끄는 3D/Status 오버레이.
#[derive(Clone, Debug, PartialEq)]
pub struct DebugOverlays {
    pub impact_markers: bool,
    pub fail_status: bool,
    pub unreachable_x: bool,
    pub joint_limits: bool,
    pub torque_hud: bool,
    pub table_obb: bool,
    pub net_gate: bool,
    pub predicted_arc: bool,
    pub truth_arc: bool,
    pub swing_ghost: bool,
    pub rail_stroke: bool,
    pub aim_band: bool,
    pub commit_bar: bool,
    pub omega_arrow: bool,
}

impl Default for DebugOverlays {
    fn default() -> Self {
        return Self::debug_defaults();
    }
}

impl DebugOverlays {
    /// 디버깅 이득이 큰 항목 on, 무거운 고스트류 off.
    pub fn debug_defaults() -> Self {
        return Self {
            impact_markers: true,
            fail_status: true,
            unreachable_x: true,
            joint_limits: true,
            torque_hud: true,
            table_obb: false,
            net_gate: false,
            predicted_arc: false,
            truth_arc: false,
            swing_ghost: false,
            rail_stroke: false,
            aim_band: false,
            commit_bar: true,
            omega_arrow: false,
        };
    }

    /// 전부 off.
    pub fn all_off() -> Self {
        return Self {
            impact_markers: false,
            fail_status: false,
            unreachable_x: false,
            joint_limits: false,
            torque_hud: false,
            table_obb: false,
            net_gate: false,
            predicted_arc: false,
            truth_arc: false,
            swing_ghost: false,
            rail_stroke: false,
            aim_band: false,
            commit_bar: false,
            omega_arrow: false,
        };
    }
}

/// 임팩트 마커 / Status용 RGBA (0..=1).
pub mod colors {
    /// 계획 성공 · committed
    pub const SUCCESS: [f32; 4] = [0.15, 0.85, 0.35, 0.95];
    /// InverseKinematicsNoSolution
    pub const IK: [f32; 4] = [0.95, 0.15, 0.12, 0.95];
    /// InsufficientTime
    pub const TIME: [f32; 4] = [1.0, 0.55, 0.1, 0.95];
    /// ReturnVelocityUnreachable
    pub const RETURN: [f32; 4] = [0.75, 0.25, 0.95, 0.95];
    /// TablePenetration
    pub const PENETRATION: [f32; 4] = [0.1, 0.85, 0.9, 0.95];
    /// JointOrTorqueLimit
    pub const LIMIT: [f32; 4] = [0.95, 0.9, 0.15, 0.95];
    /// idle + 예측만
    pub const IDLE_PRED: [f32; 4] = [1.0, 0.15, 0.95, 0.95];
    /// 도달 밖 X
    pub const UNREACHABLE_X: [f32; 4] = [1.0, 0.2, 0.15, 0.9];
    /// 예측 탄도
    pub const PRED_ARC: [f32; 4] = [0.4, 0.75, 1.0, 0.75];
    /// Rapier 진실 궤적
    pub const TRUTH_ARC: [f32; 4] = [1.0, 0.7, 0.2, 0.7];
    /// 스윙 고스트
    pub const GHOST: [f32; 4] = [0.9, 0.9, 0.95, 0.55];
    /// 리밋 관절
    pub const JOINT_LIMIT: [f32; 4] = [0.95, 0.12, 0.1, 1.0];
    /// OBB 침투
    pub const OBB_HIT: [f32; 4] = [1.0, 0.25, 0.2, 0.45];
    /// aim band
    pub const AIM_BAND: [f32; 4] = [0.2, 0.85, 0.95, 0.22];
    /// net-gate 실패 톤
    pub const NET_FAIL: [f32; 4] = [0.55, 0.55, 0.58, 0.7];
}
