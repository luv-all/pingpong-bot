use crate::error::HwError;
use crate::robot;
use crate::robot::motion;

/// 하드웨어 한계·양자화까지 반영해 실제 장치에 적용된 직접 명령.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AppliedRailRacketCommand {
    pub rail_m: f64,
    pub aim_rad: f64,
    /// 레일이 비활성인 경우 `false`다.
    pub rail_sent: bool,
}

/// 로봇 팔과 리니어 레일 구동 인터페이스.
pub trait Hardware: Send {
    fn command(&mut self, trajectory: &motion::Trajectory) -> Result<(), HwError>;

    /// 진행 중인 레일 직접 명령을 유지하며 관절 궤적만 실행한다.
    fn command_joints(&mut self, trajectory: &motion::Trajectory) -> Result<(), HwError> {
        return self.command(trajectory);
    }

    fn read_pose(&mut self) -> Result<robot::Pose, HwError>;

    /// 2단계 제어 명령: 레일과 라켓 수평 조준축만 갱신한다.
    ///
    /// 기본 궤적 명령과 분리해, 중앙 정렬이 끝난 뒤 다른 Dynamixel 축에 Goal을
    /// 다시 보내지 않는다는 실기 계약을 명시한다.
    fn command_rail_and_racket(
        &mut self,
        _rail_x: f64,
        _aim_joint_rad: f64,
        _duration_secs: f64,
    ) -> Result<AppliedRailRacketCommand, HwError> {
        return Err(HwError::InvalidConfig {
            reason: "레일+라켓 단순 추종을 지원하지 않는 하드웨어입니다".into(),
        });
    }

    fn is_busy(&mut self) -> bool {
        return false;
    }

    /// 실기 모터의 Goal/Present/Torque/Error 상태를 로그로 남긴다.
    /// 시뮬레이션 하드웨어는 할 일이 없다.
    fn log_joint_diagnostics(&mut self) {}

    /// 토크 차단·하드웨어 오류가 난 관절을 한 번 복구한다.
    /// 실제로 복구를 시도했으면 `true`, 대상이 없으면 `false`다.
    fn recover_joint_control(&mut self) -> Result<bool, HwError> {
        return Ok(false);
    }

    /// 실행 중인 궤적을 중단한다. 완료까지 기다리지 않는다.
    ///
    /// 진행 중인 전체축 궤적을 새 제어가 선점할 때 사용한다.
    fn cancel(&mut self) {}
}
