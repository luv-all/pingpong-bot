use crate::error::HwError;
use crate::robot;
use crate::robot::motion;

/// 하드웨어 한계·양자화까지 반영해 실제 장치에 적용된 직접 명령.
#[derive(Debug, Clone, PartialEq)]
pub struct AppliedRailRacketCommand {
    pub rail_m: f64,
    pub joints: robot::Joints,
    /// 레일이 비활성인 경우 `false`다.
    pub rail_sent: bool,
}

/// 로봇 팔과 리니어 레일 구동 인터페이스.
pub trait Hardware: Send {
    fn command(&mut self, trajectory: &motion::Trajectory) -> Result<(), HwError>;
    fn read_pose(&mut self) -> Result<robot::Pose, HwError>;

    /// 2단계 직접 제어 명령: 레일과 라켓 정지 자세의 전체 관절을 갱신한다.
    fn command_rail_and_racket(
        &mut self,
        _trajectory: &motion::Trajectory,
    ) -> Result<AppliedRailRacketCommand, HwError> {
        return Err(HwError::InvalidConfig {
            reason: "레일+라켓 자세 직접 추종을 지원하지 않는 하드웨어입니다".into(),
        });
    }

    fn is_busy(&mut self) -> bool {
        return false;
    }

    /// 실행 중인 궤적을 중단한다. 완료까지 기다리지 않는다.
    ///
    /// 진행 중인 전체축 궤적을 새 제어가 선점할 때 사용한다.
    fn cancel(&mut self) {}
}
