use crate::error::HwError;
use crate::robot;
use crate::robot::motion;

/// 로봇 팔과 리니어 레일 구동 인터페이스.
pub trait Hardware: Send {
    fn command(&mut self, trajectory: &motion::Trajectory) -> Result<(), HwError>;
    fn read_pose(&mut self) -> Result<robot::Pose, HwError>;

    /// 2단계 제어 시험 명령: 레일과 라켓을 잡은 마지막 관절만 갱신한다.
    ///
    /// 기본 궤적 명령과 분리해, 중앙 정렬이 끝난 뒤 다른 Dynamixel 축에 Goal을
    /// 다시 보내지 않는다는 실기 계약을 명시한다.
    fn command_rail_and_racket(
        &mut self,
        _rail_x: f64,
        _racket_joint_rad: f64,
        _duration_secs: f64,
    ) -> Result<(), HwError> {
        return Err(HwError::InvalidConfig {
            reason: "레일+라켓 단순 추종을 지원하지 않는 하드웨어입니다".into(),
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
