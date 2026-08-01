use crate::error::HwError;
use crate::robot;
use crate::robot::motion;

/// 로봇 팔과 리니어 구동 인터페이스. 위치 이동과 후속 타격 제어가 공유한다.
pub trait Hardware: Send {
    fn command(&mut self, trajectory: &motion::Trajectory) -> Result<(), HwError>;
    fn read_pose(&mut self) -> Result<robot::Pose, HwError>;

    /// 실기 시작 전 레일 중앙 정렬과 전체 관절 기본 자세 설정.
    /// 공 예측 제어와 분리해 초기화가 실제로 끝난 뒤에만 Ready가 되게 한다.
    fn command_initial_pose(
        &mut self,
        _rail_x: f64,
        _joints: &robot::Joints,
    ) -> Result<(), HwError> {
        return Err(HwError::InvalidConfig {
            reason: "초기 자세 직접 설정을 지원하지 않는 하드웨어입니다".into(),
        });
    }

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
    /// coarse 선추종처럼 **언제든 버려도 되는** 이동을 커밋 스윙이 선점하기 위한 것이다.
    /// 이게 없으면 `command`가 busy를 이유로 스윙을 조용히 무시하고 `Ok`를 돌려주며,
    /// 실측(클립 9개)에서 커밋이 7 → 3으로 떨어졌다.
    fn cancel(&mut self) {}
}
