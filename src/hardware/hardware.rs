use crate::error::HwError;
use crate::robot;
use crate::robot::motion;

/// 로봇 팔과 리니어 구동 인터페이스. 위치 이동과 후속 타격 제어가 공유한다.
pub trait Hardware: Send {
    fn command(&mut self, trajectory: &motion::Trajectory) -> Result<(), HwError>;
    fn read_pose(&mut self) -> Result<robot::Pose, HwError>;
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
