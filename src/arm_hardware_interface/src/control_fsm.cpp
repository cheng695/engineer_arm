#include "arm_hardware_interface/control_fsm.hpp"

namespace arm_hardware_interface
{

const char* ControlFsm::state_name() const
{
  switch (state_)
  {
    case State::STOP:  return "STOP";
    case State::IDLE:  return "IDLE";
    case State::DLS:   return "DLS";
    case State::JOINT: return "JOINT";
    case State::POSE:  return "POSE";
    default:           return "?";
  }
}

void ControlFsm::setState(State s)
{
  just_entered_dls_   = (s == State::DLS   && state_ != State::DLS);
  just_entered_joint_ = (s == State::JOINT && state_ != State::JOINT);
  just_exited_stop_   = (s != State::STOP  && state_ == State::STOP);
  state_ = s;
}

bool ControlFsm::update(bool twist_active, bool joint_active)
{
  // 重置单帧标志
  just_entered_dls_   = false;
  just_entered_joint_ = false;
  just_exited_stop_   = false;

  // ---- 处理请求 ----
  switch (requested_)
  {
    case Request::ENABLE:
      requested_ = Request::NONE;
      if (state_ == State::STOP)
        setState(State::IDLE);
      break;

    case Request::DISABLE:
      requested_ = Request::NONE;
      setState(State::STOP);
      return false;  // STOP 不输出

    case Request::POSE_START:
      requested_ = Request::NONE;
      if (state_ != State::STOP)
        setState(State::POSE);
      break;

    case Request::POSE_DONE:
      requested_ = Request::NONE;
      if (state_ == State::POSE)
        setState(State::IDLE);
      break;

    default:
      break;
  }

  // STOP 状态不做任何事
  if (state_ == State::STOP)
    return false;

  // ---- 状态转换（基于活跃信号） ----
  switch (state_)
  {
    case State::IDLE:
      if (twist_active)
        setState(State::DLS);
      else if (joint_active)
        setState(State::JOINT);
      break;

    case State::DLS:
      if (joint_active)
      {
        setState(State::JOINT);
        twist_pending_ = false;
      }
      break;

    case State::JOINT:
      if (twist_active)
      {
        setState(State::DLS);
        joint_pending_ = false;
      }
      break;

    case State::POSE:
      // 保持在 POSE，直到 onPoseDone
      break;

    default:
      break;
  }

  // 返回是否需要输出控制（STOP 已返回 false，其他状态返回 true）
  return state_ != State::STOP;
}

}  // namespace arm_hardware_interface
