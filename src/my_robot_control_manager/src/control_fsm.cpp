#include "my_robot_control_manager/control_fsm.hpp"

namespace my_robot_control_manager
{

void ControlFsm::init()
{
  state_ = ArmControlState::DISABLED;
}

/**
 * @brief 事件切换处理
 * 
 * @param event 事件 
 * @return FsmTransition 
 */
FsmTransition ControlFsm::dispatch(ArmControlEvent event)
{
  switch (event) 
  {
    case ArmControlEvent::EnableRequested:
      return transitionTo(ArmControlState::HOLD);

    case ArmControlEvent::DisableRequested:
      return transitionTo(ArmControlState::DISABLED);

    case ArmControlEvent::StopRequested:
      if (state_ == ArmControlState::DISABLED) 
      {
        return FsmTransition{false, false, state_, state_};
      }
      return state_ == ArmControlState::PAUSED
        ? transitionTo(ArmControlState::CARTESIAN)
        : transitionTo(ArmControlState::PAUSED);

    case ArmControlEvent::ControllerSwitchFailed:
      return transitionTo(ArmControlState::DISABLED);

    case ArmControlEvent::CartesianRequested:
      return canAcceptCommand() ? transitionTo(ArmControlState::CARTESIAN)
                                 : FsmTransition{};
    case ArmControlEvent::JointRequested:
      return canAcceptCommand() ? transitionTo(ArmControlState::JOINT)
                                 : FsmTransition{};
    case ArmControlEvent::NamedTargetRequested:
      return canAcceptCommand() ? transitionTo(ArmControlState::TRAJECTORY)
                                 : FsmTransition{};
    case ArmControlEvent::JoystickReleased:
      return (state_ == ArmControlState::CARTESIAN ||
              state_ == ArmControlState::JOINT)
        ? transitionTo(ArmControlState::HOLD)
        : FsmTransition{false, false, state_, state_};

    case ArmControlEvent::TrajectoryCompleted:
      return state_ == ArmControlState::TRAJECTORY
        ? transitionTo(ArmControlState::HOLD)
        : FsmTransition{false, false, state_, state_};

    case ArmControlEvent::None:

    default:
      break;
  }
  return FsmTransition{false, false, state_, state_};
}

FsmTransition ControlFsm::forceStop()
{
  return transitionTo(ArmControlState::DISABLED);
}

const char* ControlFsm::stateName(ArmControlState state)
{
  switch (state) 
  {
    case ArmControlState::DISABLED:   return "DISABLED";
    case ArmControlState::HOLD:       return "HOLD";
    case ArmControlState::CARTESIAN:  return "CARTESIAN";
    case ArmControlState::JOINT:      return "JOINT";
    case ArmControlState::TRAJECTORY: return "TRAJECTORY";
    case ArmControlState::PAUSED:     return "PAUSED";
    case ArmControlState::COUNT: break;
  }
  return "UNKNOWN";
}

/**
 * @brief 状态切换
 * 执行正常的状态切换
 * @param next 
 * @return FsmTransition 
 */
FsmTransition ControlFsm::transitionTo(ArmControlState next)
{
  const auto previous = state_;
  if (previous == next) 
  {
    return FsmTransition{true, false, previous, state_};
  }

  state_ = next;
  return FsmTransition{true, true, previous, state_};
}

/**
 * @brief 判断是否允许给运动期望
 * 
 * @return true 
 * @return false 
 */
bool ControlFsm::canAcceptCommand() const
{
  return state_ != ArmControlState::DISABLED && state_ != ArmControlState::PAUSED;
}

}  // namespace my_robot_control_manager
