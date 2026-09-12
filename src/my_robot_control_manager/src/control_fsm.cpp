#include "my_robot_control_manager/control_fsm.hpp"

namespace my_robot_control_manager
{

void ControlFsm::init()
{
  state_ = ArmControlState::DISABLED;
  statistics_ = {};
  statistics_[static_cast<std::size_t>(state_)].enter_count = 1;
}

FsmTransition ControlFsm::dispatch(ArmControlEvent event)
{
  switch (event) {
    case ArmControlEvent::EnableRequested:
      return transitionTo(ArmControlState::HOLD);
    case ArmControlEvent::DisableRequested:
    case ArmControlEvent::CommandTimeout:
    case ArmControlEvent::HardwareFault:
    case ArmControlEvent::ControllerSwitchFailed:
      return transitionTo(ArmControlState::DISABLED);
    case ArmControlEvent::PauseRequested:
      return transitionTo(ArmControlState::PAUSED);
    case ArmControlEvent::CartesianRequested:
      return canAcceptCommand() ? transitionTo(ArmControlState::CARTESIAN)
                                 : FsmTransition{};
    case ArmControlEvent::JointRequested:
      return canAcceptCommand() ? transitionTo(ArmControlState::JOINT)
                                 : FsmTransition{};
    case ArmControlEvent::NamedTargetRequested:
      return canAcceptCommand() ? transitionTo(ArmControlState::TRAJECTORY)
                                 : FsmTransition{};
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

void ControlFsm::update()
{
  ++statistics_[static_cast<std::size_t>(state_)].run_ticks;
}

const ArmControlStateInfo& ControlFsm::info(ArmControlState state) const
{
  return statistics_[static_cast<std::size_t>(state)];
}

const char* ControlFsm::stateName(ArmControlState state)
{
  switch (state) {
    case ArmControlState::DISABLED: return "DISABLED";
    case ArmControlState::HOLD: return "HOLD";
    case ArmControlState::CARTESIAN: return "CARTESIAN";
    case ArmControlState::JOINT: return "JOINT";
    case ArmControlState::TRAJECTORY: return "TRAJECTORY";
    case ArmControlState::PAUSED: return "PAUSED";
    case ArmControlState::COUNT: break;
  }
  return "UNKNOWN";
}

FsmTransition ControlFsm::transitionTo(ArmControlState next)
{
  const auto previous = state_;
  if (previous == next) {
    return FsmTransition{true, false, previous, state_};
  }

  statistics_[static_cast<std::size_t>(previous)].run_ticks = 0;
  state_ = next;
  auto& next_info = statistics_[static_cast<std::size_t>(state_)];
  ++next_info.enter_count;
  return FsmTransition{true, true, previous, state_};
}

bool ControlFsm::canAcceptCommand() const
{
  return state_ != ArmControlState::DISABLED && state_ != ArmControlState::PAUSED;
}

}  // namespace my_robot_control_manager
