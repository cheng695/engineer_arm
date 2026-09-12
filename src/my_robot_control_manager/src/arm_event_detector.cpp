#include "my_robot_control_manager/arm_event_detector.hpp"

namespace my_robot_control_manager
{

ArmControlEvent ArmEventDetector::detect(const ArmInputSnapshot& input)
{
  if (!initialized_) {
    initialized_ = true;
    previous_ = input;
    if (input.hardware_fault) {
      return ArmControlEvent::HardwareFault;
    }
    return input.motors_enabled
      ? ArmControlEvent::EnableRequested
      : ArmControlEvent::DisableRequested;
  }

  if (input.hardware_fault && !previous_.hardware_fault) {
    previous_ = input;
    return ArmControlEvent::HardwareFault;
  }

  if (!input.motors_enabled && previous_.motors_enabled) {
    previous_ = input;
    return ArmControlEvent::DisableRequested;
  }

  if (input.motors_enabled && !previous_.motors_enabled) {
    previous_ = input;
    return ArmControlEvent::EnableRequested;
  }

  if (input.motors_enabled && input.command_mode != previous_.command_mode) {
    previous_ = input;
    switch (input.command_mode) {
      case ArmControlState::CARTESIAN:
        return ArmControlEvent::CartesianRequested;
      case ArmControlState::JOINT:
        return ArmControlEvent::JointRequested;
      case ArmControlState::TRAJECTORY:
        return ArmControlEvent::NamedTargetRequested;
      default:
        break;
    }
  }

  previous_ = input;
  return ArmControlEvent::None;
}

void ArmEventDetector::reset()
{
  initialized_ = false;
  previous_ = ArmInputSnapshot{};
}

}  // namespace my_robot_control_manager
