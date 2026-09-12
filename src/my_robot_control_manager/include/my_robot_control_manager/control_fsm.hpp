#pragma once

#include <array>
#include <cstdint>

#include "my_robot_control_manager/arm_control_event.hpp"
#include "my_robot_control_manager/arm_control_state.hpp"

namespace my_robot_control_manager
{

struct FsmTransition
{
  bool accepted{false};
  bool changed{false};
  ArmControlState previous{ArmControlState::DISABLED};
  ArmControlState current{ArmControlState::DISABLED};
};

// 纯状态机：只处理事件和状态，不依赖 ROS、controller_manager 或 CAN。
// 当前暂不单独维护 FAULT；故障类事件统一回到 DISABLED/STOP。
class ControlFsm
{
public:
  void init();
  FsmTransition dispatch(ArmControlEvent event);
  FsmTransition forceStop();
  void update();

  ArmControlState state() const { return state_; }
  const ArmControlStateInfo& info(ArmControlState state) const;
  static const char* stateName(ArmControlState state);

private:
  FsmTransition transitionTo(ArmControlState next);
  bool canAcceptCommand() const;

  ArmControlState state_{ArmControlState::DISABLED};
  std::array<ArmControlStateInfo,
             static_cast<std::size_t>(ArmControlState::COUNT)> statistics_{};
};

}  // namespace my_robot_control_manager
