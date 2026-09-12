#pragma once

#include "my_robot_control_manager/arm_control_event.hpp"

namespace my_robot_control_manager
{

// 只负责“条件 -> 事件”，不修改 FSM 状态，也不调用 ROS。
class ArmEventDetector
{
public:
  ArmControlEvent detect(const ArmInputSnapshot& input);
  void reset();

private:
  bool initialized_{false};
  ArmInputSnapshot previous_{};
};

}  // namespace my_robot_control_manager
