#pragma once

#include "my_robot_control_manager/arm_control_state.hpp"

namespace my_robot_control_manager
{

enum class ArmControlEvent
{
  None,
  EnableRequested,  // 用户请求使能
  DisableRequested, // 用户请求失能
  PauseRequested,   // 用户请求暂停
  CartesianRequested,   // 用户请求笛卡尔空间控制
  JointRequested,       // 用户请求关节空间控制
  NamedTargetRequested, // 用户请求固定位姿控制
  CommandTimeout,           // 命令执行超时
  HardwareFault,            // 硬件故障
  ControllerSwitchFailed    // 控制器切换失败
};

// 这是事件检测器的输入，不是 ROS 消息。
// ROS 回调只负责把消息转换成这个快照。
struct ArmInputSnapshot
{
  bool motors_enabled{false};
  bool hardware_fault{false};
  ArmControlState command_mode{ArmControlState::HOLD};
};

}  // namespace my_robot_control_manager
