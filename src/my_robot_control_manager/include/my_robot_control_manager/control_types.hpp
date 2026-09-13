#pragma once

#include <cstdint>

namespace my_robot_control_manager
{

// 控制器管理器对外暴露的逻辑状态。
enum class ArmControlState : std::uint8_t
{
  DISABLED = 0, // 停止
  HOLD,         // 保持
  CARTESIAN,    // 笛卡尔
  JOINT,        // 关节
  TRAJECTORY,   // 轨迹
  PAUSED,       // 暂停
  COUNT
};

enum class ArmControlEvent
{
  None,
  EnableRequested,          // 已收到电机使能请求
  DisableRequested,         // 已收到电机失能请求
  StopRequested,            // STOP：暂停，或从暂停恢复笛卡尔控制
  CartesianRequested,       // 摇杆开始笛卡尔运动
  JointRequested,           // 右摇杆按下，或开始关节运动
  JoystickReleased,         // 摇杆回到死区
  NamedTargetRequested,     // A/B/X/Y 请求固定位姿
  TrajectoryCompleted,      // 固定位姿/轨迹执行完成
  ControllerSwitchFailed    // 控制器切换失败，暂时按 STOP 处理
};

}  // namespace my_robot_control_manager
