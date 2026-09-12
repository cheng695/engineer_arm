#pragma once

#include <cstdint>

namespace my_robot_control_manager
{

// 控制器管理器对外暴露的逻辑状态。
enum class ArmControlState : std::uint8_t
{
  DISABLED = 0, // 停止
  HOLD,         // 保持
  CARTESIAN,    // 笛卡尔控制
  JOINT,        // 关节控制
  TRAJECTORY,   // 轨迹控制
  PAUSED,       // 暂停
  COUNT         
};

struct ArmControlStateInfo
{
  std::uint32_t enter_count{0};
  std::uint64_t run_ticks{0};
};

}  // namespace my_robot_control_manager
