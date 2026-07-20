#pragma once

#include <Eigen/Core>
#include <vector>

namespace arm_hardware_interface
{

/**
 * @brief 关节空间控制器（角度控制）
 *
 *   vel_target (手柄关节速度指令)
 *     │
 *     ▼
 *   加速度限制 → 关节限位预测 → 整体速度缩放
 *     │
 *     ▼
 *   integrate(qd_tar·dt)  ──  q_des (目标关节角度)
 *
 * 和 DLS 笛卡尔控制器共用相同的安全策略。
 */
class JointController
{
public:
  struct Output
  {
    double pos;   // 目标关节位置 (rad)
    double vel;   // 目标关节速度 (rad/s)
    double eff;   // 力矩，恒为 0
  };

  struct Diagnostics
  {
    std::vector<double> target_positions;
    std::vector<double> target_velocities;
    std::vector<double> feedback_positions;
    std::vector<double> feedback_velocities;
    bool valid = false;
  };

  JointController() = default;

  /**
   * @brief 初始化
   * @param n_joints 关节数
   * @param dt       控制周期 (秒)
   */
  void Init(size_t n_joints, double dt = 0.002);

  /**
   * @brief 单步关节控制
   * @param vel_target 目标关节速度 (rad/s)，长度 n_joints
   * @param pos_ref    当前关节位置反馈 (rad)
   * @param joint_lower 关节下限 (rad)，长度 n_joints
   * @param joint_upper 关节上限 (rad)，长度 n_joints
   * @return           各关节目标 {pos, vel, eff}
   */
  /**
   * @brief 将目标位置同步到指定位置（非关节模式下调用，防切回时跳变）
   * @param pos 当前关节位置 (rad)
   */
  void SyncPositions(const std::vector<double>& pos);

  std::vector<Output> Update(const std::vector<double>& vel_target,
                              const std::vector<double>& pos_ref,
                              const std::vector<double>& joint_lower,
                              const std::vector<double>& joint_upper);

  const Diagnostics& LastDiagnostics() const { return last_diagnostics_; }

private:
  size_t n_joints_ = 0;
  double dt_ = 0.002;

  Eigen::VectorXd vel_last_;   // 上一帧速度，用于加速度限制
  Eigen::VectorXd pos_tar_;    // 积分得到的目标位置
  Diagnostics last_diagnostics_;
};

}  // namespace arm_hardware_interface
