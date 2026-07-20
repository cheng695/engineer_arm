#pragma once

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <Eigen/Core>
#include <array>
#include <memory>
#include <string>
#include <vector>

namespace arm_hardware_interface
{

/**
 * @brief 位置累积型 DLS 控制器
 *
 *   target (末端速度指令 [vx,vy,vz, wx,wy,wz], LOCAL 坐标系)
 *     │
 *     ▼
 *   J⁺_DLS · target  ────  q̇_des (关节速度)
 *     │
 *     ▼
 *   pos_tar += q̇_des·dt  ──  q_des (跨帧累积位置)
 *
 * 特性：
 * - 自适应 λ(σ_min)：远离奇异 λ≈0 灵活，靠近奇异 λ↑ 保护
 * - 方向保险：cos(achieved, desired) < 0 时停止
 * - 加速度限制：相邻帧速度变化 ≤ a_max·dt
 * - 关节限位预测：接近限位时提前禁止往里推
 * - 整体速度缩放：保持末速度方向，等比缩放关节速度
 */
class DlsController
{
public:
  struct Output
  {
    double pos;   // 目标关节位置 (rad)
    double vel;   // 目标关节速度 (rad/s)
    double tor;   // 目标关节力矩 (Nm)，速度 DLS 下恒为 0
  };

  struct Diagnostics
  {
    std::vector<double> target_positions;
    std::vector<double> target_velocities;
    std::vector<double> feedback_positions;
    std::vector<double> feedback_velocities;
    bool valid = false;
  };

  DlsController() = default;

  /**
   * @brief 初始化
   * @param model               Pinocchio 模型
   * @param terminal_frame_name 末端 frame 名称 (如 "tool_link")
   * @param dt                  控制周期 (秒)
   */
  void Init(const pinocchio::Model& model,
            const std::string& terminal_frame_name,
            double dt = 0.002);

  /**
   * @brief 单步 DLS 控制
   * @param target  末端目标速度 (LOCAL 坐标系) [vx,vy,vz, wx,wy,wz]
   * @param pos_ref 当前关节位置 (rad)
   * @param vel_ref 当前关节速度 (rad/s)
   * @return        各关节目标 {pos, vel, tor}
   */
  void SyncPositions(const std::vector<double>& pos);

  std::vector<Output> Update(const std::array<double, 6>& target,
                              const std::vector<double>& pos_ref,
                              const std::vector<double>& vel_ref);

  const Diagnostics& LastDiagnostics() const { return last_diagnostics_; }

private:
  const pinocchio::Model* model_ = nullptr;   // Pinocchio 模型（共享，不拷贝）
  std::unique_ptr<pinocchio::Data> data_;     // Pinocchio 运行时数据
  size_t terminal_frame_name_ = 0;            // 末端 frame 索引
  size_t n_joints_ = 0;                       // 关节数
  int vel_offset_ = 0;                        // 虚拟基座速度偏移量
  double dt_ = 0.002;                         // 控制周期

  Eigen::VectorXd vel_last_;                  // 上一帧速度，用于加速度限制
  Eigen::VectorXd pos_tar_;                   // 跨帧累积目标位置
  bool pos_tar_initialized_ = false;
  Diagnostics last_diagnostics_;
};

}  // namespace arm_hardware_interface
