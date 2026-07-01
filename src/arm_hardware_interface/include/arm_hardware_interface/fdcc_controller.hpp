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
 * @brief 前向动力学柔顺控制器 (Forward Dynamics Compliance Controller)
 *
 * 用 J^T（雅可比转置）替代 J⁺（雅可比伪逆）实现笛卡尔空间力控制。
 * J^T 在奇异点附近永远良态，不会出现关节速度爆炸。
 *
 * 控制律：
 *   F_virtual = twist * force_scale       // 虚拟力 [N, Nm]
 *   τ_task    = J^T * F_virtual            // 任务关节力矩
 *   τ_total   = τ_task + g(q)              // + 重力补偿
 *   q̈         = M⁻¹(τ_total - C·q̇ - g)    // 前向动力学 (aba)
 *   q̇_new     = q̇ + q̈·dt                  // 半隐式 Euler
 *   q_new     = q  + q̇_new·dt
 *
 * twist 使用 LOCAL_WORLD_ALIGNED 坐标系（平移世界系，旋转体轴系）。
 */
class FdccController
{
public:
  /// 单个关节的 FDCC 输出
  struct Output
  {
    double pos;  // 目标位置 (rad)
    double vel;  // 目标速度 (rad/s)
    double eff;  // 目标力矩 (Nm)
  };

  FdccController() = default;

  /**
   * @brief 初始化
   * @param model         Pinocchio 模型引用
   * @param ee_frame     末端 frame 名称
   * @param force_scale_lin  线速度增益 (rad/s per m/s)
   * @param force_scale_ang  角速度增益 (rad/s per rad/s)
   * @param dt           控制周期 (秒)
   * @param locked_joint 锁定的关节索引（0-based，-1 = 不锁定）
   */
  void initialize(const pinocchio::Model& model,
                  const std::string& ee_frame,
                  double force_scale_lin = 50.0,
                  double force_scale_ang = 5.0,
                  double dt = 0.002,
                  int locked_joint = -1);

  /**
   * @brief 执行一次 FDCC 计算
   * @param twist 末端 twist [vx, vy, vz, wx, wy, wz] (m/s, rad/s)
   * @param q     当前关节位置 (rad)，长度 = n_joints
   * @param qd    当前关节速度 (rad/s)
   * @return       各关节目标 {pos, vel, eff}，长度 = n_joints
   */
  std::vector<Output> compute(const std::array<double, 6>& twist,
                              const std::vector<double>& q,
                              const std::vector<double>& qd);

  /// 是否已初始化
  bool is_initialized() const { return model_ != nullptr; }

  /// 获取关节数量
  size_t n_joints() const { return n_joints_; }

private:
  const pinocchio::Model* model_ = nullptr;
  std::unique_ptr<pinocchio::Data> data_;
  size_t ee_frame_id_ = 0;
  size_t n_joints_ = 0;          // 纯关节数 (nv - 虚拟基座自由度)
  int vel_offset_ = 0;           // 虚拟基座速度偏移
  double force_scale_lin_ = 50.0;
  double force_scale_ang_ = 5.0;
  double dt_ = 0.002;
  int locked_joint_ = -1;   // -1 = 不锁, 0~6 = 锁定 joint(1~7)

  // DLS 参数
  static constexpr double kDampingBase_  = 0.1;
  static constexpr double kDampingMin_   = 0.01;
  static constexpr double kDampingMax_   = 5.0;
  static constexpr double kMuThreshold_  = 0.1;
};

}  // namespace arm_hardware_interface
