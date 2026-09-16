#pragma once

#include <string>
#include <memory>
#include <vector>

#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include "rclcpp/rclcpp.hpp"

namespace my_robot_controllers
{

/**
 * @brief 供控制器使用的重力补偿模型。
 *
 * 该类负责重力模型的生命周期管理，并向控制器提供重力力矩计算接口。
 * 这里不处理 command interface，也不包含限位、急停等安全策略；这些职责
 * 由上层控制器负责。
 */
class GravitySolver
{
public:
  GravitySolver() = default;
  ~GravitySolver() = default;

  GravitySolver(const GravitySolver&) = delete;
  GravitySolver& operator=(const GravitySolver&) = delete;

  /** @brief 根据完整的 robot_description XML 初始化重力模型。 */
  bool initialize(const std::string& robot_description, const rclcpp::Logger& logger);

  /** @brief 按控制器的关节顺序绑定求解器的输出顺序。 */
  bool bind_joints(const std::vector<std::string>& joint_names);

  /**
   * @brief 计算已绑定关节的静态重力力矩。
   *
   * 输入关节位置的单位为弧度，返回力矩的单位为 Nm。返回值由本求解器内部
   * 持有，在下一次调用 compute() 前保持有效。
   */
  const std::vector<double>& compute(const std::vector<double>& positions) const;

  bool is_initialized() const { return initialized_; }

private:
  std::unique_ptr<pinocchio::Model> model_;
  std::unique_ptr<pinocchio::Data> data_;
  std::vector<int> q_indices_;
  std::vector<int> v_indices_;
  mutable std::vector<double> effort_buffer_;
  mutable Eigen::VectorXd q_buffer_;
  mutable Eigen::VectorXd zero_buffer_;
  bool initialized_{false};
};

}  // namespace my_robot_controllers
