#include "gravity/gravity_compensation_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "pluginlib/class_list_macros.hpp"

namespace my_robot_controllers
{

controller_interface::CallbackReturn GravityCompensationController::on_init()
{
  try
  {
    joint_names_ = auto_declare<std::vector<std::string>>("joints", {});
    command_interface_name_ = auto_declare<std::string>("command_interface", "effort");
    robot_description_ = auto_declare<std::string>("robot_description", "");
    effort_scale_ = auto_declare<double>("effort_scale", 1.0);
    max_effort_ = auto_declare<double>("max_effort", 0.0);
  }
  catch (const std::exception&)
  {
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GravityCompensationController::on_configure(
    const rclcpp_lifecycle::State&)
{
  get_node()->get_parameter("joints", joint_names_);
  get_node()->get_parameter("command_interface", command_interface_name_);
  get_node()->get_parameter("robot_description", robot_description_);
  get_node()->get_parameter("effort_scale", effort_scale_);
  get_node()->get_parameter("max_effort", max_effort_);

  if (joint_names_.empty() || command_interface_name_ != "effort")
  {
    RCLCPP_ERROR(
        get_node()->get_logger(),
        "GravityCompensationController 需要非空 joints 和 effort command interface");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (robot_description_.empty() || !std::isfinite(effort_scale_) ||
      !std::isfinite(max_effort_) || max_effort_ < 0.0)
  {
    RCLCPP_ERROR(
        get_node()->get_logger(),
        "GravityCompensationController 的 robot_description/effort 参数无效");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!gravity_solver_.initialize(robot_description_, get_node()->get_logger()) ||
      !gravity_solver_.bind_joints(joint_names_))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "重力补偿模型初始化或关节绑定失败");
    return controller_interface::CallbackReturn::ERROR;
  }
  positions_.assign(joint_names_.size(), 0.0);

  RCLCPP_INFO(
      get_node()->get_logger(),
      "GravityCompensationController 已配置：关节数=%zu, scale=%.3f, max_effort=%.3f",
      joint_names_.size(), effort_scale_, max_effort_);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GravityCompensationController::on_activate(
    const rclcpp_lifecycle::State&)
{
  if (command_interfaces_.size() != joint_names_.size() ||
      state_interfaces_.size() != joint_names_.size())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "重力控制器接口数量不匹配");
    return controller_interface::CallbackReturn::ERROR;
  }

  for (auto& command : command_interfaces_)
    command.set_value(0.0);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GravityCompensationController::on_deactivate(
    const rclcpp_lifecycle::State&)
{
  for (auto& command : command_interfaces_)
    command.set_value(0.0);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type GravityCompensationController::update(
    const rclcpp::Time&, const rclcpp::Duration&)
{
  if (!gravity_solver_.is_initialized() ||
      command_interfaces_.size() != joint_names_.size() ||
      state_interfaces_.size() != joint_names_.size())
  {
    return controller_interface::return_type::ERROR;
  }

  for (size_t i = 0; i < positions_.size(); ++i)
  {
    positions_[i] = state_interfaces_[i].get_value();
    if (!std::isfinite(positions_[i]))
      return controller_interface::return_type::ERROR;
  }

  const auto& gravity = gravity_solver_.compute(positions_);
  if (gravity.size() != command_interfaces_.size())
    return controller_interface::return_type::ERROR;

  for (size_t i = 0; i < gravity.size(); ++i)
  {
    double effort = effort_scale_ * gravity[i];
    if (!std::isfinite(effort))
      return controller_interface::return_type::ERROR;
    if (max_effort_ > 0.0)
      effort = std::clamp(effort, -max_effort_, max_effort_);
    command_interfaces_[i].set_value(effort);
  }

  return controller_interface::return_type::OK;
}

controller_interface::InterfaceConfiguration
GravityCompensationController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& joint : joint_names_)
    config.names.push_back(joint + "/" + command_interface_name_);
  return config;
}

controller_interface::InterfaceConfiguration
GravityCompensationController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& joint : joint_names_)
    config.names.push_back(joint + "/position");
  return config;
}

}  // namespace my_robot_controllers

PLUGINLIB_EXPORT_CLASS(
    my_robot_controllers::GravityCompensationController,
    controller_interface::ControllerInterface)
