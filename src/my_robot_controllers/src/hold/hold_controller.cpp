#include "hold/hold_controller.hpp"

#include <cmath>
#include <stdexcept>

#include "pluginlib/class_list_macros.hpp"

namespace my_robot_controllers
{

controller_interface::CallbackReturn HoldController::on_init()
{
    try
    {
        joint_names_ = auto_declare<std::vector<std::string>>("joints", {});
        command_interface_name_ = auto_declare<std::string>("command_interface", "position");
    }
    catch (const std::exception&)
    {
        return controller_interface::CallbackReturn::ERROR;
    }
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn HoldController::on_configure(
    const rclcpp_lifecycle::State&)
{
    get_node()->get_parameter("joints", joint_names_);
    get_node()->get_parameter("command_interface", command_interface_name_);

    if (joint_names_.empty() || command_interface_name_ != "position")
    {
        RCLCPP_ERROR(
            get_node()->get_logger(),
            "HoldController 需要非空 joints 和 position command interface");
        return controller_interface::CallbackReturn::ERROR;
    }

    hold_positions_.assign(joint_names_.size(), 0.0);
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn HoldController::on_activate(
    const rclcpp_lifecycle::State&)
{
    if (command_interfaces_.size() < joint_names_.size() ||
        state_interfaces_.size() < 2 * joint_names_.size())
    {
        RCLCPP_ERROR(get_node()->get_logger(), "HoldController 激活时关节接口数量不足");
        return controller_interface::CallbackReturn::ERROR;
    }

    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
        // 优先锁定切换前控制器留下的最后位置期望，避免反馈误差造成目标跳变。
        const double command_position = command_interfaces_[i].get_value();
        hold_positions_[i] = std::isfinite(command_position)
            ? command_position
            : state_interfaces_[2 * i].get_value();
        if (!std::isfinite(hold_positions_[i])) return controller_interface::CallbackReturn::ERROR;
    }
    for (size_t i = 0; i < joint_names_.size(); ++i)
        command_interfaces_[i].set_value(hold_positions_[i]);

    RCLCPP_INFO(get_node()->get_logger(), "HoldController 已激活，保持最后的关节期望角度");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn HoldController::on_deactivate(
    const rclcpp_lifecycle::State&)
{
    for (size_t i = 0; i < hold_positions_.size() && i < command_interfaces_.size(); ++i)
    {
        command_interfaces_[i].set_value(hold_positions_[i]);
    }
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type HoldController::update(
    const rclcpp::Time&, const rclcpp::Duration&)
{
    if (command_interfaces_.size() < hold_positions_.size())
    {
        return controller_interface::return_type::ERROR;
    }

    // Hold 状态不再更新目标，只重复发送锁定的期望角度。
    for (size_t i = 0; i < hold_positions_.size(); ++i)
    {
        command_interfaces_[i].set_value(hold_positions_[i]);
    }
    return controller_interface::return_type::OK;
}

controller_interface::InterfaceConfiguration
HoldController::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto& joint : joint_names_)
    {
        config.names.push_back(joint + "/" + command_interface_name_);
    }
    return config;
}

controller_interface::InterfaceConfiguration
HoldController::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto& joint : joint_names_)
    {
        config.names.push_back(joint + "/position");
        config.names.push_back(joint + "/velocity");
    }
    return config;
}

}  // namespace my_robot_controllers

PLUGINLIB_EXPORT_CLASS(
    my_robot_controllers::HoldController,
    controller_interface::ControllerInterface)
