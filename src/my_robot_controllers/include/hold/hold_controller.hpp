#pragma once

#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"

namespace my_robot_controllers
{

class HoldController : public controller_interface::ControllerInterface
{
public:
    HoldController() = default;
    ~HoldController() override = default;

    controller_interface::CallbackReturn on_init() override;
    controller_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State& previous_state) override;
    controller_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State& previous_state) override;
    controller_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State& previous_state) override;
    controller_interface::return_type update(
        const rclcpp::Time& time, const rclcpp::Duration& period) override;

    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

private:
    std::vector<std::string> joint_names_;
    std::string command_interface_name_{"position"};
    std::vector<double> hold_positions_;
};

}  // namespace my_robot_controllers
