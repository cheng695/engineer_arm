#pragma once

#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "gravity/gravity_solver.hpp"

namespace my_robot_controllers
{

/**
 * @brief Outputs only the gravity feed-forward torque for the configured joints.
 *
 * This controller claims joint effort command interfaces and joint position
 * state interfaces. It does not write position or velocity commands, so it
 * can be activated alongside exactly one position-based motion controller.
 */
class GravityCompensationController
    : public controller_interface::ControllerInterface
{
public:
  GravityCompensationController() = default;
  ~GravityCompensationController() override = default;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::return_type update(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;

  controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

private:
  std::vector<std::string> joint_names_;
  std::string command_interface_name_{"effort"};
  std::string robot_description_;
  double effort_scale_{1.0};
  double max_effort_{0.0};
  std::vector<double> positions_;

  GravitySolver gravity_solver_;
};

}  // namespace my_robot_controllers
