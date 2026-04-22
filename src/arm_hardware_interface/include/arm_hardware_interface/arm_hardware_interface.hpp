#ifndef ARM_HARDWARE_INTERFACE__ARM_HARDWARE_INTERFACE_HPP_
#define ARM_HARDWARE_INTERFACE__ARM_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <atomic>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/bool.hpp"

// Pinocchio includes
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

namespace arm_hardware_interface
{
enum class GravityCompensationMode
{
  Off = 0,
  Assist = 1,
  GravityOnly = 2,
};

class ArmHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_UNIQUE_PTR_DEFINITIONS(ArmHardwareInterface)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::vector<double> hw_commands_;
  std::vector<double> hw_commands_vel_;
  std::vector<double> hw_commands_eff_;
  std::vector<double> hw_states_;
  std::vector<double> hw_states_vel_;
  std::vector<double> hw_states_eff_;
  std::vector<bool> use_real_joint_io_;

  // Motor power control
  std::atomic<bool> motors_enabled_{false};
  std::atomic<bool> enable_requested_{false};
  std::atomic<bool> disable_requested_{false};
  std::atomic<int> safe_zero_frames_after_enable_{0};
  std::atomic<bool> velocity_commands_armed_{false};
  rclcpp::Node::SharedPtr internal_node_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;

  // Pinocchio Model
  std::unique_ptr<pinocchio::Model> model_;
  std::unique_ptr<pinocchio::Data> data_;
  std::vector<double> gravity_compensation_eff_;
  bool pinocchio_initialized_{false};
  GravityCompensationMode gravity_mode_{GravityCompensationMode::Off};
};

}  // namespace arm_hardware_interface

#endif  // ARM_HARDWARE_INTERFACE__ARM_HARDWARE_INTERFACE_HPP_
