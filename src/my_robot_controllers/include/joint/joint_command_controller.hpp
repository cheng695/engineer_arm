#pragma once
#include "joint_diagnostics.hpp"
#include "realtime_tools/realtime_buffer.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "joint/joint_command_solver.hpp"
#include "pinocchio/multibody/model.hpp"
#include "rclcpp/subscription.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace my_robot_controllers
{

class JointCommandController : public controller_interface::ControllerInterface
{
public:
    JointCommandController() = default;
    ~JointCommandController() override = default;

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
  JointDiagnostics diagnostics_;
  realtime_tools::RealtimeBuffer<std::vector<double>> command_buffer_;
    void commandCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

    std::vector<std::string> joint_names_;
    std::string command_interface_name_{"position"};
    std::string command_topic_{"~/joint_velocity_cmd"};
    std::string robot_description_;

    std::unique_ptr<pinocchio::Model> model_;
    JointCommandSolver solver_;
    std::vector<double> velocity_command_;
    std::vector<double> positions_;
    std::vector<double> lower_limits_;
    std::vector<double> upper_limits_;
    bool target_initialized_{false};
    double command_timeout_{0.1};
    std::atomic<bool> command_received_{false};
    std::atomic<std::int64_t> last_command_time_ns_{0};
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr command_sub_;
};

}  // namespace my_robot_controllers
