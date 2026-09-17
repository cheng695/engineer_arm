#pragma once
#include "joint_diagnostics.hpp"
#include "realtime_tools/realtime_buffer.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include "dls/dls_solver.hpp"
#include <pinocchio/multibody/model.hpp>
#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/subscription.hpp"

namespace my_robot_controllers
{

// ros2_control 标准 DLS 笛卡尔控制器。
class DlsCartesianController : public controller_interface::ControllerInterface
{
public:
  DlsCartesianController() = default;
  ~DlsCartesianController() override = default;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time& time,
    const rclcpp::Duration& period) override;

  controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

private:
  JointDiagnostics diagnostics_;
  realtime_tools::RealtimeBuffer<geometry_msgs::msg::TwistStamped> command_buffer_;
  void commandCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  bool compute_tip_pose(
    const std::vector<double>& positions,
    Eigen::Vector3d& position,
    Eigen::Matrix3d& rotation);
  bool compute_tip_state(
    const std::vector<double>& positions,
    const std::vector<double>& velocities,
    Eigen::Vector3d& position,
    Eigen::Matrix3d& rotation,
    Eigen::Vector3d& linear_velocity_local,
    Eigen::Vector3d& angular_velocity_local);

  std::vector<std::string> joint_names_;
  std::string command_interface_name_{"position"};
  std::string command_topic_{"~/twist_cmd"};
  std::string robot_description_;
  std::string tip_link_{"tool_link"};
  double cartesian_position_kp_{1.5};
  double cartesian_position_kd_{0.1};
  double cartesian_orientation_kp_{1.5};
  double cartesian_orientation_kd_{0.1};
  double cartesian_linear_correction_limit_{0.05};
  double cartesian_angular_correction_limit_{0.3};
  double reference_following_error_slow_{0.05};
  double reference_following_error_stop_{0.15};
  double moving_correction_scale_{0.2};
  double holding_correction_scale_{1.0};
  double correction_scale_ramp_time_{0.3};
  double correction_scale_{0.2};

  geometry_msgs::msg::TwistStamped last_command_{};
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr command_sub_;
  std::vector<double> positions_;
  std::vector<double> velocities_;
  std::vector<double> target_positions_;
  std::vector<double> lower_limits_;
  std::vector<double> upper_limits_;
  bool target_initialized_{false};
  bool reference_pose_initialized_{false};
  Eigen::Vector3d reference_position_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond reference_orientation_{Eigen::Quaterniond::Identity()};
  double command_timeout_{0.1};
  std::atomic<bool> command_received_{false};
  std::atomic<std::int64_t> last_command_time_ns_{0};

  std::unique_ptr<pinocchio::Model> model_;
  std::unique_ptr<pinocchio::Data> diagnostics_data_;
  Eigen::VectorXd diagnostics_q_actual_;
  DlsSolver dls_solver_;
};

}  // namespace my_robot_controllers
