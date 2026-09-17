#pragma once

#include <memory>
#include <map>
#include <string>
#include <vector>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "my_robot_control_manager/control_fsm.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

namespace my_robot_control_manager
{

// ROS2 适配层：接收语义化命令、驱动纯 FSM，并执行 controller 切换。
class ControlModeManager : public rclcpp::Node
{
public:
  explicit ControlModeManager(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  void enableCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void namedTargetCallback(const std_msgs::msg::String::SharedPtr msg);
  void controlModeCallback(const std_msgs::msg::String::SharedPtr msg);
  void pauseCallback(const std_msgs::msg::Empty::SharedPtr msg);
  void cartesianCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void jointVelocityCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

  void processEvent(ArmControlEvent event);
  void tryPendingControllerSwitch();
  bool loadNamedTargets();
  void sendPendingTrajectory();
  void publishState();

  std::string controllerFor(ArmControlState state) const;

  bool enable_pending_{false}, hardware_ready_{false};
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr hardware_ready_sub_;
  ControlFsm fsm_;
  std::string pending_target_name_;
  bool cartesian_command_active_{false};
  bool joint_command_active_{false};

  std::string trajectory_controller_;
  std::string cartesian_controller_;
  std::string joint_controller_;
  std::string hold_controller_;
  std::string gravity_controller_;
  bool gravity_test_mode_{false};
  bool gravity_always_on_{false};
  std::string robot_description_semantic_;
  std::vector<std::string> trajectory_joints_;
  double trajectory_duration_{3.0};
  std::map<std::string, std::vector<double>> named_targets_;
  std::map<std::string, double> latest_joint_positions_;
  bool switch_in_progress_{false};
  bool controller_query_in_progress_{false};
  bool pending_switch_valid_{false};
  ControlFsm pending_candidate_;
  std::string pending_current_controller_;
  std::string pending_target_controller_;
  std::vector<ArmControlEvent> queued_events_;
  size_t trajectory_generation_{0};
  rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr trajectory_goal_;

  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr trajectory_action_client_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr named_target_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr control_mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr pause_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cartesian_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joint_velocity_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_client_;
  rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr list_client_;
  rclcpp::TimerBase::SharedPtr controller_ready_timer_;
};

}  // namespace my_robot_control_manager
