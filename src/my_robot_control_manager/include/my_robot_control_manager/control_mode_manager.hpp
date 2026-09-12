#pragma once

#include <memory>
#include <string>

#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "my_robot_control_manager/arm_event_detector.hpp"
#include "my_robot_control_manager/control_fsm.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

namespace my_robot_control_manager
{

// ROS2 适配层：接收命令、生成事件、驱动纯 FSM，并执行 controller 切换。
class ControlModeManager : public rclcpp::Node
{
public:
  explicit ControlModeManager(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  void enableCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void namedTargetCallback(const std_msgs::msg::String::SharedPtr msg);
  void cartesianCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void jointVelocityCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

  void processInput();
  void processEvent(ArmControlEvent event);
  void update();
  bool switchControllers(ArmControlState previous_state, ArmControlState target_state);
  void publishState();

  std::string controllerFor(ArmControlState state) const;

  ControlFsm fsm_;
  ArmEventDetector event_detector_;
  ArmInputSnapshot input_{};
  std::string pending_target_name_;

  std::string trajectory_controller_;
  std::string cartesian_controller_;
  std::string joint_controller_;
  std::string hold_controller_;
  bool switch_in_progress_{false};

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr named_target_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cartesian_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joint_velocity_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_client_;
  rclcpp::TimerBase::SharedPtr update_timer_;
};

}  // namespace my_robot_control_manager
