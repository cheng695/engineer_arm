#include "my_robot_control_manager/control_mode_manager.hpp"

#include <chrono>
#include <functional>

using namespace std::chrono_literals;

namespace my_robot_control_manager
{

ControlModeManager::ControlModeManager(const rclcpp::NodeOptions& options)
: rclcpp::Node("control_mode_manager", options)
{
  trajectory_controller_ = declare_parameter<std::string>(
    "trajectory_controller", "arm_trajectory_controller");
  cartesian_controller_ = declare_parameter<std::string>(
    "cartesian_controller", "arm_cartesian_controller");
  joint_controller_ = declare_parameter<std::string>(
    "joint_controller", "arm_joint_velocity_controller");
  hold_controller_ = declare_parameter<std::string>("hold_controller", "");

  fsm_.init();
  state_pub_ = create_publisher<std_msgs::msg::String>("/arm/state/control_mode", 10);
  switch_client_ = create_client<controller_manager_msgs::srv::SwitchController>(
    "/controller_manager/switch_controller");

  enable_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/arm_motor_enable", 10,
    std::bind(&ControlModeManager::enableCallback, this, std::placeholders::_1));
  named_target_sub_ = create_subscription<std_msgs::msg::String>(
    "/arm/command/named_target", 10,
    std::bind(&ControlModeManager::namedTargetCallback, this, std::placeholders::_1));
  cartesian_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
    "/dls_twist_cmds", 10,
    std::bind(&ControlModeManager::cartesianCallback, this, std::placeholders::_1));
  joint_velocity_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
    "/joint_vel_cmds", 10,
    std::bind(&ControlModeManager::jointVelocityCallback, this, std::placeholders::_1));

  update_timer_ = create_wall_timer(10ms, std::bind(&ControlModeManager::update, this));
  publishState();
  RCLCPP_INFO(get_logger(), "[FSM] control_mode_manager 已启动，状态=%s",
              ControlFsm::stateName(fsm_.state()));
}

void ControlModeManager::enableCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg == nullptr) {
    return;
  }
  input_.motors_enabled = msg->data;
  processInput();
}

void ControlModeManager::namedTargetCallback(const std_msgs::msg::String::SharedPtr msg)
{
  if (msg == nullptr || msg->data.empty()) {
    RCLCPP_WARN(get_logger(), "收到空的固定位姿名称");
    return;
  }
  pending_target_name_ = msg->data;
  input_.command_mode = ArmControlState::TRAJECTORY;
  processInput();
}

void ControlModeManager::cartesianCallback(
  const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  if (msg == nullptr) {
    return;
  }
  input_.command_mode = ArmControlState::CARTESIAN;
  processInput();
}

void ControlModeManager::jointVelocityCallback(
  const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (msg == nullptr) {
    return;
  }
  input_.command_mode = ArmControlState::JOINT;
  processInput();
}

void ControlModeManager::processInput()
{
  processEvent(event_detector_.detect(input_));
}

void ControlModeManager::processEvent(ArmControlEvent event)
{
  if (event == ArmControlEvent::None) {
    return;
  }

  const auto transition = fsm_.dispatch(event);
  if (!transition.accepted || !transition.changed) {
    return;
  }

  publishState();
  if (switchControllers(transition.previous, transition.current)) {
    RCLCPP_INFO(
      get_logger(), "[FSM] %s -> %s",
      ControlFsm::stateName(transition.previous),
      ControlFsm::stateName(transition.current));
  }
}

void ControlModeManager::update()
{
  fsm_.update();
}

bool ControlModeManager::switchControllers(
  ArmControlState previous_state, ArmControlState target_state)
{
  if (switch_in_progress_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "controller 切换仍在进行，忽略新的切换请求");
    return false;
  }

  const std::string current_controller = controllerFor(previous_state);
  const std::string target_controller = controllerFor(target_state);

  if (current_controller.empty() && target_controller.empty()) {
    return true;
  }

  if (!switch_client_->service_is_ready()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "controller_manager/switch_controller 服务尚未就绪");
    processEvent(ArmControlEvent::ControllerSwitchFailed);
    return false;
  }

  auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
  if (!current_controller.empty()) {
    request->deactivate_controllers.push_back(current_controller);
  }
  if (!target_controller.empty()) {
    request->activate_controllers.push_back(target_controller);
  }
  request->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;
  request->activate_asap = false;
  request->timeout.sec = 2;
  request->timeout.nanosec = 0;

  switch_in_progress_ = true;
  auto future = switch_client_->async_send_request(
    request,
    [this](rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture result) {
      switch_in_progress_ = false;
      if (!result.get()->ok) {
        RCLCPP_ERROR(get_logger(), "controller 切换失败，FSM 回到 DISABLED/STOP");
        processEvent(ArmControlEvent::ControllerSwitchFailed);
      }
    });
  (void)future;
  return true;
}

void ControlModeManager::publishState()
{
  std_msgs::msg::String msg;
  msg.data = ControlFsm::stateName(fsm_.state());
  state_pub_->publish(msg);
}

std::string ControlModeManager::controllerFor(ArmControlState state) const
{
  switch (state) {
    case ArmControlState::HOLD: return hold_controller_;
    case ArmControlState::CARTESIAN: return cartesian_controller_;
    case ArmControlState::JOINT: return joint_controller_;
    case ArmControlState::TRAJECTORY: return trajectory_controller_;
    case ArmControlState::DISABLED:
    case ArmControlState::PAUSED:
    case ArmControlState::COUNT:
      return "";
  }
  return "";
}

}  // namespace my_robot_control_manager

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<my_robot_control_manager::ControlModeManager>());
  rclcpp::shutdown();
  return 0;
}
