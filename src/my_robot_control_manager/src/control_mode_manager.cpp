#include "my_robot_control_manager/control_mode_manager.hpp"

#include <algorithm>
#include <cmath>
#include <functional>

namespace my_robot_control_manager
{

namespace
{

bool active(double value)
{
    return std::abs(value) > 1e-6;
}

bool twistActive(const geometry_msgs::msg::Twist& twist)
{
    return active(twist.linear.x)  || active(twist.linear.y)  || active(twist.linear.z) ||
           active(twist.angular.x) || active(twist.angular.y) || active(twist.angular.z);
}

}  // namespace

ControlModeManager::ControlModeManager(const rclcpp::NodeOptions& options)
    : rclcpp::Node("control_mode_manager", options)
{
    // ros2 的controller
    trajectory_controller_ = declare_parameter<std::string>(
        "trajectory_controller", "arm_trajectory_controller");
    cartesian_controller_ = declare_parameter<std::string>(
        "cartesian_controller", "arm_cartesian_controller");
    joint_controller_ = declare_parameter<std::string>(
        "joint_controller", "arm_joint_velocity_controller");
    hold_controller_ = declare_parameter<std::string>(
        "hold_controller", "arm_hold_controller");
    
    // fsm 初始化
    fsm_.init();

    // 创建状态发布者
    state_pub_ = create_publisher<std_msgs::msg::String>("/arm/state/control_mode", 10);
    // 创建ROS2 服务客户端，用于调用controller_manager
    switch_client_ = create_client<controller_manager_msgs::srv::SwitchController>(
        "/controller_manager/switch_controller");
    
    // 创建订阅者，收到信息就执行回调函数
    enable_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/arm/command/motor_enable", 10,
        std::bind(&ControlModeManager::enableCallback, this, std::placeholders::_1));
    named_target_sub_ = create_subscription<std_msgs::msg::String>(
        "/arm/command/named_target", 10,
        std::bind(&ControlModeManager::namedTargetCallback, this, std::placeholders::_1));
    control_mode_sub_ = create_subscription<std_msgs::msg::String>(
        "/arm/command/control_mode", 10,
        std::bind(&ControlModeManager::controlModeCallback, this, std::placeholders::_1));
    pause_sub_ = create_subscription<std_msgs::msg::Empty>(
        "/arm/command/pause", 10,
        std::bind(&ControlModeManager::pauseCallback, this, std::placeholders::_1));
    cartesian_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        "/arm/command/cartesian_twist", 10,
        std::bind(&ControlModeManager::cartesianCallback, this, std::placeholders::_1));
    joint_velocity_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        "/arm/command/joint_velocity", 10,
        std::bind(&ControlModeManager::jointVelocityCallback, this, std::placeholders::_1));

    // 发布状态
    publishState();
    
    RCLCPP_INFO(get_logger(), "[FSM] control_mode_manager 已启动，状态=%s",
                ControlFsm::stateName(fsm_.state()));
}

/**
 * @brief 处理电机使能回调
 * 
 * @param msg 使能消息
 */
void ControlModeManager::enableCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg == nullptr) 
    {
      return;
    }
    processEvent(msg->data ? 
      ArmControlEvent::EnableRequested
      : ArmControlEvent::DisableRequested);
}

/**
 * @brief 处理固定位姿回调
 * 
 * @param msg 固定位姿消息
 */
void ControlModeManager::namedTargetCallback(const std_msgs::msg::String::SharedPtr msg)
{
    if (msg == nullptr || msg->data.empty()) 
    {
      RCLCPP_WARN(get_logger(), "收到空的固定位姿名称");
      return;
    }
    pending_target_name_ = msg->data;
    processEvent(ArmControlEvent::NamedTargetRequested);
}

void ControlModeManager::controlModeCallback(
  const std_msgs::msg::String::SharedPtr msg)
{
    if (msg == nullptr) 
    {
      return;
    }
    if (msg->data == "cartesian") 
    {
      processEvent(ArmControlEvent::CartesianRequested);
    } 
    else if (msg->data == "joint") 
    {
      processEvent(ArmControlEvent::JointRequested);
    } 
    else 
    {
      RCLCPP_WARN(get_logger(), "未知控制模式: %s", msg->data.c_str());
    }
}

void ControlModeManager::pauseCallback(const std_msgs::msg::Empty::SharedPtr msg)
{
    if (msg != nullptr) 
    {
      processEvent(ArmControlEvent::StopRequested);
    }
}

void ControlModeManager::cartesianCallback(
  const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
    if (msg == nullptr) 
    {
      return;
    }
    const bool active_now = twistActive(msg->twist);
    if (active_now && !cartesian_command_active_) 
    {
      processEvent(ArmControlEvent::CartesianRequested);
    } 
    else if (!active_now && cartesian_command_active_) 
    {
      processEvent(ArmControlEvent::JoystickReleased);
    }
    cartesian_command_active_ = active_now;
}

void ControlModeManager::jointVelocityCallback(
  const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if (msg == nullptr) 
    {
      return;
    }
    const bool active_now = std::any_of(
      msg->data.begin(), msg->data.end(), active);
    if (active_now && !joint_command_active_) 
    {
      processEvent(ArmControlEvent::JointRequested);
    } 
    else if (!active_now && joint_command_active_) 
    {
      processEvent(ArmControlEvent::JoystickReleased);
    }
    joint_command_active_ = active_now;
}

/**
 * @brief 处理事件
 * 
 * @param event 事件
 */
void ControlModeManager::processEvent(ArmControlEvent event)
{
    if (event == ArmControlEvent::None) 
    {
        return;
    }

    // 获取状态转换
    const auto transition = fsm_.dispatch(event);
    
    if (!transition.accepted || !transition.changed) 
    {
        return;
    }

    // 发布当前状态
    publishState();
    
    // 选择控制器
    if (switchControllers(transition.previous, transition.current)) 
    {
        RCLCPP_INFO(
        get_logger(), "[FSM] %s -> %s",
        ControlFsm::stateName(transition.previous),
        ControlFsm::stateName(transition.current));
    }
}


bool ControlModeManager::switchControllers(
  ArmControlState previous_state, ArmControlState target_state)
{
    if (switch_in_progress_) 
    {
        RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "controller 切换仍在进行，忽略新的切换请求");

        return false;
    }

    const std::string current_controller = controllerFor(previous_state);
    const std::string target_controller  = controllerFor(target_state);

    if (current_controller.empty() && target_controller.empty()) 
    {
        return true;
    }

    if (!switch_client_->service_is_ready()) 
    {
        RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "controller_manager/switch_controller 服务尚未就绪");
        processEvent(ArmControlEvent::ControllerSwitchFailed);

        return false;
    }

    auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    if (!current_controller.empty()) 
    {
        request->deactivate_controllers.push_back(current_controller);
    }
    if (!target_controller.empty()) 
    {
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
        if (!result.get()->ok) 
        {
            RCLCPP_ERROR(get_logger(), "controller 切换失败，FSM 回到 DISABLED/STOP");
            processEvent(ArmControlEvent::ControllerSwitchFailed);
        }
        });
    (void)future;
    return true;
}

/**
 * @brief 发布状态
 */
void ControlModeManager::publishState()
{
  std_msgs::msg::String msg;
  msg.data = ControlFsm::stateName(fsm_.state());
  state_pub_->publish(msg);
}

/**
 * @brief 获取控制器名称
 * 
 * @param state 状态
 * @return std::string 控制器名称
 */
std::string ControlModeManager::controllerFor(ArmControlState state) const
{
  switch (state) {
    case ArmControlState::HOLD:       return hold_controller_;
    case ArmControlState::CARTESIAN:  return cartesian_controller_;
    case ArmControlState::JOINT:      return joint_controller_;
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
