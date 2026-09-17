#include "my_robot_control_manager/control_mode_manager.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <tinyxml2.h>

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
        "joint_controller", "arm_joint_controller");
    hold_controller_ = declare_parameter<std::string>(
        "hold_controller", "arm_hold_controller");
    gravity_controller_ = declare_parameter<std::string>(
        "gravity_controller", "arm_gravity_controller");

    gravity_test_mode_ = declare_parameter<bool>("gravity_test_mode", false);
    gravity_always_on_ = declare_parameter<bool>("gravity_always_on", true);

    robot_description_semantic_ = declare_parameter<std::string>(
        "robot_description_semantic", "");
    trajectory_joints_ = declare_parameter<std::vector<std::string>>(
        "trajectory_joints", {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"});
    trajectory_duration_ = declare_parameter<double>("trajectory_duration", 3.0);

    if (!loadNamedTargets()) {
      RCLCPP_ERROR(get_logger(), "无法从 robot_description_semantic 读取 SRDF 预设位姿");
    }
    trajectory_action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
        this, "/" + trajectory_controller_ + "/follow_joint_trajectory");

    // fsm 初始化
    fsm_.init();

    // 创建状态发布者
    state_pub_ = create_publisher<std_msgs::msg::String>("/arm/state/control_mode", 10);
    // 创建ROS2 服务客户端，用于调用controller_manager
    switch_client_ = create_client<controller_manager_msgs::srv::SwitchController>(
        "/controller_manager/switch_controller");
    list_client_ = create_client<controller_manager_msgs::srv::ListControllers>(
        "/controller_manager/list_controllers");
    controller_ready_timer_ = create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&ControlModeManager::tryPendingControllerSwitch, this));

    hardware_ready_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/arm/state/hardware_ready", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            if (!msg)
                return;

            hardware_ready_ = msg->data;
            if (hardware_ready_)
            {
                if (enable_pending_)
                {
                    enable_pending_ = false;
                    processEvent(ArmControlEvent::EnableRequested);
                }
            }
            else if (fsm_.state() != ArmControlState::DISABLED)
            {
                // 硬件失去 ready 后立即撤销当前控制模式，避免继续接受运动命令。
                enable_pending_ = false;
                pending_switch_valid_ = false;
                processEvent(ArmControlEvent::DisableRequested);
            }
        });
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
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        std::bind(&ControlModeManager::jointStateCallback, this, std::placeholders::_1));

    // 发布状态
    publishState();

    RCLCPP_INFO(get_logger(), "[FSM] control_mode_manager 已启动，状态=%s",
                ControlFsm::stateName(fsm_.state()));
}

/**ru he
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
    if (msg->data)
    {
        if (fsm_.state() != ArmControlState::DISABLED)
        {
            // 已经处于使能状态时，重复使能不改变当前控制模式。
            enable_pending_ = false;
            return;
        }

        // 先等待硬件完成电机使能和反馈确认，再切换控制器。
        enable_pending_ = true;
        if (hardware_ready_)
        {
            enable_pending_ = false;
            processEvent(ArmControlEvent::EnableRequested);
        }
        return;
    }

    enable_pending_ = false;
    processEvent(ArmControlEvent::DisableRequested);
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
    if (named_targets_.find(pending_target_name_) == named_targets_.end())
    {
        RCLCPP_WARN(get_logger(), "SRDF 中不存在预设位姿: %s", pending_target_name_.c_str());
        return;
    }
    processEvent(ArmControlEvent::NamedTargetRequested);
}

/**
 * @brief 是否成功加载固定位姿
 *
 * @return true
 * @return false
 */
bool ControlModeManager::loadNamedTargets()
{
    if (robot_description_semantic_.empty() || trajectory_joints_.empty()) {
      return false;
    }

    tinyxml2::XMLDocument document;
    if (document.Parse(robot_description_semantic_.c_str()) != tinyxml2::XML_SUCCESS) return false;
    const auto root = document.FirstChildElement("robot");
    if (!root) return false;
    named_targets_.clear();
    for (auto state = root->FirstChildElement("group_state"); state; state = state->NextSiblingElement("group_state")) {
        const char* group = state->Attribute("group");
        const char* name = state->Attribute("name");
        if (!group || std::string(group) != "arm" || !name) continue;
        std::map<std::string, double> values;
        bool valid = true;
        for (auto joint = state->FirstChildElement("joint"); joint; joint = joint->NextSiblingElement("joint")) {
            const char* joint_name = joint->Attribute("name");
            double value;
            if (!joint_name || joint->QueryDoubleAttribute("value", &value) != tinyxml2::XML_SUCCESS ||
                !std::isfinite(value) || !values.emplace(joint_name, value).second) { valid = false; break; }
        }
        std::vector<double> target;
        for (const auto& joint : trajectory_joints_) {
            if (!values.count(joint)) { valid = false; break; }
            target.push_back(values.at(joint));
        }
        if (valid) named_targets_.emplace(name, std::move(target));
    }
    return !named_targets_.empty();
}

/**
 * @brief 发送预设位姿
 *
 */
void ControlModeManager::sendPendingTrajectory()
{
    const auto target_it = named_targets_.find(pending_target_name_);
    if (target_it == named_targets_.end() || !trajectory_action_client_)
    {
        processEvent(ArmControlEvent::TrajectoryCompleted);
        return;
    }
    if (!trajectory_action_client_->wait_for_action_server(std::chrono::milliseconds(100)))
    {
        RCLCPP_ERROR(get_logger(), "轨迹 action server 未就绪: %s", trajectory_controller_.c_str());
        processEvent(ArmControlEvent::TrajectoryCompleted);
        return;
    }

    FollowJointTrajectory::Goal goal;
    goal.trajectory.joint_names = trajectory_joints_;
    const double duration = std::max(0.1, trajectory_duration_);

    // 显式加入当前反馈位置作为轨迹起点，避免轨迹控制器激活时
    // 直接把上一个控制器残留的目标值跳变到新的固定位姿目标。
    std::vector<double> start_positions;
    start_positions.reserve(trajectory_joints_.size());
    bool have_valid_start = true;
    for (const auto& joint : trajectory_joints_)
    {
        const auto state_it = latest_joint_positions_.find(joint);
        if (state_it == latest_joint_positions_.end() || !std::isfinite(state_it->second))
        {
            have_valid_start = false;
            break;
        }
        start_positions.push_back(state_it->second);
    }

    if (have_valid_start)
    {
        trajectory_msgs::msg::JointTrajectoryPoint start_point;
        start_point.positions = std::move(start_positions);
        start_point.time_from_start = rclcpp::Duration::from_seconds(std::min(0.1, duration * 0.25));
        goal.trajectory.points.push_back(std::move(start_point));
    }

    trajectory_msgs::msg::JointTrajectoryPoint target_point;
    target_point.positions = target_it->second;
    target_point.time_from_start = rclcpp::Duration::from_seconds(duration);
    goal.trajectory.points.push_back(std::move(target_point));

    RCLCPP_INFO(get_logger(), "开始执行 SRDF 预设位姿: %s", pending_target_name_.c_str());
    rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions options;
    const auto generation = ++trajectory_generation_;
    options.goal_response_callback = [this, generation](auto handle) {
        if (generation != trajectory_generation_ || fsm_.state() != ArmControlState::TRAJECTORY) {
            if (handle) trajectory_action_client_->async_cancel_goal(handle);
            return;
        }
        trajectory_goal_ = handle;
        if (!handle) {
            RCLCPP_ERROR(get_logger(), "轨迹目标被拒绝，进入 HOLD");
            processEvent(ArmControlEvent::TrajectoryCompleted);
        }
    };
    options.result_callback = [this, generation](const auto& result)
    {
        if (generation != trajectory_generation_ || fsm_.state() != ArmControlState::TRAJECTORY) return;
        trajectory_goal_.reset();
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED &&
            result.result && result.result->error_code == FollowJointTrajectory::Result::SUCCESSFUL)
        {
            RCLCPP_INFO(get_logger(), "轨迹执行完成，切换到 HOLD");
            processEvent(ArmControlEvent::TrajectoryCompleted);
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "轨迹执行失败，切换到 HOLD");
            processEvent(ArmControlEvent::TrajectoryCompleted);
        }
    };
    trajectory_action_client_->async_send_goal(goal, options);
}

void ControlModeManager::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    if (!msg || msg->name.size() != msg->position.size())
    {
        return;
    }

    for (size_t i = 0; i < msg->name.size(); ++i)
    {
        if (!msg->name[i].empty() && std::isfinite(msg->position[i]))
        {
            latest_joint_positions_[msg->name[i]] = msg->position[i];
        }
    }
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
    // 笛卡尔零指令时保持 CARTESIAN controller 激活，使其冻结 TCP
    // 参考位姿并从低增益运动纠偏平滑过渡到保持纠偏。显式暂停、
    // 轨迹请求、关节模式和失能仍会切换到对应 controller。
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

    if (gravity_test_mode_ &&
        (event == ArmControlEvent::JointRequested ||
         event == ArmControlEvent::CartesianRequested ||
         event == ArmControlEvent::NamedTargetRequested))
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "纯重力测试模式不接受 JOINT/CARTESIAN/TRAJECTORY 运动请求");
        return;
    }

    if (switch_in_progress_ || controller_query_in_progress_) {
        queued_events_.push_back(event);
        return;
    }
    auto candidate = fsm_;
    const auto transition = candidate.dispatch(event);
    if (!transition.accepted) return;
    if (!transition.changed) {
        if (event == ArmControlEvent::NamedTargetRequested) sendPendingTrajectory();
        return;
    }
    const auto current = controllerFor(transition.previous);
    const auto target = controllerFor(transition.current);
    if (current == target) {
        fsm_ = candidate;
        publishState();
        return;
    }
    pending_candidate_ = candidate;
    pending_current_controller_ = current;
    pending_target_controller_ = target;
    pending_switch_valid_ = true;
    tryPendingControllerSwitch();
}

void ControlModeManager::tryPendingControllerSwitch()
{
    if (!pending_switch_valid_ || switch_in_progress_ || controller_query_in_progress_ ||
        !switch_client_->service_is_ready() || !list_client_->service_is_ready())
        return;

    controller_query_in_progress_ = true;
    auto request = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
    list_client_->async_send_request(request,
        [this](rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedFuture future) {
            controller_query_in_progress_ = false;
            if (!pending_switch_valid_)
                return;

            std::map<std::string, std::string> states;
            for (const auto& controller : future.get()->controller)
                states[controller.name] = controller.state;

            const auto ready = [&states](const std::string& name) {
                if (name.empty()) return true;
                const auto it = states.find(name);
                return it != states.end() &&
                    (it->second == "active" || it->second == "inactive");
            };
            if (!ready(pending_target_controller_) ||
                (gravity_always_on_ && !pending_target_controller_.empty() &&
                 pending_target_controller_ != gravity_controller_ &&
                 !ready(gravity_controller_)))
                return;

            const auto current = pending_current_controller_;
            const auto target = pending_target_controller_;
            if (pending_candidate_.state() != ArmControlState::TRAJECTORY &&
                current == trajectory_controller_)
            {
                ++trajectory_generation_;
                if (trajectory_goal_)
                    trajectory_action_client_->async_cancel_goal(trajectory_goal_);
                trajectory_goal_.reset();
            }
            auto switch_request =
                std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
            if (!current.empty() && states[current] == "active")
                switch_request->deactivate_controllers.push_back(current);
            if (!target.empty() && states[target] != "active")
                switch_request->activate_controllers.push_back(target);
            if (gravity_always_on_ && !target.empty() && target != gravity_controller_ &&
                states[gravity_controller_] != "active")
                switch_request->activate_controllers.push_back(gravity_controller_);
            if (gravity_always_on_ && target.empty() && !gravity_controller_.empty() &&
                states[gravity_controller_] == "active")
                switch_request->deactivate_controllers.push_back(gravity_controller_);

            pending_switch_valid_ = false;
            if (switch_request->activate_controllers.empty() &&
                switch_request->deactivate_controllers.empty())
            {
                fsm_ = pending_candidate_;
                publishState();
                return;
            }
            switch_request->strictness =
                controller_manager_msgs::srv::SwitchController::Request::STRICT;
            switch_request->timeout.sec = 2;
            switch_in_progress_ = true;
            const auto candidate = pending_candidate_;
            switch_client_->async_send_request(switch_request,
                [this, candidate](rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture result) {
                    switch_in_progress_ = false;
                    if (result.get()->ok) {
                        fsm_ = candidate;
                        publishState();
                        if (fsm_.state() == ArmControlState::TRAJECTORY) sendPendingTrajectory();
                    } else {
                        RCLCPP_ERROR(get_logger(), "控制器切换失败，保留已生效状态");
                    }
                    auto events = std::move(queued_events_);
                    queued_events_.clear();
                    for (const auto event : events) processEvent(event);
                });
        });
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
    case ArmControlState::HOLD:
    case ArmControlState::PAUSED:
      return gravity_test_mode_ ? gravity_controller_ : hold_controller_;
    case ArmControlState::CARTESIAN:  return cartesian_controller_;
    case ArmControlState::JOINT:      return joint_controller_;
    case ArmControlState::TRAJECTORY: return trajectory_controller_;
    case ArmControlState::DISABLED:
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
