#include "joint/joint_command_controller.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>

#include <pinocchio/parsers/urdf.hpp>
#include "pluginlib/class_list_macros.hpp"

namespace my_robot_controllers
{

controller_interface::CallbackReturn JointCommandController::on_init()
{
    try 
    {
        joint_names_ = auto_declare<std::vector<std::string>>("joints", {});
        command_interface_name_ = auto_declare<std::string>("command_interface", "position");
        command_topic_ = auto_declare<std::string>("command_topic", "~/joint_velocity_cmd");
        robot_description_ = auto_declare<std::string>("robot_description", "");
        command_timeout_ = auto_declare<double>("command_timeout", 0.1);
    } 
    catch (const std::exception&) 
    {
        return controller_interface::CallbackReturn::ERROR;
    }
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointCommandController::on_configure(
  const rclcpp_lifecycle::State&)
{
    get_node()->get_parameter("joints", joint_names_);
    if (joint_names_.empty()) 
    {
        RCLCPP_ERROR(get_node()->get_logger(), "参数 joints 不能为空");
        return controller_interface::CallbackReturn::ERROR;
    }
    get_node()->get_parameter("joints", joint_names_);
    get_node()->get_parameter("command_interface", command_interface_name_);
    get_node()->get_parameter("command_topic", command_topic_);
    get_node()->get_parameter("robot_description", robot_description_);
    get_node()->get_parameter("command_timeout", command_timeout_);

    if (command_interface_name_ != "position") 
    {
        RCLCPP_ERROR(get_node()->get_logger(), "JointCommandController 需要 position command interface");
        return controller_interface::CallbackReturn::ERROR;
    }
    if (robot_description_.empty()) 
    {
        RCLCPP_ERROR(get_node()->get_logger(), "参数 robot_description 不能为空");
        return controller_interface::CallbackReturn::ERROR;
    }
    if (!std::isfinite(command_timeout_) || command_timeout_ <= 0.0)
    {
        RCLCPP_ERROR(get_node()->get_logger(), "command_timeout 必须为正数");
        return controller_interface::CallbackReturn::ERROR;
    }

    try 
    {
        model_ = std::make_unique<pinocchio::Model>();
        pinocchio::urdf::buildModelFromXML(robot_description_, *model_);
        solver_.Init(joint_names_.size(), 0.002);
        velocity_command_.assign(joint_names_.size(), 0.0);
        positions_.assign(joint_names_.size(), 0.0);
        lower_limits_.resize(joint_names_.size());
        upper_limits_.resize(joint_names_.size());
        for (size_t i = 0; i < joint_names_.size(); ++i) 
        {
        const auto id = model_->getJointId(joint_names_[i]);
        if (id == 0 || id >= model_->joints.size() ||
            model_->joints[id].nq() != 1 || model_->joints[id].nv() != 1) 
        {
            throw std::runtime_error("无效的一自由度关节: " + joint_names_[i]);
        }
        const auto qi = model_->joints[id].idx_q();
        if (qi >= model_->nq)
        {
            throw std::runtime_error("关节位置索引越界: " + joint_names_[i]);
        }
        lower_limits_[i] = model_->lowerPositionLimit[qi];
        upper_limits_[i] = model_->upperPositionLimit[qi];
        }
        target_initialized_ = false;
    } 
    catch (const std::exception& e) 
    {
        RCLCPP_ERROR(get_node()->get_logger(), "JointCommandController 初始化失败: %s", e.what());
        model_.reset();
        return controller_interface::CallbackReturn::ERROR;
    }

    diagnostics_.configure(get_node(), joint_names_.size(), "JOINT");
    command_sub_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
        command_topic_, rclcpp::SystemDefaultsQoS(),
        std::bind(&JointCommandController::commandCallback, this, std::placeholders::_1));
    
    RCLCPP_INFO(
        get_node()->get_logger(), "JointCommandController 已配置，关节数量=%zu，接口=%s",
        joint_names_.size(), command_interface_name_.c_str());
    
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointCommandController::on_activate(
  const rclcpp_lifecycle::State&)
{
    if (command_interfaces_.size() < joint_names_.size() ||
        state_interfaces_.size() < 2 * joint_names_.size()) 
    {
        RCLCPP_ERROR(get_node()->get_logger(), "激活时关节接口数量不足");
        return controller_interface::CallbackReturn::ERROR;
    }
    for (size_t i = 0; i < joint_names_.size(); ++i) 
    {
        const double previous_target = command_interfaces_[i].get_value();
        const double feedback = state_interfaces_[2 * i].get_value();
        positions_[i] = has_activated_ && std::isfinite(previous_target)
            ? previous_target : feedback;
        if (!std::isfinite(positions_[i]))
            return controller_interface::CallbackReturn::ERROR;
        command_interfaces_[i].set_value(positions_[i]);
        velocity_command_[i] = 0.0;
    }
    solver_.SyncPositions(positions_);
    target_initialized_ = true;
    command_received_ = false;
    last_command_time_ns_ = 0;
    has_activated_ = true;
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointCommandController::on_deactivate(
  const rclcpp_lifecycle::State&)
{
    // 保留最后位置指令，供 HoldController 接管。
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type JointCommandController::update(
  const rclcpp::Time&, const rclcpp::Duration& period)
{
    // update() 每个控制周期调用一次。只有模型、命令接口和状态接口都准备好时，
    // 才能继续计算并输出关节位置目标。
    if (!model_ || command_interfaces_.size() < joint_names_.size() ||
        state_interfaces_.size() < 2 * joint_names_.size()) 
    {
        return controller_interface::return_type::ERROR;
    }
    // 读取当前关节位置反馈。状态接口按照每个关节 [position, velocity]
    // 的顺序排列，因此位置索引为 2 * i。
    for (size_t i = 0; i < joint_names_.size(); ++i) 
    {
        positions_[i] = state_interfaces_[2 * i].get_value();
    }
    // 第一次更新时，把目标位置同步到当前实际位置，避免积分从零开始，
    // 导致位置命令突然跳变。
    if (!target_initialized_) 
    {
        solver_.SyncPositions(positions_);
        target_initialized_ = true;
    }

    const auto now = std::chrono::steady_clock::now();
    const auto last_command_ns = last_command_time_ns_.load();
    const bool command_valid = command_received_.load() && last_command_ns > 0 &&
        std::chrono::duration<double>(now.time_since_epoch()).count() -
        static_cast<double>(last_command_ns) * 1e-9 <= command_timeout_;

    if (command_valid)
    {
        if (const auto command = command_buffer_.readFromRT())
        {
            if (command->size() == velocity_command_.size())
                velocity_command_ = *command;
        }
    }
    else
    {
        // 命令超时后停止继续积分，保持当前位置目标。
        std::fill(velocity_command_.begin(), velocity_command_.end(), 0.0);
    }
    // JointCommandSolver 对速度指令进行加速度限制、速度限制和关节限位保护，
    // 再根据实际控制周期积分得到目标关节位置。
    const auto output = solver_.Update(
        velocity_command_, positions_, lower_limits_, upper_limits_, period.seconds());

    // 与 DLS 模式保持一致，降低日志输出频率，避免影响控制循环。
    static size_t log_counter = 0;
    const bool should_log = (++log_counter % 50 == 0);

    // position command interface 需要目标关节角度，因此输出积分后的 position。
    for (size_t i = 0; i < output.size(); ++i) 
    {
        command_interfaces_[i].set_value(output[i].position);

        // 定期打印 JOINT 模式下各关节的期望角度和当前反馈角度。
        if (should_log) 
        {
            diagnostics_.record(i, output[i].position, positions_[i], 0.0);
        }
    }
    return controller_interface::return_type::OK;
}

controller_interface::InterfaceConfiguration
JointCommandController::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto& joint : joint_names_) 
    {
        config.names.push_back(joint + "/" + command_interface_name_);
    }
    return config;
}

controller_interface::InterfaceConfiguration
JointCommandController::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto& joint : joint_names_) 
    {
        config.names.push_back(joint + "/position");
        config.names.push_back(joint + "/velocity");
    }
    return config;
}

void JointCommandController::commandCallback(
  const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if (msg == nullptr) 
    {
        return;
    }
    if (msg->data.size() != joint_names_.size() ||
        !std::all_of(msg->data.begin(), msg->data.end(), [](double x) { return std::isfinite(x); })) return;
    command_buffer_.writeFromNonRT(msg->data);
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    last_command_time_ns_ =
        std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
    command_received_ = true;
}

}  // namespace my_robot_controllers

PLUGINLIB_EXPORT_CLASS(
  my_robot_controllers::JointCommandController,
  controller_interface::ControllerInterface)
