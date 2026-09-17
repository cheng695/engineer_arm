#include "dls/dls_cartesian_controller.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include "pluginlib/class_list_macros.hpp"

namespace my_robot_controllers
{

controller_interface::CallbackReturn DlsCartesianController::on_init()
{
    try 
    {
        // 这些参数需要在接口配置函数被调用前就准备好，
        // 这样 controller_manager 才能知道本 controller 要声明哪些资源。
        joint_names_ = auto_declare<std::vector<std::string>>("joints", {});
        command_interface_name_ = auto_declare<std::string>(
        "command_interface", "position");
        command_topic_ = auto_declare<std::string>(
        "command_topic", "~/twist_cmd");
        robot_description_ = auto_declare<std::string>("robot_description", "");
        tip_link_ = auto_declare<std::string>("tip_link", "tool_link");
        command_timeout_ = auto_declare<double>("command_timeout", 0.1);
    } 
    catch (const std::exception&) 
    {
        return controller_interface::CallbackReturn::ERROR;
    }

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DlsCartesianController::on_configure(
  const rclcpp_lifecycle::State&)
{
    if (!get_node()->get_parameter("joints", joint_names_) || joint_names_.empty()) 
    {
        RCLCPP_ERROR(get_node()->get_logger(), "参数 joints 不能为空");
        return controller_interface::CallbackReturn::ERROR;
    }

    get_node()->get_parameter("command_interface", command_interface_name_);
    get_node()->get_parameter("command_topic", command_topic_);
    get_node()->get_parameter("robot_description", robot_description_);
    get_node()->get_parameter("tip_link", tip_link_);
    get_node()->get_parameter("command_timeout", command_timeout_);

    if (command_interface_name_ != "position") 
    {
        RCLCPP_ERROR(
        get_node()->get_logger(),
        "DlsCartesianController 需要 position command interface，当前为 %s",
        command_interface_name_.c_str());
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
        diagnostics_data_ = std::make_unique<pinocchio::Data>(*model_);
        dls_solver_.Init(*model_, tip_link_, joint_names_, 0.002);
        positions_.assign(joint_names_.size(), 0.0);
        target_positions_.assign(joint_names_.size(), 0.0);
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
        diagnostics_q_actual_ = Eigen::VectorXd::Zero(model_->nq);
        diagnostics_q_target_ = Eigen::VectorXd::Zero(model_->nq);
    } 
    catch (const std::exception& e) 
    {
        RCLCPP_ERROR(get_node()->get_logger(), "Pinocchio 初始化失败: %s", e.what());
        model_.reset();
        return controller_interface::CallbackReturn::ERROR;
    }

    diagnostics_.configure(get_node(), joint_names_.size(), "DLS");
    command_sub_ = get_node()->create_subscription<geometry_msgs::msg::TwistStamped>(
        command_topic_, rclcpp::SystemDefaultsQoS(),
        std::bind(&DlsCartesianController::commandCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
        get_node()->get_logger(),
        "DlsCartesianController 已配置，关节数量=%zu，接口=%s，命令话题=%s",
        joint_names_.size(), command_interface_name_.c_str(), command_topic_.c_str());
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DlsCartesianController::on_activate(
  const rclcpp_lifecycle::State&)
{
    if (state_interfaces_.size() < 2 * joint_names_.size()) 
    {
        RCLCPP_ERROR(get_node()->get_logger(), "激活时关节状态接口数量不足");
        return controller_interface::CallbackReturn::ERROR;
    }

    // 首次使用反馈位置，重新激活时继承已有位置目标。
    for (size_t i = 0; i < joint_names_.size(); ++i) 
    {
        const double previous_target = command_interfaces_[i].get_value();
        const double feedback = state_interfaces_[2 * i].get_value();
        target_positions_[i] = has_activated_ && std::isfinite(previous_target)
            ? previous_target : feedback;
        if (!std::isfinite(target_positions_[i]))
            return controller_interface::CallbackReturn::ERROR;
        command_interfaces_[i].set_value(target_positions_[i]);
    }
    target_initialized_ = true;
    dls_solver_.reset_velocity_history();
    last_command_ = geometry_msgs::msg::TwistStamped{};
    command_received_ = false;
    last_command_time_ns_ = 0;
    has_activated_ = true;
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DlsCartesianController::on_deactivate(
  const rclcpp_lifecycle::State&)
{
    // 保留最后位置指令，供 HoldController 接管。
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type DlsCartesianController::update(
  const rclcpp::Time&, const rclcpp::Duration& period)
{
    // 每个控制周期先确认模型和接口数量有效，避免访问越界。
    if (!model_ || command_interfaces_.size() < joint_names_.size() ||
        state_interfaces_.size() < 2 * joint_names_.size()) 
    {
        return controller_interface::return_type::ERROR;
    }

    // 读取当前关节角度。状态接口按“位置、速度”成对排列，位置位于 2*i。
    for (size_t i = 0; i < joint_names_.size(); ++i) 
    {
        positions_[i] = state_interfaces_[2 * i].get_value();
    }
    // 首次运行时将位置目标对齐到实际位置，避免控制器启动时产生跳变。
    if (!target_initialized_) 
    {
        target_positions_ = positions_;
        target_initialized_ = true;
    }
    // 将末端笛卡尔指令组装为 6 维 Twist：线速度 xyz + 角速度 xyz。
    const auto now = std::chrono::steady_clock::now();
    const auto last_command_ns = last_command_time_ns_.load();
    const bool command_valid = command_received_.load() && last_command_ns > 0 &&
        std::chrono::duration<double>(now.time_since_epoch()).count() -
        static_cast<double>(last_command_ns) * 1e-9 <= command_timeout_;
    if (command_valid)
    {
        if (const auto command = command_buffer_.readFromRT())
            last_command_ = *command;
    }
    else
    {
        // 命令超时后使用零 Twist，使位置目标停在当前位置附近。
        last_command_ = geometry_msgs::msg::TwistStamped{};
    }
    const std::array<double, 6> twist{
        last_command_.twist.linear.x, last_command_.twist.linear.y,
        last_command_.twist.linear.z, last_command_.twist.angular.x,
        last_command_.twist.angular.y, last_command_.twist.angular.z};
    // 使用本次控制周期的实际时长，避免固定周期与调度周期不一致。
    const double dt = std::max(1e-6, period.seconds());

    // DLS 根据当前关节角度和末端 Twist，求解各关节期望速度。
    const auto output = dls_solver_.Update(twist, positions_, dt);

    // 降低日志频率，避免在实时控制循环中频繁输出。
    diagnostics_.solver(dls_solver_.sigma_min(), dls_solver_.damping(), dls_solver_.blocked(),
        dls_solver_.blocked_joint(), dls_solver_.blocked_at_upper_limit(),
        dls_solver_.task_blocked(), dls_solver_.tracking_ratio(), dls_solver_.raw_tracking_ratio());
    static size_t log_counter = 0;
    const bool should_log = (++log_counter % 50 == 0);
    for (size_t i = 0; i < output.size(); ++i) 
    {
        // DLS 输出的是关节速度，这里积分成关节位置目标，供位置接口使用。
        target_positions_[i] += output[i].vel * dt;

        // 对积分后的位置目标进行关节限位保护。
        if (lower_limits_[i] > -1e10) 
        {
            target_positions_[i] = std::max(target_positions_[i], lower_limits_[i]);
        }
        if (upper_limits_[i] < 1e10) 
        {
            target_positions_[i] = std::min(target_positions_[i], upper_limits_[i]);
        }
        // 向硬件接口发送本关节的位置期望值。
        command_interfaces_[i].set_value(target_positions_[i]);

        // 周期性输出期望角度、反馈角度和 DLS 计算出的关节速度，便于调试。
        if (should_log) 
        {
            diagnostics_.record(i, target_positions_[i], positions_[i], output[i].vel);
        }
    }

    if (should_log && diagnostics_data_ && model_->existFrame(tip_link_))
    {
        const auto frame_id = model_->getFrameId(tip_link_);
        diagnostics_q_actual_.setZero();
        diagnostics_q_target_.setZero();
        for (size_t i = 0; i < joint_names_.size(); ++i)
        {
            const auto joint_id = model_->getJointId(joint_names_[i]);
            const auto q_index = model_->joints[joint_id].idx_q();
            diagnostics_q_actual_[q_index] = positions_[i];
            diagnostics_q_target_[q_index] = target_positions_[i];
        }
        pinocchio::forwardKinematics(*model_, *diagnostics_data_, diagnostics_q_actual_);
        pinocchio::updateFramePlacements(*model_, *diagnostics_data_);
        const auto actual_center = diagnostics_data_->oMf[frame_id].translation();
        pinocchio::forwardKinematics(*model_, *diagnostics_data_, diagnostics_q_target_);
        pinocchio::updateFramePlacements(*model_, *diagnostics_data_);
        const auto target_center = diagnostics_data_->oMf[frame_id].translation();
        diagnostics_.cartesian(
            twist,
            {target_center.x(), target_center.y(), target_center.z()},
            {actual_center.x(), actual_center.y(), actual_center.z()});
    }
    return controller_interface::return_type::OK;
}

controller_interface::InterfaceConfiguration
DlsCartesianController::command_interface_configuration() const
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
DlsCartesianController::state_interface_configuration() const
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

void DlsCartesianController::commandCallback(
  const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
    if (msg != nullptr) 
    {
        const auto& t = msg->twist;
        const std::array<double, 6> values{t.linear.x, t.linear.y, t.linear.z, t.angular.x, t.angular.y, t.angular.z};
        if (!std::all_of(values.begin(), values.end(), [](double v) { return std::isfinite(v); })) return;
        command_buffer_.writeFromNonRT(*msg);
        const auto now = std::chrono::steady_clock::now().time_since_epoch();
        last_command_time_ns_ =
            std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
        command_received_ = true;
    }
}

}  // namespace my_robot_controllers

PLUGINLIB_EXPORT_CLASS(
  my_robot_controllers::DlsCartesianController,
  controller_interface::ControllerInterface)
