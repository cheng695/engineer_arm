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
        cartesian_position_kp_ = auto_declare<double>("cartesian_position_kp", 1.5);
        cartesian_position_kd_ = auto_declare<double>("cartesian_position_kd", 0.1);
        cartesian_orientation_kp_ = auto_declare<double>("cartesian_orientation_kp", 1.5);
        cartesian_orientation_kd_ = auto_declare<double>("cartesian_orientation_kd", 0.1);
        cartesian_linear_correction_limit_ = auto_declare<double>(
            "cartesian_linear_correction_limit", 0.05);
        cartesian_angular_correction_limit_ = auto_declare<double>(
            "cartesian_angular_correction_limit", 0.3);
        reference_following_error_slow_ = auto_declare<double>(
            "reference_following_error_slow", 0.05);
        reference_following_error_stop_ = auto_declare<double>(
            "reference_following_error_stop", 0.15);
        moving_correction_scale_ = auto_declare<double>(
            "moving_correction_scale", 0.2);
        holding_correction_scale_ = auto_declare<double>(
            "holding_correction_scale", 1.0);
        correction_scale_ramp_time_ = auto_declare<double>(
            "correction_scale_ramp_time", 0.3);
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
    get_node()->get_parameter("cartesian_position_kp", cartesian_position_kp_);
    get_node()->get_parameter("cartesian_position_kd", cartesian_position_kd_);
    get_node()->get_parameter("cartesian_orientation_kp", cartesian_orientation_kp_);
    get_node()->get_parameter("cartesian_orientation_kd", cartesian_orientation_kd_);
    get_node()->get_parameter(
        "cartesian_linear_correction_limit", cartesian_linear_correction_limit_);
    get_node()->get_parameter(
        "cartesian_angular_correction_limit", cartesian_angular_correction_limit_);
    get_node()->get_parameter(
        "reference_following_error_slow", reference_following_error_slow_);
    get_node()->get_parameter(
        "reference_following_error_stop", reference_following_error_stop_);
    get_node()->get_parameter("moving_correction_scale", moving_correction_scale_);
    get_node()->get_parameter("holding_correction_scale", holding_correction_scale_);
    get_node()->get_parameter("correction_scale_ramp_time", correction_scale_ramp_time_);

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
    if (!std::isfinite(cartesian_position_kp_) || cartesian_position_kp_ < 0.0 ||
        !std::isfinite(cartesian_position_kd_) || cartesian_position_kd_ < 0.0 ||
        !std::isfinite(cartesian_orientation_kp_) || cartesian_orientation_kp_ < 0.0 ||
        !std::isfinite(cartesian_orientation_kd_) || cartesian_orientation_kd_ < 0.0 ||
        !std::isfinite(cartesian_linear_correction_limit_) ||
        cartesian_linear_correction_limit_ < 0.0 ||
        !std::isfinite(cartesian_angular_correction_limit_) ||
        cartesian_angular_correction_limit_ < 0.0 ||
        !std::isfinite(reference_following_error_slow_) ||
        reference_following_error_slow_ < 0.0 ||
        !std::isfinite(reference_following_error_stop_) ||
        reference_following_error_stop_ <= reference_following_error_slow_ ||
        !std::isfinite(moving_correction_scale_) || moving_correction_scale_ < 0.0 ||
        !std::isfinite(holding_correction_scale_) || holding_correction_scale_ < 0.0 ||
        !std::isfinite(correction_scale_ramp_time_) || correction_scale_ramp_time_ <= 0.0)
    {
        RCLCPP_ERROR(get_node()->get_logger(), "笛卡尔控制参数范围无效");
        return controller_interface::CallbackReturn::ERROR;
    }
    try 
    {
        model_ = std::make_unique<pinocchio::Model>();
        pinocchio::urdf::buildModelFromXML(robot_description_, *model_);
        diagnostics_data_ = std::make_unique<pinocchio::Data>(*model_);
        dls_solver_.Init(*model_, tip_link_, joint_names_, 0.002);
        positions_.assign(joint_names_.size(), 0.0);
        velocities_.assign(joint_names_.size(), 0.0);
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
        reference_pose_initialized_ = false;
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

    // 每次激活都从实际反馈接管，避免重新激活时沿用过期的位置目标。
    for (size_t i = 0; i < joint_names_.size(); ++i) 
    {
        const double feedback = state_interfaces_[2 * i].get_value();
        target_positions_[i] = feedback;
        positions_[i] = feedback;
        if (!std::isfinite(target_positions_[i]))
            return controller_interface::CallbackReturn::ERROR;
        command_interfaces_[i].set_value(target_positions_[i]);
    }
    target_initialized_ = true;
    Eigen::Vector3d position;
    Eigen::Matrix3d rotation;
    if (!compute_tip_pose(positions_, position, rotation))
        return controller_interface::CallbackReturn::ERROR;
    reference_position_ = position;
    reference_orientation_ = Eigen::Quaterniond(rotation).normalized();
    reference_pose_initialized_ = true;
    dls_solver_.reset_velocity_history();
    correction_scale_ = moving_correction_scale_;
    last_command_ = geometry_msgs::msg::TwistStamped{};
    command_received_ = false;
    last_command_time_ns_ = 0;
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DlsCartesianController::on_deactivate(
  const rclcpp_lifecycle::State&)
{
    // 失活前将位置目标收回当前反馈，避免下次激活产生位置跳变。
    for (size_t i = 0; i < joint_names_.size() && i < state_interfaces_.size() / 2; ++i)
    {
        const double feedback = state_interfaces_[2 * i].get_value();
        if (std::isfinite(feedback))
        {
            target_positions_[i] = feedback;
            if (i < command_interfaces_.size())
                command_interfaces_[i].set_value(feedback);
        }
    }
    reference_pose_initialized_ = false;
    return controller_interface::CallbackReturn::SUCCESS;
}

bool DlsCartesianController::compute_tip_pose(
  const std::vector<double>& positions,
  Eigen::Vector3d& position,
  Eigen::Matrix3d& rotation)
{
    if (!model_ || !diagnostics_data_ || !model_->existFrame(tip_link_) ||
        positions.size() != joint_names_.size())
        return false;

    Eigen::VectorXd q = Eigen::VectorXd::Zero(model_->nq);
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
        const auto id = model_->getJointId(joint_names_[i]);
        if (id == 0 || id >= model_->joints.size() ||
            model_->joints[id].nq() != 1 ||
            model_->joints[id].idx_q() >= model_->nq ||
            !std::isfinite(positions[i]))
            return false;
        q[model_->joints[id].idx_q()] = positions[i];
    }
    pinocchio::forwardKinematics(*model_, *diagnostics_data_, q);
    pinocchio::updateFramePlacements(*model_, *diagnostics_data_);
    const auto& placement = diagnostics_data_->oMf[model_->getFrameId(tip_link_)];
    position = placement.translation();
    rotation = placement.rotation();
    return position.allFinite() && rotation.allFinite();
}

bool DlsCartesianController::compute_tip_state(
  const std::vector<double>& positions,
  const std::vector<double>& velocities,
  Eigen::Vector3d& position,
  Eigen::Matrix3d& rotation,
  Eigen::Vector3d& linear_velocity_local,
  Eigen::Vector3d& angular_velocity_local)
{
    if (!model_ || !diagnostics_data_ || !model_->existFrame(tip_link_) ||
        positions.size() != joint_names_.size() ||
        velocities.size() != joint_names_.size())
        return false;

    Eigen::VectorXd q = Eigen::VectorXd::Zero(model_->nq);
    Eigen::VectorXd v = Eigen::VectorXd::Zero(model_->nv);
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
        const auto id = model_->getJointId(joint_names_[i]);
        if (id == 0 || id >= model_->joints.size() ||
            model_->joints[id].nq() != 1 || model_->joints[id].nv() != 1 ||
            model_->joints[id].idx_q() >= model_->nq ||
            model_->joints[id].idx_v() >= model_->nv ||
            !std::isfinite(positions[i]) || !std::isfinite(velocities[i]))
            return false;
        q[model_->joints[id].idx_q()] = positions[i];
        v[model_->joints[id].idx_v()] = velocities[i];
    }

    pinocchio::forwardKinematics(*model_, *diagnostics_data_, q, v);
    pinocchio::updateFramePlacements(*model_, *diagnostics_data_);
    const auto frame_id = model_->getFrameId(tip_link_);
    const auto& placement = diagnostics_data_->oMf[frame_id];
    const auto frame_velocity = pinocchio::getFrameVelocity(
        *model_, *diagnostics_data_, frame_id, pinocchio::LOCAL);
    position = placement.translation();
    rotation = placement.rotation();
    linear_velocity_local = frame_velocity.linear();
    angular_velocity_local = frame_velocity.angular();
    return position.allFinite() && rotation.allFinite() &&
        linear_velocity_local.allFinite() && angular_velocity_local.allFinite();
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

    // 读取当前关节位置和速度。状态接口按“位置、速度”成对排列。
    for (size_t i = 0; i < joint_names_.size(); ++i) 
    {
        positions_[i] = state_interfaces_[2 * i].get_value();
        velocities_[i] = state_interfaces_[2 * i + 1].get_value();
    }
    // 首次运行时将位置目标和 TCP 参考位姿对齐到实际状态，避免启动跳变。
    if (!target_initialized_) 
    {
        target_positions_ = positions_;
        target_initialized_ = true;
    }
    Eigen::Vector3d actual_position;
    Eigen::Matrix3d actual_rotation;
    Eigen::Vector3d actual_linear_velocity_local;
    Eigen::Vector3d actual_angular_velocity_local;
    if (!compute_tip_state(
            positions_, velocities_, actual_position, actual_rotation,
            actual_linear_velocity_local, actual_angular_velocity_local))
        return controller_interface::return_type::ERROR;
    if (!reference_pose_initialized_)
    {
        reference_position_ = actual_position;
        reference_orientation_ = Eigen::Quaterniond(actual_rotation).normalized();
        reference_pose_initialized_ = true;
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
    const std::array<double, 6> joystick_twist{
        last_command_.twist.linear.x, last_command_.twist.linear.y,
        last_command_.twist.linear.z, last_command_.twist.angular.x,
        last_command_.twist.angular.y, last_command_.twist.angular.z};
    // 使用本次控制周期的实际时长，避免固定周期与调度周期不一致。
    const double dt = std::max(1e-6, period.seconds());

    const Eigen::Vector3d reference_linear_velocity_local(
        joystick_twist[0], joystick_twist[1], joystick_twist[2]);
    const Eigen::Vector3d reference_angular_velocity_local(
        joystick_twist[3], joystick_twist[4], joystick_twist[5]);

    const bool joystick_active =
        reference_linear_velocity_local.squaredNorm() > 1e-12 ||
        reference_angular_velocity_local.squaredNorm() > 1e-12;

    // 当关节位置目标已经明显领先于实际反馈时，逐渐降低运动速度，
    // 防止关节目标和 TCP 参考轨迹继续向前累积。
    double maximum_joint_following_error = 0.0;
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
        maximum_joint_following_error = std::max(
            maximum_joint_following_error,
            std::abs(target_positions_[i] - positions_[i]));
    }
    double following_scale = 1.0;
    if (maximum_joint_following_error >= reference_following_error_stop_)
    {
        following_scale = 0.0;
    }
    else if (maximum_joint_following_error > reference_following_error_slow_)
    {
        following_scale =
            (reference_following_error_stop_ - maximum_joint_following_error) /
            (reference_following_error_stop_ - reference_following_error_slow_);
    }

    // 上一周期的任务实现比例用于约束参考轨迹。若上一周期被限位或
    // 奇异性阻挡，则本周期先冻结参考，但仍把摇杆方向交给 DLS，
    // 使用户能够反向退出受限区域。
    const double solver_reference_scale = dls_solver_.blocked()
        ? 0.0
        : std::clamp(dls_solver_.tracking_ratio(), 0.0, 1.0);
    const double reference_motion_scale = following_scale * solver_reference_scale;
    const Eigen::Vector3d commanded_linear_velocity_local =
        following_scale * reference_linear_velocity_local;
    const Eigen::Vector3d commanded_angular_velocity_local =
        following_scale * reference_angular_velocity_local;
    const Eigen::Vector3d scaled_reference_linear_velocity_local =
        reference_motion_scale * reference_linear_velocity_local;
    const Eigen::Vector3d scaled_reference_angular_velocity_local =
        reference_motion_scale * reference_angular_velocity_local;

    // 运动和保持阶段分别使用独立的纠偏比例，切换时平滑增加或减小。
    const double target_correction_scale = joystick_active
        ? moving_correction_scale_
        : holding_correction_scale_;
    const double maximum_scale_step = dt / correction_scale_ramp_time_;
    correction_scale_ += std::clamp(
        target_correction_scale - correction_scale_,
        -maximum_scale_step,
        maximum_scale_step);

    // 摇杆速度在 TCP 局部坐标系中积分为参考位姿；松杆后参考位姿冻结。
    if (command_valid && joystick_active)
    {
        reference_position_ +=
            actual_rotation * scaled_reference_linear_velocity_local * dt;
        const double angle = scaled_reference_angular_velocity_local.norm() * dt;
        if (angle > 1e-12)
        {
            reference_orientation_ = reference_orientation_ *
                Eigen::Quaterniond(Eigen::AngleAxisd(
                    angle, scaled_reference_angular_velocity_local.normalized()));
            reference_orientation_.normalize();
        }
    }

    // 用 TCP 位姿误差和速度误差生成局部坐标系 PD 修正速度。
    // following_scale 同时作用于前馈和纠偏；达到停止阈值后不再推进
    // 关节目标，等待实际关节追上已有目标。
    const Eigen::Vector3d position_error_local =
        actual_rotation.transpose() * (reference_position_ - actual_position);
    const Eigen::Vector3d linear_velocity_error_local =
        scaled_reference_linear_velocity_local - actual_linear_velocity_local;
    Eigen::Vector3d linear_correction = following_scale * correction_scale_ * (
        cartesian_position_kp_ * position_error_local +
        cartesian_position_kd_ * linear_velocity_error_local);
    if (linear_correction.norm() > cartesian_linear_correction_limit_ &&
        linear_correction.norm() > 1e-12)
        linear_correction *= cartesian_linear_correction_limit_ / linear_correction.norm();

    const Eigen::Matrix3d orientation_error_rotation =
        actual_rotation.transpose() * reference_orientation_.toRotationMatrix();
    const Eigen::AngleAxisd orientation_error_angle_axis(orientation_error_rotation);
    const Eigen::Vector3d angular_velocity_error_local =
        scaled_reference_angular_velocity_local - actual_angular_velocity_local;
    Eigen::Vector3d angular_correction = following_scale * correction_scale_ * (
        cartesian_orientation_kp_ * orientation_error_angle_axis.angle() *
            orientation_error_angle_axis.axis() +
        cartesian_orientation_kd_ * angular_velocity_error_local);
    if (angular_correction.norm() > cartesian_angular_correction_limit_ &&
        angular_correction.norm() > 1e-12)
        angular_correction *= cartesian_angular_correction_limit_ / angular_correction.norm();

    const std::array<double, 6> twist{
        commanded_linear_velocity_local.x() + linear_correction.x(),
        commanded_linear_velocity_local.y() + linear_correction.y(),
        commanded_linear_velocity_local.z() + linear_correction.z(),
        commanded_angular_velocity_local.x() + angular_correction.x(),
        commanded_angular_velocity_local.y() + angular_correction.y(),
        commanded_angular_velocity_local.z() + angular_correction.z()};

    // DLS 根据当前关节角度和末端 Twist，求解各关节期望速度。
    const auto output = dls_solver_.Update(twist, positions_, dt);

    // 降低日志频率，避免在实时控制循环中频繁输出。
    diagnostics_.solver(dls_solver_.sigma_min(), dls_solver_.damping(), dls_solver_.blocked(),
        dls_solver_.blocked_joint(), dls_solver_.blocked_at_upper_limit(),
        dls_solver_.task_blocked(), dls_solver_.tracking_ratio(), dls_solver_.raw_tracking_ratio());
    diagnostics_.tracking(
        maximum_joint_following_error, following_scale,
        reference_motion_scale, correction_scale_);
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

    const Eigen::Quaterniond actual_orientation(actual_rotation);
    diagnostics_.cartesian(
        twist,
        {reference_position_.x(), reference_position_.y(), reference_position_.z()},
        {actual_position.x(), actual_position.y(), actual_position.z()},
        {reference_orientation_.x(), reference_orientation_.y(),
            reference_orientation_.z(), reference_orientation_.w()},
        {actual_orientation.x(), actual_orientation.y(),
            actual_orientation.z(), actual_orientation.w()});
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
