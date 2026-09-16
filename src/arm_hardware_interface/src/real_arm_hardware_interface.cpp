#include "arm_hardware_interface/real_arm_hardware_interface.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <set>
#include <sstream>
#include <thread>

#include "arm_can/damiao_motor/dm_motor.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace arm_hardware_interface
{

RealArmHardwareInterface::~RealArmHardwareInterface()
{
    teardown_internal_node();
    disable_motors();
    device_collection_.closeCANBuses();
}

namespace
{
constexpr auto kFeedbackTimeout = std::chrono::milliseconds(100);
}

void RealArmHardwareInterface::init_gravity_mode(
    const hardware_interface::HardwareInfo& info)
{
    auto it = info.hardware_parameters.find("gravity_compensation_mode");
    const std::string mode = it != info.hardware_parameters.end() ? it->second : "off";
    external_gravity_only_ = mode == "external" || mode == "controller_only" ||
        mode == "external_gravity_only";
    RCLCPP_INFO(rclcpp::get_logger("ArmHW"), "[GRAVITY] mode=%s", mode.c_str());
}

void RealArmHardwareInterface::setup_internal_node()
{
    internal_node_ = rclcpp::Node::make_shared("arm_hw_internal");
    enable_sub_ = internal_node_->create_subscription<std_msgs::msg::Bool>(
        "/arm/command/motor_enable", 10,
        [this](const std_msgs::msg::Bool::SharedPtr message) {
            if (!message)
                return;
            if (message->data)
                enable_requested_ = true;
            else
                disable_requested_ = true;
        });
    raw_motor_state_pub_ = internal_node_->create_publisher<sensor_msgs::msg::JointState>(
        "/arm_debug/raw_motor_states", 10);
    ready_pub_ = internal_node_->create_publisher<std_msgs::msg::Bool>(
        "/arm/state/hardware_ready", 10);
    ready_timer_ = internal_node_->create_wall_timer(
        std::chrono::milliseconds(20), [this] {
            std_msgs::msg::Bool message;
            message.data = hardware_ready_.load();
            ready_pub_->publish(message);
        });
    motors_enabled_ = false;
    safe_zero_frames_ = 0;

    spin_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    spin_executor_->add_node(internal_node_);
    spin_thread_ = std::make_unique<std::thread>([this] { spin_executor_->spin(); });
}

void RealArmHardwareInterface::teardown_internal_node()
{
    if (spin_executor_)
        spin_executor_->cancel();
    if (spin_thread_)
    {
        spin_thread_->join();
        spin_thread_.reset();
    }
    spin_executor_.reset();
    ready_timer_.reset();
    ready_pub_.reset();
    raw_motor_state_pub_.reset();
    enable_sub_.reset();
    internal_node_.reset();
}

void RealArmHardwareInterface::publish_raw_motor_states()
{
    if (!raw_motor_state_pub_ || !internal_node_)
        return;
    sensor_msgs::msg::JointState message;
    message.header.stamp = internal_node_->now();
    message.name.reserve(info_.joints.size());
    message.position = raw_motor_pos_;
    message.velocity = raw_motor_vel_;
    message.effort = raw_motor_eff_;
    for (const auto& joint : info_.joints)
        message.name.push_back(joint.name);
    raw_motor_state_pub_->publish(message);
}

// ================================================================
// 生命周期
// ================================================================

hardware_interface::CallbackReturn RealArmHardwareInterface::on_init(
    const hardware_interface::HardwareInfo& info)
{
    // ros2_control 会先把 URDF 中的 hardware/joint 信息解析到 info_。
    // 后续电机数量、关节名、CAN ID、软限位都依赖这一步成功。
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
        return CallbackReturn::ERROR;

    // 初始化 ros2_control 暴露给 controller 的状态/命令缓存。
    init_joint_buffers(info);
    init_joint_limits(info);
    init_mock_joints(info);

    init_gravity_mode(info);

    // 解析 active_real_joints（允许手动覆盖哪些关节走真实 CAN I/O）。
    // 例如只想调试部分关节时，可以让没有列出的关节继续走 Mock。
    auto it = info_.hardware_parameters.find("active_real_joints");
    if (it != info_.hardware_parameters.end() && !it->second.empty())
    {
        std::stringstream ss(it->second);
        std::string name;
        std::set<std::string> active;
        while (std::getline(ss, name, ','))
        {
            name.erase(std::remove_if(name.begin(), name.end(), ::isspace), name.end());
            if (!name.empty()) active.insert(name);
        }
        for (size_t i = 0; i < info_.joints.size(); ++i)
            use_real_joint_io_[i] = use_real_joint_io_[i] && active.count(info_.joints[i].name) > 0;
    }


    joint_to_motor_.assign(info.joints.size(), static_cast<size_t>(-1));
    if (!init_motors())
    {
        RCLCPP_FATAL(rclcpp::get_logger("ArmHW"), "电机初始化失败");
        return CallbackReturn::ERROR;
    }

    const auto motor_count = device_collection_.size();
    cmd_pos_.resize(motor_count); cmd_vel_.resize(motor_count); cmd_eff_.resize(motor_count);
    cmd_kp_.resize(motor_count); cmd_kd_.resize(motor_count);
    last_feedback_counts_.assign(motor_count, 0);
    last_feedback_times_.assign(motor_count, std::chrono::steady_clock::now());
    // 读取 J2/J3 耦合系数（从 xacro 硬件参数）。
    // 这些值是启动时的默认值，on_activate() 中还会再用 YAML 参数覆盖一次。
    auto it_coupling = info_.hardware_parameters.find("j2j3_coupling");
    if (it_coupling != info_.hardware_parameters.end())
        j2j3_coupling_ = std::stod(it_coupling->second);
    auto it_j3_scale = info_.hardware_parameters.find("j2j3_j3_scale");
    if (it_j3_scale != info_.hardware_parameters.end())
        j2j3_j3_scale_ = std::stod(it_j3_scale->second);
    auto it_j3_offset = info_.hardware_parameters.find("j2j3_j3_offset");
    if (it_j3_offset != info_.hardware_parameters.end())
        j2j3_j3_offset_ = std::stod(it_j3_offset->second);
    auto it_poly_a3 = info_.hardware_parameters.find("j2j3_poly_a3");
    if (it_poly_a3 != info_.hardware_parameters.end())
        j2j3_poly_a3_ = std::stod(it_poly_a3->second);
    auto it_poly_a2 = info_.hardware_parameters.find("j2j3_poly_a2");
    if (it_poly_a2 != info_.hardware_parameters.end())
        j2j3_poly_a2_ = std::stod(it_poly_a2->second);
    auto it_poly_a1 = info_.hardware_parameters.find("j2j3_poly_a1");
    if (it_poly_a1 != info_.hardware_parameters.end())
        j2j3_poly_a1_ = std::stod(it_poly_a1->second);
    auto it_poly_a0 = info_.hardware_parameters.find("j2j3_poly_a0");
    if (it_poly_a0 != info_.hardware_parameters.end())
        j2j3_poly_a0_ = std::stod(it_poly_a0->second);
    auto it_scale_mode = info_.hardware_parameters.find("j2j3_scale_mode");
    if (it_scale_mode != info_.hardware_parameters.end())
        j2j3_scale_mode_ = it_scale_mode->second;

    RCLCPP_INFO(rclcpp::get_logger("ArmHW"), "on_init 完成 (%zu 关节, %zu 真实电机)",
        info_.joints.size(),
        std::count(use_real_joint_io_.begin(), use_real_joint_io_.end(), true));
    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RealArmHardwareInterface::on_activate(
    const rclcpp_lifecycle::State& /*prev*/)
{
    setup_internal_node();

    int opened = device_collection_.openCANBuses();

    // 某条 CAN 总线打不开时，只禁用挂在这条总线上的真实 I/O。
    // 这样 can0/can1 其中一条失败时，另一条总线上的电机仍然可以保留反馈。
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
        auto motor = device_collection_.getMotor(joint_to_motor_[i]);
        if (motor && !device_collection_.isBusOpen(motor->get_bus_name())) {
            RCLCPP_ERROR(rclcpp::get_logger("ArmHW"), "CAN 总线打开失败，拒绝激活");
            teardown_internal_node();
            device_collection_.closeCANBuses();
            return CallbackReturn::ERROR;
        }
    }

    RCLCPP_INFO(rclcpp::get_logger("ArmHW"),
        "CAN 总线: %d 条已打开", opened);

    RCLCPP_INFO(rclcpp::get_logger("ArmHW"), "on_activate 完成。电机未使能，等待 /arm_motor_enable");
    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RealArmHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State& /*prev*/)
{
    teardown_internal_node();
    disable_motors();
    device_collection_.closeCANBuses();
    return CallbackReturn::SUCCESS;
}

// ================================================================
// 接口导出
// ================================================================

std::vector<hardware_interface::StateInterface> RealArmHardwareInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> ifaces;
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
        ifaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_pos_[i]);
        ifaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_vel_[i]);
        ifaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_EFFORT,   &hw_states_eff_[i]);
    }
    return ifaces;
}

std::vector<hardware_interface::CommandInterface> RealArmHardwareInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> ifaces;
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
        ifaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_pos_[i]);
        ifaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_vel_[i]);
        ifaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_EFFORT,   &hw_commands_eff_[i]);
    }
    return ifaces;
}

// ================================================================
// read
// ================================================================

hardware_interface::return_type RealArmHardwareInterface::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    // read() 是 ros2_control 的状态更新入口：
    // 先读取 raw 电机反馈，再完成关节方向转换、Mock 回填和 J2/J3 解耦。
    read_can_feedback();
    if (!check_runtime_feedback())
    {
        disable_motors();
        return hardware_interface::return_type::ERROR;
    }
    publish_raw_motor_states();
    echo_mock_joints(info_);
    apply_j2j3_coupling();
    return hardware_interface::return_type::OK;
}

// ================================================================
// write
// ================================================================

hardware_interface::return_type RealArmHardwareInterface::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    // 优先处理使能/失能/保持请求
    process_motor_requests();
    hardware_ready_ = motors_enabled_ && safe_zero_frames_ == 0;

    if (!motors_enabled_)
    {
        // 未使能时仍发送当前位置保持命令，主要用于持续收反馈和避免控制目标悬空。
        for (size_t i = 0; i < info_.joints.size(); ++i)
        {
            if (!use_real_joint_io_[i]) continue;
            hw_commands_pos_[i] = hw_states_pos_[i];
            hw_commands_vel_[i] = 0.0;
            hw_commands_eff_[i] = 0.0;
        }
        send_can_commands();
        return hardware_interface::return_type::OK;
    }

    if (safe_zero_frames_ > 0)
    {
        // 刚使能后的短暂暖启动：命令目标强制贴住反馈，避免突然跳到旧目标。
        sync_control_targets_to_feedback();
        send_can_commands();
        safe_zero_frames_--;
        return hardware_interface::return_type::OK;
    }

    for (size_t i = 0; i < info_.joints.size(); ++i) {
        if (!std::isfinite(hw_commands_pos_[i]) || !std::isfinite(hw_commands_vel_[i]) ||
            !std::isfinite(hw_commands_eff_[i])) {
            disable_motors();
            return hardware_interface::return_type::ERROR;
        }
    }
    // 位置目标由上层 ros2_control controller 生成，硬件接口不再自行积分速度。
    return send_can_commands() ? hardware_interface::return_type::OK : hardware_interface::return_type::ERROR;
}

// ================================================================
// 电机初始化
// ================================================================

bool RealArmHardwareInterface::init_motors()
{
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
        if (!use_real_joint_io_[i]) continue;
        const auto& j = info_.joints[i];

        // 没有 can_id 的关节由 init_mock_joints 处理，跳过电机创建
        auto it_id = j.parameters.find("can_id");
        if (it_id == j.parameters.end())
        {
            // 夹爪等非 CAN 关节 — 已在 init_mock_joints() 中标记为 Mock
            continue;
        }

        uint32_t id = std::stoi(it_id->second);

        // 每个关节在 xacro 里声明电机型号、kp/kd、CAN 总线和反馈 ID。
        // 这里根据型号创建对应的达妙 MIT 电机对象。
        auto it_m = j.parameters.find("motor_model");
        std::string model = (it_m != j.parameters.end()) ? it_m->second : "J4310";
        float kp = 50.0f, kd = 1.0f;
        auto it_kp = j.parameters.find("kp"); if (it_kp != j.parameters.end()) kp = std::stof(it_kp->second);
        auto it_kd = j.parameters.find("kd"); if (it_kd != j.parameters.end()) kd = std::stof(it_kd->second);
        auto it_bus = j.parameters.find("can_bus");
        std::string bus = (it_bus != j.parameters.end()) ? it_bus->second : "can0";

        std::shared_ptr<arm_can::damiao_motor::DmMotor> motor;
        if (model == "J4310")       motor = std::make_shared<arm_can::damiao_motor::J4310>();
        else if (model == "J4340")  motor = std::make_shared<arm_can::damiao_motor::J4340>();
        else if (model == "J8009")  motor = std::make_shared<arm_can::damiao_motor::J8009>();
        else { RCLCPP_ERROR(rclcpp::get_logger("ArmHW"), "未知型号 %s", model.c_str()); return false; }

        motor->set_can_id(id);
        uint32_t recv_id = id;
        auto it_recv_id = j.parameters.find("recv_can_id");
        if (it_recv_id != j.parameters.end())
            recv_id = static_cast<uint32_t>(std::stoul(it_recv_id->second, nullptr, 0));
        motor->set_recv_can_id(recv_id);
        motor->set_bus_name(bus);
        motor->set_kp(kp);
        motor->set_kd(kd);

        // 电机方向从 xacro 关节参数读取（默认正向 +1.0）
        float direction = 1.0f;
        auto it_dir = j.parameters.find("direction");
        if (it_dir != j.parameters.end())
            direction = std::stof(it_dir->second);
        motor->set_direction(direction);

        joint_to_motor_[i] = device_collection_.size();
        device_collection_.addMotor(motor);
    }
    return true;
}

// ================================================================
// CAN 反馈读取
// ================================================================

/*
    // J2/J3 解耦参数：这些参数通常从 control_gains.yaml 来，
    // 用于把 J3 电机侧角度和关节侧角度互相转换。
    double coupling = j2j3_coupling_;
    internal_node_->get_parameter_or("j2j3_coupling", coupling, coupling);
    if (coupling != j2j3_coupling_)
    {
        j2j3_coupling_ = coupling;
        RCLCPP_INFO(rclcpp::get_logger("ArmHW"),
            "[GAIN] j2j3_coupling=%.4f (来自 YAML)", coupling);
    }
    double j3_scale = j2j3_j3_scale_;
    internal_node_->get_parameter_or("j2j3_j3_scale", j3_scale, j3_scale);
    if (std::abs(j3_scale) < 1e-9)
    {
        RCLCPP_WARN(rclcpp::get_logger("ArmHW"),
            "[GAIN] j2j3_j3_scale 太接近 0，保持 %.4f", j2j3_j3_scale_);
        j3_scale = j2j3_j3_scale_;
    }
    if (j3_scale != j2j3_j3_scale_)
    {
        j2j3_j3_scale_ = j3_scale;
        RCLCPP_INFO(rclcpp::get_logger("ArmHW"),
            "[GAIN] j2j3_j3_scale=%.4f (来自 YAML)", j3_scale);
    }
    double j3_offset = j2j3_j3_offset_;
    internal_node_->get_parameter_or("j2j3_j3_offset", j3_offset, j3_offset);
    if (j3_offset != j2j3_j3_offset_)
    {
        j2j3_j3_offset_ = j3_offset;
        RCLCPP_INFO(rclcpp::get_logger("ArmHW"),
            "[GAIN] j2j3_j3_offset=%.4f rad (来自 YAML)", j3_offset);
    }
    double poly_a3 = j2j3_poly_a3_;
    internal_node_->get_parameter_or("j2j3_poly_a3", poly_a3, poly_a3);
    double poly_a2 = j2j3_poly_a2_;
    internal_node_->get_parameter_or("j2j3_poly_a2", poly_a2, poly_a2);
    double poly_a1 = j2j3_poly_a1_;
    internal_node_->get_parameter_or("j2j3_poly_a1", poly_a1, poly_a1);
    double poly_a0 = j2j3_poly_a0_;
    internal_node_->get_parameter_or("j2j3_poly_a0", poly_a0, poly_a0);
    if (poly_a3 != j2j3_poly_a3_ || poly_a2 != j2j3_poly_a2_ ||
        poly_a1 != j2j3_poly_a1_ || poly_a0 != j2j3_poly_a0_)
    {
        j2j3_poly_a3_ = poly_a3;
        j2j3_poly_a2_ = poly_a2;
        j2j3_poly_a1_ = poly_a1;
        j2j3_poly_a0_ = poly_a0;
        RCLCPP_INFO(rclcpp::get_logger("ArmHW"),
            "[GAIN] j2j3_poly=[%.6f, %.6f, %.6f, %.6f] (来自 YAML)",
            poly_a3, poly_a2, poly_a1, poly_a0);
    }
    std::string scale_mode = j2j3_scale_mode_;
    internal_node_->get_parameter_or("j2j3_scale_mode", scale_mode, scale_mode);
    if (scale_mode != j2j3_scale_mode_)
    {
        j2j3_scale_mode_ = scale_mode;
        RCLCPP_INFO(rclcpp::get_logger("ArmHW"),
            "[GAIN] j2j3_scale_mode=%s (来自 YAML)", scale_mode.c_str());
    }
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
        const auto& name = info_.joints[i].name;
        auto motor = device_collection_.getMotor(joint_to_motor_[i]);
        if (!motor) continue;

        double kp = motor->get_kp();
        double kd = motor->get_kd();

        auto read_gains = [&](const std::string& key) {
            std::string prefix = "arm_control_gains." + key + ".";
            internal_node_->get_parameter_or(prefix + "kp", kp, kp);
            internal_node_->get_parameter_or(prefix + "kd", kd, kd);
        };

        // 先按关节名读取；夹爪再额外兼容 arm_control_gains.gripper 这个别名。
        read_gains(name);
        if (name == "joint_right_finger")
            read_gains("gripper");

        if (kp != motor->get_kp() || kd != motor->get_kd())
        {
            motor->set_kp(static_cast<float>(kp));
            motor->set_kd(static_cast<float>(kd));
            RCLCPP_INFO(rclcpp::get_logger("ArmHW"),
                "[GAIN] %s: kp=%.1f kd=%.1f (来自 YAML)", name.c_str(), kp, kd);
        }
    }

}
*/

void RealArmHardwareInterface::read_can_feedback()
{
    device_collection_.readFeedback();

    // device_collection_ 只包含 CAN 电机，索引与关节不完全对应
    // 这里按 CAN 电机添加顺序读取（跳过 Mock 关节）
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
        if (!use_real_joint_io_[i]) continue;
        auto motor = device_collection_.getMotor(joint_to_motor_[i]);
        if (motor)
        {
            raw_motor_pos_[i] = motor->get_angle_rad();
            raw_motor_vel_[i] = motor->get_velocity_rad();
            raw_motor_eff_[i] = motor->get_torque_nm();
            const double direction = motor->get_direction();
            hw_states_pos_[i] = raw_motor_pos_[i] * direction;
            hw_states_vel_[i] = raw_motor_vel_[i] * direction;
            hw_states_eff_[i] = raw_motor_eff_[i] * direction;
        }
    }
}

bool RealArmHardwareInterface::check_runtime_feedback()
{
    if (!motors_enabled_)
    {
        reset_feedback_monitor();
        return true;
    }

    const auto now = std::chrono::steady_clock::now();
    if (!feedback_monitor_active_)
    {
        for (size_t i = 0; i < device_collection_.size(); ++i)
        {
            const auto* motor = device_collection_.getMotorConst(i);
            last_feedback_counts_[i] = motor ? motor->get_feedback_count() : 0;
            last_feedback_times_[i] = now;
        }
        feedback_monitor_active_ = true;
        return true;
    }

    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
        if (!use_real_joint_io_[i])
            continue;

        const size_t motor_index = joint_to_motor_[i];
        const auto* motor = device_collection_.getMotorConst(motor_index);
        if (!motor)
        {
            RCLCPP_ERROR(rclcpp::get_logger("ArmHW"),
                "[FEEDBACK] %s 对应的电机不存在，立即失能", info_.joints[i].name.c_str());
            return false;
        }

        const size_t feedback_count = motor->get_feedback_count();
        if (feedback_count > last_feedback_counts_[motor_index])
        {
            last_feedback_counts_[motor_index] = feedback_count;
            last_feedback_times_[motor_index] = now;
        }

        const bool feedback_stale = now - last_feedback_times_[motor_index] > kFeedbackTimeout;
        const bool motor_fault = !motor->is_enabled() || motor->get_error_code() != 1;
        if (feedback_stale || motor_fault)
        {
            RCLCPP_ERROR(rclcpp::get_logger("ArmHW"),
                "[FEEDBACK] %s 异常，count=%zu，error_code=%u，stale=%s，立即失能",
                info_.joints[i].name.c_str(), feedback_count,
                static_cast<unsigned int>(motor->get_error_code()),
                feedback_stale ? "true" : "false");
            return false;
        }
    }
    return true;
}

void RealArmHardwareInterface::reset_feedback_monitor()
{
    feedback_monitor_active_ = false;
    std::fill(last_feedback_counts_.begin(), last_feedback_counts_.end(), 0);
    const auto now = std::chrono::steady_clock::now();
    std::fill(last_feedback_times_.begin(), last_feedback_times_.end(), now);
}

void RealArmHardwareInterface::sync_control_targets_to_feedback()
{
    // 把所有控制器内部目标同步到当前反馈。
    // enable/disable/hold/暖启动都会调用它，核心目的都是消除旧目标。
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
        hw_commands_pos_[i] = hw_states_pos_[i];
        hw_commands_vel_[i] = 0.0;
        hw_commands_eff_[i] = 0.0;
    }

}

// ================================================================
// 电机控制请求
// ================================================================

void RealArmHardwareInterface::process_motor_requests()
{
    if (disable_requested_.exchange(false))
    {
        enable_requested_ = false;
        disable_motors();
        return;
    }
    if (enable_requested_.exchange(false) && !motors_enabled_ &&
        enable_phase_ == EnablePhase::Idle)
        {
        enable_attempts_ = 0;
        enable_motors();
    }
    const auto now = std::chrono::steady_clock::now();
    if (enable_phase_ == EnablePhase::Clearing && now >= enable_deadline_)
    {
        enable_feedback_counts_ = real_motor_feedback_counts();
        device_collection_.enableAll();
        enable_phase_ = EnablePhase::Waiting;
        enable_deadline_ = now + std::chrono::milliseconds(100);
    }
    if (enable_phase_ == EnablePhase::Waiting)
    {
        if (all_real_motors_feedback_ok(enable_feedback_counts_))
        {
            motors_enabled_ = true;
            enable_phase_ = EnablePhase::Idle;
            safe_zero_frames_ = kSafeZeroFrames;
            reset_feedback_monitor();
        } else if (now >= enable_deadline_)
        {
            if (enable_attempts_ < 8) enable_motors();
            else disable_motors();
        }
    }
}

void RealArmHardwareInterface::enable_motors()
{
    ++enable_attempts_;
    sync_control_targets_to_feedback();
    device_collection_.clearAllErrors();
    enable_phase_ = EnablePhase::Clearing;
    enable_deadline_ = std::chrono::steady_clock::now() + std::chrono::milliseconds(10);
}

std::vector<size_t> RealArmHardwareInterface::real_motor_feedback_counts() const
{
    std::vector<size_t> counts;
    counts.reserve(device_collection_.size());
    for (size_t i = 0; i < device_collection_.size(); ++i)
    {
        const auto* motor = device_collection_.getMotorConst(i);
        counts.push_back(motor ? motor->get_feedback_count() : 0);
    }
    return counts;
}

bool RealArmHardwareInterface::all_real_motors_feedback_ok(
    const std::vector<size_t>& feedback_counts_before,
    std::string* detail) const
{
    // 判断本轮使能后每个真实电机是否都返回了新反馈，并且反馈状态为 enabled。
    // detail 用于日志里指出具体是哪一个关节缺反馈或状态不对。
    bool ok = true;
    std::ostringstream oss;
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
        if (!use_real_joint_io_[i]) continue;
        const size_t motor_idx = joint_to_motor_[i];

        const auto* motor = device_collection_.getMotorConst(motor_idx);
        const size_t before =
            motor_idx < feedback_counts_before.size() ? feedback_counts_before[motor_idx] : 0;
        const bool has_new_feedback = motor && motor->get_feedback_count() > before;
        const bool enabled = motor && motor->is_enabled();
        if (!has_new_feedback || !enabled)
        {
            ok = false;
            if (oss.tellp() > 0) oss << "; ";
            oss << info_.joints[i].name
                << "(id=" << (motor ? motor->get_can_id() : 0)
                << ", feedback=" << (has_new_feedback ? "new" : "missing")
                << ", status=" << (motor ? static_cast<int>(motor->get_error_code()) : -1)
                << ")";
        }
    }

    if (detail)
        *detail = oss.str();
    return ok;
}

void RealArmHardwareInterface::disable_motors()
{
    enable_phase_ = EnablePhase::Idle;
    hardware_ready_ = false;
    sync_control_targets_to_feedback();
    motors_enabled_ = false;
    reset_feedback_monitor();
    device_collection_.disableAll();
    RCLCPP_INFO(rclcpp::get_logger("ArmHW"), "[POWER] 失能");
}

// ================================================================
// CAN 指令发送
// ================================================================
bool RealArmHardwareInterface::send_can_commands()
{
    // 重力力矩由 GravityCompensationController 写入 hw_commands_eff_，
    // 硬件接口只负责把控制器输出发送给真实电机。

    // 构建电机级命令（仅 CAN 电机，不含 Mock 关节）
    auto& cmd_pos = cmd_pos_;
    auto& cmd_vel = cmd_vel_;
    auto& cmd_eff = cmd_eff_;
    auto& cmd_kp = cmd_kp_;
    auto& cmd_kd = cmd_kd_;
    std::fill(cmd_kp.begin(), cmd_kp.end(), 0.0);
    std::fill(cmd_kd.begin(), cmd_kd.end(), 0.0);

    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
        if (!use_real_joint_io_[i]) continue;
        const size_t motor_idx = joint_to_motor_[i];

        const bool gravity_only_output = external_gravity_only_;
        double pos = gravity_only_output ? hw_states_pos_[i] : hw_commands_pos_[i];
        double vel = gravity_only_output ? 0.0 : hw_commands_vel_[i];
        double eff = hw_commands_eff_[i];

        if (i == kJ3Index && use_real_joint_io_[kJ2Index])
        {
            // J3 关节目标 -> J3 电机目标：
            // controller 看到的是解耦后的关节角，发给电机前要把 J2 引入的耦合量反算回去。
            const double j2_pos = gravity_only_output ? hw_states_pos_[kJ2Index] : hw_commands_pos_[kJ2Index];
            const double j2_vel = gravity_only_output ? 0.0 : hw_commands_vel_[kJ2Index];

            const double correction = j2j3_poly_correction(j2_pos);
            const double derivative = j2j3_poly_derivative(j2_pos);
            const double scale = j2j3_j3_scale_;

            if (j2j3_scale_mode_is_multiply())
            {
                pos = (pos - correction) / scale;
                vel = (vel - derivative * j2_vel) / scale;
                eff *= scale;
            }
            else
            {
                pos = scale * pos - correction;
                vel = scale * vel - derivative * j2_vel;
                eff /= scale;
            }
        }
        if (i == kJ2Index && use_real_joint_io_[kJ3Index])
        {
            // J3 电机力矩会通过同步带反作用到 J2 电机侧；
            // 这里把 J3 的力矩命令按解耦导数映射回 J2，保证力矩通道一致。
            const double j3_eff = hw_commands_eff_[kJ3Index];
            const double j2_pos = gravity_only_output ? hw_states_pos_[kJ2Index] : hw_commands_pos_[kJ2Index];
            const double derivative = j2j3_poly_derivative(j2_pos);
            eff += derivative * j3_eff / (j2j3_scale_mode_is_multiply() ? 1.0 : j2j3_j3_scale_);
        }

        cmd_pos[motor_idx] = pos;
        cmd_vel[motor_idx] = vel;
        cmd_eff[motor_idx] = eff;

        // gravity_only 模式只输出重力前馈，不让位置环继续拉目标，所以 kp/kd 置 0。
        cmd_kp[motor_idx] = gravity_only_output ? 0.0 : device_collection_.getMotor(motor_idx)->get_kp();
        cmd_kd[motor_idx] = gravity_only_output ? 0.0 : device_collection_.getMotor(motor_idx)->get_kd();
    }

    return device_collection_.sendCommandsWithGains(cmd_pos, cmd_vel, cmd_eff, cmd_kp, cmd_kd);
}

}  // namespace arm_hardware_interface

PLUGINLIB_EXPORT_CLASS(
    arm_hardware_interface::RealArmHardwareInterface,
    hardware_interface::SystemInterface)
