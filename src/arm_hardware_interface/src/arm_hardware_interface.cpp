#include "arm_hardware_interface/arm_hardware_interface.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>
#include <set>
#include <sstream>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace arm_hardware_interface
{

namespace
{
// 默认限位（当 joint 参数中未指定 lower_limit/upper_limit 时回退使用）
constexpr double kDefaultLowerLimit = -std::numeric_limits<double>::infinity();
constexpr double kDefaultUpperLimit =  std::numeric_limits<double>::infinity();
}  // namespace

// ================================================================
// 生命周期
// ================================================================

hardware_interface::CallbackReturn ArmHardwareInterface::on_init(
    const hardware_interface::HardwareInfo& info)
{
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
        return CallbackReturn::ERROR;

    init_joint_buffers();
    init_joint_limits();
    init_mock_joints();

    if (!init_pinocchio())
        RCLCPP_WARN(rclcpp::get_logger("ArmHW"), "重力补偿未启用");

    if (!init_motors())
    {
        RCLCPP_FATAL(rclcpp::get_logger("ArmHW"), "电机初始化失败");
        return CallbackReturn::ERROR;
    }

    // 读取 J2/J3 耦合系数（从 xacro 硬件参数）
    auto it_coupling = info_.hardware_parameters.find("j2j3_coupling");
    if (it_coupling != info_.hardware_parameters.end())
        j2j3_coupling_ = std::stod(it_coupling->second);

    // 解析 active_real_joints（允许手动覆盖）
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
            use_real_joint_io_[i] = active.count(info_.joints[i].name) > 0;
    }

    RCLCPP_INFO(rclcpp::get_logger("ArmHW"), "on_init 完成 (%zu 关节, %zu 真实电机)",
        info_.joints.size(),
        std::count(use_real_joint_io_.begin(), use_real_joint_io_.end(), true));
    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArmHardwareInterface::on_activate(
    const rclcpp_lifecycle::State& /*prev*/)
{
    int opened = device_collection_.openCANBuses();

    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
        auto motor = device_collection_.getMotor(i);
        if (motor && !device_collection_.isBusOpen(motor->get_bus_name()))
            use_real_joint_io_[i] = false;
    }

    RCLCPP_INFO(rclcpp::get_logger("ArmHW"),
        "CAN 总线: %d 条已打开", opened);

    internal_node_ = rclcpp::Node::make_shared("arm_hw_internal");

    enable_sub_ = internal_node_->create_subscription<std_msgs::msg::Bool>(
        "/arm_motor_enable", 10,
        [this](const std_msgs::msg::Bool::SharedPtr m) {
            if (m->data) enable_requested_ = true; else disable_requested_ = true;
        });

    hold_sub_ = internal_node_->create_subscription<std_msgs::msg::Bool>(
        "/arm_hold_position", 10,
        [this](const std_msgs::msg::Bool::SharedPtr m) {
            if (m->data) hold_requested_ = true;
        });

    motors_enabled_ = false;
    safe_zero_frames_ = 0;

    // 从 ROS 参数同步控制增益（覆盖 xacro 默认值）
    sync_control_gains();

    RCLCPP_INFO(rclcpp::get_logger("ArmHW"), "on_activate 完成。电机未使能，等待 /arm_motor_enable");
    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArmHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State& /*prev*/)
{
    disable_motors();
    device_collection_.closeCANBuses();
    return CallbackReturn::SUCCESS;
}

// ================================================================
// 接口导出
// ================================================================

std::vector<hardware_interface::StateInterface> ArmHardwareInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> ifaces;
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
        ifaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_pos_[i]);
        ifaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_vel_[i]);
        ifaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_states_eff_[i]);
    }
    return ifaces;
}

std::vector<hardware_interface::CommandInterface> ArmHardwareInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> ifaces;
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
        ifaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_pos_[i]);
        ifaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_vel_[i]);
        ifaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_commands_eff_[i]);
    }
    return ifaces;
}

// ================================================================
// read
// ================================================================

hardware_interface::return_type ArmHardwareInterface::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    read_can_feedback();

    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
        if (!use_real_joint_io_[i])
        {
            hw_states_pos_[i] = hw_commands_pos_[i];
            hw_states_vel_[i] = hw_commands_vel_[i];
            hw_states_eff_[i] = 0.0;
        }
    }

    apply_j2j3_coupling();

    if (gravity_compensator_.is_initialized())
    {
        auto tau = gravity_compensator_.compute(hw_states_pos_);
        for (size_t i = 0; i < std::min(tau.size(), hw_states_eff_.size()); ++i)
            hw_states_eff_[i] = tau[i];
    }

    return hardware_interface::return_type::OK;
}

// ================================================================
// write
// ================================================================

hardware_interface::return_type ArmHardwareInterface::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& period)
{
    process_motor_requests();

    if (!motors_enabled_)
    {
        for (size_t i = 0; i < info_.joints.size(); ++i)
        {
            if (!use_real_joint_io_[i]) continue;
            hw_commands_pos_[i] = hw_states_pos_[i];
            hw_commands_vel_[i] = 0.0;
        }
        send_can_commands();
        return hardware_interface::return_type::OK;
    }

    if (safe_zero_frames_ > 0)
    {
        for (size_t i = 0; i < info_.joints.size(); ++i)
            { hw_commands_vel_[i] = 0.0; hw_commands_eff_[i] = 0.0; }
        send_can_commands();
        safe_zero_frames_--;
        return hardware_interface::return_type::OK;
    }

    // 速度模式检测
    bool has_velocity = false;
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
        if (std::abs(hw_commands_vel_[i]) > 1e-6)
        {
            has_velocity = true;
            break;
        }
    }

    double dt = period.seconds();

    if (has_velocity)
    {
        if (!vel_mode_active_)
        {
            integrated_pos_ = hw_commands_pos_;
            RCLCPP_INFO(rclcpp::get_logger("ArmHW"), "[VEL_MODE] 进入速度模式");
        }
        vel_mode_active_ = true;

        for (size_t i = 0; i < info_.joints.size(); ++i)
        {
            if (!use_real_joint_io_[i]) continue;
            integrated_pos_[i] = std::clamp(
                integrated_pos_[i] + hw_commands_vel_[i] * dt,
                joint_lower_limits_[i], joint_upper_limits_[i]);
            hw_commands_pos_[i] = integrated_pos_[i];
        }
    }
    else
    {
        if (vel_mode_active_)
        {
            RCLCPP_INFO(rclcpp::get_logger("ArmHW"), "[VEL_MODE] 退出速度模式");
        }
        vel_mode_active_ = false;
    }

    send_can_commands();
    return hardware_interface::return_type::OK;
}

// ================================================================
// 初始化辅助
// ================================================================

void ArmHardwareInterface::init_joint_buffers()
{
    const size_t n = info_.joints.size();
    hw_states_pos_.resize(n, 0.0);   hw_states_vel_.resize(n, 0.0);   hw_states_eff_.resize(n, 0.0);
    hw_commands_pos_.resize(n, 0.0); hw_commands_vel_.resize(n, 0.0); hw_commands_eff_.resize(n, 0.0);
    use_real_joint_io_.resize(n, true);
}

void ArmHardwareInterface::init_joint_limits()
{
    for (const auto& j : info_.joints)
    {
        double lo = kDefaultLowerLimit, hi = kDefaultUpperLimit;

        auto it_lo = j.parameters.find("lower_limit");
        if (it_lo != j.parameters.end())
            lo = std::stod(it_lo->second);

        auto it_hi = j.parameters.find("upper_limit");
        if (it_hi != j.parameters.end())
            hi = std::stod(it_hi->second);

        joint_lower_limits_.push_back(lo);
        joint_upper_limits_.push_back(hi);
    }
}

void ArmHardwareInterface::init_mock_joints()
{
    // 没有 can_id 参数的关节自动标记为 Mock（如夹爪）
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
        if (info_.joints[i].parameters.find("can_id") == info_.joints[i].parameters.end())
            use_real_joint_io_[i] = false;
    }
}

bool ArmHardwareInterface::init_pinocchio()
{
    // 优先从 hardware_parameters 获取 robot_description
    // 其次尝试 robot_description_path（遗留兼容）
    auto it = info_.hardware_parameters.find("robot_description");
    if (it != info_.hardware_parameters.end())
        return gravity_compensator_.initialize(it->second, rclcpp::get_logger("ArmHW"));

    auto it_path = info_.hardware_parameters.find("robot_description_path");
    if (it_path != info_.hardware_parameters.end())
        return gravity_compensator_.initialize(it_path->second, rclcpp::get_logger("ArmHW"));

    return false;
}

bool ArmHardwareInterface::init_motors()
{
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
        const auto& j = info_.joints[i];

        // 没有 can_id 的关节由 mock_joints 处理，跳过电机创建
        auto it_id = j.parameters.find("can_id");
        if (it_id == j.parameters.end())
        {
            // 夹爪等非 CAN 关节 — 已在 init_mock_joints() 中标记为 Mock
            continue;
        }

        uint32_t id = std::stoi(it_id->second);

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
        motor->set_bus_name(bus);
        motor->set_kp(kp);
        motor->set_kd(kd);

        // 电机方向从 xacro 关节参数读取（默认正向 +1.0）
        float direction = 1.0f;
        auto it_dir = j.parameters.find("direction");
        if (it_dir != j.parameters.end())
            direction = std::stof(it_dir->second);
        motor->set_direction(direction);

        device_collection_.addMotor(motor);
    }
    return true;
}

void ArmHardwareInterface::sync_control_gains()
{
    // 从 ROS 参数读取控制增益，覆盖 xacro 默认值（对标 OpenArm control_gains.yaml）
    if (!internal_node_)
        return;

    // j2j3_coupling
    double coupling = j2j3_coupling_;
    internal_node_->get_parameter_or("j2j3_coupling", coupling, coupling);
    if (coupling != j2j3_coupling_)
    {
        j2j3_coupling_ = coupling;
        RCLCPP_INFO(rclcpp::get_logger("ArmHW"),
            "[GAIN] j2j3_coupling=%.4f (来自 YAML)", coupling);
    }

    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
        const auto& name = info_.joints[i].name;
        auto motor = device_collection_.getMotor(i);
        if (!motor) continue;

        std::string prefix = "arm_control_gains." + name + ".";

        double kp = motor->get_kp();
        double kd = motor->get_kd();

        internal_node_->get_parameter_or(prefix + "kp", kp, kp);
        internal_node_->get_parameter_or(prefix + "kd", kd, kd);

        if (kp != motor->get_kp() || kd != motor->get_kd())
        {
            motor->set_kp(static_cast<float>(kp));
            motor->set_kd(static_cast<float>(kd));
            RCLCPP_INFO(rclcpp::get_logger("ArmHW"),
                "[GAIN] %s: kp=%.1f kd=%.1f (来自 YAML)", name.c_str(), kp, kd);
        }
    }
}

// ================================================================
// read 辅助
// ================================================================

void ArmHardwareInterface::read_can_feedback()
{
    device_collection_.readFeedback();

    // 注意：device_collection_ 只包含 CAN 电机，索引与关节不完全对应
    // 我们需要通过 can_id 来查找，但这里简化为：
    // - CAN 电机的索引按添加顺序（跳过 Mock 关节后的顺序）
    // - Mock 关节在上层已通过 use_real_joint_io_[i] 标记
    size_t motor_idx = 0;
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
        if (!use_real_joint_io_[i]) continue;

        auto motor = device_collection_.getMotor(motor_idx++);
        if (motor)
        {
            hw_states_pos_[i] = motor->get_angle_rad();
            hw_states_vel_[i] = motor->get_velocity_rad();
            hw_states_eff_[i] = motor->get_torque_nm();
        }
    }
}

void ArmHardwareInterface::apply_j2j3_coupling()
{
    if (use_real_joint_io_[kJ2Index] && use_real_joint_io_[kJ3Index])
    {
        double raw_j3_pos = hw_states_pos_[kJ3Index];
        double raw_j3_vel = hw_states_vel_[kJ3Index];
        hw_states_pos_[kJ3Index] = raw_j3_pos + j2j3_coupling_ * hw_states_pos_[kJ2Index];
        hw_states_vel_[kJ3Index] = raw_j3_vel + j2j3_coupling_ * hw_states_vel_[kJ2Index];
    }
}

// ================================================================
// 电机控制
// ================================================================

void ArmHardwareInterface::process_motor_requests()
{
    if (enable_requested_.exchange(false))  enable_motors();
    if (disable_requested_.exchange(false)) disable_motors();
    if (hold_requested_.exchange(false))    hold_position();
}

void ArmHardwareInterface::enable_motors()
{
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
        hw_commands_pos_[i] = hw_states_pos_[i];
        hw_commands_vel_[i] = 0.0;
        hw_commands_eff_[i] = 0.0;
    }

    device_collection_.enableAll();

    motors_enabled_ = true;
    safe_zero_frames_ = kSafeZeroFrames;
    RCLCPP_INFO(rclcpp::get_logger("ArmHW"), "[POWER] 使能 (%d 帧暖启动)", kSafeZeroFrames);
}

void ArmHardwareInterface::disable_motors()
{
    motors_enabled_ = false;
    device_collection_.disableAll();
    RCLCPP_INFO(rclcpp::get_logger("ArmHW"), "[POWER] 失能");
}

void ArmHardwareInterface::hold_position()
{
    for (size_t i = 0; i < info_.joints.size(); ++i)
        { hw_commands_pos_[i] = hw_states_pos_[i]; hw_commands_vel_[i] = 0.0; }
    safe_zero_frames_ = 0;
    RCLCPP_INFO(rclcpp::get_logger("ArmHW"), "[HOLD] 位置锁定");
}

// ================================================================
// CAN 交替发送
// ================================================================

void ArmHardwareInterface::send_can_commands()
{
    std::vector<double> grav(kJointCount, 0.0);
    if (gravity_compensator_.is_initialized())
        grav = gravity_compensator_.compute(hw_states_pos_);

    // 构建电机级命令（仅 CAN 电机，不含 Mock 关节）
    // device_collection_ 中电机按添加顺序排列（与关节顺序相同，但只含 CAN 关节）
    size_t n_motors = device_collection_.size();
    std::vector<double> cmd_pos(n_motors, 0.0);
    std::vector<double> cmd_vel(n_motors, 0.0);
    std::vector<double> cmd_eff(n_motors, 0.0);

    size_t motor_idx = 0;
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
        if (!use_real_joint_io_[i]) continue;
        cmd_pos[motor_idx] = hw_commands_pos_[i];
        cmd_vel[motor_idx] = hw_commands_vel_[i];
        cmd_eff[motor_idx] = hw_commands_eff_[i] + (i < grav.size() ? grav[i] : 0.0);
        motor_idx++;
    }

    device_collection_.sendCommands(cmd_pos, cmd_vel, cmd_eff);
}

}  // namespace arm_hardware_interface

PLUGINLIB_EXPORT_CLASS(
    arm_hardware_interface::ArmHardwareInterface,
    hardware_interface::SystemInterface)
