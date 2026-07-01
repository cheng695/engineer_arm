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

    // 初始化 FDCC 控制器（共享 GravityCompensator 的 Pinocchio 模型）
    if (gravity_compensator_.is_initialized())
    {
        fdcc_controller_.initialize(gravity_compensator_.model(), "tool_link",
                                     50.0, 5.0, 0.002, -1);  // 不锁定任何关节
        fdcc_enabled_ = true;
        RCLCPP_INFO(rclcpp::get_logger("ArmHW"), "FDCC 控制器已初始化");
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

    // FDCC twist 订阅（非实时，仅缓存最新 twist）
    fdcc_sub_ = internal_node_->create_subscription<geometry_msgs::msg::TwistStamped>(
        "/fdcc/twist_cmds", 10,
        [this](const geometry_msgs::msg::TwistStamped::SharedPtr m) {
            fdcc_twist_ = {m->twist.linear.x, m->twist.linear.y, m->twist.linear.z,
                           m->twist.angular.x, m->twist.angular.y, m->twist.angular.z};
            fdcc_twist_countdown_.store(kFdccTimeout, std::memory_order_release);
        });

    motors_enabled_ = false;
    safe_zero_frames_ = 0;

    // 启动 internal_node_ 的独立 spin 线程（处理 /arm_motor_enable /arm_hold_position /fdcc/twist_cmds 订阅）
    rclcpp::executors::SingleThreadedExecutor::SharedPtr exec =
        std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    exec->add_node(internal_node_);
    spin_executor_ = exec;
    spin_thread_ = std::make_unique<std::thread>([exec]() {
        exec->spin();
    });

    // 从 ROS 参数同步控制增益（覆盖 xacro 默认值）
    sync_control_gains();

    RCLCPP_INFO(rclcpp::get_logger("ArmHW"), "on_activate 完成。电机未使能，等待 /arm_motor_enable");
    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArmHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State& /*prev*/)
{
    // 停止 internal_node_ 的 spin 线程
    if (spin_executor_) {
        spin_executor_->cancel();
    }
    if (spin_thread_) {
        spin_thread_->join();
        spin_thread_.reset();
    }
    spin_executor_.reset();

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
    // FDCC 模式优先：有新 twist 且电机已使能，则走 FDCC 路径
    if (process_fdcc())
        return hardware_interface::return_type::OK;

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
    auto it = info_.hardware_parameters.find("robot_description");
    if (it == info_.hardware_parameters.end())
        return false;

    const auto& value = it->second;
    RCLCPP_INFO(rclcpp::get_logger("ArmHW"), "robot_description (%zu 字符)", value.size());

    // 如果是文件路径（以 / 开头或包含 .urdf），用文件方式加载
    if (value.size() < 500 && (value[0] == '/' || value.find(".urdf") != std::string::npos))
        return gravity_compensator_.initialize_from_file(value, rclcpp::get_logger("ArmHW"));
    else
        return gravity_compensator_.initialize(value, rclcpp::get_logger("ArmHW"));
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

bool ArmHardwareInterface::process_fdcc()
{
    if (!fdcc_enabled_) return false;

    int countdown = fdcc_twist_countdown_.load(std::memory_order_acquire);
    if (countdown <= 0) return false;
    fdcc_twist_countdown_.store(countdown - 1, std::memory_order_release);

    RCLCPP_INFO_ONCE(rclcpp::get_logger("ArmHW"), "[FDCC] ACTIVE — J^T 路径运行中");

    // 调试：每秒打印一次 twist 值（看线速度和角速度有没有搞反）
    static auto last_debug = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::seconds>(now - last_debug).count() >= 1)
    {
        last_debug = now;
        RCLCPP_INFO(rclcpp::get_logger("ArmHW"),
            "[FDCC] twist: lin=[%.3f %.3f %.3f] ang=[%.3f %.3f %.3f]",
            fdcc_twist_[0], fdcc_twist_[1], fdcc_twist_[2],
            fdcc_twist_[3], fdcc_twist_[4], fdcc_twist_[5]);
    }

    // 将 twist 发给 FDCC 控制器，计算 {pos, vel, eff}
    auto outputs = fdcc_controller_.compute(fdcc_twist_, hw_states_pos_, hw_states_vel_);

    // 覆盖所有 7 个臂关节的命令（含 Mock），含 NaN 安全检查
    for (size_t i = 0; i < std::min(outputs.size(), info_.joints.size()); ++i)
    {
        if (std::isnan(outputs[i].pos) || std::isnan(outputs[i].vel) || std::isnan(outputs[i].eff))
        {
            RCLCPP_ERROR_ONCE(rclcpp::get_logger("ArmHW"), "[FDCC] NaN detected at joint %zu, aborting", i);
            return false;
        }
        hw_commands_pos_[i] = std::clamp(outputs[i].pos,
            joint_lower_limits_[i], joint_upper_limits_[i]);
        hw_commands_vel_[i] = std::clamp(outputs[i].vel, -30.0, 30.0);
        hw_commands_eff_[i] = outputs[i].eff;
    }

    send_can_commands();
    return true;
}

}  // namespace arm_hardware_interface

PLUGINLIB_EXPORT_CLASS(
    arm_hardware_interface::ArmHardwareInterface,
    hardware_interface::SystemInterface)
