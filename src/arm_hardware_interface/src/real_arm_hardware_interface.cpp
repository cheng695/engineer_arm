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

// ================================================================
// 生命周期
// ================================================================

hardware_interface::CallbackReturn RealArmHardwareInterface::on_init(
    const hardware_interface::HardwareInfo& info)
{
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
        return CallbackReturn::ERROR;

    init_joint_buffers(info);
    init_joint_limits(info);
    init_mock_joints(info);

    if (!init_gravity_compensator(info))
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

    auto it_gravity_scale = info_.hardware_parameters.find("gravity_effort_scale");
    if (it_gravity_scale != info_.hardware_parameters.end())
        gravity_effort_scale_ = std::stod(it_gravity_scale->second);
    auto it_j3_gravity_scale = info_.hardware_parameters.find("j2j3_j3_gravity_effort_scale");
    if (it_j3_gravity_scale != info_.hardware_parameters.end())
        j3_gravity_effort_scale_ = std::stod(it_j3_gravity_scale->second);

    // 解析 active_real_joints（允许手动覆盖哪些关节走真实 CAN I/O）
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

    init_dls();
    init_joint_controller(info);

    RCLCPP_INFO(rclcpp::get_logger("ArmHW"), "on_init 完成 (%zu 关节, %zu 真实电机)",
        info_.joints.size(),
        std::count(use_real_joint_io_.begin(), use_real_joint_io_.end(), true));
    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RealArmHardwareInterface::on_activate(
    const rclcpp_lifecycle::State& /*prev*/)
{
    setup_internal_node("arm_hw_internal");

    int opened = device_collection_.openCANBuses();

    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
        auto motor = device_collection_.getMotor(i);
        if (motor && !device_collection_.isBusOpen(motor->get_bus_name()))
            use_real_joint_io_[i] = false;
    }

    RCLCPP_INFO(rclcpp::get_logger("ArmHW"),
        "CAN 总线: %d 条已打开", opened);

    // 从 ROS 参数同步控制增益（覆盖 xacro 默认值）
    sync_control_gains();

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
    read_can_feedback();
    echo_mock_joints(info_);
    apply_j2j3_coupling();
    apply_gravity_to_effort();
    publish_feedback_debug(info_, true);
    return hardware_interface::return_type::OK;
}

// ================================================================
// write
// ================================================================

hardware_interface::return_type RealArmHardwareInterface::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& period)
{
    // 优先处理使能/失能/保持请求（不被 FDCC 阻塞）
    process_motor_requests();

    if (!motors_enabled_)
    {
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
        sync_control_targets_to_feedback();
        send_can_commands();
        safe_zero_frames_--;
        return hardware_interface::return_type::OK;
    }

    // FSM 统一控制
    if (process_control(info_))
    {
        send_can_commands();
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
// 电机初始化
// ================================================================

bool RealArmHardwareInterface::init_motors()
{
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
        const auto& j = info_.joints[i];

        // 没有 can_id 的关节由 init_mock_joints 处理，跳过电机创建
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

        device_collection_.addMotor(motor);
    }
    return true;
}

// ================================================================
// 控制增益同步
// ================================================================

void RealArmHardwareInterface::sync_control_gains()
{
    // 从 ROS 参数读取控制增益，覆盖 xacro 默认值
    if (!internal_node_)
        return;

    // j2j3 coupling parameters
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
    double j3_gravity_scale = j3_gravity_effort_scale_;
    internal_node_->get_parameter_or(
        "j2j3_j3_gravity_effort_scale", j3_gravity_scale, j3_gravity_scale);
    if (j3_gravity_scale != j3_gravity_effort_scale_)
    {
        j3_gravity_effort_scale_ = j3_gravity_scale;
        RCLCPP_INFO(rclcpp::get_logger("ArmHW"),
            "[GAIN] j2j3_j3_gravity_effort_scale=%.4f (来自 YAML)", j3_gravity_scale);
    }

    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
        const auto& name = info_.joints[i].name;
        auto motor = device_collection_.getMotor(i);
        if (!motor) continue;

        double kp = motor->get_kp();
        double kd = motor->get_kd();

        auto read_gains = [&](const std::string& key) {
            std::string prefix = "arm_control_gains." + key + ".";
            internal_node_->get_parameter_or(prefix + "kp", kp, kp);
            internal_node_->get_parameter_or(prefix + "kd", kd, kd);
        };

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

    sync_gravity_parameters();
}

void RealArmHardwareInterface::sync_gravity_parameters()
{
    if (!internal_node_)
        return;

    double scale = gravity_effort_scale_;
    internal_node_->get_parameter_or("gravity_effort_scale", scale, scale);
    gravity_effort_scale_ = scale;

    RCLCPP_INFO(rclcpp::get_logger("ArmHW"),
        "[GRAVITY] effort_scale=%.3f", gravity_effort_scale_);
}

// ================================================================
// CAN 反馈读取
// ================================================================

void RealArmHardwareInterface::read_can_feedback()
{
    device_collection_.readFeedback();

    // device_collection_ 只包含 CAN 电机，索引与关节不完全对应
    // 这里按 CAN 电机添加顺序读取（跳过 Mock 关节）
    size_t motor_idx = 0;
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
        if (!use_real_joint_io_[i]) continue;

        auto motor = device_collection_.getMotor(motor_idx++);
        if (motor)
        {
            raw_motor_pos_[i] = motor->get_angle_rad();
            raw_motor_vel_[i] = motor->get_velocity_rad();
            raw_motor_eff_[i] = motor->get_torque_nm();
            hw_states_pos_[i] = raw_motor_pos_[i];
            hw_states_vel_[i] = raw_motor_vel_[i];
            hw_states_eff_[i] = raw_motor_eff_[i];
        }
    }
}

void RealArmHardwareInterface::refresh_feedback_before_enable()
{
    for (int n = 0; n < 3; ++n)
    {
        for (size_t i = 0; i < info_.joints.size(); ++i)
        {
            if (!use_real_joint_io_[i]) continue;
            hw_commands_pos_[i] = hw_states_pos_[i];
            hw_commands_vel_[i] = 0.0;
            hw_commands_eff_[i] = 0.0;
        }

        send_can_commands();
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        read_can_feedback();
        echo_mock_joints(info_);
        apply_j2j3_coupling();
    }
}

void RealArmHardwareInterface::sync_control_targets_to_feedback()
{
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
        hw_commands_pos_[i] = hw_states_pos_[i];
        hw_commands_vel_[i] = 0.0;
        hw_commands_eff_[i] = 0.0;
        if (i < hold_position_target_.size())
            hold_position_target_[i] = hw_states_pos_[i];
    }

    std::vector<double> cur_pos(kJointCount, 0.0);
    for (size_t i = 0; i < std::min(kJointCount, hw_states_pos_.size()); ++i)
    {
        cur_pos[i] = hw_states_pos_[i];
    }

    std::fill(dls_twist_.begin(), dls_twist_.end(), 0.0);
    std::fill(joint_vel_target_.begin(), joint_vel_target_.end(), 0.0);
    dls_twist_countdown_.store(0, std::memory_order_release);
    joint_vel_countdown_.store(0, std::memory_order_release);
    dls_controller_.SyncPositions(cur_pos);
    joint_controller_.SyncPositions(cur_pos);
}

// ================================================================
// 电机控制请求
// ================================================================

void RealArmHardwareInterface::process_motor_requests()
{
    if (enable_requested_.exchange(false))  enable_motors();
    if (disable_requested_.exchange(false)) disable_motors();
    if (hold_requested_.exchange(false))    hold_position();
}

void RealArmHardwareInterface::enable_motors()
{
    refresh_feedback_before_enable();
    sync_control_targets_to_feedback();
    fsm_.onEnable();
    fsm_.update(false, false);

    if (!clear_errors_enable_and_wait())
        return;

    motors_enabled_ = true;
    safe_zero_frames_ = kSafeZeroFrames;
    RCLCPP_INFO(rclcpp::get_logger("ArmHW"),
        "[POWER] 使能，同步当前位置 J1=%.3f J2=%.3f (%d 帧暖启动)",
        hw_states_pos_.size() > 0 ? hw_states_pos_[0] : 0.0,
        hw_states_pos_.size() > 1 ? hw_states_pos_[1] : 0.0,
        kSafeZeroFrames);
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
    bool ok = true;
    std::ostringstream oss;
    size_t motor_idx = 0;
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
        if (!use_real_joint_io_[i]) continue;

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
        ++motor_idx;
    }

    if (detail)
        *detail = oss.str();
    return ok;
}

bool RealArmHardwareInterface::clear_errors_enable_and_wait()
{
    static constexpr int kMaxEnableAttempts = 8;
    static constexpr int kFeedbackPollsPerAttempt = 10;
    static constexpr auto kPollSleep = std::chrono::milliseconds(2);
    static constexpr auto kCommandGap = std::chrono::milliseconds(10);

    for (int attempt = 1; attempt <= kMaxEnableAttempts; ++attempt)
    {
        const auto counts_before = real_motor_feedback_counts();

        device_collection_.clearAllErrors();
        std::this_thread::sleep_for(kCommandGap);
        device_collection_.enableAll();

        for (int poll = 0; poll < kFeedbackPollsPerAttempt; ++poll)
        {
            std::this_thread::sleep_for(kPollSleep);
            read_can_feedback();
            echo_mock_joints(info_);
            apply_j2j3_coupling();

            std::string detail;
            if (all_real_motors_feedback_ok(counts_before, &detail))
            {
                RCLCPP_INFO(rclcpp::get_logger("ArmHW"),
                    "[POWER] 清错+使能成功 (attempt=%d)", attempt);
                return true;
            }
        }

        std::string detail;
        all_real_motors_feedback_ok(counts_before, &detail);
        RCLCPP_WARN(rclcpp::get_logger("ArmHW"),
            "[POWER] 清错+使能后仍未确认全部电机正常 (attempt=%d/%d): %s",
            attempt, kMaxEnableAttempts, detail.c_str());
    }

    RCLCPP_ERROR(rclcpp::get_logger("ArmHW"),
        "[POWER] 多次清错+使能失败，保持未使能状态");
    device_collection_.disableAll();
    return false;
}

void RealArmHardwareInterface::disable_motors()
{
    fsm_.onDisable();
    fsm_.update(false, false);
    sync_control_targets_to_feedback();
    motors_enabled_ = false;
    device_collection_.disableAll();
    RCLCPP_INFO(rclcpp::get_logger("ArmHW"), "[POWER] 失能");
}

void RealArmHardwareInterface::hold_position()
{
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
        hw_commands_pos_[i] = hw_states_pos_[i];
        hw_commands_vel_[i] = 0.0;
        if (i < hold_position_target_.size())
            hold_position_target_[i] = hw_states_pos_[i];
    }
    safe_zero_frames_ = 0;
    RCLCPP_INFO(rclcpp::get_logger("ArmHW"), "[HOLD] 位置锁定");
}

// ================================================================
// CAN 指令发送
// ================================================================

void RealArmHardwareInterface::send_can_commands()
{
    std::vector<double> grav(kJointCount, 0.0);
    const auto gravity_mode = gravity_compensator_.mode();
    const bool gravity_ready =
        motors_enabled_ && gravity_compensator_.is_initialized() && gravity_effort_scale_ > 0.0;
    const bool gravity_only =
        gravity_mode == GravityCompensator::Mode::GravityOnly && gravity_ready;
    const bool gravity_assist =
        gravity_mode == GravityCompensator::Mode::Assist && gravity_ready;

    if ((gravity_only || gravity_assist) && gravity_compensator_.is_initialized())
    {
        grav = gravity_compensator_.compute(hw_states_pos_);
        for (double& tau : grav)
            tau *= gravity_effort_scale_;
        if (kJ3Index < grav.size())
            grav[kJ3Index] *= j3_gravity_effort_scale_;
    }
    else if (gravity_mode == GravityCompensator::Mode::GravityOnly &&
             motors_enabled_ &&
             !gravity_ready)
    {
        RCLCPP_WARN_ONCE(rclcpp::get_logger("ArmHW"),
            "[GRAVITY] gravity_only 已请求，但模型未就绪或 gravity_effort_scale<=0，保持普通控制输出");
    }

    // 构建电机级命令（仅 CAN 电机，不含 Mock 关节）
    size_t n_motors = device_collection_.size();
    std::vector<double> cmd_pos(n_motors, 0.0);
    std::vector<double> cmd_vel(n_motors, 0.0);
    std::vector<double> cmd_eff(n_motors, 0.0);
    std::vector<double> cmd_kp;
    std::vector<double> cmd_kd;
    cmd_kp.reserve(n_motors);
    cmd_kd.reserve(n_motors);

    size_t motor_idx = 0;
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
        if (!use_real_joint_io_[i]) continue;
        const bool arm_gravity_only = gravity_only && i < kJointCount;
        double pos = arm_gravity_only ? hw_states_pos_[i] : hw_commands_pos_[i];
        double vel = arm_gravity_only ? 0.0 : hw_commands_vel_[i];
        double eff = (arm_gravity_only ? 0.0 : hw_commands_eff_[i]) +
            (i < grav.size() ? grav[i] : 0.0);
        if (i == kJ3Index && use_real_joint_io_[kJ2Index])
        {
            const double j3_scale = std::abs(j2j3_j3_scale_) > 1e-9 ? j2j3_j3_scale_ : 1.0;
            const double j2_pos = arm_gravity_only ? hw_states_pos_[kJ2Index] : hw_commands_pos_[kJ2Index];
            const double j2_vel = arm_gravity_only ? 0.0 : hw_commands_vel_[kJ2Index];
            const double correction = j2j3_poly_correction(j2_pos);
            const double derivative = j2j3_poly_derivative(j2_pos);
            if (j2j3_scale_mode_is_multiply())
            {
                pos = (pos - correction) / j3_scale;
                vel = (vel - derivative * j2_vel) / j3_scale;
                eff *= j3_scale;
            }
            else
            {
                pos = j3_scale * pos - correction;
                vel = j3_scale * vel - derivative * j2_vel;
                eff /= j3_scale;
            }
        }
        if (i == kJ2Index && use_real_joint_io_[kJ3Index])
        {
            const double j3_scale = std::abs(j2j3_j3_scale_) > 1e-9 ? j2j3_j3_scale_ : 1.0;
            const double j3_eff =
                (arm_gravity_only ? 0.0 : hw_commands_eff_[kJ3Index]) +
                (kJ3Index < grav.size() ? grav[kJ3Index] : 0.0);
            const double j2_pos = arm_gravity_only ? hw_states_pos_[kJ2Index] : hw_commands_pos_[kJ2Index];
            const double derivative = j2j3_poly_derivative(j2_pos);
            eff += (j2j3_scale_mode_is_multiply() ? derivative : derivative / j3_scale) * j3_eff;
        }
        cmd_pos[motor_idx] = pos;
        cmd_vel[motor_idx] = vel;
        cmd_eff[motor_idx] = eff;
        cmd_kp.push_back(arm_gravity_only ? 0.0 : device_collection_.getMotor(motor_idx)->get_kp());
        cmd_kd.push_back(arm_gravity_only ? 0.0 : device_collection_.getMotor(motor_idx)->get_kd());

        if (info_.joints[i].name == "joint_right_finger")
        {
            static size_t gripper_log_counter = 0;
            if (++gripper_log_counter % 250 == 0)
            {
                RCLCPP_INFO(rclcpp::get_logger("ArmHW"),
                    "[GRIPPER-HW] target=%.4f rad feedback=%.4f rad raw=%.4f rad vel_cmd=%.4f rad/s "
                    "motor_cmd=%.4f rad mode=%s kp=%.2f kd=%.2f",
                    hw_commands_pos_[i], hw_states_pos_[i], raw_motor_pos_[i], hw_commands_vel_[i],
                    pos, gravity_only ? "gravity_only" : "normal",
                    cmd_kp.back(), cmd_kd.back());
            }
        }
        motor_idx++;
    }

    device_collection_.sendCommandsWithGains(cmd_pos, cmd_vel, cmd_eff, cmd_kp, cmd_kd);
}

}  // namespace arm_hardware_interface

PLUGINLIB_EXPORT_CLASS(
    arm_hardware_interface::RealArmHardwareInterface,
    hardware_interface::SystemInterface)
