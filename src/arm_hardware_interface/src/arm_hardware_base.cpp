#include "arm_hardware_interface/arm_hardware_base.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <set>
#include <sstream>
#include <thread>

namespace arm_hardware_interface
{

namespace
{
constexpr double kDefaultLowerLimit = -std::numeric_limits<double>::infinity();
constexpr double kDefaultUpperLimit =  std::numeric_limits<double>::infinity();
}  // namespace

// ================================================================
// 初始化辅助
// ================================================================

void ArmHardwareBase::init_joint_buffers(const hardware_interface::HardwareInfo& info)
{
    const size_t n = info.joints.size();
    hw_states_pos_.resize(n, 0.0);
    hw_states_vel_.resize(n, 0.0);
    hw_states_eff_.resize(n, 0.0);
    hw_commands_pos_.resize(n, 0.0);
    hw_commands_vel_.resize(n, 0.0);
    hw_commands_eff_.resize(n, 0.0);
    hw_sim_pos_.resize(n, 0.0);
    hw_sim_vel_.resize(n, 0.0);
    use_real_joint_io_.resize(n, true);
}

void ArmHardwareBase::init_joint_limits(const hardware_interface::HardwareInfo& info)
{
    for (const auto& j : info.joints)
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

void ArmHardwareBase::init_mock_joints(const hardware_interface::HardwareInfo& info)
{
    for (size_t i = 0; i < info.joints.size(); ++i)
    {
        if (info.joints[i].parameters.find("can_id") == info.joints[i].parameters.end())
            use_real_joint_io_[i] = false;
    }
}

bool ArmHardwareBase::init_gravity_compensator(const hardware_interface::HardwareInfo& info)
{
    auto it = info.hardware_parameters.find("robot_description");
    if (it == info.hardware_parameters.end())
        return false;

    const auto& value = it->second;
    RCLCPP_INFO(rclcpp::get_logger("ArmHW"), "robot_description (%zu 字符)", value.size());

    if (value.size() < 500 && (value[0] == '/' || value.find(".urdf") != std::string::npos))
        return gravity_compensator_.initialize_from_file(value, rclcpp::get_logger("ArmHW"));
    else
        return gravity_compensator_.initialize(value, rclcpp::get_logger("ArmHW"));
}

void ArmHardwareBase::init_dls()
{
    if (gravity_compensator_.is_initialized())
    {
        dls_controller_.Init(gravity_compensator_.model(), "tool_link", 0.002);
        RCLCPP_INFO(rclcpp::get_logger("ArmHW"), "DLS 控制器已初始化");
    }
}

void ArmHardwareBase::init_joint_controller(const hardware_interface::HardwareInfo& info)
{
    (void)info;
    joint_controller_.Init(kJointCount, 0.002);
    joint_vel_target_.resize(kJointCount, 0.0);
    joint_pos_target_.resize(kJointCount, 0.0);
    RCLCPP_INFO(rclcpp::get_logger("ArmHW"), "JointController 已初始化");
}

// ================================================================
// 内部 ROS 节点
// ================================================================

void ArmHardwareBase::setup_internal_node(const std::string& node_name)
{
    internal_node_ = rclcpp::Node::make_shared(node_name);

    enable_sub_ = internal_node_->create_subscription<std_msgs::msg::Bool>(
        "/arm_motor_enable", 10,
        [this](const std_msgs::msg::Bool::SharedPtr m) {
            if (m->data) { enable_requested_ = true; fsm_.onEnable(); }
            else         { disable_requested_ = true; fsm_.onDisable(); }
        });

    hold_sub_ = internal_node_->create_subscription<std_msgs::msg::Bool>(
        "/arm_hold_position", 10,
        [this](const std_msgs::msg::Bool::SharedPtr m) {
            if (m->data) hold_requested_ = true;
        });

    dls_sub_ = internal_node_->create_subscription<geometry_msgs::msg::TwistStamped>(
        "/dls/twist_cmds", 10,
        [this](const geometry_msgs::msg::TwistStamped::SharedPtr m) {
            dls_twist_ = {m->twist.linear.x, m->twist.linear.y, m->twist.linear.z,
                           m->twist.angular.x, m->twist.angular.y, m->twist.angular.z};
            dls_twist_countdown_.store(kDlsTimeout, std::memory_order_release);
            fsm_.onTwist();
        });

    joint_vel_sub_ = internal_node_->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/joint_vel_cmds", 10,
        [this](const std_msgs::msg::Float64MultiArray::SharedPtr m) {
            if (m->data.size() >= kJointCount) {
                std::copy_n(m->data.begin(), kJointCount, joint_vel_target_.begin());
                joint_vel_countdown_.store(kDlsTimeout, std::memory_order_release);
                fsm_.onJoint();
            }
        });

    joint_pos_sub_ = internal_node_->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/joint_pos_cmds", 10,
        [this](const std_msgs::msg::Float64MultiArray::SharedPtr m) {
            joint_pos_target_ = m->data;
            joint_pos_pending_.store(true, std::memory_order_release);
        });

    pose_sub_ = internal_node_->create_subscription<std_msgs::msg::Bool>(
        "/pose_active", 10,
        [this](const std_msgs::msg::Bool::SharedPtr m) {
            if (m->data) fsm_.onPoseStart(); else fsm_.onPoseDone();
        });

    motors_enabled_ = false;
    safe_zero_frames_ = 0;

    rclcpp::executors::SingleThreadedExecutor::SharedPtr exec =
        std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    exec->add_node(internal_node_);
    spin_executor_ = exec;
    spin_thread_ = std::make_unique<std::thread>([exec]() {
        exec->spin();
    });
}

void ArmHardwareBase::teardown_internal_node()
{
    if (spin_executor_) {
        spin_executor_->cancel();
    }
    if (spin_thread_) {
        spin_thread_->join();
        spin_thread_.reset();
    }
    spin_executor_.reset();
}

// ================================================================
// read / write 辅助
// ================================================================

void ArmHardwareBase::echo_mock_joints(const hardware_interface::HardwareInfo& info)
{
    for (size_t i = 0; i < info.joints.size(); ++i)
    {
        if (!use_real_joint_io_[i])
        {
            hw_states_pos_[i] = hw_commands_pos_[i];
            hw_states_vel_[i] = hw_commands_vel_[i];
            hw_states_eff_[i] = 0.0;
        }
    }
}

void ArmHardwareBase::apply_j2j3_coupling()
{
    if (use_real_joint_io_[kJ2Index] && use_real_joint_io_[kJ3Index])
    {
        double raw_j3_pos = hw_states_pos_[kJ3Index];
        double raw_j3_vel = hw_states_vel_[kJ3Index];
        hw_states_pos_[kJ3Index] = raw_j3_pos + j2j3_coupling_ * hw_states_pos_[kJ2Index];
        hw_states_vel_[kJ3Index] = raw_j3_vel + j2j3_coupling_ * hw_states_vel_[kJ2Index];
    }
}

void ArmHardwareBase::apply_gravity_to_effort()
{
    if (gravity_compensator_.is_initialized())
    {
        auto tau = gravity_compensator_.compute(hw_states_pos_);
        for (size_t i = 0; i < std::min(tau.size(), hw_states_eff_.size()); ++i)
            hw_states_eff_[i] = tau[i];
    }
}

bool ArmHardwareBase::process_control(const hardware_interface::HardwareInfo& info)
{
    // ---- FSM 更新 ----
    bool dls_active   = dls_twist_countdown_.load(std::memory_order_acquire) > 0;
    bool joint_active = joint_vel_countdown_.load(std::memory_order_acquire) > 0;
    if (dls_active)   dls_twist_countdown_.store(dls_twist_countdown_.load(std::memory_order_acquire) - 1, std::memory_order_release);
    if (joint_active) joint_vel_countdown_.store(joint_vel_countdown_.load(std::memory_order_acquire) - 1, std::memory_order_release);

    fsm_.update(dls_active, joint_active);

    // ---- 状态同步：首次进入时对齐位置 ----
    auto sync_all = [&]() 
    {
        std::vector<double> cur_pos(kJointCount);
        for (size_t i = 0; i < kJointCount; ++i) 
        {
            cur_pos[i] = hw_states_pos_[i];
        }
        dls_controller_.SyncPositions(cur_pos);
        joint_controller_.SyncPositions(cur_pos);
    };

    if (fsm_.justExitedStop()) sync_all();
    if (fsm_.justExitedStop()) sync_all();
    if (fsm_.justEnteredDls()) sync_all();
    if (fsm_.justEnteredJoint()) sync_all();

    // 进 POSE 那一帧：同步到反馈（挡住 arm_controller 的旧 hold），后续帧不插手
    static auto prev_state = ControlFsm::State::STOP;
    auto cur_state = fsm_.state();
    if (cur_state == ControlFsm::State::POSE && prev_state != ControlFsm::State::POSE)
    {
        // 直接用关节反馈值，防止 arm_controller 残留的旧轨迹末端位置导致跳变
        for (size_t i = 0; i < kJointCount; ++i)
            hw_commands_pos_[i] = hw_states_pos_[i];
        dls_twist_countdown_.store(0, std::memory_order_release);
        joint_vel_countdown_.store(0, std::memory_order_release);
        RCLCPP_INFO(rclcpp::get_logger("ArmHW"), "[FSM] → POSE, synced to feedback");
    }
    prev_state = cur_state;

    // ---- 执行控制 ----
    switch (fsm_.state())
    {
    case ControlFsm::State::DLS:
    {
        RCLCPP_INFO_ONCE(rclcpp::get_logger("ArmHW"), "[DLS] 笛卡尔控制");

        auto outputs = dls_controller_.Update(dls_twist_, hw_states_pos_, hw_states_vel_);
        for (size_t i = 0; i < std::min(outputs.size(), info.joints.size()); ++i)
        {
            if (std::isnan(outputs[i].pos) || std::isnan(outputs[i].vel))
            {
                return false;
            }
            hw_commands_pos_[i] = std::clamp(outputs[i].pos, joint_lower_limits_[i], joint_upper_limits_[i]);
            hw_commands_vel_[i] = std::clamp(outputs[i].vel, -30.0, 30.0);
            hw_commands_eff_[i] = outputs[i].tor;
        }
        // 同步关节目标，切回时不跳变
        joint_controller_.SyncPositions(std::vector<double>(hw_commands_pos_.begin(), hw_commands_pos_.begin() + kJointCount));
        return true;
    }

    case ControlFsm::State::JOINT:
    {
        RCLCPP_INFO_ONCE(rclcpp::get_logger("ArmHW"), "[JOINT] 关节控制");

        std::vector<double> pos_ref(kJointCount), vel_target(kJointCount);
        for (size_t i = 0; i < kJointCount; ++i) 
        {
            pos_ref[i] = hw_states_pos_[i];
            vel_target[i] = joint_vel_target_[i];
        }
        auto outputs = joint_controller_.Update(vel_target, pos_ref, joint_lower_limits_, joint_upper_limits_);
        for (size_t i = 0; i < std::min(outputs.size(), kJointCount); ++i) 
        {
            if (std::isnan(outputs[i].pos) || std::isnan(outputs[i].vel)) 
            { 
                return false; 
            }
            hw_commands_pos_[i] = outputs[i].pos;
            hw_commands_vel_[i] = outputs[i].vel;
        }
        return true;
    }

    case ControlFsm::State::IDLE:
        // 每帧同步期望到反馈，确保下次进入控制状态不跳变
        for (size_t i = 0; i < kJointCount; ++i) {
            hw_commands_pos_[i] = hw_states_pos_[i];
            hw_commands_vel_[i] = 0.0;
        }
        sync_all();
        return true;

    case ControlFsm::State::POSE:
        // 收到新的目标位姿：启动插值
        if (joint_pos_pending_.exchange(false, std::memory_order_acquire))
        {
            if (joint_pos_target_.size() >= kJointCount)
            {
                // 从当前反馈位置插值到目标
                pose_interp_start_.resize(kJointCount);
                pose_interp_target_.resize(kJointCount);
                for (size_t i = 0; i < kJointCount; ++i)
                {
                    pose_interp_start_[i]  = hw_states_pos_[i];
                    pose_interp_target_[i] = std::clamp(static_cast<double>(joint_pos_target_[i]),
                                                        joint_lower_limits_[i],
                                                        joint_upper_limits_[i]);
                }
                pose_interp_counter_ = kPoseInterpTotal;
                RCLCPP_INFO(rclcpp::get_logger("ArmHW"),
                    "[POSE] interpolating over %d frames", kPoseInterpTotal);
            }
        }

        // 插值执行中
        if (pose_interp_counter_ > 0)
        {
            double t = 1.0 - static_cast<double>(pose_interp_counter_) / kPoseInterpTotal;
            for (size_t i = 0; i < kJointCount && i < pose_interp_start_.size(); ++i)
            {
                hw_commands_pos_[i] = pose_interp_start_[i] +
                                      t * (pose_interp_target_[i] - pose_interp_start_[i]);
                hw_commands_vel_[i] = 0.0;
            }
            --pose_interp_counter_;
        }
        else if (pose_interp_counter_ == 0 && !pose_interp_start_.empty())
        {
            // 插值完成，hold 在目标
            for (size_t i = 0; i < kJointCount && i < pose_interp_target_.size(); ++i)
            {
                hw_commands_pos_[i] = pose_interp_target_[i];
                hw_commands_vel_[i] = 0.0;
            }
            // 标记完成（只 hold，不重复 log）
            pose_interp_counter_ = -1;
        }
        else
        {
            // 还没收到目标位姿：保持当前位置，防止 arm_controller
            // (joint_trajectory_controller) 在上一个轨迹终点持续写入导致跳变
            for (size_t i = 0; i < kJointCount; ++i) {
                hw_commands_pos_[i] = hw_states_pos_[i];
                hw_commands_vel_[i] = 0.0;
            }
        }
        return false;

    case ControlFsm::State::STOP:
        // STOP：持续同步指令到反馈位置
        for (size_t i = 0; i < kJointCount; ++i) 
        {
            hw_commands_pos_[i] = hw_states_pos_[i];
            hw_commands_vel_[i] = 0.0;
        }
        return false;
    }
}

}  // namespace arm_hardware_interface
