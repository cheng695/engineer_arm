#pragma once

#include <array>
#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/hardware_info.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

#include "arm_hardware_interface/control_fsm.hpp"
#include "arm_hardware_interface/dls_controller.hpp"
#include "arm_hardware_interface/joint_controller.hpp"
#include "arm_hardware_interface/gravity_compensator.hpp"

namespace arm_hardware_interface
{

/**
 * @brief 共享工具基类 — 存放 Real/Mock 两个硬件接口插件共用的数据与辅助方法。
 *
 * 本类 **不** 继承 hardware_interface::SystemInterface，
 * 子类需要同时继承 SystemInterface 和本类。
 */
class ArmHardwareBase
{
public:
    ArmHardwareBase() = default;
    virtual ~ArmHardwareBase() = default;

    // 禁止拷贝
    ArmHardwareBase(const ArmHardwareBase&) = delete;
    ArmHardwareBase& operator=(const ArmHardwareBase&) = delete;

protected:
    // ================================================================
    // 初始化辅助（由子类 on_init 调用）
    // ================================================================

    /// 根据 HardwareInfo 分配关节缓冲区并清零
    void init_joint_buffers(const hardware_interface::HardwareInfo& info);

    /// 从关节参数中解析 lower_limit / upper_limit
    void init_joint_limits(const hardware_interface::HardwareInfo& info);

    /// 将没有 can_id 参数的关节标记为 Mock（use_real_joint_io_[i]=false）
    void init_mock_joints(const hardware_interface::HardwareInfo& info);

    /// 从 hardware_parameters["robot_description"] 加载 Pinocchio 模型
    bool init_gravity_compensator(const hardware_interface::HardwareInfo& info);

    /// 初始化控制器
    void init_dls();
    void init_joint_controller(const hardware_interface::HardwareInfo& info);

    // ================================================================
    // 内部 ROS 节点（由子类 on_activate / on_deactivate 调用）
    // ================================================================

    /// 创建内部节点 + 订阅 + 启动独立 spin 线程
    void setup_internal_node(const std::string& node_name);

    /// 停止 spin 线程并释放资源
    void teardown_internal_node();

    // ================================================================
    // read / write 辅助
    // ================================================================

    /// 对 use_real_joint_io_[i]==false 的关节点：hw_command → hw_state 回显
    void echo_mock_joints(const hardware_interface::HardwareInfo& info);

    /// 对 J2/J3 状态施加同步带耦合解耦
    void apply_j2j3_coupling();

    /// 计算重力力矩并覆盖 hw_states_eff_（前提：gravity_compensator_ 已初始化）
    void apply_gravity_to_effort();

    /// 统一控制入口：FSM 决定走哪个控制器
    /// @return true 表示本周期由控制器接管
    bool process_control(const hardware_interface::HardwareInfo& info);

    // ================================================================
    // 常量
    // ================================================================
    static constexpr size_t kJointCount = 7;
    static constexpr size_t kJ2Index = 1;
    static constexpr size_t kJ3Index = 2;
    static constexpr int    kSafeZeroFrames = 50;
    static constexpr int    kDlsTimeout = 50;

    // ================================================================
    // 关节缓冲（command / state 接口指向这些内存）
    // ================================================================
    std::vector<double> hw_states_pos_;
    std::vector<double> hw_states_vel_;
    std::vector<double> hw_states_eff_;
    std::vector<double> hw_commands_pos_;
    std::vector<double> hw_commands_vel_;
    std::vector<double> hw_commands_eff_;

    /// 仿真专用：process_control() 输出后的位置快照，read() 从这里回显
    /// 隔离 arm_controller 在 update() 时对 hw_commands_pos_ 的覆盖
    std::vector<double> hw_sim_pos_;
    std::vector<double> hw_sim_vel_;

    std::vector<double> joint_lower_limits_;
    std::vector<double> joint_upper_limits_;

    /// 标记哪些关节走真实 I/O（CAN 读取/发送），其余回显
    std::vector<bool> use_real_joint_io_;

    // ================================================================
    // 硬件参数
    // ================================================================
    double j2j3_coupling_ = 0.986;

    // ================================================================
    // 重力补偿
    // ================================================================
    GravityCompensator gravity_compensator_;

    // ================================================================
    // 控制状态机 + 控制器
    // ================================================================
    ControlFsm fsm_;
    DlsController dls_controller_;
    JointController joint_controller_;
    std::array<double, 6> dls_twist_{};
    std::vector<double> joint_vel_target_;
    std::vector<double> joint_pos_target_;
    std::atomic<int> dls_twist_countdown_{0};
    std::atomic<int> joint_vel_countdown_{0};
    std::atomic<bool> joint_pos_pending_{false};

    // ================================================================
    // 电机使能 / 保持（原子标志，由内部 ROS 订阅回调设置）
    // ================================================================
    bool motors_enabled_{false};
    std::atomic<bool> enable_requested_{false};
    std::atomic<bool> disable_requested_{false};
    std::atomic<bool> hold_requested_{false};

    // ================================================================
    // 安全零速帧 & 速度模式
    // ================================================================
    int safe_zero_frames_{0};
    std::vector<double> integrated_pos_;
    bool vel_mode_active_{false};
    int pose_guard_count_{0};
    bool pose_waiting_for_current_command_{false};
    ControlFsm::State previous_control_state_{ControlFsm::State::STOP};

    // ================================================================
    // 内部 ROS 节点（独立 spin 线程，不触碰实时循环）
    // ================================================================
    rclcpp::Node::SharedPtr internal_node_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr hold_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr dls_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joint_vel_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joint_pos_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pose_sub_;
    rclcpp::executors::SingleThreadedExecutor::SharedPtr spin_executor_;
    std::unique_ptr<std::thread> spin_thread_;
};

}  // namespace arm_hardware_interface
