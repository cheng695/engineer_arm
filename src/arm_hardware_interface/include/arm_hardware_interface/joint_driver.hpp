#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <array>
#include <vector>
#include <string>

namespace remote { class Remote; }

namespace arm_hardware_interface
{

/**
 * @brief 关节空间驱动。
 *
 * 两种模式：
 * - 速度模式（真机）：发布 Float64MultiArray 速度到
 *   /forward_velocity_controller/commands，由硬件接口做积分
 * - 位置模式（仿真）：内部做速度→位置积分，发布 JointTrajectory 到
 *   /arm_controller/joint_trajectory
 */
class JointDriver
{
public:
    static constexpr size_t kJointCount = 7;

    /**
     * @param[in] node ROS 节点
     * @param[in] use_position_mode true=位置模式（仿真），false=速度模式（真机）
     */
    explicit JointDriver(rclcpp::Node::SharedPtr node, bool use_position_mode = false);

    /// @brief 初始化积分位置（位置模式时调用）
    void init_positions(const std::vector<double>& current_positions);

    /// @brief 从 Remote 对象驱动
    void drive(const remote::Remote& remote);

    /// @brief 用 7 个关节速度驱动
    void drive(const std::array<double, kJointCount>& velocities);

    /// @brief 停止（零指令）
    void stop();

    /// @brief 设置速度倍率
    void set_gain(double gain);

    /// @brief 设置积分步长（位置模式）
    void set_dt(double dt);

private:
    void publish_velocity(const std::array<double, kJointCount>& velocities);
    void publish_position(const std::array<double, kJointCount>& velocities, double dt);

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr vel_pub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;
    double gain_{5.0};
    double dt_{0.02};

    bool use_position_mode_{false};
    std::array<double, kJointCount> integrated_pos_{};
    bool positions_initialized_{false};
    uint32_t traj_seq_{0};

    static constexpr double kDefaultPosLimit = 3.14;
    static const std::array<std::string, kJointCount> kJointNames;
};

}  // namespace arm_hardware_interface
