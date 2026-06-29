#pragma once

#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace my_robot_commander_cpp
{

/**
 * @brief MoveIt 固定点位规划器。
 *
 * 封装 MoveGroupInterface，提供规划并异步执行命名目标的能力。
 * 支持机械臂和夹爪两个规划组，执行期间 is_busy() 返回 true。
 */
class MoveItPlanner
{
public:
    explicit MoveItPlanner(rclcpp::Node::SharedPtr node);
    ~MoveItPlanner();

    // ---- 机械臂命名目标 ----
    void go_home();
    void go_right();
    void go_left();
    void go_up();
    void go_through();

    // ---- 夹爪 ----
    void open_gripper();
    void close_gripper();

    // ---- 速度/加速度缩放 ----
    void set_velocity_scaling(double factor);
    void set_acceleration_scaling(double factor);

    // ---- 急停 & 状态 ----
    void stop();
    bool is_busy() const { return is_busy_; }

    /// @brief 获取当前关节角度（用于 IK 求解）
    std::vector<double> get_joint_values() const;

private:
    void plan_and_execute(
        const std::shared_ptr<moveit::planning_interface::MoveGroupInterface>& group,
        const std::string& target);

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_;
    std::atomic<bool> is_busy_{false};
    std::atomic<bool> shutting_down_{false};
    std::jthread planning_thread_;
};

}  // namespace my_robot_commander_cpp
