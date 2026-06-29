#include "my_robot_commander_cpp/moveit_planner.hpp"
#include <thread>

namespace my_robot_commander_cpp
{

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

MoveItPlanner::MoveItPlanner(rclcpp::Node::SharedPtr node)
{
    arm_ = std::make_shared<MoveGroupInterface>(node, "arm");
    gripper_ = std::make_shared<MoveGroupInterface>(node, "gripper");
    set_velocity_scaling(1.0);
    set_acceleration_scaling(0.5);
}

MoveItPlanner::~MoveItPlanner()
{
    shutting_down_ = true;
    // std::jthread 在析构时自动 request_stop + join，安全等待线程退出
}

void MoveItPlanner::go_home()    { plan_and_execute(arm_, "home"); }
void MoveItPlanner::go_right()   { plan_and_execute(arm_, "right"); }
void MoveItPlanner::go_left()    { plan_and_execute(arm_, "left"); }
void MoveItPlanner::go_up()      { plan_and_execute(arm_, "up"); }
void MoveItPlanner::go_through() { plan_and_execute(arm_, "through"); }
void MoveItPlanner::open_gripper()  { plan_and_execute(gripper_, "open"); }
void MoveItPlanner::close_gripper() { plan_and_execute(gripper_, "close"); }

void MoveItPlanner::set_velocity_scaling(double factor)
{
    arm_->setMaxVelocityScalingFactor(factor);
    gripper_->setMaxVelocityScalingFactor(factor);
}

void MoveItPlanner::set_acceleration_scaling(double factor)
{
    arm_->setMaxAccelerationScalingFactor(factor);
    gripper_->setMaxAccelerationScalingFactor(factor);
}

void MoveItPlanner::stop()
{
    arm_->stop();
    gripper_->stop();
}

void MoveItPlanner::plan_and_execute(
    const std::shared_ptr<MoveGroupInterface>& group,
    const std::string& target)
{
    group->setStartStateToCurrentState();
    group->setNamedTarget(target);

    MoveGroupInterface::Plan plan;
    if (group->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_WARN(rclcpp::get_logger("MoveItPlanner"),
            "规划失败: %s", target.c_str());
        return;
    }

    is_busy_ = true;

    // std::jthread 在析构时自动 join，替代 v9.0 的 detach（避免节点关闭时崩溃）
    planning_thread_ = std::jthread([this, group, plan]()
    {
        if (shutting_down_) { is_busy_ = false; return; }

        auto result = group->execute(plan);
        if (result == moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_INFO(rclcpp::get_logger("MoveItPlanner"), "运动执行成功");
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("MoveItPlanner"),
                "运动中断或失败, code=%d", result.val);
        }
        is_busy_ = false;
    });
}

std::vector<double> MoveItPlanner::get_joint_values() const
{
    return arm_->getCurrentJointValues();
}

}  // namespace my_robot_commander_cpp
