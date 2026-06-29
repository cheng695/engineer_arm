#include "arm_hardware_interface/joint_driver.hpp"
#include "arm_hardware_interface/remote.hpp"

#include <algorithm>
#include <cmath>

namespace arm_hardware_interface
{

const std::array<std::string, JointDriver::kJointCount> JointDriver::kJointNames = {
    "joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"
};

JointDriver::JointDriver(rclcpp::Node::SharedPtr node, bool use_position_mode)
    : use_position_mode_(use_position_mode)
{
    if (use_position_mode_)
    {
        // 位置模式：发 JointTrajectory 到 arm_controller 的 topic 接口
        traj_pub_ = node->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/arm_controller/joint_trajectory", 10);
    }
    else
    {
        // 速度模式：发速度到 ForwardCommandController
        vel_pub_ = node->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/forward_velocity_controller/commands", 10);
    }
}

void JointDriver::init_positions(const std::vector<double>& current_positions)
{
    for (size_t i = 0; i < std::min(current_positions.size(), kJointCount); ++i)
        integrated_pos_[i] = current_positions[i];
    positions_initialized_ = true;
}

void JointDriver::drive(const remote::Remote& remote)
{
    std::array<double, kJointCount> vels{};
    vels[0] = remote.j1() * gain_;
    vels[1] = remote.j2() * gain_;
    vels[2] = remote.j3() * gain_;
    vels[3] = remote.j4() * gain_;
    vels[4] = remote.j5() * gain_;
    vels[5] = remote.j6() * gain_;
    vels[6] = remote.j7() * gain_;

    if (use_position_mode_)
        publish_position(vels, dt_);
    else
        publish_velocity(vels);
}

void JointDriver::drive(const std::array<double, kJointCount>& velocities)
{
    std::array<double, kJointCount> vels{};
    for (size_t i = 0; i < kJointCount; ++i)
        vels[i] = velocities[i] * gain_;

    if (use_position_mode_)
        publish_position(vels, dt_);
    else
        publish_velocity(vels);
}

void JointDriver::stop()
{
    if (use_position_mode_)
    {
        // 发送当前位置作为保持轨迹
        if (!positions_initialized_) return;
        auto msg = std::make_unique<trajectory_msgs::msg::JointTrajectory>();
        msg->header.stamp = rclcpp::Clock().now();
        msg->header.frame_id = "";
        msg->joint_names.assign(kJointNames.begin(), kJointNames.end());
        trajectory_msgs::msg::JointTrajectoryPoint pt;
        pt.positions.assign(integrated_pos_.begin(), integrated_pos_.end());
        pt.velocities.assign(kJointCount, 0.0);
        pt.time_from_start = rclcpp::Duration::from_seconds(0.02);
        msg->points.push_back(pt);
        traj_pub_->publish(std::move(msg));
    }
    else
    {
        auto msg = std::make_unique<std_msgs::msg::Float64MultiArray>();
        msg->data.assign(kJointCount, 0.0);
        vel_pub_->publish(std::move(msg));
    }
}

void JointDriver::set_gain(double gain) { gain_ = gain; }
void JointDriver::set_dt(double dt) { dt_ = dt; }

void JointDriver::publish_velocity(const std::array<double, kJointCount>& velocities)
{
    auto msg = std::make_unique<std_msgs::msg::Float64MultiArray>();
    msg->data.resize(kJointCount);
    for (size_t i = 0; i < kJointCount; ++i)
        msg->data[i] = velocities[i];
    vel_pub_->publish(std::move(msg));
}

void JointDriver::publish_position(const std::array<double, kJointCount>& velocities,
                                    double dt)
{
    if (!positions_initialized_)
        return;

    // 积分
    for (size_t i = 0; i < kJointCount; ++i)
    {
        integrated_pos_[i] += velocities[i] * dt;
        integrated_pos_[i] = std::clamp(integrated_pos_[i],
                                        -kDefaultPosLimit, kDefaultPosLimit);
    }

    // 构建单点 JointTrajectory
    auto msg = std::make_unique<trajectory_msgs::msg::JointTrajectory>();
    msg->header.stamp = rclcpp::Clock().now();
    msg->header.frame_id = "";
    msg->joint_names.assign(kJointNames.begin(), kJointNames.end());

    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions.assign(integrated_pos_.begin(), integrated_pos_.end());
    pt.velocities.assign(velocities.begin(), velocities.end());
    pt.time_from_start = rclcpp::Duration::from_seconds(dt);
    msg->points.push_back(pt);

    traj_pub_->publish(std::move(msg));
}

}  // namespace arm_hardware_interface
