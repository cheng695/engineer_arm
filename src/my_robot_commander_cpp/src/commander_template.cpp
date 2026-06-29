/**
 * joy_to_servo — 摇杆转 Servo 转发器（内嵌 Servo 实例）
 *
 * joy → Remote 解析 → TwistStamped/JointJog → MoveIt Servo → arm_controller
 *
 * Servo 在此节点内，MoveGroupInterface 不在此节点。
 * 固定点位规划使用 RViz 的 MotionPlanning 面板。
 * 默认笛卡尔模式。
 */
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <memory>
#include <string>

#include <moveit_servo/servo.h>
#include <moveit_servo/servo_parameters.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include "arm_hardware_interface/remote.hpp"

using Joy = sensor_msgs::msg::Joy;
using TwistStamped = geometry_msgs::msg::TwistStamped;
using JointJog = control_msgs::msg::JointJog;

class JoyToServo
{
public:
    explicit JoyToServo(const rclcpp::NodeOptions& options)
    {
        node_ = rclcpp::Node::make_shared("joy_to_servo", options);

        // ---- Servo ----
        RCLCPP_INFO(node_->get_logger(), "[SERVO] Loading...");
        auto servo_params = moveit_servo::ServoParameters::makeServoParameters(node_);
        if (!servo_params)
        {
            RCLCPP_FATAL(node_->get_logger(), "[SERVO] Failed to load parameters");
            throw std::runtime_error("Servo init failed");
        }

        planning_scene_monitor_ =
            std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
                node_, "robot_description");
        if (planning_scene_monitor_->getPlanningScene())
        {
            planning_scene_monitor_->startStateMonitor(servo_params->joint_topic);
            planning_scene_monitor_->startSceneMonitor(
                servo_params->monitored_planning_scene_topic);
            planning_scene_monitor_->setPlanningScenePublishingFrequency(25);
            planning_scene_monitor_->getStateMonitor()->enableCopyDynamics(true);
        }

        servo_ = std::make_unique<moveit_servo::Servo>(
            node_, servo_params, planning_scene_monitor_);
        servo_->start();
        RCLCPP_INFO(node_->get_logger(), "[SERVO] Started");

        command_frame_id_ = servo_params->robot_link_command_frame;

        twist_pub_ = node_->create_publisher<TwistStamped>(
            servo_params->cartesian_command_in_topic, 10);
        joint_pub_ = node_->create_publisher<JointJog>(
            servo_params->joint_command_in_topic, 10);

        RCLCPP_INFO(node_->get_logger(), "[BOOT] joy_to_servo 启动完成");
        RCLCPP_INFO(node_->get_logger(), "[MODE] 默认笛卡尔 | R3→关节 | RT→暂停");

        joy_sub_ = node_->create_subscription<Joy>(
            "/joy", 10, [this](const Joy::SharedPtr m) { joyCallback(m); });
    }

    rclcpp::Node::SharedPtr getNode() { return node_; }

private:
    void joyCallback(const Joy::SharedPtr msg)
    {
        if (!remote_.update(*msg)) return;
        RCLCPP_INFO_ONCE(node_->get_logger(), "Received first Joy message");

        if (remote_.joint())     joint_mode_ = true;
        if (remote_.cartesian()) joint_mode_ = false;
        if (remote_.stop())      { servo_enabled_ = false; publishStop(); return; }
        if (remote_.continue_()) { servo_enabled_ = true;  return; }
        if (remote_.paused())    { publishStop(); return; }
        if (!servo_enabled_) return;

        if (joint_mode_)  publishJointJog();
        else              publishTwist();
    }

    void publishJointJog()
    {
        auto msg = std::make_unique<JointJog>();
        msg->header.stamp = node_->now();
        msg->header.frame_id = command_frame_id_;
        auto add = [&](const std::string& n, double v) {
            if (std::abs(v) > 0.05) { msg->joint_names.push_back(n); msg->velocities.push_back(v * 3.0); }
        };
        add("joint1", remote_.j1()); add("joint2", remote_.j2());
        add("joint3", remote_.j3()); add("joint4", remote_.j4());
        add("joint5", remote_.j5()); add("joint6", remote_.j6());
        add("joint7", remote_.j7());
        if (!msg->joint_names.empty()) joint_pub_->publish(std::move(msg));
    }

    void publishTwist()
    {
        auto msg = std::make_unique<TwistStamped>();
        msg->header.stamp = node_->now();
        msg->header.frame_id = command_frame_id_;
        msg->twist.linear.x  = remote_.x()     * 0.5;
        msg->twist.linear.y  = remote_.y()     * 0.5;
        msg->twist.linear.z  = remote_.z()     * 0.5;
        msg->twist.angular.x = remote_.roll()  * 2.0;
        msg->twist.angular.y = remote_.pitch() * 2.0;
        msg->twist.angular.z = remote_.yaw()   * 2.0;
        twist_pub_->publish(std::move(msg));
    }

    void publishStop()
    {
        auto msg = std::make_unique<TwistStamped>();
        msg->header.stamp = node_->now();
        msg->header.frame_id = command_frame_id_;
        twist_pub_->publish(std::move(msg));
    }

    rclcpp::Node::SharedPtr node_;
    remote::Remote remote_;
    std::string command_frame_id_ = "base_link";
    std::unique_ptr<moveit_servo::Servo> servo_;
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
    rclcpp::Publisher<TwistStamped>::SharedPtr twist_pub_;
    rclcpp::Publisher<JointJog>::SharedPtr joint_pub_;
    rclcpp::Subscription<Joy>::SharedPtr joy_sub_;
    std::atomic<bool> servo_enabled_{true};
    bool joint_mode_{false};
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto node = std::make_shared<JoyToServo>(options);
    rclcpp::spin(node->getNode());
    rclcpp::shutdown();
    return 0;
}
