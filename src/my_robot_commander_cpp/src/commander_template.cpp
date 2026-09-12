#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <array>
#include <algorithm>
#include <memory>
#include <string>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include "arm_hardware_interface/remote.hpp"
#include "std_msgs/msg/string.hpp"

using Joy = sensor_msgs::msg::Joy;
using TwistStamped = geometry_msgs::msg::TwistStamped;
using Float64MultiArray = std_msgs::msg::Float64MultiArray;
using NamedTargetCommand = std_msgs::msg::String;
using Bool = std_msgs::msg::Bool;
using JointState = sensor_msgs::msg::JointState;

class TeleopCommandNode
{
public:
    explicit TeleopCommandNode(const rclcpp::NodeOptions& options)
    {
        // 创建节点
        node_ = rclcpp::Node::make_shared("teleop_command_node", options);

        // 节点运行参数声明、读取与校验
        const auto arm_version =
            node_->declare_parameter<std::string>("arm_version", "v1_0");

        gripper_min_angle_ =
            node_->declare_parameter<double>("gripper_min_angle", -1.45);

        gripper_max_angle_ =
            node_->declare_parameter<double>("gripper_max_angle", 0.0);

        gripper_speed_ =
            node_->declare_parameter<double>("gripper_speed", 0.8);

        const auto directions =
            node_->declare_parameter<std::vector<double>>(
                "joint_control_directions",
                {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0});

        if (directions.size() != joint_control_directions_.size())
        {
            RCLCPP_WARN(
                node_->get_logger(),
                "[JOINT] joint_control_directions 大小=%zu, 期望=%zu; "
                "将使用默认方向",
                directions.size(),
                joint_control_directions_.size());
        }
        else
        {
            std::copy(
                directions.begin(),
                directions.end(),
                joint_control_directions_.begin());
        }

        // 创建发布者
        dls_pub_     = node_->create_publisher<TwistStamped>("/arm/command/cartesian_twist", 10);
        joint_pub_   = node_->create_publisher<Float64MultiArray>("/arm/command/joint_velocity", 10);
        name_pub_    = node_->create_publisher<NamedTargetCommand>("/arm/command/named_target", 10);
        gripper_pub_ = node_->create_publisher<Float64MultiArray>("/arm/command/gripper_position", 10);
        motor_enable_pub_ = node_->create_publisher<Bool>("/arm_motor_enable", 10);


        // 订阅信息
        joy_sub_ = node_->create_subscription<Joy>(
            "/joy", 10,
            [this](const Joy::SharedPtr msg) { joyCallback(msg); });
        joint_state_sub_ = node_->create_subscription<JointState>(
            "/joint_states", 10,
            [this](const JointState::SharedPtr msg) { last_joint_state_ = msg; });


        RCLCPP_INFO(node_->get_logger(), "[BOOT] 启动完成 (DLS-only)");
        RCLCPP_INFO(node_->get_logger(),
            "[JOINT] arm_version=%s, directions=[%.0f %.0f %.0f %.0f %.0f %.0f %.0f]",
            arm_version.c_str(),
            joint_control_directions_[0], joint_control_directions_[1], joint_control_directions_[2],
            joint_control_directions_[3], joint_control_directions_[4], joint_control_directions_[5],
            joint_control_directions_[6]);
        RCLCPP_INFO(node_->get_logger(), "[MODE] 默认笛卡尔 | R3→关节 | RT→暂停");
    }

    rclcpp::Node::SharedPtr getNode() { return node_; }

private:
    void joyCallback(const Joy::SharedPtr msg)
    {
        if(!msg)
        {
            return;
        }

        // 手柄信息传递到 Remote
        remote::RawJoystickState input;
        input.axes = msg->axes;
        input.buttons = msg->buttons;

        if (!remote_.update(input))
        {
            RCLCPP_WARN_THROTTLE(
                node_->get_logger(),
                *node_->get_clock(),
                1000,
                "Remote 输入数据长度不足: axes=%zu buttons=%zu",
                input.axes.size(),
                input.buttons.size());
            return;
        }

        // 模式判断
        if (remote_.joint())     joint_mode_ = true;
        if (remote_.cartesian()) joint_mode_ = false;

        // ---- 电机使能/失能 ----
        if (remote_.enable())  { auto m = std::make_unique<Bool>(); m->data = true;  motor_enable_pub_->publish(std::move(m)); }
        if (remote_.disable()) { auto m = std::make_unique<Bool>(); m->data = false; motor_enable_pub_->publish(std::move(m)); }

        // ---- 固定点位 ----
        if (remote_.a_rising()) { publishNamedTarget("home");  return; }
        if (remote_.b_rising()) { publishNamedTarget("right"); return; }
        if (remote_.y_rising()) { publishNamedTarget("up");    return; }
        if (remote_.x_rising()) { publishNamedTarget("left");  return; }

        // ---- 夹爪控制 ----
        publishGripper();

        // 关节与笛卡尔指令
        if (joint_mode_)
        {
            publishJoint();
        }
        else
        {
            publishCartesian();
        }
    }
    void publishCartesian()
    {
        auto msg = std::make_unique<TwistStamped>();
        msg->header.stamp = node_->now();

        msg->twist.linear.x  = -remote_.x()    * 0.3;
        msg->twist.linear.y  = -remote_.y()    * 0.3;
        msg->twist.linear.z  = remote_.z()     * 0.3;
        msg->twist.angular.x = remote_.roll()  * 1.0;
        msg->twist.angular.y = remote_.pitch() * 1.0;
        msg->twist.angular.z = -remote_.yaw()  * 1.0;

        // 始终发 DLS
        auto dls_msg = std::make_unique<TwistStamped>(*msg);
        dls_pub_->publish(std::move(dls_msg));
    }

    void publishJoint()
    {
        const std::array<double, 7> joint_cmds = {
            joint_control_directions_[0] * remote_.j1(),
            joint_control_directions_[1] * remote_.j2(),
            joint_control_directions_[2] * remote_.j3(),
            joint_control_directions_[3] * remote_.j4(),
            joint_control_directions_[4] * remote_.j5(),
            joint_control_directions_[5] * remote_.j6(),
            joint_control_directions_[6] * remote_.j7()
        };

        auto msg = std::make_unique<Float64MultiArray>();
        constexpr double k = 2.5;
        msg->data = {
            k * joint_cmds[0],
            k * joint_cmds[1],
            k * joint_cmds[2],
            k * joint_cmds[3],
            k * joint_cmds[4],
            k * joint_cmds[5],
            k * joint_cmds[6]
        };
        joint_pub_->publish(std::move(msg));
    }

    void publishGripper()
    {
        const bool open  = remote_.open_gripper();
        const bool close = remote_.close_gripper();
        if (open == close) return;

        if (!gripper_target_initialized_)
        {
            gripper_target_angle_ = currentGripperPosition();
            gripper_target_initialized_ = true;
        }

        constexpr double kJoyPeriod = 0.01;
        const double direction = open ? 1.0 : -1.0;

        gripper_target_angle_ += direction * gripper_speed_ * kJoyPeriod;
        gripper_target_angle_ = std::clamp(gripper_target_angle_, gripper_min_angle_, gripper_max_angle_);

        auto msg = std::make_unique<Float64MultiArray>();
        msg->data = {gripper_target_angle_};
        gripper_pub_->publish(std::move(msg));

        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 500,
            "[GRIPPER-CMD] target=%.4f rad feedback=%.4f rad speed=%.3f rad/s",
            gripper_target_angle_, currentGripperPosition(), gripper_speed_);
    }

    void publishNamedTarget(const std::string& target_name)
    {
        NamedTargetCommand msg;
        msg.data = target_name;
        name_pub_->publish(msg);
    }

    double currentGripperPosition() const
    {
        if (last_joint_state_)
        {
            for (size_t i = 0; i < last_joint_state_->name.size() && i < last_joint_state_->position.size(); ++i)
            {
                if (last_joint_state_->name[i] == "joint_right_finger")
                {
                    return std::clamp(last_joint_state_->position[i], gripper_min_angle_, gripper_max_angle_);
                }
            }
        }
        return gripper_min_angle_;
    }


    rclcpp::Node::SharedPtr node_;
    remote::Remote remote_;
    sensor_msgs::msg::JointState::SharedPtr last_joint_state_;
    std::array<double, 7> joint_control_directions_ = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

    // 发布者
    rclcpp::Publisher<TwistStamped>::SharedPtr dls_pub_;
    rclcpp::Publisher<Float64MultiArray>::SharedPtr joint_pub_;
    rclcpp::Publisher<Float64MultiArray>::SharedPtr gripper_pub_;
    rclcpp::Publisher<NamedTargetCommand>::SharedPtr name_pub_;
    rclcpp::Publisher<Bool>::SharedPtr motor_enable_pub_;

    // 订阅者
    rclcpp::Subscription<Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<JointState>::SharedPtr joint_state_sub_;

    double gripper_min_angle_{-4.8};
    double gripper_max_angle_{0.0};
    double gripper_speed_{0.8};
    double gripper_target_angle_{0.0};
    bool gripper_target_initialized_{false};

    bool joint_mode_{false};
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto node = std::make_shared<TeleopCommandNode>(options);
    rclcpp::spin(node->getNode());
    rclcpp::shutdown();
    return 0;
}
