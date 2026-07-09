/**
 * joy_to_servo — 摇杆遥操作
 *
 * 摇杆 → Remote → DLS (硬件接口) + Servo (可选)
 * 仿真时 use_servo:=false，走纯 DLS 路径。
 */
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <memory>
#include <string>

#include <moveit_servo/servo.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_servo/servo_parameters.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include "arm_hardware_interface/remote.hpp"

using Joy = sensor_msgs::msg::Joy;
using TwistStamped = geometry_msgs::msg::TwistStamped;
using JointJog = control_msgs::msg::JointJog;
using Float64MultiArray = std_msgs::msg::Float64MultiArray;

class JoyToServo
{
public:
    explicit JoyToServo(const rclcpp::NodeOptions& options)
    {
        node_ = rclcpp::Node::make_shared("joy_to_servo", options);

        // 是否启用 Servo（仿真可关闭，走纯 DLS）
        node_->declare_parameter("use_servo", true);
        use_servo_ = node_->get_parameter("use_servo").as_bool();

        command_frame_id_ = "base_link";
        dls_pub_ = node_->create_publisher<TwistStamped>("/dls/twist_cmds", 10);

        // PlanningSceneMonitor：MoveIt 需要它来获取当前位姿（共用）
        planning_scene_monitor_ =
            std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
                node_, "robot_description");
        if (planning_scene_monitor_->getPlanningScene())
        {
            planning_scene_monitor_->startStateMonitor("/joint_states");
            planning_scene_monitor_->startSceneMonitor("/monitored_planning_scene");
            planning_scene_monitor_->setPlanningScenePublishingFrequency(25);
        }

        if (use_servo_)
        {
            // ---- Servo ----
            auto servo_params = moveit_servo::ServoParameters::makeServoParameters(node_);
            if (!servo_params) throw std::runtime_error("Servo init failed");

            servo_ = std::make_unique<moveit_servo::Servo>(node_, servo_params, planning_scene_monitor_);
            servo_->start();
            RCLCPP_INFO(node_->get_logger(), "[SERVO] Started");

            command_frame_id_ = servo_params->robot_link_command_frame;
            twist_pub_ = node_->create_publisher<TwistStamped>(servo_params->cartesian_command_in_topic, 10);
            joint_pub_ = node_->create_publisher<JointJog>(servo_params->joint_command_in_topic, 10);
        }
        else
        {
            // 纯 DLS 模式：关节模式发到 /joint_vel_cmds
            vel_pub_ = node_->create_publisher<Float64MultiArray>("/joint_vel_cmds", 10);
            RCLCPP_INFO(node_->get_logger(), "[DLS-ONLY] 纯DLS模式，不启动Servo");
        }


        // ---- 电机使能/失能 ----
        joint_pos_pub_ = node_->create_publisher<Float64MultiArray>("/joint_pos_cmds", 10);
        motor_enable_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/arm_motor_enable", 10);

        // ---- 固定位姿通知 ----
        pose_active_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/pose_active", 10);

        // ---- MoveGroup（固定点位） ----
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "arm");
        move_group_->setGoalPositionTolerance(0.01);
        move_group_->setGoalOrientationTolerance(0.01);
        RCLCPP_INFO(node_->get_logger(), "[PLAN] MoveGroupInterface 就绪");

        // 缓存最新关节状态（固定位姿规划用）
        joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            [this](const sensor_msgs::msg::JointState::SharedPtr m) { last_joint_state_ = m; });

        joy_sub_ = node_->create_subscription<Joy>(
            "/joy", 10, [this](const Joy::SharedPtr m) { joyCallback(m); });

        RCLCPP_INFO(node_->get_logger(), "[BOOT] 启动完成 (DLS %s Servo)",
            use_servo_ ? "+" : "only, no");
        RCLCPP_INFO(node_->get_logger(), "[MODE] 默认笛卡尔 | R3→关节 | RT→暂停");
    }

    rclcpp::Node::SharedPtr getNode() { return node_; }

private:
    void joyCallback(const Joy::SharedPtr msg)
    {
        if (!remote_.update(*msg)) return;

        if (remote_.joint())     joint_mode_ = true;
        if (remote_.cartesian()) joint_mode_ = false;

        // ---- 电机使能/失能 ----
        if (remote_.enable())  { auto m = std::make_unique<std_msgs::msg::Bool>(); m->data = true;  motor_enable_pub_->publish(std::move(m)); }
        if (remote_.disable()) { auto m = std::make_unique<std_msgs::msg::Bool>(); m->data = false; motor_enable_pub_->publish(std::move(m)); }

        // ---- 固定点位 ----
        if (remote_.a_btn()) { goNamedTarget("home");  return; }
        if (remote_.b_btn()) { goNamedTarget("right"); return; }
        if (remote_.y_btn()) { goNamedTarget("up");    return; }
        if (remote_.x_btn()) { goNamedTarget("left");  return; }

        if (!enabled_) return;

        if (joint_mode_)  publishJoint();
        else              publishCartesian();
    }

    void publishCartesian()
    {
        auto msg = std::make_unique<TwistStamped>();
        msg->header.stamp = node_->now();
        msg->header.frame_id = command_frame_id_;
        msg->twist.linear.x  = -remote_.x()     * 0.2;
        msg->twist.linear.y  = -remote_.y()     * 0.2;
        msg->twist.linear.z  = remote_.z()     * 0.2;
        msg->twist.angular.x = remote_.roll()  * 1.0;
        msg->twist.angular.y = remote_.pitch() * 1.0;
        msg->twist.angular.z = -remote_.yaw()   * 1.0;

        // 始终发 DLS
        auto dls_msg = std::make_unique<TwistStamped>(*msg);
        dls_pub_->publish(std::move(dls_msg));

        // 有 Servo 时也发一份
        if (use_servo_)
            twist_pub_->publish(std::move(msg));
    }

    void publishJoint()
    {
        if (use_servo_)
        {
            auto msg = std::make_unique<JointJog>();
            msg->header.stamp = node_->now();
            msg->header.frame_id = command_frame_id_;
            auto add = [&](const std::string& n, double v) {
                if (std::abs(v) > 0.05) { msg->joint_names.push_back(n); msg->velocities.push_back(v * 1.5); }
            };
            add("joint1", remote_.j1()); add("joint2", remote_.j2());
            add("joint3", remote_.j3()); add("joint4", remote_.j4());
            add("joint5", -remote_.j5()); add("joint6", remote_.j6());
            add("joint7", remote_.j7());
            if (!msg->joint_names.empty()) joint_pub_->publish(std::move(msg));
        }
        else
        {
            // 纯DLS模式：关节速度发 /joint_vel_cmds
            auto msg = std::make_unique<Float64MultiArray>();
            constexpr double k = 0.5;
            msg->data = {
                k * remote_.j1(), k * remote_.j2(), k * remote_.j3(), k * remote_.j4(),
                k * remote_.j5(), k * remote_.j6(), k * remote_.j7()
            };
            vel_pub_->publish(std::move(msg));
        }
    }

    void goNamedTarget(const std::string& target)
    {
        enabled_ = false;
        if (use_servo_) servo_->setPaused(true);
        { auto m = std::make_unique<std_msgs::msg::Bool>(); m->data = true; pose_active_pub_->publish(std::move(m)); }
        rclcpp::sleep_for(std::chrono::milliseconds(50));  // 等 FSM 切到 POSE 并完成位置同步

        // 取消 arm_controller 对上次轨迹终点的 hold
        move_group_->stop();
        rclcpp::sleep_for(std::chrono::milliseconds(50));

        RCLCPP_INFO(node_->get_logger(), "[PLAN] → %s", target.c_str());
        try
        {
            if (last_joint_state_ && !last_joint_state_->position.empty())
            {
                moveit::core::RobotState start_state(move_group_->getRobotModel());
                start_state.setVariablePositions(last_joint_state_->name, last_joint_state_->position);
                start_state.update();
                move_group_->setStartState(start_state);
                RCLCPP_INFO(node_->get_logger(), "[PLAN] from J1=%.3f J2=%.3f → %s",
                    last_joint_state_->position[0], last_joint_state_->position[1], target.c_str());
            }
            else { move_group_->setStartStateToCurrentState(); }
            move_group_->setNamedTarget(target);
            auto target_map = move_group_->getNamedTargetValues(target);
            if (!target_map.empty()) {
                const char* jn[7] = {"joint1","joint2","joint3","joint4","joint5","joint6","joint7"};
                std::vector<double> vals(7, 0.0);
                for (int i = 0; i < 7; ++i) vals[i] = target_map[jn[i]];
                auto msg = std::make_unique<Float64MultiArray>();
                msg->data = vals;
                joint_pos_pub_->publish(std::move(msg));
                // 等硬件接口完成插值（750 帧 @ 500Hz ≈ 1.5s），额外加 200ms 余量
                rclcpp::sleep_for(std::chrono::milliseconds(1700));
                RCLCPP_INFO(node_->get_logger(), "[PLAN] interpolated → %s", target.c_str());
            } else RCLCPP_WARN(node_->get_logger(), "[PLAN] target not found");
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(node_->get_logger(), "[PLAN] %s", e.what());
        }
        if (use_servo_) servo_->setPaused(false);
        { auto m = std::make_unique<std_msgs::msg::Bool>(); m->data = false; pose_active_pub_->publish(std::move(m)); }
        enabled_ = true;
        RCLCPP_INFO(node_->get_logger(), "[PLAN] resumed");
    }

    rclcpp::Node::SharedPtr node_;
    remote::Remote remote_;
    sensor_msgs::msg::JointState::SharedPtr last_joint_state_;
    std::string command_frame_id_ = "base_link";

    // Servo（可选）
    bool use_servo_ = true;
    std::unique_ptr<moveit_servo::Servo> servo_;
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
    rclcpp::Publisher<TwistStamped>::SharedPtr twist_pub_;
    rclcpp::Publisher<JointJog>::SharedPtr joint_pub_;

    // 电机使能
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr motor_enable_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pose_active_pub_;

    // DLS
    rclcpp::Publisher<TwistStamped>::SharedPtr dls_pub_;

    // 纯DLS关节模式
    rclcpp::Publisher<Float64MultiArray>::SharedPtr vel_pub_;
    rclcpp::Publisher<Float64MultiArray>::SharedPtr joint_pos_pub_;

    // MoveGroup
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

    rclcpp::Subscription<Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    std::atomic<bool> enabled_{true};
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
