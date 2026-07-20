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
#include <array>
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
        node_->declare_parameter("arm_version", "v1_0");
        const auto arm_version = node_->get_parameter("arm_version").as_string();
        node_->declare_parameter<std::vector<double>>(
            "joint_control_directions", {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
        const auto directions = node_->get_parameter("joint_control_directions").as_double_array();
        if (directions.size() == joint_control_directions_.size())
        {
            std::copy(directions.begin(), directions.end(), joint_control_directions_.begin());
        }
        else
        {
            RCLCPP_WARN(node_->get_logger(),
                "[JOINT] joint_control_directions size=%zu, expected 7; using all +1",
                directions.size());
        }

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
        msg->twist.linear.x  = -remote_.x()    * 0.5;
        msg->twist.linear.y  = -remote_.y()    * 0.5;
        msg->twist.linear.z  = remote_.z()     * 0.5;
        msg->twist.angular.x = remote_.roll()  * 1.5;
        msg->twist.angular.y = remote_.pitch() * 1.5;
        msg->twist.angular.z = remote_.yaw()   * 1.5;

        // 始终发 DLS
        auto dls_msg = std::make_unique<TwistStamped>(*msg);
        dls_pub_->publish(std::move(dls_msg));

        // 有 Servo 时也发一份
        if (use_servo_)
            twist_pub_->publish(std::move(msg));
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

        if (use_servo_)
        {
            auto msg = std::make_unique<JointJog>();
            msg->header.stamp = node_->now();
            msg->header.frame_id = command_frame_id_;
            auto add = [&](const std::string& n, double v) {
                if (std::abs(v) > 0.05) { msg->joint_names.push_back(n); msg->velocities.push_back(v * 1.5); }
            };
            add("joint1", joint_cmds[0]); add("joint2", joint_cmds[1]);
            add("joint3", joint_cmds[2]); add("joint4", joint_cmds[3]);
            add("joint5", -joint_cmds[4]); add("joint6", joint_cmds[5]);
            add("joint7", joint_cmds[6]);
            if (!msg->joint_names.empty()) joint_pub_->publish(std::move(msg));
        }
        else
        {
            // 纯DLS模式：关节速度发 /joint_vel_cmds
            auto msg = std::make_unique<Float64MultiArray>();
            constexpr double k = 2.5;
            msg->data = {
                k * joint_cmds[0], k * joint_cmds[1], k * joint_cmds[2], k * joint_cmds[3],
                k * joint_cmds[4], k * joint_cmds[5], k * joint_cmds[6]
            };
            vel_pub_->publish(std::move(msg));
        }
    }

    void publishTeleopHold()
    {
        auto twist = std::make_unique<TwistStamped>();
        twist->header.stamp = node_->now();
        twist->header.frame_id = command_frame_id_;
        dls_pub_->publish(std::move(twist));

        if (vel_pub_)
        {
            auto joint = std::make_unique<Float64MultiArray>();
            joint->data.assign(7, 0.0);
            vel_pub_->publish(std::move(joint));
        }
    }

    void setPoseActive(bool active)
    {
        auto m = std::make_unique<std_msgs::msg::Bool>();
        m->data = active;
        pose_active_pub_->publish(std::move(m));
    }

    void logStateValidity(const std::string& label, moveit::core::RobotState state)
    {
        const auto* arm_group = state.getRobotModel()->getJointModelGroup("arm");
        const bool bounds_ok = arm_group ? state.satisfiesBounds(arm_group) : state.satisfiesBounds();
        state.update();

        bool colliding = false;
        if (planning_scene_monitor_ && planning_scene_monitor_->getPlanningScene())
        {
            planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor_);
            if (scene)
            {
                colliding = scene->isStateColliding(state, "arm", true);
            }
        }

        RCLCPP_WARN(node_->get_logger(),
            "[PLAN-CHECK] %s bounds=%s collision=%s",
            label.c_str(), bounds_ok ? "OK" : "BAD", colliding ? "YES" : "NO");
    }

    void goNamedTarget(const std::string& target)
    {
        enabled_ = false;
        if (use_servo_) servo_->setPaused(true);

        // 停止遥操作输出，等待硬件侧回到当前位置保持，再基于真实当前状态规划。
        publishTeleopHold();
        move_group_->stop();
        rclcpp::sleep_for(std::chrono::milliseconds(120));

        RCLCPP_INFO(node_->get_logger(), "[PLAN] → %s", target.c_str());
        bool pose_started = false;
        try
        {
            // 设置起点为当前关节状态
            if (last_joint_state_ && !last_joint_state_->position.empty())
            {
                moveit::core::RobotState start_state(move_group_->getRobotModel());
                start_state.setVariablePositions(last_joint_state_->name, last_joint_state_->position);
                start_state.update();
                move_group_->setStartState(start_state);
                logStateValidity("start", start_state);
                RCLCPP_INFO(node_->get_logger(), "[PLAN] from J1=%.3f J2=%.3f → %s",
                    last_joint_state_->position[0], last_joint_state_->position[1], target.c_str());
            }
            else { move_group_->setStartStateToCurrentState(); }

            const auto* arm_group = move_group_->getRobotModel()->getJointModelGroup("arm");
            moveit::core::RobotState target_state(move_group_->getRobotModel());
            target_state.setToDefaultValues();
            if (arm_group && target_state.setToDefaultValues(arm_group, target))
            {
                logStateValidity("target " + target, target_state);
            }
            else
            {
                RCLCPP_ERROR(node_->get_logger(), "[PLAN-CHECK] named target '%s' not found", target.c_str());
            }

            move_group_->setNamedTarget(target);

            // 规划轨迹
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            auto success = move_group_->plan(plan);
            if (success == moveit::core::MoveItErrorCode::SUCCESS)
            {
                // 执行前才进入 POSE，避免规划期间 arm_controller 旧终点泄漏。
                setPoseActive(true);
                pose_started = true;
                rclcpp::sleep_for(std::chrono::milliseconds(20));

                RCLCPP_INFO(node_->get_logger(), "[PLAN] executing → %s", target.c_str());
                move_group_->execute(plan);  // 阻塞直到轨迹执行完成
            }
            else
            {
                RCLCPP_ERROR(node_->get_logger(), "[PLAN] plan failed for %s", target.c_str());
            }
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(node_->get_logger(), "[PLAN] %s", e.what());
        }

        // 退出 POSE → IDLE（同步到反馈 = 保持在目标位姿）
        if (pose_started)
        {
            setPoseActive(false);
            rclcpp::sleep_for(std::chrono::milliseconds(100));
        }

        if (use_servo_) servo_->setPaused(false);
        enabled_ = true;
        RCLCPP_INFO(node_->get_logger(), "[PLAN] resumed");
    }

    rclcpp::Node::SharedPtr node_;
    remote::Remote remote_;
    sensor_msgs::msg::JointState::SharedPtr last_joint_state_;
    std::string command_frame_id_ = "base_link";

    // Servo（可选）
    bool use_servo_ = true;
    std::array<double, 7> joint_control_directions_ = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
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
