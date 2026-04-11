#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <example_interfaces/msg/bool.hpp>
#include <example_interfaces/msg/float64_multi_array.hpp>
#include <example_interfaces/msg/string.hpp>
#include <my_robot_interfaces/msg/pose_command.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <tf2/LinearMath/Matrix3x3.h>

#include <sensor_msgs/msg/joy.hpp>
#include <moveit_servo/servo.h>
#include <moveit_servo/status_codes.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using Bool = example_interfaces::msg::Bool;
using FloatArray = example_interfaces::msg::Float64MultiArray;
using StringMsg = example_interfaces::msg::String;
using PoseCmd = my_robot_interfaces::msg::PoseCommand;
using Joy = sensor_msgs::msg::Joy;
using TwistStamped = geometry_msgs::msg::TwistStamped;
using namespace std::placeholders;


class Commander
{   
    public:
        Commander(std::shared_ptr<rclcpp::Node> node)
        {   
            node_ = node;
            arm_ = std::make_shared<MoveGroupInterface>(node_, "arm");
            arm_->setMaxVelocityScalingFactor(1.0);
            arm_->setMaxAccelerationScalingFactor(1.0);
            gripper_ = std::make_shared<MoveGroupInterface>(node_, "gripper");

            // --- Setup Planning Scene Monitor (Required by Servo) ---
            psm_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node_, "robot_description");
            
            // Wait for planning scene monitor to load robot model
            int retry = 0;
            while (rclcpp::ok() && !psm_->getRobotModel() && retry < 10) {
                RCLCPP_INFO(node_->get_logger(), "Waiting for robot model to load in PlanningSceneMonitor...");
                rclcpp::sleep_for(std::chrono::milliseconds(500));
                retry++;
            }

            if (psm_->getRobotModel()) {
                psm_->startStateMonitor();
                psm_->startSceneMonitor();
                // Load Servo parameters
                auto servo_parameters = moveit_servo::ServoParameters::makeServoParameters(node_, "moveit_servo");
                if (!servo_parameters) {
                    RCLCPP_ERROR(node_->get_logger(), "Failed to load MoveIt Servo parameters!");
                    return;
                }

                // Initialize Servo
                servo_ = std::make_unique<moveit_servo::Servo>(node_, servo_parameters, psm_);
                servo_->start();
                RCLCPP_INFO(node_->get_logger(), "MoveIt Servo started successfully.");
            } else {
                RCLCPP_ERROR(node_->get_logger(), "Failed to initialize PlanningSceneMonitor: Robot model is null.");
            }

            twist_pub_ = node_->create_publisher<TwistStamped>("servo_node/delta_twist_cmds", 10);

            // --- Subscriptions ---
            open_gripper_sub_ = node_->create_subscription<Bool>("open_gripper", 10,
                std::bind(&Commander::openGripperCallback, this, _1));
            
            joint_cmd_sub_ = node_->create_subscription<FloatArray>("joint_command", 10,
                std::bind(&Commander::jointComCallback, this, _1));

            pose_cmd_sub_ = node_->create_subscription<PoseCmd>("pose_command", 10,
                std::bind(&Commander::poseComCallback, this, _1));

            named_target_sub_ = node_->create_subscription<StringMsg>("named_target_command", 10,
                std::bind(&Commander::namedTargetCallback, this, _1));

            joy_sub_ = node_->create_subscription<Joy>("joy", 10,
                std::bind(&Commander::joyCallback, this, _1));
        }

        void goToNamedTarget(const std::string &name)
        {
            arm_->setStartStateToCurrentState();
            arm_->setNamedTarget(name);
            planAndExecute(arm_);
        }

        void openGripper()
        {
            gripper_->setStartStateToCurrentState();
            gripper_->setNamedTarget("open");
            planAndExecute(gripper_);
        }

        void closeGripper()
        {
            gripper_->setStartStateToCurrentState();
            gripper_->setNamedTarget("close");
            planAndExecute(gripper_);
        }
    private:
        void planAndExecute(const std::shared_ptr<MoveGroupInterface> &interface)
        {
            MoveGroupInterface::Plan plan;
            bool success = (interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

            if(success)
            {
                interface->execute(plan);
            }
        }

        void openGripperCallback(const Bool &msg)
        {
            if(msg.data) openGripper();
            else closeGripper();
        }

        void jointComCallback(const FloatArray &msg)
        {
            auto joints = msg.data;
            if(joints.size() == 7) {
                arm_->setStartStateToCurrentState();
                arm_->setJointValueTarget(joints);
                planAndExecute(arm_);
            }
        }

        void poseComCallback(const PoseCmd &msg)
        {
            tf2::Quaternion q;
            q.setRPY(msg.roll, msg.pitch, msg.yaw);
            geometry_msgs::msg::PoseStamped target_pose;
            target_pose.header.frame_id = "base_link";
            target_pose.pose.position.x = msg.x;
            target_pose.pose.position.y = msg.y;
            target_pose.pose.position.z = msg.z;
            target_pose.pose.orientation = tf2::toMsg(q);

            arm_->setStartStateToCurrentState();
            arm_->setPoseTarget(target_pose);
            planAndExecute(arm_);
        }

        void namedTargetCallback(const StringMsg &msg)
        {
            goToNamedTarget(msg.data);
        }

        void joyCallback(const Joy &msg)
        {
            // --- Buttons (High Level Commands) ---
            if(msg.buttons[0]) { is_busy_ = true; goToNamedTarget("home"); is_busy_ = false; return; }
            if(msg.buttons[1]) { is_busy_ = true; goToNamedTarget("right"); is_busy_ = false; return; }
            if(msg.buttons[2]) { is_busy_ = true; goToNamedTarget("left"); is_busy_ = false; return; }
            if(msg.buttons[3]) { is_busy_ = true; goToNamedTarget("up"); is_busy_ = false; return; }
            if(msg.buttons[4]) { is_busy_ = true; openGripper(); is_busy_ = false; return; }
            if(msg.buttons[5]) { is_busy_ = true; closeGripper(); is_busy_ = false; return; }

            if(is_busy_) return; 

            // --- Servo Velocity Control (Continuous) ---
            auto twist_msg = std::make_unique<TwistStamped>();
            twist_msg->header.stamp = node_->now();
            twist_msg->header.frame_id = "base_link";

            auto stick_val = [](double val) { 
                const double deadzone = 0.15;
                if (std::abs(val) < deadzone) return 0.0;
                return (val - (val > 0 ? deadzone : -deadzone)) / (1.0 - deadzone);
            };

            // Map sticks to Twist
            twist_msg->twist.linear.x = stick_val(msg.axes[1]); // Left Stick Up/Down -> X
            twist_msg->twist.linear.y = stick_val(msg.axes[0]); // Left Stick Left/Right -> Y
            
            // Right Stick Y -> Z
            if(msg.axes.size() > 4) twist_msg->twist.linear.z = stick_val(msg.axes[4]);

            // Right Stick X -> Roll
            if(msg.axes.size() > 3) twist_msg->twist.angular.x = stick_val(msg.axes[3]);
            
            // LT/RT -> Pitch (Axes 2 and 5 usually)
            if(msg.axes.size() > 2) twist_msg->twist.angular.y = stick_val(msg.axes[2]);
            
            // D-pad -> Yaw (Axis 6)
            if(msg.axes.size() > 6) twist_msg->twist.angular.z = stick_val(msg.axes[6]);

            // Only publish if non-zero
            if(std::abs(twist_msg->twist.linear.x) > 0 || std::abs(twist_msg->twist.linear.y) > 0 || std::abs(twist_msg->twist.linear.z) > 0 ||
               std::abs(twist_msg->twist.angular.x) > 0 || std::abs(twist_msg->twist.angular.y) > 0 || std::abs(twist_msg->twist.angular.z) > 0)
            {
                twist_pub_->publish(std::move(twist_msg));
            }
        }

        std::shared_ptr<rclcpp::Node> node_;
        std::shared_ptr<MoveGroupInterface> arm_;
        std::shared_ptr<MoveGroupInterface> gripper_;
        std::atomic<bool> is_busy_{false};
        
        // --- MoveIt Servo ---
        std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> psm_;
        std::unique_ptr<moveit_servo::Servo> servo_;
        rclcpp::Publisher<TwistStamped>::SharedPtr twist_pub_;

        rclcpp::Subscription<Bool>::SharedPtr open_gripper_sub_;
        rclcpp::Subscription<FloatArray>::SharedPtr joint_cmd_sub_;
        rclcpp::Subscription<PoseCmd>::SharedPtr pose_cmd_sub_;
        rclcpp::Subscription<StringMsg>::SharedPtr named_target_sub_;
        rclcpp::Subscription<Joy>::SharedPtr joy_sub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("commander");
    
    // MultiThreadedExecutor is REQUIRED for Servo and PSM to work in parallel
    rclcpp::executors::MultiThreadedExecutor executor;
    auto commander = Commander(node);
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
