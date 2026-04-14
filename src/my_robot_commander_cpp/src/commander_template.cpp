#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <example_interfaces/msg/bool.hpp>
#include <example_interfaces/msg/float64_multi_array.hpp>
#include <example_interfaces/msg/string.hpp>
#include <std_msgs/msg/int8.hpp>
#include <my_robot_interfaces/msg/pose_command.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <tf2/LinearMath/Matrix3x3.h>

#include <sensor_msgs/msg/joy.hpp>
#include <moveit_servo/servo.h>
#include <moveit_servo/status_codes.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <controller_manager_msgs/srv/set_hardware_component_state.hpp>
#include <std_msgs/msg/bool.hpp>

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using Bool = example_interfaces::msg::Bool;
using FloatArray = example_interfaces::msg::Float64MultiArray;
using StringMsg = example_interfaces::msg::String;
using PoseCmd = my_robot_interfaces::msg::PoseCommand;
using Joy = sensor_msgs::msg::Joy;
using TwistStamped = geometry_msgs::msg::TwistStamped;
using JointJog = control_msgs::msg::JointJog;
using Int8 = std_msgs::msg::Int8;
using SetHWState = controller_manager_msgs::srv::SetHardwareComponentState;
using BoolMsg = std_msgs::msg::Bool;
using namespace std::placeholders;


class Commander
{   
    public:
        Commander(std::shared_ptr<rclcpp::Node> node)
        {   
            node_ = node;
            RCLCPP_INFO(node_->get_logger(), " --- V5.2 COMPLETE ELITE COMMANDER --- ");

            arm_ = std::make_shared<MoveGroupInterface>(node_, "arm");
            gripper_ = std::make_shared<MoveGroupInterface>(node_, "gripper");

            psm_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node_, "robot_description");
            
            servo_parameters_ = moveit_servo::ServoParameters::makeServoParameters(node_, "moveit_servo");
            if (!servo_parameters_) {
                 servo_parameters_ = moveit_servo::ServoParameters::makeServoParameters(node_);
            }

            int retry = 0;
            while (rclcpp::ok() && !psm_->getRobotModel() && retry < 20) {
                RCLCPP_INFO(node_->get_logger(), "Waiting for robot model... (%d)", retry+1);
                rclcpp::sleep_for(std::chrono::milliseconds(500));
                retry++;
            }

            if (psm_->getRobotModel() && servo_parameters_) {
                psm_->startStateMonitor(servo_parameters_->joint_topic);
                psm_->startSceneMonitor(servo_parameters_->monitored_planning_scene_topic);
                psm_->startWorldGeometryMonitor();
                psm_->getStateMonitor()->enableCopyDynamics(true);

                startServo();

                twist_pub_ = node_->create_publisher<TwistStamped>(servo_parameters_->cartesian_command_in_topic, 10);
                joint_pub_ = node_->create_publisher<JointJog>(servo_parameters_->joint_command_in_topic, 10);
                
                debug_sub_ = node_->create_subscription<trajectory_msgs::msg::JointTrajectory>(
                    servo_parameters_->command_out_topic, 10,
                    [this](const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
                        if (!msg->points.empty() && !msg->points[0].velocities.empty()) {
                            // double v0 = msg->points[0].velocities[0];
                        }
                    });

            } else {
                RCLCPP_ERROR(node_->get_logger(), "CRITICAL: Init Failed.");
            }

            hw_client_ = node_->create_client<SetHWState>("/controller_manager/set_hardware_component_state");
            motor_enable_pub_ = node_->create_publisher<BoolMsg>("/arm_motor_enable", 10);
            joy_sub_ = node_->create_subscription<Joy>("joy", 10, std::bind(&Commander::joyCallback, this, _1));
        }

    private:
        bool motor_enabled_{false};
        bool prev_guide_pressed_{false};
        rclcpp::Client<SetHWState>::SharedPtr hw_client_;
        rclcpp::Publisher<BoolMsg>::SharedPtr motor_enable_pub_;
        void startServo() {
            if (servo_) return;
            try {
                servo_ = std::make_unique<moveit_servo::Servo>(node_, servo_parameters_, psm_);
                servo_->start();
                RCLCPP_INFO(node_->get_logger(), "[SERVOMAN] Instances Linked.");
            } catch (const std::exception& e) {
                RCLCPP_ERROR(node_->get_logger(), "[SERVOMAN] Start Error: %s", e.what());
            }
        }

        void stopServo() {
            if (servo_) {
                servo_->setPaused(true);
                servo_.reset();
                RCLCPP_INFO(node_->get_logger(), "[SERVOMAN] Instances Collapsed.");
            }
        }

        void planAndExecute(const std::shared_ptr<MoveGroupInterface> &interface)
        {
            MoveGroupInterface::Plan plan;
            if(interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
                is_busy_ = true;
                
                // ONLY reset Servo if the Arm is moving (to prevent snap-back)
                // Gripper is independent and doesn't need to kill the arm engine.
                bool is_arm = (interface == arm_);
                if (is_arm) stopServo(); 
                
                RCLCPP_INFO(node_->get_logger(), "[ASYNC] Starting Execution (%s)...", is_arm ? "ARM" : "GRIPPER");
                
                std::thread([this, interface, plan, is_arm]() {
                    auto result = interface->execute(plan); 
                    if (result == moveit::core::MoveItErrorCode::SUCCESS) {
                        RCLCPP_INFO(node_->get_logger(), "[SETTLE] Motion Finished Successfully.");
                    } else {
                        RCLCPP_ERROR(node_->get_logger(), "[ABORT] Motion Interrupted/Failed. Code: %d", result.val);
                    }
                    
                    if (is_arm) {
                        rclcpp::sleep_for(std::chrono::milliseconds(1200)); 
                        psm_->requestPlanningSceneState();
                        startServo();
                    }
                    
                    is_busy_ = false;
                    RCLCPP_INFO(node_->get_logger(), "[READY] System Restored.");
                }).detach();
            } else {
                RCLCPP_WARN(node_->get_logger(), "Planning failed!");
            }
        }

        void joyCallback(const Joy &msg)
        {
            auto now = node_->now();
            auto msg_time = rclcpp::Time(msg.header.stamp);
            if ((now - msg_time).seconds() > 0.5) return; 

            // Xbox Button (Button 8) - Hardware Motor Toggle
            bool guide_pressed = (msg.buttons.size() > 8 && msg.buttons[8]);
            if (guide_pressed && !prev_guide_pressed_) {
                motor_enabled_ = !motor_enabled_;
                auto enable_msg = std::make_unique<BoolMsg>();
                enable_msg->data = motor_enabled_;
                motor_enable_pub_->publish(std::move(enable_msg));
                RCLCPP_WARN(node_->get_logger(), "[POWER] Motor Torque %s", motor_enabled_ ? "ENABLE" : "DISABLE");
            }
            prev_guide_pressed_ = guide_pressed;

            // RT Trigger Detection (RT = Axis 5 or Button 7)
            bool rt_pressed = false;
            if (msg.axes.size() > 5 && msg.axes[5] < 0.7) rt_pressed = true;
            if (msg.buttons.size() > 7 && msg.buttons[7]) rt_pressed = true;

            if (rt_pressed && !prev_rt_pressed_) {
                RCLCPP_WARN(node_->get_logger(), " !!! [STOP] INTERRUPT DETECTED !!! ");
                if (is_busy_) {
                    RCLCPP_ERROR(node_->get_logger(), ">>> FORCING ABORT ON ALL MOTIONS <<<");
                    arm_->stop();
                    gripper_->stop();
                }
                manual_paused_ = !manual_paused_;
                if (manual_paused_) {
                    RCLCPP_WARN(node_->get_logger(), "[LOCK] Controller Input Inhibited.");
                    if (servo_) servo_->setPaused(true);
                } else {
                    RCLCPP_INFO(node_->get_logger(), "[UNLOCK] Control Restored.");
                    if (servo_) servo_->setPaused(false);
                }
            }
            prev_rt_pressed_ = rt_pressed;

            // Mode Toggle (Right Stick Click - R3 - Button 10)
            bool r3_toggle = (msg.buttons.size() > 10 && msg.buttons[10]);
            if (r3_toggle && !prev_r3_pressed_) {
                global_joint_mode_ = !global_joint_mode_;
                if (global_joint_mode_) {
                    RCLCPP_WARN(node_->get_logger(), "[MODE] FULL JOINT SPACE ACTIVATED");
                } else {
                    RCLCPP_INFO(node_->get_logger(), "[MODE] CARTESIAN SPACE RESTORED");
                }
            }
            prev_r3_pressed_ = r3_toggle;

            if(is_busy_ || manual_paused_) return; 

            // Buttons mapping ...
            if(msg.buttons.size() > 5) {
                if(msg.buttons[0]) { arm_->setStartStateToCurrentState(); arm_->setNamedTarget("home"); planAndExecute(arm_); return; }
                if(msg.buttons[1]) { arm_->setStartStateToCurrentState(); arm_->setNamedTarget("right"); planAndExecute(arm_); return; }
                if(msg.buttons[2]) { arm_->setStartStateToCurrentState(); arm_->setNamedTarget("left"); planAndExecute(arm_); return; }
                if(msg.buttons[3]) { arm_->setStartStateToCurrentState(); arm_->setNamedTarget("up"); planAndExecute(arm_); return; }
                if(msg.buttons[4]) { gripper_->setStartStateToCurrentState(); gripper_->setNamedTarget("open"); planAndExecute(gripper_); return; }
                if(msg.buttons[5]) { gripper_->setStartStateToCurrentState(); gripper_->setNamedTarget("close"); planAndExecute(gripper_); return; }
            }

            auto stick_val = [](double val) { 
                const double deadzone = 0.12; 
                if (std::abs(val) < deadzone) return 0.0;
                return (val - (val > 0 ? deadzone : -deadzone)) / (1.0 - deadzone);
            };

            if (!servo_) return;

            // Mode Logic
            bool tool_mode = (msg.axes.size() > 2 && msg.axes[2] < 0.7) ||
                             (msg.buttons.size() > 6 && msg.buttons[6]);
            
            if (global_joint_mode_) {
                auto joint_msg = std::make_unique<JointJog>();
                joint_msg->header.stamp = node_->now();
                joint_msg->header.frame_id = "base_link";
                joint_msg->joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};
                
                // L3 Hold check (Context Modifier - Left Stick Click - Button 9)
                bool l3_hold = (msg.buttons.size() > 9 && msg.buttons[9]);

                // J1, J2, J3 (Inverted)
                joint_msg->velocities.push_back(-stick_val(msg.axes[0])); // J1 (LX)
                joint_msg->velocities.push_back(-stick_val(msg.axes[1])); // J2 (LY)
                joint_msg->velocities.push_back(-stick_val(msg.axes[4])); // J3 (RY)
                
                // J4 (RX) - Restricted if L3 is held (Inverted)
                joint_msg->velocities.push_back(l3_hold ? 0.0 : -stick_val(msg.axes[3]));
                
                // J5, J6 (DPAD) - Not inverted
                joint_msg->velocities.push_back(msg.axes.size() > 7 ? stick_val(msg.axes[7]) : 0.0);
                joint_msg->velocities.push_back(msg.axes.size() > 6 ? stick_val(msg.axes[6]) : 0.0);
                
                // J7 (RX if L3 held) (Inverted)
                joint_msg->velocities.push_back(l3_hold ? -stick_val(msg.axes[3]) : 0.0);

                if (joint_pub_) {
                    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, "[TELEMETRY] JOINT Space Jogging Active.");
                    joint_pub_->publish(std::move(joint_msg));
                }
            } else {
                auto twist_msg = std::make_unique<TwistStamped>();
                twist_msg->header.stamp = node_->now();
                twist_msg->header.frame_id = tool_mode ? "gripper_center" : "base_link";
                double lx = stick_val(msg.axes[1]); double ly = stick_val(msg.axes[0]);
                double lz = (msg.axes.size() > 4) ? stick_val(msg.axes[4]) : 0.0;
                double ax = (msg.axes.size() > 3) ? -stick_val(msg.axes[3]) : 0.0;
                double ay = (msg.axes.size() > 7) ? stick_val(msg.axes[7]) : 0.0;
                double az = (msg.axes.size() > 6) ? stick_val(msg.axes[6]) : 0.0;

                twist_msg->twist.linear.x = lx; twist_msg->twist.linear.y = ly; twist_msg->twist.linear.z = lz; 
                twist_msg->twist.angular.x = ax; twist_msg->twist.angular.y = ay; twist_msg->twist.angular.z = az; 

                if (std::abs(lx)>0.01 || std::abs(lz)>0.01) {
                    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, "[TELEMETRY] Axis Processing Active.");
                }
                if(twist_pub_) twist_pub_->publish(std::move(twist_msg));
            }
        }

        std::shared_ptr<rclcpp::Node> node_;
        std::shared_ptr<MoveGroupInterface> arm_;
        std::shared_ptr<MoveGroupInterface> gripper_;
        std::atomic<bool> is_busy_{false};
        bool manual_paused_{false};
        bool prev_rt_pressed_{false};
        bool global_joint_mode_{false};
        bool prev_r3_pressed_{false};
        std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> psm_;
        std::unique_ptr<moveit_servo::Servo> servo_;
        std::shared_ptr<const moveit_servo::ServoParameters> servo_parameters_;
        rclcpp::Publisher<TwistStamped>::SharedPtr twist_pub_;
        rclcpp::Publisher<JointJog>::SharedPtr joint_pub_;
        rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr debug_sub_;
        rclcpp::Subscription<Joy>::SharedPtr joy_sub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("commander");
    rclcpp::executors::MultiThreadedExecutor executor;
    auto commander = Commander(node);
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}