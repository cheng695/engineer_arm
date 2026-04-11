#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <example_interfaces/msg/bool.hpp>
#include <example_interfaces/msg/float64_multi_array.hpp>
#include <example_interfaces/msg/string.hpp>
#include <my_robot_interfaces/msg/pose_command.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

#include <sensor_msgs/msg/joy.hpp>

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using Bool = example_interfaces::msg::Bool;
using FloatArray = example_interfaces::msg::Float64MultiArray;
using StringMsg = example_interfaces::msg::String;
using PoseCmd = my_robot_interfaces::msg::PoseCommand;
using Joy = sensor_msgs::msg::Joy;
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

        void goToJointTarget(const std::vector<double> &joints)
        {
            arm_->setStartStateToCurrentState();
            arm_->setJointValueTarget(joints);
            planAndExecute(arm_);
        }

        void goToPoseTarget(double x, double y, double z, double roll, double pitch, double yaw, bool cartesian_path = false)
        {
            tf2::Quaternion q;
            q.setRPY(roll, pitch, yaw);
            q = q.normalize();

            geometry_msgs::msg::PoseStamped target_pose;
            target_pose.header.frame_id = "base_link";
            target_pose.pose.position.x = x;
            target_pose.pose.position.y = y;
            target_pose.pose.position.z = z;
            target_pose.pose.orientation.x = q.getX();
            target_pose.pose.orientation.y = q.getY();
            target_pose.pose.orientation.z = q.getZ();
            target_pose.pose.orientation.w = q.getW();


            arm_->setStartStateToCurrentState();
            
            if(!cartesian_path)
            { 
                arm_->setPoseTarget(target_pose);
                planAndExecute(arm_);
            }
            else
            {
                std::vector<geometry_msgs::msg::Pose> waypoints;
                waypoints.push_back(target_pose.pose);
                moveit_msgs::msg::RobotTrajectory trajectory;

                double fraction = arm_->computeCartesianPath(waypoints, 0.01, 0.0, trajectory, true);

                if(fraction == 1.0)
                {
                    arm_->execute(trajectory);
                }
            }
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
            if(msg.data)
            {
                openGripper();
            }
            else
            {
                closeGripper();
            }
        }

        void jointComCallback(const FloatArray &msg)
        {
            auto joints = msg.data;
            
            if(joints.size() == 7)
            {
                goToJointTarget(joints);
            }
        }

        void poseComCallback(const PoseCmd &msg)
        {
            goToPoseTarget(msg.x, msg.y, msg.z, msg.roll, msg.pitch, msg.yaw, msg.cartesian_path);
        }

        void namedTargetCallback(const StringMsg &msg)
        {
            goToNamedTarget(msg.data);
        }

        void joyCallback(const Joy &msg)
        {
            // Simple mapping example:
            // buttons[0] (A) -> home
            // buttons[1] (B) -> right
            // buttons[2] (X) -> open gripper
            // buttons[3] (Y) -> close gripper

            if(msg.buttons[0]) goToNamedTarget("home");
            else if(msg.buttons[1]) goToNamedTarget("right");
            
            if(msg.buttons[2]) openGripper();
            else if(msg.buttons[3]) closeGripper();

            // Example for Cartesian jogging (moving relative to current pose)
            // Left Stick (axes 1, 0) for X, Y
            if(std::abs(msg.axes[1]) > 0.5 || std::abs(msg.axes[0]) > 0.5 || std::abs(msg.axes[4]) > 0.5)
            {
                auto current_pose = arm_->getCurrentPose().pose;
                double step = 0.05; // 5cm step
                
                double target_x = current_pose.position.x + msg.axes[1] * step;
                double target_y = current_pose.position.y + msg.axes[0] * step;
                double target_z = current_pose.position.z + msg.axes[4] * step;

                // For simplicity, keep current orientation
                // In a real application, you'd want smoother jogging (e.g. MoveIt Servo)
                // goToPoseTarget(target_x, target_y, target_z, ...);
                RCLCPP_INFO(node_->get_logger(), "Joy Jog target: X=%.2f, Y=%.2f, Z=%.2f", target_x, target_y, target_z);
            }
        }

        std::shared_ptr<rclcpp::Node> node_;
        std::shared_ptr<MoveGroupInterface> arm_;
        std::shared_ptr<MoveGroupInterface> gripper_;

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
    auto commander = Commander(node);
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
