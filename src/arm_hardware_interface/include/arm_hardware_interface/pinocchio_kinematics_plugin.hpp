#pragma once

#include <memory>
#include <string>
#include <vector>

#include <moveit/kinematics_base/kinematics_base.h>
#include <pinocchio/multibody/fwd.hpp>
#include <Eigen/Core>

namespace arm_hardware_interface
{

/**
 * @brief Pinocchio-based DLS (Damped Least Squares) kinematics plugin for MoveIt.
 *
 * Reuses the same Pinocchio model loading and adaptive DLS algorithm as
 * CartesianController, but wraps it in the standard kinematics::KinematicsBase
 * interface so MoveIt Servo and MoveIt planning can both use it.
 */
class PinocchioKinematicsPlugin : public kinematics::KinematicsBase
{
public:
    PinocchioKinematicsPlugin();
    ~PinocchioKinematicsPlugin() override;

    // ---- KinematicsBase overrides ----

    bool initialize(const rclcpp::Node::SharedPtr& node,
                    const moveit::core::RobotModel& robot_model,
                    const std::string& group_name,
                    const std::string& base_frame,
                    const std::vector<std::string>& tip_frames,
                    double search_discretization) override;

    const std::vector<std::string>& getJointNames() const override;
    const std::vector<std::string>& getLinkNames() const override;

    // Single-solution IK (closest to seed)
    bool getPositionIK(
        const geometry_msgs::msg::Pose& ik_pose,
        const std::vector<double>& ik_seed_state,
        std::vector<double>& solution,
        moveit_msgs::msg::MoveItErrorCodes& error_code,
        const kinematics::KinematicsQueryOptions& options
            = kinematics::KinematicsQueryOptions()) const override;

    // Multi-solution IK (discretized redundant joint)
    bool getPositionIK(
        const std::vector<geometry_msgs::msg::Pose>& ik_poses,
        const std::vector<double>& ik_seed_state,
        std::vector<std::vector<double>>& solutions,
        kinematics::KinematicsResult& result,
        const kinematics::KinematicsQueryOptions& options) const override;

    // Search IK (four overloads)
    bool searchPositionIK(
        const geometry_msgs::msg::Pose& ik_pose,
        const std::vector<double>& ik_seed_state,
        double timeout,
        std::vector<double>& solution,
        moveit_msgs::msg::MoveItErrorCodes& error_code,
        const kinematics::KinematicsQueryOptions& options
            = kinematics::KinematicsQueryOptions()) const override;

    bool searchPositionIK(
        const geometry_msgs::msg::Pose& ik_pose,
        const std::vector<double>& ik_seed_state,
        double timeout,
        const std::vector<double>& consistency_limits,
        std::vector<double>& solution,
        moveit_msgs::msg::MoveItErrorCodes& error_code,
        const kinematics::KinematicsQueryOptions& options
            = kinematics::KinematicsQueryOptions()) const override;

    bool searchPositionIK(
        const geometry_msgs::msg::Pose& ik_pose,
        const std::vector<double>& ik_seed_state,
        double timeout,
        std::vector<double>& solution,
        const IKCallbackFn& solution_callback,
        moveit_msgs::msg::MoveItErrorCodes& error_code,
        const kinematics::KinematicsQueryOptions& options
            = kinematics::KinematicsQueryOptions()) const override;

    bool searchPositionIK(
        const geometry_msgs::msg::Pose& ik_pose,
        const std::vector<double>& ik_seed_state,
        double timeout,
        const std::vector<double>& consistency_limits,
        std::vector<double>& solution,
        const IKCallbackFn& solution_callback,
        moveit_msgs::msg::MoveItErrorCodes& error_code,
        const kinematics::KinematicsQueryOptions& options
            = kinematics::KinematicsQueryOptions()) const override;

    bool getPositionFK(
        const std::vector<std::string>& link_names,
        const std::vector<double>& joint_angles,
        std::vector<geometry_msgs::msg::Pose>& poses) const override;

private:
    // ---- Core iterative DLS IK ----
    /// Shared logic for all searchPositionIK overloads (re-seeding + callback)
    bool ikSolve(
        const geometry_msgs::msg::Pose& ik_pose,
        const std::vector<double>& ik_seed_state,
        const std::vector<double>& consistency_limits,
        const IKCallbackFn& solution_callback,
        std::vector<double>& solution,
        moveit_msgs::msg::MoveItErrorCodes& error_code,
        const kinematics::KinematicsQueryOptions& options) const;

    /// Pure iterative DLS IK (no re-seeding — the actual algorithm)
    bool dlsIk(const geometry_msgs::msg::Pose& ik_pose,
               const std::vector<double>& ik_seed_state,
               std::vector<double>& solution,
               moveit_msgs::msg::MoveItErrorCodes& error_code) const;

    // ---- Single DLS step (reuses CartController algorithm) ----
    Eigen::VectorXd computeDlsStep(const Eigen::VectorXd& q,
                                    const Eigen::Matrix<double, 6, 1>& twist) const;

    // ---- Helpers ----
    void twistToPose(const Eigen::Matrix<double, 6, 1>& twist,
                     geometry_msgs::msg::Pose& pose) const;

    // ---- Parameters ----
    double gain_{1.0};
    double dt_{0.1};
    int max_iterations_{150};
    double pos_tolerance_{5e-4};
    double rot_tolerance_{5e-3};
    double lambda_base_{0.1};
    double lambda_min_{0.01};
    double lambda_max_{5.0};
    double mu_threshold_{0.1};

    // ---- Pinocchio model (owned) ----
    std::unique_ptr<pinocchio::Model> model_;
    mutable std::unique_ptr<pinocchio::Data> data_;
    int ee_frame_id_{-1};

    // ---- Joint info ----
    std::vector<std::string> joint_names_;
    std::vector<std::string> link_names_;
    std::vector<double> joint_lower_limits_;
    std::vector<double> joint_upper_limits_;
    /// Number of velocity DOFs to skip (virtual base joints in Pinocchio model)
    size_t joint_vel_offset_{0};
    /// Number of actual arm joints
    size_t arm_joint_count_{0};
};

}  // namespace arm_hardware_interface
