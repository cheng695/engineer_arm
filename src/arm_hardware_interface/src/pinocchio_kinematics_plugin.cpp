#include "arm_hardware_interface/pinocchio_kinematics_plugin.hpp"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial/se3.hpp>

#include <Eigen/Cholesky>
#include <Eigen/Geometry>
#include <moveit/robot_model/robot_model.h>
#include <pluginlib/class_list_macros.hpp>
#include <random>
#include <rclcpp/rclcpp.hpp>

namespace arm_hardware_interface
{

// ================================================================
// Lifecycle
// ================================================================

PinocchioKinematicsPlugin::PinocchioKinematicsPlugin() = default;

PinocchioKinematicsPlugin::~PinocchioKinematicsPlugin() = default;

bool PinocchioKinematicsPlugin::initialize(
    const rclcpp::Node::SharedPtr& node,
    const moveit::core::RobotModel& robot_model,
    const std::string& group_name,
    const std::string& base_frame,
    const std::vector<std::string>& tip_frames,
    double search_discretization)
{
    // ---- Set up base class members ----
    storeValues(robot_model, group_name, base_frame, tip_frames, search_discretization);

    // ---- Load URDF into Pinocchio ----
    auto urdf_ptr = robot_model.getURDF();
    if (!urdf_ptr)
    {
        RCLCPP_ERROR(rclcpp::get_logger("PinocchioKine"), "RobotModel has no URDF");
        return false;
    }

    model_ = std::make_unique<pinocchio::Model>();
    try
    {
        pinocchio::urdf::buildModel(urdf_ptr, *model_);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("PinocchioKine"),
            "Failed to build Pinocchio model: %s", e.what());
        return false;
    }

    data_ = std::make_unique<pinocchio::Data>(*model_);

    // ---- Find end-effector frame ----
    const std::string& ee_frame = tip_frames.empty() ? tip_frames_[0] : tip_frames.front();
    if (!model_->existFrame(ee_frame))
    {
        RCLCPP_ERROR(rclcpp::get_logger("PinocchioKine"),
            "Frame '%s' not found in model", ee_frame.c_str());
        return false;
    }
    ee_frame_id_ = model_->getFrameId(ee_frame);

    // ---- Extract joint names and link names from MoveIt ----
    const auto* jmg = robot_model.getJointModelGroup(group_name);
    if (!jmg)
    {
        RCLCPP_ERROR(rclcpp::get_logger("PinocchioKine"),
            "JointModelGroup '%s' not found", group_name.c_str());
        return false;
    }
    joint_names_ = jmg->getActiveJointModelNames();
    link_names_  = jmg->getLinkModelNames();

    // ---- Find joint velocity offset (skip virtual base joints) ----
    joint_vel_offset_ = 0;
    arm_joint_count_ = joint_names_.size();
    if (!joint_names_.empty())
    {
        // Pinocchio model has all joints; find index of first arm joint
        for (size_t i = 0; i < model_->joints.size(); ++i)
        {
            if (model_->names[i] == joint_names_.front())
            {
                // The velocity index is joint index - 1 (universe joint at idx 0)
                joint_vel_offset_ = model_->joints[i].idx_v();
                break;
            }
        }
    }

    // ---- Extract joint limits from RobotModel ----
    const auto& bounds = robot_model.getActiveJointModelsBounds();
    joint_lower_limits_.resize(arm_joint_count_, -M_PI);
    joint_upper_limits_.resize(arm_joint_count_,  M_PI);
    for (size_t i = 0; i < arm_joint_count_ && i < bounds.size(); ++i)
    {
        if (bounds[i])
        {
            if (bounds[i]->at(0).position_bounded_)
            {
                joint_lower_limits_[i] = bounds[i]->at(0).min_position_;
                joint_upper_limits_[i] = bounds[i]->at(0).max_position_;
            }
        }
    }

    // ---- Read optional numeric parameters ----
    lookupParam(node, "dls_gain", gain_, gain_);
    lookupParam(node, "dls_dt", dt_, dt_);
    lookupParam(node, "dls_max_iterations", max_iterations_, max_iterations_);
    lookupParam(node, "dls_pos_tolerance", pos_tolerance_, pos_tolerance_);
    lookupParam(node, "dls_rot_tolerance", rot_tolerance_, rot_tolerance_);
    lookupParam(node, "dls_lambda_base", lambda_base_, lambda_base_);
    lookupParam(node, "dls_lambda_min", lambda_min_, lambda_min_);
    lookupParam(node, "dls_lambda_max", lambda_max_, lambda_max_);
    lookupParam(node, "dls_mu_threshold", mu_threshold_, mu_threshold_);

    RCLCPP_INFO(rclcpp::get_logger("PinocchioKine"),
        "Initialized: %zu joints, %zu links, ee_frame=%s, vel_offset=%zu",
        arm_joint_count_, link_names_.size(), ee_frame.c_str(), joint_vel_offset_);

    return true;
}

// ================================================================
// Joint and Link names
// ================================================================

const std::vector<std::string>& PinocchioKinematicsPlugin::getJointNames() const
{
    return joint_names_;
}

const std::vector<std::string>& PinocchioKinematicsPlugin::getLinkNames() const
{
    return link_names_;
}

// ================================================================
// getPositionFK
// ================================================================

bool PinocchioKinematicsPlugin::getPositionFK(
    const std::vector<std::string>& link_names,
    const std::vector<double>& joint_angles,
    std::vector<geometry_msgs::msg::Pose>& poses) const
{
    if (!model_ || !data_)
        return false;

    // Convert joint angles to full Pinocchio q vector
    Eigen::VectorXd q = Eigen::VectorXd::Zero(model_->nv);
    for (size_t i = 0; i < std::min(joint_angles.size(), arm_joint_count_); ++i)
        q[joint_vel_offset_ + i] = joint_angles[i];

    pinocchio::forwardKinematics(*model_, *data_, q);
    pinocchio::updateFramePlacements(*model_, *data_);

    poses.resize(link_names.size());
    for (size_t i = 0; i < link_names.size(); ++i)
    {
        if (!model_->existFrame(link_names[i]))
            return false;

        int fid = model_->getFrameId(link_names[i]);
        const auto& T = data_->oMf[fid];

        poses[i].position.x = T.translation().x();
        poses[i].position.y = T.translation().y();
        poses[i].position.z = T.translation().z();

        Eigen::Quaterniond quat(T.rotation());
        poses[i].orientation.x = quat.x();
        poses[i].orientation.y = quat.y();
        poses[i].orientation.z = quat.z();
        poses[i].orientation.w = quat.w();
    }

    return true;
}

// ================================================================
// getPositionIK (single pose)
// ================================================================

bool PinocchioKinematicsPlugin::getPositionIK(
    const geometry_msgs::msg::Pose& ik_pose,
    const std::vector<double>& ik_seed_state,
    std::vector<double>& solution,
    moveit_msgs::msg::MoveItErrorCodes& error_code,
    const kinematics::KinematicsQueryOptions& options) const
{
    return ikSolve(ik_pose, ik_seed_state, {},
                   IKCallbackFn(),
                   solution, error_code, options);
}

// ================================================================
// getPositionIK (multi-pose, discretized redundant)
// ================================================================

bool PinocchioKinematicsPlugin::getPositionIK(
    const std::vector<geometry_msgs::msg::Pose>& ik_poses,
    const std::vector<double>& ik_seed_state,
    std::vector<std::vector<double>>& solutions,
    kinematics::KinematicsResult& result,
    const kinematics::KinematicsQueryOptions& options) const
{
    if (ik_poses.size() == 1)
    {
        solutions.resize(1);
        moveit_msgs::msg::MoveItErrorCodes ec;
        bool ok = dlsIk(ik_poses[0], ik_seed_state, solutions[0], ec);
        result.kinematic_error = ok ? kinematics::KinematicError::OK
                                    : kinematics::KinematicError::NO_SOLUTION;
        return ok;
    }
    result.kinematic_error = kinematics::KinematicError::MULTIPLE_TIPS_NOT_SUPPORTED;
    return false;
}

// ================================================================
// searchPositionIK (4 overloads)
// ================================================================

bool PinocchioKinematicsPlugin::searchPositionIK(
    const geometry_msgs::msg::Pose& ik_pose,
    const std::vector<double>& ik_seed_state,
    double /*timeout*/,
    std::vector<double>& solution,
    moveit_msgs::msg::MoveItErrorCodes& error_code,
    const kinematics::KinematicsQueryOptions& options) const
{
    return ikSolve(ik_pose, ik_seed_state, {},
                   IKCallbackFn(),
                   solution, error_code, options);
}

bool PinocchioKinematicsPlugin::searchPositionIK(
    const geometry_msgs::msg::Pose& ik_pose,
    const std::vector<double>& ik_seed_state,
    double /*timeout*/,
    const std::vector<double>& consistency_limits,
    std::vector<double>& solution,
    moveit_msgs::msg::MoveItErrorCodes& error_code,
    const kinematics::KinematicsQueryOptions& options) const
{
    return ikSolve(ik_pose, ik_seed_state, consistency_limits,
                   IKCallbackFn(),
                   solution, error_code, options);
}

bool PinocchioKinematicsPlugin::searchPositionIK(
    const geometry_msgs::msg::Pose& ik_pose,
    const std::vector<double>& ik_seed_state,
    double /*timeout*/,
    std::vector<double>& solution,
    const IKCallbackFn& solution_callback,
    moveit_msgs::msg::MoveItErrorCodes& error_code,
    const kinematics::KinematicsQueryOptions& options) const
{
    return ikSolve(ik_pose, ik_seed_state, {},
                   solution_callback, solution, error_code, options);
}

bool PinocchioKinematicsPlugin::searchPositionIK(
    const geometry_msgs::msg::Pose& ik_pose,
    const std::vector<double>& ik_seed_state,
    double /*timeout*/,
    const std::vector<double>& consistency_limits,
    std::vector<double>& solution,
    const IKCallbackFn& solution_callback,
    moveit_msgs::msg::MoveItErrorCodes& error_code,
    const kinematics::KinematicsQueryOptions& options) const
{
    return ikSolve(ik_pose, ik_seed_state, consistency_limits,
                   solution_callback, solution, error_code, options);
}

// ================================================================
// Core IK solver
// ================================================================

bool PinocchioKinematicsPlugin::ikSolve(
    const geometry_msgs::msg::Pose& ik_pose,
    const std::vector<double>& ik_seed_state,
    const std::vector<double>& consistency_limits,
    const IKCallbackFn& solution_callback,
    std::vector<double>& solution,
    moveit_msgs::msg::MoveItErrorCodes& error_code,
    const kinematics::KinematicsQueryOptions& /*options*/) const
{
    if (!model_ || !data_)
    {
        error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
        return false;
    }

    // ---- First attempt with seed ----
    bool ok = dlsIk(ik_pose, ik_seed_state, solution, error_code);

    // If successful and no callback filtering, we're done
    if (ok && !solution_callback)
        return true;

    if (ok && solution_callback)
    {
        moveit_msgs::msg::MoveItErrorCodes cb_err;
        cb_err.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
        solution_callback(ik_pose, solution, cb_err);
        if (cb_err.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
            return true;
    }

    // ---- Re-seed and retry (for redundancy resolution) ----
    // If the seed failed, try random perturbations within consistency_limits
    if (!consistency_limits.empty() && consistency_limits.size() == arm_joint_count_)
    {
        std::random_device rd;
        std::mt19937 gen(rd());
        const int max_attempts = 10;

        for (int attempt = 0; attempt < max_attempts; ++attempt)
        {
            std::vector<double> perturbed_seed = ik_seed_state;
            for (size_t i = 0; i < arm_joint_count_; ++i)
            {
                double limit = consistency_limits[i];
                if (limit > 0.0)
                {
                    std::uniform_real_distribution<> dis(-limit, limit);
                    perturbed_seed[i] += dis(gen);
                    perturbed_seed[i] = std::clamp(
                        perturbed_seed[i],
                        joint_lower_limits_[i],
                        joint_upper_limits_[i]);
                }
            }

            moveit_msgs::msg::MoveItErrorCodes attempt_err;
            std::vector<double> attempt_sol;
            if (dlsIk(ik_pose, perturbed_seed, attempt_sol, attempt_err))
            {
                if (!solution_callback)
                {
                    solution = std::move(attempt_sol);
                    error_code = attempt_err;
                    return true;
                }

                moveit_msgs::msg::MoveItErrorCodes cb_err;
                cb_err.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
                solution_callback(ik_pose, attempt_sol, cb_err);
                if (cb_err.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
                {
                    solution = std::move(attempt_sol);
                    error_code = attempt_err;
                    return true;
                }
            }
        }
    }

    error_code.val = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
    return false;
}

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
// Better DLS implementation
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

// ==== Internal iterative DLS IK (the actual algorithm) ====
bool PinocchioKinematicsPlugin::dlsIk(
    const geometry_msgs::msg::Pose& ik_pose,
    const std::vector<double>& ik_seed_state,
    std::vector<double>& solution,
    moveit_msgs::msg::MoveItErrorCodes& error_code) const
{
    if (!model_ || !data_)
    {
        error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
        return false;
    }

    // ---- Build target SE3 ----
    Eigen::Translation3d t_target(ik_pose.position.x, ik_pose.position.y, ik_pose.position.z);
    Eigen::Quaterniond q_target(ik_pose.orientation.w, ik_pose.orientation.x,
                                ik_pose.orientation.y, ik_pose.orientation.z);
    pinocchio::SE3 T_desired(q_target.toRotationMatrix(), t_target.vector());

    // ---- Initialize joint position vector ----
    Eigen::VectorXd q = Eigen::VectorXd::Zero(model_->nv);
    for (size_t i = 0; i < std::min(ik_seed_state.size(), arm_joint_count_); ++i)
        q[joint_vel_offset_ + i] = ik_seed_state[i];

    // ---- Iterative DLS loop ----
    for (int iter = 0; iter < max_iterations_; ++iter)
    {
        // FK
        pinocchio::forwardKinematics(*model_, *data_, q);
        pinocchio::updateFramePlacements(*model_, *data_);
        const auto& T_current = data_->oMf[ee_frame_id_];

        // Pose error in WORLD-aligned coordinates (matches LOCAL_WORLD_ALIGNED Jacobian)
        Eigen::Vector3d pos_err = T_desired.translation() - T_current.translation();
        Eigen::Matrix3d R_err = T_desired.rotation() * T_current.rotation().transpose();
        Eigen::Vector3d rot_err = pinocchio::log3(R_err);

        double pos_norm = pos_err.norm();
        double rot_norm = rot_err.norm();

        // Convergence check
        if (pos_norm < pos_tolerance_ && rot_norm < rot_tolerance_)
        {
            solution.resize(arm_joint_count_);
            for (size_t i = 0; i < arm_joint_count_; ++i)
                solution[i] = q[joint_vel_offset_ + i];
            error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
            return true;
        }

        // Build twist
        Eigen::Matrix<double, 6, 1> twist;
        twist.head<3>() = pos_err * gain_;
        twist.tail<3>() = rot_err * gain_;

        // DLS step
        Eigen::VectorXd q_dot = computeDlsStep(q, twist);

        // Integrate
        for (size_t i = 0; i < arm_joint_count_; ++i)
        {
            size_t vidx = joint_vel_offset_ + i;
            q[vidx] = std::clamp(
                q[vidx] + q_dot[vidx] * dt_,
                joint_lower_limits_[i],
                joint_upper_limits_[i]);
        }
    }

    error_code.val = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
    return false;
}

// ================================================================
// Single DLS step (reuses the same math as CartesianController)
// ================================================================

Eigen::VectorXd PinocchioKinematicsPlugin::computeDlsStep(
    const Eigen::VectorXd& q,
    const Eigen::Matrix<double, 6, 1>& twist) const
{
    using namespace Eigen;

    const int nv = model_->nv;
    Matrix<double, 6, Dynamic> J(6, nv);
    J.setZero();

    pinocchio::computeFrameJacobian(
        *model_, *data_, q, ee_frame_id_,
        pinocchio::LOCAL_WORLD_ALIGNED, J);

    // Adaptive damping based on manipulability
    Matrix<double, 6, 6> JJt = J * J.transpose();
    double det = JJt.determinant();
    double mu = (det > 0.0) ? std::sqrt(std::abs(det)) : 0.0;
    double lambda = lambda_base_ * std::max(1.0, mu_threshold_ / (mu + 1e-8));
    lambda = std::clamp(lambda, lambda_min_, lambda_max_);

    // DLS: q_dot = J^T * (J*J^T + lambda^2 * I)^(-1) * twist
    JJt.diagonal() += VectorXd::Constant(6, lambda * lambda);
    VectorXd q_dot = J.transpose() * JJt.ldlt().solve(twist);

    return q_dot;
}

}  // namespace arm_hardware_interface

PLUGINLIB_EXPORT_CLASS(
    arm_hardware_interface::PinocchioKinematicsPlugin,
    kinematics::KinematicsBase)
