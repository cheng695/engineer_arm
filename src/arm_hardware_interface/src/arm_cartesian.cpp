#include "arm_hardware_interface/cartesian_controller.hpp"

#include <algorithm>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <Eigen/Dense>

namespace arm_hardware_interface
{

bool CartesianController::initialize(const pinocchio::Model& model,
                                     const std::string& ee_frame,
                                     const rclcpp::Logger& logger)
{
    model_ = &model;
    initialized_ = false;

    if (!model_->existFrame(ee_frame))
    {
        RCLCPP_ERROR(logger,
            "末端 frame '%s' 不存在于模型中，笛卡尔 IK 不可用",
            ee_frame.c_str());
        return false;
    }

    ee_frame_id_ = model_->getFrameId(ee_frame);
    data_ = std::make_unique<pinocchio::Data>(*model_);

    // 查找关节速度偏移量（跳过虚拟基座 DOF）
    for (int j = 1; j < model_->njoints; ++j)
    {
        if (model_->names[static_cast<size_t>(j)] == "joint1")
        {
            joint_vel_offset_ = static_cast<size_t>(model_->joints[j].idx_v());
            break;
        }
    }

    initialized_ = true;

    RCLCPP_INFO(logger,
        "笛卡尔 IK 初始化成功: EE='%s' (id=%d), nv=%d, offset=%zu",
        ee_frame.c_str(), ee_frame_id_, model_->nv, joint_vel_offset_);

    return true;
}

std::vector<double> CartesianController::solve(
    const std::array<double, 6>& twist,
    const std::vector<double>& joint_positions)
{
    if (!initialized_ || !model_ || !data_)
    {
        return {};
    }

    // 构造 q，末尾补零（适配 nq > 实际关节数的情况）
    Eigen::VectorXd q = Eigen::VectorXd::Zero(model_->nq);
    for (size_t i = 0; i < std::min(static_cast<size_t>(model_->nq), joint_positions.size()); ++i)
    {
        q[i] = joint_positions[i];
    }

    // 构造 twist
    Eigen::Matrix<double, 6, 1> twist_eigen;
    for (size_t i = 0; i < 6; ++i)
    {
        twist_eigen[i] = twist[i];
    }

    // 用全局 logger（initialize 时未保存 logger，这里用通用 logger）
    Eigen::VectorXd joint_vels = compute_dls(
        q, twist_eigen, rclcpp::get_logger("CartesianController"));

    // 提取臂关节速度（跳过虚拟基座）
    const size_t arm_joints = model_->nv - joint_vel_offset_;
    std::vector<double> result(arm_joints, 0.0);
    for (size_t i = 0; i < arm_joints; ++i)
    {
        Eigen::Index src = static_cast<Eigen::Index>(joint_vel_offset_ + i);
        if (src < joint_vels.size())
        {
            constexpr double kMaxJointVel = 3.0;  // rad/s
            result[i] = std::clamp(joint_vels[src], -kMaxJointVel, kMaxJointVel);
        }
    }

    return result;
}

Eigen::VectorXd CartesianController::compute_dls(
    const Eigen::VectorXd& q,
    const Eigen::Matrix<double, 6, 1>& twist,
    const rclcpp::Logger& logger)
{
    const int nv = model_->nv;

    // 1. 计算 frame Jacobian (6 x nv)
    Eigen::Matrix<double, 6, Eigen::Dynamic> J(6, nv);
    J.setZero();

    pinocchio::computeFrameJacobian(
        *model_, *data_, q, ee_frame_id_,
        pinocchio::LOCAL_WORLD_ALIGNED, J);

    // 2. 计算 manipulability μ = sqrt(det(J·Jᵀ))
    Eigen::Matrix<double, 6, 6> JJt = J * J.transpose();
    double det = JJt.determinant();
    double mu = (det > 0.0) ? std::sqrt(std::abs(det)) : 0.0;

    // 3. 自适应阻尼因子
    constexpr double kBaseLambda   = 0.1;
    constexpr double kMinLambda    = 0.01;
    constexpr double kMaxLambda    = 5.0;
    constexpr double kMuThreshold  = 0.1;
    constexpr double kEpsilon      = 1e-8;

    double lambda = kBaseLambda * std::max(1.0, kMuThreshold / (mu + kEpsilon));
    lambda = std::clamp(lambda, kMinLambda, kMaxLambda);
    last_lambda_ = lambda;

    // 4. DLS 求解: q̇ = Jᵀ · (J·Jᵀ + λ²I)⁻¹ · twist
    JJt.diagonal() += Eigen::VectorXd::Constant(6, lambda * lambda);
    Eigen::VectorXd q_dot = J.transpose() * JJt.ldlt().solve(twist);

    // 5. 周期性遥测
    static int count = 0;
    if (count++ % 200 == 0)
    {
        RCLCPP_INFO(logger,
            "[DLS] mu=%.4f lambda=%.3f "
            "|twist|=[%.3f %.3f %.3f | %.3f %.3f %.3f] "
            "|q_dot|=[%.2f %.2f %.2f %.2f %.2f %.2f %.2f]",
            mu, lambda,
            twist[0], twist[1], twist[2], twist[3], twist[4], twist[5],
            q_dot[0], q_dot[1], q_dot[2], q_dot[3], q_dot[4], q_dot[5], q_dot[6]);
    }

    return q_dot;
}

}  // namespace arm_hardware_interface
