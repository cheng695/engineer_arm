#include "dls/dls_solver.hpp"

#include <algorithm>
#include <stdexcept>

#include <Eigen/Cholesky>
#include <Eigen/SVD>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <rclcpp/rclcpp.hpp>

namespace my_robot_controllers
{

void DlsSolver::Init(
  const pinocchio::Model& model,
  const std::string& terminal_frame_name,
  const std::vector<std::string>& joint_names,
  double dt)
{
    // Init() 仅在 controller 配置阶段调用，用来准备 Update() 所需的
    // Pinocchio 运行数据、末端 frame 编号和关节索引映射。
    if (joint_names.empty()) 
    {
        throw std::runtime_error("DlsSolver: no controlled joints");
    }
    if (!model.existFrame(terminal_frame_name)) 
    {
        throw std::runtime_error(
        "DlsSolver: frame '" + terminal_frame_name + "' not found");
    }

    // 求解器只保存 Model 的指针，不复制模型；Model 的生命周期由 controller 管理。
    model_ = &model;

    // Data 是 Pinocchio 的运行时缓存，供后续 FK 和 Jacobian 计算反复复用。
    data_ = std::make_unique<pinocchio::Data>(model);

    // 保存末端 frame 的编号，Update() 将使用它计算末端速度 Jacobian。
    terminal_frame_name_ = model.getFrameId(terminal_frame_name);

    // 保存控制周期，用于限制相邻周期之间的关节速度变化。
    dt_ = std::max(1e-6, dt);
    n_joints_ = joint_names.size();
    // 建立 controller 关节顺序到 Pinocchio q/v 向量索引的映射。
    controlled_q_indices_.clear();
    controlled_v_indices_.clear();
    controlled_q_indices_.reserve(n_joints_);
    controlled_v_indices_.reserve(n_joints_);

    for (const auto& name : joint_names) 
    {
        const auto id = model.getJointId(name);
        if (id == 0 || id >= model.joints.size() ||
            model.joints[id].nq() != 1 || model.joints[id].nv() != 1) 
        {
            throw std::runtime_error("DlsSolver: invalid one-DOF joint '" + name + "'");
        }
        // q 保存关节位置，v 保存关节速度，两者的索引需要分别记录。
        if (model.joints[id].idx_q() >= model.nq || model.joints[id].idx_v() >= model.nv)
            throw std::runtime_error("DlsSolver: joint index out of range '" + name + "'");
        controlled_q_indices_.push_back(model.joints[id].idx_q());
        controlled_v_indices_.push_back(model.joints[id].idx_v());
    }
    // 第一周期没有上一帧命令速度，因此从零开始。
    vel_last_ = Eigen::VectorXd::Zero(model.nv);
}

/**
 * @brief 获取关节位置索引
 * 
 * @param i 
 * @return int 
 */
int DlsSolver::qIndex(size_t i) const
{
    return controlled_q_indices_[i];
}

/**
 * @brief 获取关节速度索引
 * 
 * @param i 
 * @return int 
 */
int DlsSolver::vIndex(size_t i) const
{
    return controlled_v_indices_[i];
}

/**
 * @brief DLS  求解器
 * 
 * @param target 
 * @param pos_ref 
 * @param vel_ref 
 * @return std::vector<DlsSolver::Output> 
 */
std::vector<DlsSolver::Output> DlsSolver::Update(
  const std::array<double, 6>& target,
  const std::vector<double>& pos_ref,
  double dt)
{
    if (!model_ || !data_) 
    {
        throw std::runtime_error("DlsSolver not initialized");
    }

    // 获取当前关节位置
    const size_t nq = static_cast<size_t>(model_->nq);
    const size_t nv = static_cast<size_t>(model_->nv);
    Eigen::VectorXd q = Eigen::VectorXd::Zero(nq);
    for (size_t i = 0; i < n_joints_; ++i) {
        q[qIndex(i)] = i < pos_ref.size() ? pos_ref[i] : 0.0;
    }

    // FK 计算各个关节位姿 和 frame位姿
    pinocchio::forwardKinematics(*model_, *data_, q);
    pinocchio::updateFramePlacements(*model_, *data_);

    // 创建雅可比矩阵
    Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian(
        6, static_cast<Eigen::Index>(nv));
    jacobian.setZero();

    // 计算当前位姿下，末端frame的速度雅可比
    pinocchio::computeFrameJacobian(
        *model_, *data_, q, terminal_frame_name_, pinocchio::LOCAL, jacobian);
    
    // 将期望值转化为向量
    Eigen::Matrix<double, 6, 1> twist;
    for (size_t i = 0; i < 6; ++i) 
    {
        twist[static_cast<Eigen::Index>(i)] = target[i];
    }

    // 创建一个svd变量并对雅可比矩阵做奇异值分解
    const Eigen::JacobiSVD<Eigen::Matrix<double, 6, Eigen::Dynamic>> svd(
        jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
    // 计算奇异值
    const auto& singular_values = svd.singularValues();
    // 获取最小奇异值
    const double sigma_min = singular_values.size() == 0 ? 0.0 : singular_values.tail(1)[0];
    // 计算动态阻尼
    constexpr double lambda0 = 0.005;
    constexpr double lambda_epsilon = 0.005;
    const double lambda = lambda0 / (sigma_min + lambda_epsilon);

    sigma_min_ = sigma_min;
    damping_ = lambda;
    //  计算 q̇ = Jᵀ (J Jᵀ + λ²I)⁻¹ ẋ
    Eigen::Matrix<double, 6, 6> regularized = jacobian * jacobian.transpose();
    regularized.diagonal().array() += lambda * lambda;
    const auto ldlt = regularized.ldlt();
    Eigen::VectorXd qdot = Eigen::VectorXd::Zero(nv);
    if (ldlt.info() == Eigen::Success) 
    {
        qdot.noalias() = jacobian.transpose() * ldlt.solve(twist);
    }

    // 对 DLS 输出的关节速度进行安全限制：限制加速度、最大速度和关节位置限位。
    // 单位分别为 rad/s²、rad/s 和 rad。
    constexpr double max_acceleration = 30.0;
    constexpr double max_velocity = 6.0;
    constexpr double limit_margin = 0.05;

    // 一个控制周期内允许的最大速度变化量：Δq̇_max = a_max * dt。
    const double max_delta = max_acceleration * std::max(1e-6, dt);

    std::vector<Output> output;
    output.reserve(n_joints_);
    // 先限制每个关节的速度，再判断整组速度是否仍能实现期望末端运动。
    // 任一关节被限位方向阻挡时，整组关节停止，避免末端轨迹发生变形。
    bool limit_blocked = false;
    blocked_joint_ = -1;
    blocked_at_upper_limit_ = false;
    // 对比限幅前后的实现比例，便于识别正常加速被误判为受限的情况。
    const double target_squared_norm = twist.squaredNorm();
    raw_tracking_ratio_ = target_squared_norm > 1e-12
        ? twist.dot(jacobian * qdot) / target_squared_norm : 1.0;

    for (size_t i = 0; i < n_joints_; ++i) 
    {
        const int qi = qIndex(i);
        const int vi = vIndex(i);

        // 速度变化不能超过加速度限制，避免关节速度指令突变。
        double velocity = std::clamp(
            qdot[vi], 
            vel_last_[vi] - max_delta, 
            vel_last_[vi] + max_delta);

        // 限制关节速度绝对值，防止输出超过最大速度。
        velocity = std::clamp(velocity, -max_velocity, max_velocity);
        qdot[vi] = velocity;

        // 接近下限且速度指向下限时，标记整组运动被限位阻挡。
        if (model_->lowerPositionLimit[qi] > -1e10 &&
                q[qi] <= model_->lowerPositionLimit[qi] + limit_margin && velocity < 0.0) 
        {
            limit_blocked = true;
            if (blocked_joint_ < 0)
                blocked_joint_ = static_cast<int>(i);
        }

        // 接近上限且速度指向上限时，标记整组运动被限位阻挡。
        if (model_->upperPositionLimit[qi] < 1e10 &&
                q[qi] >= model_->upperPositionLimit[qi] - limit_margin && velocity > 0.0) 
        {
            limit_blocked = true;
            if (blocked_joint_ < 0)
            {
                blocked_joint_ = static_cast<int>(i);
                blocked_at_upper_limit_ = true;
            }
        }
    }

    // 用经过速度和加速度限制后的关节速度计算实际可实现的末端 Twist。
    const Eigen::Matrix<double, 6, 1> achieved_twist = jacobian * qdot;
    const double tracking_ratio = target_squared_norm > 1e-12
        ? twist.dot(achieved_twist) / target_squared_norm : 1.0;

    // 用原始 DLS 解判断任务是否可实现。加速和换向期间的限幅后比例
    // 仅用于诊断，不能据此清零，否则每帧从零起步可能导致永久受限。
    constexpr double minimum_tracking_ratio = 0.1;
    const bool task_blocked = target_squared_norm > 1e-12 &&
        raw_tracking_ratio_ < minimum_tracking_ratio;
    tracking_ratio_ = tracking_ratio;
    task_blocked_ = task_blocked;
    
    blocked_ = limit_blocked || task_blocked;
    if (blocked_) 
    {
        qdot.setZero();

    }

    for (size_t i = 0; i < n_joints_; ++i) 
    {
        const int qi = qIndex(i);
        const int vi = vIndex(i);
        // 保存本周期最终速度，作为下一周期的加速度限制参考值。
        vel_last_[vi] = qdot[vi];
        // 输出当前关节位置、限制后的目标速度和未使用的力矩值。
        output.push_back({q[qi], qdot[vi], 0.0});
    }
    return output;
}

}  // namespace my_robot_controllers
