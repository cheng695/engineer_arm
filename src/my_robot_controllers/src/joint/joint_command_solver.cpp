#include "joint/joint_command_solver.hpp"

#include <algorithm>
#include <stdexcept>

namespace my_robot_controllers
{

void JointCommandSolver::Init(std::size_t joint_count, double dt)
{
    if (joint_count == 0) 
    {
        throw std::runtime_error("JointCommandSolver: 关节数量不能为零");
    }
    joint_count_ = joint_count;
    dt_ = std::max(1e-6, dt);
    last_velocity_    = Eigen::VectorXd::Zero(joint_count_);
    target_positions_ = Eigen::VectorXd::Zero(joint_count_);
}

/**
 * @brief 同步当前关节位置
 *
 * @param positions
 */
void JointCommandSolver::SyncPositions(const std::vector<double>& positions)
{
    if (target_positions_.size() == 0) 
    {
        return;
    }
    for (std::size_t i = 0; i < joint_count_ && i < positions.size(); ++i) 
    {
        target_positions_[static_cast<Eigen::Index>(i)] = positions[i];
    }
}

/**
 * @brief 更新
 *
 * @param velocity_command
 * @param positions
 * @param lower_limits
 * @param upper_limits
 * @param dt
 * @return std::vector<JointCommandSolver::Output>
 */
std::vector<JointCommandSolver::Output> JointCommandSolver::Update(
    const std::vector<double>& velocity_command,
    const std::vector<double>& positions,
    const std::vector<double>& lower_limits,
    const std::vector<double>& upper_limits,
    double dt)
{
    if (joint_count_ == 0) 
    {
        throw std::runtime_error("JointCommandSolver 未初始化");
    }
    if (positions.size() < joint_count_ ||
        lower_limits.size() < joint_count_ || upper_limits.size() < joint_count_) 
    {
        throw std::runtime_error("JointCommandSolver 输入长度不足");
    }

    const double safe_dt = std::max(1e-6, dt > 0.0 ? dt : dt_);
    constexpr double max_acceleration = 30.0;
    constexpr double max_velocity = 6.0;
    constexpr double margin = 0.05;
    const double max_delta = max_acceleration * safe_dt;

    std::vector<Output> output;
    output.reserve(joint_count_);
    
    for (std::size_t i = 0; i < joint_count_; ++i) 
    {
        const double requested = i < velocity_command.size() ? velocity_command[i] : 0.0;
        double velocity = std::clamp(
        requested,
        last_velocity_[static_cast<Eigen::Index>(i)] - max_delta,
        last_velocity_[static_cast<Eigen::Index>(i)] + max_delta);
        velocity = std::clamp(velocity, -max_velocity, max_velocity);

        if (lower_limits[i] > -1e10 && positions[i] <= lower_limits[i] + margin && velocity < 0.0) 
        {
            velocity = 0.0;
        }
        if (upper_limits[i] < 1e10 && positions[i] >= upper_limits[i] - margin && velocity > 0.0) 
        {
            velocity = 0.0;
        }

        target_positions_[static_cast<Eigen::Index>(i)] += velocity * safe_dt;
        if (lower_limits[i] > -1e10) 
        {
            target_positions_[static_cast<Eigen::Index>(i)] = std::max(
                target_positions_[static_cast<Eigen::Index>(i)], lower_limits[i]);
        }
        if (upper_limits[i] < 1e10) 
        {
            target_positions_[static_cast<Eigen::Index>(i)] = std::min(
                target_positions_[static_cast<Eigen::Index>(i)], upper_limits[i]);
        }

        last_velocity_[static_cast<Eigen::Index>(i)] = velocity;
        output.push_back({target_positions_[static_cast<Eigen::Index>(i)], velocity});
    }
    return output;
}

}  // namespace my_robot_controllers
