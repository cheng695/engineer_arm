#pragma once

#include <cstddef>
#include <vector>

#include <Eigen/Core>

namespace my_robot_controllers
{

/**
 * @brief 关节空间速度命令到位置目标的求解器。
 *
 * 对输入速度进行限位和加速度限制后积分为位置目标，供 position
 * command interface 使用。
 */
class JointCommandSolver
{
public:
    struct Output
    {
        double position;
        double velocity;
    };

    void Init(std::size_t joint_count, double dt = 0.002);

    void SyncPositions(const std::vector<double>& positions);

    std::vector<Output> Update(
        const std::vector<double>& velocity_command,
        const std::vector<double>& positions,
        const std::vector<double>& lower_limits,
        const std::vector<double>& upper_limits,
        double dt);

private:
    std::size_t joint_count_{0};
    double dt_{0.002};
    Eigen::VectorXd last_velocity_;
    Eigen::VectorXd target_positions_;
};

}  // namespace my_robot_controllers
