#pragma once

#include <cstddef>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <Eigen/Core>

namespace my_robot_controllers
{

class DlsSolver
{
public:
    struct Output
    {
        double pos;   // 目标关节位置 (rad)
        double vel;   // 目标关节速度 (rad/s)
        double tor;   // 目标关节力矩 (Nm)，速度 DLS 下恒为 0
    };

    DlsSolver() = default;

  /**
   * @brief 初始化
   * @param model               Pinocchio 模型
   * @param terminal_frame_name 末端 frame 名称 (如 "tool_link")
   * @param joint_names         关节名称列表
   * @param dt                  控制周期 (秒)
   */
    void Init(const pinocchio::Model& model,
                const std::string& terminal_frame_name,
                const std::vector<std::string>& joint_names,
                double dt = 0.002);

  /** @brief 根据末端 Twist 和关节位置计算关节速度。 */
  std::vector<Output> Update(const std::array<double, 6>& target,
                              const std::vector<double>& pos_ref,
                              double dt);

    double sigma_min() const { return sigma_min_; }
    double damping() const { return damping_; }
    bool blocked() const { return blocked_; }
    /** @brief 重新接管位置指令时，从零指令速度开始加速，清除停用前的历史。 */
    void reset_velocity_history() { vel_last_.setZero(); }
    // 记录本周期首个触发限位的关节（控制器顺序，-1 表示没有）。
    int blocked_joint() const { return blocked_joint_; }
    bool blocked_at_upper_limit() const { return blocked_at_upper_limit_; }
    bool task_blocked() const { return task_blocked_; }
    double tracking_ratio() const { return tracking_ratio_; }
    double raw_tracking_ratio() const { return raw_tracking_ratio_; }
private:
    int blocked_joint_{-1};
    bool blocked_at_upper_limit_{false}, task_blocked_{false};
    double tracking_ratio_{1.0}, raw_tracking_ratio_{1.0};
    double sigma_min_{0}, damping_{0};
    bool blocked_{false};
    const pinocchio::Model* model_ = nullptr;   // Pinocchio 模型
    std::unique_ptr<pinocchio::Data> data_;     // Pinocchio 运行时数据
    size_t terminal_frame_name_ = 0;            // 末端 frame 索引
    size_t n_joints_ = 0;                       // 关节数
    double dt_ = 0.002;                         // 控制周期

    Eigen::VectorXd vel_last_;                  // 上一帧速度，用于加速度限制
    std::vector<int> controlled_q_indices_;     // 控制的关节角度索引
    std::vector<int> controlled_v_indices_;     // 控制的关节速度索引

    int qIndex(size_t i) const;
    int vIndex(size_t i) const;
};

}  // namespace my_robot_controllers
