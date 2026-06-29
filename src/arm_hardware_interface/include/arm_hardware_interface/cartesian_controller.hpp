#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <rclcpp/rclcpp.hpp>

namespace arm_hardware_interface
{

/**
 * @brief 笛卡尔空间 DLS IK 控制器。
 *
 * 使用 Pinocchio 计算末端 Jacobian，通过 Damped Least-Squares 将
 * 末端 Twist 转为关节速度。阻尼因子根据 manipulability 自适应调节，
 * 在奇异点附近自动抑制关节速度暴涨。
 *
 * Pinocchio Model 通过 initialize() 传入引用（不持有所有权），
 * 应与 GravityCompensator 共享同一个 Model。
 */
class CartesianController
{
public:
    CartesianController() = default;

    // ================================================================
    // 初始化
    // ================================================================

    /**
     * @brief 绑定 Pinocchio 模型并查找末端 frame。
     * @param[in] model    已加载的 Pinocchio 模型（不持有所有权）
     * @param[in] ee_frame 末端执行器 frame 名称，如 "gripper_center"
     * @param[in] logger   日志输出
     * @return true 末端 frame 存在，初始化成功
     */
    bool initialize(const pinocchio::Model& model,
                    const std::string& ee_frame,
                    const rclcpp::Logger& logger);

    /**
     * @brief 是否已成功初始化。
     */
    bool is_initialized() const { return initialized_; }

    /**
     * @brief 虚拟基座的速度 DOF 偏移量（Jacobian 列偏移）。
     */
    size_t joint_vel_offset() const { return joint_vel_offset_; }

    /**
     * @brief 最近一次求解的阻尼因子（用于遥测）。
     */
    double last_lambda() const { return last_lambda_; }

    // ================================================================
    // 求解
    // ================================================================

    /**
     * @brief 将末端 Twist 转为关节速度。
     *
     * DLS 公式: q̇ = Jᵀ·(J·Jᵀ + λ²I)⁻¹·twist
     *
     * @param[in] twist            末端速度 [vx,vy,vz, wx,wy,wz]，已缩放
     * @param[in] joint_positions  当前关节位置弧度，长度须 ≥ 实际臂关节数
     * @return 臂关节速度 rad/s（已去掉虚拟基座 DOF），未初始化返回空 vector
     */
    std::vector<double> solve(const std::array<double, 6>& twist,
                              const std::vector<double>& joint_positions);

private:
    // 内部 DLS 求解（Eigen 版本）
    Eigen::VectorXd compute_dls(const Eigen::VectorXd& q,
                                const Eigen::Matrix<double, 6, 1>& twist,
                                const rclcpp::Logger& logger);

    const pinocchio::Model* model_{nullptr};  ///< 共享模型（不持有所有权）
    std::unique_ptr<pinocchio::Data> data_;   ///< Pinocchio 运行时缓存

    int ee_frame_id_{-1};              ///< 末端 frame 在模型中的 ID
    size_t joint_vel_offset_{0};       ///< 虚拟基座速度 DOF 偏移
    double last_lambda_{0.1};          ///< 最近一次阻尼因子
    bool initialized_{false};          ///< 初始化标志
};

}  // namespace arm_hardware_interface
