#pragma once

#include <memory>
#include <string>
#include <vector>

#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <rclcpp/rclcpp.hpp>

namespace arm_hardware_interface
{

/**
 * @brief 基于 Pinocchio RNEA 的重力补偿器。
 *
 * 在静态假设（速度 v=0，加速度 a=0）下调用 pinocchio::rnea，
 * 其输出的 tau 即为各关节的静态重力项，用于前馈重力补偿。
 *
 * 支持三种补偿模式：
 * - Off:        不补偿
 * - Assist:     在前馈力矩上叠加补偿
 * - GravityOnly: 纯重力模式（kp=kd=0，仅重力项）
 */
class GravityCompensator
{
public:
    enum class Mode
    {
        Off = 0,
        Assist = 1,
        GravityOnly = 2,
    };

    GravityCompensator() = default;

    // ================================================================
    // 初始化
    // ================================================================

    /**
     * @brief 从 URDF XML 字符串初始化 Pinocchio 模型。
     * @param[in] urdf_xml 完整的 URDF 字符串（通常来自 robot_description 参数）
     * @param[in] logger   用于日志输出的 logger
     * @return true 成功
     */
    bool initialize(const std::string& urdf_xml, const rclcpp::Logger& logger);

    /**
     * @brief 从 .urdf 文件路径初始化 Pinocchio 模型。
     * @param[in] urdf_path URDF 文件路径（不支持 .xacro）
     * @param[in] logger    用于日志输出的 logger
     * @return true 成功
     */
    bool initialize_from_file(const std::string& urdf_path, const rclcpp::Logger& logger);

    /**
     * @brief 查询是否已成功初始化。
     */
    bool is_initialized() const { return initialized_; }

    /// @brief 模型总速度维度（含虚拟基座 DOF），通常 > 实际臂关节数
    size_t nv() const { return model_ ? model_->nv : 0; }

    /**
     * @brief 实际臂关节数（去掉虚拟基座 DOF）。
     * @param[in] base_offset 虚拟基座占用的速度 DOF 数量
     */
    size_t joint_count(size_t base_offset = 0) const
    {
        return model_ ? model_->nv - base_offset : 0;
    }

    // ================================================================
    // 计算
    // ================================================================

    /**
     * @brief 根据当前关节角计算静态重力力矩。
     * @param[in] joint_positions 关节位置弧度，长度须 ≥ nv
     * @return 各关节重力力矩 Nm，长度 = nv。未初始化返回空 vector
     */
    std::vector<double> compute(const std::vector<double>& joint_positions) const;

    // ================================================================
    // 模型共享（供 CartController 等复用）
    // ================================================================

    const pinocchio::Model& model() const { return *model_; }
    pinocchio::Data& data() { return *data_; }

    // ================================================================
    // 补偿模式
    // ================================================================

    void set_mode(Mode mode) { mode_ = mode; }
    Mode mode() const { return mode_; }

private:
    std::unique_ptr<pinocchio::Model> model_;
    std::unique_ptr<pinocchio::Data> data_;
    Mode mode_{Mode::Off};
    bool initialized_{false};
};

}  // namespace arm_hardware_interface
