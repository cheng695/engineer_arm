#include "arm_hardware_interface/gravity_compensator.hpp"
#include <algorithm>

namespace arm_hardware_interface
{

bool GravityCompensator::initialize(const std::string& urdf_xml, const rclcpp::Logger& logger)
{
    initialized_ = false;
    model_.reset();
    data_.reset();

    if (urdf_xml.empty())
    {
        RCLCPP_WARN(logger, "URDF 字符串为空，重力补偿已禁用");
        return false;
    }

    try
    {
        model_ = std::make_unique<pinocchio::Model>();
        pinocchio::urdf::buildModelFromXML(urdf_xml, *model_);  // 解析urdf
        data_ = std::make_unique<pinocchio::Data>(*model_);
        initialized_ = true;

        RCLCPP_INFO(logger,
            "重力补偿器初始化成功 (nq=%d, nv=%d)",
            model_->nq, model_->nv);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(logger, "重力补偿器初始化失败: %s", e.what());
    }

    return initialized_;
}

bool GravityCompensator::initialize_from_file(const std::string& urdf_path, const rclcpp::Logger& logger)
{
    initialized_ = false;
    model_.reset();
    data_.reset();

    if (urdf_path.empty())
    {
        RCLCPP_WARN(logger, "URDF 路径为空，重力补偿已禁用");
        return false;
    }

    if (urdf_path.find(".xacro") != std::string::npos)
    {
        RCLCPP_WARN(logger, "路径指向 .xacro 文件，Pinocchio 需要 .urdf。重力补偿已禁用");
        return false;
    }

    try
    {
        model_ = std::make_unique<pinocchio::Model>();
        pinocchio::urdf::buildModel(urdf_path, *model_);
        data_ = std::make_unique<pinocchio::Data>(*model_);
        initialized_ = true;

        RCLCPP_INFO(logger,
            "重力补偿器初始化成功 (nq=%d, nv=%d)",
            model_->nq, model_->nv);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(logger, "重力补偿器初始化失败: %s", e.what());
    }

    return initialized_;
}

std::vector<double> GravityCompensator::compute(const std::vector<double>& joint_positions) const
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

    // RNEA τ = M(q)·a + C(q,v)·v + g(q) 
    // 静态计算：v=0, a=0 → tau = 重力项
    Eigen::VectorXd v = Eigen::VectorXd::Zero(model_->nv);
    Eigen::VectorXd a = Eigen::VectorXd::Zero(model_->nv);
    pinocchio::rnea(*model_, *data_, q, v, a);

    // 返回前 nv 个关节的重力力矩
    std::vector<double> gravity_effort(model_->nv);
    for (int i = 0; i < model_->nv; ++i)
    {
        gravity_effort[i] = data_->tau[i];
    }
    return gravity_effort;
}

}  // namespace arm_hardware_interface
