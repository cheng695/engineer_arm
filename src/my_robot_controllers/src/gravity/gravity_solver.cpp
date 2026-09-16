#include "gravity/gravity_solver.hpp"

#include <exception>

namespace my_robot_controllers
{

bool GravitySolver::initialize(
  const std::string& robot_description, const rclcpp::Logger& logger)
{
  initialized_ = false;
  model_.reset();
  data_.reset();
  q_indices_.clear();
  v_indices_.clear();
  effort_buffer_.clear();

  if (robot_description.empty())
  {
    RCLCPP_ERROR(logger, "robot_description 为空，无法初始化重力模型");
    return false;
  }

  try
  {
    model_ = std::make_unique<pinocchio::Model>();
    pinocchio::urdf::buildModelFromXML(robot_description, *model_);
    data_ = std::make_unique<pinocchio::Data>(*model_);
    initialized_ = true;
    RCLCPP_INFO(logger, "重力模型初始化成功 (nq=%d, nv=%d)", model_->nq, model_->nv);
  }
  catch (const std::exception& error)
  {
    RCLCPP_ERROR(logger, "重力模型初始化失败: %s", error.what());
  }
  return initialized_;
}

bool GravitySolver::bind_joints(const std::vector<std::string>& joint_names)
{
  if (!initialized_ || !model_ || joint_names.empty())
    return false;

  q_indices_.clear();
  v_indices_.clear();
  q_indices_.reserve(joint_names.size());
  v_indices_.reserve(joint_names.size());

  for (const auto& name : joint_names)
  {
    const auto joint_id = model_->getJointId(name);
    if (joint_id == 0 || joint_id >= model_->joints.size() ||
        model_->joints[joint_id].nq() != 1 || model_->joints[joint_id].nv() != 1)
    {
      initialized_ = false;
      effort_buffer_.clear();
      return false;
    }
    const auto q_index = model_->joints[joint_id].idx_q();
    const auto v_index = model_->joints[joint_id].idx_v();
    if (q_index >= model_->nq || v_index >= model_->nv)
    {
      initialized_ = false;
      effort_buffer_.clear();
      return false;
    }
    q_indices_.push_back(q_index);
    v_indices_.push_back(v_index);
  }

  effort_buffer_.assign(joint_names.size(), 0.0);
  q_buffer_ = Eigen::VectorXd::Zero(model_->nq);
  zero_buffer_ = Eigen::VectorXd::Zero(model_->nv);
  return true;
}

const std::vector<double>& GravitySolver::compute(
  const std::vector<double>& positions) const
{
  if (!initialized_ || !model_ || !data_ || positions.size() != q_indices_.size())
  {
    effort_buffer_.clear();
    return effort_buffer_;
  }

  // 速度和加速度均设为零，RNEA 输出的主要就是当前姿态的静态重力项。
  for (size_t i = 0; i < q_indices_.size(); ++i)
    q_buffer_[q_indices_[i]] = positions[i];
  pinocchio::rnea(*model_, *data_, q_buffer_, zero_buffer_, zero_buffer_);

  for (size_t i = 0; i < v_indices_.size(); ++i)
    effort_buffer_[i] = data_->tau[v_indices_[i]];
  return effort_buffer_;
}

}  // namespace my_robot_controllers
