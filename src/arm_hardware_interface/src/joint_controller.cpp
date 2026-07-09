#include "arm_hardware_interface/joint_controller.hpp"

#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace arm_hardware_interface
{

void JointController::Init(size_t n_joints, double dt)
{
  n_joints_ = n_joints;
  dt_ = dt;
  vel_last_ = Eigen::VectorXd::Zero(n_joints_);
  pos_tar_  = Eigen::VectorXd::Zero(n_joints_);
}

void JointController::SyncPositions(const std::vector<double>& pos)
{
  for (size_t i = 0; i < std::min(n_joints_, pos.size()); ++i)
    pos_tar_[i] = pos[i];
}

std::vector<JointController::Output>
JointController::Update(const std::vector<double>& vel_target,
                         const std::vector<double>& pos_ref,
                         const std::vector<double>& joint_lower,
                         const std::vector<double>& joint_upper)
{
  Eigen::Map<const Eigen::VectorXd> qd_in(vel_target.data(), n_joints_);
  Eigen::Map<const Eigen::VectorXd> q_fb(pos_ref.data(), n_joints_);

  Eigen::VectorXd qd_tar = qd_in;
  Eigen::VectorXd q_ref = q_fb;

  // ---- 1. 关节限位预测 ----
  constexpr double margin = 0.05;
  for (size_t i = 0; i < n_joints_; ++i) 
  {
    double lo = joint_lower[i], hi = joint_upper[i];
    if (lo > -1e10 && q_ref[i] < lo + margin && qd_tar[i] < 0.0) qd_tar[i] = 0.0;
    if (hi <  1e10 && q_ref[i] > hi - margin && qd_tar[i] > 0.0) qd_tar[i] = 0.0;
  }

  // ---- 2. 加速度限制 ----
  constexpr double a_max = 30.0;
  double dq_max = a_max * dt_;
  for (size_t i = 0; i < n_joints_; ++i) 
  {
    double dq = qd_tar[i] - vel_last_[i];
    dq = std::clamp(dq, -dq_max, dq_max);
    qd_tar[i] = vel_last_[i] + dq;
  }
  vel_last_ = qd_tar;

  // ---- 3. 整体速度缩放 ----
  constexpr double max_qd = 6.0;
  double scale = 1.0;
  for (size_t i = 0; i < n_joints_; ++i) 
  {
    double s = std::abs(qd_tar[i]) / max_qd;
    if (s > scale) scale = s;
  }
  if (scale > 1.0) qd_tar /= scale;

  // ---- 4. 积分：pos_tar += qd_tar * dt ----
  pos_tar_ += qd_tar * dt_;

  // 关节限位 clamp
  for (size_t i = 0; i < n_joints_; ++i) 
  {
    double lo = joint_lower[i], hi = joint_upper[i];
    if (lo > -1e10) pos_tar_[i] = std::max(pos_tar_[i], lo);
    if (hi <  1e10) pos_tar_[i] = std::min(pos_tar_[i], hi);
  }

  // ---- 5. 诊断 ----
  static int diag_n = 0;
  if (++diag_n % 50 == 0) 
  {
    std::ostringstream oss;
    oss << "J: ";
    for (size_t i = 0; i < n_joints_; ++i) 
    {
      oss << "J" << (i+1) << ":"
          << " pos_tar=" << std::fixed << std::setprecision(3) << pos_tar_[i]
          << " qd_tar=" << qd_tar[i]
          << " pos_ref=" << q_ref[i]
          << " | ";
    }
    RCLCPP_INFO(rclcpp::get_logger("JointCtl"), "%s", oss.str().c_str());
  }

  // ---- 6. 输出 ----
  std::vector<Output> out;
  out.reserve(n_joints_);
  for (size_t i = 0; i < n_joints_; ++i)
  {
    out.push_back({pos_tar_[i], qd_tar[i], 0.0});
  }
  return out;
}

}  // namespace arm_hardware_interface
