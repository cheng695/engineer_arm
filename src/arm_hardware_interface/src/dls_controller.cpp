#include "arm_hardware_interface/dls_controller.hpp"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/SVD>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace arm_hardware_interface
{

void DlsController::Init(const pinocchio::Model& model,
                         const std::string& terminal_frame_name,
                         double dt)
{
  model_ = &model;
  dt_ = dt;

  terminal_frame_name_ = model.getFrameId(terminal_frame_name);
  if (terminal_frame_name_ >= static_cast<size_t>(model.nframes))
  {
    throw std::runtime_error("DlsController: frame '" + terminal_frame_name + "' not found");
  }
  
  data_ = std::make_unique<pinocchio::Data>(model);
  n_joints_   = static_cast<size_t>(model.nq);
  vel_offset_ = static_cast<int>(model.nv) - static_cast<int>(model.nq);
  vel_last_ = Eigen::VectorXd::Zero(model.nv);
  pos_tar_  = Eigen::VectorXd::Zero(model.nv);
  pos_tar_initialized_ = false;
}

void DlsController::SyncPositions(const std::vector<double>& pos)
{
  for (size_t i = 0; i < std::min(n_joints_, pos.size()); ++i) {
    int idx = vel_offset_ + static_cast<int>(i);
    pos_tar_[idx] = pos[i];
  }
  pos_tar_initialized_ = true;
}

std::vector<DlsController::Output>
DlsController::Update(const std::array<double, 6>& target,
                         const std::vector<double>& pos_ref,
                         const std::vector<double>& vel_ref)
{
  if (!model_) 
  {
    throw std::runtime_error("DlsController not initialized");
  }
  
  // ---- 反馈值的传参 ----
  const size_t nv = static_cast<size_t>(model_->nv);    // 速度维度总数
  Eigen::Map<const Eigen::VectorXd> q_eigen(pos_ref.data(), n_joints_);
  Eigen::VectorXd q_full = Eigen::VectorXd::Zero(nv);
  q_full.tail(n_joints_) = q_eigen; // 所有关节的当前位置
  // ---- 期望值的传参 ----
  Eigen::Matrix<double, 6, 1> twist_local;
  twist_local << target[0], target[1], target[2], target[3], target[4], target[5];

  // ---- 1. FK + Jacobian (LOCAL) ----
  pinocchio::forwardKinematics(*model_, *data_, q_full);  // 求雅可比用
  pinocchio::updateFramePlacements(*model_, *data_);

  Eigen::Matrix<double, 6, Eigen::Dynamic> J(6, nv);
  J.setZero();
  pinocchio::computeFrameJacobian(*model_, *data_, q_full, terminal_frame_name_,
                                  pinocchio::LOCAL, J); //求的是速度雅可比

  // ---- 2. SVD: σ_min, σ_max, cond奇异程度 ----
  Eigen::JacobiSVD<Eigen::Matrix<double, 6, Eigen::Dynamic>> svd(
      J, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const Eigen::VectorXd& sigma = svd.singularValues();
  double sigma_min = sigma.tail(1)(0);
  double sigma_max = sigma(0);
  double cond = (sigma_min > 1e-12) ? sigma_max / sigma_min : 1e6; // 奇异程度

  // ---- 4. 连续自适应 λ = λ₀/(σ_min + ε) ----
  constexpr double lambda0 = 0.005;
  constexpr double eps_lambda = 0.005;
  double lambda = lambda0 / (sigma_min + eps_lambda);

  // ---- 5. DLS: q̇ = Jᵀ·(J·Jᵀ + λ²·I)⁻¹·x_dot ----
  Eigen::Matrix<double, 6, 6> JJt = J * J.transpose();
  JJt.diagonal().array() += lambda * lambda;

  Eigen::LDLT<Eigen::Matrix<double, 6, 6>> ldlt(JJt);
  Eigen::VectorXd qd_tar = Eigen::VectorXd::Zero(nv);

  if (ldlt.info() == Eigen::Success) 
  {
    Eigen::Matrix<double, 6, 1> y = ldlt.solve(twist_local);
    qd_tar = J.transpose() * y; // 最终关节目标速度
  } 
  else 
  {
    RCLCPP_ERROR_ONCE(rclcpp::get_logger("DLS"), "LDLT failed");
  }


  // ---- 6. cos 方向保险 ----
  Eigen::Matrix<double, 6, 1> v_achieved = J * qd_tar; // 计算末端速度
  double tw_n = twist_local.norm(); // 期望速度大小
  double ach_n = v_achieved.norm(); // 实际速度大小
  if (tw_n > 0.01 && ach_n > 1e-12) 
  {
    double cos_a = twist_local.dot(v_achieved) / (tw_n * ach_n);
    if (cos_a < 0.0f)  
    {
      qd_tar.setZero();
      RCLCPP_WARN_ONCE(rclcpp::get_logger("DLS"),
        "direction guard: cos=%.3f", cos_a);
    }
  }

  // ---- 7. 关节限位预测 margin ----
  constexpr double margin = 0.05;   // 边距
  for (size_t i = 0; i < n_joints_; ++i) 
  {
    int idx = vel_offset_ + static_cast<int>(i);
    double lo = model_->lowerPositionLimit[idx];
    double hi = model_->upperPositionLimit[idx];
    // 到达边界时，速度置零
    if (lo > -1e10 && q_full[idx] < lo + margin && qd_tar[idx] < 0.0) qd_tar[idx] = 0.0;
    if (hi <  1e10 && q_full[idx] > hi - margin && qd_tar[idx] > 0.0) qd_tar[idx] = 0.0;
  }

  // ---- 8. 加速度限制 ----
  constexpr double a_max = 30.0;
  double dq_max = a_max * dt_;  // 最大速度0.06 rad/s
  for (size_t i = 0; i < n_joints_; ++i) 
  {
    int idx = vel_offset_ + static_cast<int>(i);
    double dq = qd_tar[idx] - vel_last_[idx];
    dq = std::clamp(dq, -dq_max, dq_max);
    qd_tar[idx] = vel_last_[idx] + dq;
  }
  vel_last_ = qd_tar;

  // ---- 9. 整体速度缩放 ----
  constexpr double max_qd = 6.0;
  double scale = 1.0;
  for (size_t i = 0; i < n_joints_; ++i) 
  {
    int idx = vel_offset_ + static_cast<int>(i);
    double s = std::abs(qd_tar[idx]) / max_qd; // 缩放比例
    if (s > scale) 
    {
        scale = s;
    }
  }
  if (scale > 1.0) qd_tar /= scale;

  // ---- 10. 位置累积：pos_tar += qd_tar·dt（跨帧累积，力矩更大） ----
  if (!pos_tar_initialized_) {
    pos_tar_ = q_full;
    pos_tar_initialized_ = true;
  }
  pos_tar_ += qd_tar * dt_;
  // 关节限位 clamp
  for (size_t i = 0; i < n_joints_; ++i) {
    int idx = vel_offset_ + static_cast<int>(i);
    double lo = model_->lowerPositionLimit[idx];
    double hi = model_->upperPositionLimit[idx];
    if (lo > -1e10) pos_tar_[idx] = std::max(pos_tar_[idx], lo);
    if (hi <  1e10) pos_tar_[idx] = std::min(pos_tar_[idx], hi);
  }

  // ---- 11. 诊断（每 50 周期 ≈ 10Hz） ----
  //
  // 奇异指标判断标准：
  //   σ_min > 0.05  → 正常    σ_min < 0.01  → 接近奇异
  //   cond  < 100   → 正常    cond  > 1000  → 严重奇异
  //   λ ≈ 0 → 远离奇异        λ > 0.5 → 阻尼很大，末端慢
  //   scale = 1.0 → 正常      scale > 2.0 → 关节速度被压缩
  static int diag_n = 0;
  if (++diag_n % 50 == 0) 
  {
    RCLCPP_INFO(rclcpp::get_logger("DLS"),
        "σ=[%.4f %.4f] cond=%.0f λ=%.4f scale=%.2f tw=[%.2f %.2f %.2f | %.2f %.2f %.2f] ach=[%.2f %.2f %.2f | %.2f %.2f %.2f]",
        sigma_min, sigma_max, cond, lambda, scale,
        twist_local[0], twist_local[1], twist_local[2],
        twist_local[3], twist_local[4], twist_local[5],
        v_achieved[0], v_achieved[1], v_achieved[2],
        v_achieved[3], v_achieved[4], v_achieved[5]);
    // 逐关节：目标位置/速度，反馈位置/速度
    std::ostringstream oss;
    oss << "J: ";
    for (size_t i = 0; i < n_joints_; ++i) 
    {
      int idx = vel_offset_ + static_cast<int>(i);
      double qd_ref = (i < vel_ref.size()) ? vel_ref[i] : 0.0;
      oss << "J" << (i+1) << ":"
          << " pos_tar=" << std::fixed << std::setprecision(3) << pos_tar_[idx]
          << " vel_tar=" << qd_tar[idx]
          << " pos_ref=" << q_full[idx]
          << " vel_ref=" << qd_ref
          << " | ";
    }
    RCLCPP_INFO(rclcpp::get_logger("DLS"), "%s", oss.str().c_str());
  }

  // ---- 12. 输出 ----
  std::vector<Output> out;
  out.reserve(n_joints_);
  for (size_t i = 0; i < n_joints_; ++i) 
  {
    int idx = vel_offset_ + static_cast<int>(i);
    out.push_back({pos_tar_[idx], qd_tar[idx], 0.0});
  }
  return out;
}

}  // namespace arm_hardware_interface
