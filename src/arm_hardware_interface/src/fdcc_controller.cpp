#include "arm_hardware_interface/fdcc_controller.hpp"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace arm_hardware_interface
{

void FdccController::initialize(const pinocchio::Model& model,
                                const std::string& ee_frame,
                                double force_scale_lin,
                                double force_scale_ang,
                                double dt,
                                int locked_joint)
{
  model_           = &model;
  force_scale_lin_ = force_scale_lin;
  force_scale_ang_ = force_scale_ang;
  dt_              = dt;
  locked_joint_    = locked_joint;

  ee_frame_id_ = model.getFrameId(ee_frame);
  if (ee_frame_id_ >= static_cast<size_t>(model.nframes))
    throw std::runtime_error("FdccController: frame '" + ee_frame + "' not found");

  data_ = std::make_unique<pinocchio::Data>(model);

  vel_offset_ = static_cast<int>(model.nv) - static_cast<int>(model.nq);
  n_joints_   = static_cast<size_t>(model.nq);
}

std::vector<FdccController::Output>
FdccController::compute(const std::array<double, 6>& twist,
                         const std::vector<double>& q,
                         const std::vector<double>& qd)
{
  if (!model_) throw std::runtime_error("FdccController not initialized");

  const size_t nv = static_cast<size_t>(model_->nv);

  // ---- Eigen 映射 ----
  Eigen::Map<const Eigen::VectorXd> q_eigen(q.data(), n_joints_);
  Eigen::Map<const Eigen::VectorXd> qd_eigen(qd.data(), n_joints_);

  Eigen::VectorXd q_full  = Eigen::VectorXd::Zero(nv);
  Eigen::VectorXd qd_full = Eigen::VectorXd::Zero(nv);
  q_full.tail(n_joints_)  = q_eigen;
  qd_full.tail(n_joints_) = qd_eigen;

  // ---- 1. FK + Jacobian ----
  pinocchio::forwardKinematics(*model_, *data_, q_full);
  pinocchio::updateFramePlacements(*model_, *data_);

  Eigen::Matrix<double, 6, Eigen::Dynamic> J(6, nv);
  J.setZero();
  pinocchio::computeFrameJacobian(*model_, *data_, q_full, ee_frame_id_,
                                  pinocchio::LOCAL, J);

  // ---- 2. 构造 twist 向量 ----
  Eigen::Matrix<double, 6, 1> twist_vec;
  twist_vec << twist[0], twist[1], twist[2],
               twist[3], twist[4], twist[5];

  // ---- 3. 锁定关节：将其雅可比列置零，速度强制为 0 ----
  if (locked_joint_ >= 0 && locked_joint_ < static_cast<int>(n_joints_))
  {
    int j_col = vel_offset_ + locked_joint_;
    J.col(j_col).setZero();
  }

  // ---- 4. DLS: θ̇ = J^T (J·J^T + λ²·I)⁻¹ · ẋ ----
  Eigen::Matrix<double, 6, 6> JJt = J * J.transpose();

  // 固定小阻尼（先验证通路，后续再自适应）
  double lambda = 0.05;

  // 正则化：JJt + λ²·I
  JJt.diagonal().array() += lambda * lambda;

  // 解 (J·J^T + λ²·I) · y = ẋ，然后 θ̇ = J^T · y
  Eigen::Matrix<double, 6, 1> y = JJt.ldlt().solve(twist_vec);

  Eigen::VectorXd qd_full_new = J.transpose() * y;

  // 关节限位保护：快到极限时锁住速度，不往里推
  for (size_t i = 0; i < n_joints_; ++i)
  {
    int idx = vel_offset_ + static_cast<int>(i);
    double lo = model_->lowerPositionLimit[idx];
    double hi = model_->upperPositionLimit[idx];
    double margin = 0.05;
    if (lo > -1e10 && q_full[idx] <= lo + margin && qd_full_new[idx] < 0.0)
      qd_full_new[idx] = 0.0;
    if (hi <  1e10 && q_full[idx] >= hi - margin && qd_full_new[idx] > 0.0)
      qd_full_new[idx] = 0.0;
  }

  // 防变形：只拦截"完全反向"的情况（关节限位导致）
  // 允许正常的数值交叉耦合，不干涉
  Eigen::Matrix<double, 6, 1> achieved = J * qd_full_new;
  for (int k = 0; k < 6; ++k)
  {
    // 只在期望非零且实际反向时才拦截
    if (std::abs(twist_vec[k]) > 0.01 && achieved[k] * twist_vec[k] < 0)
    {
      qd_full_new.setZero();
      RCLCPP_INFO_ONCE(rclcpp::get_logger("Fdcc"),
          "motion stopped at joint limit");
      break;
    }
  }

  // 锁定关节速度置零
  if (locked_joint_ >= 0 && locked_joint_ < static_cast<int>(n_joints_))
  {
    int j_idx = vel_offset_ + locked_joint_;
    qd_full_new[j_idx] = 0.0;
  }

  // ---- 6. 验证：J * θ̇ 应该 ≈ twist ----
  static int verify_count = 0;
  if (++verify_count % 100 == 0)  // 每 100 次 (0.2s) 验证一次
  {
    Eigen::Matrix<double, 6, 1> achieved = J * qd_full_new;
    RCLCPP_INFO(rclcpp::get_logger("Fdcc"),
        "verify: des=[%.3f %.3f %.3f | %.3f %.3f %.3f] ach=[%.3f %.3f %.3f | %.3f %.3f %.3f]",
        twist[0], twist[1], twist[2], twist[3], twist[4], twist[5],
        achieved[0], achieved[1], achieved[2], achieved[3], achieved[4], achieved[5]);
  }

  // ---- 7. 积分 q_new = q + θ̇ · dt ----
  Eigen::VectorXd q_full_new = q_full + qd_full_new * dt_;

  // ---- 6. 提取纯关节部分 + 限位 ----
  std::vector<Output> out;
  out.reserve(n_joints_);

  for (size_t i = 0; i < n_joints_; ++i)
  {
    int idx = vel_offset_ + static_cast<int>(i);

    double pos = q_full_new[idx];
    double vel = qd_full_new[idx];

    double lo = model_->lowerPositionLimit[idx];
    double hi = model_->upperPositionLimit[idx];
    if (lo > -1e10) pos = std::max(pos, lo);
    if (hi <  1e10) pos = std::min(pos, hi);

    vel = std::clamp(vel, -30.0, 30.0);

    out.push_back({pos, vel, 0.0});
  }

  return out;
}

}  // namespace arm_hardware_interface
