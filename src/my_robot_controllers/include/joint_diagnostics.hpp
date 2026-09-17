#pragma once
#include <mutex>
#include <string>
#include <vector>
#include <array>
#include "rclcpp/rclcpp.hpp"

namespace my_robot_controllers {
// 周期线程仅尝试更新预分配快照；格式化和日志在 executor 定时器中执行。
class JointDiagnostics {
public:
    template<class Node>
    void configure(const std::shared_ptr<Node>& node, size_t count, std::string mode) {
        positions_.resize(count); feedback_.resize(count); velocities_.resize(count);
        mode_ = std::move(mode);
        timer_ = node->create_wall_timer(std::chrono::milliseconds(100), [this, logger = node->get_logger()] {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!pending_) return;
            for (size_t i = 0; i < positions_.size(); ++i)
                RCLCPP_INFO(logger, "%s模式，关节 %zu：期望角度=%.6f，反馈角度=%.6f，目标速度=%.6f",
                    mode_.c_str(), i + 1, positions_[i], feedback_[i], velocities_[i]);
            if (mode_ == "DLS")
            {
                RCLCPP_INFO(logger,
                    "笛卡尔指令：linear=[%.6f, %.6f, %.6f] m/s, angular=[%.6f, %.6f, %.6f] rad/s",
                    twist_[0], twist_[1], twist_[2], twist_[3], twist_[4], twist_[5]);
                RCLCPP_INFO(logger,
                    "夹爪中心：期望=[%.6f, %.6f, %.6f]，实际=[%.6f, %.6f, %.6f]，误差=[%.6f, %.6f, %.6f] m",
                    target_center_[0], target_center_[1], target_center_[2],
                    actual_center_[0], actual_center_[1], actual_center_[2],
                    center_error_[0], center_error_[1], center_error_[2]);
                RCLCPP_INFO(logger, "DLS状态：sigma_min=%.6f，lambda=%.6f，受限=%s",
                            sigma_, damping_, blocked_ ? "是" : "否");
                RCLCPP_INFO(logger,
                    "TCP防跑飞：最大关节跟随误差=%.6f rad，运动缩放=%.3f，参考缩放=%.3f，纠偏增益比例=%.3f",
                    maximum_joint_following_error_, following_scale_,
                    reference_motion_scale_, correction_scale_);
                if (blocked_joint_ >= 0)
                    RCLCPP_INFO(logger,
                        "DLS受限原因：关节限位，首个触发关节序号=%d，方向=%s，保护区=0.05 rad",
                        blocked_joint_ + 1, upper_limit_ ? "上限" : "下限");
                if (task_blocked_)
                    RCLCPP_INFO(logger,
                        "DLS受限原因：原始解的末端速度实现比例不足，限幅前=%.6f，限幅后=%.6f，判定阈值(限幅前)=0.100000",
                        raw_tracking_ratio_, tracking_ratio_);
            }
            pending_ = false;
        });
    }
    void cartesian(const std::array<double, 6>& twist,
                   const std::array<double, 3>& target_center,
                   const std::array<double, 3>& actual_center) {
        std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
        if (!lock) return;
        twist_ = twist;
        target_center_ = target_center;
        actual_center_ = actual_center;
        for (size_t i = 0; i < 3; ++i)
            center_error_[i] = target_center_[i] - actual_center_[i];
        pending_ = true;
    }
    void record(size_t i, double position, double feedback, double velocity) {
        std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
        if (!lock || i >= positions_.size()) return;
        positions_[i] = position; feedback_[i] = feedback; velocities_[i] = velocity;
        pending_ = true;
    }
    void solver(double sigma, double damping, bool blocked, int blocked_joint,
                bool upper_limit, bool task_blocked, double tracking_ratio,
                double raw_tracking_ratio) {
        std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
        if (!lock) return;
        sigma_ = sigma; damping_ = damping; blocked_ = blocked;
        blocked_joint_ = blocked_joint; upper_limit_ = upper_limit;
        task_blocked_ = task_blocked; tracking_ratio_ = tracking_ratio;
        raw_tracking_ratio_ = raw_tracking_ratio;
    }
    void tracking(double maximum_joint_following_error, double following_scale,
                  double reference_motion_scale, double correction_scale) {
        std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
        if (!lock) return;
        maximum_joint_following_error_ = maximum_joint_following_error;
        following_scale_ = following_scale;
        reference_motion_scale_ = reference_motion_scale;
        correction_scale_ = correction_scale;
    }
private:
    double sigma_{0}, damping_{0};
    bool blocked_{false};
    int blocked_joint_{-1};
    bool upper_limit_{false}, task_blocked_{false};
    double tracking_ratio_{1.0}, raw_tracking_ratio_{1.0};
    double maximum_joint_following_error_{0.0};
    double following_scale_{1.0}, reference_motion_scale_{1.0}, correction_scale_{1.0};
    std::array<double, 6> twist_{};
    std::array<double, 3> target_center_{}, actual_center_{}, center_error_{};
    std::mutex mutex_;
    std::vector<double> positions_, feedback_, velocities_;
    std::string mode_;
    bool pending_{false};
    rclcpp::TimerBase::SharedPtr timer_;
};
}
