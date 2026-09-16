#pragma once
#include <mutex>
#include <string>
#include <vector>
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
                RCLCPP_INFO(logger, "DLS状态：sigma_min=%.6f，lambda=%.6f，受限=%s",
                            sigma_, damping_, blocked_ ? "是" : "否");
            pending_ = false;
        });
    }
    void record(size_t i, double position, double feedback, double velocity) {
        std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
        if (!lock || i >= positions_.size()) return;
        positions_[i] = position; feedback_[i] = feedback; velocities_[i] = velocity;
        pending_ = true;
    }
    void solver(double sigma, double damping, bool blocked) {
        std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
        if (!lock) return;
        sigma_ = sigma; damping_ = damping; blocked_ = blocked;
    }
private:
    double sigma_{0}, damping_{0};
    bool blocked_{false};
    std::mutex mutex_;
    std::vector<double> positions_, feedback_, velocities_;
    std::string mode_;
    bool pending_{false};
    rclcpp::TimerBase::SharedPtr timer_;
};
}
