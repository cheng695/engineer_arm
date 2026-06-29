#pragma once

#include <sensor_msgs/msg/joy.hpp>
#include <array>
#include <cmath>
#include <cstddef>

namespace remote
{

/**
 * @brief 手柄遥控器状态封装。
 *
 * 调用 update() 喂入原始的 Joy 消息，然后通过各个 getter 获取当前状态。
 *
 * - 摇杆/方向键：返回归一化后的值 [-1.0, 1.0]，中位死区已处理
 * - 瞬时按键（A/B/X/Y/open/close）：返回当前帧按钮是否按下
 * - 边沿触发（stop/continue/enable/disable/joint/cartesian）：
 *   在对应按钮的上升沿返回 true，其他帧返回 false
 */
class Remote
{
public:
    // @brief 手柄死区阈值（中位附近的输入归零）
    static constexpr double kDeadzone = 0.12;

    // ================================================================
    // 笛卡尔速度（摇杆归一化值）
    // ================================================================

    double x()     const { return -apply_deadzone(axes_[1]); }
    double y()     const { return -apply_deadzone(axes_[0]); }
    double z()     const { return apply_deadzone(dpad_up_ - dpad_down_); }
    double yaw()   const { return apply_deadzone(dpad_right_ - dpad_left_); }
    double pitch() const { return -apply_deadzone(axes_[4]); }
    double roll()  const { return -apply_deadzone(axes_[3]); }

    // ================================================================
    // 关节速度（摇杆归一化值）
    // ================================================================

    double j1() const { return -apply_deadzone(axes_[0]); }
    double j2() const { return -apply_deadzone(axes_[1]); }
    double j3() const { return -apply_deadzone(axes_[4]); }
    double j4() const { return l3_hold_ ? 0.0 : -apply_deadzone(axes_[3]); }
    double j5() const { return apply_deadzone(dpad_up_ - dpad_down_); }
    double j6() const { return apply_deadzone(dpad_right_ - dpad_left_); }
    double j7() const { return l3_hold_ ? -apply_deadzone(axes_[3]) : 0.0; }

    // ==========================================================
    // 瞬时按键（当前帧状态）
    // ================================================================

    bool a_btn()         const { return button(0); }
    bool b_btn()         const { return button(1); }
    bool x_btn()         const { return button(3); }
    bool y_btn()         const { return button(2); }
    bool open_gripper()  const { return button(4); }
    bool close_gripper() const { return button(5); }

    // ================================================================
    // 边沿触发（上升沿为 true，需在 update() 之后尽快消费）
    // ================================================================

    bool stop()      const { return rt_rising_ && !paused_; }
    bool continue_() const { return rt_rising_ && paused_; }
    bool enable()    const { return guide_rising_ && !motors_on_; }
    bool disable()   const { return guide_rising_ && motors_on_; }
    bool joint()     const { return r3_rising_ && mode_is_cartesian_; }
    bool cartesian() const { return r3_rising_ && !mode_is_cartesian_; }

    // ================================================================
    // 状态查询
    // ================================================================

    bool motors_on()     const { return motors_on_; }
    bool paused()        const { return paused_; }
    bool is_cartesian()  const { return mode_is_cartesian_; }
    bool is_joint()      const { return !mode_is_cartesian_; }

    // ================================================================
    // 更新接口（实现在 remote.cpp）
    // ================================================================

    /**
     * @brief 喂入一帧 Joy 消息，更新所有内部状态。
     * @return true 消息有效已更新，false 格式异常
     */
    bool update(const sensor_msgs::msg::Joy& msg);

private:
    /**
     * @brief 应用死区处理。
     * @param val 摇杆值
     * @return 处理后的摇杆值
     */
    static double apply_deadzone(double val)
    {
        if (std::abs(val) < kDeadzone) return 0.0;
        return (val - (val > 0.0 ? kDeadzone : -kDeadzone)) / (1.0 - kDeadzone);
    }

    /**
     * @brief 获取按钮当前帧状态。
     * @param i 按钮索引
     * @return true 按钮当前帧按下
     */
    bool button(size_t i) const
    {
        return (i < kMaxButtons) && buttons_[i];
    }

    static constexpr size_t kMaxButtons = 17;

    // ---- 摇杆 ----
    std::array<double, 6> axes_{};      //< 摇杆轴原始值 [LX, LY, LT, RX, RY, RT]
    double dpad_up_{0.0};               //< D-Pad 上 (0=松开, 1=按下)
    double dpad_down_{0.0};             //< D-Pad 下
    double dpad_left_{0.0};             //< D-Pad 左
    double dpad_right_{0.0};            //< D-Pad 右
    bool l3_hold_{false};               //< L3 是否按住，用于 J4/J7 切换

    // ---- 当前帧按钮（每帧 update 刷新） ----
    bool guide_pressed_{false};         //< Guide 当前帧按下
    bool rt_pressed_{false};            //< RT 当前帧按下（轴值或按键回退）
    bool r3_pressed_{false};            //< R3 当前帧按下
    std::array<bool, kMaxButtons> buttons_{}; //< 全部按钮当前帧状态

    // ---- 边沿检测（上升沿为 true，仅持续一帧） ----
    bool guide_rising_{false};          //< Guide 上升沿：松→按
    bool rt_rising_{false};             //< RT 上升沿
    bool r3_rising_{false};             //< R3 上升沿

    // ---- 持久状态（跨帧保持，由边沿翻转） ----
    bool motors_on_{false};             //< 电机是否使能 (Guide ↑翻转)
    bool paused_{false};                //< 是否暂停 (RT ↑翻转)
    bool mode_is_cartesian_{false};     //< 笛卡尔/关节模式 (R3 ↑翻转)
};

}  // namespace remote
