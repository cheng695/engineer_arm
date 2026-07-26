#include "arm_hardware_interface/remote.hpp"
#include <algorithm>

namespace remote
{

bool Remote::update(const sensor_msgs::msg::Joy& msg)
{
    if (msg.axes.size() < 8 || msg.buttons.size() < kMaxButtons)
    {
        return false;
    }

    // ---- 缓存上一帧状态用于边沿检测 ----
    const bool prev_guide = enable_pressed_;
    const bool prev_rt    = stop_pressed_;
    const bool prev_r3    = change_pressed_;

    // ---- 解析摇杆轴 ----
    axes_[0] = static_cast<double>(msg.axes[0]);
    axes_[1] = static_cast<double>(msg.axes[1]);
    axes_[2] = static_cast<double>(msg.axes[2]);
    axes_[3] = static_cast<double>(msg.axes[3]);
    axes_[4] = static_cast<double>(msg.axes[4]);
    axes_[5] = static_cast<double>(msg.axes[5]);
    axes_[6] = static_cast<double>(msg.axes[6]);
    axes_[7] = static_cast<double>(msg.axes[7]);

    // ---- 解析按钮 ----
    enable_pressed_ = msg.buttons[6];
    stop_pressed_   = msg.buttons[7];
    change_pressed_ = msg.buttons[10];
    J7_hold_        = msg.buttons[9];

    // 十字键（axes[6]左/右=±1, axes[7]上/下=±1, 不按=0）
    dpad_up_    = (msg.axes[7] > 0.5) ? 1.0 : 0.0;
    dpad_down_  = (msg.axes[7] < -0.5) ? 1.0 : 0.0;
    dpad_left_  = (msg.axes[6] > 0.5) ? 1.0 : 0.0;
    dpad_right_ = (msg.axes[6] < -0.5) ? 1.0 : 0.0;

    // ---- 缓存原始按钮数组 ----
    for (size_t i = 0; i < std::min(msg.buttons.size(), kMaxButtons); ++i)
    {
        buttons_[i] = msg.buttons[i];
    }

    // ---- 边沿检测 ----
    enable_rising_ = enable_pressed_ && !prev_guide;
    stop_rising_    = stop_pressed_    && !prev_rt;
    change_rising_    = change_pressed_    && !prev_r3;

    // ---- 处理边沿触发的状态翻转 ----
    if (enable_rising_)  motors_on_         = !motors_on_;
    if (stop_rising_)     paused_            = !paused_;
    if (change_rising_)     mode_is_cartesian_ = !mode_is_cartesian_;

    return true;
}

}  // namespace remote
