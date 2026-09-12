#include <algorithm>

#include "arm_hardware_interface/remote.hpp"

namespace remote
{

bool Remote::update(const RawJoystickState& input)
{
    // 当前映射使用 axes[0..5] 和 buttons[0..13]。
    if (input.axes.size() < axes_.size() || input.buttons.size() < buttons_.size())
    {
        return false;
    }

    // ---- 缓存上一帧状态用于边沿检测 ----
    const bool prev_guide = enable_pressed_;
    const bool prev_r3    = change_pressed_;

    // ---- 解析摇杆轴 ----
    for (size_t i = 0; i < axes_.size(); ++i)
    {
        axes_[i] = static_cast<double>(input.axes[i]);
    }

    // ---- 解析按钮 ----
    enable_pressed_ = input.buttons[9] != 0;
    change_pressed_ = input.buttons[13] != 0;
    J7_hold_        = input.buttons[12] != 0;

    // 保持现有手柄映射：D-pad 使用 axes[4] 和 axes[5]。
    dpad_up_    = (axes_[5] > 0.5) ? 1.0 : 0.0;
    dpad_down_  = (axes_[5] < -0.5) ? 1.0 : 0.0;
    dpad_left_  = (axes_[4] > 0.5) ? 1.0 : 0.0;
    dpad_right_ = (axes_[4] < -0.5) ? 1.0 : 0.0;

    // ---- 缓存原始按钮数组 ----
    a_rising_ = input.buttons[0] != 0 && !buttons_[0];
    b_rising_ = input.buttons[1] != 0 && !buttons_[1];
    x_rising_ = input.buttons[3] != 0 && !buttons_[3];
    y_rising_ = input.buttons[2] != 0 && !buttons_[2];

    for (size_t i = 0; i < buttons_.size(); ++i)
    {
        buttons_[i] = input.buttons[i] != 0;
    }

    // ---- 边沿检测 ----
    enable_rising_ = enable_pressed_ && !prev_guide;
    change_rising_ = change_pressed_ && !prev_r3;

    // ---- 处理边沿触发的状态翻转 ----
    if (enable_rising_) motors_on_ = !motors_on_;
    if (change_rising_) mode_is_cartesian_ = !mode_is_cartesian_;

    return true;
}

}  // namespace remote
