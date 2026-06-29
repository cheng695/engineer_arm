#include "arm_hardware_interface/remote.hpp"
#include <algorithm>

namespace remote
{

bool Remote::update(const sensor_msgs::msg::Joy& msg)
{
    if (msg.axes.size() < 5 || msg.buttons.size() < 13)
    {
        return false;
    }

    // ---- 缓存上一帧状态用于边沿检测 ----
    const bool prev_guide = guide_pressed_;
    const bool prev_rt    = rt_pressed_;
    const bool prev_r3    = r3_pressed_;

    // ---- 解析摇杆轴 ----
    axes_[0] = static_cast<double>(msg.axes[0]);
    axes_[1] = static_cast<double>(msg.axes[1]);
    axes_[2] = static_cast<double>(msg.axes[2]);
    axes_[3] = static_cast<double>(msg.axes[3]);
    axes_[4] = static_cast<double>(msg.axes[4]);
    axes_[5] = static_cast<double>(msg.axes[5]);

    // ---- 解析按钮 ----
    guide_pressed_ = msg.buttons[9];
    rt_pressed_    = msg.buttons[7];
    r3_pressed_    = msg.buttons[12];
    l3_hold_       = msg.buttons[11];

    // 十字键索引 
    dpad_up_    = msg.buttons[13];
    dpad_down_  = msg.buttons[14];
    dpad_left_  = msg.buttons[15];
    dpad_right_ = msg.buttons[16];

    // ---- 缓存原始按钮数组 ----
    for (size_t i = 0; i < std::min(msg.buttons.size(), kMaxButtons); ++i)
    {
        buttons_[i] = msg.buttons[i];
    }

    // ---- 边沿检测 ----
    guide_rising_ = guide_pressed_ && !prev_guide;
    rt_rising_    = rt_pressed_    && !prev_rt;
    r3_rising_    = r3_pressed_    && !prev_r3;

    // ---- 处理边沿触发的状态翻转 ----
    if (guide_rising_)  motors_on_         = !motors_on_;
    if (rt_rising_)     paused_            = !paused_;
    if (r3_rising_)     mode_is_cartesian_ = !mode_is_cartesian_;

    return true;
}

}  // namespace remote
