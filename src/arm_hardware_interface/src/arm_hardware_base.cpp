#include "arm_hardware_interface/arm_hardware_base.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace arm_hardware_interface
{

namespace
{
constexpr double kDefaultLowerLimit = -std::numeric_limits<double>::infinity();
constexpr double kDefaultUpperLimit =  std::numeric_limits<double>::infinity();
}  // namespace


/**
 * @brief 初始化joint_buffers_
 *
 * @param info
 */
void ArmHardwareBase::init_joint_buffers(const hardware_interface::HardwareInfo& info)
{
    const size_t n = info.joints.size();

    hw_states_pos_.resize(n, 0.0);
    hw_states_vel_.resize(n, 0.0);
    hw_states_eff_.resize(n, 0.0);

    hw_commands_pos_.resize(n, 0.0);
    hw_commands_vel_.resize(n, 0.0);
    hw_commands_eff_.resize(n, 0.0);

    use_real_joint_io_.resize(n, true);

    raw_motor_pos_.resize(n, 0.0);
    raw_motor_vel_.resize(n, 0.0);
    raw_motor_eff_.resize(n, 0.0);
}

/**
 * @brief 初始化joint_limits_
 *
 * @param info
 */
void ArmHardwareBase::init_joint_limits(const hardware_interface::HardwareInfo& info)
{
    for (const auto& j : info.joints)
    {
        double lo = kDefaultLowerLimit, hi = kDefaultUpperLimit;

        auto it_lo = j.parameters.find("lower_limit");
        if (it_lo != j.parameters.end())
            lo = std::stod(it_lo->second);

        auto it_hi = j.parameters.find("upper_limit");
        if (it_hi != j.parameters.end())
            hi = std::stod(it_hi->second);

        joint_lower_limits_.push_back(lo);
        joint_upper_limits_.push_back(hi);
    }
}

/**
 * @brief 检测mock joint
 *
 * @param info
 */
void ArmHardwareBase::init_mock_joints(const hardware_interface::HardwareInfo& info)
{
    for (size_t i = 0; i < info.joints.size(); ++i)
    {
        if (info.joints[i].parameters.find("can_id") == info.joints[i].parameters.end())
            use_real_joint_io_[i] = false;
    }
}

/**
 * @brief 模拟joint
 *
 * @param info
 */
void ArmHardwareBase::echo_mock_joints(const hardware_interface::HardwareInfo& info)
{
    for (size_t i = 0; i < info.joints.size(); ++i)
    {
        if (!use_real_joint_io_[i])
        {
            hw_states_pos_[i] = hw_commands_pos_[i];
            hw_states_vel_[i] = hw_commands_vel_[i];
            hw_states_eff_[i] = 0.0;
        }
    }
}

/**
 * @brief J2J3解偶
 *
 */
void ArmHardwareBase::apply_j2j3_coupling()
{
    if (use_real_joint_io_[kJ2Index] && use_real_joint_io_[kJ3Index])
    {
        const double j3_scale = std::abs(j2j3_j3_scale_) > 1e-9 ? j2j3_j3_scale_ : 1.0;
        const double raw_j2_pos = hw_states_pos_[kJ2Index];
        const double raw_j2_vel = hw_states_vel_[kJ2Index];
        const double raw_j2_eff = hw_states_eff_[kJ2Index];
        const double raw_j3_pos = hw_states_pos_[kJ3Index];
        const double raw_j3_vel = hw_states_vel_[kJ3Index];
        const double raw_j3_eff = hw_states_eff_[kJ3Index];
        const double correction = j2j3_poly_correction(raw_j2_pos);
        const double derivative = j2j3_poly_derivative(raw_j2_pos);

        if (j2j3_scale_mode_is_multiply())
        {
            hw_states_pos_[kJ3Index] = j3_scale * raw_j3_pos + correction;
            hw_states_vel_[kJ3Index] = j3_scale * raw_j3_vel + derivative * raw_j2_vel;
            const double j3_eff = raw_j3_eff / j3_scale;
            hw_states_eff_[kJ2Index] = raw_j2_eff - derivative * j3_eff;
            hw_states_eff_[kJ3Index] = j3_eff;
        }
        else
        {
            hw_states_pos_[kJ3Index] = (raw_j3_pos + correction) / j3_scale;
            hw_states_vel_[kJ3Index] = (raw_j3_vel + derivative * raw_j2_vel) / j3_scale;
            hw_states_eff_[kJ2Index] = raw_j2_eff - derivative * raw_j3_eff;
            hw_states_eff_[kJ3Index] = j3_scale * raw_j3_eff;
        }
    }
}

double ArmHardwareBase::j2j3_poly_correction(double j2_pos) const
{
    return j2j3_coupling_ * j2_pos - j2j3_j3_offset_ +
        j2j3_poly_a3_ * j2_pos * j2_pos * j2_pos +
        j2j3_poly_a2_ * j2_pos * j2_pos +
        j2j3_poly_a1_ * j2_pos +
        j2j3_poly_a0_;
}

double ArmHardwareBase::j2j3_poly_derivative(double j2_pos) const
{
    return j2j3_coupling_ +
        3.0 * j2j3_poly_a3_ * j2_pos * j2_pos +
        2.0 * j2j3_poly_a2_ * j2_pos +
        j2j3_poly_a1_;
}

bool ArmHardwareBase::j2j3_scale_mode_is_multiply() const
{
    return j2j3_scale_mode_ == "multiply" || j2j3_scale_mode_ == "mul";
}

}  // namespace arm_hardware_interface
