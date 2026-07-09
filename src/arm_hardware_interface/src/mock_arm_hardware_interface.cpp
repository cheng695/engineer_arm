#include "arm_hardware_interface/mock_arm_hardware_interface.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace arm_hardware_interface
{

// ================================================================
// 生命周期
// ================================================================

hardware_interface::CallbackReturn MockArmHardwareInterface::on_init(
    const hardware_interface::HardwareInfo& info)
{
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
        return CallbackReturn::ERROR;

    init_joint_buffers(info);
    init_joint_limits(info);

    // 仿真模式：所有关节均回显（无 CAN 硬件）
    std::fill(use_real_joint_io_.begin(), use_real_joint_io_.end(), false);

    if (!init_gravity_compensator(info))
        RCLCPP_WARN(rclcpp::get_logger("MockArmHW"), "重力补偿未启用");

    init_dls();
    init_joint_controller(info);

    RCLCPP_INFO(rclcpp::get_logger("MockArmHW"), "MockArmHardwareInterface on_init 完成 (%zu 关节, 全仿真)",
        info.joints.size());
    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MockArmHardwareInterface::on_activate(
    const rclcpp_lifecycle::State& /*prev*/)
{
    setup_internal_node("mock_arm_hw_internal");
    fsm_.onEnable();  // 仿真直接使能，跳过电机控制
    RCLCPP_INFO(rclcpp::get_logger("MockArmHW"), "on_activate 完成（仿真模式）");
    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MockArmHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State& /*prev*/)
{
    teardown_internal_node();
    RCLCPP_INFO(rclcpp::get_logger("MockArmHW"), "on_deactivate 完成");
    return CallbackReturn::SUCCESS;
}

// ================================================================
// 接口导出
// ================================================================

std::vector<hardware_interface::StateInterface> MockArmHardwareInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> ifaces;
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
        ifaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_pos_[i]);
        ifaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_vel_[i]);
        ifaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_EFFORT,   &hw_states_eff_[i]);
    }
    return ifaces;
}

std::vector<hardware_interface::CommandInterface> MockArmHardwareInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> ifaces;
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
        ifaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_pos_[i]);
        ifaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_vel_[i]);
        ifaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_EFFORT,   &hw_commands_eff_[i]);
    }
    return ifaces;
}

// ================================================================
// read
// ================================================================

hardware_interface::return_type MockArmHardwareInterface::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
        hw_states_pos_[i] = hw_commands_pos_[i];
        hw_states_vel_[i] = hw_commands_vel_[i];
        hw_states_eff_[i] = 0.0;
    }
    apply_gravity_to_effort();
    return hardware_interface::return_type::OK;
}

// ================================================================
// write
// ================================================================

hardware_interface::return_type MockArmHardwareInterface::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    process_control(info_);

    static int diag = 0;
    if (++diag % 25 == 0) {
        RCLCPP_INFO(rclcpp::get_logger("MockHW"),
            "WRITE cmd: J1=%.3f J2=%.3f J3=%.3f J4=%.3f J5=%.3f J6=%.3f J7=%.3f | FSM=%s",
            hw_commands_pos_[0], hw_commands_pos_[1], hw_commands_pos_[2],
            hw_commands_pos_[3], hw_commands_pos_[4], hw_commands_pos_[5],
            hw_commands_pos_[6], fsm_.state_name());
    }
    return hardware_interface::return_type::OK;
}

}  // namespace arm_hardware_interface

PLUGINLIB_EXPORT_CLASS(
    arm_hardware_interface::MockArmHardwareInterface,
    hardware_interface::SystemInterface)
