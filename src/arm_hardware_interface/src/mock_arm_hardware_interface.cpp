#include "arm_hardware_interface/mock_arm_hardware_interface.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace arm_hardware_interface
{

MockArmHardwareInterface::~MockArmHardwareInterface()
{
    teardown_internal_node();
    hardware_ready_ = false;
}

void MockArmHardwareInterface::setup_internal_node()
{
    internal_node_ = rclcpp::Node::make_shared("mock_arm_hw_internal");
    ready_pub_ = internal_node_->create_publisher<std_msgs::msg::Bool>(
        "/arm/state/hardware_ready", 10);
    ready_timer_ = internal_node_->create_wall_timer(
        std::chrono::milliseconds(20), [this] {
            std_msgs::msg::Bool message;
            message.data = hardware_ready_.load();
            ready_pub_->publish(message);
        });
    spin_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    spin_executor_->add_node(internal_node_);
    spin_thread_ = std::make_unique<std::thread>([this] { spin_executor_->spin(); });
}

void MockArmHardwareInterface::teardown_internal_node()
{
    if (spin_executor_)
        spin_executor_->cancel();
    if (spin_thread_)
    {
        spin_thread_->join();
        spin_thread_.reset();
    }
    spin_executor_.reset();
    ready_timer_.reset();
    ready_pub_.reset();
    internal_node_.reset();
}

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

    RCLCPP_INFO(rclcpp::get_logger("MockArmHW"), "MockArmHardwareInterface on_init 完成 (%zu 关节, 全仿真)",
        info.joints.size());
    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MockArmHardwareInterface::on_activate(
    const rclcpp_lifecycle::State& /*prev*/)
{
    setup_internal_node();
    hardware_ready_ = true;
    RCLCPP_INFO(rclcpp::get_logger("MockArmHW"), "on_activate 完成（仿真模式）");
    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MockArmHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State& /*prev*/)
{
    hardware_ready_ = false;
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
    echo_mock_joints(info_);
    return hardware_interface::return_type::OK;
}

// ================================================================
// write
// ================================================================

hardware_interface::return_type MockArmHardwareInterface::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    return hardware_interface::return_type::OK;
}

}  // namespace arm_hardware_interface

PLUGINLIB_EXPORT_CLASS(
    arm_hardware_interface::MockArmHardwareInterface,
    hardware_interface::SystemInterface)
