#pragma once

#include <atomic>
#include <chrono>
#include <cstddef>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/bool.hpp"

#include "arm_hardware_interface/arm_hardware_base.hpp"

struct mjModel_;
struct mjData_;

namespace arm_hardware_interface
{

/**
 * @brief MuJoCo-backed ros2_control system interface.
 *
 * Position commands and effort commands are sent to separate MuJoCo actuators
 * and are applied together. This lets motion controllers and the gravity
 * controller run at the same time.
 * This plugin has no CAN dependency and must be selected explicitly.
 */
class MujocoArmHardwareInterface
    : public hardware_interface::SystemInterface
    , public ArmHardwareBase
{
public:
    RCLCPP_UNIQUE_PTR_DEFINITIONS(MujocoArmHardwareInterface)

    MujocoArmHardwareInterface() = default;
    ~MujocoArmHardwareInterface() override;

    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo& info) override;
    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State& prev) override;
    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State& prev) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::return_type read(
        const rclcpp::Time& time, const rclcpp::Duration& period) override;
    hardware_interface::return_type write(
        const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
    bool load_model(const std::string& model_path);
    bool configure_joints();
    void update_state_from_mujoco();
    void setup_internal_node();
    void teardown_internal_node();

    mjModel_* model_{nullptr};
    mjData_* data_{nullptr};
    std::string model_path_;
    int simulation_steps_{1};
    std::vector<int> mujoco_joint_ids_;
    std::vector<int> mujoco_position_actuator_ids_;
    std::vector<int> mujoco_effort_actuator_ids_;
    std::vector<int> mujoco_qpos_addresses_;
    std::vector<int> mujoco_dof_addresses_;

    rclcpp::Node::SharedPtr internal_node_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enable_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
    rclcpp::executors::SingleThreadedExecutor::SharedPtr spin_executor_;
    std::unique_ptr<std::thread> spin_thread_;
    rclcpp::TimerBase::SharedPtr status_timer_;
    std::atomic<bool> hardware_ready_{false};
};

}  // namespace arm_hardware_interface
