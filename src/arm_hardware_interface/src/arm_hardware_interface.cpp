#include "arm_hardware_interface/arm_hardware_interface.hpp"

#include <chrono>
#include <thread>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <map>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

// 新增加的头文件
#include "arm_hardware_interface/socket_can.hpp"
#include "arm_hardware_interface/motor_drivers/dm_motor.hpp"

namespace arm_hardware_interface
{
// 使用命名空间简化代码
using namespace motor_drivers::DM;

// 静态字典，管理所有被用到的 CAN 总线
static std::map<std::string, std::shared_ptr<SocketCan>> can_buses;
static std::vector<std::shared_ptr<DmMotor>> joints_motors;

hardware_interface::CallbackReturn ArmHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_states_.resize(info_.joints.size(), 0.0);
  hw_commands_.resize(info_.joints.size(), 0.0);
  hw_states_vel_.resize(info_.joints.size(), 0.0);
  hw_commands_vel_.resize(info_.joints.size(), 0.0);

  // 清理旧数据，防止重复初始化
  joints_motors.clear();
  can_buses.clear();

  // 为每个关节显示初始化电机 (可以根据实际型号和总线进行配置)
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
      // 1. 读取 CAN ID
      if (info_.joints[i].parameters.find("can_id") == info_.joints[i].parameters.end()) 
      {
          RCLCPP_FATAL(rclcpp::get_logger("ArmHardwareInterface"), "Joint '%s' missing 'can_id' parameter!", info_.joints[i].name.c_str());
          return hardware_interface::CallbackReturn::ERROR;
      }
      uint32_t id = std::stoi(info_.joints[i].parameters.at("can_id"));

      // 2. 读取总线名称 (can0, can1, etc.)
      std::string bus_name = "can0";
      if (info_.joints[i].parameters.find("can_bus") != info_.joints[i].parameters.end()) 
      {
          bus_name = info_.joints[i].parameters.at("can_bus");
      }
      
      // 3. 读取电机型号参数 (motor_model)
      std::string model = "J4310"; // 默认值
      if (info_.joints[i].parameters.find("motor_model") != info_.joints[i].parameters.end()) 
      {
          model = info_.joints[i].parameters.at("motor_model");
      }

      // 4. 读取 PID 增益 (kp, kd)
      float kp = 50.0f;
      float kd = 1.0f;
      if (info_.joints[i].parameters.find("kp") != info_.joints[i].parameters.end()) 
      {
          kp = std::stof(info_.joints[i].parameters.at("kp"));
      }
      if (info_.joints[i].parameters.find("kd") != info_.joints[i].parameters.end()) 
      {
          kd = std::stof(info_.joints[i].parameters.at("kd"));
      }

      std::shared_ptr<DmMotor> motor;
      if (model == "J4310") 
      {
          motor = std::make_shared<J4310>();
      } 
      else if (model == "J4340") 
      {
          motor = std::make_shared<J4340>();
      } 
      else if (model == "J8009") 
      {
          motor = std::make_shared<J8009>();
      }
      else 
      {
          RCLCPP_ERROR(rclcpp::get_logger("ArmHardwareInterface"), "Unknown motor model '%s' for joint '%s'!", model.c_str(), info_.joints[i].name.c_str());
          return hardware_interface::CallbackReturn::ERROR;
      }
      
      if (motor) 
      {
          motor->setCanId(id);
          motor->setBusName(bus_name);
          motor->setKp(kp);
          motor->setKd(kd);
          joints_motors.push_back(motor);

          // 5. 确保该总线的 Socket 实例已存在
          if (can_buses.find(bus_name) == can_buses.end()) 
          {
              can_buses[bus_name] = std::make_shared<SocketCan>();
          }
      }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ArmHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_vel_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ArmHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_vel_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn ArmHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Open all CAN buses
  for (auto const& [name, bus] : can_buses) 
  {
    RCLCPP_INFO(rclcpp::get_logger("ArmHardwareInterface"), "Opening CAN bus: %s", name.c_str());
    if (!bus->open(name)) 
    {
      RCLCPP_ERROR(rclcpp::get_logger("ArmHardwareInterface"), "Failed to open %s!", name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // DO NOT enable motors here. Wait for Xbox button.
  motors_enabled_ = false;
  RCLCPP_WARN(rclcpp::get_logger("ArmHardwareInterface"),
      "[POWER] CAN buses opened. Motors are OFF. Press Xbox button to enable.");

  // Create internal node for motor enable/disable subscription
  if (!internal_node_) {
    internal_node_ = rclcpp::Node::make_shared("arm_hw_internal");
    enable_sub_ = internal_node_->create_subscription<std_msgs::msg::Bool>(
        "/arm_motor_enable", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
          if (msg->data) {
            enable_requested_ = true;
          } else {
            disable_requested_ = true;
          }
        });
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArmHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // 失能所有电机
  for (auto & motor : joints_motors) 
  {
    uint8_t disable_data[8];
    motor->get_disable_command(disable_data);
    can_buses[motor->getBusName()]->write_frame(motor->getCanId(), disable_data);
  }

  for (auto const& [name, bus] : can_buses) 
  {
    bus->close_socket();
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ArmHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Process enable/disable topic callbacks
  if (internal_node_) {
    rclcpp::spin_some(internal_node_);
  }

  struct can_frame frame;
  for (auto const& [name, bus] : can_buses) 
  {
    while (bus->read_frame(frame) > 0) 
    {
      uint32_t received_id = frame.can_id & 0x7FF;
      
      for (size_t i = 0; i < joints_motors.size(); i++)
      {
          bool id_match = (joints_motors[i]->getCanId() == received_id);
          bool payload_match = (received_id == 0x00 && (frame.data[0] & 0x0F) == joints_motors[i]->getCanId());

          if ((id_match || payload_match) && joints_motors[i]->getBusName() == name)
          {
              joints_motors[i]->parse_feedback(frame.data);
              hw_states_[i] = joints_motors[i]->getAngleRad();
              hw_states_vel_[i] = joints_motors[i]->getVelocityRad();
              
              if (joints_motors[i]->getErrorCode() != 0) 
              {
                  static int error_log_count = 0;
                  if (error_log_count++ % 100 == 0) 
                  {
                      RCLCPP_WARN(rclcpp::get_logger("ArmHardwareInterface"), 
                          "Motor ID %u (Joint %zu) on %s reported ERROR CODE: 0x%02X", 
                          joints_motors[i]->getCanId(), i+1, name.c_str(), joints_motors[i]->getErrorCode());
                  }
              }

              static int r_count = 0;
              if (r_count++ % 100 == 0) 
              {
                  RCLCPP_INFO(rclcpp::get_logger("ArmHardwareInterface"), 
                      "Feedback matched %s ID %d: Pos=%.3f", name.c_str(), joints_motors[i]->getCanId(), hw_states_[i]);
              }
              break;
          }
      }
    }
  }

  // --- KINEMATIC DECOUPLING (J2 & J3 Belt Coupling) ---
  // J2 Index: 1, J3 Index: 2
  // Formula: J3_actual = -(J3_motor - 0.986 * J2_actual)
  double raw_j3 = hw_states_[2];
  double raw_j3_vel = hw_states_vel_[2];
  hw_states_[2] = -(raw_j3 - 0.986 * hw_states_[1]);
  hw_states_vel_[2] = -(raw_j3_vel - 0.986 * hw_states_vel_[1]);

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ArmHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // ====================================================================
  // Phase 0: Process Enable/Disable Requests (from Xbox button topic)
  // ====================================================================
  if (enable_requested_.exchange(false))
  {
    // ANTI-JUMP: Sync commands to current positions BEFORE enabling
    for (size_t i = 0; i < joints_motors.size(); i++)
    {
      hw_commands_[i] = hw_states_[i];
    }
    RCLCPP_WARN(rclcpp::get_logger("ArmHardwareInterface"),
        "[POWER] Position synced. Commands locked to current state.");

    // Now send enable to all motors (interleaved)
    for (auto & motor : joints_motors)
    {
      uint8_t cmd[8];
      motor->get_enable_command(cmd);
      can_buses[motor->getBusName()]->write_frame(motor->getCanId(), cmd);
      std::this_thread::sleep_for(std::chrono::microseconds(200));
    }
    motors_enabled_ = true;
    RCLCPP_WARN(rclcpp::get_logger("ArmHardwareInterface"),
        "[POWER] ALL MOTORS ENABLED (Torque ON)");
  }

  if (disable_requested_.exchange(false))
  {
    for (auto & motor : joints_motors)
    {
      uint8_t cmd[8];
      motor->get_disable_command(cmd);
      can_buses[motor->getBusName()]->write_frame(motor->getCanId(), cmd);
      std::this_thread::sleep_for(std::chrono::microseconds(200));
    }
    motors_enabled_ = false;
    RCLCPP_WARN(rclcpp::get_logger("ArmHardwareInterface"),
        "[POWER] ALL MOTORS DISABLED (Torque OFF)");
  }

  // If motors are not enabled, send a Passive Probe (KP=0, KD=0) to trigger feedback
  if (!motors_enabled_) {
    for (auto & motor : joints_motors) {
      uint8_t probe[8];
      // Sending Target=0, Vel=0, KP=0, KD=0, Torque=0
      // This will not apply torque but will force the motor to respond with its current state
      motor->pack_mit_command(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, probe);
      can_buses[motor->getBusName()]->write_frame(motor->getCanId(), probe);
      std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
    return hardware_interface::return_type::OK;
  }
  // ====================================================================
  // Phase 1: Error Recovery
  // If any motor reports error_code != 1 (1 = normal), perform:
  //   clear_errors → 1ms → disable → 1ms → enable
  // ====================================================================
  for (size_t i = 0; i < joints_motors.size(); i++)
  {
    uint8_t err = joints_motors[i]->getErrorCode();
    if (err != 0 && err != 1) // 0 = no feedback yet, 1 = normal
    {
      RCLCPP_WARN(rclcpp::get_logger("ArmHardwareInterface"),
          "[RECOVERY] Motor ID %u on %s: error_code=0x%02X, initiating recovery...",
          joints_motors[i]->getCanId(), joints_motors[i]->getBusName().c_str(), err);

      auto& bus = can_buses[joints_motors[i]->getBusName()];
      uint32_t id = joints_motors[i]->getCanId();
      uint8_t cmd[8];

      // Step 1: Clear Error
      joints_motors[i]->get_clear_errors_command(cmd);
      bus->write_frame(id, cmd);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));

      // Step 2: Disable
      joints_motors[i]->get_disable_command(cmd);
      bus->write_frame(id, cmd);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));

      // Step 3: Re-Enable
      joints_motors[i]->get_enable_command(cmd);
      bus->write_frame(id, cmd);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));

      RCLCPP_INFO(rclcpp::get_logger("ArmHardwareInterface"),
          "[RECOVERY] Motor ID %u recovery sequence sent.", joints_motors[i]->getCanId());
    }
  }

  // ====================================================================
  // Phase 2: Interleaved CAN Write
  // Group motors by bus, then alternate: bus0[0], bus1[0], bus0[1], bus1[1]...
  // Insert 100μs between each frame to prevent TX mailbox overflow.
  // ====================================================================
  std::map<std::string, std::vector<size_t>> bus_groups;
  for (size_t i = 0; i < joints_motors.size(); i++)
  {
    if (!std::isnan(hw_commands_[i]))
    {
      bus_groups[joints_motors[i]->getBusName()].push_back(i);
    }
  }

  // Find the maximum number of motors on any single bus
  size_t max_depth = 0;
  for (auto const& [name, indices] : bus_groups)
  {
    if (indices.size() > max_depth) max_depth = indices.size();
  }

  // Interleaved send: round-robin across buses
  uint8_t cmd_data[8];
  for (size_t slot = 0; slot < max_depth; slot++)
  {
    for (auto const& [bus_name, indices] : bus_groups)
    {
      if (slot < indices.size())
      {
        size_t i = indices[slot];
        
        double target_pos = hw_commands_[i];
        
        // --- COMMAND COUPLING (J2 & J3 Belt Compensation) ---
        // If it's Joint 3 (index 2), we need to reverse the decoupling formula:
        if (i == 2) {
            target_pos = -hw_commands_[2] + 0.986 * hw_commands_[1];
        }

        joints_motors[i]->pack_mit_command(
            target_pos, 0.0f,
            joints_motors[i]->getKp(), joints_motors[i]->getKd(),
            0.0f, cmd_data);
        can_buses[bus_name]->write_frame(joints_motors[i]->getCanId(), cmd_data);

        // Inter-frame delay: 100μs to let the CAN controller drain the TX mailbox
        std::this_thread::sleep_for(std::chrono::microseconds(100));
      }
    }
  }

  static int w_count = 0;
  if (w_count++ % 200 == 0)
  {
    RCLCPP_INFO(rclcpp::get_logger("ArmHardwareInterface"),
        "[WRITE] Interleaved send complete. %zu motors across %zu buses.",
        joints_motors.size(), bus_groups.size());
  }

  return hardware_interface::return_type::OK;
}

}  // namespace arm_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  arm_hardware_interface::ArmHardwareInterface, hardware_interface::SystemInterface)
