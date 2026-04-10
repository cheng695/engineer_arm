#include "arm_hardware_interface/arm_hardware_interface.hpp"

#include <chrono>
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

  // 清理旧数据，防止重复初始化
  joints_motors.clear();
  can_buses.clear();

  // 为每个关节显示初始化电机 (可以根据实际型号和总线进行配置)
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
      // 1. 读取 CAN ID
      if (info_.joints[i].parameters.find("can_id") == info_.joints[i].parameters.end()) {
          RCLCPP_FATAL(rclcpp::get_logger("ArmHardwareInterface"), "Joint '%s' missing 'can_id' parameter!", info_.joints[i].name.c_str());
          return hardware_interface::CallbackReturn::ERROR;
      }
      uint32_t id = std::stoi(info_.joints[i].parameters.at("can_id"));

      // 2. 读取总线名称 (can0, can1, etc.)
      std::string bus_name = "can0";
      if (info_.joints[i].parameters.find("can_bus") != info_.joints[i].parameters.end()) {
          bus_name = info_.joints[i].parameters.at("can_bus");
      }
      
      // 3. 读取电机型号参数 (motor_model)
      std::string model = "J4310"; // 默认值
      if (info_.joints[i].parameters.find("motor_model") != info_.joints[i].parameters.end()) {
          model = info_.joints[i].parameters.at("motor_model");
      }

      // 4. 读取 PID 增益 (kp, kd)
      float kp = 50.0f;
      float kd = 1.0f;
      if (info_.joints[i].parameters.find("kp") != info_.joints[i].parameters.end()) {
          kp = std::stof(info_.joints[i].parameters.at("kp"));
      }
      if (info_.joints[i].parameters.find("kd") != info_.joints[i].parameters.end()) {
          kd = std::stof(info_.joints[i].parameters.at("kd"));
      }

      std::shared_ptr<DmMotor> motor;
      if (model == "J4310") {
          motor = std::make_shared<J4310>();
      } else if (model == "J4340") {
          motor = std::make_shared<J4340>();
      } else {
          RCLCPP_ERROR(rclcpp::get_logger("ArmHardwareInterface"), "Unknown motor model '%s' for joint '%s'!", model.c_str(), info_.joints[i].name.c_str());
          return hardware_interface::CallbackReturn::ERROR;
      }
      
      if (motor) {
          motor->setCanId(id);
          motor->setBusName(bus_name);
          motor->setKp(kp);
          motor->setKd(kd);
          joints_motors.push_back(motor);

          // 5. 确保该总线的 Socket 实例已存在
          if (can_buses.find(bus_name) == can_buses.end()) {
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
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn ArmHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // 打开所有在使用的总线
  for (auto const& [name, bus] : can_buses) 
  {
    RCLCPP_INFO(rclcpp::get_logger("ArmHardwareInterface"), "Opening CAN bus: %s", name.c_str());
    if (!bus->open(name)) 
    {
      RCLCPP_ERROR(rclcpp::get_logger("ArmHardwareInterface"), "Failed to open %s!", name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // 使能所有电机 (使用每个电机自己的协议命令)
  for (auto & motor : joints_motors) 
  {
    uint8_t enable_data[8];
    motor->get_enable_command(enable_data);
    can_buses[motor->getBusName()]->write_frame(motor->getCanId(), enable_data);
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
  struct can_frame frame;
  // 遍历所有打开的总线读取数据
  for (auto const& [name, bus] : can_buses) 
  {
    while (bus->read_frame(frame) > 0) 
    {
      uint32_t received_id = frame.can_id & 0x7FF;
      
      // 遍历所有电机，匹配 ID 和 总线名称
      for (size_t i = 0; i < joints_motors.size(); i++)
      {
          // 达妙 MIT 反馈：支持 ID 直接匹配 或 ID=0 且数据首字节匹配
          bool id_match = (joints_motors[i]->getCanId() == received_id);
          bool payload_match = (received_id == 0x00 && (frame.data[0] & 0x0F) == joints_motors[i]->getCanId());

          if ((id_match || payload_match) && joints_motors[i]->getBusName() == name)
          {
              joints_motors[i]->parse_feedback(frame.data);
              hw_states_[i] = joints_motors[i]->getAngleRad();
              
              static int r_count = 0;
              if (r_count++ % 100 == 0) {
                  RCLCPP_INFO(rclcpp::get_logger("ArmHardwareInterface"), 
                      "Feedback matched %s ID %d: Pos=%.3f", name.c_str(), joints_motors[i]->getCanId(), hw_states_[i]);
              }
              break;
          }
      }
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ArmHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  uint8_t cmd_data[8];
  for (size_t i = 0; i < joints_motors.size(); i++) 
  {
    if (!std::isnan(hw_commands_[i])) 
    {
      // 使用电机自己的 ID 发送指令到对应的总线，应用独立的 KP/KD
      joints_motors[i]->pack_mit_command(hw_commands_[i], 0.0f, joints_motors[i]->getKp(), joints_motors[i]->getKd(), 0.0f, cmd_data);
      can_buses[joints_motors[i]->getBusName()]->write_frame(joints_motors[i]->getCanId(), cmd_data);
      
      static int w_count = 0;
      if (w_count++ % 100 == 0) {
          RCLCPP_INFO(rclcpp::get_logger("ArmHardwareInterface"), 
              "Writing to %s ID %d: CmdPos=%.3f", joints_motors[i]->getBusName().c_str(), joints_motors[i]->getCanId(), hw_commands_[i]);
      }
    }
  }

  return hardware_interface::return_type::OK;
}

}  // namespace arm_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  arm_hardware_interface::ArmHardwareInterface, hardware_interface::SystemInterface)
