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
  hw_states_eff_.resize(info_.joints.size(), 0.0);
  hw_commands_eff_.resize(info_.joints.size(), 0.0);
  gravity_compensation_eff_.resize(info_.joints.size(), 0.0);

  // --- Pinocchio 动力学库初始化 ---
  try {
    std::string urdf_xml;
    std::string urdf_path;
    if (info_.hardware_parameters.find("robot_description") != info_.hardware_parameters.end()) 
    {
      urdf_xml = info_.hardware_parameters.at("robot_description");
    }
    if (info_.hardware_parameters.find("robot_description_path") != info_.hardware_parameters.end()) 
    {
      urdf_path = info_.hardware_parameters.at("robot_description_path");
    }

    model_ = std::make_unique<pinocchio::Model>();
    if (!urdf_xml.empty()) 
    {
      pinocchio::urdf::buildModelFromXML(urdf_xml, *model_);
      pinocchio_initialized_ = true;
    } 
    else if (!urdf_path.empty()) 
    {
      // 注意：Pinocchio 需要的是生成的 URDF 文件，而不是 XACRO。 
      // 这仅在路径指向生成的 URDF 文件时才工作。
      if (urdf_path.find(".xacro") != std::string::npos) 
      {
        RCLCPP_WARN(rclcpp::get_logger("ArmHardwareInterface"), "robot_description_path 指向了 .xacro 文件。Pinocchio 需要 .urdf。重力补偿已禁用。");
      } 
      else 
      {
        pinocchio::urdf::buildModel(urdf_path, *model_);
        pinocchio_initialized_ = true;
      }
    }

    if (pinocchio_initialized_) 
    {
      data_ = std::make_unique<pinocchio::Data>(*model_);
      RCLCPP_INFO(rclcpp::get_logger("ArmHardwareInterface"), "Pinocchio 模型初始化成功 (nq=%d)。重力补偿已启用。", model_->nq);
    } 
    else 
    {
      RCLCPP_WARN(rclcpp::get_logger("ArmHardwareInterface"), "既没有找到 robot_description 也没有找到有效的 URDF 路径。重力补偿已禁用。");
    }
  } 
  catch (const std::exception & e) 
  {
    RCLCPP_ERROR(rclcpp::get_logger("ArmHardwareInterface"), "Pinocchio 初始化失败: %s", e.what());
  }

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
  // 导出状态接口（给上层控制器读取）
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_vel_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_states_eff_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ArmHardwareInterface::export_command_interfaces()
{
  // 导出命令接口（给上层控制器写入）
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_vel_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_commands_eff_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn ArmHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // 激活阶段：打开所有配置的 CAN 总线
  for (auto const& [name, bus] : can_buses) 
  {
    RCLCPP_INFO(rclcpp::get_logger("ArmHardwareInterface"), "正在打开 CAN 总线: %s", name.c_str());
    if (!bus->open(name)) 
    {
      RCLCPP_ERROR(rclcpp::get_logger("ArmHardwareInterface"), "打开总线 %s 失败!", name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // 不要在此处使能电机。等待手柄按钮点火。
  motors_enabled_ = false;
  RCLCPP_WARN(rclcpp::get_logger("ArmHardwareInterface"),
      "[POWER] CAN 总线已打开。电机当前处于关闭状态。请按下手柄 Xbox 键使能电机。");

  // 创建内部节点用于电机使能/失能话题订阅
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
  // 处理电机使能/失能话题的回调
  if (internal_node_) 
  {
    rclcpp::spin_some(internal_node_);
  }

  struct can_frame frame;
  // 遍历所有 CAN 总线读取反馈帧
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
              // 解析电机反馈报文（弧度、速度、力矩）
              joints_motors[i]->parse_feedback(frame.data);
              hw_states_[i] = joints_motors[i]->getAngleRad();
              hw_states_vel_[i] = joints_motors[i]->getVelocityRad();
              hw_states_eff_[i] = joints_motors[i]->getTorqueNm();
              
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

  // --- 运动学解耦 (J2 & J3 同步带耦合处理) ---
  // J2 索引: 1, J3 索引: 2
  // 公式: J3_实际角度 = -(J3_电机值 - 0.986 * J2_实际角度)
  double raw_j3 = hw_states_[2];
  double raw_j3_vel = hw_states_vel_[2];
  hw_states_[2] = -(raw_j3 - 0.986 * hw_states_[1]);
  hw_states_vel_[2] = -(raw_j3_vel - 0.986 * hw_states_vel_[1]);

  // --- Pinocchio 重力补偿计算 ---
  if (pinocchio_initialized_) {
    Eigen::VectorXd q = Eigen::VectorXd::Zero(model_->nq);
    // 将 hw_states_ 映射到 Pinocchio 的 q 向量。 
    // 假设关节名称与模型关节名称匹配。
    // 为简单起见，我们假设前 n 个关节是匹配的。
    for (size_t i = 0; i < std::min((size_t)model_->nq, hw_states_.size()); ++i) {
      q[i] = hw_states_[i];
    }
    
    // 使用 RNEA 计算重力（速度 v=0, 加速度 a=0）
    Eigen::VectorXd v = Eigen::VectorXd::Zero(model_->nv);
    Eigen::VectorXd a = Eigen::VectorXd::Zero(model_->nv);
    pinocchio::rnea(*model_, *data_, q, v, a);
    
    // 存储重力扭矩（RNEA 在 v=0, a=0 时的结果存储在 data_->tau 中）
    for (size_t i = 0; i < std::min((size_t)model_->nv, gravity_compensation_eff_.size()); ++i) {
      gravity_compensation_eff_[i] = data_->tau[i];
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ArmHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // ====================================================================
  // 阶段 0: 处理使能/失能请求 (来自手柄按键话题)
  // ====================================================================
  if (enable_requested_.exchange(false))
  {
    // 防跳跃：在使能前，将目标命令同步为当前实际位置
    for (size_t i = 0; i < joints_motors.size(); i++)
    {
      hw_commands_[i] = hw_states_[i];
    }
    RCLCPP_WARN(rclcpp::get_logger("ArmHardwareInterface"),
        "[POWER] 位置已同步。命令已锁定到当前状态。");

    // 现在向所有电机发送使能指令（交替发送，避免拥堵）
    for (auto & motor : joints_motors)
    {
      uint8_t cmd[8];
      motor->get_enable_command(cmd);
      can_buses[motor->getBusName()]->write_frame(motor->getCanId(), cmd);
      std::this_thread::sleep_for(std::chrono::microseconds(200));
    }
    motors_enabled_ = true;
    RCLCPP_WARN(rclcpp::get_logger("ArmHardwareInterface"),
        "[POWER] 所有电机已使能 (扭矩已开启)");
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
        "[POWER] 所有电机已失能 (扭矩已关闭)");
  }

  // 如果电机未使能，发送被动探测帧 (KP=0, KD=0) 以触发反馈回复
  if (!motors_enabled_) {
    for (auto & motor : joints_motors) {
      uint8_t probe[8];
      // 发送 目标=0, 速度=0, KP=0, KD=0, 扭矩=0
      // 这不会产生扭矩，但会强制电机回复当前状态
      motor->pack_mit_command(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, probe);
      can_buses[motor->getBusName()]->write_frame(motor->getCanId(), probe);
      std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
    return hardware_interface::return_type::OK;
  }
  // ====================================================================
  // 阶段 1: 错误恢复
  // 如果任何电机报告 error_code != 1 (1 = 正常), 执行:
  //   清除错误 -> 1ms -> 失能 -> 1ms -> 使能
  // ====================================================================
  for (size_t i = 0; i < joints_motors.size(); i++)
  {
    uint8_t err = joints_motors[i]->getErrorCode();
    if (err != 0 && err != 1) // 0 = 尚未获取反馈, 1 = 正常
    {
      RCLCPP_WARN(rclcpp::get_logger("ArmHardwareInterface"),
          "[RECOVERY] 电机 ID %u 在总线 %s 上: error_code=0x%02X, 启动恢复序列...",
          joints_motors[i]->getCanId(), joints_motors[i]->getBusName().c_str(), err);

      auto& bus = can_buses[joints_motors[i]->getBusName()];
      uint32_t id = joints_motors[i]->getCanId();
      uint8_t cmd[8];

      // 步骤 1: 清除错误
      joints_motors[i]->get_clear_errors_command(cmd);
      bus->write_frame(id, cmd);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));

      // 步骤 2: 失能
      joints_motors[i]->get_disable_command(cmd);
      bus->write_frame(id, cmd);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));

      // 步骤 3: 重新使能
      joints_motors[i]->get_enable_command(cmd);
      bus->write_frame(id, cmd);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));

      RCLCPP_INFO(rclcpp::get_logger("ArmHardwareInterface"),
          "[RECOVERY] 电机 ID %u 恢复序列已发送。", joints_motors[i]->getCanId());
    }
  }

  // ====================================================================
  // 阶段 2: 交替式 CAN 写入
  // 按总线分组电机，然后交叉发送：bus0[0], bus1[0], bus0[1], bus1[1]...
  // 插入 100μs 延迟以防止 TX 邮箱溢出。
  // ====================================================================
  std::map<std::string, std::vector<size_t>> bus_groups;
  for (size_t i = 0; i < joints_motors.size(); i++)
  {
    if (!std::isnan(hw_commands_[i]))
    {
      bus_groups[joints_motors[i]->getBusName()].push_back(i);
    }
  }

  // 找到单条总线上最多的电机数量
  size_t max_depth = 0;
  for (auto const& [name, indices] : bus_groups)
  {
    if (indices.size() > max_depth) max_depth = indices.size();
  }

  // 交叉发送：在不同总线之间轮询
  uint8_t cmd_data[8];
  for (size_t slot = 0; slot < max_depth; slot++)
  {
    for (auto const& [bus_name, indices] : bus_groups)
    {
      if (slot < indices.size())
      {
        size_t i = indices[slot];
        
        double target_pos = hw_commands_[i];
        double target_vel = hw_commands_vel_[i];
        
        // 最终力矩 = 控制器下达的力矩 + 重力补偿力矩
        double target_eff = hw_commands_eff_[i] + gravity_compensation_eff_[i];
        
        // --- 命令解耦 (J2 & J3 同步带反向补偿) ---
        // 如果是关节 3 (索引 2)，我们需要反转解耦公式:
        if (i == 2) {
            target_pos = -hw_commands_[2] + 0.986 * hw_commands_[1];
            target_vel = -hw_commands_vel_[2] + 0.986 * hw_commands_vel_[1];
            // 力矩是否也需要解耦？ 
            // 通常只有 J3 电机扭矩作用于 J3 实际框架（如果 J2 固定）。
            // 暂时假设简单的力矩“透传”。
            target_eff = -hw_commands_eff_[2]; 
        }

        joints_motors[i]->pack_mit_command(
            target_pos, static_cast<float>(target_vel),
            joints_motors[i]->getKp(), joints_motors[i]->getKd(),
            static_cast<float>(target_eff), cmd_data);
        can_buses[bus_name]->write_frame(joints_motors[i]->getCanId(), cmd_data);

        // 帧间延迟：100微秒，让 CAN 控制器排空 TX 邮箱
        std::this_thread::sleep_for(std::chrono::microseconds(100));
      }
    }
  }

  static int w_count = 0;
  if (w_count++ % 200 == 0)
  {
    RCLCPP_INFO(rclcpp::get_logger("ArmHardwareInterface"),
        "[WRITE] 交叉发送完成。共 %zu 个电机分布在 %zu 条总线上。",
        joints_motors.size(), bus_groups.size());
  }

  return hardware_interface::return_type::OK;
}

}  // namespace arm_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  arm_hardware_interface::ArmHardwareInterface, hardware_interface::SystemInterface)
