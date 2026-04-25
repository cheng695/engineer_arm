#include "arm_hardware_interface/arm_hardware_interface.hpp"

#include <chrono>
#include <cctype>
#include <thread>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <map>
#include <set>
#include <sstream>
#include <algorithm>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/float64_multi_array.hpp"

// 新增加的头文件
#include "arm_hardware_interface/socket_can.hpp"
#include "arm_hardware_interface/motor_drivers/dm_motor.hpp"

namespace arm_hardware_interface
{
// 文件级流程说明（ros2_control 生命周期）：
// 1) on_init      : 解析 URDF/参数，创建电机对象与 CAN 总线映射
// 2) on_activate  : 打开 CAN，总线就绪；等待外部话题触发电机上电
// 3) read         : 收反馈、更新关节状态、做 J2/J3 解耦、计算重力补偿
// 4) write        : 处理上电/下电、异常恢复、将控制命令编码后发到各 CAN 总线

// 使用命名空间简化代码
using namespace motor_drivers::DM;

namespace
{
constexpr size_t kJ1Index = 0;
constexpr size_t kJ2Index = 1;
constexpr size_t kJ3Index = 2;
constexpr size_t kJ6Index = 5;
constexpr size_t kJ7Index = 6;
constexpr size_t kJ5Index = 4;
constexpr double kJ2J3Coupling = 0.986;
constexpr int kSafeZeroFramesAfterEnable = 50;
constexpr double kVelocityArmThreshold = 0.02;

double lower_limit_for_joint_name(const std::string & joint_name)
{
  if (joint_name == "joint1") return -2.3562;
  if (joint_name == "joint2") return -0.2620;
  if (joint_name == "joint3") return -0.9599;
  if (joint_name == "joint4") return -2.3562;
  if (joint_name == "joint5") return -1.5708;
  if (joint_name == "joint6") return -1.5708;
  if (joint_name == "joint7") return -1.5708;
  return -std::numeric_limits<double>::infinity();
}

double upper_limit_for_joint_name(const std::string & joint_name)
{
  if (joint_name == "joint1") return 2.3562;
  if (joint_name == "joint2") return 1.1340;
  if (joint_name == "joint3") return 0.9599;
  if (joint_name == "joint4") return 2.3562;
  if (joint_name == "joint5") return 1.5708;
  if (joint_name == "joint6") return 1.5708;
  if (joint_name == "joint7") return 1.5708;
  return std::numeric_limits<double>::infinity();
}

double feedback_sign_for_joint_space(size_t joint_index)
{
  switch (joint_index) {
    case kJ1Index:
    case kJ5Index:
      return -1.0;
    default:
      return 1.0;
  }
}

double decouple_j3_to_joint_space(double raw_j3, double j2_value)
{
  return raw_j3 + kJ2J3Coupling * j2_value;
}

double couple_j3_to_motor_space(double joint_j3, double joint_j2)
{
  return joint_j3 + kJ2J3Coupling * joint_j2;
}

GravityCompensationMode parse_gravity_mode(const hardware_interface::HardwareInfo & info)
{
  const auto it = info.hardware_parameters.find("gravity_compensation_mode");
  if (it == info.hardware_parameters.end()) {
    return GravityCompensationMode::Off;
  }

  std::string mode = it->second;
  std::transform(mode.begin(), mode.end(), mode.begin(), ::tolower);

  if (mode == "off") {
    return GravityCompensationMode::Off;
  }
  if (mode == "assist") {
    return GravityCompensationMode::Assist;
  }
  if (mode == "gravity_only") {
    return GravityCompensationMode::GravityOnly;
  }

  RCLCPP_WARN(
    rclcpp::get_logger("ArmHardwareInterface"),
    "Unknown gravity_compensation_mode='%s', fallback to 'off'. Valid values: off, assist, gravity_only.",
    it->second.c_str());
  return GravityCompensationMode::Off;
}

const char * gravity_mode_to_cstr(GravityCompensationMode mode)
{
  switch (mode) {
    case GravityCompensationMode::Off:
      return "off";
    case GravityCompensationMode::Assist:
      return "assist";
    case GravityCompensationMode::GravityOnly:
      return "gravity_only";
    default:
      return "off";
  }
}
}  // namespace

namespace {
double g_last_raw_j3_pos = 0.0;
double g_last_raw_j3_vel = 0.0;
bool g_has_raw_j3_feedback = false;
}  // namespace

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
  use_real_joint_io_.resize(info_.joints.size(), true);
  joint_lower_limits_.resize(info_.joints.size(), -std::numeric_limits<double>::infinity());
  joint_upper_limits_.resize(info_.joints.size(), std::numeric_limits<double>::infinity());
  gravity_compensation_eff_.resize(info_.joints.size(), 0.0);
  direct_joint_vel_cmds_.resize(info_.joints.size(), 0.0);
  direct_joint_target_pos_.resize(info_.joints.size(), 0.0);
  gravity_mode_ = parse_gravity_mode(info_);

  std::set<std::string> active_real_joints;
  if (info_.hardware_parameters.find("active_real_joints") != info_.hardware_parameters.end()) {
    std::stringstream ss(info_.hardware_parameters.at("active_real_joints"));
    std::string joint_name;
    while (std::getline(ss, joint_name, ',')) {
      joint_name.erase(
        std::remove_if(joint_name.begin(), joint_name.end(), [](unsigned char c) { return std::isspace(c); }),
        joint_name.end());
      if (!joint_name.empty()) {
        active_real_joints.insert(joint_name);
      }
    }
  }

  if (!active_real_joints.empty()) {
    std::string enabled_joints_log;
    for (size_t i = 0; i < info_.joints.size(); ++i) {
      use_real_joint_io_[i] = active_real_joints.count(info_.joints[i].name) > 0;
      joint_lower_limits_[i] = lower_limit_for_joint_name(info_.joints[i].name);
      joint_upper_limits_[i] = upper_limit_for_joint_name(info_.joints[i].name);
      if (use_real_joint_io_[i]) {
        if (!enabled_joints_log.empty()) {
          enabled_joints_log += ", ";
        }
        enabled_joints_log += info_.joints[i].name;
      }
    }
    RCLCPP_WARN(
      rclcpp::get_logger("ArmHardwareInterface"),
      "[TEST] 单关节实测模式已启用。真实 CAN 关节: [%s]；其余关节将使用本地假反馈。",
      enabled_joints_log.c_str());
  }

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    joint_lower_limits_[i] = lower_limit_for_joint_name(info_.joints[i].name);
    joint_upper_limits_[i] = upper_limit_for_joint_name(info_.joints[i].name);
  }

  // --- Pinocchio 动力学库初始化 ---
  // 优先使用 robot_description(XML 字符串)，其次尝试 robot_description_path(URDF 文件路径)。
  // 初始化失败不会中断硬件接口，仅会禁用重力补偿。
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

  RCLCPP_INFO(
    rclcpp::get_logger("ArmHardwareInterface"),
    "重力补偿模式: %s",
    gravity_mode_to_cstr(gravity_mode_));
  if (gravity_mode_ != GravityCompensationMode::Off && !pinocchio_initialized_) {
    RCLCPP_WARN(
      rclcpp::get_logger("ArmHardwareInterface"),
      "重力补偿已请求，但 Pinocchio 未初始化成功。当前将退化为无重力补偿输出。");
  }

  // 清理旧数据，防止重复初始化
  joints_motors.clear();
  can_buses.clear();

  // 为每个关节显式初始化电机对象，并建立“关节 <-> 电机 <-> CAN总线”映射
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

      // 5. 确保该总线的 SocketCan 实例已存在（同总线多电机共用一个 socket）
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

  // 不在 activate 阶段直接上电，避免系统启动时电机立刻出力。
  // 上电由 /arm_motor_enable 话题显式触发（例如手柄按钮）。
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
    hold_sub_ = internal_node_->create_subscription<std_msgs::msg::Bool>(
        "/arm_hold_position", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
          if (msg->data) {
            hold_position_requested_ = true;
          }
        });
    direct_joint_vel_sub_ =
      internal_node_->create_subscription<example_interfaces::msg::Float64MultiArray>(
        "/arm_direct_joint_vel_cmds", 10,
        [this](const example_interfaces::msg::Float64MultiArray::SharedPtr msg) {
          if (msg->data.size() != direct_joint_vel_cmds_.size()) {
            RCLCPP_WARN_THROTTLE(
              rclcpp::get_logger("ArmHardwareInterface"),
              *internal_node_->get_clock(),
              2000,
              "[DIRECT] 收到的关节直通速度命令维度错误: expected=%zu actual=%zu",
              direct_joint_vel_cmds_.size(),
              msg->data.size());
            return;
          }

          direct_joint_vel_cmds_ = msg->data;
          last_direct_joint_vel_stamp_ = internal_node_->now();
          direct_joint_vel_active_ = true;
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
    const size_t motor_index = static_cast<size_t>(&motor - &joints_motors[0]);
    if (!use_real_joint_io_[motor_index]) {
      continue;
    }
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
  bool j3_feedback_updated = false;
  // 遍历所有 CAN 总线读取反馈帧
  for (auto const& [name, bus] : can_buses) 
  {
    while (bus->read_frame(frame) > 0) 
    {
      uint32_t received_id = frame.can_id & 0x7FF;
      
      for (size_t i = 0; i < joints_motors.size(); i++)
      {
          bool id_match = (joints_motors[i]->getCanId() == received_id);
          // 某些反馈帧使用 0x00 作为 CAN ID，并把真实电机 ID 放在 payload 低 4bit。
          bool payload_match = (received_id == 0x00 && (frame.data[0] & 0x0F) == joints_motors[i]->getCanId());

          if ((id_match || payload_match) && joints_motors[i]->getBusName() == name)
          {
              if (!use_real_joint_io_[i]) {
                break;
              }
              // 解析电机反馈报文（弧度、速度、力矩）
              joints_motors[i]->parse_feedback(frame.data);
              const double feedback_sign = feedback_sign_for_joint_space(i);
              hw_states_[i] = feedback_sign * joints_motors[i]->getAngleRad();
              hw_states_vel_[i] = feedback_sign * joints_motors[i]->getVelocityRad();
              hw_states_eff_[i] = feedback_sign * joints_motors[i]->getTorqueNm();

              if (i == kJ3Index)
              {
                g_last_raw_j3_pos = hw_states_[i];
                g_last_raw_j3_vel = hw_states_vel_[i];
                g_has_raw_j3_feedback = true;
                j3_feedback_updated = true;
              }
              
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

  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    if (!use_real_joint_io_[i]) {
      hw_states_[i] = hw_commands_[i];
      hw_states_vel_[i] = hw_commands_vel_[i];
      hw_states_eff_[i] = 0.0;
    }
  }

  // --- 运动学解耦 (J2 & J3 同步带耦合处理) ---
  // 电机侧反馈值与“关节实际角”不同，这里在状态侧先转换为控制器可直接使用的关节角。
  // J2 索引: 1, J3 索引: 2
  // read():  电机空间 -> 关节空间
  // write(): 关节空间 -> 电机空间
  // 两边必须保持互逆，否则会在同步目标值/重新使能时引入跳变。
  double raw_j3 = g_has_raw_j3_feedback ? g_last_raw_j3_pos : hw_states_[kJ3Index];
  double raw_j3_vel = g_has_raw_j3_feedback ? g_last_raw_j3_vel : hw_states_vel_[kJ3Index];
  if (use_real_joint_io_[kJ3Index] && (j3_feedback_updated || g_has_raw_j3_feedback))
  {
    hw_states_[kJ3Index] = decouple_j3_to_joint_space(raw_j3, hw_states_[kJ2Index]);
    hw_states_vel_[kJ3Index] = decouple_j3_to_joint_space(raw_j3_vel, hw_states_vel_[kJ2Index]);
  }

  static int j3_read_count = 0;
  if (j3_read_count++ % 100 == 0)
  {
    RCLCPP_WARN(
      rclcpp::get_logger("ArmHardwareInterface"),
      "[J3] enabled=%s j2=%.4f raw_j3=%.4f joint_j3=%.4f raw_j3_vel=%.4f joint_j3_vel=%.4f",
      motors_enabled_ ? "true" : "false",
      hw_states_[kJ2Index],
      raw_j3,
      hw_states_[kJ3Index],
      raw_j3_vel,
      hw_states_vel_[kJ3Index]);
  }

  static int j6_read_count = 0;
  if (j6_read_count++ % 100 == 0)
  {
    RCLCPP_WARN(
      rclcpp::get_logger("ArmHardwareInterface"),
      "[J6] enabled=%s cmd_pos=%.4f fb_pos=%.4f cmd_vel=%.4f fb_vel=%.4f cmd_eff=%.4f fb_eff=%.4f",
      motors_enabled_ ? "true" : "false",
      hw_commands_[kJ6Index],
      hw_states_[kJ6Index],
      hw_commands_vel_[kJ6Index],
      hw_states_vel_[kJ6Index],
      hw_commands_eff_[kJ6Index],
      hw_states_eff_[kJ6Index]);
  }

  static double last_logged_j6_cmd_pos = std::numeric_limits<double>::quiet_NaN();
  static double last_logged_j6_cmd_vel = std::numeric_limits<double>::quiet_NaN();
  const double j6_cmd_pos = hw_commands_[kJ6Index];
  const double j6_cmd_vel = hw_commands_vel_[kJ6Index];
  if (
    std::isnan(last_logged_j6_cmd_pos) ||
    std::abs(j6_cmd_pos - last_logged_j6_cmd_pos) > 1e-4 ||
    std::abs(j6_cmd_vel - last_logged_j6_cmd_vel) > 1e-4)
  {
    RCLCPP_WARN(
      rclcpp::get_logger("ArmHardwareInterface"),
      "[J6-CMD] enabled=%s hw_commands_pos=%.4f hw_commands_vel=%.4f",
      motors_enabled_ ? "true" : "false",
      j6_cmd_pos,
      j6_cmd_vel);
    last_logged_j6_cmd_pos = j6_cmd_pos;
    last_logged_j6_cmd_vel = j6_cmd_vel;
  }


  // --- Pinocchio 重力补偿计算 ---
  // 在 q=当前关节角、v=0、a=0 时，RNEA 输出即为静态重力项 tau_g。
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
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  const double direct_joint_cmd_timeout = 0.2;
  bool direct_joint_mode_active = false;
  if (internal_node_ && direct_joint_vel_active_) {
    const double age = (internal_node_->now() - last_direct_joint_vel_stamp_).seconds();
    direct_joint_mode_active = age >= 0.0 && age < direct_joint_cmd_timeout;
  }

  if (direct_joint_vel_active_ && !direct_joint_mode_active) {
    std::fill(direct_joint_vel_cmds_.begin(), direct_joint_vel_cmds_.end(), 0.0);
    direct_joint_vel_active_ = false;
  }

  if (direct_joint_mode_active && !direct_joint_mode_latched_) {
    direct_joint_target_pos_ = hw_states_;
    direct_joint_mode_latched_ = true;
    RCLCPP_WARN(
      rclcpp::get_logger("ArmHardwareInterface"),
      "[DIRECT] 进入关节直通模式：目标位置已从当前反馈锁存。");
  } else if (!direct_joint_mode_active && direct_joint_mode_latched_) {
    direct_joint_mode_latched_ = false;
    RCLCPP_WARN(
      rclcpp::get_logger("ArmHardwareInterface"),
      "[DIRECT] 退出关节直通模式。");
  }

  auto sync_commands_to_current_state = [this]() {
    for (size_t i = 0; i < joints_motors.size(); i++)
    {
      // Keep the desired position aligned with the latest feedback so that
      // re-enabling torque does not cause the controller to chase a stale target.
      hw_commands_[i] = hw_states_[i];
      direct_joint_target_pos_[i] = hw_states_[i];
      hw_commands_vel_[i] = 0.0;
      hw_commands_eff_[i] = 0.0;
    }
  };

  auto apply_direct_joint_commands = [this, &period]() {
    const double dt = period.seconds();
    for (size_t i = 0; i < hw_commands_.size() && i < direct_joint_vel_cmds_.size(); ++i) {
      hw_commands_vel_[i] = direct_joint_vel_cmds_[i];
      direct_joint_target_pos_[i] = std::clamp(
        direct_joint_target_pos_[i] + direct_joint_vel_cmds_[i] * dt,
        joint_lower_limits_[i],
        joint_upper_limits_[i]);
      hw_commands_[i] = direct_joint_target_pos_[i];
      hw_commands_eff_[i] = 0.0;
    }
  };

  // ====================================================================
  // 阶段 0: 处理使能/失能请求 (来自手柄按键话题)
  // ====================================================================
  if (enable_requested_.exchange(false))
  {
    // 防跳跃：在使能前，将目标命令同步为当前实际位置
    sync_commands_to_current_state();
    RCLCPP_WARN(rclcpp::get_logger("ArmHardwareInterface"),
        "[POWER] 位置已同步。命令已锁定到当前状态。");

    // 现在向所有电机发送使能指令（交替发送，避免拥堵）
    for (auto & motor : joints_motors)
    {
      const size_t motor_index = static_cast<size_t>(&motor - &joints_motors[0]);
      if (!use_real_joint_io_[motor_index]) {
        continue;
      }
      uint8_t cmd[8];
      motor->get_enable_command(cmd);
      can_buses[motor->getBusName()]->write_frame(motor->getCanId(), cmd);
      std::this_thread::sleep_for(std::chrono::microseconds(200));
    }
    motors_enabled_ = true;
    safe_zero_frames_after_enable_ = kSafeZeroFramesAfterEnable;
    velocity_commands_armed_ = false;
    RCLCPP_WARN(rclcpp::get_logger("ArmHardwareInterface"),
        "[POWER] 所有电机已使能 (扭矩已开启)，进入 %.2f 秒零命令暖启动窗口。",
        static_cast<double>(kSafeZeroFramesAfterEnable) / 100.0);
  }

  if (disable_requested_.exchange(false))
  {
    sync_commands_to_current_state();
    for (auto & motor : joints_motors)
    {
      const size_t motor_index = static_cast<size_t>(&motor - &joints_motors[0]);
      if (!use_real_joint_io_[motor_index]) {
        continue;
      }
      uint8_t cmd[8];
      motor->get_disable_command(cmd);
      can_buses[motor->getBusName()]->write_frame(motor->getCanId(), cmd);
      std::this_thread::sleep_for(std::chrono::microseconds(200));
    }
    motors_enabled_ = false;
    velocity_commands_armed_ = false;
    RCLCPP_WARN(rclcpp::get_logger("ArmHardwareInterface"),
        "[POWER] 所有电机已失能 (扭矩已关闭)");
  }

  if (hold_position_requested_.exchange(false))
  {
    sync_commands_to_current_state();
    std::fill(direct_joint_vel_cmds_.begin(), direct_joint_vel_cmds_.end(), 0.0);
    direct_joint_vel_active_ = false;
    direct_joint_mode_latched_ = false;
    velocity_commands_armed_ = false;
    RCLCPP_WARN(
      rclcpp::get_logger("ArmHardwareInterface"),
      "[HOLD] 已锁定当前位置并清零速度命令。");
  }

  // 如果电机未使能，发送被动探测帧 (KP=0, KD=0) 以触发反馈回复
  if (!motors_enabled_) {
    sync_commands_to_current_state();
    for (auto & motor : joints_motors) {
      const size_t motor_index = static_cast<size_t>(&motor - &joints_motors[0]);
      if (!use_real_joint_io_[motor_index]) {
        continue;
      }
      uint8_t probe[8];
      // 发送 目标=0, 速度=0, KP=0, KD=0, 扭矩=0：
      // 该帧用于“只读状态不出力”，便于未上电时仍刷新反馈。
      motor->pack_mit_command(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, probe);
      can_buses[motor->getBusName()]->write_frame(motor->getCanId(), probe);
      std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
    return hardware_interface::return_type::OK;
  }

  if (direct_joint_mode_active) {
    apply_direct_joint_commands();
  }

  if (!velocity_commands_armed_.load()) {
    bool velocities_near_zero = true;
    for (double cmd_vel : hw_commands_vel_) {
      if (std::abs(cmd_vel) > kVelocityArmThreshold) {
        velocities_near_zero = false;
        break;
      }
    }

    if (velocities_near_zero) {
      velocity_commands_armed_ = true;
      RCLCPP_WARN(
        rclcpp::get_logger("ArmHardwareInterface"),
        "[SAFE] 速度指令已回零，重新开放 velocity 通道。");
    }
  }

  int safe_frames_remaining = safe_zero_frames_after_enable_.load();
  if (safe_frames_remaining > 0) {
    if (direct_joint_mode_active) {
      apply_direct_joint_commands();
    }
    for (auto & motor : joints_motors) {
      const size_t motor_index = static_cast<size_t>(&motor - &joints_motors[0]);
      if (!use_real_joint_io_[motor_index]) {
        continue;
      }
      uint8_t zero_cmd[8];
      motor->pack_mit_command(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, zero_cmd);
      can_buses[motor->getBusName()]->write_frame(motor->getCanId(), zero_cmd);
      std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
    safe_zero_frames_after_enable_ = safe_frames_remaining - 1;

    if (safe_frames_remaining == kSafeZeroFramesAfterEnable || safe_frames_remaining == 1) {
      RCLCPP_WARN(
        rclcpp::get_logger("ArmHardwareInterface"),
        "[SAFE] enable后零命令暖启动中，剩余 %d 帧。",
        safe_frames_remaining - 1);
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
    if (!use_real_joint_io_[i]) {
      continue;
    }
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
      if (!use_real_joint_io_[i]) {
        continue;
      }
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
        double target_vel = velocity_commands_armed_.load() ? hw_commands_vel_[i] : 0.0;
        double target_eff = hw_commands_eff_[i];
        double kp = joints_motors[i]->getKp();
        double kd = joints_motors[i]->getKd();

        if (gravity_mode_ == GravityCompensationMode::Assist && pinocchio_initialized_) {
          target_eff += gravity_compensation_eff_[i];
        } else if (gravity_mode_ == GravityCompensationMode::GravityOnly) {
          target_pos = 0.0;
          target_vel = 0.0;
          target_eff = pinocchio_initialized_ ? gravity_compensation_eff_[i] : 0.0;
          kp = 0.0;
          kd = 0.0;
        }
        
        // --- 命令解耦 (J2 & J3 同步带反向补偿) ---
        // 如果是关节 3 (索引 2)，我们需要反转解耦公式:
        if (i == kJ3Index) {
            // 临时旁路 J3：用于隔离特殊关节的命令链问题。
            // 保持与极简 keepalive 一致，只发全零 MIT 命令。
            target_pos = 0.0;
            target_vel = 0.0;
            target_eff = 0.0;
            kp = 0.0;
            kd = 0.0;
        }

        joints_motors[i]->pack_mit_command(
            target_pos, static_cast<float>(target_vel),
            static_cast<float>(kp), static_cast<float>(kd),
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
