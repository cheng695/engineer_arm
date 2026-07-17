#ifndef ARM_CAN_DAMIAO_MOTOR_MOTOR_HPP
#define ARM_CAN_DAMIAO_MOTOR_MOTOR_HPP

#include <cstdint>
#include <string>
#include <vector>

namespace arm_can::damiao_motor
{

/**
 * @brief 电机运行时状态快照。
 */
struct MotorState
{
  double angle_Deg    = 0.0;   // 当前角度 (度)
  double angle_Rad    = 0.0;   // 当前角度 (弧度)
  double velocity_Rad = 0.0;   // 当前速度 (rad/s)
  double velocity_Rpm = 0.0;   // 当前速度 (rpm)
  double current_A    = 0.0;   // 当前电流 (A)
  double torque_Nm    = 0.0;   // 当前转矩 (Nm)
  double temperature_C = 0.0;  // 温度 (C)

  double last_angle   = 0.0;   // 上一周期角度 (用于多圈累积)
  double add_angle    = 0.0;   // 累积角度
  uint8_t error_code  = 0;     // 错误码
};

/**
 * @brief 电机硬件配置与控制参数。
 */
struct MotorConfig
{
  uint32_t can_id    = 0;       // CAN 标识符
  uint32_t recv_can_id = 0;     // 电机反馈 CAN 标识符
  std::string bus_name = "can0"; // 所属 CAN 总线名称
  float kp           = 0.0f;    // MIT 模式比例增益
  float kd           = 0.3f;    // MIT 模式微分增益
  float direction    = 1.0f;    // 旋转方向 (+1.0 或 -1.0)
};

/**
 * @brief 抽象电机基类。
 *
 * 每个子类实现特定电机协议的：
 * - MIT 命令打包 (pack_mit_command)
 * - 反馈解析 (parse_feedback)
 * - 使能/失能/清错命令 (enable/disable/clear_errors)
 */
class MotorBase
{
protected:
  MotorState  state_;
  MotorConfig config_;
  bool        is_enabled_ = false;

public:
  MotorBase()          = default;
  virtual ~MotorBase() = default;

  // ========== 协议接口（纯虚函数） ==========

  /** @brief 获取使能命令帧（8 字节） */
  virtual void get_enable_command(uint8_t data[8]) = 0;

  /** @brief 获取失能命令帧（8 字节） */
  virtual void get_disable_command(uint8_t data[8]) = 0;

  /** @brief 获取清除错误命令帧（8 字节） */
  virtual void get_clear_errors_command(uint8_t data[8]) = 0;

  /** @brief 打包 MIT 模式控制命令 */
  virtual void pack_mit_command(
      float pos, float vel, float kp, float kd, float torque,
      uint8_t data[8]) = 0;

  /** @brief 从 8 字节 CAN 帧解析反馈数据 */
  virtual void parse_feedback(const uint8_t data[8]) = 0;

  // ========== 配置访问器 ==========

  MotorConfig&       config()       { return config_; }
  const MotorConfig& config() const { return config_; }

  MotorState&       state()       { return state_; }
  const MotorState& state() const { return state_; }

  bool is_enabled() const { return is_enabled_; }
  void set_enabled(bool enabled) { is_enabled_ = enabled; }

  // ========== 便捷访问器 ==========

  uint32_t    get_can_id()   const { return config_.can_id; }
  void        set_can_id(uint32_t id) { config_.can_id = id; }
  uint32_t    get_recv_can_id() const { return config_.recv_can_id; }
  void        set_recv_can_id(uint32_t id) { config_.recv_can_id = id; }

  std::string get_bus_name() const { return config_.bus_name; }
  void        set_bus_name(const std::string& name) { config_.bus_name = name; }

  float get_kp() const { return config_.kp; }
  void  set_kp(float kp) { config_.kp = kp; }

  float get_kd() const { return config_.kd; }
  void  set_kd(float kd) { config_.kd = kd; }

  float get_direction() const { return config_.direction; }
  void  set_direction(float dir) { config_.direction = dir; }

  double  get_angle_rad()    const { return state_.angle_Rad; }
  double  get_velocity_rad() const { return state_.velocity_Rad; }
  double  get_torque_nm()    const { return state_.torque_Nm; }
  uint8_t get_error_code()   const { return state_.error_code; }
};

}  // namespace arm_can::damiao_motor

#endif  // ARM_CAN_DAMIAO_MOTOR_MOTOR_HPP
