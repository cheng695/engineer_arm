#ifndef ARM_CAN_DAMIAO_MOTOR_DM_MOTOR_HPP
#define ARM_CAN_DAMIAO_MOTOR_DM_MOTOR_HPP

#include "motor.hpp"
#include <cmath>
#include <cstring>

namespace arm_can::damiao_motor
{

/**
 * @brief DM 系列电机参数范围（MIT 模式下的极限值）。
 *
 * 用于 16-bit 位置、12-bit 速度/转矩/Kp/Kd 的定点数编解码。
 */
struct DMParameters
{
  float P_MIN, P_MAX;
  float V_MIN, V_MAX;
  float T_MIN, T_MAX;
  float KP_MIN, KP_MAX;
  float KD_MIN, KD_MAX;

  static constexpr double rad_to_deg = 57.29577951308232;

  DMParameters(float pmin, float pmax, float vmin, float vmax,
               float tmin, float tmax, float kpmin, float kpmax,
               float kdmin, float kdmax)
    : P_MIN(pmin), P_MAX(pmax), V_MIN(vmin), V_MAX(vmax)
    , T_MIN(tmin), T_MAX(tmax), KP_MIN(kpmin), KP_MAX(kpmax)
    , KD_MIN(kdmin), KD_MAX(kdmax) {}
};

/**
 * @brief DM (DaMiao) 系列电机基类。
 *
 * 实现 MIT 控制模式的命令打包与反馈解析。
 * 具体型号（J4310/J4340/J8009）通过不同的 DMParameters 区分。
 */
class DmMotor : public MotorBase
{
public:
  explicit DmMotor(const DMParameters& params) : params_(params) {}
  virtual ~DmMotor() = default;

  // ========== 协议实现 ==========

  /** @brief 打包 MIT 模式控制命令到 8 字节缓冲区 */
  void pack_mit_command(float pos, float vel, float kp, float kd,
                        float torque, uint8_t data[8]) override
  {
    uint16_t pos_tmp = float_to_uint(pos, params_.P_MIN, params_.P_MAX, 16);
    uint16_t vel_tmp = float_to_uint(vel, params_.V_MIN, params_.V_MAX, 12);
    uint16_t kp_tmp  = float_to_uint(kp,  params_.KP_MIN, params_.KP_MAX, 12);
    uint16_t kd_tmp  = float_to_uint(kd,  params_.KD_MIN, params_.KD_MAX, 12);
    uint16_t tor_tmp = float_to_uint(torque, params_.T_MIN, params_.T_MAX, 12);

    data[0] = (pos_tmp >> 8);
    data[1] = (pos_tmp & 0xFF);
    data[2] = (vel_tmp >> 4);
    data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
    data[4] = (kp_tmp & 0xFF);
    data[5] = (kd_tmp >> 4);
    data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
    data[7] = (tor_tmp & 0xFF);
  }

  /** @brief 从 8 字节 CAN 帧解析反馈数据 */
  void parse_feedback(const uint8_t data[8]) override
  {
    // Byte 0: [ID: 0-3 bit] [Status/Error: 4-7 bit]
    state_.error_code = (data[0] >> 4);

    // 提取原始定点数
    uint16_t p_int = (data[1] << 8) | data[2];
    uint16_t v_int = (data[3] << 4) | (data[4] >> 4);
    uint16_t t_int = ((data[4] & 0x0F) << 8) | data[5];

    // 转换为浮点数
    state_.angle_Rad    = uint_to_float(p_int, params_.P_MIN, params_.P_MAX, 16);
    state_.angle_Deg    = state_.angle_Rad * params_.rad_to_deg;
    state_.velocity_Rad = uint_to_float(v_int, params_.V_MIN, params_.V_MAX, 12);
    state_.torque_Nm    = uint_to_float(t_int, params_.T_MIN, params_.T_MAX, 12);
    state_.temperature_C = static_cast<double>(data[6]);
  }

  /** @brief 使能命令帧（DaMiao MIT 模式：0xFFFFFFFC） */
  void get_enable_command(uint8_t data[8]) override
  {
    for (int i = 0; i < 7; i++) data[i] = 0xFF;
    data[7] = 0xFC;
  }

  /** @brief 失能命令帧（DaMiao MIT 模式：0xFFFFFFFD） */
  void get_disable_command(uint8_t data[8]) override
  {
    for (int i = 0; i < 7; i++) data[i] = 0xFF;
    data[7] = 0xFD;
  }

  /** @brief 清除错误命令帧（0xFFFFFFFB） */
  void get_clear_errors_command(uint8_t data[8]) override
  {
    for (int i = 0; i < 7; i++) data[i] = 0xFF;
    data[7] = 0xFB;
  }

  /** @brief 保存零点命令帧（0xFFFFFFFE） */
  void get_save_zero_command(uint8_t data[8])
  {
    for (int i = 0; i < 7; i++) data[i] = 0xFF;
    data[7] = 0xFE;
  }

  /** @brief 获取电机参数配置 */
  const DMParameters& parameters() const { return params_; }

private:
  // ========== 定点数编解码 ==========

  static float uint_to_float(int x_int, float x_min, float x_max, int bits)
  {
    float span = x_max - x_min;
    return static_cast<float>(x_int) * span
         / static_cast<float>((1 << bits) - 1) + x_min;
  }

  static int float_to_uint(float x, float x_min, float x_max, int bits)
  {
    float span = x_max - x_min;
    if (x < x_min) x = x_min;
    if (x > x_max) x = x_max;
    return static_cast<int>((x - x_min)
         * static_cast<float>((1 << bits) - 1) / span);
  }

protected:
  DMParameters params_;
};

// ========== 具体型号 ==========

/**
 * @brief J4310 电机（小转矩）
 *
 * 位置 ±3.14 rad, 速度 ±30 rad/s, 转矩 ±3 Nm
 */
class J4310 : public DmMotor
{
public:
  J4310() : DmMotor(DMParameters(
      -3.14f, 3.14f, -30.0f, 30.0f, -3.0f, 3.0f,
      0.0f, 500.0f, 0.0f, 5.0f)) {}
};

/**
 * @brief J4340 电机（中转矩）
 *
 * 位置 ±3.14 rad, 速度 ±50 rad/s, 转矩 ±9 Nm
 */
class J4340 : public DmMotor
{
public:
  J4340() : DmMotor(DMParameters(
      -3.14f, 3.14f, -50.0f, 50.0f, -9.0f, 9.0f,
      0.0f, 500.0f, 0.0f, 5.0f)) {}
};

/**
 * @brief J8009 电机（大转矩）
 *
 * 位置 ±3.14 rad, 速度 ±50 rad/s, 转矩 ±20 Nm
 */
class J8009 : public DmMotor
{
public:
  J8009() : DmMotor(DMParameters(
      -3.14f, 3.14f, -50.0f, 50.0f, -20.0f, 20.0f,
      0.0f, 500.0f, 0.0f, 5.0f)) {}
};

}  // namespace arm_can::damiao_motor

#endif  // ARM_CAN_DAMIAO_MOTOR_DM_MOTOR_HPP
