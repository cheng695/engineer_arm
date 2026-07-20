#ifndef ARM_CAN_DAMIAO_MOTOR_DM_DEVICE_COLLECTION_HPP
#define ARM_CAN_DAMIAO_MOTOR_DM_DEVICE_COLLECTION_HPP

#include "dm_motor.hpp"
#include "arm_can/canbus/can_socket.hpp"

#include <algorithm>
#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace arm_can::damiao_motor
{

/**
 * @brief DM 电机设备集合 —— 管理多条 CAN 总线上的多个 DM 电机。
 *
 * 封装了：
 * - CAN 总线的打开/关闭
 * - 反馈帧的读取与电机匹配
 * - MIT 命令的按总线轮询交替发送
 * - 批量使能/失能
 *
 * 这是 arm_can 库的主要入口类，上层（如 arm_hardware_interface）通过它
 * 操作电机，无需直接处理 CAN 套接字或协议细节。
 *
 * 使用方式：
 * @code
 *   DMDeviceCollection collection;
 *   collection.addMotor(std::make_shared<J4310>());
 *   collection.addMotor(std::make_shared<J8009>());
 *   // ... 设置每个电机的 can_id、bus_name、kp、kd、direction ...
 *   collection.openCANBuses();
 *   collection.enableAll();
 *
 *   // 控制循环
 *   while (running) {
 *     collection.readFeedback();
 *     // ... 计算控制量 ...
 *     collection.sendCommands(positions, velocities, efforts);
 *   }
 *
 *   collection.disableAll();
 *   collection.closeCANBuses();
 * @endcode
 */
class DMDeviceCollection
{
public:
  DMDeviceCollection() = default;
  ~DMDeviceCollection() = default;

  // 禁止拷贝
  DMDeviceCollection(const DMDeviceCollection&) = delete;
  DMDeviceCollection& operator=(const DMDeviceCollection&) = delete;

  // ========== 电机管理 ==========

  /**
   * @brief 向集合中添加一个电机。
   *
   * 电机在集合中的索引与添加顺序一致，
   * 后续 readFeedback/sendCommands 均按此索引操作。
   *
   * @param motor 电机实例（shared_ptr）
   * @return 该电机在集合中的索引
   */
  size_t addMotor(std::shared_ptr<DmMotor> motor)
  {
    motors_.push_back(motor);
    return motors_.size() - 1;
  }

  /** @brief 获取电机数量 */
  size_t size() const { return motors_.size(); }

  /** @brief 按索引访问电机 */
  std::shared_ptr<DmMotor> getMotor(size_t index)
  {
    return (index < motors_.size()) ? motors_[index] : nullptr;
  }

  /** @brief 按索引访问电机（只读） */
  const DmMotor* getMotorConst(size_t index) const
  {
    return (index < motors_.size()) ? motors_[index].get() : nullptr;
  }

  // ========== CAN 总线生命周期 ==========

  /**
   * @brief 打开所有 CAN 总线。
   *
   * 根据已添加电机的 bus_name 创建并打开对应的 CANSocket。
   * 同名总线共享同一个套接字。
   *
   * @return 成功打开的总线数量
   */
  int openCANBuses()
  {
    int opened = 0;
    for (auto& motor : motors_)
    {
      const auto& bus_name = motor->get_bus_name();
      if (!can_buses_.count(bus_name))
      {
        auto socket = std::make_shared<canbus::CANSocket>();
        if (socket->open(bus_name))
        {
          can_buses_[bus_name] = socket;
          opened++;
        }
      }
    }
    return opened;
  }

  /**
   * @brief 关闭所有 CAN 总线。
   */
  void closeCANBuses()
  {
    for (auto& [_, bus] : can_buses_)
      bus->close_socket();
    can_buses_.clear();
  }

  /**
   * @brief 检查指定总线是否已打开。
   */
  bool isBusOpen(const std::string& bus_name) const
  {
    auto it = can_buses_.find(bus_name);
    return it != can_buses_.end() && it->second->is_open();
  }

  // ========== 反馈读取 ==========

  /**
   * @brief 从所有 CAN 总线读取反馈，匹配并更新各电机状态。
   *
   * 非阻塞读取：每条总线上的所有可用帧都会被处理。
   * 支持三种匹配方式：
   * - 精确匹配：帧的 CAN ID == 电机的 recv_can_id
   * - 兼容匹配：帧的 CAN ID == 电机的 can_id
   * - 广播匹配：帧 ID == 0x00 时，从 data[0] 低 4 位提取电机 ID
   */
  void readFeedback()
  {
    can_frame frame;
    for (auto& [bus_name, bus] : can_buses_)
    {
      if (!bus->is_open()) continue;

      while (bus->read_frame(frame) > 0)
      {
        uint32_t id = frame.can_id & CAN_SFF_MASK;
        for (auto& motor : motors_)
        {
          if (motor->get_bus_name() != bus_name) continue;

          bool match = (motor->get_recv_can_id() == id)
                    || (motor->get_can_id() == id)
                    || (id == 0x00 && (frame.data[0] & 0x0F) == static_cast<uint8_t>(motor->get_can_id()));
          if (match)
          {
            motor->parse_feedback(frame.data);
            // 应用方向符号（正反转），使电机状态值即为关节值
            float dir = motor->get_direction();
            motor->state().angle_Rad    *= dir;
            motor->state().velocity_Rad *= dir;
            motor->state().torque_Nm    *= dir;
            break;  // 一帧只匹配一个电机
          }
        }
      }
    }
  }

  // ========== 命令发送 ==========

  /**
   * @brief 向所有电机发送 MIT 控制命令（按总线轮询交替发送）。
   *
   * 为确保多总线上的命令不互相阻塞，采用交替发送策略：
   * 每轮对每条总线发送一个电机的命令帧，帧间隔 100us。
   * 电机使用其预配置的 kp/kd，pos/vel/eff 由调用者提供。
   *
   * @param positions  各电机目标位置 (rad)，长度需等于 size()
   * @param velocities 各电机目标速度 (rad/s)
   * @param efforts    各电机目标转矩 (Nm)
   */
  void sendCommands(const std::vector<double>& positions,
                    const std::vector<double>& velocities,
                    const std::vector<double>& efforts)
  {
    sendCommandsWithGains(positions, velocities, efforts, {}, {});
  }

  /**
   * @brief 向所有电机发送 MIT 控制命令，并允许逐电机覆盖 Kp/Kd。
   *
   * kps/kds 为空或长度不足时，对应电机继续使用预配置的 kp/kd。
   */
  void sendCommandsWithGains(const std::vector<double>& positions,
                             const std::vector<double>& velocities,
                             const std::vector<double>& efforts,
                             const std::vector<double>& kps,
                             const std::vector<double>& kds)
  {
    // 按总线分组电机索引
    std::map<std::string, std::vector<size_t>> groups;
    for (size_t i = 0; i < motors_.size(); ++i)
      groups[motors_[i]->get_bus_name()].push_back(i);

    // 计算最大组大小（决定交替轮数）
    size_t max_per_bus = 0;
    for (const auto& [_, g] : groups)
      max_per_bus = std::max(max_per_bus, g.size());

    uint8_t data[8];
    for (size_t round = 0; round < max_per_bus; ++round)
    {
      for (auto& [bus_name, group] : groups)
      {
        if (round >= group.size()) continue;
        size_t idx = group[round];

        auto& motor = motors_[idx];
        float dir = motor->get_direction();

        motor->pack_mit_command(
            static_cast<float>(positions[idx] * dir),
            static_cast<float>(velocities[idx] * dir),
            static_cast<float>(idx < kps.size() ? kps[idx] : motor->get_kp()),
            static_cast<float>(idx < kds.size() ? kds[idx] : motor->get_kd()),
            static_cast<float>(efforts[idx]),
            data);

        auto it = can_buses_.find(bus_name);
        if (it != can_buses_.end() && it->second->is_open())
        {
          it->second->write_frame(motor->get_can_id(), data);
          std::this_thread::sleep_for(std::chrono::microseconds(kInterFrameDelayUs));
        }
      }
    }
  }

  // ========== 批量控制 ==========

  /** @brief 使能所有电机 */
  void enableAll()
  {
    sendBroadcastCommand([](DmMotor& m, uint8_t d[8]) { m.get_enable_command(d); });
  }

  /** @brief 失能所有电机 */
  void disableAll()
  {
    sendBroadcastCommand([](DmMotor& m, uint8_t d[8]) { m.get_disable_command(d); });
  }

  /** @brief 清除所有电机的错误 */
  void clearAllErrors()
  {
    sendBroadcastCommand([](DmMotor& m, uint8_t d[8]) { m.get_clear_errors_command(d); });
  }

  /** @brief 保存所有电机的零点 */
  void saveAllZeroPosition()
  {
    sendBroadcastCommand([](DmMotor& m, uint8_t d[8]) { m.get_save_zero_command(d); });
  }

  /** @brief 对单个电机发送命令帧 */
  bool sendToMotor(size_t index,
                   void (DmMotor::*cmd)(uint8_t[8]))
  {
    if (index >= motors_.size()) return false;

    uint8_t data[8];
    (motors_[index].get()->*cmd)(data);

    auto it = can_buses_.find(motors_[index]->get_bus_name());
    if (it != can_buses_.end() && it->second->is_open())
      return it->second->write_frame(motors_[index]->get_can_id(), data);
    return false;
  }

private:
  /** @brief 向所有电机逐帧发送同类命令（如使能/失能） */
  template <typename Func>
  void sendBroadcastCommand(Func get_command)
  {
    uint8_t data[8];
    for (auto& motor : motors_)
    {
      get_command(*motor, data);
      auto it = can_buses_.find(motor->get_bus_name());
      if (it != can_buses_.end() && it->second->is_open())
      {
        it->second->write_frame(motor->get_can_id(), data);
        std::this_thread::sleep_for(std::chrono::microseconds(kInterFrameDelayUs));
      }
    }
  }

  // ========== 数据成员 ==========

  /// 电机列表（按添加顺序，索引与硬件接口关节顺序对应）
  std::vector<std::shared_ptr<DmMotor>> motors_;

  /// CAN 总线映射（总线名 → 套接字）
  std::map<std::string, std::shared_ptr<canbus::CANSocket>> can_buses_;

  /// CAN 帧间延迟（微秒），防止总线拥塞
  static constexpr int kInterFrameDelayUs = 100;
};

}  // namespace arm_can::damiao_motor

#endif  // ARM_CAN_DAMIAO_MOTOR_DM_DEVICE_COLLECTION_HPP
