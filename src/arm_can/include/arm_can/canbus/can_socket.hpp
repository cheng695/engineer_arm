#ifndef ARM_CAN_CANBUS_CAN_SOCKET_HPP
#define ARM_CAN_CANBUS_CAN_SOCKET_HPP

#include <string>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

namespace arm_can::canbus
{

/**
 * @brief Linux SocketCAN 的轻量 RAII 封装。
 *
 * 负责 CAN 套接字的打开/关闭及非阻塞读写。
 * 一个 CANSocket 实例对应一条 CAN 总线接口（如 can0、can1），
 * 同一条总线上的多个设备共享同一个 CANSocket 实例。
 *
 * 使用方式：
 * - open("can0")  打开并绑定 CAN 接口，设为非阻塞模式
 * - write_frame() 发送标准/扩展 CAN 帧
 * - read_frame()  非阻塞读取 CAN 帧（无数据时返回 ≤0）
 * - 析构或 close_socket() 自动关闭套接字
 */
class CANSocket
{
public:
  /**
   * @brief 构造函数，初始化套接字文件描述符为 -1（未打开）。
   */
  CANSocket() : socket_fd_(-1) {}

  /**
   * @brief 析构函数，自动关闭 CAN 套接字。
   */
  ~CANSocket() { close_socket(); }

  // 禁止拷贝，允许移动
  CANSocket(const CANSocket&) = delete;
  CANSocket& operator=(const CANSocket&) = delete;
  CANSocket(CANSocket&& other) noexcept : socket_fd_(other.socket_fd_)
  {
    other.socket_fd_ = -1;
  }
  CANSocket& operator=(CANSocket&& other) noexcept
  {
    if (this != &other)
    {
      close_socket();
      socket_fd_ = other.socket_fd_;
      other.socket_fd_ = -1;
    }
    return *this;
  }

  /**
   * @brief 打开并绑定 CAN 接口，设为非阻塞模式。
   *
   * 执行步骤：
   * -# 创建 PF_CAN / SOCK_RAW 套接字
   * -# 通过 ioctl 获取接口索引
   * -# 绑定套接字到指定 CAN 接口
   * -# 设置 O_NONBLOCK 标志
   *
   * @param[in] interface_name CAN 接口名称（如 "can0", "can1"）
   * @return true  打开成功
   * @return false 任一步骤失败（套接字创建、ioctl、bind 等）
   */
  bool open(const std::string& interface_name)
  {
    socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd_ < 0) return false;

    struct ifreq ifr;
    std::strncpy(ifr.ifr_name, interface_name.c_str(), IFNAMSIZ);
    if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) return false;

    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(socket_fd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) return false;

    // 设为非阻塞模式
    int flags = fcntl(socket_fd_, F_GETFL, 0);
    fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK);

    return true;
  }

  /**
   * @brief 关闭 CAN 套接字。
   *
   * 如果套接字已打开（fd >= 0），关闭并重置为 -1。
   * 多次调用安全（幂等）。
   */
  void close_socket()
  {
    if (socket_fd_ >= 0)
    {
      close(socket_fd_);
      socket_fd_ = -1;
    }
  }

  /**
   * @brief 检查套接字是否已打开。
   */
  bool is_open() const { return socket_fd_ >= 0; }

  /**
   * @brief 向 CAN 总线写入一帧数据。
   *
   * 构造 struct can_frame，默认发送标准帧（11-bit ID），
   * 可通过 is_extended 发送扩展帧（29-bit ID）。
   *
   * @param[in] can_id       CAN 标识符（11-bit 或 29-bit）
   * @param[in] data         8 字节数据缓冲区
   * @param[in] is_extended  是否为扩展帧（默认 false，即标准帧）
   * @return true  写入成功（写入了完整的 can_frame）
   * @return false 写入失败或部分写入
   */
  bool write_frame(uint32_t can_id, const uint8_t data[8], bool is_extended = false)
  {
    if (socket_fd_ < 0) return false;

    struct can_frame frame;
    frame.can_id = can_id;
    if (is_extended) frame.can_id |= CAN_EFF_FLAG;
    frame.can_dlc = 8;
    std::memcpy(frame.data, data, 8);

    return write(socket_fd_, &frame, sizeof(struct can_frame)) == sizeof(struct can_frame);
  }

  /**
   * @brief 从 CAN 总线非阻塞读取一帧数据。
   *
   * 以非阻塞方式读取，无数据时返回 ≤0（不会阻塞调用线程）。
   *
   * @param[out] frame 读取到的 CAN 帧数据
   * @return >0  成功读取的字节数
   * @return ≤0  无数据或读取出错
   */
  int read_frame(struct can_frame& frame)
  {
    if (socket_fd_ < 0) return -1;
    return read(socket_fd_, &frame, sizeof(struct can_frame));
  }

private:
  /** @brief CAN 套接字文件描述符，-1 表示未打开 */
  int socket_fd_;
};

}  // namespace arm_can::canbus

#endif  // ARM_CAN_CANBUS_CAN_SOCKET_HPP
