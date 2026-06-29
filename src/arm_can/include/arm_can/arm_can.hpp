#ifndef ARM_CAN_HPP
#define ARM_CAN_HPP

/**
 * @file arm_can.hpp
 * @brief arm_can 库的统一便利头文件。
 *
 * arm_can 是独立的 CAN-FD 电机控制库，采用三层架构：
 *
 *   1. canbus      —— 底层 SocketCAN 封装
 *   2. damiao_motor —— MIT 协议电机驱动与多设备总线管理
 *   3. (未来) arm_component —— 机械臂级协调
 *
 * 依赖：仅需 Linux SocketCAN 系统头文件，无 ROS 依赖。
 */

#include "arm_can/canbus/can_socket.hpp"
#include "arm_can/damiao_motor/motor.hpp"
#include "arm_can/damiao_motor/dm_motor.hpp"
#include "arm_can/damiao_motor/dm_device_collection.hpp"

#endif  // ARM_CAN_HPP
