#!/usr/bin/env python3
"""实时绘制 DLS 控制器的 TCP 期望、反馈及跟踪误差。"""

import math
from collections import deque

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import rclpy
from geometry_msgs.msg import PoseArray
from rclpy.node import Node


def quaternion_to_rpy(quaternion):
    """将 geometry_msgs Quaternion 转换为 roll/pitch/yaw。"""
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    sin_roll_cos_pitch = 2.0 * (w * x + y * z)
    cos_roll_cos_pitch = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sin_roll_cos_pitch, cos_roll_cos_pitch)

    sin_pitch = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sin_pitch) if abs(sin_pitch) >= 1.0 else math.asin(sin_pitch)

    sin_yaw_cos_pitch = 2.0 * (w * z + x * y)
    cos_yaw_cos_pitch = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(sin_yaw_cos_pitch, cos_yaw_cos_pitch)
    return [roll, pitch, yaw]


def unwrap_rpy(current, previous):
    """消除 RPY 穿过正负 pi 时的绘图跳变。"""
    if previous is None:
        return current
    result = []
    for value, old_value in zip(current, previous):
        while value - old_value > math.pi:
            value -= 2.0 * math.pi
        while value - old_value < -math.pi:
            value += 2.0 * math.pi
        result.append(value)
    return result


def orientation_error_angle(desired, actual):
    """计算两个单位四元数之间的最短旋转角。"""
    desired_norm = math.sqrt(
        desired.x * desired.x + desired.y * desired.y +
        desired.z * desired.z + desired.w * desired.w)
    actual_norm = math.sqrt(
        actual.x * actual.x + actual.y * actual.y +
        actual.z * actual.z + actual.w * actual.w)
    if desired_norm < 1e-12 or actual_norm < 1e-12:
        return 0.0
    dot = (
        desired.x * actual.x + desired.y * actual.y +
        desired.z * actual.z + desired.w * actual.w) / (desired_norm * actual_norm)
    return 2.0 * math.acos(min(1.0, max(0.0, abs(dot))))


class TcpTrackingPlotter(Node):
    def __init__(self):
        super().__init__("tcp_tracking_plotter")
        self.topic = self.declare_parameter(
            "topic", "/arm_debug/tcp_tracking").value
        self.window = float(self.declare_parameter("window", 20.0).value)
        self.maximum_points = int(self.declare_parameter("maximum_points", 5000).value)
        if self.window <= 0.0:
            raise ValueError("window 必须大于 0")
        if self.maximum_points < 10:
            raise ValueError("maximum_points 必须至少为 10")

        self.times = deque(maxlen=self.maximum_points)
        self.desired_position = [deque(maxlen=self.maximum_points) for _ in range(3)]
        self.actual_position = [deque(maxlen=self.maximum_points) for _ in range(3)]
        self.desired_rpy = [deque(maxlen=self.maximum_points) for _ in range(3)]
        self.actual_rpy = [deque(maxlen=self.maximum_points) for _ in range(3)]
        self.position_error = deque(maxlen=self.maximum_points)
        self.orientation_error = deque(maxlen=self.maximum_points)
        self.start_time = None
        self.last_desired_rpy = None
        self.last_actual_rpy = None

        self.subscription = self.create_subscription(
            PoseArray, self.topic, self.pose_callback, 10)
        self.get_logger().info(
            f"监听 {self.topic}，显示最近 {self.window:.1f} 秒 TCP 跟踪曲线")

    def pose_callback(self, message):
        if len(message.poses) < 2:
            self.get_logger().warning("TCP 跟踪消息必须至少包含两个 Pose")
            return

        desired = message.poses[0]
        actual = message.poses[1]
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        if stamp <= 0.0:
            stamp = self.get_clock().now().nanoseconds * 1e-9
        if self.start_time is None:
            self.start_time = stamp
        relative_time = stamp - self.start_time

        desired_position = [desired.position.x, desired.position.y, desired.position.z]
        actual_position = [actual.position.x, actual.position.y, actual.position.z]
        desired_rpy = unwrap_rpy(
            quaternion_to_rpy(desired.orientation), self.last_desired_rpy)
        actual_rpy = unwrap_rpy(
            quaternion_to_rpy(actual.orientation), self.last_actual_rpy)
        self.last_desired_rpy = desired_rpy
        self.last_actual_rpy = actual_rpy

        self.times.append(relative_time)
        for index in range(3):
            self.desired_position[index].append(desired_position[index])
            self.actual_position[index].append(actual_position[index])
            self.desired_rpy[index].append(desired_rpy[index])
            self.actual_rpy[index].append(actual_rpy[index])

        position_error = math.sqrt(sum(
            (desired_position[index] - actual_position[index]) ** 2
            for index in range(3)))
        self.position_error.append(position_error)
        self.orientation_error.append(
            orientation_error_angle(desired.orientation, actual.orientation))


class PlotWindow:
    def __init__(self, node):
        self.node = node
        self.figure, self.axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
        if self.figure.canvas.manager is not None:
            self.figure.canvas.manager.set_window_title("ROS 2 TCP Tracking")
        colors = ["tab:red", "tab:green", "tab:blue"]
        names = ["X", "Y", "Z"]
        angle_names = ["Roll", "Pitch", "Yaw"]

        self.position_lines = []
        self.rpy_lines = []
        for index in range(3):
            desired_line, = self.axes[0].plot(
                [], [], color=colors[index], label=f"{names[index]} desired")
            actual_line, = self.axes[0].plot(
                [], [], color=colors[index], linestyle="--", label=f"{names[index]} actual")
            self.position_lines.append((desired_line, actual_line))

            desired_rpy_line, = self.axes[1].plot(
                [], [], color=colors[index], label=f"{angle_names[index]} desired")
            actual_rpy_line, = self.axes[1].plot(
                [], [], color=colors[index], linestyle="--", label=f"{angle_names[index]} actual")
            self.rpy_lines.append((desired_rpy_line, actual_rpy_line))

        self.position_error_line, = self.axes[2].plot(
            [], [], color="tab:purple", label="Position error")
        self.orientation_error_line, = self.axes[3].plot(
            [], [], color="tab:orange", label="Orientation error")

        self.axes[0].set_ylabel("Position [m]")
        self.axes[1].set_ylabel("RPY [rad]")
        self.axes[2].set_ylabel("Position error [m]")
        self.axes[3].set_ylabel("Orientation error [rad]")
        self.axes[3].set_xlabel("Time [s]")
        for axis in self.axes:
            axis.grid(True, alpha=0.3)
            axis.legend(loc="upper left", ncol=3)
        self.figure.tight_layout()

        all_lines = [line for pair in self.position_lines for line in pair]
        all_lines += [line for pair in self.rpy_lines for line in pair]
        all_lines += [self.position_error_line, self.orientation_error_line]
        self.all_lines = all_lines
        self.animation = FuncAnimation(
            self.figure, self.update, interval=50, blit=False,
            cache_frame_data=False)

    def update(self, _frame):
        for _ in range(20):
            rclpy.spin_once(self.node, timeout_sec=0.0)
        if not self.node.times:
            return self.all_lines

        times = list(self.node.times)
        newest_time = times[-1]
        first_visible = max(0.0, newest_time - self.node.window)
        first_index = next(
            (index for index, value in enumerate(times) if value >= first_visible), 0)
        visible_times = times[first_index:]

        for index, (desired_line, actual_line) in enumerate(self.position_lines):
            desired_line.set_data(
                visible_times, list(self.node.desired_position[index])[first_index:])
            actual_line.set_data(
                visible_times, list(self.node.actual_position[index])[first_index:])
        for index, (desired_line, actual_line) in enumerate(self.rpy_lines):
            desired_line.set_data(
                visible_times, list(self.node.desired_rpy[index])[first_index:])
            actual_line.set_data(
                visible_times, list(self.node.actual_rpy[index])[first_index:])
        self.position_error_line.set_data(
            visible_times, list(self.node.position_error)[first_index:])
        self.orientation_error_line.set_data(
            visible_times, list(self.node.orientation_error)[first_index:])

        right_limit = max(self.node.window, newest_time)
        left_limit = max(0.0, right_limit - self.node.window)
        for axis in self.axes:
            axis.set_xlim(left_limit, right_limit)
            axis.relim()
            axis.autoscale_view(scalex=False, scaley=True)
        return self.all_lines


def main():
    rclpy.init()
    node = TcpTrackingPlotter()
    window = PlotWindow(node)
    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        del window
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
