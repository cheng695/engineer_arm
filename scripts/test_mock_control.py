"""Isolated Mock-only control-chain smoke test. Never loads real CAN hardware."""
import os
import signal
import subprocess
import time
import rclpy
from std_msgs.msg import Bool, String, Float64MultiArray, Empty
from controller_manager_msgs.srv import ListControllers

def main():
    rclpy.init()
    node = rclpy.create_node("mock_control_contract")
    observed = []
    sub = node.create_subscription(String, "/arm/state/control_mode", lambda m: observed.append(m.data), 10)
    enable = node.create_publisher(Bool, "/arm/command/motor_enable", 10)
    joint = node.create_publisher(Float64MultiArray, "/arm/command/joint_velocity", 10)
    target = node.create_publisher(String, "/arm/command/named_target", 10)
    pause = node.create_publisher(Empty, "/arm/command/pause", 10)
    client = node.create_client(ListControllers, "/controller_manager/list_controllers")
    log = open("/tmp/arm_mock_contract.log", "w")
    process = subprocess.Popen(
        ["ros2", "launch", "my_robot_bringup", "arm_bringup.launch.py",
         "use_mock_hardware:=true", "robot:=v1.1"],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    def spin_until(predicate, timeout=15):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if predicate(): return
        raise RuntimeError("Mock condition timed out; see /tmp/arm_mock_contract.log")
    def wait_state(state, timeout=15):
        spin_until(lambda: bool(observed) and observed[-1] == state, timeout)
    try:
        spin_until(lambda: client.service_is_ready())
        expected = {"arm_joint_controller", "arm_cartesian_controller",
                    "arm_trajectory_controller", "arm_hold_controller"}
        def loaded():
            future = client.call_async(ListControllers.Request())
            spin_until(future.done)
            return expected <= {c.name for c in future.result().controller}
        spin_until(loaded)
        spin_until(lambda: enable.get_subscription_count() > 0)
        enable.publish(Bool(data=True))
        wait_state("HOLD")
        joint.publish(Float64MultiArray(data=[0.02] + [0.0] * 6))
        wait_state("JOINT")
        joint.publish(Float64MultiArray(data=[0.0] * 7))
        wait_state("HOLD")
        target.publish(String(data="up"))
        wait_state("TRAJECTORY")
        wait_state("HOLD")
        pause.publish(Empty())
        wait_state("PAUSED")
        enable.publish(Bool(data=False))
        wait_state("DISABLED")
        print("PASS: load, HOLD, JOINT, release, trajectory completion, PAUSED, DISABLED")
    finally:
        os.killpg(process.pid, signal.SIGINT)
        try: process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=5)
        log.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
