#!/usr/bin/env python3
"""Publish a J2/J3 decoupled JointState preview from raw motor angles."""

import argparse
import json
import math
import time
from pathlib import Path
from typing import Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


def decouple_joint3(raw_j2: float, raw_j3: float, coeff, scale_mode: str) -> float:
    a3, a2, a1, a0 = coeff[:4]
    correction = a3 * raw_j2**3 + a2 * raw_j2**2 + a1 * raw_j2 + a0
    if len(coeff) >= 5:
        scale = coeff[4]
        if scale_mode == "multiply":
            return scale * raw_j3 + correction
        if abs(scale) < 1e-9:
            scale = 1e-9 if scale >= 0.0 else -1e-9
        return (raw_j3 + correction) / scale
    return raw_j3 + correction


def decouple_joint3_velocity(
    raw_j2: float,
    raw_j2_vel: float,
    raw_j3_vel: float,
    coeff,
    scale_mode: str,
) -> float:
    a3, a2, a1, _ = coeff[:4]
    d_correction_d_j2 = 3.0 * a3 * raw_j2**2 + 2.0 * a2 * raw_j2 + a1
    correction_vel = d_correction_d_j2 * raw_j2_vel
    if len(coeff) >= 5:
        scale = coeff[4]
        if scale_mode == "multiply":
            return scale * raw_j3_vel + correction_vel
        if abs(scale) < 1e-9:
            scale = 1e-9 if scale >= 0.0 else -1e-9
        return (raw_j3_vel + correction_vel) / scale
    return raw_j3_vel + correction_vel


class J2J3MotorPitchPreview(Node):
    def __init__(self, args):
        super().__init__("j2j3_motor_pitch_preview")
        self.args = args
        self.coeff = args.coeff
        self.last_log_time = 0.0

        self.pitch_pub = self.create_publisher(Float64, args.pitch_topic, 10)
        self.state_pub = self.create_publisher(JointState, args.state_topic, 10)
        self.decoupled_pub = self.create_publisher(JointState, args.decoupled_topic, 10)
        self.create_subscription(JointState, args.raw_topic, self._on_raw_state, 10)

        self.get_logger().info(
            f"listening {args.raw_topic}; publishing {args.pitch_topic}, "
            f"{args.state_topic}, and {args.decoupled_topic}"
        )
        self.get_logger().info(
            "model: joint3_decoupled = raw_joint3 + "
            "a3*raw_joint2^3 + a2*raw_joint2^2 + a1*raw_joint2 + a0"
            " [divided by scale when coeff has 5 values]"
        )
        self.get_logger().info(f"coeff: {self.coeff}")

    def _joint_value(self, msg: JointState, joint_name: str) -> Tuple[float, float, float]:
        try:
            idx = msg.name.index(joint_name)
        except ValueError as exc:
            names = ", ".join(msg.name)
            raise ValueError(f"joint '{joint_name}' not in JointState names: [{names}]") from exc

        pos = msg.position[idx] if idx < len(msg.position) else math.nan
        vel = msg.velocity[idx] if idx < len(msg.velocity) else math.nan
        effort = msg.effort[idx] if idx < len(msg.effort) else math.nan
        return pos, vel, effort

    def _on_raw_state(self, msg: JointState):
        try:
            raw_j2, raw_j2_vel, _ = self._joint_value(msg, self.args.j2_joint)
            raw_j3, raw_j3_vel, _ = self._joint_value(msg, self.args.j3_joint)
        except ValueError as exc:
            now = time.time()
            if now - self.last_log_time > 2.0:
                self.get_logger().warn(str(exc))
                self.last_log_time = now
            return

        if not math.isfinite(raw_j2) or not math.isfinite(raw_j3):
            return

        decoupled_j3 = decouple_joint3(raw_j2, raw_j3, self.coeff, self.args.scale_mode)
        decoupled_j3_vel = decouple_joint3_velocity(
            raw_j2,
            raw_j2_vel,
            raw_j3_vel,
            self.coeff,
            self.args.scale_mode,
        )
        pitch_rad = decoupled_j3 - raw_j2
        pitch_deg = math.degrees(pitch_rad)

        pitch_msg = Float64()
        pitch_msg.data = pitch_deg
        self.pitch_pub.publish(pitch_msg)

        state = JointState()
        state.header = msg.header
        state.name = [
            "fk_pitch_deg",
            "fk_pitch_rad",
            "raw_joint2_rad",
            "raw_joint3_rad",
            "decoupled_joint3_rad",
            "decoupled_link3_pitch_rad",
            "raw_joint2_vel",
            "raw_joint3_vel",
            "decoupled_joint3_vel",
        ]
        state.position = [
            pitch_deg,
            pitch_rad,
            raw_j2,
            raw_j3,
            decoupled_j3,
            decoupled_j3 - raw_j2,
            raw_j2_vel,
            raw_j3_vel,
            decoupled_j3_vel,
        ]
        self.state_pub.publish(state)

        decoupled = JointState()
        decoupled.header = msg.header
        decoupled.name = list(msg.name)
        decoupled.position = list(msg.position)
        decoupled.velocity = list(msg.velocity)
        decoupled.effort = list(msg.effort)

        try:
            j3_idx = decoupled.name.index(self.args.j3_joint)
        except ValueError:
            return

        if j3_idx >= len(decoupled.position):
            decoupled.position.extend([math.nan] * (j3_idx + 1 - len(decoupled.position)))
        decoupled.position[j3_idx] = decoupled_j3

        if decoupled.velocity:
            if j3_idx >= len(decoupled.velocity):
                decoupled.velocity.extend([math.nan] * (j3_idx + 1 - len(decoupled.velocity)))
            decoupled.velocity[j3_idx] = decoupled_j3_vel

        self.decoupled_pub.publish(decoupled)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Preview fitted J2/J3 decoupling from raw motor angles for Foxglove."
    )
    parser.add_argument("--raw-topic", default="/arm_debug/raw_motor_states")
    parser.add_argument("--pitch-topic", default="/arm_debug/j2j3_fit_pitch_deg")
    parser.add_argument("--state-topic", default="/arm_debug/j2j3_fit_preview")
    parser.add_argument("--decoupled-topic", default="/arm_debug/j2j3_decoupled_preview")
    parser.add_argument("--j2-joint", default="joint2")
    parser.add_argument("--j3-joint", default="joint3")
    parser.add_argument(
        "--scale-mode",
        default=None,
        choices=["divide", "multiply"],
        help=(
            "Scale formula for 5 coeffs. If omitted and --fit-json exists, "
            "the mode is loaded from JSON."
        ),
    )
    parser.add_argument(
        "--fit-json",
        default="tools/j2j3_fit/data/j2j3_imu_fk_fit.json",
        help="Fit JSON to load coeff from when --coeff is not provided.",
    )
    parser.add_argument(
        "--coeff",
        type=float,
        nargs="+",
        default=None,
        metavar="COEFF",
        help=(
            "4 coeffs [A3 A2 A1 A0] or 5 coeffs [A3 A2 A1 A0 SCALE] for "
            "joint3_decoupled = (raw_j3 + A3*j2^3 + A2*j2^2 + A1*j2 + A0) / SCALE."
        ),
    )
    return parser.parse_args()


def load_coeff(args):
    if args.coeff is not None:
        return args.coeff

    fit_path = Path(args.fit_json).expanduser().resolve()
    if fit_path.exists():
        with fit_path.open() as f:
            fit = json.load(f)
        coeff = fit.get("coeff")
        if isinstance(coeff, list) and len(coeff) in (4, 5):
            print(f"[LOAD] coeff <- {fit_path}")
            if args.scale_mode is None:
                args.scale_mode = fit.get("scale_mode") or "divide"
            return [float(v) for v in coeff]
        raise SystemExit(f"{fit_path} does not contain 4 or 5 coeff values")

    if args.scale_mode is None:
        args.scale_mode = "divide"
    print(f"[WARN] {fit_path} not found; using fallback coeff [0, 0, 0.986, 0]")
    return [0.0, 0.0, 0.986, 0.0]


def main():
    args = parse_args()
    args.coeff = load_coeff(args)
    if args.scale_mode is None:
        args.scale_mode = "divide"
    rclpy.init()
    node = J2J3MotorPitchPreview(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
