#!/usr/bin/env python3
"""Collect static joint position/effort samples for gravity identification."""

import argparse
import csv
import math
import select
import statistics
import sys
import termios
import time
import tty
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


DEFAULT_JOINTS = [f"joint{i}" for i in range(1, 8)]


@dataclass
class StaticSample:
    stamp: float
    count: int
    positions: List[float]
    velocities: List[float]
    efforts: List[float]
    effort_stddev: List[float]
    j2j3_effort_derivative: float


class RawTerminal:
    def __enter__(self):
        self._old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        return self

    def __exit__(self, exc_type, exc, tb):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._old_settings)

    @staticmethod
    def read_key(timeout_s: float = 0.05) -> Optional[str]:
        ready, _, _ = select.select([sys.stdin], [], [], timeout_s)
        if not ready:
            return None
        return sys.stdin.read(1)


def _stamp_seconds(msg: JointState) -> float:
    stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    return stamp if stamp > 0.0 else time.time()


def _values_by_name(msg: JointState, values: Sequence[float], joints: Sequence[str]) -> Dict[str, float]:
    out = {}
    for joint in joints:
        try:
            idx = msg.name.index(joint)
        except ValueError as exc:
            names = ", ".join(msg.name)
            raise ValueError(f"joint '{joint}' not in JointState names: [{names}]") from exc
        out[joint] = values[idx] if idx < len(values) else math.nan
    return out


class GravityStaticSampler(Node):
    def __init__(self, args):
        super().__init__("gravity_static_sampler")
        self.args = args
        self.joints = list(args.joints)
        self.latest_state_msg: Optional[JointState] = None
        self.latest_effort_msg: Optional[JointState] = None
        self.samples: List[StaticSample] = []
        self.output_dir = Path(args.output_dir).expanduser().resolve()
        self.output_dir.mkdir(parents=True, exist_ok=True)

        self.create_subscription(JointState, args.topic, self._on_state, 50)
        if args.effort_topic == args.topic:
            self.latest_effort_msg = self.latest_state_msg
        else:
            self.create_subscription(JointState, args.effort_topic, self._on_effort, 50)
        self.get_logger().info(
            f"listening state_topic={args.topic}, effort_topic={args.effort_topic}, "
            f"joints={','.join(self.joints)}"
        )

    def _on_state(self, msg: JointState):
        self.latest_state_msg = msg
        if self.args.effort_topic == self.args.topic:
            self.latest_effort_msg = msg

    def _on_effort(self, msg: JointState):
        self.latest_effort_msg = msg

    def _read_latest(self):
        if self.latest_state_msg is None:
            raise RuntimeError(f"no JointState received yet on state topic {self.args.topic}")
        if self.latest_effort_msg is None:
            raise RuntimeError(f"no JointState received yet on effort topic {self.args.effort_topic}")

        state_msg = self.latest_state_msg
        effort_msg = self.latest_effort_msg
        pos = _values_by_name(state_msg, state_msg.position, self.joints)
        vel = _values_by_name(state_msg, state_msg.velocity, self.joints)
        eff = _values_by_name(effort_msg, effort_msg.effort, self.joints)
        eff = self._map_efforts(pos, eff)

        for name in self.joints:
            if not math.isfinite(pos[name]):
                raise RuntimeError(f"{name} position is missing/non-finite")
            if self.args.require_effort and not math.isfinite(eff[name]):
                raise RuntimeError(f"{name} effort is missing/non-finite")

        return _stamp_seconds(state_msg), pos, vel, eff

    def _j2j3_poly_derivative(self, j2_pos: float) -> float:
        return (
            self.args.j2j3_coupling
            + 3.0 * self.args.j2j3_poly_a3 * j2_pos * j2_pos
            + 2.0 * self.args.j2j3_poly_a2 * j2_pos
            + self.args.j2j3_poly_a1
        )

    def _map_efforts(self, pos: Dict[str, float], raw_eff: Dict[str, float]) -> Dict[str, float]:
        mapped = dict(raw_eff)
        if not self.args.decouple_j2j3_effort or self.args.effort_topic == self.args.topic:
            return mapped
        if "joint2" not in mapped or "joint3" not in mapped or "joint2" not in pos:
            return mapped

        j3_scale = self.args.j2j3_j3_scale if abs(self.args.j2j3_j3_scale) > 1e-9 else 1.0
        derivative = self._j2j3_poly_derivative(pos["joint2"])
        raw_j2_eff = raw_eff["joint2"]
        raw_j3_eff = raw_eff["joint3"]

        if self.args.j2j3_scale_mode in ("multiply", "mul"):
            joint3_eff = raw_j3_eff / j3_scale
            mapped["joint2"] = raw_j2_eff - derivative * joint3_eff
            mapped["joint3"] = joint3_eff
        else:
            mapped["joint2"] = raw_j2_eff - derivative * raw_j3_eff
            mapped["joint3"] = j3_scale * raw_j3_eff
        return mapped

    def capture_static_sample(self) -> Optional[StaticSample]:
        deadline = time.monotonic() + self.args.window
        frames = []

        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            try:
                frames.append(self._read_latest())
            except RuntimeError as exc:
                print(f"[WARN] {exc}")
                return None

        quiet_frames = []
        rejected = []
        for frame in frames:
            _, _, vel, _ = frame
            finite_velocities = [
                (joint, abs(vel[joint])) for joint in self.joints if math.isfinite(vel[joint])
            ]
            if not finite_velocities:
                rejected.append(("all_velocity_non_finite", math.nan))
                continue
            max_joint, max_abs_vel = max(finite_velocities, key=lambda item: item[1])
            if max_abs_vel <= self.args.max_abs_velocity:
                quiet_frames.append(frame)
            else:
                rejected.append((max_joint, max_abs_vel))

        if len(quiet_frames) < self.args.min_frames:
            reason = ""
            if rejected:
                max_joint, max_abs_vel = max(
                    rejected,
                    key=lambda item: item[1] if math.isfinite(item[1]) else float("inf"),
                )
                reason = (
                    f" max rejected velocity: {max_joint}={max_abs_vel:.5f} rad/s,"
                    f" threshold={self.args.max_abs_velocity:.5f}."
                )
            print(
                f"[DROP] only {len(quiet_frames)} quiet frames; "
                f"need >= {self.args.min_frames}.{reason} Hold still and retry."
            )
            return None

        sample = self._average_frames(quiet_frames)
        self.samples.append(sample)
        print(
            f"[SAMPLE {len(self.samples):03d}] frames={sample.count} "
            f"q=[{', '.join(f'{v:+.4f}' for v in sample.positions)}] "
            f"tau=[{', '.join(f'{v:+.4f}' for v in sample.efforts)}]"
        )
        return sample

    def _average_frames(self, frames) -> StaticSample:
        positions = []
        velocities = []
        efforts = []
        effort_stddev = []

        for joint in self.joints:
            pos_values = [frame[1][joint] for frame in frames]
            vel_values = [frame[2][joint] for frame in frames]
            eff_values = [frame[3][joint] for frame in frames]

            positions.append(float(statistics.fmean(pos_values)))
            velocities.append(float(statistics.fmean(vel_values)))
            efforts.append(float(statistics.fmean(eff_values)))
            effort_stddev.append(float(statistics.pstdev(eff_values)) if len(eff_values) > 1 else 0.0)

        avg_j2_pos = float(statistics.fmean(frame[1].get("joint2", 0.0) for frame in frames))
        return StaticSample(
            stamp=frames[-1][0],
            count=len(frames),
            positions=positions,
            velocities=velocities,
            efforts=efforts,
            effort_stddev=effort_stddev,
            j2j3_effort_derivative=self._j2j3_poly_derivative(avg_j2_pos),
        )

    def save(self):
        csv_path = self.output_dir / self.args.csv_name
        with csv_path.open("w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(
                ["stamp", "sample_frame_count"]
                + self.joints
                + [f"{joint}_velocity" for joint in self.joints]
                + [f"{joint}_effort" for joint in self.joints]
                + [f"{joint}_effort_stddev" for joint in self.joints]
                + ["j2j3_effort_derivative"]
            )
            for sample in self.samples:
                writer.writerow(
                    [sample.stamp, sample.count]
                    + sample.positions
                    + sample.velocities
                    + sample.efforts
                    + sample.effort_stddev
                    + [sample.j2j3_effort_derivative]
                )
        print(f"[SAVE] samples -> {csv_path}")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Press SPACE to collect averaged static q/tau samples from a JointState topic."
    )
    parser.add_argument("--topic", default="/arm_debug/processed_joint_states")
    parser.add_argument(
        "--effort-topic",
        default="/arm_debug/processed_joint_states",
        help=(
            "JointState topic used for effort values. Use /arm_debug/raw_motor_states "
            "to collect real motor feedback while keeping positions from --topic."
        ),
    )
    parser.add_argument("--joints", nargs="+", default=DEFAULT_JOINTS)
    parser.add_argument("--output-dir", default="tools/gravity_identification/data")
    parser.add_argument("--csv-name", default="gravity_static_samples.csv")
    parser.add_argument("--window", type=float, default=1.0, help="Averaging window in seconds.")
    parser.add_argument("--min-frames", type=int, default=20)
    parser.add_argument(
        "--max-abs-velocity",
        type=float,
        default=0.05,
        help="Keep only frames where every selected joint is below this velocity in rad/s.",
    )
    parser.add_argument(
        "--no-require-effort",
        dest="require_effort",
        action="store_false",
        help="Allow saving even if effort values are missing/non-finite.",
    )
    parser.add_argument(
        "--no-decouple-j2j3-effort",
        dest="decouple_j2j3_effort",
        action="store_false",
        help="Do not map raw J2/J3 motor efforts into decoupled joint-side efforts.",
    )
    parser.add_argument("--j2j3-coupling", type=float, default=0.986)
    parser.add_argument("--j2j3-j3-scale", type=float, default=1.0)
    parser.add_argument("--j2j3-poly-a3", type=float, default=0.0)
    parser.add_argument("--j2j3-poly-a2", type=float, default=0.0)
    parser.add_argument("--j2j3-poly-a1", type=float, default=0.0)
    parser.add_argument("--j2j3-scale-mode", choices=("divide", "multiply", "mul"), default="divide")
    parser.set_defaults(require_effort=True)
    parser.set_defaults(decouple_j2j3_effort=True)
    return parser.parse_args()


def main():
    args = parse_args()
    if args.window <= 0.0:
        raise SystemExit("--window must be > 0")
    if args.min_frames < 1:
        raise SystemExit("--min-frames must be >= 1")

    rclpy.init()
    node = GravityStaticSampler(args)

    print()
    print("Controls:")
    print("  SPACE : collect one averaged static posture sample")
    print("  s     : save CSV")
    print("  q     : save CSV and quit")
    print()

    try:
        with RawTerminal():
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.02)
                key = RawTerminal.read_key(timeout_s=0.02)
                if key == " ":
                    node.capture_static_sample()
                elif key == "s":
                    node.save()
                elif key == "q":
                    node.save()
                    break
    except KeyboardInterrupt:
        node.save()
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
