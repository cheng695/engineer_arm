#!/usr/bin/env python3
"""Interactive sampler and polynomial fitter for J2/J3 coupling data."""

import argparse
import csv
import json
import math
import select
import sys
import termios
import time
import tty
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


@dataclass
class Sample:
    stamp: float
    x: float
    y: float
    x_vel: float
    y_vel: float
    x_effort: float
    y_effort: float


def _normalize_name(name: str) -> str:
    return "".join(ch for ch in name.lower() if ch.isalnum())


def _resolve_column(fieldnames: Sequence[str], requested: str) -> str:
    if requested in fieldnames:
        return requested

    normalized = {_normalize_name(name): name for name in fieldnames}
    key = _normalize_name(requested)
    if key in normalized:
        return normalized[key]

    aliases = {
        "joint2": ["j2", "motor2", "joint2rad", "j2rad", "joint2feedback"],
        "joint3": ["j3", "motor3", "joint3rad", "j3rad", "joint3feedback"],
        "stamp": ["time", "timestamp", "sec"],
    }
    for alias in aliases.get(key, []):
        alias_key = _normalize_name(alias)
        if alias_key in normalized:
            return normalized[alias_key]

    raise ValueError(
        f"CSV column '{requested}' not found. Available columns: {', '.join(fieldnames)}"
    )


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


class J2J3Sampler(Node):
    def __init__(self, args):
        super().__init__("j2j3_sampler_fit")
        self.args = args
        self.latest_msg: Optional[JointState] = None
        self.samples: List[Sample] = []
        self.output_dir = Path(args.output_dir).expanduser().resolve()
        self.output_dir.mkdir(parents=True, exist_ok=True)

        self.create_subscription(JointState, args.topic, self._on_joint_state, 10)
        self.get_logger().info(
            f"listening topic={args.topic}, x={args.x_joint}, y={args.y_joint}"
        )

    def _on_joint_state(self, msg: JointState):
        self.latest_msg = msg

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

    def _range_error(self, sample: Sample) -> Optional[str]:
        if not math.isfinite(sample.x) or not math.isfinite(sample.y):
            return "non-finite joint value"
        if sample.x < self.args.x_min or sample.x > self.args.x_max:
            return (
                f"{self.args.x_joint}={sample.x:+.6f} outside "
                f"[{self.args.x_min:+.6f}, {self.args.x_max:+.6f}]"
            )
        if sample.y < self.args.y_min or sample.y > self.args.y_max:
            return (
                f"{self.args.y_joint}={sample.y:+.6f} outside "
                f"[{self.args.y_min:+.6f}, {self.args.y_max:+.6f}]"
            )
        return None

    def _append_sample(self, sample: Sample, source: str) -> bool:
        range_error = self._range_error(sample)
        if range_error is not None:
            print(f"[DROP] {source}: {range_error}")
            return False
        self.samples.append(sample)
        return True

    def capture_sample(self) -> Optional[Sample]:
        if self.latest_msg is None:
            print("[WARN] no JointState received yet")
            return None

        try:
            x, x_vel, x_effort = self._joint_value(self.latest_msg, self.args.x_joint)
            y, y_vel, y_effort = self._joint_value(self.latest_msg, self.args.y_joint)
        except ValueError as exc:
            print(f"[WARN] {exc}")
            return None

        stamp = self.latest_msg.header.stamp.sec + self.latest_msg.header.stamp.nanosec * 1e-9
        if stamp <= 0.0:
            stamp = time.time()

        sample = Sample(stamp, x, y, x_vel, y_vel, x_effort, y_effort)
        if not self._append_sample(sample, "live sample"):
            return None
        print(
            f"[SAMPLE {len(self.samples):03d}] "
            f"{self.args.x_joint}={x:+.6f}, {self.args.y_joint}={y:+.6f}"
        )
        return sample

    def load_csv(self, csv_path: Path):
        csv_path = csv_path.expanduser().resolve()
        loaded = 0
        dropped = 0

        with csv_path.open(newline="") as f:
            reader = csv.DictReader(f)
            if reader.fieldnames is None:
                raise ValueError(f"{csv_path} has no header row")

            x_name = _resolve_column(reader.fieldnames, self.args.x_joint)
            y_name = _resolve_column(reader.fieldnames, self.args.y_joint)
            try:
                stamp_name = _resolve_column(reader.fieldnames, "stamp")
            except ValueError:
                stamp_name = None

            optional_columns: Dict[str, Optional[str]] = {}
            for attr, requested in (
                ("x_vel", f"{self.args.x_joint}_velocity"),
                ("y_vel", f"{self.args.y_joint}_velocity"),
                ("x_effort", f"{self.args.x_joint}_effort"),
                ("y_effort", f"{self.args.y_joint}_effort"),
            ):
                try:
                    optional_columns[attr] = _resolve_column(reader.fieldnames, requested)
                except ValueError:
                    optional_columns[attr] = None

            for row_index, row in enumerate(reader, start=2):
                try:
                    stamp = float(row[stamp_name]) if stamp_name is not None else time.time()
                    x = float(row[x_name])
                    y = float(row[y_name])
                    x_vel = self._optional_float(row, optional_columns["x_vel"])
                    y_vel = self._optional_float(row, optional_columns["y_vel"])
                    x_effort = self._optional_float(row, optional_columns["x_effort"])
                    y_effort = self._optional_float(row, optional_columns["y_effort"])
                except (TypeError, ValueError) as exc:
                    print(f"[DROP] {csv_path}:{row_index}: bad numeric value ({exc})")
                    dropped += 1
                    continue

                sample = Sample(stamp, x, y, x_vel, y_vel, x_effort, y_effort)
                if self._append_sample(sample, f"{csv_path}:{row_index}"):
                    loaded += 1
                else:
                    dropped += 1

        print(f"[LOAD] {csv_path}: kept {loaded}, dropped {dropped}")

    @staticmethod
    def _optional_float(row: Dict[str, str], column: Optional[str]) -> float:
        if column is None or row.get(column, "") == "":
            return math.nan
        return float(row[column])

    def fit_samples(self):
        min_points = self.args.degree + 1
        if len(self.samples) < min_points:
            print(
                f"[WARN] need at least {min_points} samples for degree {self.args.degree}, "
                f"currently {len(self.samples)}"
            )
            return None

        x = np.array([s.x for s in self.samples], dtype=float)
        y = np.array([s.y for s in self.samples], dtype=float)
        coeff = np.polyfit(x, y, deg=self.args.degree)
        y_hat = np.polyval(coeff, x)
        residual = y_hat - y
        rmse = float(np.sqrt(np.mean(residual**2)))
        max_abs = float(np.max(np.abs(residual)))

        fit = {
            "topic": self.args.topic,
            "x_joint": self.args.x_joint,
            "y_joint": self.args.y_joint,
            "model": f"{self.args.y_joint} = poly({self.args.x_joint})",
            "degree": self.args.degree,
            "coeff_high_to_low": coeff.tolist(),
            "sample_count": len(self.samples),
            "active_range_rad": {
                self.args.x_joint: [self.args.x_min, self.args.x_max],
                self.args.y_joint: [self.args.y_min, self.args.y_max],
            },
            "rmse_rad": rmse,
            "max_abs_error_rad": max_abs,
            "created_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        }

        print()
        print("[FIT]")
        print(f"  model: {fit['model']}")
        print(f"  degree: {self.args.degree}")
        print(f"  coeff high->low: {np.array2string(coeff, precision=10)}")
        print(f"  rmse: {rmse:.8f} rad")
        print(f"  max_abs_error: {max_abs:.8f} rad")
        self.save(fit)
        return fit

    def save(self, fit=None):
        csv_path = self.output_dir / self.args.csv_name
        with csv_path.open("w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(
                [
                    "stamp",
                    self.args.x_joint,
                    self.args.y_joint,
                    f"{self.args.x_joint}_velocity",
                    f"{self.args.y_joint}_velocity",
                    f"{self.args.x_joint}_effort",
                    f"{self.args.y_joint}_effort",
                ]
            )
            for s in self.samples:
                writer.writerow([s.stamp, s.x, s.y, s.x_vel, s.y_vel, s.x_effort, s.y_effort])
        print(f"[SAVE] samples -> {csv_path}")

        if fit is not None:
            fit_path = self.output_dir / self.args.fit_name
            with fit_path.open("w") as f:
                json.dump(fit, f, indent=2)
                f.write("\n")
            print(f"[SAVE] fit -> {fit_path}")


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Press SPACE to sample J2/J3 angles from a JointState topic, "
            "press ENTER to fit y=poly(x)."
        )
    )
    parser.add_argument(
        "dataset_csv",
        nargs="?",
        help="Optional CSV dataset path to import before sampling/fitting.",
    )
    parser.add_argument("--topic", default="/arm_debug/raw_motor_states")
    parser.add_argument("--x-joint", default="joint2")
    parser.add_argument("--y-joint", default="joint3")
    parser.add_argument("--degree", type=int, default=1)
    parser.add_argument("--output-dir", default="tools/j2j3_fit/data")
    parser.add_argument("--csv-name", default="j2j3_samples.csv")
    parser.add_argument("--fit-name", default="j2j3_fit.json")
    parser.add_argument("--input-csv", help="Optional CSV to import before interactive sampling.")
    parser.add_argument(
        "--fit-and-exit",
        action="store_true",
        help="Fit imported samples, save outputs, and exit without interactive sampling.",
    )
    parser.add_argument("--x-min", type=float, default=-0.262, help="Minimum allowed x/joint2 rad.")
    parser.add_argument("--x-max", type=float, default=1.204, help="Maximum allowed x/joint2 rad.")
    parser.add_argument("--y-min", type=float, default=-0.9599, help="Minimum allowed y/joint3 rad.")
    parser.add_argument("--y-max", type=float, default=1.221, help="Maximum allowed y/joint3 rad.")
    return parser.parse_args()


def main():
    args = parse_args()
    if args.dataset_csv and args.input_csv:
        raise SystemExit("use either dataset_csv or --input-csv, not both")
    if args.dataset_csv:
        args.input_csv = args.dataset_csv

    if args.degree < 1:
        raise SystemExit("--degree must be >= 1")
    if args.x_min > args.x_max:
        raise SystemExit("--x-min must be <= --x-max")
    if args.y_min > args.y_max:
        raise SystemExit("--y-min must be <= --y-max")

    rclpy.init()
    node = J2J3Sampler(args)
    if args.input_csv:
        node.load_csv(Path(args.input_csv))
        if args.fit_and_exit:
            node.fit_samples()
            node.destroy_node()
            rclpy.shutdown()
            return 0

    print()
    print(
        f"Active range: {args.x_joint}=[{args.x_min:+.6f}, {args.x_max:+.6f}], "
        f"{args.y_joint}=[{args.y_min:+.6f}, {args.y_max:+.6f}] rad"
    )
    print("Controls:")
    print("  SPACE : capture current sample")
    print("  ENTER : fit collected samples and save CSV/JSON")
    print("  s     : save CSV only")
    print("  q     : quit")
    print()

    try:
        with RawTerminal():
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.02)
                key = RawTerminal.read_key(timeout_s=0.02)
                if key == " ":
                    node.capture_sample()
                elif key in ("\n", "\r"):
                    node.fit_samples()
                elif key == "s":
                    node.save()
                elif key == "q":
                    break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
