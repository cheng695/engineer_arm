#!/usr/bin/env python3
"""Compare measured static efforts against Pinocchio gravity torques."""

import argparse
import csv
import json
import math
import sys
import time
from pathlib import Path
from typing import Dict, List

import numpy as np


DEFAULT_JOINTS = [f"joint{i}" for i in range(1, 8)]


def _load_pinocchio():
    try:
        import pinocchio as pin  # type: ignore
    except ImportError as exc:
        raise SystemExit(
            "Python package 'pinocchio' is not installed. Install Pinocchio in the ROS/Python "
            "environment, then rerun this script."
        ) from exc
    return pin


def _read_samples(csv_path: Path, joints: List[str]):
    with csv_path.open(newline="") as f:
        rows = list(csv.DictReader(f))

    if not rows:
        raise ValueError(f"{csv_path} has no samples")

    required = joints + [f"{joint}_effort" for joint in joints]
    missing = [name for name in required if name not in rows[0]]
    if missing:
        raise ValueError(f"{csv_path} missing columns: {', '.join(missing)}")

    q = np.array([[float(row[joint]) for joint in joints] for row in rows], dtype=float)
    tau = np.array(
        [[float(row[f"{joint}_effort"]) for joint in joints] for row in rows],
        dtype=float,
    )
    return rows, q, tau


def _build_model(pin, urdf_path: Path):
    model = pin.Model()
    pin.buildModelFromUrdf(str(urdf_path), model)
    data = pin.Data(model)
    return model, data


def _joint_indices(model, joints: List[str]) -> Dict[str, Dict[str, int]]:
    indices = {}
    for joint in joints:
        joint_id = model.getJointId(joint)
        if joint_id >= len(model.joints):
            raise ValueError(f"joint '{joint}' not found in Pinocchio model")
        joint_model = model.joints[joint_id]
        indices[joint] = {
            "q": int(joint_model.idx_q),
            "v": int(joint_model.idx_v),
        }
    return indices


def _compute_gravity(pin, model, data, joint_indices, joints: List[str], q_samples):
    tau_pin = np.zeros((q_samples.shape[0], len(joints)), dtype=float)
    v = np.zeros(model.nv)
    a = np.zeros(model.nv)

    for row_idx, q_row in enumerate(q_samples):
        q_model = np.zeros(model.nq)
        for joint_idx, joint in enumerate(joints):
            q_model[joint_indices[joint]["q"]] = q_row[joint_idx]
        pin.rnea(model, data, q_model, v, a)
        for joint_idx, joint in enumerate(joints):
            tau_pin[row_idx, joint_idx] = data.tau[joint_indices[joint]["v"]]

    return tau_pin


def _fit_scale_bias(tau_pin, tau_measured):
    fitted = np.zeros_like(tau_pin)
    params = []

    for joint_idx in range(tau_pin.shape[1]):
        x = tau_pin[:, joint_idx]
        y = tau_measured[:, joint_idx]
        design = np.column_stack([x, np.ones_like(x)])
        scale, bias = np.linalg.lstsq(design, y, rcond=None)[0]
        fitted[:, joint_idx] = scale * x + bias
        params.append({"scale": float(scale), "bias": float(bias)})

    return fitted, params


def _metrics(pred, measured, joints: List[str]):
    residual = pred - measured
    per_joint = {}
    for idx, joint in enumerate(joints):
        r = residual[:, idx]
        y = measured[:, idx]
        span = float(np.max(y) - np.min(y))
        ss_res = float(np.sum(r**2))
        ss_tot = float(np.sum((y - np.mean(y)) ** 2))
        rmse = float(np.sqrt(np.mean(r**2)))
        per_joint[joint] = {
            "rmse_nm": rmse,
            "mae_nm": float(np.mean(np.abs(r))),
            "max_abs_nm": float(np.max(np.abs(r))),
            "measured_span_nm": span,
            "nrmse_by_span": float(rmse / span) if span > 1e-12 else math.nan,
            "r_squared": float(1.0 - ss_res / ss_tot) if ss_tot > 1e-12 else math.nan,
        }
    return {
        "overall_rmse_nm": float(np.sqrt(np.mean(residual**2))),
        "overall_mae_nm": float(np.mean(np.abs(residual))),
        "per_joint": per_joint,
    }


def _plot_results(output_dir: Path, joints: List[str], tau_pin, tau_measured, tau_fitted):
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("[WARN] matplotlib not installed; skipping plots")
        return []

    output_dir.mkdir(parents=True, exist_ok=True)
    saved = []
    x = np.arange(tau_measured.shape[0])

    fig, axes = plt.subplots(len(joints), 1, figsize=(11, 1.8 * len(joints)), sharex=True)
    if len(joints) == 1:
        axes = [axes]
    for idx, joint in enumerate(joints):
        axes[idx].plot(x, tau_measured[:, idx], "k.", label="measured")
        axes[idx].plot(x, tau_pin[:, idx], "-", label="pinocchio raw")
        axes[idx].plot(x, tau_fitted[:, idx], "--", label="scale+bias")
        axes[idx].set_ylabel(f"{joint}\nNm")
        axes[idx].grid(True, alpha=0.25)
    axes[0].legend(loc="best", ncol=3)
    axes[-1].set_xlabel("sample index")
    fig.tight_layout()
    path = output_dir / "pinocchio_gravity_timeseries.png"
    fig.savefig(path, dpi=160)
    plt.close(fig)
    saved.append(path)

    fig, axes = plt.subplots(2, 4, figsize=(13, 7))
    axes = axes.ravel()
    for idx, joint in enumerate(joints):
        ax = axes[idx]
        ax.scatter(tau_measured[:, idx], tau_pin[:, idx], s=18, label="raw")
        ax.scatter(tau_measured[:, idx], tau_fitted[:, idx], s=18, label="scale+bias")
        lo = min(np.min(tau_measured[:, idx]), np.min(tau_pin[:, idx]), np.min(tau_fitted[:, idx]))
        hi = max(np.max(tau_measured[:, idx]), np.max(tau_pin[:, idx]), np.max(tau_fitted[:, idx]))
        ax.plot([lo, hi], [lo, hi], "k:", linewidth=1)
        ax.set_title(joint)
        ax.set_xlabel("measured Nm")
        ax.set_ylabel("predicted Nm")
        ax.grid(True, alpha=0.25)
    for idx in range(len(joints), len(axes)):
        axes[idx].axis("off")
    axes[0].legend(loc="best")
    fig.tight_layout()
    path = output_dir / "pinocchio_gravity_scatter.png"
    fig.savefig(path, dpi=160)
    plt.close(fig)
    saved.append(path)

    raw_rmse = np.sqrt(np.mean((tau_pin - tau_measured) ** 2, axis=0))
    fit_rmse = np.sqrt(np.mean((tau_fitted - tau_measured) ** 2, axis=0))
    width = 0.36
    idx = np.arange(len(joints))
    fig, ax = plt.subplots(figsize=(10, 4.5))
    ax.bar(idx - width / 2, raw_rmse, width, label="raw")
    ax.bar(idx + width / 2, fit_rmse, width, label="scale+bias")
    ax.set_xticks(idx)
    ax.set_xticklabels(joints)
    ax.set_ylabel("RMSE Nm")
    ax.grid(True, axis="y", alpha=0.25)
    ax.legend()
    fig.tight_layout()
    path = output_dir / "pinocchio_gravity_rmse.png"
    fig.savefig(path, dpi=160)
    plt.close(fig)
    saved.append(path)

    return saved


def _write_prediction_csv(path: Path, rows, joints, tau_pin, tau_measured, tau_fitted):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(
            ["stamp"]
            + [f"{joint}_pinocchio" for joint in joints]
            + [f"{joint}_measured" for joint in joints]
            + [f"{joint}_scale_bias_fit" for joint in joints]
            + [f"{joint}_pinocchio_residual" for joint in joints]
            + [f"{joint}_scale_bias_residual" for joint in joints]
        )
        for row, pin_row, measured_row, fitted_row in zip(rows, tau_pin, tau_measured, tau_fitted):
            writer.writerow(
                [row.get("stamp", "")]
                + list(pin_row)
                + list(measured_row)
                + list(fitted_row)
                + list(pin_row - measured_row)
                + list(fitted_row - measured_row)
            )


def parse_args():
    parser = argparse.ArgumentParser(
        description="Read static q/tau CSV and compare measured efforts with Pinocchio gravity torque."
    )
    parser.add_argument(
        "dataset_csv",
        nargs="?",
        help="Static sample CSV generated by gravity_static_sampler.py.",
    )
    parser.add_argument(
        "--csv",
        default="tools/gravity_identification/data/gravity_static_samples.csv",
        help="Static sample CSV generated by gravity_static_sampler.py. Ignored when dataset_csv is provided.",
    )
    parser.add_argument(
        "--urdf",
        default="src/arm_description/urdf/v1_1_full_robot.urdf",
        help="URDF file used by Pinocchio.",
    )
    parser.add_argument("--joints", nargs="+", default=DEFAULT_JOINTS)
    parser.add_argument(
        "--effort-sign",
        type=float,
        default=1.0,
        choices=(-1.0, 1.0),
        help="Multiply measured effort by this sign before comparing.",
    )
    parser.add_argument(
        "--output-dir",
        default="tools/gravity_identification/data",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    pin = _load_pinocchio()

    csv_path = Path(args.dataset_csv or args.csv).expanduser().resolve()
    urdf_path = Path(args.urdf).expanduser().resolve()
    output_dir = Path(args.output_dir).expanduser().resolve()

    rows, q_samples, tau_measured = _read_samples(csv_path, args.joints)
    tau_measured = args.effort_sign * tau_measured

    model, data = _build_model(pin, urdf_path)
    joint_indices = _joint_indices(model, args.joints)
    tau_pin = _compute_gravity(pin, model, data, joint_indices, args.joints, q_samples)
    tau_fitted, fit_params = _fit_scale_bias(tau_pin, tau_measured)

    pin_metrics = _metrics(tau_pin, tau_measured, args.joints)
    fit_metrics = _metrics(tau_fitted, tau_measured, args.joints)

    report = {
        "created_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "csv": str(csv_path),
        "urdf": str(urdf_path),
        "sample_count": len(rows),
        "joints": args.joints,
        "effort_sign": args.effort_sign,
        "pinocchio_model": {"nq": model.nq, "nv": model.nv},
        "raw_pinocchio_metrics": pin_metrics,
        "scale_bias_fit": {
            "model": "tau_measured[j] ~= scale[j] * tau_pinocchio[j] + bias[j]",
            "params_by_joint": dict(zip(args.joints, fit_params)),
            "metrics": fit_metrics,
        },
    }

    output_dir.mkdir(parents=True, exist_ok=True)
    report_path = output_dir / "pinocchio_gravity_compare.json"
    with report_path.open("w") as f:
        json.dump(report, f, indent=2)
        f.write("\n")

    pred_path = output_dir / "pinocchio_gravity_predictions.csv"
    _write_prediction_csv(pred_path, rows, args.joints, tau_pin, tau_measured, tau_fitted)
    plot_paths = _plot_results(output_dir, args.joints, tau_pin, tau_measured, tau_fitted)

    print(f"[LOAD] samples={len(rows)} csv={csv_path}")
    print(f"[MODEL] nq={model.nq} nv={model.nv} urdf={urdf_path}")
    print(f"[RAW] overall_rmse={pin_metrics['overall_rmse_nm']:.6f} Nm")
    print(f"[SCALE_BIAS] overall_rmse={fit_metrics['overall_rmse_nm']:.6f} Nm")
    for joint in args.joints:
        raw = pin_metrics["per_joint"][joint]["rmse_nm"]
        fit = fit_metrics["per_joint"][joint]["rmse_nm"]
        params = report["scale_bias_fit"]["params_by_joint"][joint]
        print(
            f"  {joint}: raw_rmse={raw:.6f}, fit_rmse={fit:.6f}, "
            f"scale={params['scale']:.6f}, bias={params['bias']:.6f}"
        )
    print(f"[SAVE] report -> {report_path}")
    print(f"[SAVE] predictions -> {pred_path}")
    for path in plot_paths:
        print(f"[SAVE] plot -> {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
