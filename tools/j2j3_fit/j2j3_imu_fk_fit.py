#!/usr/bin/env python3
"""Fit J2/J3 decoupling polynomial parameters against Link3 IMU pitch."""

import argparse
import csv
import json
import math
import time
from pathlib import Path
from typing import Dict, Sequence, Tuple

import numpy as np
from scipy.optimize import least_squares, minimize


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
        "joint2": ["i2", "j2", "motor2", "joint2rad", "j2rad", "joint2feedback"],
        "joint3": ["i3", "j3", "motor3", "joint3rad", "j3rad", "joint3feedback"],
        "imupitchdeg": [
            "i1",
            "pitch",
            "imupitch",
            "pitchdeg",
            "imu",
            "link3pitch",
            "link3imupitch",
        ],
    }
    for alias in aliases.get(key, []):
        alias_key = _normalize_name(alias)
        if alias_key in normalized:
            return normalized[alias_key]

    raise ValueError(
        f"CSV column '{requested}' not found. Available columns: {', '.join(fieldnames)}"
    )


def load_samples(
    csv_path: Path,
    j2_column: str,
    j3_column: str,
    imu_pitch_column: str,
    imu_pitch_unit: str,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, Dict[str, str]]:
    with csv_path.open(newline="") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            raise ValueError(f"{csv_path} has no header row")

        j2_name = _resolve_column(reader.fieldnames, j2_column)
        j3_name = _resolve_column(reader.fieldnames, j3_column)
        imu_name = _resolve_column(reader.fieldnames, imu_pitch_column)

        j2_values = []
        j3_values = []
        imu_values = []
        for row_index, row in enumerate(reader, start=2):
            try:
                j2 = float(row[j2_name])
                j3 = float(row[j3_name])
                imu_pitch = float(row[imu_name])
            except (TypeError, ValueError) as exc:
                raise ValueError(f"bad numeric value in {csv_path}:{row_index}") from exc

            if math.isfinite(j2) and math.isfinite(j3) and math.isfinite(imu_pitch):
                if imu_pitch_unit == "rad":
                    imu_pitch = math.degrees(imu_pitch)
                j2_values.append(j2)
                j3_values.append(j3)
                imu_values.append(imu_pitch)

    if not j2_values:
        raise ValueError(f"{csv_path} has no finite samples")

    columns = {"joint2": j2_name, "joint3": j3_name, "imu_pitch_deg": imu_name}
    return (
        np.array(j2_values, dtype=float),
        np.array(j3_values, dtype=float),
        np.array(imu_values, dtype=float),
        columns,
    )


def decouple_joint3(
    raw_j2: np.ndarray,
    raw_j3: np.ndarray,
    params: np.ndarray,
    scale_mode: str = "divide",
) -> np.ndarray:
    """Cubic correction with optional scale.

    Params:
      [a3, a2, a1, a0]:
        j3_decoupled = raw_j3 + a3*j2^3 + a2*j2^2 + a1*j2 + a0
      [a3, a2, a1, a0, scale]:
        j3_decoupled = (raw_j3 + a3*j2^3 + a2*j2^2 + a1*j2 + a0) / scale
        or j3_decoupled = scale*raw_j3 + a3*j2^3 + a2*j2^2 + a1*j2 + a0
    """
    a3, a2, a1, a0 = params[:4]
    correction = a3 * raw_j2**3 + a2 * raw_j2**2 + a1 * raw_j2 + a0
    if len(params) >= 5:
        scale = params[4]
        if scale_mode == "multiply":
            return scale * raw_j3 + correction
        if abs(scale) < 1e-9:
            scale = 1e-9 if scale >= 0.0 else -1e-9
        return (raw_j3 + correction) / scale
    return raw_j3 + correction


def link3_pitch_fk_deg(joint2: np.ndarray, joint3_decoupled: np.ndarray) -> np.ndarray:
    # URDF: joint2 axis is -Y and joint3 axis is +Y, so Link3 pitch is j3 - j2.
    return np.rad2deg(joint3_decoupled - joint2)


def residual_deg(
    params: np.ndarray,
    raw_j2: np.ndarray,
    raw_j3: np.ndarray,
    imu_pitch_deg: np.ndarray,
    imu_sign: float,
    imu_offset_deg: float,
    scale_mode: str,
) -> np.ndarray:
    joint3_decoupled = decouple_joint3(raw_j2, raw_j3, params, scale_mode)
    fk_pitch_deg = link3_pitch_fk_deg(raw_j2, joint3_decoupled)
    corrected_imu_pitch_deg = imu_sign * imu_pitch_deg + imu_offset_deg
    return fk_pitch_deg - corrected_imu_pitch_deg


def r_squared(y_true: np.ndarray, y_pred: np.ndarray) -> float:
    ss_res = float(np.sum((y_pred - y_true) ** 2))
    ss_tot = float(np.sum((y_true - np.mean(y_true)) ** 2))
    if ss_tot <= 0.0:
        return math.nan
    return 1.0 - ss_res / ss_tot


def save_fit_plot(
    plot_path: Path,
    raw_j2: np.ndarray,
    corrected_imu_pitch_deg: np.ndarray,
    fk_pitch_deg: np.ndarray,
    residual: np.ndarray,
    r2: float,
):
    plot_path.parent.mkdir(parents=True, exist_ok=True)

    try:
        import os

        os.environ.setdefault("MPLCONFIGDIR", str(plot_path.parent / ".matplotlib"))
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("[WARN] matplotlib is not installed; skip fit plot")
        return

    order = np.argsort(raw_j2)

    fig, (ax_fit, ax_residual) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    ax_fit.scatter(raw_j2, corrected_imu_pitch_deg, s=18, label="IMU corrected", alpha=0.75)
    ax_fit.plot(raw_j2[order], fk_pitch_deg[order], color="tab:red", label="FK prediction")
    ax_fit.set_ylabel("Pitch (deg)")
    ax_fit.set_title(f"J2/J3 IMU FK fit, R^2={r2:.6f}")
    ax_fit.grid(True, alpha=0.3)
    ax_fit.legend()

    ax_residual.axhline(0.0, color="black", linewidth=1.0, alpha=0.6)
    ax_residual.scatter(raw_j2, residual, s=18, color="tab:purple", alpha=0.75)
    ax_residual.set_xlabel("joint2 raw (rad)")
    ax_residual.set_ylabel("Residual (deg)")
    ax_residual.grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(plot_path, dpi=160)
    plt.close(fig)
    print(f"[SAVE] plot -> {plot_path}")


def finite_difference_gradient(fn, params, eps=1e-6):
    grad = np.zeros_like(params)
    for i in range(len(params)):
        step = np.zeros_like(params)
        step[i] = eps
        grad[i] = (fn(params + step) - fn(params - step)) / (2.0 * eps)
    return grad


def run_adam(residual_fn, initial_params, learning_rate, max_iter, tolerance):
    params = np.array(initial_params, dtype=float)
    m = np.zeros_like(params)
    v = np.zeros_like(params)
    beta1 = 0.9
    beta2 = 0.999
    eps = 1e-8
    last_loss = math.inf

    for step in range(1, max_iter + 1):
        r = residual_fn(params)
        loss = 0.5 * float(np.dot(r, r))
        if abs(last_loss - loss) < tolerance:
            return params, step
        last_loss = loss

        grad = finite_difference_gradient(lambda p: 0.5 * float(np.dot(residual_fn(p), residual_fn(p))), params)
        m = beta1 * m + (1.0 - beta1) * grad
        v = beta2 * v + (1.0 - beta2) * (grad * grad)
        m_hat = m / (1.0 - beta1**step)
        v_hat = v / (1.0 - beta2**step)
        params -= learning_rate * m_hat / (np.sqrt(v_hat) + eps)

    return params, max_iter


def fit(args):
    csv_path = Path(args.csv).expanduser().resolve()
    raw_j2, raw_j3, imu_pitch_deg, columns = load_samples(
        csv_path, args.j2_column, args.j3_column, args.imu_pitch_column, args.imu_pitch_unit
    )

    corrected_imu_pitch_deg = args.imu_sign * imu_pitch_deg + args.imu_offset_deg
    model = args.model

    if model == "fk-poly":
        initial_params = np.array(args.initial_params, dtype=float)
        if args.fit_scale and len(initial_params) == 4:
            initial_params = np.append(initial_params, args.initial_scale)

        def residual_fn(params):
            return residual_deg(
                params,
                raw_j2,
                raw_j3,
                imu_pitch_deg,
                args.imu_sign,
                args.imu_offset_deg,
                args.scale_mode,
            )

        optimizer = args.optimizer.lower()
        if optimizer in ("lm", "levenberg-marquardt", "levenberg_marquardt"):
            if len(raw_j2) < len(initial_params):
                raise SystemExit("Levenberg-Marquardt needs at least as many samples as parameters")
            if args.fit_scale and (args.scale_min is not None or args.scale_max is not None):
                lower = np.full_like(initial_params, -np.inf)
                upper = np.full_like(initial_params, np.inf)
                lower[-1] = -np.inf if args.scale_min is None else args.scale_min
                upper[-1] = np.inf if args.scale_max is None else args.scale_max
                initial_params[-1] = float(np.clip(initial_params[-1], lower[-1], upper[-1]))
                result = least_squares(
                    residual_fn,
                    initial_params,
                    bounds=(lower, upper),
                    method="trf",
                    max_nfev=args.max_iter,
                )
                optimizer = "trf-bounded-least-squares"
            else:
                result = least_squares(residual_fn, initial_params, method="lm", max_nfev=args.max_iter)
            params = result.x
            success = bool(result.success)
            message = str(result.message)
            iterations = int(result.nfev)
        elif optimizer in ("lbfgs", "l-bfgs", "l-bfgs-b"):
            def objective(params):
                r = residual_fn(params)
                return 0.5 * float(np.dot(r, r))

            result = minimize(
                objective,
                initial_params,
                method="L-BFGS-B",
                options={"maxiter": args.max_iter, "ftol": args.tolerance},
            )
            params = result.x
            success = bool(result.success)
            message = str(result.message)
            iterations = int(result.nit)
        elif optimizer == "adam":
            params, iterations = run_adam(
                residual_fn,
                initial_params,
                args.learning_rate,
                args.max_iter,
                args.tolerance,
            )
            success = True
            message = "Adam finished"
        else:
            raise SystemExit(f"unsupported optimizer: {args.optimizer}")

        residual = residual_fn(params)
        joint3_decoupled = decouple_joint3(raw_j2, raw_j3, params, args.scale_mode)
        fk_pitch_deg = link3_pitch_fk_deg(raw_j2, joint3_decoupled)
        coeff_names = ["a3", "a2", "a1", "a0"]
        if len(params) >= 5:
            coeff_names.append("scale")
            if args.scale_mode == "multiply":
                model_text = (
                    "joint3_decoupled = scale*raw_joint3 + "
                    "a3*raw_joint2^3 + a2*raw_joint2^2 + a1*raw_joint2 + a0; "
                    "fk_pitch_deg = rad2deg(joint3_decoupled - raw_joint2)"
                )
            else:
                model_text = (
                    "joint3_decoupled = (raw_joint3 + "
                    "a3*raw_joint2^3 + a2*raw_joint2^2 + a1*raw_joint2 + a0) / scale; "
                    "fk_pitch_deg = rad2deg(joint3_decoupled - raw_joint2)"
                )
        else:
            model_text = (
                "joint3_decoupled = raw_joint3 + "
                "a3*raw_joint2^3 + a2*raw_joint2^2 + a1*raw_joint2 + a0; "
                "fk_pitch_deg = rad2deg(joint3_decoupled - raw_joint2)"
            )
    else:
        raise SystemExit(f"unsupported model: {args.model}")

    r2 = r_squared(corrected_imu_pitch_deg, fk_pitch_deg)

    fit_result = {
        "source_csv": str(csv_path),
        "columns": columns,
        "model": model_text,
        "model_type": model,
        "scale_mode": args.scale_mode if args.fit_scale else None,
        "optimizer": optimizer,
        "optimizer_success": success,
        "optimizer_message": message,
        "iterations": iterations,
        "coeff_names": coeff_names,
        "coeff": params.tolist(),
        "initial_coeff": initial_params.tolist(),
        "sample_count": int(len(raw_j2)),
        "rmse_deg": float(np.sqrt(np.mean(residual**2))),
        "r_squared": r2,
        "max_abs_error_deg": float(np.max(np.abs(residual))),
        "mean_error_deg": float(np.mean(residual)),
        "imu_sign": args.imu_sign,
        "imu_offset_deg": args.imu_offset_deg,
        "imu_pitch_unit": args.imu_pitch_unit,
        "created_at": time.strftime("%Y-%m-%d %H:%M:%S"),
    }

    print()
    print("[J2/J3 DECOUPLING FIT]")
    print(f"  source: {csv_path}")
    print(f"  columns: j2={columns['joint2']}, j3={columns['joint3']}, imu={columns['imu_pitch_deg']}")
    print(f"  model: {model_text}")
    print(f"  optimizer: {optimizer} ({message})")
    print(f"  coeff {coeff_names}: {np.array2string(params, precision=10)}")
    print(f"  rmse: {fit_result['rmse_deg']:.8f} deg")
    print(f"  r_squared: {r2:.8f}")
    print(f"  max_abs_error: {fit_result['max_abs_error_deg']:.8f} deg")

    output_path = Path(args.output_json).expanduser().resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w") as f:
        json.dump(fit_result, f, indent=2)
        f.write("\n")
    print(f"[SAVE] fit -> {output_path}")

    if args.prediction_csv:
        prediction_path = Path(args.prediction_csv).expanduser().resolve()
        prediction_path.parent.mkdir(parents=True, exist_ok=True)
        with prediction_path.open("w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(
                [
                    "joint2_raw_rad",
                    "joint3_raw_rad",
                    "joint3_decoupled_rad",
                    "fk_pitch_deg",
                    "imu_pitch_deg",
                    "imu_pitch_corrected_deg",
                    "residual_deg",
                ]
            )
            for values in zip(
                raw_j2,
                raw_j3,
                joint3_decoupled,
                fk_pitch_deg,
                imu_pitch_deg,
                corrected_imu_pitch_deg,
                residual,
            ):
                writer.writerow(values)
        print(f"[SAVE] predictions -> {prediction_path}")

    if args.plot_png:
        plot_path = Path(args.plot_png).expanduser().resolve()
        save_fit_plot(
            plot_path,
            raw_j2,
            corrected_imu_pitch_deg,
            fk_pitch_deg,
            residual,
            r2,
        )

    return fit_result


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Fit a cubic J2/J3 decoupling polynomial by minimizing "
            "Link3 FK pitch minus IMU pitch."
        )
    )
    parser.add_argument(
        "dataset_csv",
        nargs="?",
        help="CSV/SCV file with motor2 rad, motor3 rad, and IMU pitch deg.",
    )
    parser.add_argument(
        "--csv",
        dest="csv",
        help="CSV/SCV file with motor2 rad, motor3 rad, and IMU pitch deg.",
    )
    parser.add_argument("--j2-column", default="joint2")
    parser.add_argument("--j3-column", default="joint3")
    parser.add_argument("--imu-pitch-column", default="imu_pitch_deg")
    parser.add_argument(
        "--model",
        default="fk-poly",
        choices=["fk-poly"],
        help=(
            "Fit decoupled joint3 polynomial, then compare FK pitch with IMU pitch."
        ),
    )
    parser.add_argument(
        "--imu-pitch-unit",
        default="deg",
        choices=["deg", "rad"],
        help="Unit of the IMU pitch column. Values are converted to deg before fitting.",
    )
    parser.add_argument(
        "--optimizer",
        default="lm",
        choices=["lm", "levenberg-marquardt", "lbfgs", "l-bfgs-b", "adam"],
        help="LM is the default because this is a small least-squares fit.",
    )
    parser.add_argument(
        "--initial-params",
        type=float,
        nargs=4,
        default=[0.0, 0.0, 1.0, 0.0],
        metavar=("A3", "A2", "A1", "A0"),
        help="Initial coeffs for raw_j3 + A3*j2^3 + A2*j2^2 + A1*j2 + A0.",
    )
    parser.add_argument(
        "--fit-scale",
        action="store_true",
        help="Also fit j3 scale: (raw_j3 + polynomial) / scale.",
    )
    parser.add_argument(
        "--scale-mode",
        default="divide",
        choices=["divide", "multiply"],
        help=(
            "Scale formula when --fit-scale is enabled: "
            "divide uses (raw_j3 + poly) / scale; multiply uses scale*raw_j3 + poly."
        ),
    )
    parser.add_argument(
        "--initial-scale",
        type=float,
        default=1.0,
        help="Initial scale when --fit-scale is enabled.",
    )
    parser.add_argument(
        "--scale-min",
        type=float,
        help="Minimum fitted scale. Only used with --fit-scale and least-squares optimizer.",
    )
    parser.add_argument(
        "--scale-max",
        type=float,
        help="Maximum fitted scale. Only used with --fit-scale and least-squares optimizer.",
    )
    parser.add_argument(
        "--imu-sign",
        type=float,
        default=1.0,
        help=(
            "Use 1 when IMU pitch uses the same sign as FK pitch. "
            "Use -1 if the IMU pitch sign is opposite."
        ),
    )
    parser.add_argument(
        "--imu-offset-deg",
        type=float,
        default=0.0,
        help="Constant offset added to IMU pitch before residual calculation.",
    )
    parser.add_argument("--max-iter", type=int, default=1000)
    parser.add_argument("--tolerance", type=float, default=1e-10)
    parser.add_argument("--learning-rate", type=float, default=1e-2)
    parser.add_argument("--output-json", default="tools/j2j3_fit/data/j2j3_imu_fk_fit.json")
    parser.add_argument("--prediction-csv", help="Optional per-sample prediction/residual CSV.")
    parser.add_argument(
        "--plot-png",
        default="tools/j2j3_fit/data/j2j3_imu_fk_fit.png",
        help="Fit plot PNG path. Use an empty string to disable plotting.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    if args.dataset_csv and args.csv:
        raise SystemExit("use either dataset_csv or --csv, not both")
    args.csv = args.csv or args.dataset_csv
    if not args.csv:
        try:
            args.csv = input("Dataset CSV path: ").strip()
        except EOFError:
            args.csv = ""
    if not args.csv:
        raise SystemExit("dataset CSV path is required")
    if args.scale_min is not None and args.scale_max is not None and args.scale_min > args.scale_max:
        raise SystemExit("--scale-min must be <= --scale-max")

    fit(args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
