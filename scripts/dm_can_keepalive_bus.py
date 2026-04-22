#!/usr/bin/env python3

import argparse
import signal
import socket
import struct
import sys
import time
from typing import Dict, List, Tuple


ENABLE_FRAME = bytes([0xFF] * 7 + [0xFC])
DISABLE_FRAME = bytes([0xFF] * 7 + [0xFD])
CLEAR_ERROR_FRAME = bytes([0xFF] * 7 + [0xFB])


MODEL_LIMITS = {
    "J4310": (-3.14, 3.14, -30.0, 30.0, -3.0, 3.0, 0.0, 500.0, 0.0, 5.0),
    "J4340": (-3.14, 3.14, -50.0, 50.0, -9.0, 9.0, 0.0, 500.0, 0.0, 5.0),
    "J8009": (-3.14, 3.14, -50.0, 50.0, -20.0, 20.0, 0.0, 500.0, 0.0, 5.0),
}


def float_to_uint(x: float, x_min: float, x_max: float, bits: int) -> int:
    span = x_max - x_min
    x = min(max(x, x_min), x_max)
    return int((x - x_min) * ((1 << bits) - 1) / span)


def uint_to_float(x_int: int, x_min: float, x_max: float, bits: int) -> float:
    span = x_max - x_min
    return x_int * span / ((1 << bits) - 1) + x_min


def pack_mit_command(model: str) -> bytes:
    p_min, p_max, v_min, v_max, t_min, t_max, kp_min, kp_max, kd_min, kd_max = MODEL_LIMITS[model]
    pos_tmp = float_to_uint(0.0, p_min, p_max, 16)
    vel_tmp = float_to_uint(0.0, v_min, v_max, 12)
    kp_tmp = float_to_uint(0.0, kp_min, kp_max, 12)
    kd_tmp = float_to_uint(0.0, kd_min, kd_max, 12)
    tor_tmp = float_to_uint(0.0, t_min, t_max, 12)

    data = bytearray(8)
    data[0] = (pos_tmp >> 8) & 0xFF
    data[1] = pos_tmp & 0xFF
    data[2] = (vel_tmp >> 4) & 0xFF
    data[3] = ((vel_tmp & 0x0F) << 4) | ((kp_tmp >> 8) & 0x0F)
    data[4] = kp_tmp & 0xFF
    data[5] = (kd_tmp >> 4) & 0xFF
    data[6] = ((kd_tmp & 0x0F) << 4) | ((tor_tmp >> 8) & 0x0F)
    data[7] = tor_tmp & 0xFF
    return bytes(data)


def parse_feedback(data: bytes, model: str) -> Tuple[int, float, float, float, int]:
    p_min, p_max, v_min, v_max, t_min, t_max, *_ = MODEL_LIMITS[model]
    error_code = data[0] >> 4
    p_int = (data[1] << 8) | data[2]
    v_int = (data[3] << 4) | (data[4] >> 4)
    t_int = ((data[4] & 0x0F) << 8) | data[5]
    pos = uint_to_float(p_int, p_min, p_max, 16)
    vel = uint_to_float(v_int, v_min, v_max, 12)
    torque = uint_to_float(t_int, t_min, t_max, 12)
    temp = data[6]
    return error_code, pos, vel, torque, temp


def build_can_frame(can_id: int, data: bytes) -> bytes:
    return struct.pack("=IB3x8s", can_id, 8, data)


def parse_motor_spec(spec: str) -> Tuple[int, str]:
    can_id_str, model = spec.split(":", 1)
    can_id = int(can_id_str, 0)
    if model not in MODEL_LIMITS:
        raise ValueError(f"unsupported model '{model}'")
    return can_id, model


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Minimal Damiao MIT-mode bus keepalive test over SocketCAN."
    )
    parser.add_argument("--bus", default="can1", help="SocketCAN interface name")
    parser.add_argument(
        "--motor",
        action="append",
        required=True,
        help="Motor spec in the form CAN_ID:MODEL, e.g. 0x05:J4340",
    )
    parser.add_argument("--rate", type=float, default=100.0, help="Bus keepalive rate in Hz")
    parser.add_argument("--duration", type=float, default=0.0, help="Run time in seconds, 0 means until Ctrl+C")
    parser.add_argument("--clear-errors", action="store_true", help="Send clear-error frame before enable")
    parser.add_argument("--disable-on-exit", action="store_true", help="Send disable frame on exit")
    parser.add_argument("--feedback-every", type=int, default=50, help="Print every N received frames per motor")
    args = parser.parse_args()

    motors: List[Tuple[int, str]] = [parse_motor_spec(spec) for spec in args.motor]
    zero_cmds: Dict[int, bytes] = {can_id: pack_mit_command(model) for can_id, model in motors}
    feedback_counts: Dict[int, int] = {can_id: 0 for can_id, _ in motors}
    motor_models: Dict[int, str] = {can_id: model for can_id, model in motors}

    sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    sock.bind((args.bus,))
    sock.settimeout(0.0)

    stop = False

    def handle_stop(_sig, _frame):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, handle_stop)
    signal.signal(signal.SIGTERM, handle_stop)

    motor_desc = ", ".join(f"0x{can_id:X}:{model}" for can_id, model in motors)
    print(f"[dm-bus] bus={args.bus} motors=[{motor_desc}] rate={args.rate:.1f}Hz")

    for can_id, _model in motors:
        if args.clear_errors:
            sock.send(build_can_frame(can_id, CLEAR_ERROR_FRAME))
            print(f"[dm-bus] clear-error sent to 0x{can_id:X}")
            time.sleep(0.005)

    for can_id, _model in motors:
        sock.send(build_can_frame(can_id, ENABLE_FRAME))
        print(f"[dm-bus] enable sent to 0x{can_id:X}")
        time.sleep(0.005)

    period = 1.0 / args.rate
    start_time = time.monotonic()
    next_tick = start_time
    tx_count = 0
    rx_count = 0

    try:
        while not stop:
            now = time.monotonic()
            if args.duration > 0.0 and now - start_time >= args.duration:
                break

            if now < next_tick:
                time.sleep(next_tick - now)
                continue

            for can_id, _model in motors:
                sock.send(build_can_frame(can_id, zero_cmds[can_id]))
                tx_count += 1

            next_tick += period

            while True:
                try:
                    frame = sock.recv(16)
                except BlockingIOError:
                    break
                except TimeoutError:
                    break

                can_id_raw, can_dlc, payload = struct.unpack("=IB3x8s", frame)
                recv_id = can_id_raw & 0x7FF
                payload = payload[:can_dlc]

                matched_id = None
                if recv_id in motor_models:
                    matched_id = recv_id
                elif recv_id == 0x00:
                    embedded_id = payload[0] & 0x0F
                    if embedded_id in motor_models:
                        matched_id = embedded_id

                if matched_id is None:
                    continue

                rx_count += 1
                feedback_counts[matched_id] += 1

                if args.feedback_every > 0 and feedback_counts[matched_id] % args.feedback_every == 0:
                    model = motor_models[matched_id]
                    err, pos, vel, torque, temp = parse_feedback(payload, model)
                    print(
                        f"[dm-bus] id=0x{matched_id:X} rx={feedback_counts[matched_id]} "
                        f"err=0x{err:02X} pos={pos:+.4f} vel={vel:+.4f} torque={torque:+.4f} temp={temp}"
                    )
    finally:
        if args.disable_on_exit:
            for can_id, _model in motors:
                try:
                    sock.send(build_can_frame(can_id, DISABLE_FRAME))
                    print(f"[dm-bus] disable sent to 0x{can_id:X}")
                    time.sleep(0.005)
                except OSError as exc:
                    print(f"[dm-bus] failed to disable 0x{can_id:X}: {exc}", file=sys.stderr)
        sock.close()
        print(f"[dm-bus] done, tx={tx_count}, rx={rx_count}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
