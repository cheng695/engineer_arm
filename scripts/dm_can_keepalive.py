#!/usr/bin/env python3

import argparse
import math
import signal
import socket
import struct
import sys
import time
from typing import Tuple


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


def pack_mit_command(
    pos: float,
    vel: float,
    kp: float,
    kd: float,
    torque: float,
    model: str,
) -> bytes:
    p_min, p_max, v_min, v_max, t_min, t_max, kp_min, kp_max, kd_min, kd_max = MODEL_LIMITS[model]

    pos_tmp = float_to_uint(pos, p_min, p_max, 16)
    vel_tmp = float_to_uint(vel, v_min, v_max, 12)
    kp_tmp = float_to_uint(kp, kp_min, kp_max, 12)
    kd_tmp = float_to_uint(kd, kd_min, kd_max, 12)
    tor_tmp = float_to_uint(torque, t_min, t_max, 12)

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


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Minimal Damiao MIT-mode keepalive test over SocketCAN."
    )
    parser.add_argument("--bus", default="can0", help="SocketCAN interface name")
    parser.add_argument("--id", type=lambda x: int(x, 0), required=True, help="Motor CAN ID, e.g. 0x01")
    parser.add_argument(
        "--model",
        choices=sorted(MODEL_LIMITS.keys()),
        default="J4310",
        help="Motor model limits used for MIT packing/parsing",
    )
    parser.add_argument("--rate", type=float, default=100.0, help="Keepalive frequency in Hz")
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="Run time in seconds, 0 means run until Ctrl+C",
    )
    parser.add_argument(
        "--clear-errors",
        action="store_true",
        help="Send clear-error frame before enable",
    )
    parser.add_argument(
        "--disable-on-exit",
        action="store_true",
        help="Send disable frame on exit",
    )
    parser.add_argument(
        "--feedback-every",
        type=int,
        default=20,
        help="Print parsed feedback every N received frames",
    )
    args = parser.parse_args()

    if args.rate <= 0.0:
        print("rate must be > 0", file=sys.stderr)
        return 2

    period = 1.0 / args.rate
    zero_cmd = pack_mit_command(0.0, 0.0, 0.0, 0.0, 0.0, args.model)

    sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    sock.bind((args.bus,))
    sock.settimeout(0.0)

    stop = False

    def handle_stop(_sig, _frame):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, handle_stop)
    signal.signal(signal.SIGTERM, handle_stop)

    print(f"[dm-can] bus={args.bus} id=0x{args.id:X} model={args.model} rate={args.rate:.1f}Hz")

    if args.clear_errors:
        sock.send(build_can_frame(args.id, CLEAR_ERROR_FRAME))
        print("[dm-can] clear-error frame sent")
        time.sleep(0.01)

    sock.send(build_can_frame(args.id, ENABLE_FRAME))
    print("[dm-can] enable frame sent")
    time.sleep(0.01)

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

            sock.send(build_can_frame(args.id, zero_cmd))
            tx_count += 1
            next_tick += period

            while True:
                try:
                    frame = sock.recv(16)
                except BlockingIOError:
                    break
                except TimeoutError:
                    break

                can_id, can_dlc, payload = struct.unpack("=IB3x8s", frame)
                recv_id = can_id & 0x7FF
                payload = payload[:can_dlc]

                if recv_id != args.id and recv_id != 0x00:
                    continue

                if recv_id == 0x00 and ((payload[0] & 0x0F) != args.id):
                    continue

                rx_count += 1
                if args.feedback_every > 0 and (rx_count % args.feedback_every == 0):
                    err, pos, vel, torque, temp = parse_feedback(payload, args.model)
                    print(
                        f"[dm-can] rx={rx_count} err=0x{err:02X} pos={pos:+.4f} vel={vel:+.4f} "
                        f"torque={torque:+.4f} temp={temp}"
                    )
    finally:
        if args.disable_on_exit:
            try:
                sock.send(build_can_frame(args.id, DISABLE_FRAME))
                print("[dm-can] disable frame sent")
            except OSError as exc:
                print(f"[dm-can] failed to send disable frame: {exc}", file=sys.stderr)
        sock.close()
        print(f"[dm-can] done, tx={tx_count}, rx={rx_count}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
