#!/usr/bin/env python3
"""Bridge ReSpeaker DOA angles from Jetson to the Ultra96 PS UDP bridge.

This is a small hardware test path:

    ReSpeaker USB DOA -> Jetson -> UDP A command -> Ultra96 PS -> Dynamixel

The Ultra96 PS bridge already accepts the audio command as:

    A <angle_deg> <conf> <valid>

Angles are normalized to a relative -180..180 convention, then clamped to
the default -90..90 range to avoid commanding the pan axis behind the camera.
"""

import argparse
import os
import sys
import time


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
JETSON_DIR = os.path.join(ROOT, "jetson")
if JETSON_DIR not in sys.path:
    sys.path.insert(0, JETSON_DIR)

from src.audio_fallback import ReSpeakerDOA
from src.control.ultra_yubin_motor import UltraYubinMotorController


def normalize_relative_angle(angle, clamp_deg=180.0, front_doa_deg=90.0, sign=1.0):
    rel = ((float(angle) - float(front_doa_deg)) * float(sign) + 180.0) % 360.0 - 180.0
    if clamp_deg is not None:
        limit = abs(float(clamp_deg))
        rel = max(-limit, min(limit, rel))
    return rel


def angle_delta(a, b):
    return abs(((float(a) - float(b) + 180.0) % 360.0) - 180.0)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Send ReSpeaker DOA angles to Ultra96 PS audio command path."
    )
    parser.add_argument("--host", default=os.getenv("ULTRA_YUBIN_HOST", "192.168.3.1"))
    parser.add_argument("--port", type=int, default=int(os.getenv("ULTRA_YUBIN_PORT", "5016")))
    parser.add_argument("--timeout-sec", type=float, default=float(os.getenv("ULTRA_YUBIN_TIMEOUT_SEC", "0.08")))
    parser.add_argument("--offset", type=int, default=int(os.getenv("ULTRA_YUBIN_DOA_OFFSET", "0")))
    parser.add_argument("--front-doa-deg", type=float,
                        default=float(os.getenv("TELLO_AUDIO_MOTOR_ZERO_DOA_DEG", "90")),
                        help="ReSpeaker DOA angle that maps to motor/front 0 deg")
    parser.add_argument("--sign", type=float,
                        default=float(os.getenv("TELLO_AUDIO_DOA_SIGN", "1")),
                        help="Motor angle sign after subtracting front-doa-deg")
    parser.add_argument("--period-sec", type=float, default=0.10)
    parser.add_argument("--min-change-deg", type=float, default=4.0)
    parser.add_argument("--keepalive-sec", type=float, default=1.0)
    parser.add_argument("--clamp-deg", type=float, default=180.0)
    parser.add_argument("--no-clamp", action="store_true")
    parser.add_argument("--center-on-start", action="store_true")
    parser.add_argument("--once", action="store_true")
    parser.add_argument("--duration-sec", type=float, default=0.0)
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    os.environ["ULTRA_YUBIN_HOST"] = str(args.host)
    os.environ["ULTRA_YUBIN_PORT"] = str(args.port)
    os.environ["ULTRA_YUBIN_TIMEOUT_SEC"] = str(args.timeout_sec)
    os.environ["ULTRA_YUBIN_CENTER_ON_START"] = "1" if args.center_on_start else "0"

    clamp_deg = None if args.no_clamp else args.clamp_deg
    doa = ReSpeakerDOA(offset=args.offset)
    motor = None
    if not args.dry_run:
        motor = UltraYubinMotorController().start()
        if not motor.ready:
            print(f"[respeaker-bridge] Ultra96 not ready host={args.host} port={args.port}")
            doa.stop()
            return 2

    print(
        f"[respeaker-bridge] host={args.host}:{args.port} offset={args.offset} "
        f"front_doa={args.front_doa_deg} sign={args.sign} "
        f"period={args.period_sec}s min_change={args.min_change_deg}deg "
        f"clamp={'off' if clamp_deg is None else clamp_deg} dry_run={int(args.dry_run)}"
    )

    started = time.monotonic()
    last_sent_angle = None
    next_keepalive = 0.0

    try:
        while True:
            now = time.monotonic()
            raw_angle = doa.read()
            rel_angle = normalize_relative_angle(
                raw_angle,
                clamp_deg=clamp_deg,
                front_doa_deg=args.front_doa_deg,
                sign=args.sign,
            )
            send = (
                last_sent_angle is None
                or angle_delta(rel_angle, last_sent_angle) >= args.min_change_deg
                or now >= next_keepalive
                or args.once
            )

            if send:
                out_angle = int(round(rel_angle))
                if args.dry_run:
                    print(f"[respeaker-bridge] raw={raw_angle:3d} motor={out_angle:4d} dry-run")
                else:
                    telemetry = motor.turn_to_doa(out_angle)
                    print(
                        f"[respeaker-bridge] raw={raw_angle:3d} motor={out_angle:4d} "
                        f"pan={telemetry.get('pan')} tilt={telemetry.get('tilt')} "
                        f"usb={telemetry.get('usb_ok')} reply={telemetry.get('fpga_reply', '')}"
                    )
                last_sent_angle = rel_angle
                next_keepalive = now + max(0.1, args.keepalive_sec)

            if args.once:
                break
            if args.duration_sec > 0 and now - started >= args.duration_sec:
                break
            time.sleep(max(0.01, args.period_sec))
    except KeyboardInterrupt:
        print("[respeaker-bridge] Ctrl+C received; stopping")
    finally:
        doa.stop()
        if motor is not None:
            motor.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
