#!/usr/bin/env python3
"""Manual laser C-motor calibration against the camera center reticle.

This tool does not use YOLO. Put a white board at a known distance, run this
tool with that distance, nudge the laser Dynamixel until the laser dot is on the
camera center reticle, then press SPACE to save the sample.
"""

import argparse
import json
import os
import socket
import sys
import time

VENV_PYTHON = "/home/jetson/yubin/.venv/bin/python3"
if sys.executable != VENV_PYTHON and os.path.exists(VENV_PYTHON):
    os.execv(VENV_PYTHON, [VENV_PYTHON] + sys.argv)

import cv2


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_OUTPUT = os.path.join(ROOT, "models", "laser_tick_calibration.json")
JETSON_SRC = os.path.join(ROOT, "jetson")
if JETSON_SRC not in sys.path:
    sys.path.insert(0, JETSON_SRC)

try:
    import src.config as config
    from src.control.laser import LaserController
except Exception as exc:
    config = None
    LaserController = None
    _LASER_IMPORT_ERROR = exc
else:
    _LASER_IMPORT_ERROR = None


def request(host, port, cmd, timeout):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(timeout)
    try:
        sock.sendto((cmd.rstrip() + "\n").encode("ascii"), (host, port))
        data, _ = sock.recvfrom(2048)
        return data.decode("ascii", errors="replace").strip()
    finally:
        sock.close()


def parse_fields(reply):
    fields = {}
    for item in str(reply).split(","):
        if "=" not in item:
            continue
        key, value = item.split("=", 1)
        fields[key.strip()] = value.strip()
    return fields


def clamp_tick(value):
    return max(0, min(4095, int(round(value))))


def read_laser_tick(args):
    reply = request(args.host, args.port, f"DREL {args.laser_id} 0", args.timeout)
    fields = parse_fields(reply)
    if fields.get("read") == "0" or fields.get("usb") == "0":
        return clamp_tick(args.laser_center), reply
    tick = int(fields.get("goal", fields.get("present", args.laser_center)))
    return clamp_tick(tick), reply


def move_laser(args, current_tick, delta):
    target = clamp_tick(current_tick + int(delta))
    reply = request(args.host, args.port, f"D {args.laser_id} {target}", args.timeout)
    fields = parse_fields(reply)
    if fields.get("usb") == "0" or fields.get("config") == "0":
        return clamp_tick(current_tick), reply
    tick = int(fields.get("goal", target))
    return clamp_tick(tick), reply


def load_output(path):
    if not os.path.exists(path):
        return {"type": "distance_to_laser_tick", "samples": []}
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    data.setdefault("type", "distance_to_laser_tick")
    data.setdefault("samples", [])
    return data


def save_sample(args, tick):
    data = load_output(args.output)
    sample = {
        "distance_m": float(args.distance_m),
        "distance_mm": int(round(float(args.distance_m) * 1000.0)),
        "laser_id": int(args.laser_id),
        "laser_tilt_tick": int(tick),
        "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
    }
    data["samples"].append(sample)
    data["samples"].sort(key=lambda item: (float(item.get("distance_m", 0.0)), int(item.get("laser_tilt_tick", 0))))
    os.makedirs(os.path.dirname(args.output), exist_ok=True)
    tmp = args.output + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)
        f.write("\n")
    os.replace(tmp, args.output)
    return sample


def open_camera(args):
    if args.camera == "auto":
        candidates = [
            "/dev/v4l/by-path/platform-3610000.usb-usb-0:2.2:1.0-video-index0",
            "/dev/video0",
            "/dev/video1",
        ]
    else:
        candidates = [args.camera]
    for source in candidates:
        cap = cv2.VideoCapture(source, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap.release()
            continue
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
        cap.set(cv2.CAP_PROP_FPS, args.fps)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        ok, frame = cap.read()
        if ok and frame is not None:
            print(f"[laser-cal] camera={source} size={frame.shape[1]}x{frame.shape[0]}")
            return cap, source
        cap.release()
    raise RuntimeError("camera open failed")


def center_pan_tilt(args):
    reply = request(args.host, args.port, "CENTER", args.timeout)
    return reply


def start_laser(args):
    if args.no_laser_on:
        print("[laser-cal] laser GPIO auto-on disabled")
        return None
    if LaserController is None or config is None:
        print(f"[laser-cal] laser GPIO unavailable: {_LASER_IMPORT_ERROR}")
        return None
    laser = LaserController(
        pin=args.laser_pin,
        enabled=True,
        pin_mode=args.laser_pin_mode,
        active_high=args.laser_active_high,
    )
    laser.set_active(True, reason="calibration")
    return laser


def parse_pan_tilt(reply, default_pan, default_tilt):
    fields = parse_fields(reply)
    pan = int(fields.get("pan", default_pan))
    tilt = int(fields.get("tilt", default_tilt))
    return clamp_tick(pan), clamp_tick(tilt)


def move_tilt(args, fixed_pan, current_tilt, delta):
    target_tilt = clamp_tick(current_tilt + int(delta))
    reply = request(args.host, args.port, f"G {fixed_pan} {target_tilt}", args.timeout)
    pan, tilt = parse_pan_tilt(reply, fixed_pan, target_tilt)
    return pan, tilt, reply


def hold_pan_tilt(args, fixed_pan, fixed_tilt):
    reply = request(args.host, args.port, f"G {fixed_pan} {fixed_tilt}", args.timeout)
    pan, tilt = parse_pan_tilt(reply, fixed_pan, fixed_tilt)
    return pan, tilt, reply


def draw_overlay(frame, args, laser_tick, laser_step, tilt_tick, tilt_step, fixed_pan, last_reply, saved_count):
    h, w = frame.shape[:2]
    cx = w // 2
    cy = h // 2
    green = (80, 230, 90)
    amber = (40, 190, 255)
    white = (245, 245, 245)
    black = (0, 0, 0)
    muted = (150, 155, 160)

    cv2.line(frame, (cx - 70, cy), (cx - 14, cy), green, 2, cv2.LINE_AA)
    cv2.line(frame, (cx + 14, cy), (cx + 70, cy), green, 2, cv2.LINE_AA)
    cv2.line(frame, (cx, cy - 70), (cx, cy - 14), green, 2, cv2.LINE_AA)
    cv2.line(frame, (cx, cy + 14), (cx, cy + 70), green, 2, cv2.LINE_AA)
    cv2.circle(frame, (cx, cy), 4, green, -1, cv2.LINE_AA)
    cv2.circle(frame, (cx, cy), 34, green, 1, cv2.LINE_AA)

    cv2.rectangle(frame, (0, 0), (w, 82), black, -1)
    cv2.putText(frame, f"LASER CAL  distance={args.distance_m:.2f}m  pan_hold={fixed_pan}  tilt={tilt_tick}  C{args.laser_id}={laser_tick}",
                (14, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.62, white, 1, cv2.LINE_AA)
    tilt_mode = f"tilt w/s W/S step={tilt_step}" if args.allow_tilt_adjust else "tilt LOCKED"
    cv2.putText(frame, f"C j/k J/K step={laser_step} | {tilt_mode} | SPACE save | r read | q quit saved={saved_count}",
                (14, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.50, amber, 1, cv2.LINE_AA)
    if last_reply:
        cv2.putText(frame, last_reply[:130], (14, h - 16), cv2.FONT_HERSHEY_SIMPLEX, 0.42, muted, 1, cv2.LINE_AA)
    return frame


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--distance-m", type=float, required=True, help="Known board distance in meters")
    parser.add_argument("--laser-id", type=int, default=3)
    parser.add_argument("--laser-center", type=int, default=2048)
    parser.add_argument("--host", default="192.168.3.1")
    parser.add_argument("--port", type=int, default=5016)
    parser.add_argument("--timeout", type=float, default=2.0)
    parser.add_argument("--camera", default="auto")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--step", type=int, default=32, help="Laser C-motor fine step")
    parser.add_argument("--big-step", type=int, default=160, help="Laser C-motor big step")
    parser.add_argument("--tilt-step", type=int, default=12)
    parser.add_argument("--tilt-big-step", type=int, default=60)
    parser.add_argument("--front-pan", type=int, default=int(os.getenv("LASER_CAL_PAN_TICK", "2048")))
    parser.add_argument("--front-tilt", type=int, default=int(os.getenv("LASER_CAL_TILT_TICK", "2952")))
    parser.add_argument("--allow-tilt-adjust", action="store_true", help="Allow w/s keys to move tilt during calibration")
    parser.add_argument("--laser-pin", type=int, default=int(os.getenv("LASER_PIN", "7")))
    parser.add_argument("--laser-pin-mode", default=os.getenv("LASER_PIN_MODE", "BOARD"))
    parser.add_argument("--laser-active-high", action=argparse.BooleanOptionalAction, default=os.getenv("LASER_ACTIVE_HIGH", "1") == "1")
    parser.add_argument("--no-laser-on", action="store_true", help="Do not force the visible laser GPIO on")
    parser.add_argument("--keep-laser-on-exit", action="store_true", help="Leave the visible laser GPIO on when quitting")
    parser.add_argument("--output", default=DEFAULT_OUTPUT)
    parser.add_argument("--no-center-pan-tilt", action="store_true")
    args = parser.parse_args()

    laser = start_laser(args)
    cap, _source = open_camera(args)
    fixed_pan = clamp_tick(args.front_pan)
    tilt_tick = clamp_tick(args.front_tilt)
    if not args.no_center_pan_tilt:
        center_reply = center_pan_tilt(args)
        print(f"[laser-cal] CENTER => {center_reply}")
        fixed_pan = clamp_tick(args.front_pan)
        tilt_tick = clamp_tick(args.front_tilt)
        fixed_pan, tilt_tick, hold_reply = hold_pan_tilt(args, fixed_pan, tilt_tick)
        print(f"[laser-cal] HOLD => {hold_reply}")
    laser_tick, last_reply = read_laser_tick(args)
    laser_step = max(1, int(args.step))
    tilt_step = max(1, int(args.tilt_step))
    saved_count = 0
    window = "ULTRA YUBIN V1 LASER CAL"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    print(f"[laser-cal] output={args.output}")
    print("[laser-cal] pan is held fixed. keys: j/k C | J/K C big | w/s tilt | W/S tilt big | [/] C step | ,/. tilt step | SPACE save | r read | q quit")

    try:
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                print("[laser-cal] camera frame failed")
                break
            draw_overlay(frame, args, laser_tick, laser_step, tilt_tick, tilt_step, fixed_pan, last_reply, saved_count)
            cv2.imshow(window, frame)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("["):
                laser_step = max(1, laser_step // 2)
            elif key == ord("]"):
                laser_step = min(512, laser_step * 2)
            elif key == ord(","):
                tilt_step = max(1, tilt_step // 2)
            elif key == ord("."):
                tilt_step = min(256, tilt_step * 2)
            elif key == ord("r"):
                laser_tick, last_reply = read_laser_tick(args)
                print(f"READ C => tick={laser_tick} reply={last_reply}")
            elif key == ord("j"):
                laser_tick, last_reply = move_laser(args, laser_tick, laser_step)
                print(f"C +{laser_step} => tick={laser_tick} reply={last_reply}")
            elif key == ord("k"):
                laser_tick, last_reply = move_laser(args, laser_tick, -laser_step)
                print(f"C -{laser_step} => tick={laser_tick} reply={last_reply}")
            elif key == ord("J"):
                laser_tick, last_reply = move_laser(args, laser_tick, args.big_step)
                print(f"C +{args.big_step} => tick={laser_tick} reply={last_reply}")
            elif key == ord("K"):
                laser_tick, last_reply = move_laser(args, laser_tick, -args.big_step)
                print(f"C -{args.big_step} => tick={laser_tick} reply={last_reply}")
            elif key == ord("w") and args.allow_tilt_adjust:
                fixed_pan, tilt_tick, last_reply = move_tilt(args, fixed_pan, tilt_tick, -tilt_step)
                print(f"TILT -{tilt_step} => pan={fixed_pan} tilt={tilt_tick} reply={last_reply}")
            elif key == ord("s") and args.allow_tilt_adjust:
                fixed_pan, tilt_tick, last_reply = move_tilt(args, fixed_pan, tilt_tick, tilt_step)
                print(f"TILT +{tilt_step} => pan={fixed_pan} tilt={tilt_tick} reply={last_reply}")
            elif key == ord("W") and args.allow_tilt_adjust:
                fixed_pan, tilt_tick, last_reply = move_tilt(args, fixed_pan, tilt_tick, -args.tilt_big_step)
                print(f"TILT -{args.tilt_big_step} => pan={fixed_pan} tilt={tilt_tick} reply={last_reply}")
            elif key == ord("S") and args.allow_tilt_adjust:
                fixed_pan, tilt_tick, last_reply = move_tilt(args, fixed_pan, tilt_tick, args.tilt_big_step)
                print(f"TILT +{args.tilt_big_step} => pan={fixed_pan} tilt={tilt_tick} reply={last_reply}")
            elif key == ord(" "):
                sample = save_sample(args, laser_tick)
                saved_count += 1
                print(f"SAVED => {sample}")
    finally:
        if laser is not None and not args.keep_laser_on_exit:
            laser.set_active(False, reason="calibration-exit")
            laser.cleanup()
        cap.release()
        cv2.destroyWindow(window)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[laser-cal] stopped")
    except Exception as exc:
        print(f"[laser-cal] ERROR: {exc}", file=sys.stderr)
        raise
