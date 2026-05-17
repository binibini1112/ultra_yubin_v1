#!/usr/bin/env python3
"""Collect bbox-height to laser C-motor tick samples using the real drone."""

import argparse
import json
import os
import socket
import sys
import time
from collections import deque

VENV_PYTHON = "/home/jetson/yubin/.venv/bin/python3"
if sys.executable != VENV_PYTHON and os.path.exists(VENV_PYTHON):
    os.execv(VENV_PYTHON, [VENV_PYTHON] + sys.argv)

import cv2
import numpy as np


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
JETSON_SRC = os.path.join(ROOT, "jetson")
if JETSON_SRC not in sys.path:
    sys.path.insert(0, JETSON_SRC)

import src.config as config
from src.control.camera import CameraStream
from src.control.laser import LaserController
from src.vision.vision_tracker import VisionDetector


DEFAULT_OUTPUT = os.path.join(ROOT, "models", "laser_bbox_tick_calibration.json")


def request(host, port, text, timeout):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(timeout)
    try:
        sock.sendto((text.strip() + "\n").encode("ascii"), (host, port))
        data, _ = sock.recvfrom(2048)
        return data.decode("utf-8", errors="replace").strip()
    finally:
        sock.close()


def parse_fields(reply):
    fields = {}
    for item in reply.strip().split(","):
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
        return {"type": "bbox_height_to_laser_tick", "samples": []}
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    data.setdefault("type", "bbox_height_to_laser_tick")
    data.setdefault("samples", [])
    return data


def save_sample(args, recent, laser_tick):
    rows = list(recent)
    bbox_h = float(np.mean([r["bbox_h"] for r in rows]))
    bbox_w = float(np.mean([r["bbox_w"] for r in rows]))
    area = float(np.mean([r["area"] for r in rows]))
    conf = float(np.mean([r["conf"] for r in rows]))
    sample = {
        "bbox_h": round(bbox_h, 2),
        "bbox_w": round(bbox_w, 2),
        "bbox_area": round(area, 2),
        "confidence": round(conf, 4),
        "laser_id": int(args.laser_id),
        "laser_tilt_tick": int(laser_tick),
        "sample_count": len(rows),
        "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
    }
    data = load_output(args.output)
    data["samples"].append(sample)
    data["samples"].sort(key=lambda item: float(item.get("bbox_h", 0.0)))
    os.makedirs(os.path.dirname(args.output), exist_ok=True)
    tmp = args.output + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)
        f.write("\n")
    os.replace(tmp, args.output)
    return sample


def best_detection(result):
    if result is None or result.boxes is None or len(result.boxes) == 0:
        return None
    boxes = result.boxes.xyxy.cpu().numpy()
    confs = result.boxes.conf.cpu().numpy() if result.boxes.conf is not None else np.ones(len(boxes))
    best = None
    best_score = -1.0
    for box, conf in zip(boxes, confs):
        x1, y1, x2, y2 = [float(v) for v in box]
        w = max(0.0, x2 - x1)
        h = max(0.0, y2 - y1)
        area = w * h
        score = float(conf) * max(1.0, area)
        if score > best_score:
            best_score = score
            best = {
                "box": (int(round(x1)), int(round(y1)), int(round(x2)), int(round(y2))),
                "bbox_w": w,
                "bbox_h": h,
                "area": area,
                "conf": float(conf),
            }
    return best


def draw(frame, args, det, recent, laser_tick, step, saved_count, last_saved):
    h, w = frame.shape[:2]
    cx, cy = w // 2, h // 2
    green = (80, 230, 90)
    amber = (40, 190, 255)
    white = (245, 245, 245)
    red = (60, 70, 255)
    muted = (150, 155, 160)
    black = (0, 0, 0)

    cv2.rectangle(frame, (0, 0), (w, 92), black, -1)
    cv2.line(frame, (cx - 42, cy), (cx + 42, cy), green, 1, cv2.LINE_AA)
    cv2.line(frame, (cx, cy - 42), (cx, cy + 42), green, 1, cv2.LINE_AA)
    cv2.circle(frame, (cx, cy), 4, green, -1, cv2.LINE_AA)
    cv2.putText(frame, f"DRONE LASER CAL  C{args.laser_id}={laser_tick} step={step} recent={len(recent)}/{args.window}",
                (14, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.60, white, 1, cv2.LINE_AA)
    cv2.putText(frame, "j/k C | J/K big | [/] step | SPACE save when pointer hits bbox center | c clear | q quit",
                (14, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.48, amber, 1, cv2.LINE_AA)

    if det:
        x1, y1, x2, y2 = det["box"]
        dcx, dcy = (x1 + x2) // 2, (y1 + y2) // 2
        cv2.rectangle(frame, (x1, y1), (x2, y2), green, 2, cv2.LINE_AA)
        cv2.line(frame, (dcx - 18, dcy), (dcx + 18, dcy), green, 1, cv2.LINE_AA)
        cv2.line(frame, (dcx, dcy - 18), (dcx, dcy + 18), green, 1, cv2.LINE_AA)
        cv2.putText(frame, f"h={det['bbox_h']:.1f} w={det['bbox_w']:.1f} conf={det['conf']:.2f}",
                    (x1, max(18, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.50, green, 1, cv2.LINE_AA)
    else:
        cv2.putText(frame, "NO DRONE BBOX", (14, 88), cv2.FONT_HERSHEY_SIMPLEX, 0.58, red, 1, cv2.LINE_AA)

    if recent:
        avg_h = float(np.mean([r["bbox_h"] for r in recent]))
        avg_w = float(np.mean([r["bbox_w"] for r in recent]))
        cv2.putText(frame, f"avg h={avg_h:.1f} w={avg_w:.1f} saved={saved_count}",
                    (14, h - 42), cv2.FONT_HERSHEY_SIMPLEX, 0.52, white, 1, cv2.LINE_AA)
    if last_saved:
        cv2.putText(frame, f"saved: {last_saved}",
                    (14, h - 16), cv2.FONT_HERSHEY_SIMPLEX, 0.42, muted, 1, cv2.LINE_AA)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", default=DEFAULT_OUTPUT)
    parser.add_argument("--camera", default="auto")
    parser.add_argument("--model", default=os.path.join(ROOT, "models", "drone_yolov8s_fromjunmo.engine"))
    parser.add_argument("--window", type=int, default=20)
    parser.add_argument("--conf", type=float, default=float(os.getenv("YOLO_CONF", "0.60")))
    parser.add_argument("--imgsz", type=int, default=int(os.getenv("YOLO_IMGSZ", "640")))
    parser.add_argument("--host", default=os.getenv("ULTRA_CHAN_HOST", os.getenv("ULTRA_YUBIN_HOST", "192.168.3.1")))
    parser.add_argument("--port", type=int, default=int(os.getenv("ULTRA_CHAN_PORT", os.getenv("ULTRA_YUBIN_PORT", "5016"))))
    parser.add_argument("--timeout", type=float, default=0.25)
    parser.add_argument("--laser-id", type=int, default=int(os.getenv("ULTRA_CHAN_LASER_ID", "3")))
    parser.add_argument("--laser-center", type=int, default=int(os.getenv("ULTRA_CHAN_LASER_CENTER", "2048")))
    parser.add_argument("--step", type=int, default=4)
    parser.add_argument("--big-step", type=int, default=40)
    args = parser.parse_args()

    os.environ["YOLO_CONF"] = str(args.conf)
    os.environ["YOLO_IMGSZ"] = str(args.imgsz)
    os.environ.setdefault("YOLO_SKIP_FRAMES", "1")
    os.environ.setdefault("YOLO_FAST_DETECT", "1")
    os.environ.setdefault("CAMERA_APPLY_GLARE_DEFAULTS", "0")

    laser = LaserController(
        pin=config.LASER_PIN,
        enabled=config.LASER_ENABLED,
        pin_mode=config.LASER_PIN_MODE,
        active_high=config.LASER_ACTIVE_HIGH,
    )
    laser.set_active(True, "drone-calibration")
    cam = CameraStream(args.camera).start()
    detector = VisionDetector(args.model)
    recent = deque(maxlen=max(1, int(args.window)))
    laser_tick, last_reply = read_laser_tick(args)
    step = max(1, int(args.step))
    saved_count = 0
    last_saved = ""
    window_name = "ULTRA YUBIN V1 DRONE LASER CAL"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    print(f"[laser-drone-cal] output={args.output}")
    print("[laser-drone-cal] Put laser pointer on bbox center, then SPACE save.")

    try:
        while True:
            frame = cam.read()
            if frame is None:
                time.sleep(0.01)
                continue
            result = detector.track(frame, persist=False)
            det = best_detection(result)
            if det and det["conf"] >= args.conf:
                recent.append(det)
            draw(frame, args, det, recent, laser_tick, step, saved_count, last_saved)
            cv2.imshow(window_name, frame)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("c"):
                recent.clear()
                last_saved = "cleared"
                print("[laser-drone-cal] cleared recent samples")
            elif key == ord("["):
                step = max(1, step // 2)
                print(f"[laser-drone-cal] step={step}")
            elif key == ord("]"):
                step = min(512, step * 2)
                print(f"[laser-drone-cal] step={step}")
            elif key == ord("j"):
                laser_tick, last_reply = move_laser(args, laser_tick, -step)
                print(f"C -{step} => tick={laser_tick} reply={last_reply}")
            elif key == ord("k"):
                laser_tick, last_reply = move_laser(args, laser_tick, step)
                print(f"C +{step} => tick={laser_tick} reply={last_reply}")
            elif key == ord("J"):
                laser_tick, last_reply = move_laser(args, laser_tick, -args.big_step)
                print(f"C -{args.big_step} => tick={laser_tick} reply={last_reply}")
            elif key == ord("K"):
                laser_tick, last_reply = move_laser(args, laser_tick, args.big_step)
                print(f"C +{args.big_step} => tick={laser_tick} reply={last_reply}")
            elif key == ord(" "):
                if recent:
                    sample = save_sample(args, recent, laser_tick)
                    saved_count += 1
                    last_saved = f"h={sample['bbox_h']} C={sample['laser_tilt_tick']} n={sample['sample_count']}"
                    print(f"SAVED => {sample}")
                else:
                    print("[laser-drone-cal] no recent bbox samples; not saved")
    finally:
        laser.set_active(False, "drone-calibration-exit")
        cam.stop()
        cv2.destroyWindow(window_name)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[laser-drone-cal] stopped")
