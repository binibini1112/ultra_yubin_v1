#!/usr/bin/env python3
"""Collect bbox-height to distance calibration samples for laser C-motor aiming."""

import argparse
import json
import os
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
from src.control.ultra_yubin_motor import UltraYubinMotorController
from src.vision.vision_tracker import VisionDetector


DEFAULT_OUTPUT = os.path.join(ROOT, "models", "laser_distance_calibration.json")


def load_output(path):
    if not os.path.exists(path):
        return {"type": "bbox_height_to_distance", "samples": []}
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    data.setdefault("type", "bbox_height_to_distance")
    data.setdefault("samples", [])
    return data


def save_sample(args, recent):
    if not recent:
        raise RuntimeError("no bbox samples to save")
    rows = list(recent)
    bbox_h = float(np.mean([r["bbox_h"] for r in rows]))
    bbox_w = float(np.mean([r["bbox_w"] for r in rows]))
    area = float(np.mean([r["area"] for r in rows]))
    conf = float(np.mean([r["conf"] for r in rows]))

    sample = {
        "distance_m": float(args.distance_m),
        "distance_mm": int(round(float(args.distance_m) * 1000.0)),
        "bbox_h": round(bbox_h, 2),
        "bbox_w": round(bbox_w, 2),
        "bbox_area": round(area, 2),
        "confidence": round(conf, 4),
        "sample_count": len(rows),
        "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
    }
    data = load_output(args.output)
    data["samples"].append(sample)
    data["samples"].sort(key=lambda item: (float(item.get("distance_mm", 0)), -float(item.get("bbox_h", 0))))
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


def drive_motor(motor, det, frame_shape, args):
    if motor is None or not det or det["conf"] < args.conf:
        return None
    frame_h, frame_w = frame_shape[:2]
    x1, y1, x2, y2 = det["box"]
    cx = int(round((x1 + x2) / 2.0))
    cy = int(round((y1 + y2) / 2.0))
    distance_mm = int(round(float(args.distance_m) * 1000.0))
    return motor.control(
        cx,
        cy,
        frame_w,
        frame_h,
        bbox_width=det["bbox_w"],
        bbox_height=det["bbox_h"],
        distance_mm=distance_mm,
    )


def draw(frame, args, det, recent, saved_count, last_saved, motor_status=None):
    h, w = frame.shape[:2]
    cx, cy = w // 2, h // 2
    green = (80, 230, 90)
    amber = (40, 190, 255)
    white = (245, 245, 245)
    red = (60, 70, 255)
    muted = (150, 155, 160)
    black = (0, 0, 0)

    cv2.rectangle(frame, (0, 0), (w, 88), black, -1)
    cv2.line(frame, (cx - 42, cy), (cx + 42, cy), green, 1, cv2.LINE_AA)
    cv2.line(frame, (cx, cy - 42), (cx, cy + 42), green, 1, cv2.LINE_AA)
    cv2.circle(frame, (cx, cy), 4, green, -1, cv2.LINE_AA)

    cv2.putText(frame, f"BBOX DIST CAL distance={args.distance_m:.2f}m recent={len(recent)}/{args.window}",
                (14, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.62, white, 1, cv2.LINE_AA)
    cv2.putText(frame, "SPACE save recent average | c clear | q quit",
                (14, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.52, amber, 1, cv2.LINE_AA)

    if det:
        x1, y1, x2, y2 = det["box"]
        cv2.rectangle(frame, (x1, y1), (x2, y2), green, 2, cv2.LINE_AA)
        cv2.circle(frame, ((x1 + x2) // 2, (y1 + y2) // 2), 4, green, -1, cv2.LINE_AA)
        cv2.putText(frame, f"h={det['bbox_h']:.1f} w={det['bbox_w']:.1f} conf={det['conf']:.2f}",
                    (x1, max(18, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.50, green, 1, cv2.LINE_AA)
    else:
        cv2.putText(frame, "NO DRONE BBOX", (14, 86), cv2.FONT_HERSHEY_SIMPLEX, 0.58, red, 1, cv2.LINE_AA)

    if recent:
        avg_h = float(np.mean([r["bbox_h"] for r in recent]))
        avg_w = float(np.mean([r["bbox_w"] for r in recent]))
        avg_conf = float(np.mean([r["conf"] for r in recent]))
        cv2.putText(frame, f"avg h={avg_h:.1f} w={avg_w:.1f} conf={avg_conf:.2f} saved={saved_count}",
                    (14, h - 42), cv2.FONT_HERSHEY_SIMPLEX, 0.52, white, 1, cv2.LINE_AA)
    if motor_status:
        reply = str(motor_status.get("reply_kind", motor_status.get("fpga_reply", "")))
        pan = motor_status.get("pan", "-")
        tilt = motor_status.get("tilt", "-")
        src = motor_status.get("src", "")
        cv2.putText(frame, f"drive={reply} src={src} pan={pan} tilt={tilt}",
                    (w - 430, h - 16), cv2.FONT_HERSHEY_SIMPLEX, 0.42, amber, 1, cv2.LINE_AA)
    if last_saved:
        cv2.putText(frame, f"saved: {last_saved}",
                    (14, h - 16), cv2.FONT_HERSHEY_SIMPLEX, 0.42, muted, 1, cv2.LINE_AA)
    return frame


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--distance-m", type=float, required=True)
    parser.add_argument("--output", default=DEFAULT_OUTPUT)
    parser.add_argument("--camera", default="auto")
    parser.add_argument("--model", default=os.path.join(ROOT, "models", "drone_best_final_0520.engine"))
    parser.add_argument("--window", type=int, default=30)
    parser.add_argument("--conf", type=float, default=float(os.getenv("YOLO_CONF", "0.60")))
    parser.add_argument("--imgsz", type=int, default=int(os.getenv("YOLO_IMGSZ", "640")))
    parser.add_argument("--drive", action="store_true", help="drive Ultra96 PL pan/tilt while collecting bbox samples")
    parser.add_argument("--drive-echo-every", type=int, default=30)
    args = parser.parse_args()

    os.environ["YOLO_CONF"] = str(args.conf)
    os.environ["YOLO_IMGSZ"] = str(args.imgsz)
    os.environ.setdefault("YOLO_SKIP_FRAMES", "1")
    os.environ.setdefault("YOLO_FAST_DETECT", "1")
    os.environ.setdefault("CAMERA_APPLY_GLARE_DEFAULTS", "0")

    cam = CameraStream(args.camera).start()
    detector = VisionDetector(args.model)
    motor = UltraYubinMotorController().start() if args.drive else None
    recent = deque(maxlen=max(1, int(args.window)))
    saved_count = 0
    last_saved = ""
    last_motor_status = None
    frame_idx = 0
    window_name = "ULTRA YUBIN V1 BBOX DIST CAL"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    print(f"[bbox-cal] distance={args.distance_m:.2f}m output={args.output} drive={int(args.drive)}")
    print("[bbox-cal] Put drone at this distance. SPACE save | c clear | q quit")

    try:
        while True:
            frame = cam.read()
            if frame is None:
                time.sleep(0.01)
                continue
            frame_idx += 1
            result = detector.track(frame, persist=False)
            det = best_detection(result)
            if det and det["conf"] >= args.conf:
                recent.append(det)
                last_motor_status = drive_motor(motor, det, frame.shape, args)
                if (
                    last_motor_status
                    and args.drive_echo_every > 0
                    and frame_idx % args.drive_echo_every == 0
                ):
                    print(
                        "[bbox-cal-drive] "
                        f"f={frame_idx} cx={(det['box'][0] + det['box'][2]) // 2} "
                        f"cy={(det['box'][1] + det['box'][3]) // 2} "
                        f"h={det['bbox_h']:.1f} conf={det['conf']:.2f} "
                        f"reply='{last_motor_status.get('fpga_reply', '')}'"
                    )
            draw(frame, args, det, recent, saved_count, last_saved, last_motor_status)
            cv2.imshow(window_name, frame)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("c"):
                recent.clear()
                last_saved = "cleared"
                print("[bbox-cal] cleared recent samples")
            elif key == ord(" "):
                if recent:
                    sample = save_sample(args, recent)
                    saved_count += 1
                    last_saved = f"{sample['distance_mm']}mm h={sample['bbox_h']} w={sample['bbox_w']} n={sample['sample_count']}"
                    print(f"SAVED => {sample}")
                else:
                    print("[bbox-cal] no recent bbox samples; not saved")
    finally:
        if motor is not None:
            motor.stop()
        cam.stop()
        cv2.destroyWindow(window_name)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[bbox-cal] stopped")
