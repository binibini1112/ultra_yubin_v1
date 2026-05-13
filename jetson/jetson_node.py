#!/usr/bin/env python3
"""Minimal Jetson node: camera YOLO bbox -> Ultra96 ultra_yubin UDP bridge."""

import argparse
import os
import sys
import time

import cv2

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from ultralytics import YOLO

from src import config
from src.audio_fallback import TelloAudioFallback
from src.control.ultra_yubin_motor import UltraYubinMotorController


def open_camera(index, width, height):
    cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap.release()
        cap = cv2.VideoCapture(index)
    if not cap.isOpened():
        raise RuntimeError(f"camera open failed: {index}")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return cap


def choose_largest_box(result):
    boxes = result.boxes
    if boxes is None or len(boxes) == 0:
        return None

    best = None
    best_area = -1.0
    for box in boxes:
        x1, y1, x2, y2 = [float(v) for v in box.xyxy[0].tolist()]
        area = max(0.0, x2 - x1) * max(0.0, y2 - y1)
        if area > best_area:
            best = (x1, y1, x2, y2, float(box.conf[0]))
            best_area = area
    return best


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--model", default=config.YOLO_MODEL)
    parser.add_argument("--host", default=config.ULTRA_YUBIN_HOST)
    parser.add_argument("--port", type=int, default=config.ULTRA_YUBIN_PORT)
    parser.add_argument("--width", type=int, default=config.CAMERA_WIDTH)
    parser.add_argument("--height", type=int, default=config.CAMERA_HEIGHT)
    parser.add_argument("--conf", type=float, default=config.YOLO_CONF)
    parser.add_argument("--imgsz", type=int, default=config.YOLO_IMGSZ)
    parser.add_argument("--device", default=config.YOLO_DEVICE)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument("--audio-fallback", action="store_true")
    parser.add_argument("--audio-threshold", type=float, default=config.TELLO_AUDIO_THRESHOLD)
    parser.add_argument("--audio-alsa-device", default=config.TELLO_AUDIO_ALSA_DEVICE)
    return parser.parse_args()


def main():
    args = parse_args()
    os.environ["ULTRA_YUBIN_HOST"] = args.host
    os.environ["ULTRA_YUBIN_PORT"] = str(args.port)

    motor = UltraYubinMotorController().start()
    model = YOLO(args.model, task="detect")
    cap = open_camera(args.camera, args.width, args.height)
    audio = None
    if args.audio_fallback:
        try:
            audio = TelloAudioFallback(
                model_path=config.TELLO_AUDIO_TFLITE,
                config_path=config.TELLO_AUDIO_CONFIG,
                alsa_device=args.audio_alsa_device,
                channels=config.TELLO_AUDIO_CHANNELS,
                threshold=args.audio_threshold,
                consecutive=config.TELLO_AUDIO_CONSECUTIVE,
                min_rms=config.TELLO_AUDIO_MIN_RMS,
                doa_offset=config.TELLO_AUDIO_DOA_OFFSET,
            ).start()
            print("[audio] fallback enabled")
        except Exception as exc:
            print(f"[audio] fallback disabled: {exc}")

    frame_count = 0
    prev = time.perf_counter()
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                raise RuntimeError("camera frame read failed")
            frame_count += 1

            result = model.predict(
                frame,
                verbose=False,
                conf=args.conf,
                imgsz=args.imgsz,
                device=args.device,
                classes=config.AERIAL_TARGET_CLASSES,
            )[0]
            box = choose_largest_box(result)
            if box is not None:
                x1, y1, x2, y2, conf = box
                cx = int((x1 + x2) / 2)
                cy = int((y1 + y2) / 2)
                telemetry = motor.control(cx, cy, frame.shape[1], frame.shape[0])
                if not args.headless:
                    cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 220, 255), 2)
                    cv2.circle(frame, (cx, cy), 4, (0, 220, 255), -1)
                    cv2.putText(frame, telemetry.get("fpga_reply", ""), (10, 24),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 220, 255), 1, cv2.LINE_AA)
            else:
                audio_detection = audio.get_detection() if audio is not None else None
                if audio_detection is not None:
                    angle = audio_detection["angle"]
                    telemetry = motor.turn_to_doa(angle)
                    if not args.headless:
                        cv2.putText(frame, f"AUDIO {angle:.0f} deg {audio_detection['score']:.2f}", (10, 24),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 180, 255), 2, cv2.LINE_AA)
                        cv2.putText(frame, telemetry.get("fpga_reply", ""), (10, 48),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 180, 255), 1, cv2.LINE_AA)
                elif not args.headless:
                    cv2.putText(frame, "NO TARGET", (10, 24),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (80, 80, 255), 2, cv2.LINE_AA)

            now = time.perf_counter()
            fps = 1.0 / max(1e-6, now - prev)
            prev = now
            if not args.headless:
                cv2.putText(frame, f"FPS {fps:.1f}", (10, frame.shape[0] - 12),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
                cv2.imshow("ultra_yubin Jetson Node", frame)
                if (cv2.waitKey(1) & 0xFF) == 27:
                    break

            if args.max_frames and frame_count >= args.max_frames:
                break
    finally:
        cap.release()
        if audio is not None:
            audio.stop()
        motor.stop()
        if not args.headless:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
