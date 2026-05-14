#!/usr/bin/env python3
"""Jetson YOLO/HUD node using the jh camera, TensorRT YOLO, and HUD UI.

The Jetson side is intentionally limited to camera capture, Tello/drone YOLO,
GUI rendering, and sending the selected bbox center to the Ultra96 bridge.
"""
import argparse
import json
import os
import sys
import time
from datetime import datetime

import cv2

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

import src.config as config
from src.audio_fallback import ReSpeakerDOA, TelloAudioFallback
from src.control.camera import CameraStream
from src.control.ultra_yubin_motor import UltraYubinMotorController
from src.core.decision import DecisionMaker
from src.core.state import SharedState, SystemState
from src.core.threat_analyzer import ThreatAnalyzer
from src.ui.display import AntiDroneDisplay
from src.vision.vision_tracker import VisionDetector


class PipelineLogger:
    def __init__(self, path=None, enabled=True, echo=False, echo_every=30):
        self.enabled = enabled
        self.echo = echo
        self.echo_every = max(1, int(echo_every))
        self.flush_every = max(1, int(os.getenv("PIPELINE_LOG_FLUSH_EVERY", "30")))
        self.path = path
        self.fp = None
        if self.enabled:
            if not self.path:
                root = os.path.dirname(SCRIPT_DIR)
                log_dir = os.path.join(root, "benchmark_logs")
                os.makedirs(log_dir, exist_ok=True)
                stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                self.path = os.path.join(log_dir, f"jetson_pipeline_{stamp}.jsonl")
            self.fp = open(self.path, "a", encoding="utf-8")
            print(f"[pipeline] log={self.path}")

    def write(self, frame_idx, event, source, target=None, telemetry=None, extra=None):
        telemetry = telemetry or {}
        record = {
            "ts": datetime.now().isoformat(timespec="milliseconds"),
            "frame": int(frame_idx),
            "event": event,
            "source": source,
            "target": target or {},
            "camera": extra.get("camera", {}) if extra else {},
            "ultra96": {
                "ready": bool(telemetry.get("ready", False)),
                "pan": telemetry.get("pan"),
                "tilt": telemetry.get("tilt"),
                "usb": telemetry.get("usb_ok"),
                "src": telemetry.get("src", ""),
                "reply": telemetry.get("fpga_reply", ""),
                "tx": telemetry.get("tx_cmd", ""),
                "rtt_ms": telemetry.get("motor_ms", 0.0),
                "aim_cx": telemetry.get("aim_cx"),
                "aim_cy": telemetry.get("aim_cy"),
                "send_cx": telemetry.get("send_cx"),
                "send_cy": telemetry.get("send_cy"),
            },
        }
        if extra:
            for key, value in extra.items():
                if key != "camera":
                    record[key] = value
        if self.fp:
            self.fp.write(json.dumps(record, ensure_ascii=False) + "\n")
            if frame_idx % self.flush_every == 0:
                self.fp.flush()
        if self.echo and frame_idx % self.echo_every == 0:
            tgt = record["target"]
            print(
                f"[PIPE] f={frame_idx} event={event} target={int(bool(tgt.get('detected', False)))} "
                f"cx={tgt.get('cx', '-')} cy={tgt.get('cy', '-')} "
                f"conf={tgt.get('conf', '-')} cam_center=({record['camera'].get('cx', '-')},{record['camera'].get('cy', '-')}) "
                f"sent=({record['ultra96'].get('send_cx', '-')},{record['ultra96'].get('send_cy', '-')}) "
                f"pan={record['ultra96']['pan']} "
                f"tilt={record['ultra96']['tilt']} usb={record['ultra96']['usb']} "
                f"src={record['ultra96'].get('src', '')} "
                f"reply='{str(record['ultra96']['reply'])[:130]}'"
            )

    def close(self):
        if self.fp:
            self.fp.close()
            self.fp = None


def parse_camera_arg(value):
    if value in (None, "auto", "AUTO"):
        return "auto"
    if isinstance(value, str) and value.isdigit():
        return int(value)
    return value


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--camera", type=parse_camera_arg, default=config.CAMERA_ID)
    parser.add_argument("--model", default=config.YOLO_MODEL_PATH)
    parser.add_argument("--device", default=config.YOLO_DEVICE)
    parser.add_argument("--conf", type=float, default=config.YOLO_CONF)
    parser.add_argument("--imgsz", type=int, default=config.YOLO_IMGSZ)
    parser.add_argument("--headless", "--no-display", action="store_true")
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument("--pipeline-log", default=None)
    parser.add_argument("--no-pipeline-log", action="store_true")
    parser.add_argument("--pipeline-echo", action="store_true")
    parser.add_argument("--pipeline-echo-every", type=int, default=30)
    parser.add_argument("--audio-fallback", action="store_true", default=config.TELLO_AUDIO_FALLBACK)
    parser.add_argument("--no-audio-fallback", action="store_false", dest="audio_fallback")
    return parser.parse_args()


def target_payload(target_box):
    if not target_box:
        return {"detected": False}
    x1, y1, x2, y2 = target_box["box"]
    return {
        "detected": True,
        "id": int(target_box.get("id", 0)),
        "conf": round(float(target_box.get("conf", 0.0)), 3),
        "x1": int(x1),
        "y1": int(y1),
        "x2": int(x2),
        "y2": int(y2),
        "cx": int((x1 + x2) / 2),
        "cy": int((y1 + y2) / 2),
        "w": int(x2 - x1),
        "h": int(y2 - y1),
        "held": bool(target_box.get("held", False)),
    }


def motor_skip_telemetry(base, reason, cx, cy, aim_cx, aim_cy, active=1):
    telemetry = dict(base or {})
    telemetry.update({
        "ready": bool(telemetry.get("ready", False)),
        "tx_cmd": "",
        "rx_reply": f"SKIP,{reason}",
        "reply_kind": "SKIP",
        "err_x": int(cx - aim_cx),
        "err_y": int(cy - aim_cy),
        "cmd_x": float(cx - aim_cx),
        "cmd_y": float(cy - aim_cy),
        "aim_cx": int(aim_cx),
        "aim_cy": int(aim_cy),
        "send_cx": int(cx),
        "send_cy": int(cy),
        "fpga_reply": f"SKIP,{reason}",
        "motor_ms": 0.0,
        "target_active": int(active),
        "usb_ok": 0,
    })
    return telemetry


def clamp_audio_angle(angle):
    limit = abs(float(config.TELLO_AUDIO_CLAMP_DEG))
    return max(-limit, min(limit, float(angle)))


def audio_section(angle):
    angle = float(angle)
    if -45.0 <= angle <= 45.0:
        return 1
    if angle > 45.0:
        return 2
    if angle < -45.0:
        return 4
    return 3


def audio_delta(a, b):
    return abs(((float(a) - float(b) + 180.0) % 360.0) - 180.0)


def normalize_audio_angle(angle):
    return ((float(angle) + 180.0) % 360.0) - 180.0


def main():
    args = parse_args()
    os.environ["YOLO_DEVICE"] = str(args.device)
    os.environ["YOLO_CONF"] = str(args.conf)
    os.environ["YOLO_IMGSZ"] = str(args.imgsz)

    state = SharedState()
    decision = DecisionMaker(state)
    motor = UltraYubinMotorController().start()
    display = None if args.headless else AntiDroneDisplay(state)
    pipeline = PipelineLogger(
        path=args.pipeline_log,
        enabled=not args.no_pipeline_log,
        echo=args.pipeline_echo,
        echo_every=args.pipeline_echo_every,
    )

    cam = CameraStream(args.camera).start()
    detector = VisionDetector(args.model)
    analyzer = ThreatAnalyzer(config.CAMERA_WIDTH, config.CAMERA_HEIGHT)
    audio = None
    audio_mode = str(config.TELLO_AUDIO_MODE).lower()
    if args.audio_fallback:
        try:
            if audio_mode == "model":
                audio = TelloAudioFallback(
                    config.TELLO_AUDIO_TFLITE,
                    config.TELLO_AUDIO_CONFIG,
                    alsa_device=config.TELLO_AUDIO_ALSA_DEVICE,
                    channels=config.TELLO_AUDIO_CHANNELS,
                    threshold=config.TELLO_AUDIO_THRESHOLD,
                    consecutive=config.TELLO_AUDIO_CONSECUTIVE,
                    min_rms=config.TELLO_AUDIO_MIN_RMS,
                    doa_offset=config.TELLO_AUDIO_DOA_OFFSET,
                ).start()
                audio_device = audio.alsa_device
            else:
                audio_mode = "doa"
                audio = ReSpeakerDOA(offset=config.TELLO_AUDIO_DOA_OFFSET)
                audio_device = "respeaker-usb-doa"
            state.update_audio_status("AUDIO READY", 0.0, 0.0, None, None, False)
            print(
                f"[audio] fallback=on mode={audio_mode} device={audio_device} "
                f"threshold={config.TELLO_AUDIO_THRESHOLD} "
                f"consecutive={config.TELLO_AUDIO_CONSECUTIVE} "
                f"offset={config.TELLO_AUDIO_DOA_OFFSET}"
            )
        except Exception as exc:
            audio = None
            state.update_audio_status(f"AUDIO ERROR: {exc}", 0.0, 0.0, None, None, False, add_log=True)
            print(f"[audio] fallback disabled: {exc}")

    print(
        f"[jetson] camera={args.camera} source={cam.active_source} "
        f"model={args.model} device={args.device} conf={args.conf} imgsz={args.imgsz} "
        f"size={config.CAMERA_WIDTH}x{config.CAMERA_HEIGHT}"
    )

    frame_idx = 0
    detect_count = 0
    no_target_count = 0
    started = time.perf_counter()
    last_telemetry = motor.last_telemetry
    threat_info = None
    last_audio_sent = 0.0
    last_audio_angle = None

    try:
        while True:
            frame = cam.read()
            frame_idx += 1
            frame_h, frame_w = frame.shape[:2]
            camera_center_x = frame_w // 2
            camera_center_y = frame_h // 2
            analyzer.set_frame_size(frame_w, frame_h)

            result = detector.track(frame)
            _visible_ids, target_visible, bboxes = decision.process_tracking(frame, result)
            state.update_vision_status(target_visible)
            audio_detection = None

            target = None
            for box in bboxes:
                if box.get("is_target"):
                    target = box
                    break

            if target:
                detect_count += 1
                x1, y1, x2, y2 = target["box"]
                cx = int((x1 + x2) / 2)
                cy = int((y1 + y2) / 2)
                bw = int(x2 - x1)
                bh = int(y2 - y1)
                threat_info = analyzer.update(target["box"])
                conf = float(target.get("conf", 0.0))
                held = bool(target.get("held", False))
                edge_x = int(config.TRACK_MOTOR_EDGE_MARGIN_X)
                edge_y = int(config.TRACK_MOTOR_EDGE_MARGIN_Y)
                aim_x = camera_center_x + int(getattr(motor, "aim_offset_x", 0))
                aim_y = camera_center_y + int(getattr(motor, "aim_offset_y", 0))
                near_edge = (
                    cx < edge_x or cx > frame_w - edge_x or
                    cy < edge_y or cy > frame_h - edge_y
                )
                if held:
                    last_telemetry = motor_skip_telemetry(
                        last_telemetry, "held", cx, cy, aim_x, aim_y
                    )
                elif conf < float(config.TRACK_MOTOR_MIN_CONF):
                    last_telemetry = motor_skip_telemetry(
                        last_telemetry, f"low_conf:{conf:.2f}", cx, cy, aim_x, aim_y
                    )
                elif near_edge:
                    last_telemetry = motor_skip_telemetry(
                        last_telemetry, "edge", cx, cy, aim_x, aim_y
                    )
                else:
                    last_telemetry = motor.control(
                        cx,
                        cy,
                        frame_w,
                        frame_h,
                        bw,
                        bh,
                        aim_center_x=camera_center_x,
                        aim_center_y=camera_center_y,
                    )
                event = "target_track"
            else:
                no_target_count += 1
                threat_info = None
                if audio and audio_mode == "model":
                    audio_detection = audio.get_detection(config.TELLO_AUDIO_MAX_AGE_SEC)
                elif audio and audio_mode == "doa":
                    raw_angle = audio.read()
                    audio_detection = {
                        "angle": normalize_audio_angle(raw_angle),
                        "raw_angle": int(raw_angle),
                        "score": 1.0,
                        "rms": 0.0,
                        "mode": "doa",
                    }
                else:
                    audio_detection = None
                if audio_detection:
                    audio_angle = clamp_audio_angle(audio_detection["angle"])
                    section = audio_section(audio_angle)
                    state.update_audio_status(
                        "AUDIO TELLO" if audio_mode == "model" else "AUDIO DOA",
                        audio_detection.get("score", 0.0),
                        audio_detection.get("rms", 0.0),
                        audio_angle,
                        section,
                        True,
                    )
                    now = time.perf_counter()
                    should_send_audio = (
                        now - last_audio_sent >= float(config.TELLO_AUDIO_CONTROL_PERIOD_SEC)
                        and (
                            last_audio_angle is None
                            or audio_delta(audio_angle, last_audio_angle) >= float(config.TELLO_AUDIO_MIN_CHANGE_DEG)
                        )
                    )
                    if should_send_audio:
                        last_telemetry = motor.turn_to_doa(audio_angle)
                        last_audio_sent = time.perf_counter()
                        last_audio_angle = audio_angle
                        event = "audio_fallback"
                    else:
                        last_telemetry = motor_skip_telemetry(
                            last_telemetry,
                            "audio_rate",
                            camera_center_x,
                            camera_center_y,
                            camera_center_x,
                            camera_center_y,
                            active=0,
                        )
                        event = "audio_hold"
                elif audio:
                    if frame_idx % 30 == 0:
                        state.update_audio_status("AUDIO LISTEN", 0.0, 0.0, None, None, False)
                        last_telemetry = motor.read_status()
                    event = "no_target"
                elif frame_idx % 30 == 0:
                    last_telemetry = motor.read_status()
                    event = "no_target"
                else:
                    event = "no_target"

            decision.update_engagement_state(target_visible, threat_info)
            if target_visible and state.system_state == SystemState.DETECTED:
                state.transition(SystemState.TRACKING)

            payload = target_payload(target)
            pipeline.write(
                frame_idx,
                event,
                "jetson_yolo",
                target=payload,
                telemetry=last_telemetry,
                extra={
                    "fps": detector._last_infer_ms,
                    "model": detector._model_name,
                    "skipped": detector._last_track_was_skipped,
                    "camera": {
                        "width": frame_w,
                        "height": frame_h,
                        "cx": camera_center_x,
                        "cy": camera_center_y,
                    },
                    "audio": audio_detection or {},
                },
            )

            if display:
                fps = 0.0
                elapsed = time.perf_counter() - started
                if elapsed > 0:
                    fps = frame_idx / elapsed
                display.draw(
                    frame,
                    bboxes,
                    int(fps),
                    threat_info,
                    decision,
                    fire_status=None,
                    motor_info=last_telemetry,
                )
                key = cv2.waitKeyEx(1) & 0xFFFFFFFF
                clicked = display.get_clicked_button()
                if key in (27, ord("q"), ord("Q")) or clicked == "RESET":
                    break

            if args.max_frames and frame_idx >= args.max_frames:
                break

    except KeyboardInterrupt:
        print("[jetson] Ctrl+C received; stopping")
    finally:
        elapsed = max(0.001, time.perf_counter() - started)
        avg_fps = frame_idx / elapsed
        print(
            f"[jetson] done frames={frame_idx} detections={detect_count} "
            f"no_target={no_target_count} avg_fps={avg_fps:.1f}"
        )
        pipeline.close()
        cam.stop()
        if audio:
            audio.stop()
        motor.stop()
        if display:
            try:
                cv2.destroyAllWindows()
            except cv2.error:
                pass


if __name__ == "__main__":
    main()
