#!/usr/bin/env python3
"""Jetson YOLO/HUD node using the jh camera, TensorRT YOLO, and HUD UI.

The Jetson side is intentionally limited to camera capture, Tello/drone YOLO,
GUI rendering, and sending the selected bbox center to the Ultra96 bridge.
"""
import argparse
import json
import math
import os
import sys
import time
from collections import deque
from datetime import datetime

import cv2

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

import src.config as config
from src.audio_fallback import JunmoDroneAudioFallback, ReSpeakerDOA, TelloAudioFallback
from src.control.camera import CameraStream
from src.control.laser import LaserController
from src.control.ultra_yubin_motor import UltraYubinMotorController
from src.core.decision import DecisionMaker
from src.core.state import SharedState, SystemState
from src.distance_model import DistanceEstimator, LaserTickEstimator
from src.core.threat_analyzer import ThreatAnalyzer
from src.ui.display import AntiDroneDisplay
from src.utils.jetson_sender import JetsonTelemetrySender
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
                "distance_mm": telemetry.get("distance_mm"),
                "laser_center_tick": telemetry.get("laser_center_lock_tick"),
                "laser_range_offset_tick": telemetry.get("laser_range_offset_tick"),
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
            audio = record.get("audio") or {}
            audio_text = ""
            if audio:
                audio_text = (
                    f" audio_angle={audio.get('angle', '-')}"
                    f" raw={audio.get('raw_angle', '-')}"
                    f" signed={audio.get('signed_angle', '-')}"
                    f" send={audio.get('send_angle', '-')}"
                    f" stable={audio.get('stable_reason', '-')}"
                )
            print(
                f"[PIPE] f={frame_idx} event={event} target={int(bool(tgt.get('detected', False)))} "
                f"cx={tgt.get('cx', '-')} cy={tgt.get('cy', '-')} "
                f"conf={tgt.get('conf', '-')} cam_center=({record['camera'].get('cx', '-')},{record['camera'].get('cy', '-')}) "
                f"sent=({record['ultra96'].get('send_cx', '-')},{record['ultra96'].get('send_cy', '-')}) "
                f"pan={record['ultra96']['pan']} "
                f"tilt={record['ultra96']['tilt']} usb={record['ultra96']['usb']} "
                f"src={record['ultra96'].get('src', '')} "
                f"dist_m={fmt_distance_m(record['ultra96'].get('distance_mm'))} "
                f"laser_tick={record['ultra96'].get('laser_center_tick', '-')} "
                f"laser_off={record['ultra96'].get('laser_range_offset_tick', '-')} "
                f"{audio_text} "
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
    parser.add_argument("--skip-frames", type=int, default=config.YOLO_SKIP_FRAMES,
                        help="Run YOLO once every N frames; reused boxes fill skipped frames")
    parser.add_argument("--fast-detect", action="store_true", default=config.YOLO_FAST_DETECT,
                        help="Use YOLO predict() instead of tracker mode for lower overhead")
    parser.add_argument("--headless", "--no-display", action="store_true")
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument("--pipeline-log", default=None)
    parser.add_argument("--no-pipeline-log", action="store_true")
    parser.add_argument("--pipeline-echo", action="store_true")
    parser.add_argument("--pipeline-echo-every", type=int, default=30)
    parser.add_argument("--async-motor", action="store_true", default=config.ULTRA_CHAN_ASYNC_SEND,
                        help="Send target commands without waiting for Ultra96 USB ACK")
    parser.add_argument("--no-center-on-start", action="store_true",
                        default=not config.ULTRA_CHAN_CENTER_ON_START,
                        help="Do not send CENTER when the Ultra96 bridge connects")
    parser.add_argument("--audio-fallback", action="store_true", default=config.TELLO_AUDIO_FALLBACK)
    parser.add_argument("--no-audio-fallback", action="store_false", dest="audio_fallback")
    parser.add_argument("--jetson-sender-host", default=config.JETSON_SENDER_HOST,
                        help="Raspberry Pi dashboard UDP telemetry IP")
    parser.add_argument("--jetson-sender-port", type=int,
                        default=config.JETSON_SENDER_PORT,
                        help="Raspberry Pi dashboard UDP telemetry port")
    parser.add_argument("--jetson-sender-rate", type=float,
                        default=config.JETSON_SENDER_RATE_HZ,
                        help="Jetson -> Raspberry Pi telemetry send rate Hz")
    parser.add_argument("--no-jetson-sender", action="store_true",
                        help="Disable Raspberry Pi dashboard UDP telemetry")
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


def save_laser_bbox_tick_sample(path, recent, laser_tick):
    rows = list(recent or [])
    if not rows:
        return None
    bbox_h = sum(float(r["bbox_h"]) for r in rows) / len(rows)
    bbox_w = sum(float(r["bbox_w"]) for r in rows) / len(rows)
    bbox_area = sum(float(r["bbox_area"]) for r in rows) / len(rows)
    conf = sum(float(r["conf"]) for r in rows) / len(rows)
    sample = {
        "bbox_h": round(bbox_h, 2),
        "bbox_w": round(bbox_w, 2),
        "bbox_area": round(bbox_area, 2),
        "confidence": round(conf, 4),
        "laser_id": int(getattr(config, "ULTRA_CHAN_LASER_ID", 3)),
        "laser_tilt_tick": int(laser_tick),
        "sample_count": len(rows),
        "timestamp": datetime.now().astimezone().strftime("%Y-%m-%dT%H:%M:%S%z"),
    }
    data = {"type": "bbox_height_to_laser_tick", "samples": []}
    if path and os.path.exists(path):
        with open(path, "r", encoding="utf-8") as f:
            loaded = json.load(f)
        if isinstance(loaded, dict):
            data.update(loaded)
            data.setdefault("samples", [])
    data["type"] = "bbox_height_to_laser_tick"
    data["samples"].append(sample)
    data["samples"].sort(key=lambda item: float(item.get("bbox_h", 0.0)))
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)
        f.write("\n")
    os.replace(tmp, path)
    return sample


def clamp_audio_angle(angle):
    angle = normalize_audio_angle(angle)
    angle = angle * float(getattr(config, "TELLO_AUDIO_DOA_SIGN", 1))
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


def clamp_tick(tick):
    return max(0, min(4095, int(round(float(tick)))))


def fmt_distance_m(distance_mm):
    if distance_mm in (None, ""):
        return "-"
    try:
        return f"{float(distance_mm) / 1000.0:.2f}"
    except Exception:
        return "-"


def laser_image_offset_ticks(cy, frame_h):
    frame_h = max(1, int(frame_h or 0))
    fov_deg = float(getattr(config, "ULTRA_CHAN_LASER_VERTICAL_FOV_DEG", 43.0))
    err_y = float(cy) - float(frame_h // 2)
    ticks_per_deg_num = 1024.0
    ticks_per_deg_den = 90.0
    ticks = -err_y * fov_deg * ticks_per_deg_num / (ticks_per_deg_den * frame_h)
    return int(ticks)


def laser_goal_for_bbox(base_tick, bbox_cy, frame_h):
    if base_tick is None:
        return None
    goal = (
        int(base_tick)
        + laser_image_offset_ticks(bbox_cy, frame_h)
        + int(getattr(config, "LASER_BBOX_AIM_OFFSET_TICK", 0))
    )
    return clamp_tick(goal)


def laser_center_range_offset_ticks(bbox_h, distance_mm=None):
    if not bool(getattr(config, "LASER_CAMERA_CENTER_RANGE_COMP", False)):
        return 0
    if bool(getattr(config, "LASER_CAMERA_CENTER_RANGE_COMP_USE_DISTANCE", False)) and distance_mm:
        near_mm = float(getattr(config, "LASER_CAMERA_CENTER_NEAR_DISTANCE_MM", 1000))
        far_mm = float(getattr(config, "LASER_CAMERA_CENTER_FAR_DISTANCE_MM", 3000))
        far_offset = int(getattr(config, "LASER_CAMERA_CENTER_FAR_OFFSET_TICK", 36))
        if far_mm <= near_mm:
            return 0
        ratio = (float(distance_mm) - near_mm) / max(1e-6, far_mm - near_mm)
        ratio = max(0.0, min(1.0, ratio))
        return int(round(ratio * far_offset))
    near_h = float(getattr(config, "LASER_CAMERA_CENTER_NEAR_BBOX_H", 64.0))
    far_h = float(getattr(config, "LASER_CAMERA_CENTER_FAR_BBOX_H", 19.0))
    far_offset = int(getattr(config, "LASER_CAMERA_CENTER_FAR_OFFSET_TICK", 36))
    h = float(max(1.0, bbox_h or 0.0))
    if near_h <= far_h:
        return 0
    if h >= near_h:
        return 0
    if h <= far_h:
        return far_offset
    ratio = (near_h - h) / max(1e-6, near_h - far_h)
    return int(round(ratio * far_offset))


def audio_doa_to_motor_angle(doa_angle):
    zero_doa = float(getattr(config, "TELLO_AUDIO_MOTOR_ZERO_DOA_DEG", 90.0))
    sign = float(getattr(config, "TELLO_AUDIO_DOA_SIGN", 1))
    return normalize_audio_angle((float(doa_angle) - zero_doa) * sign)


class AudioDirectionStabilizer:
    def __init__(self, window=7, min_votes=3, max_spread_deg=35.0, reject_rear=True):
        self.window = max(1, int(window))
        self.min_votes = max(1, int(min_votes))
        self.max_spread_deg = float(max_spread_deg)
        self.reject_rear = bool(reject_rear)
        self.samples = []
        self.last_reason = "empty"

    def _bin(self, angle):
        angle = normalize_audio_angle(angle)
        if -45.0 <= angle <= 45.0:
            return "front"
        if 45.0 < angle <= 135.0:
            return "right"
        if -135.0 <= angle < -45.0:
            return "left"
        return "rear"

    def update(self, angle, score=1.0):
        angle = normalize_audio_angle(angle)
        self.samples.append({
            "angle": angle,
            "score": float(score),
            "bin": self._bin(angle),
            "time": time.perf_counter(),
        })
        self.samples = self.samples[-self.window:]
        counts = {}
        for sample in self.samples:
            counts[sample["bin"]] = counts.get(sample["bin"], 0) + 1
        if not counts:
            self.last_reason = "empty"
            return None
        best_bin, votes = max(counts.items(), key=lambda item: item[1])
        if votes < self.min_votes:
            self.last_reason = f"votes:{votes}/{self.min_votes}"
            return None
        if best_bin == "rear" and self.reject_rear:
            self.last_reason = "rear"
            return None
        angles = sorted(sample["angle"] for sample in self.samples if sample["bin"] == best_bin)
        spread = angles[-1] - angles[0] if len(angles) > 1 else 0.0
        if spread > self.max_spread_deg:
            self.last_reason = f"spread:{spread:.1f}"
            return None
        mid = len(angles) // 2
        if len(angles) % 2:
            stable_angle = angles[mid]
        else:
            stable_angle = (angles[mid - 1] + angles[mid]) / 2.0
        self.last_reason = f"stable:{best_bin}:{votes}"
        return stable_angle


class AudioSearchLimiter:
    """Bound audio-only pan searches so they cannot loop forever."""

    def __init__(self, bin_deg=15.0, direction_cooldown_sec=5.0,
                 max_attempts=3, attempt_reset_sec=10.0):
        self.bin_deg = max(1.0, float(bin_deg))
        self.direction_cooldown_sec = max(0.0, float(direction_cooldown_sec))
        self.max_attempts = max(0, int(max_attempts))
        self.attempt_reset_sec = max(0.0, float(attempt_reset_sec))
        self._last_by_bin = {}
        self._attempts = 0
        self._last_attempt = 0.0
        self.last_reason = "ready"

    def reset(self):
        self._attempts = 0
        self._last_attempt = 0.0
        self._last_by_bin.clear()
        self.last_reason = "reset"

    def _direction_bin(self, angle):
        return int(round(float(angle) / self.bin_deg))

    def allow(self, angle, now=None):
        now = time.perf_counter() if now is None else float(now)
        if self._last_attempt and now - self._last_attempt >= self.attempt_reset_sec:
            self._attempts = 0
            self._last_by_bin.clear()

        if self.max_attempts > 0 and self._attempts >= self.max_attempts:
            self.last_reason = f"audio_attempt_limit:{self._attempts}/{self.max_attempts}"
            return False

        direction_bin = self._direction_bin(angle)
        last_same_direction = self._last_by_bin.get(direction_bin)
        if (
            last_same_direction is not None
            and now - last_same_direction < self.direction_cooldown_sec
        ):
            remain = self.direction_cooldown_sec - (now - last_same_direction)
            self.last_reason = f"audio_dir_cooldown:{remain:.1f}s"
            return False

        self._attempts += 1
        self._last_attempt = now
        self._last_by_bin[direction_bin] = now
        limit_text = "unlimited" if self.max_attempts <= 0 else str(self.max_attempts)
        self.last_reason = f"audio_search:{self._attempts}/{limit_text}"
        return True


def is_vision_active_for_audio(system_state):
    value = getattr(system_state, "value", str(system_state))
    return value in {
        "DETECTED",
        "TRACKING",
        "LOCKED",
        "ENGAGED",
        "ENGAGED_FIRING",
        "ENGAGED_ASSESSING",
    }


def main():
    args = parse_args()
    os.environ["YOLO_DEVICE"] = str(args.device)
    os.environ["YOLO_CONF"] = str(args.conf)
    os.environ["YOLO_IMGSZ"] = str(args.imgsz)
    os.environ["YOLO_SKIP_FRAMES"] = str(max(1, int(args.skip_frames)))
    os.environ["YOLO_FAST_DETECT"] = "1" if args.fast_detect else "0"
    os.environ["ULTRA_CHAN_ASYNC_SEND"] = "1" if args.async_motor else "0"
    os.environ["ULTRA_YUBIN_ASYNC_SEND"] = "1" if args.async_motor else "0"
    center_on_start = not args.no_center_on_start
    os.environ["ULTRA_CHAN_CENTER_ON_START"] = "1" if center_on_start else "0"
    os.environ["ULTRA_YUBIN_CENTER_ON_START"] = "1" if center_on_start else "0"

    state = SharedState()
    decision = DecisionMaker(state)
    motor = UltraYubinMotorController().start()
    laser = LaserController(
        pin=config.LASER_PIN,
        enabled=config.LASER_ENABLED,
        pin_mode=config.LASER_PIN_MODE,
        active_high=config.LASER_ACTIVE_HIGH,
    )
    display = None if args.headless else AntiDroneDisplay(state)
    jetson_sender = JetsonTelemetrySender(
        args.jetson_sender_host,
        args.jetson_sender_port,
        args.jetson_sender_rate,
        enabled=(config.JETSON_SENDER_ENABLED and not args.no_jetson_sender),
    )
    if jetson_sender.enabled:
        print(
            "[JETSON-SENDER] UDP dashboard telemetry "
            f"{jetson_sender.host}:{jetson_sender.port} "
            f"rate={1.0 / jetson_sender.min_interval:.1f}Hz"
        )
    else:
        print("[JETSON-SENDER] dashboard telemetry disabled")
    pipeline = PipelineLogger(
        path=args.pipeline_log,
        enabled=not args.no_pipeline_log,
        echo=args.pipeline_echo,
        echo_every=args.pipeline_echo_every,
    )

    cam = CameraStream(args.camera).start()
    detector = VisionDetector(args.model)
    analyzer = ThreatAnalyzer(config.CAMERA_WIDTH, config.CAMERA_HEIGHT)
    distance_estimator = DistanceEstimator(
        config.DISTANCE_MODEL_PATH,
        default_mm=config.DISTANCE_DEFAULT_MM,
        min_mm=config.DISTANCE_MIN_MM,
        max_mm=config.DISTANCE_MAX_MM,
    )
    laser_tick_estimator = LaserTickEstimator(
        getattr(config, "LASER_BBOX_TICK_MODEL_PATH", ""),
    )
    laser_center_lock_enabled = bool(getattr(config, "LASER_CAMERA_CENTER_LOCK", False))
    laser_center_lock_tick = int(getattr(config, "LASER_CAMERA_CENTER_TICK", 1965))
    audio = None
    audio_started = False
    audio_lazy_start = bool(getattr(config, "TELLO_AUDIO_LAZY_START", False))
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
                    min_avg_score=config.TELLO_AUDIO_MIN_AVG_SCORE,
                    consecutive=config.TELLO_AUDIO_CONSECUTIVE,
                    min_rms=config.TELLO_AUDIO_MIN_RMS,
                    doa_offset=config.TELLO_AUDIO_DOA_OFFSET,
                )
                if not audio_lazy_start:
                    audio.start()
                    audio_started = True
                audio_device = audio.alsa_device
            elif audio_mode == "junmo":
                audio = JunmoDroneAudioFallback(
                    config.TELLO_AUDIO_JUNMO_MODEL,
                    project_root=config.TELLO_AUDIO_JUNMO_ROOT,
                    alsa_device=config.TELLO_AUDIO_ALSA_DEVICE,
                    channels=config.TELLO_AUDIO_CHANNELS,
                    threshold=config.TELLO_AUDIO_THRESHOLD,
                    min_avg_score=config.TELLO_AUDIO_MIN_AVG_SCORE,
                    consecutive=config.TELLO_AUDIO_CONSECUTIVE,
                    cooldown_sec=config.TELLO_AUDIO_COOLDOWN_SEC,
                    min_rms=config.TELLO_AUDIO_MIN_RMS,
                    doa_offset=config.TELLO_AUDIO_DOA_OFFSET,
                    doa_method=config.TELLO_AUDIO_DOA_METHOD,
                    mic_distance=config.TELLO_AUDIO_MIC_DISTANCE,
                    audio_backend=config.TELLO_AUDIO_BACKEND,
                    verbose=config.TELLO_AUDIO_VERBOSE,
                )
                if not audio_lazy_start:
                    audio.start()
                    audio_started = True
                audio_device = f"junmoyolo26:{config.TELLO_AUDIO_ALSA_DEVICE}"
            else:
                audio_mode = "doa"
                audio = ReSpeakerDOA(offset=config.TELLO_AUDIO_DOA_OFFSET)
                audio_started = True
                audio_device = "respeaker-usb-doa"
            state.update_audio_status(
                "AUDIO STANDBY" if audio_lazy_start and not audio_started else "AUDIO READY",
                0.0,
                0.0,
                None,
                None,
                False,
            )
            print(
                f"[audio] fallback=on mode={audio_mode} device={audio_device} "
                f"threshold={config.TELLO_AUDIO_THRESHOLD} "
                f"min_avg={config.TELLO_AUDIO_MIN_AVG_SCORE} "
                f"consecutive={config.TELLO_AUDIO_CONSECUTIVE} "
                f"offset={config.TELLO_AUDIO_DOA_OFFSET} "
                f"lazy={int(audio_lazy_start)} verbose={int(config.TELLO_AUDIO_VERBOSE)}"
            )
        except Exception as exc:
            audio = None
            state.update_audio_status(f"AUDIO ERROR: {exc}", 0.0, 0.0, None, None, False, add_log=True)
            print(f"[audio] fallback disabled: {exc}")

    print(
        f"[jetson] camera={args.camera} source={cam.active_source} "
        f"model={args.model} device={args.device} conf={args.conf} imgsz={args.imgsz} "
        f"skip={max(1, int(args.skip_frames))} fast_detect={int(args.fast_detect)} "
        f"async_motor={int(args.async_motor)} center_on_start={int(center_on_start)} "
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
    last_audio_detection_time = 0.0
    smooth_target_center = None
    last_raw_target_center = None
    last_motor_target_center = None
    lead_velocity = None
    pending_jump_center = None
    lost_since = None
    last_laser_target_seen = 0.0
    laser_status = laser.status()
    laser_runtime_cal_enabled = bool(getattr(config, "LASER_RUNTIME_CALIBRATION", False)) and display is not None
    laser_manual_tick = None
    laser_cal_step = max(1, int(getattr(config, "LASER_RUNTIME_CAL_STEP", 4)))
    laser_cal_big_step = max(laser_cal_step, int(getattr(config, "LASER_RUNTIME_CAL_BIG_STEP", 40)))
    laser_cal_recent = deque(maxlen=20)
    last_laser_aim_sent = 0.0
    last_laser_aim_tick = None
    if laser_runtime_cal_enabled:
        if laser_center_lock_enabled:
            print(
                "[LASER-CAL] camera-center laser lock: "
                "j/k C center adjust | J/K big | [/] step | SPACE laser pattern"
            )
        else:
            print(
                "[LASER-CAL] final-run calibration enabled: "
                "j/k C | J/K big | [/] step | s save bbox-center hit | SPACE laser pattern"
            )
    audio_stabilizer = AudioDirectionStabilizer(
        window=config.TELLO_AUDIO_STABLE_WINDOW,
        min_votes=config.TELLO_AUDIO_STABLE_MIN_VOTES,
        max_spread_deg=config.TELLO_AUDIO_STABLE_MAX_SPREAD_DEG,
        reject_rear=config.TELLO_AUDIO_REJECT_REAR,
    )
    audio_search_limiter = AudioSearchLimiter(
        bin_deg=config.TELLO_AUDIO_DIRECTION_BIN_DEG,
        direction_cooldown_sec=config.TELLO_AUDIO_DIRECTION_COOLDOWN_SEC,
        max_attempts=config.TELLO_AUDIO_MAX_SEARCH_ATTEMPTS,
        attempt_reset_sec=config.TELLO_AUDIO_ATTEMPT_RESET_SEC,
    )

    try:
        while True:
            loop_started = time.perf_counter()
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

            laser_should_on = bool(target)
            laser.set_active(laser_should_on, "bbox" if laser_should_on else "no_target")
            laser_status = laser.status()

            if target and config.TRACK_VISION_MOTOR_ENABLE:
                lost_since = None
                audio_search_limiter.reset()
                detect_count += 1
                x1, y1, x2, y2 = target["box"]
                raw_cx = int((x1 + x2) / 2)
                raw_cy = int((y1 + y2) / 2)
                raw_target_center = (raw_cx, raw_cy)
                previous_raw_target_center = last_raw_target_center
                if previous_raw_target_center is None:
                    target_jump_px = 0.0
                else:
                    target_jump_px = math.hypot(
                        raw_target_center[0] - previous_raw_target_center[0],
                        raw_target_center[1] - previous_raw_target_center[1],
                    )
                last_raw_target_center = raw_target_center
                bw = int(x2 - x1)
                bh = int(y2 - y1)
                distance_mm = distance_estimator.estimate(bw, bh)
                range_laser_offset_tick = laser_center_range_offset_ticks(bh, distance_mm)
                active_laser_center_lock_tick = clamp_tick(laser_center_lock_tick + range_laser_offset_tick)
                if laser_center_lock_enabled:
                    laser_base_tick = active_laser_center_lock_tick
                    laser_goal_tick = active_laser_center_lock_tick
                else:
                    laser_base_tick = laser_tick_estimator.estimate(bw, bh)
                    laser_goal_tick = laser_goal_for_bbox(laser_base_tick, raw_cy, frame_h)
                if laser_runtime_cal_enabled:
                    laser_cal_recent.append({
                        "bbox_h": float(max(0, bh)),
                        "bbox_w": float(max(0, bw)),
                        "bbox_area": float(max(0, bw * bh)),
                        "conf": float(target.get("conf", 0.0)),
                    })
                    if laser_manual_tick is not None:
                        laser_base_tick = laser_manual_tick
                        if laser_center_lock_enabled:
                            active_laser_center_lock_tick = int(laser_manual_tick)
                            laser_goal_tick = laser_base_tick
                        else:
                            laser_goal_tick = laser_goal_for_bbox(laser_base_tick, raw_cy, frame_h)
                threat_info = analyzer.update(target["box"])
                conf = float(target.get("conf", 0.0))
                held = bool(target.get("held", False))
                area = max(0, bw * bh)
                edge_x = int(config.TRACK_MOTOR_EDGE_MARGIN_X)
                edge_y = int(config.TRACK_MOTOR_EDGE_MARGIN_Y)
                motor_min_area = int(getattr(config, "TRACK_MOTOR_MIN_AREA", 0))
                motor_min_w = int(getattr(config, "TRACK_MOTOR_MIN_W", 0))
                motor_min_h = int(getattr(config, "TRACK_MOTOR_MIN_H", 0))
                aim_x = camera_center_x + int(getattr(motor, "aim_offset_x", 0))
                aim_y = camera_center_y + int(getattr(motor, "aim_offset_y", 0))
                motor_raw_cx = raw_cx
                motor_raw_cy = raw_cy
                lead_frames = max(0.0, float(getattr(config, "TRACK_LEAD_FRAMES", 0.0)))
                lead_min_conf = float(getattr(config, "TRACK_LEAD_MIN_CONF", 0.65))
                if (
                    lead_frames > 0.0
                    and previous_raw_target_center is not None
                    and not held
                    and conf >= lead_min_conf
                ):
                    inst_vx = float(raw_cx - previous_raw_target_center[0])
                    inst_vy = float(raw_cy - previous_raw_target_center[1])
                    lead_alpha = max(0.0, min(1.0, float(getattr(config, "TRACK_LEAD_VELOCITY_ALPHA", 0.55))))
                    reset_jump_px = float(getattr(config, "TRACK_LEAD_RESET_JUMP_PX", 240.0))
                    if lead_velocity is None or target_jump_px > reset_jump_px:
                        lead_velocity = (inst_vx, inst_vy)
                    else:
                        lead_velocity = (
                            lead_velocity[0] + (inst_vx - lead_velocity[0]) * lead_alpha,
                            lead_velocity[1] + (inst_vy - lead_velocity[1]) * lead_alpha,
                        )
                    max_lead_px = max(0.0, float(getattr(config, "TRACK_LEAD_MAX_PX", 80.0)))
                    lead_dx = max(-max_lead_px, min(max_lead_px, lead_velocity[0] * lead_frames))
                    lead_dy = max(-max_lead_px, min(max_lead_px, lead_velocity[1] * lead_frames))
                    motor_raw_cx = max(0, min(frame_w, int(round(raw_cx + lead_dx))))
                    motor_raw_cy = max(0, min(frame_h, int(round(raw_cy + lead_dy))))
                elif target_jump_px > float(getattr(config, "TRACK_LEAD_RESET_JUMP_PX", 240.0)):
                    lead_velocity = None
                raw_err_x = motor_raw_cx - aim_x
                raw_err_y = motor_raw_cy - aim_y
                center_locked = (
                    abs(raw_err_x) <= int(getattr(config, "TRACK_CENTER_LOCK_X_PX", 0))
                    and abs(raw_err_y) <= int(getattr(config, "TRACK_CENTER_LOCK_Y_PX", 0))
                )
                servo_gain = max(0.05, min(1.0, float(getattr(config, "TRACK_SERVO_GAIN", 1.0))))
                servo_raw_center = (
                    int(round(aim_x + raw_err_x * servo_gain)),
                    int(round(aim_y + raw_err_y * servo_gain)),
                )
                if smooth_target_center is None:
                    init_dx = float(servo_raw_center[0] - aim_x)
                    init_dy = float(servo_raw_center[1] - aim_y)
                    init_dist = math.hypot(init_dx, init_dy)
                    init_step = float(getattr(config, "TRACK_SERVO_INITIAL_STEP_PX", 0.0))
                    if init_step > 0.0 and init_dist > init_step:
                        scale = init_step / init_dist
                        servo_raw_center = (
                            int(round(aim_x + init_dx * scale)),
                            int(round(aim_y + init_dy * scale)),
                        )
                    smooth_target_center = (
                        float(servo_raw_center[0]),
                        float(servo_raw_center[1]),
                    )
                    candidate_motor_center = servo_raw_center
                else:
                    sx, sy = smooth_target_center
                    dx = float(servo_raw_center[0]) - sx
                    dy = float(servo_raw_center[1]) - sy
                    dist = math.hypot(dx, dy)
                    max_step = float(getattr(config, "TARGET_MAX_CENTER_STEP_PX", 520.0))
                    step_cx = servo_raw_center[0]
                    step_cy = servo_raw_center[1]
                    if dist > max_step and dist > 1e-6:
                        scale = max_step / dist
                        step_cx = int(round(sx + dx * scale))
                        step_cy = int(round(sy + dy * scale))
                        dx = float(step_cx) - sx
                        dy = float(step_cy) - sy
                        dist = math.hypot(dx, dy)
                    near_alpha = float(getattr(
                        config,
                        "TARGET_CENTER_SMOOTHING_NEAR",
                        getattr(config, "TARGET_CENTER_SMOOTHING", 0.80),
                    ))
                    far_alpha = float(getattr(
                        config,
                        "TARGET_CENTER_SMOOTHING_FAR",
                        getattr(config, "TARGET_CENTER_SMOOTHING", 0.80),
                    ))
                    far_px = max(1.0, float(getattr(config, "TARGET_CENTER_SMOOTHING_FAR_PX", 180.0)))
                    alpha = near_alpha + (far_alpha - near_alpha) * min(1.0, dist / far_px)
                    smooth_target_center = (
                        sx + (float(step_cx) - sx) * alpha,
                        sy + (float(step_cy) - sy) * alpha,
                    )
                    candidate_motor_center = (
                        int(round(smooth_target_center[0])),
                        int(round(smooth_target_center[1])),
                    )
                previous_motor_center = last_motor_target_center
                if previous_motor_center is not None:
                    mdx = candidate_motor_center[0] - previous_motor_center[0]
                    mdy = candidate_motor_center[1] - previous_motor_center[1]
                    if math.hypot(mdx, mdy) < float(getattr(config, "TARGET_MOTOR_CENTER_DEAD_BAND_PX", 5.0)):
                        candidate_motor_center = previous_motor_center
                jump_hold_px = float(getattr(config, "TRACK_MOTOR_JUMP_HOLD_PX", 200.0))
                if previous_motor_center is not None and target_jump_px > jump_hold_px:
                    confirm_px = float(getattr(config, "TRACK_MOTOR_JUMP_CONFIRM_PX", 90.0))
                    confirmed_jump = (
                        pending_jump_center is not None
                        and math.hypot(
                            raw_target_center[0] - pending_jump_center[0],
                            raw_target_center[1] - pending_jump_center[1],
                        ) <= confirm_px
                    )
                    if confirmed_jump:
                        pending_jump_center = None
                    else:
                        pending_jump_center = raw_target_center
                        candidate_motor_center = previous_motor_center
                        smooth_target_center = (
                            float(previous_motor_center[0]),
                            float(previous_motor_center[1]),
                        )
                else:
                    pending_jump_center = None
                if previous_motor_center is not None:
                    mdx = candidate_motor_center[0] - previous_motor_center[0]
                    mdy = candidate_motor_center[1] - previous_motor_center[1]
                    motor_step = math.hypot(mdx, mdy)
                    max_motor_step = float(getattr(config, "TARGET_MOTOR_MAX_STEP_PX", 0.0))
                    if max_motor_step > 0.0 and motor_step > max_motor_step:
                        scale = max_motor_step / motor_step
                        candidate_motor_center = (
                            int(round(previous_motor_center[0] + mdx * scale)),
                            int(round(previous_motor_center[1] + mdy * scale)),
                        )
                        smooth_target_center = (
                            float(candidate_motor_center[0]),
                            float(candidate_motor_center[1]),
                        )
                if center_locked:
                    candidate_motor_center = (aim_x, aim_y)
                    smooth_target_center = (float(aim_x), float(aim_y))
                    pending_jump_center = None
                cx, cy = candidate_motor_center
                last_motor_target_center = candidate_motor_center
                near_edge = (
                    raw_cx < edge_x or raw_cx > frame_w - edge_x or
                    raw_cy < edge_y or raw_cy > frame_h - edge_y
                )
                too_small_for_motor = (
                    (motor_min_area > 0 and area < motor_min_area) or
                    (motor_min_w > 0 and bw < motor_min_w) or
                    (motor_min_h > 0 and bh < motor_min_h)
                )
                held_motor_ok = held and bool(getattr(config, "TRACK_MOTOR_USE_HELD", True))
                if center_locked:
                    last_telemetry = motor_skip_telemetry(
                        last_telemetry, "center_lock", cx, cy, aim_x, aim_y
                    )
                elif held and not held_motor_ok:
                    last_telemetry = motor_skip_telemetry(
                        last_telemetry, "held", cx, cy, aim_x, aim_y
                    )
                elif not held_motor_ok and conf < float(config.TRACK_MOTOR_MIN_CONF):
                    last_telemetry = motor_skip_telemetry(
                        last_telemetry, f"low_conf:{conf:.2f}", cx, cy, aim_x, aim_y
                    )
                elif too_small_for_motor:
                    last_telemetry = motor_skip_telemetry(
                        last_telemetry,
                        f"small_box:{bw}x{bh}",
                        cx,
                        cy,
                        aim_x,
                        aim_y,
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
                        distance_mm=distance_mm,
                        laser_base_tick=laser_base_tick,
                        laser_center_lock_tick=active_laser_center_lock_tick if laser_center_lock_enabled else None,
                        aim_center_x=camera_center_x,
                        aim_center_y=camera_center_y,
                    )
                last_telemetry.update({
                    "distance_mm": distance_mm,
                    "laser_range_offset_tick": range_laser_offset_tick,
                    "laser_center_lock_tick": active_laser_center_lock_tick if laser_center_lock_enabled else None,
                })
                telemetry_cmd = str(last_telemetry.get("tx_cmd", ""))
                if telemetry_cmd.startswith("T "):
                    last_laser_aim_sent = time.perf_counter()
                    last_laser_aim_tick = laser_goal_tick
                elif (
                    bool(getattr(config, "LASER_BBOX_DIRECT_AIM", True))
                    and laser_goal_tick is not None
                    and not too_small_for_motor
                    and not near_edge
                ):
                    now_laser_aim = time.perf_counter()
                    min_period = float(getattr(config, "LASER_BBOX_AIM_UPDATE_PERIOD_SEC", 0.06))
                    min_delta = int(getattr(config, "LASER_BBOX_AIM_MIN_DELTA_TICK", 2))
                    if (
                        now_laser_aim - last_laser_aim_sent >= min_period
                        and (
                            last_laser_aim_tick is None
                            or abs(int(laser_goal_tick) - int(last_laser_aim_tick)) >= min_delta
                        )
                    ):
                        ok, goal, reply = motor.set_laser_tick(laser_goal_tick)
                        last_telemetry = motor.last_telemetry
                        last_telemetry.update({
                            "distance_mm": distance_mm,
                            "laser_range_offset_tick": range_laser_offset_tick,
                            "laser_center_lock_tick": active_laser_center_lock_tick if laser_center_lock_enabled else None,
                        })
                        last_laser_aim_sent = now_laser_aim
                        last_laser_aim_tick = goal
                        if frame_idx % 30 == 0:
                            aim_mode = "center" if laser_center_lock_enabled else "bbox"
                            print(
                                f"[LASER-AIM] mode={aim_mode} bbox_h={bh} cy={raw_cy} "
                                f"range_offset={range_laser_offset_tick} base={laser_base_tick} "
                                f"goal={goal} ok={int(ok)} reply={reply}"
                            )
                event = "target_lock" if center_locked else "target_track"
            else:
                no_target_count += 1
                threat_info = None
                now_lost = time.perf_counter()
                if lost_since is None:
                    lost_since = now_lost
                hold_motor_on_lost = bool(getattr(config, "TRACK_HOLD_MOTOR_ON_LOST", False))
                if not hold_motor_on_lost:
                    smooth_target_center = None
                    last_raw_target_center = None
                    last_motor_target_center = None
                    lead_velocity = None
                    pending_jump_center = None
                else:
                    # Keep the motor at its last commanded pose. Also keep the
                    # previous filtered target so reacquire continues smoothly
                    # instead of jumping from the image center.
                    last_raw_target_center = None
                    pending_jump_center = None
                if (
                    audio
                    and not audio_started
                    and now_lost - lost_since >= float(getattr(config, "TELLO_AUDIO_LAZY_START_AFTER_SEC", 0.8))
                ):
                    try:
                        audio.start()
                        audio_started = True
                        state.update_audio_status("AUDIO READY", 0.0, 0.0, None, None, False)
                        print("[audio] lazy fallback started after vision loss")
                    except Exception as exc:
                        audio = None
                        state.update_audio_status(
                            f"AUDIO ERROR: {exc}",
                            0.0,
                            0.0,
                            None,
                            None,
                            False,
                            add_log=True,
                        )
                        print(f"[audio] lazy fallback disabled: {exc}")
                audio_allowed = bool(audio) and audio_started and not state.is_vision_active(
                    hold_sec=float(getattr(config, "TELLO_AUDIO_VISION_HOLD_SEC", 0.15))
                )
                if audio and not audio_allowed:
                    state.update_audio_status(
                        "AUDIO STANDBY" if not audio_started else "AUDIO HOLD VISION",
                        state.audio_score,
                        state.audio_rms,
                        state.audio_doa,
                        state.audio_section,
                        False,
                    )
                    audio_detection = None
                elif audio and audio_mode in ("model", "junmo"):
                    audio_detection = audio.get_detection(config.TELLO_AUDIO_MAX_AGE_SEC)
                elif audio and audio_mode == "doa":
                    raw_angle = audio.read()
                    if bool(getattr(config, "TELLO_AUDIO_DOA_ONLY_SEARCH", False)):
                        audio_detection = {
                            "angle": float(raw_angle),
                            "raw_angle": int(raw_angle),
                            "score": 1.0,
                            "rms": 0.0,
                            "mode": "doa",
                        }
                    else:
                        state.update_audio_status(
                            "AUDIO DOA RAW",
                            0.0,
                            0.0,
                            audio_doa_to_motor_angle(raw_angle),
                            audio_section(audio_doa_to_motor_angle(raw_angle)),
                            False,
                        )
                        audio_detection = None
                else:
                    audio_detection = None
                if audio_detection:
                    detection_time = float(audio_detection.get("time", 0.0) or 0.0)
                    if detection_time and detection_time <= last_audio_detection_time:
                        audio_detection = None
                if audio_detection:
                    detection_time = float(audio_detection.get("time", 0.0) or 0.0)
                    if detection_time:
                        last_audio_detection_time = detection_time
                    if audio_mode in ("junmo", "doa"):
                        signed_angle = audio_doa_to_motor_angle(audio_detection["angle"])
                    else:
                        signed_angle = normalize_audio_angle(audio_detection["angle"])
                        signed_angle *= float(getattr(config, "TELLO_AUDIO_DOA_SIGN", 1))
                    stable_angle = audio_stabilizer.update(
                        signed_angle,
                        audio_detection.get("score", 1.0),
                    )
                    audio_detection["stable_reason"] = audio_stabilizer.last_reason
                    audio_detection["signed_angle"] = signed_angle
                    if stable_angle is None:
                        audio_angle = max(
                            -abs(float(config.TELLO_AUDIO_CLAMP_DEG)),
                            min(abs(float(config.TELLO_AUDIO_CLAMP_DEG)), signed_angle),
                        )
                        audio_detection["send_angle"] = None
                    else:
                        audio_angle = max(
                            -abs(float(config.TELLO_AUDIO_CLAMP_DEG)),
                            min(abs(float(config.TELLO_AUDIO_CLAMP_DEG)), stable_angle),
                        )
                        audio_detection["send_angle"] = audio_angle
                    section = audio_section(audio_angle)
                    state.update_audio_status(
                        "AUDIO TELLO" if audio_mode in ("model", "junmo") else "AUDIO DOA",
                        audio_detection.get("score", 0.0),
                        audio_detection.get("rms", 0.0),
                        audio_angle,
                        section,
                        stable_angle is not None,
                    )
                    now = time.perf_counter()
                    should_send_audio = (
                        stable_angle is not None
                        and
                        now - last_audio_sent >= float(config.TELLO_AUDIO_CONTROL_PERIOD_SEC)
                        and (
                            bool(getattr(config, "TELLO_AUDIO_ALWAYS_SEND", False))
                            or
                            last_audio_angle is None
                            or audio_delta(audio_angle, last_audio_angle) >= float(config.TELLO_AUDIO_MIN_CHANGE_DEG)
                            or now - last_audio_sent >= float(config.TELLO_AUDIO_KEEPALIVE_SEC)
                        )
                    )
                    if should_send_audio and not audio_search_limiter.allow(audio_angle, now):
                        should_send_audio = False
                        audio_detection["stable_reason"] = audio_search_limiter.last_reason
                        if audio_search_limiter.last_reason.startswith("audio_attempt_limit"):
                            with state.lock:
                                current_system_state = state.system_state
                            if current_system_state != SystemState.SCANNING:
                                state.transition(SystemState.SCANNING)
                    if should_send_audio:
                        if state.is_vision_active(hold_sec=0.0):
                            last_telemetry = motor_skip_telemetry(
                                last_telemetry,
                                "audio_cancel_vision",
                                camera_center_x,
                                camera_center_y,
                                camera_center_x,
                                camera_center_y,
                                active=0,
                            )
                            event = "audio_hold"
                        else:
                            last_telemetry = motor.turn_to_doa(audio_angle)
                            last_audio_sent = time.perf_counter()
                            last_audio_angle = audio_angle
                            event = "audio_fallback"
                    else:
                        last_telemetry = motor_skip_telemetry(
                            last_telemetry,
                            audio_detection.get(
                                "stable_reason",
                                audio_stabilizer.last_reason if stable_angle is None else "audio_rate",
                            ),
                            camera_center_x,
                            camera_center_y,
                            camera_center_x,
                            camera_center_y,
                            active=0,
                        )
                        event = "audio_hold"
                elif hold_motor_on_lost:
                    last_cx, last_cy = last_motor_target_center or (
                        int(last_telemetry.get("send_cx", camera_center_x)),
                        int(last_telemetry.get("send_cy", camera_center_y)),
                    )
                    last_telemetry = motor_skip_telemetry(
                        last_telemetry,
                        f"lost_hold:{now_lost - lost_since:.2f}s",
                        last_cx,
                        last_cy,
                        camera_center_x,
                        camera_center_y,
                        active=0,
                    )
                    event = "no_target_hold"
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
            elapsed_for_fps = max(1e-6, time.perf_counter() - started)
            loop_fps = frame_idx / elapsed_for_fps
            with state.lock:
                audio_info = {
                    "enabled": bool(args.audio_fallback),
                    "direction_deg": state.audio_doa,
                    "confidence": state.audio_score,
                }
            jetson_sender.maybe_send(
                frame_id=frame_idx,
                frame_w=frame_w,
                frame_h=frame_h,
                state_value=state.system_state.value,
                fps=loop_fps,
                target_bbox=target["box"] if target else None,
                target_conf=float(target.get("conf", 0.0)) if target else 0.0,
                motor_info=last_telemetry,
                audio_info=audio_info,
                laser_info={
                    "armed": bool(laser_status.get("laser_output", False)),
                    "hit_detected": bool(
                        laser_status.get(
                            "hit_detected",
                            laser_status.get("laser_hit_detected", False),
                        )
                    ),
                },
                pan_min=config.PAN_MIN,
                pan_max=config.PAN_MAX,
                pan_dir=config.PAN_DIR,
            )
            pipeline.write(
                frame_idx,
                event,
                "jetson_yolo",
                target=payload,
                telemetry=last_telemetry,
                extra={
                    "fps": loop_fps,
                    "infer_ms": detector._last_infer_ms,
                    "vision_step_ms": detector._last_step_ms,
                    "loop_ms": (time.perf_counter() - loop_started) * 1000.0,
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
                    fire_status=laser_status,
                    motor_info=last_telemetry,
                )
                key = cv2.waitKeyEx(1) & 0xFFFFFFFF
                clicked = display.get_clicked_button()
                if key in (27, ord("q"), ord("Q")) or clicked == "RESET":
                    break
                if (
                    key == ord(" ")
                    and bool(getattr(config, "LASER_PATTERN_ON_SPACE", True))
                ):
                    started_pattern = laser.pattern(
                        getattr(config, "LASER_PATTERN_BITS", "110111"),
                        unit_sec=float(getattr(config, "LASER_PATTERN_UNIT_SEC", 0.12)),
                        gap_sec=float(getattr(config, "LASER_PATTERN_GAP_SEC", 0.04)),
                        reason="space",
                    )
                    laser_status = laser.status()
                    print(
                        f"[LASER] SPACE pattern "
                        f"{'started' if started_pattern else 'busy'} "
                        f"bits={getattr(config, 'LASER_PATTERN_BITS', '110111')}"
                    )
                elif laser_runtime_cal_enabled and key != 0xFFFFFFFF:
                    current_tick = laser_manual_tick
                    if current_tick is None:
                        current_tick = last_telemetry.get("laser")
                    if current_tick is None:
                        current_tick = last_telemetry.get("laser_base_tick")
                    if current_tick is None:
                        current_tick = int(getattr(config, "ULTRA_CHAN_LASER_CENTER", 2048))
                    if key == ord("["):
                        laser_cal_step = max(1, laser_cal_step // 2)
                        print(f"[LASER-CAL] step={laser_cal_step}")
                    elif key == ord("]"):
                        laser_cal_step = min(512, laser_cal_step * 2)
                        print(f"[LASER-CAL] step={laser_cal_step}")
                    elif key in (ord("j"), ord("k"), ord("J"), ord("K")):
                        delta = laser_cal_step
                        if key in (ord("J"), ord("K")):
                            delta = laser_cal_big_step
                        if key in (ord("j"), ord("J")):
                            delta = -delta
                        ok, goal, reply = motor.set_laser_tick(int(current_tick) + int(delta))
                        laser_manual_tick = goal
                        last_telemetry = motor.last_telemetry
                        print(f"[LASER-CAL] C {delta:+d} => tick={goal} ok={int(ok)} reply={reply}")
                    elif key in (ord("s"), ord("S")):
                        if laser_center_lock_enabled:
                            print("[LASER-CAL] center-lock mode: s save skipped; use j/k and LASER_CAMERA_CENTER_TICK")
                            continue
                        save_tick = laser_manual_tick
                        if save_tick is None:
                            save_tick = last_telemetry.get("laser")
                        if save_tick is None:
                            print("[LASER-CAL] no laser tick yet; j/k once or wait for tracking")
                        elif not laser_cal_recent:
                            print("[LASER-CAL] no recent bbox; show drone first")
                        else:
                            sample = save_laser_bbox_tick_sample(
                                getattr(config, "LASER_BBOX_TICK_MODEL_PATH", ""),
                                laser_cal_recent,
                                int(save_tick),
                            )
                            laser_tick_estimator = LaserTickEstimator(
                                getattr(config, "LASER_BBOX_TICK_MODEL_PATH", ""),
                            )
                            print(f"[LASER-CAL] SAVED {sample}")

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
        laser.cleanup()
        cam.stop()
        if audio:
            audio.stop()
        motor.stop()
        jetson_sender.close()
        if display:
            try:
                cv2.destroyAllWindows()
            except cv2.error:
                pass


if __name__ == "__main__":
    main()
