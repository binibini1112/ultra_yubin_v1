"""Minimal demo UI: camera, reticle, drone bbox, and audio detection only."""
import time

import cv2

from src.core.state import SystemState
import src.config as config


WHITE = (245, 245, 245)
MUTED = (150, 155, 160)
GREEN = (80, 230, 90)
AMBER = (40, 190, 255)
RED = (60, 70, 255)
CYAN = (230, 190, 70)
BLACK = (0, 0, 0)


class AntiDroneDisplay:
    """Small, fast overlay for the live camera feed."""

    def __init__(self, state):
        self.state = state
        self.window_name = "ULTRA YUBIN V1 CCTV"
        self.ui_buttons = {}
        self._window_created = False
        self._last_clicked = None

    def _ensure_window(self):
        if self._window_created:
            return
        flags = cv2.WINDOW_NORMAL if config.UI_FULLSCREEN else cv2.WINDOW_AUTOSIZE
        try:
            cv2.namedWindow(self.window_name, flags)
            if config.UI_FULLSCREEN:
                cv2.setWindowProperty(
                    self.window_name,
                    cv2.WND_PROP_FULLSCREEN,
                    cv2.WINDOW_FULLSCREEN,
                )
            cv2.setMouseCallback(self.window_name, self._mouse_cb)
        except cv2.error:
            pass
        self._window_created = True

    def draw(self, frame, bboxes, fps, threat_info, decision_maker,
             fire_status=None, motor_info=None):
        self._ensure_window()
        self._draw_overlay(frame, bboxes, fps, motor_info)
        cv2.imshow(self.window_name, frame)
        return frame

    def draw_light(self, frame, bboxes, fps, threat_info,
                   fire_status=None, motor_info=None):
        return self.draw(frame, bboxes, fps, threat_info, None, fire_status, motor_info)

    def _draw_overlay(self, frame, bboxes, fps, motor_info):
        h, w = frame.shape[:2]
        target = next((b for b in (bboxes or []) if b.get("is_target", False)), None)
        state = self.state.system_state
        target_visible = target is not None

        with self.state.lock:
            audio_detected = bool(self.state.audio_detected)
            audio_status = str(self.state.audio_status)
            audio_score = float(self.state.audio_score)
            audio_doa = self.state.audio_doa
            audio_age = time.time() - self.state.audio_updated_at if self.state.audio_updated_at else 999.0

        reticle_x = w // 2
        reticle_y = h // 2
        if target_visible and getattr(config, "UI_RETICLE_FOLLOW_TARGET", True):
            x1, y1, x2, y2 = [int(v) for v in target["box"]]
            reticle_x = (x1 + x2) // 2
            reticle_y = (y1 + y2) // 2
            if getattr(config, "UI_SHOW_CAMERA_CENTER", False):
                self._draw_camera_center(frame, w // 2, h // 2)

        reticle_color = GREEN if target_visible else AMBER if audio_detected and audio_age < 2.0 else MUTED
        self._draw_target_box(frame, target)
        if getattr(config, "UI_SHOW_LASER_CENTER_MARKER", False):
            self._draw_laser_center_marker(frame, w // 2, h // 2)
        if getattr(config, "UI_SHOW_RETICLE", True):
            self._draw_reticle(frame, reticle_x, reticle_y, reticle_color)
        self._draw_top_status(frame, fps, state, target, audio_detected, audio_status, audio_score, audio_doa, audio_age)
        self._draw_motor_hint(frame, motor_info)

    def _draw_reticle(self, frame, cx, cy, color):
        gap = 14
        arm = 62
        cv2.line(frame, (cx - arm, cy), (cx - gap, cy), color, 2, cv2.LINE_AA)
        cv2.line(frame, (cx + gap, cy), (cx + arm, cy), color, 2, cv2.LINE_AA)
        cv2.line(frame, (cx, cy - arm), (cx, cy - gap), color, 2, cv2.LINE_AA)
        cv2.line(frame, (cx, cy + gap), (cx, cy + arm), color, 2, cv2.LINE_AA)
        cv2.circle(frame, (cx, cy), 4, color, -1, cv2.LINE_AA)
        cv2.circle(frame, (cx, cy), 34, color, 1, cv2.LINE_AA)

    def _draw_camera_center(self, frame, cx, cy):
        cv2.line(frame, (cx - 10, cy), (cx + 10, cy), MUTED, 1, cv2.LINE_AA)
        cv2.line(frame, (cx, cy - 10), (cx, cy + 10), MUTED, 1, cv2.LINE_AA)
        cv2.circle(frame, (cx, cy), 18, MUTED, 1, cv2.LINE_AA)

    def _draw_laser_center_marker(self, frame, cx, cy):
        gap = 34
        arm = 76
        color = GREEN
        cv2.circle(frame, (cx, cy), 26, color, 2, cv2.LINE_AA)
        cv2.circle(frame, (cx, cy), 4, CYAN, -1, cv2.LINE_AA)
        cv2.line(frame, (cx - arm, cy), (cx - gap, cy), color, 2, cv2.LINE_AA)
        cv2.line(frame, (cx + gap, cy), (cx + arm, cy), color, 2, cv2.LINE_AA)
        cv2.line(frame, (cx, cy - arm), (cx, cy - gap), color, 2, cv2.LINE_AA)
        cv2.line(frame, (cx, cy + gap), (cx, cy + arm), color, 2, cv2.LINE_AA)

    def _draw_target_box(self, frame, target):
        if not target:
            return
        x1, y1, x2, y2 = [int(v) for v in target["box"]]
        conf = float(target.get("conf", 0.0))
        color = GREEN if conf >= 0.60 else AMBER
        cv2.rectangle(frame, (x1, y1), (x2, y2), BLACK, 5, cv2.LINE_AA)
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2, cv2.LINE_AA)
        cx = (x1 + x2) // 2
        cy = (y1 + y2) // 2
        if getattr(config, "UI_SHOW_BBOX_CROSSHAIR", True):
            cv2.line(frame, (cx - 12, cy), (cx + 12, cy), color, 2, cv2.LINE_AA)
            cv2.line(frame, (cx, cy - 12), (cx, cy + 12), color, 2, cv2.LINE_AA)
        if getattr(config, "UI_SHOW_BBOX_LABEL", True):
            label = f"DRONE {conf:.2f}"
            self._label(frame, label, x1, max(26, y1 - 8), color)

    def _draw_top_status(self, frame, fps, state, target, audio_detected,
                         audio_status, audio_score, audio_doa, audio_age):
        h, w = frame.shape[:2]
        cv2.rectangle(frame, (0, 0), (w, 40), BLACK, -1)
        vision_text = "DRONE DETECTED" if target else "SEARCHING"
        vision_color = GREEN if target else MUTED
        if state in (SystemState.TRACKING, SystemState.LOCKED):
            vision_text = state.value
            vision_color = GREEN if state == SystemState.LOCKED else CYAN

        cv2.putText(frame, "CCTV", (14, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.65, WHITE, 1, cv2.LINE_AA)
        cv2.putText(frame, vision_text, (92, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.62, vision_color, 2, cv2.LINE_AA)

        audio_fresh = audio_detected and audio_age < 2.0
        audio_color = AMBER if audio_fresh else MUTED
        doa_text = "--" if audio_doa is None else f"{audio_doa:.0f}deg"
        audio_text = f"AUDIO {'DETECT' if audio_fresh else 'LISTEN'} {audio_score:.2f} {doa_text}"
        cv2.putText(frame, audio_text, (w - 390, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.55, audio_color, 1, cv2.LINE_AA)
        cv2.putText(frame, f"FPS {int(fps)}", (w - 88, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.50, MUTED, 1, cv2.LINE_AA)

        if audio_fresh and not target:
            self._label(frame, "DRONE SOUND DETECTED", 14, h - 18, AMBER)
        elif target:
            self._label(frame, "VISION TRACKING", 14, h - 18, GREEN)

    def _draw_motor_hint(self, frame, motor_info):
        if not motor_info:
            return
        reply = str(motor_info.get("fpga_reply", ""))
        if reply.startswith("SKIP,"):
            text = reply[:42]
            color = AMBER
        elif reply.startswith("T,"):
            text = "PL TRACK"
            color = GREEN
        elif reply.startswith("A,"):
            text = "AUDIO PAN"
            color = AMBER
        else:
            return
        cv2.putText(frame, text, (14, 64), cv2.FONT_HERSHEY_SIMPLEX, 0.50, color, 1, cv2.LINE_AA)

    def _label(self, frame, text, x, y, color):
        (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.56, 1)
        x = max(0, min(frame.shape[1] - tw - 16, int(x)))
        y = max(th + 8, min(frame.shape[0] - 8, int(y)))
        cv2.rectangle(frame, (x, y - th - 8), (x + tw + 14, y + 5), BLACK, -1)
        cv2.rectangle(frame, (x, y - th - 8), (x + tw + 14, y + 5), color, 1, cv2.LINE_AA)
        cv2.putText(frame, text, (x + 7, y), cv2.FONT_HERSHEY_SIMPLEX, 0.56, color, 1, cv2.LINE_AA)

    def _mouse_cb(self, event, x, y, flags, param):
        return

    def get_clicked_button(self):
        clicked = self._last_clicked
        self._last_clicked = None
        return clicked
