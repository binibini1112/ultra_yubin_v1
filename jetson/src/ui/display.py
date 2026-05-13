"""
display.py — Anti-Drone Defense HUD
==========================================
군사 방공 시스템(Iron Dome/C-RAM) 스타일 인터페이스
- 다크 네이비 배경 + 시안/그린 HUD
- 타겟팅 레티클 + 궤적 트레일
- 미니 레이더 디스플레이
- 위협 등급 / 조준 정확도 / 교전 확률 게이지
"""
import cv2
import time
import math
import os
import numpy as np
import re
import subprocess
from src.core.state import SystemState
import src.config as config

try:
    from PIL import Image, ImageDraw, ImageFont
except Exception:
    Image = ImageDraw = ImageFont = None

# ── 색상 팔레트 (BGR) ─────────────────────────────────────────────
BG_DARK     = (15, 12, 8)        # 패널 배경
HUD_CYAN    = (230, 200, 0)      # 시안 HUD
HUD_GREEN   = (100, 255, 80)     # 그린
HUD_AMBER   = (50, 180, 255)     # 앰버
HUD_RED     = (60, 60, 255)      # 레드
HUD_GOLD    = (40, 210, 255)
HUD_BLUE    = (255, 120, 30)
HUD_WHITE   = (220, 220, 220)
HUD_DIM     = (100, 100, 80)     # 흐린 텍스트
THREAT_COLORS = {
    "GREEN":  (80, 220, 60),
    "YELLOW": (40, 220, 240),
    "ORANGE": (40, 140, 255),
    "RED":    (50, 50, 255),
}

KR_FONT_PATH = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
try:
    KR_FONT_16 = ImageFont.truetype(KR_FONT_PATH, 16) if ImageFont else None
    KR_FONT_14 = ImageFont.truetype(KR_FONT_PATH, 14) if ImageFont else None
    KR_FONT_12 = ImageFont.truetype(KR_FONT_PATH, 12) if ImageFont else None
except Exception:
    KR_FONT_16 = KR_FONT_14 = KR_FONT_12 = None


def put_text(frame, text, org, font_scale=0.45, color=HUD_WHITE, thickness=1):
    """Draw ASCII with OpenCV and Korean with PIL when a CJK font is available."""
    if all(ord(ch) < 128 for ch in str(text)) or KR_FONT_14 is None:
        cv2.putText(frame, str(text), org, cv2.FONT_HERSHEY_SIMPLEX,
                    font_scale, color, thickness, cv2.LINE_AA)
        return

    font = KR_FONT_16 if font_scale >= 0.48 else KR_FONT_14 if font_scale >= 0.38 else KR_FONT_12
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    pil = Image.fromarray(rgb)
    draw = ImageDraw.Draw(pil)
    b, g, r = color
    draw.text(org, str(text), font=font, fill=(r, g, b))
    frame[:] = cv2.cvtColor(np.asarray(pil), cv2.COLOR_RGB2BGR)

STATE_THEME = {
    SystemState.SCANNING:    {"color": HUD_DIM,   "label": "SCANNING",    "blink": False},
    SystemState.DETECTED:    {"color": HUD_AMBER, "label": "DETECTED",    "blink": False},
    SystemState.TRACKING:    {"color": HUD_CYAN,  "label": "TRACKING",    "blink": False},
    SystemState.LOCKED:      {"color": HUD_RED,   "label": "LOCKED ON",   "blink": False},
    SystemState.ENGAGED:     {"color": HUD_RED,   "label": "!! ENGAGED !!", "blink": False},
    SystemState.NEUTRALIZED: {"color": HUD_GREEN, "label": "NEUTRALIZED", "blink": False},
}


class AntiDroneDisplay:
    """군사급 방공 HUD 인터페이스"""

    PANEL_W = 300

    def __init__(self, state):
        self.state = state
        self.window_name = "ANTI-DRONE DEFENSE SYSTEM"
        self.ui_buttons  = {}
        self._window_created  = False
        self._last_panel      = None
        self._screen_size     = None
        self._display_size    = None
        self._panel_w         = self.PANEL_W
        self._camera_display_w = 0
        self._frame_w         = 0

    def _ensure_window(self):
        if self._window_created:
            return
        try:
            flags = cv2.WINDOW_NORMAL if config.UI_FULLSCREEN else cv2.WINDOW_AUTOSIZE
            cv2.namedWindow(self.window_name, flags)
            if config.UI_FULLSCREEN:
                cv2.setWindowProperty(
                    self.window_name,
                    cv2.WND_PROP_FULLSCREEN,
                    cv2.WINDOW_FULLSCREEN,
                )
            cv2.setMouseCallback(self.window_name, self._mouse_cb)
        except cv2.error:
            pass  # headless 환경에서는 imshow가 자동 생성
        self._window_created = True

    def _detect_screen_size(self):
        if config.UI_SCREEN_WIDTH > 0 and config.UI_SCREEN_HEIGHT > 0:
            return config.UI_SCREEN_WIDTH, config.UI_SCREEN_HEIGHT

        display = os.environ.get("DISPLAY")
        if not display:
            return None
        try:
            result = subprocess.run(
                ["xdpyinfo"],
                check=False,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=1.0,
            )
        except Exception:
            return None
        if result.returncode != 0:
            return None
        match = re.search(r"dimensions:\s+(\d+)x(\d+)\s+pixels", result.stdout)
        if not match:
            return None
        return int(match.group(1)), int(match.group(2))

    def _target_display_size(self, frame_w, frame_h):
        if not config.UI_FULLSCREEN:
            return frame_w + self.PANEL_W, frame_h
        if self._screen_size is None:
            self._screen_size = self._detect_screen_size()
        if self._screen_size is None:
            return frame_w + self.PANEL_W, frame_h
        return self._screen_size

    def _panel_width_for(self, display_w):
        configured = int(getattr(config, "UI_PANEL_WIDTH", self.PANEL_W))
        if configured > 0:
            return max(260, min(display_w // 2, configured))
        return max(300, min(420, int(display_w * 0.22)))

    def _resize_cover(self, image, target_w, target_h):
        src_h, src_w = image.shape[:2]
        if src_w <= 0 or src_h <= 0 or target_w <= 0 or target_h <= 0:
            return np.zeros((max(1, target_h), max(1, target_w), 3), dtype=np.uint8)
        if getattr(config, "UI_CAMERA_FIT_MODE", "stretch") == "stretch":
            return cv2.resize(image, (target_w, target_h), interpolation=cv2.INTER_LINEAR)
        scale = max(target_w / float(src_w), target_h / float(src_h))
        resized_w = max(1, int(round(src_w * scale)))
        resized_h = max(1, int(round(src_h * scale)))
        resized = cv2.resize(image, (resized_w, resized_h), interpolation=cv2.INTER_LINEAR)
        x0 = max(0, (resized_w - target_w) // 2)
        y0 = max(0, (resized_h - target_h) // 2)
        return resized[y0:y0 + target_h, x0:x0 + target_w]

    def _compose_output(self, camera_frame, panel):
        frame_h, frame_w = camera_frame.shape[:2]
        display_w, display_h = self._target_display_size(frame_w, frame_h)
        panel_w = panel.shape[1]
        camera_w = max(1, display_w - panel_w)

        camera_view = self._resize_cover(camera_frame, camera_w, display_h)
        if panel.shape[0] != display_h:
            panel = cv2.resize(panel, (panel_w, display_h), interpolation=cv2.INTER_LINEAR)
        self._display_size = (display_w, display_h)
        self._panel_w = panel_w
        self._camera_display_w = camera_w
        return np.hstack((camera_view, panel))

    def _glow_line(self, frame, p1, p2, color, thickness=1, glow=5, alpha=0.26):
        cv2.line(frame, p1, p2, color, thickness, cv2.LINE_AA)

    def _glow_circle(self, frame, center, radius, color, thickness=1, glow=5, alpha=0.20):
        cv2.circle(frame, center, radius, color, thickness, cv2.LINE_AA)

    def _glow_ellipse(self, frame, center, axes, angle, start, end, color,
                      thickness=1, glow=5, alpha=0.22):
        cv2.ellipse(frame, center, axes, angle, start, end, color,
                    thickness, cv2.LINE_AA)

    # ══════════════════════════════════════════════════════════════
    #  PUBLIC
    # ══════════════════════════════════════════════════════════════

    def draw(self, frame, bboxes, fps, threat_info, decision_maker, fire_status=None, motor_info=None):
        """매 프레임 호출 — 전체 UI 렌더링"""
        self._ensure_window()

        h, w = frame.shape[:2]
        sys_state = self.state.system_state
        if getattr(config, "UI_MINIMAL", True):
            self._draw_minimal_overlay(frame, bboxes, fps, sys_state, motor_info)
            full = self._compose_minimal_output(frame)
            cv2.imshow(self.window_name, full)
            return full

        display_w, display_h = self._target_display_size(w, h)
        self._panel_w = self._panel_width_for(display_w)
        self._camera_display_w = max(1, display_w - self._panel_w)
        self._frame_w = self._camera_display_w   # 버튼 좌표 계산용
        theme = STATE_THEME[sys_state]

        self._draw_camera_overlay(
            frame, bboxes, fps, threat_info, sys_state, theme,
            fire_status, motor_info,
        )

        # 2. 우측 패널
        panel = self._build_panel(display_h, threat_info, sys_state, theme, decision_maker)
        self._last_panel = panel.copy()

        # 3. 합성
        full = self._compose_output(frame, panel)
        cv2.imshow(self.window_name, full)
        return full

    def draw_light(self, frame, bboxes, fps, threat_info, fire_status=None, motor_info=None):
        """Show latest camera with full camera overlay, reusing the side panel."""
        self._ensure_window()
        h, w = frame.shape[:2]
        sys_state = self.state.system_state
        if getattr(config, "UI_MINIMAL", True):
            self._draw_minimal_overlay(frame, bboxes, fps, sys_state, motor_info)
            full = self._compose_minimal_output(frame)
            cv2.imshow(self.window_name, full)
            return full

        display_w, display_h = self._target_display_size(w, h)
        self._panel_w = self._panel_width_for(display_w)
        self._camera_display_w = max(1, display_w - self._panel_w)
        self._frame_w = self._camera_display_w
        theme = STATE_THEME[sys_state]

        self._draw_camera_overlay(
            frame, bboxes, fps, threat_info, sys_state, theme,
            fire_status, motor_info,
        )

        panel = self._last_panel
        if panel is None or panel.shape[0] != display_h or panel.shape[1] != self._panel_w:
            panel = np.zeros((display_h, self._panel_w, 3), dtype=np.uint8)
            panel[:] = BG_DARK
        full = self._compose_output(frame, panel)
        cv2.imshow(self.window_name, full)
        return full

    def _compose_minimal_output(self, camera_frame):
        frame_h, frame_w = camera_frame.shape[:2]
        if not config.UI_FULLSCREEN:
            self._display_size = (frame_w, frame_h)
            self._camera_display_w = frame_w
            self._frame_w = frame_w
            return camera_frame
        if self._screen_size is None:
            self._screen_size = self._detect_screen_size()
        if self._screen_size is None:
            self._display_size = (frame_w, frame_h)
            self._camera_display_w = frame_w
            self._frame_w = frame_w
            return camera_frame
        display_w, display_h = self._screen_size
        self._display_size = (display_w, display_h)
        self._camera_display_w = display_w
        self._frame_w = display_w
        return self._resize_cover(camera_frame, display_w, display_h)

    def _draw_minimal_overlay(self, frame, bboxes, fps, sys_state, motor_info=None):
        h, w = frame.shape[:2]
        target = next((b for b in bboxes if b.get("is_target", False)), None)
        theme = STATE_THEME[sys_state]
        status = "DRONE DETECTED" if target else "SEARCHING"
        color = HUD_GREEN if target else HUD_DIM
        if sys_state in (SystemState.TRACKING, SystemState.LOCKED, SystemState.ENGAGED):
            status = "DRONE TRACKING" if sys_state != SystemState.LOCKED else "DRONE LOCKED"
            color = HUD_RED if sys_state == SystemState.LOCKED else HUD_CYAN

        self._draw_border(frame, {"color": color, "blink": False})
        self._draw_crosshair(frame, w, h, sys_state, motor_info)
        self._draw_minimal_bboxes(frame, bboxes)
        self._draw_minimal_status(frame, status, color, fps, motor_info, target)

    def _draw_minimal_bboxes(self, frame, bboxes):
        for b in bboxes:
            if not b.get("is_target", False):
                continue
            x1, y1, x2, y2 = [int(v) for v in b["box"]]
            conf = float(b.get("conf", 0.0))
            c = HUD_GREEN if conf >= 0.80 else HUD_AMBER if conf >= 0.65 else HUD_RED
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 0), 4, cv2.LINE_AA)
            cv2.rectangle(frame, (x1, y1), (x2, y2), c, 2, cv2.LINE_AA)
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
            cv2.line(frame, (cx - 10, cy), (cx + 10, cy), c, 1, cv2.LINE_AA)
            cv2.line(frame, (cx, cy - 10), (cx, cy + 10), c, 1, cv2.LINE_AA)
            label = f"DRONE {conf:.0%}"
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
            ly = max(24, y1 - 8)
            cv2.rectangle(frame, (x1, ly - th - 8), (x1 + tw + 12, ly + 4), (0, 0, 0), -1)
            cv2.putText(frame, label, (x1 + 6, ly),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, c, 1, cv2.LINE_AA)

    def _draw_minimal_status(self, frame, status, color, fps, motor_info=None, target=None):
        h, w = frame.shape[:2]
        cv2.rectangle(frame, (0, 0), (w, 34), (0, 0, 0), -1)
        cv2.putText(frame, "CCTV", (10, 23), cv2.FONT_HERSHEY_SIMPLEX,
                    0.62, HUD_WHITE, 1, cv2.LINE_AA)
        cv2.circle(frame, (88, 18), 6, color, -1, cv2.LINE_AA)
        cv2.putText(frame, status, (104, 23), cv2.FONT_HERSHEY_SIMPLEX,
                    0.58, color, 1, cv2.LINE_AA)

        right = f"FPS {int(fps)}"
        if motor_info:
            src = str(motor_info.get("src", "") or "")
            usb = int(motor_info.get("usb_ok") or 0)
            reply = str(motor_info.get("fpga_reply", "") or "")
            if reply.startswith("SKIP"):
                right += f"  {reply[:18]}"
            elif src:
                right += f"  {src.upper()}"
            right += f"  USB {usb}"
        (tw, _), _ = cv2.getTextSize(right, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
        cv2.putText(frame, right, (max(8, w - tw - 12), 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, HUD_WHITE, 1, cv2.LINE_AA)

        if target:
            x1, y1, x2, y2 = [int(v) for v in target["box"]]
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
            bottom = f"bbox center=({cx},{cy}) size={x2-x1}x{y2-y1}"
            cv2.rectangle(frame, (0, h - 28), (w, h), (0, 0, 0), -1)
            cv2.putText(frame, bottom, (10, h - 9),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.48, HUD_WHITE, 1, cv2.LINE_AA)

    def _draw_camera_overlay(self, frame, bboxes, fps, threat_info, sys_state,
                             theme, fire_status=None, motor_info=None):
        """Draw a minimal camera overlay focused on the aim point."""
        h, w = frame.shape[:2]
        self._draw_border(frame, theme)
        self._draw_crosshair(frame, w, h, sys_state, motor_info)
        self._draw_bboxes(frame, bboxes, threat_info, sys_state)
        self._draw_laser_spot(frame, fire_status)
        self._draw_hit_banner(frame, w, h, fire_status)
        self._draw_topbar(frame, fps, sys_state, theme)
        self._draw_bottom_status(frame, w, h, sys_state, threat_info, fire_status)

    # ══════════════════════════════════════════════════════════════
    #  HUD OVERLAYS (camera view)
    # ══════════════════════════════════════════════════════════════

    def _draw_border(self, frame, theme):
        h, w = frame.shape[:2]
        color = theme["color"]
        thick = 3 if theme["blink"] and int(time.time() * 4) % 2 == 0 else 1
        self._glow_line(frame, (0, 0), (w - 1, 0), color, thick, 5, 0.18)
        self._glow_line(frame, (w - 1, 0), (w - 1, h - 1), color, thick, 5, 0.18)
        self._glow_line(frame, (w - 1, h - 1), (0, h - 1), color, thick, 5, 0.18)
        self._glow_line(frame, (0, h - 1), (0, 0), color, thick, 5, 0.18)

    def _draw_corner_brackets(self, frame, w, h):
        c, L = HUD_CYAN, 30
        m = 15
        # 4 corners
        for (x, y, dx, dy) in [
            (m, m, L, L), (w-m, m, -L, L),
            (m, h-m, L, -L), (w-m, h-m, -L, -L)]:
            cv2.line(frame, (x, y), (x+dx, y), c, 1, cv2.LINE_AA)
            cv2.line(frame, (x, y), (x, y+dy), c, 1, cv2.LINE_AA)

    def _draw_crosshair(self, frame, w, h, sys_state, motor_info=None):
        cx, cy = w // 2, h // 2
        if motor_info:
            aim_x = motor_info.get("aim_cx")
            aim_y = motor_info.get("aim_cy")
            if aim_x is not None and aim_y is not None:
                try:
                    aim_x = int(aim_x)
                    aim_y = int(aim_y)
                    if 0 <= aim_x < w and 0 <= aim_y < h:
                        cx, cy = aim_x, aim_y
                except (TypeError, ValueError):
                    pass
        if sys_state in (SystemState.LOCKED, SystemState.ENGAGED):
            c = HUD_WHITE
            t = 2
        elif sys_state == SystemState.TRACKING:
            c = HUD_CYAN
            t = 1
        else:
            c = HUD_DIM
            t = 1

        gap = 18
        arm = 46
        cv2.line(frame, (cx-arm, cy), (cx-gap, cy), (0, 0, 0), t + 2, cv2.LINE_AA)
        cv2.line(frame, (cx+gap, cy), (cx+arm, cy), (0, 0, 0), t + 2, cv2.LINE_AA)
        cv2.line(frame, (cx, cy-arm), (cx, cy-gap), (0, 0, 0), t + 2, cv2.LINE_AA)
        cv2.line(frame, (cx, cy+gap), (cx, cy+arm), (0, 0, 0), t + 2, cv2.LINE_AA)
        cv2.line(frame, (cx-arm, cy), (cx-gap, cy), c, t, cv2.LINE_AA)
        cv2.line(frame, (cx+gap, cy), (cx+arm, cy), c, t, cv2.LINE_AA)
        cv2.line(frame, (cx, cy-arm), (cx, cy-gap), c, t, cv2.LINE_AA)
        cv2.line(frame, (cx, cy+gap), (cx, cy+arm), c, t, cv2.LINE_AA)
        cv2.circle(frame, (cx, cy), 3, HUD_RED, -1, cv2.LINE_AA)

    def _draw_bboxes(self, frame, bboxes, threat_info, sys_state):
        for b in bboxes:
            x1, y1, x2, y2 = b["box"]
            is_target = b.get("is_target", False)

            if is_target:
                tl = threat_info.get("threat_level", "GREEN") if threat_info else "GREEN"
                aim = threat_info.get("aim_accuracy", 0.0) if threat_info else 0.0
                if sys_state in (SystemState.LOCKED, SystemState.ENGAGED) or aim >= 82:
                    c = HUD_RED
                elif aim >= 62:
                    c = HUD_AMBER
                else:
                    c = THREAT_COLORS.get(tl, HUD_CYAN)

                bw, bh = x2 - x1, y2 - y1
                cx_b, cy_b = (x1+x2)//2, (y1+y2)//2
                half = max(bw, bh) // 2
                lock_factor = max(0.0, min(1.0, aim / 100.0))
                pulse = (math.sin(time.time() * 7.0) + 1.0) * 0.5
                settle = 1.0 - lock_factor
                bracket_pad = int(8 + 24 * settle + 5 * pulse)
                left = max(0, x1 - bracket_pad)
                right = min(frame.shape[1] - 1, x2 + bracket_pad)
                top = max(0, y1 - bracket_pad)
                bottom = min(frame.shape[0] - 1, y2 + bracket_pad)
                L = max(14, min(34, int(max(bw, bh) * 0.28)))
                thick = 2 if sys_state not in (SystemState.LOCKED, SystemState.ENGAGED) else 3

                corners = [
                    (left, top, 1, 1),
                    (right, top, -1, 1),
                    (left, bottom, 1, -1),
                    (right, bottom, -1, -1),
                ]
                for px, py, dx, dy in corners:
                    cv2.line(frame, (px, py), (px + L * dx, py), c, thick, cv2.LINE_AA)
                    cv2.line(frame, (px, py), (px, py + L * dy), c, thick, cv2.LINE_AA)

                rail_gap = 6
                cv2.line(frame, (left + rail_gap, top - 5), (right - rail_gap, top - 5),
                         HUD_DIM, 1, cv2.LINE_AA)
                cv2.line(frame, (left + rail_gap, bottom + 5), (right - rail_gap, bottom + 5),
                         HUD_DIM, 1, cv2.LINE_AA)

                ring_base = max(18, min(80, int(half + 12 + 18 * settle)))
                if sys_state in (SystemState.LOCKED, SystemState.ENGAGED):
                    ring_base += int(4 * pulse)
                angle = (time.time() * 120.0) % 360.0
                cv2.ellipse(frame, (cx_b, cy_b), (ring_base, ring_base),
                            angle, 20, 150, c, 1, cv2.LINE_AA)
                cv2.ellipse(frame, (cx_b, cy_b), (ring_base + 8, ring_base + 8),
                            -angle * 0.7, 210, 340, HUD_GOLD, 1, cv2.LINE_AA)
                if aim >= 70:
                    inner_r = max(8, int(ring_base * 0.42))
                    cv2.ellipse(frame, (cx_b, cy_b), (inner_r, inner_r),
                                -angle, 45, 135, HUD_WHITE, 1, cv2.LINE_AA)

                tick_len = 8 if sys_state not in (SystemState.LOCKED, SystemState.ENGAGED) else 12
                cv2.line(frame, (cx_b - tick_len, cy_b), (cx_b - 3, cy_b), c, 1, cv2.LINE_AA)
                cv2.line(frame, (cx_b + 3, cy_b), (cx_b + tick_len, cy_b), c, 1, cv2.LINE_AA)
                cv2.line(frame, (cx_b, cy_b - tick_len), (cx_b, cy_b - 3), c, 1, cv2.LINE_AA)
                cv2.line(frame, (cx_b, cy_b + 3), (cx_b, cy_b + tick_len), c, 1, cv2.LINE_AA)

                sweep = math.radians(angle)
                s1 = (int(cx_b + math.cos(sweep) * (ring_base - 6)),
                      int(cy_b + math.sin(sweep) * (ring_base - 6)))
                s2 = (int(cx_b + math.cos(sweep) * (ring_base + 18)),
                      int(cy_b + math.sin(sweep) * (ring_base + 18)))
                cv2.line(frame, s1, s2, HUD_WHITE, 1, cv2.LINE_AA)

                # ID + Conf label
                conf = b.get('conf', 0)
                label = f"TGT-{b['id']:03d} {conf:.0%} AIM {aim:.0f}%"
                label_y = max(18, top - 10)
                cv2.rectangle(frame, (left, label_y - 15),
                              (min(frame.shape[1] - 1, left + 190), label_y + 4),
                              (25, 20, 12), -1)
                cv2.rectangle(frame, (left, label_y - 15),
                              (min(frame.shape[1] - 1, left + 190), label_y + 4),
                              c, 1)
                cv2.putText(frame, label, (left + 6, label_y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.42, c, 1, cv2.LINE_AA)
            else:
                cv2.rectangle(frame, (x1, y1), (x2, y2), HUD_DIM, 1)
                conf = b.get('conf', 0)
                cv2.putText(frame, f"OBJ-{b['id']} {conf:.0%}", (x1, max(12, y1-5)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, HUD_DIM, 1, cv2.LINE_AA)

    def _draw_trail(self, frame, threat_info):
        if not threat_info:
            return
        trail = threat_info.get("trajectory", [])
        if len(trail) < 2:
            return
        pts = [(int(t[0]), int(t[1])) for t in trail]
        for i in range(1, len(pts)):
            alpha = i / len(pts)
            c = (int(50*alpha), int(200*alpha), int(60*alpha))
            thick = max(1, int(alpha * 2))
            cv2.line(frame, pts[i-1], pts[i], c, thick, cv2.LINE_AA)

    def _draw_prediction_line(self, frame, threat_info):
        if not threat_info:
            return
        trail = threat_info.get("trajectory", [])
        pred  = threat_info.get("predicted_pos")
        if not trail or not pred:
            return
        last = (int(trail[-1][0]), int(trail[-1][1]))
        pred_pt = (int(pred[0]), int(pred[1]))
        # dashed prediction line
        cv2.line(frame, last, pred_pt, HUD_AMBER, 1, cv2.LINE_AA)
        cv2.circle(frame, pred_pt, 4, HUD_AMBER, 1, cv2.LINE_AA)
        cv2.putText(frame, "PRED", (pred_pt[0]+6, pred_pt[1]-4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.3, HUD_AMBER, 1, cv2.LINE_AA)

    def _draw_hit_banner(self, frame, w, h, fire_status):
        if not fire_status or not fire_status.get("hit_confirmed"):
            return

        label = str(fire_status.get("hit_label") or "LASER HIT")
        font = cv2.FONT_HERSHEY_DUPLEX
        scale = max(2.2, min(4.8, w / 300.0))
        thickness = max(5, int(scale * 2))
        (tw, th), base = cv2.getTextSize(label, font, scale, thickness)
        cx = w // 2
        cy = min(h - 70, max(h // 2 + 120, int(h * 0.68)))
        pad_x, pad_y = 54, 34
        x1 = max(0, cx - tw // 2 - pad_x)
        y1 = max(0, cy - th // 2 - pad_y)
        x2 = min(w - 1, cx + tw // 2 + pad_x)
        y2 = min(h - 1, cy + th // 2 + pad_y + base)

        overlay = frame.copy()
        cv2.rectangle(overlay, (x1, y1), (x2, y2), (0, 70, 0), -1)
        cv2.addWeighted(overlay, 0.78, frame, 0.22, 0, frame)
        cv2.rectangle(frame, (x1, y1), (x2, y2), HUD_GREEN, 4, cv2.LINE_AA)
        cv2.putText(frame, label, (cx - tw // 2, cy + th // 2),
                    font, scale, (0, 0, 0), thickness + 3, cv2.LINE_AA)
        cv2.putText(frame, label, (cx - tw // 2, cy + th // 2),
                    font, scale, HUD_GREEN, thickness, cv2.LINE_AA)

    def _draw_laser_spot(self, frame, fire_status):
        if not fire_status:
            return
        spot = fire_status.get("spot_center")
        h, w = frame.shape[:2]
        if not spot and fire_status.get("active"):
            spot = (w // 2, h // 2)
        if not spot:
            return
        try:
            sx, sy = int(spot[0]), int(spot[1])
        except Exception:
            return
        if not (0 <= sx < w and 0 <= sy < h):
            return

        area = int(fire_status.get("spot_area") or 0)
        hit_confirmed = bool(fire_status.get("hit_confirmed"))
        ring_color = HUD_GREEN if hit_confirmed else HUD_RED
        cv2.circle(frame, (sx, sy), 15, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.circle(frame, (sx, sy), 15, ring_color, 2, cv2.LINE_AA)
        cv2.circle(frame, (sx, sy), 7, (255, 255, 255), -1, cv2.LINE_AA)
        cv2.circle(frame, (sx, sy), 5, HUD_RED, -1, cv2.LINE_AA)
        label = "LASER"
        if area > 0:
            label += f" A{area}"
        tx = min(max(8, sx + 22), max(8, w - 100))
        ty = min(max(22, sy - 18), h - 12)
        cv2.putText(frame, label, (tx, ty), cv2.FONT_HERSHEY_SIMPLEX,
                    0.45, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(frame, label, (tx, ty), cv2.FONT_HERSHEY_SIMPLEX,
                    0.45, ring_color, 1, cv2.LINE_AA)

    def _pan_tick_to_deg(self, pan_tick):
        pan_center = (config.PAN_MAX + config.PAN_MIN) / 2.0
        pan_span = max(1.0, float(config.PAN_MAX - config.PAN_MIN))
        pan_dir = float(config.PAN_DIR) if config.PAN_DIR else 1.0
        deg = ((float(pan_tick) - pan_center) * pan_dir / pan_span) * 360.0
        return ((deg + 180.0) % 360.0) - 180.0

    def _radar_point(self, rx, ry, radius, angle_deg):
        angle = math.radians(float(angle_deg) % 360.0)
        return (
            int(rx - math.sin(angle) * radius),
            int(ry - math.cos(angle) * radius),
        )

    def _draw_radar(self, frame, bboxes, w, h, motor_info=None):
        """미니 레이더 (좌하단): 북쪽 0도 기준 pan 방향 + 소리 방향."""
        rr = 64
        rx, ry = 84, h - 84

        cv2.circle(frame, (rx, ry), rr + 12, (12, 10, 8), -1, cv2.LINE_AA)
        cv2.circle(frame, (rx, ry), rr + 7, (28, 20, 10), 2, cv2.LINE_AA)

        # rings & crosshairs
        cv2.circle(frame, (rx, ry), rr, HUD_CYAN, 1, cv2.LINE_AA)
        cv2.circle(frame, (rx, ry), rr * 2 // 3, HUD_DIM, 1, cv2.LINE_AA)
        cv2.circle(frame, (rx, ry), rr // 3, HUD_DIM, 1, cv2.LINE_AA)
        cv2.line(frame, (rx-rr, ry), (rx+rr, ry), HUD_DIM, 1, cv2.LINE_AA)
        cv2.line(frame, (rx, ry-rr), (rx, ry+rr), HUD_DIM, 1, cv2.LINE_AA)
        for a in range(0, 360, 45):
            outer = rr
            inner = rr - 10
            p1 = self._radar_point(rx, ry, outer, a)
            p2 = self._radar_point(rx, ry, inner, a)
            cv2.line(frame, p1, p2, HUD_CYAN, 1, cv2.LINE_AA)

        # CCTV 위치 (정중앙)
        cv2.circle(frame, (rx, ry), 4, HUD_GOLD, 1, cv2.LINE_AA)
        cv2.circle(frame, (rx, ry), 2, HUD_WHITE, -1, cv2.LINE_AA)

        # sweep line (rotating)
        sweep_angle = (time.time() * 115) % 360
        sx, sy = self._radar_point(rx, ry, rr, sweep_angle)
        
        cv2.line(frame, (rx, ry), (sx, sy), HUD_GREEN, 1, cv2.LINE_AA)

        with self.state.lock:
            audio_doa = self.state.audio_doa
            audio_section = self.state.audio_section
            audio_detected = self.state.audio_detected
            audio_updated = self.state.audio_updated_at

        audio_age = time.time() - audio_updated if audio_updated else 999.0
        if audio_doa is not None and audio_age < 5.0:
            doa = float(audio_doa) % 360.0
            audio_len = rr - 12
            ax, ay = self._radar_point(rx, ry, audio_len, doa)
            audio_color = HUD_RED if audio_detected and audio_age < 2.0 else HUD_AMBER
            cv2.arrowedLine(frame, (rx, ry), (ax, ay), audio_color, 2,
                            cv2.LINE_AA, tipLength=0.22)
            cv2.circle(frame, (ax, ay), 5, audio_color, 1, cv2.LINE_AA)
            dir_label = {
                1: "F",
                2: "R",
                3: "B",
                4: "L",
            }.get(audio_section, "")
            cv2.putText(frame, f"AUD {doa:.0f} {dir_label}", (rx - 48, ry + rr + 16),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, audio_color, 1, cv2.LINE_AA)

        target_visible = any(b.get("is_target", False) for b in bboxes)
        if target_visible and motor_info and motor_info.get("pan") is not None:
            pan_deg = self._pan_tick_to_deg(motor_info["pan"])
            pan_label = pan_deg % 360.0
            nx, ny = self._radar_point(rx, ry, rr - 24, pan_label)
            pulse = int(abs(math.sin(time.time() * 10)) * 4)
            cv2.line(frame, (rx, ry), (nx, ny), HUD_RED, 1, cv2.LINE_AA)
            cv2.circle(frame, (nx, ny), 6 + pulse, HUD_RED, 1, cv2.LINE_AA)
            cv2.circle(frame, (nx, ny), 3, HUD_RED, -1, cv2.LINE_AA)
            cv2.putText(frame, f"TGT {pan_label:.0f}", (nx + 8, ny + 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, HUD_RED, 1, cv2.LINE_AA)

        # 레이더 제목
        # 한글 폰트를 지원하지 않는 환경을 대비해 레이더는 간단한 영문 혼용 유지 (VISION 레이더)
        cv2.putText(frame, "RADAR", (rx - 18, ry - rr - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, HUD_CYAN, 1, cv2.LINE_AA)

    def _draw_topbar(self, frame, fps, sys_state, theme):
        w = frame.shape[1]
        cv2.rectangle(frame, (0, 0), (w - 1, 30), (22, 16, 8), -1)
        cv2.line(frame, (0, 29), (w - 1, 29), HUD_CYAN, 1, cv2.LINE_AA)
        cv2.line(frame, (0, 4), (180, 4), HUD_GOLD, 2, cv2.LINE_AA)
        cv2.line(frame, (280, 4), (500, 4), HUD_CYAN, 1, cv2.LINE_AA)
        cv2.line(frame, (w - 300, 4), (w - 1, 4), HUD_RED if sys_state == SystemState.ENGAGED else HUD_DIM, 2, cv2.LINE_AA)
        cv2.putText(frame, "ANTI-DRONE DEFENSE HUD", (8, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, HUD_CYAN, 1, cv2.LINE_AA)
        cv2.putText(frame, f"FPS:{fps}", (520, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, HUD_GREEN, 1, cv2.LINE_AA)
        ts = time.strftime("%H:%M:%S")
        cv2.putText(frame, ts, (585, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.36, HUD_GOLD, 1, cv2.LINE_AA)

    def _draw_audio_status(self, frame):
        with self.state.lock:
            status = self.state.audio_status
            score = self.state.audio_score
            rms = self.state.audio_rms
            doa = self.state.audio_doa
            section = self.state.audio_section
            detected = self.state.audio_detected
            updated = self.state.audio_updated_at

        age = time.time() - updated if updated else 999.0
        color = HUD_RED if detected and age < 2.0 else HUD_GREEN
        label = status if age < 3.0 else "오디오 대기"
        doa_text = "--" if doa is None else f"{doa:.0f}deg"
        sec_text = "-" if section is None else str(section)
        direction = {
            1: "FRONT",
            2: "RIGHT",
            3: "REAR",
            4: "LEFT",
        }.get(section, "--")

        x1, y1 = 8, 34
        x2, y2 = 315, 84
        cv2.rectangle(frame, (x1, y1), (x2, y2), (20, 15, 10), -1)
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 1)
        put_text(frame, label, (x1 + 8, y1 + 6), 0.50, color, 1)
        cv2.putText(frame, f"score={score:.2f} rms={rms:.3f} doa={doa_text} {direction} sec={sec_text}",
                    (x1 + 8, y1 + 42),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, HUD_WHITE, 1, cv2.LINE_AA)

    def _draw_bottom_status(self, frame, w, h, sys_state, threat_info, fire_status=None):
        cv2.rectangle(frame, (0, h-25), (w, h), (20, 15, 10), -1)
        theme = STATE_THEME[sys_state]
        lbl = theme["label"]
        color = theme["color"]
        hit_confirmed = bool(fire_status and fire_status.get("hit_confirmed"))
        if hit_confirmed:
            lbl = str(fire_status.get("hit_label") or "LASER HIT")
            color = HUD_GREEN
        if fire_status and fire_status.get("active") and not hit_confirmed:
            if fire_status.get("simulated"):
                if fire_status.get("sim_hit"):
                    lbl = "SIM HIT CONFIRMED"
                    color = HUD_GREEN
                else:
                    lbl = "NO-LASER SIM FIRE"
                    color = HUD_RED
            else:
                lbl = "PATTERN FIRE"
                color = HUD_AMBER
        if (not fire_status or not fire_status.get("active")):
            if theme["blink"] and int(time.time() * 3) % 2 == 0:
                lbl = ""
        cv2.putText(frame, lbl, (8, h-7),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)

        if threat_info:
            aim = threat_info.get("aim_accuracy", 0)
            cv2.putText(frame, f"AIM:{aim:.0f}%", (250, h-7),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, HUD_GREEN, 1, cv2.LINE_AA)
            spd = threat_info.get("speed_px_per_sec", 0)
            cv2.putText(frame, f"SPD:{spd:.0f}px/s", (380, h-7),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, HUD_AMBER, 1, cv2.LINE_AA)

    # ══════════════════════════════════════════════════════════════
    #  RIGHT PANEL
    # ══════════════════════════════════════════════════════════════

    def _build_panel(self, frame_h, threat_info, sys_state, theme, decision_maker):
        pw = self._panel_w
        panel = np.zeros((frame_h, pw, 3), dtype=np.uint8)
        panel[:] = BG_DARK
        self.ui_buttons.clear()
        y = 0

        # ── Header ──
        cv2.rectangle(panel, (0, 0), (pw, 38), (30, 22, 10), -1)
        cv2.line(panel, (0, 37), (pw, 37), HUD_GOLD, 2, cv2.LINE_AA)
        cv2.line(panel, (0, 3), (120, 3), HUD_RED if sys_state == SystemState.ENGAGED else HUD_CYAN, 2, cv2.LINE_AA)
        cv2.putText(panel, "TACTICAL CONTROL", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.54, HUD_CYAN, 1, cv2.LINE_AA)
        y = 42

        # ── System State ──
        cv2.putText(panel, "SYSTEM STATUS", (10, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, HUD_DIM, 1, cv2.LINE_AA)
        y += 20
        # LED dot
        dot_c = theme["color"]
        cv2.circle(panel, (18, y-4), 5, dot_c, -1, cv2.LINE_AA)
        cv2.putText(panel, theme["label"], (30, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, dot_c, 1, cv2.LINE_AA)
        y += 22

        with self.state.lock:
            audio_status = self.state.audio_status
            audio_score = self.state.audio_score
            audio_doa = self.state.audio_doa
            audio_detected = self.state.audio_detected
        a_color = HUD_RED if audio_detected else HUD_GREEN
        put_text(panel, "AUDIO", (10, y), 0.35, HUD_DIM, 1)
        y += 18
        put_text(panel, audio_status, (10, y), 0.42, a_color, 1)
        y += 18
        doa_text = "--" if audio_doa is None else f"{audio_doa:.0f}deg"
        cv2.putText(panel, f"score {audio_score:.2f}  doa {doa_text}", (10, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, HUD_WHITE, 1, cv2.LINE_AA)
        y += 18

        # ── Threat Level ──
        cv2.line(panel, (10, y), (pw-10, y), HUD_DIM, 1)
        y += 15
        cv2.putText(panel, "THREAT LEVEL", (10, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, HUD_DIM, 1, cv2.LINE_AA)
        y += 18
        tl = threat_info.get("threat_level", "GREEN") if threat_info else "GREEN"
        ts = threat_info.get("threat_score", 0.0) if threat_info else 0.0
        tc = THREAT_COLORS.get(tl, HUD_GREEN)
        # bar
        bar_w = int((pw - 40) * ts)
        cv2.rectangle(panel, (15, y), (pw-25, y+14), (40, 35, 25), -1)
        if bar_w > 0:
            cv2.rectangle(panel, (15, y), (15+bar_w, y+14), tc, -1)
        cv2.rectangle(panel, (15, y), (pw-25, y+14), tc, 1)
        cv2.putText(panel, tl, (pw-25-60, y+12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255,255,255), 1, cv2.LINE_AA)
        y += 28

        # ── Aim Accuracy ──
        cv2.putText(panel, "AIM ACCURACY", (10, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, HUD_DIM, 1, cv2.LINE_AA)
        y += 18
        aim = threat_info.get("aim_accuracy", 0.0) if threat_info else 0.0
        aim_c = HUD_GREEN if aim >= 80 else HUD_AMBER if aim >= 50 else HUD_RED
        bar_w = int((pw - 40) * aim / 100.0)
        cv2.rectangle(panel, (15, y), (pw-25, y+14), (40, 35, 25), -1)
        if bar_w > 0:
            cv2.rectangle(panel, (15, y), (15+bar_w, y+14), aim_c, -1)
        cv2.rectangle(panel, (15, y), (pw-25, y+14), aim_c, 1)
        cv2.putText(panel, f"{aim:.1f}%", (pw-25-55, y+12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255,255,255), 1, cv2.LINE_AA)
        y += 28

        # ── Engagement Probability ──
        cv2.putText(panel, "ENGAGE PROBABILITY", (10, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, HUD_DIM, 1, cv2.LINE_AA)
        y += 18
        ep = threat_info.get("engagement_prob", 0.0) if threat_info else 0.0
        ep_c = HUD_GREEN if ep >= 70 else HUD_AMBER if ep >= 40 else HUD_DIM
        bar_w = int((pw - 40) * ep / 100.0)
        cv2.rectangle(panel, (15, y), (pw-25, y+14), (40, 35, 25), -1)
        if bar_w > 0:
            cv2.rectangle(panel, (15, y), (15+bar_w, y+14), ep_c, -1)
        cv2.rectangle(panel, (15, y), (pw-25, y+14), ep_c, 1)
        cv2.putText(panel, f"{ep:.1f}%", (pw-25-55, y+12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255,255,255), 1, cv2.LINE_AA)
        y += 32

        # ── Engagement Stats ──
        cv2.line(panel, (10, y), (pw-10, y), HUD_DIM, 1)
        y += 15
        cv2.putText(panel, "ENGAGEMENT STATS", (10, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, HUD_DIM, 1, cv2.LINE_AA)
        y += 20
        st = self.state
        stats = [
            ("TRACKS",      st.total_tracks,       HUD_CYAN),
            ("ENGAGEMENTS", st.total_engagements,   HUD_AMBER),
            ("NEUTRALIZED", st.total_neutralized,   HUD_GREEN),
        ]
        for label, val, c in stats:
            cv2.putText(panel, f"  {label}:", (10, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, HUD_DIM, 1, cv2.LINE_AA)
            cv2.putText(panel, str(val), (pw-50, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, c, 1, cv2.LINE_AA)
            y += 18

        # ── Buttons ──
        y += 8
        cv2.line(panel, (10, y), (pw-10, y), HUD_DIM, 1)
        y += 12

        with self.state.lock:
            laser_armed = bool(getattr(self.state, "laser_armed", False))

        arm_color = HUD_RED if laser_armed else HUD_AMBER
        arm_fill = (45, 20, 20) if laser_armed else (35, 35, 20)
        cv2.rectangle(panel, (15, y), (pw-15, y+30), arm_fill, -1)
        cv2.rectangle(panel, (15, y), (pw-15, y+30), arm_color, 1)
        arm_label = "ARMED" if laser_armed else "ARM SYSTEM"
        cv2.putText(panel, arm_label, (86 if laser_armed else 72, y+21),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, arm_color, 1, cv2.LINE_AA)
        self.ui_buttons["ARM"] = (self._frame_w+15, y, self._frame_w+pw-15, y+30)
        y += 38

        laser_fill = (55, 10, 10) if laser_armed else (35, 35, 35)
        laser_color = HUD_RED if laser_armed else HUD_DIM
        cv2.rectangle(panel, (15, y), (pw-15, y+30), laser_fill, -1)
        cv2.rectangle(panel, (15, y), (pw-15, y+30), laser_color, 1)
        cv2.putText(panel, "LASER FIRE", (82, y+21),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, laser_color, 1, cv2.LINE_AA)
        self.ui_buttons["LASER"] = (self._frame_w+15, y, self._frame_w+pw-15, y+30)
        y += 40

        # RESET button (목표물 초기화 버튼 복구)
        cv2.rectangle(panel, (15, y), (pw-15, y+30), (50, 50, 50), -1)
        cv2.rectangle(panel, (15, y), (pw-15, y+30), HUD_DIM, 1)
        cv2.putText(panel, "SYSTEM RESET", (80, y+21),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, HUD_WHITE, 1, cv2.LINE_AA)
        self.ui_buttons["RESET"] = (self._frame_w+15, y, self._frame_w+pw-15, y+30)
        y += 40

        # ── Event Log ──
        cv2.line(panel, (10, y), (pw-10, y), HUD_DIM, 1)
        y += 12
        cv2.putText(panel, "EVENT LOG", (10, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, HUD_DIM, 1, cv2.LINE_AA)
        y += 16
        with self.state.lock:
            events = self.state.event_log[-6:]
        for ts_val, txt in events:
            t_str = time.strftime("%H:%M:%S", time.localtime(ts_val))
            put_text(panel, f"{t_str} {txt}", (10, y - 10), 0.30, HUD_DIM, 1)
            y += 14

        return panel

    # ══════════════════════════════════════════════════════════════
    #  MOUSE CALLBACK
    # ══════════════════════════════════════════════════════════════

    def _mouse_cb(self, event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        for btn_id, (bx1, by1, bx2, by2) in self.ui_buttons.items():
            if bx1 <= x <= bx2 and by1 <= y <= by2:
                self._on_button(btn_id)
                break

    def _on_button(self, btn_id):
        # 버튼 동작은 main.py에서 콜백으로 처리
        self._last_clicked = btn_id

    def get_clicked_button(self):
        btn = getattr(self, '_last_clicked', None)
        self._last_clicked = None
        return btn
