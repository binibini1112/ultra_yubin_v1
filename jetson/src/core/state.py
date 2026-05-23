"""
Anti-Drone Defense — 6단계 교전 상태 머신
==========================================
SCANNING → DETECTED → TRACKING → LOCKED → ENGAGED → NEUTRALIZED
                                                          ↓
                                                      SCANNING
"""
import time
from enum import Enum
from threading import Lock


class SystemState(Enum):
    SCANNING     = "SCANNING"       # 스캔 대기 (드론 미발견)
    DETECTED     = "DETECTED"       # 드론 최초 발견 (확인 중)
    TRACKING     = "TRACKING"       # 드론 추적 중 (조준 불안정)
    LOCKED       = "LOCKED"         # 조준 고정 (교전 가능)
    ENGAGED      = "ENGAGED"        # 교전 중 (레이저 발사)
    NEUTRALIZED  = "NEUTRALIZED"    # 무력화 완료


class SharedState:
    """스레드 안전한 시스템 공유 상태"""

    def __init__(self):
        self.lock = Lock()

        # ── System state ──────────────────────────────────────
        self.system_state: SystemState = SystemState.SCANNING
        self._prev_state:  SystemState = SystemState.SCANNING
        self._state_entered_at: float  = time.time()

        # ── Tracking ──────────────────────────────────────────
        self.drone_visible      = False
        self.drone_lost_since   = None    # 드론 소실 시각
        self.detect_count       = 0       # 연속 감지 프레임 수
        self.vision_updated_at  = 0.0

        # ── Engagement stats ──────────────────────────────────
        self.total_tracks       = 0
        self.total_engagements  = 0
        self.total_neutralized  = 0
        self.laser_armed        = False

        # ── Event log ─────────────────────────────────────────
        self.event_log = []   # [(timestamp, event_str)]

        # ── Audio trigger status ──────────────────────────────
        self.audio_status = "오디오 대기"
        self.audio_score = 0.0
        self.audio_rms = 0.0
        self.audio_doa = None
        self.audio_section = None
        self.audio_detected = False
        self.audio_updated_at = 0.0
        self.audio_last_detected_at = 0.0
        self.audio_last_detection_doa = None

    def transition(self, new_state: SystemState):
        if self.system_state == new_state:
            return
        elapsed = round(time.time() - self._state_entered_at, 1)
        old = self.system_state.value
        print(f"[STATE] {old} ──► {new_state.value}  ({elapsed}s)")
        with self.lock:
            self._prev_state       = self.system_state
            self.system_state      = new_state
            self._state_entered_at = time.time()
        self.log_event(new_state.value)

    def log_event(self, text: str):
        with self.lock:
            self.event_log.append((time.time(), text))
            if len(self.event_log) > 50:
                self.event_log = self.event_log[-50:]

    def update_audio_status(self, text, score, rms, doa, section, detected, add_log=False):
        with self.lock:
            self.audio_status = text
            self.audio_score = float(score)
            self.audio_rms = float(rms)
            self.audio_doa = None if doa is None else float(doa)
            self.audio_section = section
            self.audio_detected = bool(detected)
            self.audio_updated_at = time.time()
            if detected:
                self.audio_last_detected_at = self.audio_updated_at
                self.audio_last_detection_doa = self.audio_doa
            if add_log:
                self.event_log.append((time.time(), text))
                if len(self.event_log) > 50:
                    self.event_log = self.event_log[-50:]

    def update_vision_status(self, visible: bool):
        with self.lock:
            self.drone_visible = bool(visible)
            self.vision_updated_at = time.time()

    def is_vision_active(self, hold_sec=0.8) -> bool:
        with self.lock:
            return self.drone_visible and (time.time() - self.vision_updated_at <= hold_sec)

    def time_in_state(self) -> float:
        return time.time() - self._state_entered_at
