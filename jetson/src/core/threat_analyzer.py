"""
threat_analyzer.py — 실시간 드론 위협 분석 엔진
================================================
1. 칼만 필터 기반 위치/속도 추정 + 궤적 예측
2. 위협 등급 분류 (GREEN → YELLOW → ORANGE → RED)
3. 조준 안정도(Aim Accuracy) 계산
4. 교전 확률(Engagement Probability) 산출
"""
import time
import math
import numpy as np
from collections import deque


class SimpleKalmanFilter:
    """2D 위치+속도 칼만 필터 [x, y, vx, vy]"""

    def __init__(self):
        self.x = np.zeros(4)           # state: [x, y, vx, vy]
        self.P = np.eye(4) * 500       # covariance
        self.F = np.eye(4)             # state transition
        self.H = np.zeros((2, 4))      # observation matrix
        self.H[0, 0] = 1
        self.H[1, 1] = 1
        self.R = np.eye(2) * 10        # measurement noise
        self.Q = np.eye(4) * 0.5       # process noise
        self.Q[2, 2] = 2.0
        self.Q[3, 3] = 2.0
        self._initialized = False

    def update(self, cx, cy, dt=0.033):
        if not self._initialized:
            self.x[:2] = [cx, cy]
            self._initialized = True
            return
        # Update F with dt
        self.F[0, 2] = dt
        self.F[1, 3] = dt
        # Predict
        x_pred = self.F @ self.x
        P_pred = self.F @ self.P @ self.F.T + self.Q
        # Update
        z = np.array([cx, cy])
        y = z - self.H @ x_pred
        S = self.H @ P_pred @ self.H.T + self.R
        K = P_pred @ self.H.T @ np.linalg.inv(S)
        self.x = x_pred + K @ y
        self.P = (np.eye(4) - K @ self.H) @ P_pred

    def predict_future(self, steps=10, dt=0.033):
        """미래 위치 예측 (steps 프레임 후)"""
        future_x = self.x[0] + self.x[2] * dt * steps
        future_y = self.x[1] + self.x[3] * dt * steps
        return float(future_x), float(future_y)

    @property
    def velocity(self):
        return float(np.sqrt(self.x[2]**2 + self.x[3]**2))

    @property
    def position(self):
        return float(self.x[0]), float(self.x[1])


class ThreatAnalyzer:
    """실시간 드론 위협 분석 + 교전 확률 산출"""

    THREAT_LEVELS = ["GREEN", "YELLOW", "ORANGE", "RED"]

    def __init__(self, frame_w=640, frame_h=480):
        self.kf = SimpleKalmanFilter()
        self.frame_w = frame_w
        self.frame_h = frame_h
        self._center = (frame_w // 2, frame_h // 2)

        # History
        self._pos_history  = deque(maxlen=15)   # 0.5초 궤적 (화면 가림 방지)
        self._aim_errors   = deque(maxlen=30)   # 1초 조준 오차
        self._area_history = deque(maxlen=30)   # 1초 면적 변화
        self._last_time    = time.time()

        # Lock tracking
        self._lock_start   = None
        self._locked       = False

    def set_frame_size(self, frame_w, frame_h):
        frame_w = int(frame_w)
        frame_h = int(frame_h)
        if frame_w <= 0 or frame_h <= 0:
            return
        if frame_w == self.frame_w and frame_h == self.frame_h:
            return
        self.frame_w = frame_w
        self.frame_h = frame_h
        self._center = (frame_w // 2, frame_h // 2)
        self._aim_errors.clear()
        self._lock_start = None

    def update(self, bbox):
        """매 프레임: 드론 bbox로 위협 정보 갱신. bbox=[x1,y1,x2,y2]"""
        now = time.time()
        dt  = max(now - self._last_time, 0.001)
        self._last_time = now

        cx = (bbox[0] + bbox[2]) / 2.0
        cy = (bbox[1] + bbox[3]) / 2.0
        area = (bbox[2] - bbox[0]) * (bbox[3] - bbox[1])

        # 칼만 필터 업데이트
        self.kf.update(cx, cy, dt)

        # 히스토리 저장
        self._pos_history.append((cx, cy, now))
        self._area_history.append(area)

        # 조준 오차 (드론 중심 ↔ 화면 중심)
        aim_err = math.sqrt((cx - self._center[0])**2 + (cy - self._center[1])**2)
        self._aim_errors.append(aim_err)

        # 분석 결과 산출
        speed          = self.kf.velocity
        aim_accuracy   = self._calc_aim_accuracy()
        threat_level   = self._calc_threat_level(area, speed)
        engage_prob    = self._calc_engagement_prob(aim_accuracy)
        approach_rate  = self._calc_approach_rate()
        pred_pos       = self.kf.predict_future(steps=15)

        # Lock 판정 (조준 80%+ 유지)
        if aim_accuracy >= 80.0:
            if self._lock_start is None:
                self._lock_start = now
        else:
            self._lock_start = None

        return {
            "threat_level":     threat_level,
            "threat_score":     self._threat_to_score(threat_level),
            "aim_accuracy":     round(aim_accuracy, 1),
            "engagement_prob":  round(engage_prob, 1),
            "speed_px_per_sec": round(speed * 30, 1),  # px/frame → px/sec
            "predicted_pos":    pred_pos,
            "trajectory":       list(self._pos_history),
            "approach_rate":    round(approach_rate, 2),
            "drone_area":       area,
            "lock_duration":    (now - self._lock_start) if self._lock_start else 0.0,
        }

    def _calc_aim_accuracy(self) -> float:
        if len(self._aim_errors) < 3:
            return 0.0
        avg_err = sum(self._aim_errors) / len(self._aim_errors)
        max_err = math.sqrt(self._center[0]**2 + self._center[1]**2)
        acc = max(0, (1.0 - avg_err / max_err)) * 100
        return min(100.0, acc)

    def _calc_threat_level(self, area, speed) -> str:
        # 면적이 크면 = 가까이 있음 = 위험
        # 속도가 빠르면 = 기동 중 = 위험
        score = 0.0
        if area > 15000:
            score += 0.5
        elif area > 5000:
            score += 0.3
        elif area > 1000:
            score += 0.1

        if speed > 8:
            score += 0.3
        elif speed > 3:
            score += 0.15

        # 접근 중이면 추가 위험
        approach = self._calc_approach_rate()
        if approach > 0.5:   # 면적 증가율 = 접근 중
            score += 0.2

        if score >= 0.7:
            return "RED"
        elif score >= 0.45:
            return "ORANGE"
        elif score >= 0.2:
            return "YELLOW"
        return "GREEN"

    def _calc_approach_rate(self) -> float:
        if len(self._area_history) < 10:
            return 0.0
        recent = list(self._area_history)
        first_half = np.mean(recent[:len(recent)//2])
        second_half = np.mean(recent[len(recent)//2:])
        if first_half < 1:
            return 0.0
        return (second_half - first_half) / first_half

    def _calc_engagement_prob(self, aim_accuracy) -> float:
        # 조준 정확도 + 안정성(분산)으로 교전 확률 산출
        if len(self._aim_errors) < 5:
            return 0.0
        stability = 1.0 - min(1.0, np.std(list(self._aim_errors)) / 100.0)
        prob = (aim_accuracy / 100.0) * 0.6 + stability * 0.4
        return min(100.0, prob * 100)

    def _threat_to_score(self, level) -> float:
        return {"GREEN": 0.15, "YELLOW": 0.4, "ORANGE": 0.7, "RED": 0.95}.get(level, 0)

    def reset(self):
        self.kf = SimpleKalmanFilter()
        self._pos_history.clear()
        self._aim_errors.clear()
        self._area_history.clear()
        self._lock_start = None
