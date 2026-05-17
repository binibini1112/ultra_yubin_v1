"""
decision.py — 드론 방공 전용 타겟 선택 + 교전 프로토콜
======================================================
- 타겟 선택: confidence 최대 → Sticky (거리 기반)
- Re-ID 완전 제거
- 6단계 교전 상태 전환 로직 내장
"""
import time
import numpy as np
from src.core.state import SharedState, SystemState
import src.config as config


class DecisionMaker:
    """드론 방공 전용 추적 결정기"""

    def __init__(self, state: SharedState):
        self.state = state
        self._last_target_bbox    = None
        self._drone_absent_frames = 0
        self.DRONE_ABSENT_RESET   = 90
        self.last_reject_reason   = ""

    def _box_center(self, box):
        return (box[0] + box[2]) / 2, (box[1] + box[3]) / 2

    def _box_area(self, box):
        return (box[2] - box[0]) * (box[3] - box[1])

    def _center_dist(self, box_a, box_b):
        cx_a, cy_a = self._box_center(box_a)
        cx_b, cy_b = self._box_center(box_b)
        return float(np.sqrt((cx_a - cx_b)**2 + (cy_a - cy_b)**2))

    def process_tracking(self, frame, tracker_res):
        """YOLO 결과 → 타겟 드론 선택 + bbox 리스트 반환"""
        visible_ids    = set()
        target_visible = False
        bboxes         = []
        self.last_reject_reason = ""

        if tracker_res.boxes is None or len(tracker_res.boxes) == 0:
            self._drone_absent_frames += 1
            self.last_reject_reason = "no_boxes"
            if self._drone_absent_frames > self.DRONE_ABSENT_RESET:
                self._last_target_bbox    = None
                self._drone_absent_frames = 0
            return visible_ids, target_visible, bboxes

        boxes = tracker_res.boxes.xyxy.cpu().numpy()
        yids  = (tracker_res.boxes.id.int().cpu().numpy()
                 if tracker_res.boxes.id is not None else None)
        # ★ confidence 점수 추출
        confs = tracker_res.boxes.conf.cpu().numpy() if tracker_res.boxes.conf is not None else None

        # ── 타겟 선택 ──
        best_idx = -1
        candidates = []

        for i, box in enumerate(boxes):
            if np.isnan(box).any():
                continue
            c = float(confs[i]) if confs is not None else 0.5
            area = self._box_area(box)
            candidates.append((i, box, c, area))

        if not candidates:
            self._drone_absent_frames += 1
            self.last_reject_reason = "no_valid_boxes"
            if self._drone_absent_frames > self.DRONE_ABSENT_RESET:
                self._last_target_bbox    = None
                self._drone_absent_frames = 0
            return visible_ids, target_visible, bboxes

        if self._last_target_bbox is None:
            # ★ 최초: confidence가 가장 높은 박스 = 가장 확실한 드론
            # 구석 마진 제한 해제: 화면 어느 곳에서든 나타나면 즉시 타겟으로 선택
            best_conf = -1.0
            
            for i, box, c, _area in candidates:
                if c > best_conf:
                    best_conf = c
                    best_idx  = i
        else:
            # 이후: 이전 위치에 가장 가까운 박스 (Sticky)
            best_dist = float("inf")
            best_conf = 0.0
            for i, box, c, _area in candidates:
                dist = self._center_dist(box, self._last_target_bbox)
                if dist < best_dist:
                    best_dist = dist
                    best_conf = c
                    best_idx  = i

            sticky_max_dist = float(config.TRACK_STICKY_MAX_DIST_PX)
            if best_dist > sticky_max_dist:
                self._drone_absent_frames += 1
                self.last_reject_reason = f"sticky_dist:{best_dist:.0f}"
                if self._drone_absent_frames > self.DRONE_ABSENT_RESET:
                    self._last_target_bbox    = None
                    self._drone_absent_frames = 0
                return visible_ids, target_visible, bboxes

        # ── 결과 빌드 ──
        for i, box in enumerate(boxes):
            if np.isnan(box).any():
                continue
            x1, y1, x2, y2 = map(int, box)
            yid = int(yids[i]) if yids is not None else i
            conf = float(confs[i]) if confs is not None else 0.0
            is_target = (i == best_idx)

            if is_target:
                target_visible         = True
                self._last_target_bbox = [x1, y1, x2, y2]
                self._drone_absent_frames = 0

            visible_ids.add(yid)
            
            # ★ 타겟이 한 번 지정되면 다른 타겟은 일체 잡지 않고 오직 타겟만 계속 추적
            if self._last_target_bbox is None or is_target:
                bboxes.append({
                    "id": yid,
                    "box": [x1, y1, x2, y2],
                    "is_target": is_target,
                    "conf": conf,           # ★ confidence 포함
                })

        if not target_visible:
            self._drone_absent_frames += 1
            if self._drone_absent_frames > self.DRONE_ABSENT_RESET:
                self._last_target_bbox    = None
                self._drone_absent_frames = 0

        return visible_ids, target_visible, bboxes

    def update_engagement_state(self, target_visible, threat_info):
        """6단계 교전 프로토콜 상태 전환"""
        st = self.state
        current = st.system_state

        if current == SystemState.SCANNING:
            if target_visible:
                st.detect_count += 1
                if st.detect_count >= config.DETECT_CONFIRM_FRAMES:
                    st.transition(SystemState.DETECTED)
                    st.total_tracks += 1
                    st.detect_count = 0
            else:
                st.detect_count = 0

        elif current == SystemState.DETECTED:
            if target_visible:
                st.transition(SystemState.TRACKING)
                st.drone_lost_since = None
            else:
                self._check_lost(st)

        elif current == SystemState.TRACKING:
            if target_visible:
                st.drone_lost_since = None
                if threat_info and threat_info["lock_duration"] >= config.LOCK_HOLD_SECONDS:
                    if threat_info["aim_accuracy"] >= config.LOCK_AIM_THRESHOLD:
                        st.transition(SystemState.LOCKED)
            else:
                self._check_lost(st)

        elif current == SystemState.LOCKED:
            if target_visible:
                st.drone_lost_since = None
                if threat_info and threat_info["aim_accuracy"] < config.LOCK_AIM_THRESHOLD * 0.7:
                    st.transition(SystemState.TRACKING)
            else:
                self._check_lost(st)

        elif current == SystemState.ENGAGED:
            # 발사 결과(피격/미스)는 main.py의 레이저 점 검출 루프가 결정한다.
            pass

        elif current == SystemState.NEUTRALIZED:
            if st.time_in_state() >= config.NEUTRALIZED_HOLD_SEC:
                st.transition(SystemState.SCANNING)
                self._last_target_bbox = None

    def _check_lost(self, st):
        if st.drone_lost_since is None:
            st.drone_lost_since = time.time()
        elif time.time() - st.drone_lost_since >= config.DRONE_LOST_RESET_SEC:
            st.transition(SystemState.SCANNING)
            st.drone_lost_since = None
            self._last_target_bbox = None

    def trigger_engage(self):
        if self.state.system_state == SystemState.LOCKED:
            self.state.transition(SystemState.ENGAGED)
            self.state.total_engagements += 1
            return True
        return False

    def reset(self):
        self._last_target_bbox = None
        self._drone_absent_frames = 0
        self.state.transition(SystemState.SCANNING)
