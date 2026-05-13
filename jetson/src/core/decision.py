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
        self._last_target_id      = None
        self._last_target_conf    = 0.0
        self._drone_absent_frames = 0
        self.DRONE_ABSENT_RESET   = 90
        self.last_reject_reason   = ""

    def _hold_last_target(self, visible_ids, bboxes):
        if self._last_target_bbox is None:
            return visible_ids, False, bboxes
        if self._drone_absent_frames > int(config.TRACK_HOLD_LAST_FRAMES):
            return visible_ids, False, bboxes
        yid = int(self._last_target_id) if self._last_target_id is not None else 0
        visible_ids.add(yid)
        bboxes.append({
            "id": yid,
            "box": list(map(int, self._last_target_bbox)),
            "is_target": True,
            "conf": float(self._last_target_conf),
            "held": True,
        })
        return visible_ids, True, bboxes

    def _box_center(self, box):
        return (box[0] + box[2]) / 2, (box[1] + box[3]) / 2

    def _box_area(self, box):
        return (box[2] - box[0]) * (box[3] - box[1])

    def _center_dist(self, box_a, box_b):
        cx_a, cy_a = self._box_center(box_a)
        cx_b, cy_b = self._box_center(box_b)
        return float(np.sqrt((cx_a - cx_b)**2 + (cy_a - cy_b)**2))

    def _candidate_ok(self, box, conf):
        w = float(box[2] - box[0])
        h = float(box[3] - box[1])
        if w <= 0.0 or h <= 0.0:
            return False, "bad_size"

        area = w * h
        aspect = w / h
        min_conf = float(config.TRACK_TARGET_MIN_CONF)
        min_area = float(config.TRACK_TARGET_MIN_AREA)
        min_w = float(config.TRACK_TARGET_MIN_W)
        min_h = float(config.TRACK_TARGET_MIN_H)
        min_aspect = float(config.TRACK_TARGET_MIN_ASPECT)
        max_aspect = float(config.TRACK_TARGET_MAX_ASPECT)
        if min_conf > 0.0 and conf < min_conf:
            return False, f"low_conf:{conf:.2f}"
        if min_area > 0.0 and area < min_area:
            return False, f"small_area:{area:.0f}"
        if (min_w > 0.0 and w < min_w) or (min_h > 0.0 and h < min_h):
            return False, f"small_box:{w:.0f}x{h:.0f}"
        if (min_aspect > 0.0 and aspect < min_aspect) or aspect > max_aspect:
            return False, f"bad_aspect:{aspect:.2f}"
        return True, ""

    def process_tracking(self, frame, tracker_res):
        """YOLO 결과 → 타겟 드론 선택 + bbox 리스트 반환"""
        visible_ids    = set()
        target_visible = False
        bboxes         = []
        self.last_reject_reason = ""

        if tracker_res.boxes is None or len(tracker_res.boxes) == 0:
            self._drone_absent_frames += 1
            self.last_reject_reason = "no_boxes"
            held = self._hold_last_target(visible_ids, bboxes)
            if held[1]:
                return held
            if self._drone_absent_frames > self.DRONE_ABSENT_RESET:
                self._last_target_bbox    = None
                self._last_target_id      = None
                self._last_target_conf    = 0.0
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
        candidate_indices = set()

        for i, box in enumerate(boxes):
            if np.isnan(box).any():
                continue
            c = float(confs[i]) if confs is not None else 0.5
            ok, reason = self._candidate_ok(box, c)
            if not ok:
                self.last_reject_reason = reason
                continue
            area = self._box_area(box)
            candidates.append((i, box, c, area))
            candidate_indices.add(i)

        if not candidates:
            self._drone_absent_frames += 1
            if not self.last_reject_reason:
                self.last_reject_reason = "no_valid_boxes"
            held = self._hold_last_target(visible_ids, bboxes)
            if held[1]:
                return held
            if self._drone_absent_frames > self.DRONE_ABSENT_RESET:
                self._last_target_bbox    = None
                self._last_target_id      = None
                self._last_target_conf    = 0.0
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
            # 이후: 같은 tracker id를 우선 유지하고, 없으면 이전 위치에 가장 가까운 박스.
            # 검출이 흔들릴 때 다른 물체로 순간 점프하면 팬틸트가 엉뚱한 곳을 따라가므로
            # 시각 모델의 target 선택만 안정화하고, goal 계산은 계속 PL이 맡는다.
            same_id_idx = -1
            same_id_dist = float("inf")
            if bool(config.TRACK_PREFER_SAME_ID) and self._last_target_id is not None and yids is not None:
                for i, box, _c, _area in candidates:
                    yid = int(yids[i])
                    if yid != self._last_target_id:
                        continue
                    dist = self._center_dist(box, self._last_target_bbox)
                    if dist < same_id_dist:
                        same_id_dist = dist
                        same_id_idx = i

            if same_id_idx >= 0 and same_id_dist <= float(config.TRACK_SAME_ID_MAX_DIST_PX):
                best_idx = same_id_idx
            else:
                # 같은 id를 잃었으면 이전 위치에 가장 가까운 박스 (Sticky)
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
                    if self._drone_absent_frames < int(config.TRACK_REACQUIRE_AFTER_ABSENT_FRAMES):
                        self.last_reject_reason = f"sticky_dist:{best_dist:.0f}"
                        held = self._hold_last_target(visible_ids, bboxes)
                        if held[1]:
                            return held
                        return visible_ids, target_visible, bboxes
                    if best_conf < float(config.TRACK_REACQUIRE_MIN_CONF):
                        self.last_reject_reason = f"reacquire_conf:{best_conf:.2f}"
                        held = self._hold_last_target(visible_ids, bboxes)
                        if held[1]:
                            return held
                        return visible_ids, target_visible, bboxes
                    self._last_target_bbox = None
                    self._last_target_id = None
                    self._last_target_conf = 0.0
                    if self._drone_absent_frames > self.DRONE_ABSENT_RESET:
                        self._last_target_bbox    = None
                        self._last_target_id      = None
                        self._last_target_conf    = 0.0
                        self._drone_absent_frames = 0

        if best_idx < 0:
            self._drone_absent_frames += 1
            self.last_reject_reason = "no_best_target"
            held = self._hold_last_target(visible_ids, bboxes)
            if held[1]:
                return held
            return visible_ids, target_visible, bboxes

        # ── 결과 빌드 ──
        for i, box in enumerate(boxes):
            if np.isnan(box).any():
                continue
            if i not in candidate_indices:
                continue
            x1, y1, x2, y2 = map(int, box)
            yid = int(yids[i]) if yids is not None else i
            conf = float(confs[i]) if confs is not None else 0.0
            is_target = (i == best_idx)

            if is_target:
                target_visible         = True
                self._last_target_bbox = [x1, y1, x2, y2]
                self._last_target_id   = yid
                self._last_target_conf = conf
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
                self._last_target_id      = None
                self._last_target_conf    = 0.0
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
                self._last_target_conf = 0.0

    def _check_lost(self, st):
        if st.drone_lost_since is None:
            st.drone_lost_since = time.time()
        elif time.time() - st.drone_lost_since >= config.DRONE_LOST_RESET_SEC:
            st.transition(SystemState.SCANNING)
            st.drone_lost_since = None
            self._last_target_bbox = None
            self._last_target_id = None
            self._last_target_conf = 0.0

    def trigger_engage(self):
        if self.state.system_state == SystemState.LOCKED:
            self.state.transition(SystemState.ENGAGED)
            self.state.total_engagements += 1
            return True
        return False

    def reset(self):
        self._last_target_bbox = None
        self._last_target_id = None
        self._last_target_conf = 0.0
        self._drone_absent_frames = 0
        self.state.transition(SystemState.SCANNING)
