#!/usr/bin/env python3
"""Run camera-free checks for the perception risk decision rules.

This script does not start ROS2, load YOLO, or require a camera. It feeds fake
class names, bounding boxes, mouth points, and hand points through the same
decision policy documented for yolo_detector_node.py.

Usage:
    python3 robot_ws/src/robot_perception/scripts/risk_logic_unit_check.py
"""

from dataclasses import dataclass
import math
from pathlib import Path
import sys


try:
    from robot_perception.risk_db import DEFAULT_CLASS_ALIASES, RISK_DATABASE
except ImportError:
    package_root = Path(__file__).resolve().parents[1]
    sys.path.insert(0, str(package_root))
    from robot_perception.risk_db import DEFAULT_CLASS_ALIASES, RISK_DATABASE


@dataclass(frozen=True)
class DetectionInput:
    model_class: str
    bbox: tuple[int, int, int, int]
    confidence: float


@dataclass(frozen=True)
class RiskDecision:
    model_class: str
    risk_class: str
    risk_level: int
    action: str
    risk_score: float
    seen_frames: int
    stable: bool
    reason_codes: tuple[str, ...]


class RiskLogicProbe:
    def __init__(self):
        self.class_aliases = {
            src.lower(): dst.lower()
            for src, dst in DEFAULT_CLASS_ALIASES.items()
        }
        self.stable_detection_frames = 3
        self.track_iou_threshold = 0.3
        self.mouth_threshold_px = 120
        self.mouth_threshold_scale = 4.0
        self.mouth_min_threshold_px = 60.0
        self.mouth_max_threshold_px = 180.0
        self.enable_unknown_object_risk = True
        self.unknown_small_object_class_name = "unknown_small_object"
        self.unknown_large_object_class_name = "unknown_large_object"
        self.small_object_reference_classes = {"battery", "coin", "lego"}
        self.small_object_reference_area_scale = 1.0
        self.small_object_fallback_max_area_ratio = 0.035
        self.mouth_size_area_scale = 1.0
        self.mouth_fallback_area_ratio = 0.035
        self._tracks = {}
        self._next_track_id = 1

    @staticmethod
    def _bbox_area(bbox):
        x1, y1, x2, y2 = bbox
        return max(0, x2 - x1) * max(0, y2 - y1)

    @staticmethod
    def _bbox_iou(a, b):
        ax1, ay1, ax2, ay2 = a
        bx1, by1, bx2, by2 = b
        inter_x1 = max(ax1, bx1)
        inter_y1 = max(ay1, by1)
        inter_x2 = min(ax2, bx2)
        inter_y2 = min(ay2, by2)
        inter_area = max(0, inter_x2 - inter_x1) * max(0, inter_y2 - inter_y1)
        if inter_area <= 0:
            return 0.0
        area_a = RiskLogicProbe._bbox_area(a)
        area_b = RiskLogicProbe._bbox_area(b)
        union_area = area_a + area_b - inter_area
        return inter_area / union_area if union_area else 0.0

    @staticmethod
    def _distance(p1, p2):
        return math.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)

    @staticmethod
    def _risk_score(base_level, confidence, is_mouth_threat, is_grabbed,
                    is_track_stable, is_unknown_small_object):
        score = base_level * 20.0
        score += max(0.0, min(confidence, 1.0)) * 10.0
        if is_track_stable:
            score += 10.0
        if is_grabbed:
            score += 15.0
        if is_mouth_threat:
            score += 40.0
        if is_unknown_small_object:
            score += 5.0
        return round(score, 2)

    def _risk_class_for_model_class(self, class_name):
        normalized = class_name.lower()
        return self.class_aliases.get(normalized, normalized)

    def _mouth_distance_threshold(self, mouth_width_px, frame_area):
        if mouth_width_px and mouth_width_px > 0:
            threshold = mouth_width_px * self.mouth_threshold_scale
        else:
            frame_side = math.sqrt(frame_area) if frame_area > 0 else self.mouth_threshold_px
            threshold = min(self.mouth_threshold_px, frame_side * 0.16)
        threshold = max(self.mouth_min_threshold_px, threshold)
        threshold = min(self.mouth_max_threshold_px, threshold)
        return threshold

    def _mouth_area_threshold(self, mouth_width_px, frame_area):
        if mouth_width_px and mouth_width_px > 0:
            return (mouth_width_px ** 2) * self.mouth_size_area_scale
        if frame_area > 0 and self.mouth_fallback_area_ratio > 0:
            return frame_area * self.mouth_fallback_area_ratio
        return None

    def _is_unknown_small_object(self, bbox_area, frame_area, reference_areas):
        if bbox_area <= 0 or frame_area <= 0:
            return False
        if reference_areas:
            return bbox_area <= max(reference_areas) * self.small_object_reference_area_scale
        return (bbox_area / frame_area) <= self.small_object_fallback_max_area_ratio

    def _unknown_object_risk_class(self, bbox_area, frame_area, reference_areas,
                                   mouth_area_threshold):
        if not self.enable_unknown_object_risk or bbox_area <= 0:
            return None
        if self._is_unknown_small_object(bbox_area, frame_area, reference_areas):
            return self.unknown_small_object_class_name
        if mouth_area_threshold is not None and bbox_area > mouth_area_threshold:
            return self.unknown_large_object_class_name
        return None

    def _update_track(self, risk_class_name, bbox, matched_track_ids):
        best_track_id = None
        best_iou = 0.0
        for track_id, track in self._tracks.items():
            if track_id in matched_track_ids:
                continue
            if track["risk_class_name"] != risk_class_name:
                continue
            iou = self._bbox_iou(bbox, track["bbox"])
            if iou > best_iou:
                best_track_id = track_id
                best_iou = iou

        if best_track_id is None or best_iou < self.track_iou_threshold:
            best_track_id = self._next_track_id
            self._next_track_id += 1
            self._tracks[best_track_id] = {
                "risk_class_name": risk_class_name,
                "bbox": bbox,
                "seen_frames": 0,
            }

        track = self._tracks[best_track_id]
        track["bbox"] = bbox
        track["seen_frames"] += 1
        matched_track_ids.add(best_track_id)
        return track["seen_frames"], track["seen_frames"] >= self.stable_detection_frames

    def evaluate_frame(self, detections, frame_size=(640, 480), hands=(),
                       mouth=None, mouth_width_px=None):
        width, height = frame_size
        frame_area = width * height
        parsed = []
        reference_areas = []
        matched_track_ids = set()
        mouth_area_threshold = self._mouth_area_threshold(mouth_width_px, frame_area)
        mouth_distance_threshold = self._mouth_distance_threshold(mouth_width_px, frame_area)

        for detection in detections:
            risk_class = self._risk_class_for_model_class(detection.model_class)
            bbox_area = self._bbox_area(detection.bbox)
            parsed.append((detection, risk_class, bbox_area))
            if risk_class in self.small_object_reference_classes:
                reference_areas.append(bbox_area)

        decisions = []
        for detection, risk_class, bbox_area in parsed:
            model_class = detection.model_class.lower()
            risk_info = RISK_DATABASE.get(risk_class)
            reason_codes = []
            is_unknown_small_object = False

            if risk_class != model_class:
                reason_codes.append(f"CLASS_ALIAS:{model_class}->{risk_class}")

            if risk_info is None:
                risk_class = self._unknown_object_risk_class(
                    bbox_area, frame_area, reference_areas, mouth_area_threshold)
                if risk_class is None:
                    continue
                risk_info = RISK_DATABASE[risk_class]
                is_unknown_small_object = risk_class == self.unknown_small_object_class_name
                if is_unknown_small_object:
                    reason_codes.append("UNKNOWN_SMALL_OBJECT")
                elif risk_class == self.unknown_large_object_class_name:
                    reason_codes.append("UNKNOWN_LARGE_OBJECT")

            base_level = risk_info["level"]
            action = risk_info["robot_action_cmd"]
            reason_codes.append(f"RISK_LEVEL_{base_level}")

            x1, y1, x2, y2 = detection.bbox
            center = (int((x1 + x2) / 2), int((y1 + y2) / 2))
            is_grabbed = False
            grabbed_hand = None
            for hand in hands:
                hx, hy = hand
                if x1 <= hx <= x2 and y1 <= hy <= y2:
                    is_grabbed = True
                    grabbed_hand = hand
                    reason_codes.append("HAND_INSIDE_BBOX")
                    break

            is_mouth_threat = False
            if mouth is not None:
                if grabbed_hand is not None:
                    if self._distance(grabbed_hand, mouth) < mouth_distance_threshold:
                        is_mouth_threat = True
                        reason_codes.append("HAND_NEAR_MOUTH")
                if self._distance(center, mouth) < mouth_distance_threshold:
                    is_mouth_threat = True
                    reason_codes.append("OBJECT_NEAR_MOUTH")

            seen_frames, is_track_stable = self._update_track(
                risk_class, detection.bbox, matched_track_ids)
            if is_track_stable:
                reason_codes.append(f"STABLE_TRACK_{seen_frames}_FRAMES")
            else:
                reason_codes.append(
                    f"UNSTABLE_TRACK_{seen_frames}_OF_{self.stable_detection_frames}")

            risk_score = self._risk_score(
                base_level, detection.confidence, is_mouth_threat, is_grabbed,
                is_track_stable, is_unknown_small_object)

            if is_mouth_threat and base_level >= 2:
                action = "EMERGENCY_STOP"
            elif not is_track_stable and action == "REMOVE":
                action = "NONE"

            decisions.append(RiskDecision(
                model_class=model_class,
                risk_class=risk_class,
                risk_level=base_level,
                action=action,
                risk_score=risk_score,
                seen_frames=seen_frames,
                stable=is_track_stable,
                reason_codes=tuple(reason_codes),
            ))

        return decisions


def _assert_single(decisions):
    assert len(decisions) == 1, f"expected 1 decision, got {len(decisions)}"
    return decisions[0]


def _assert_contains(decision, reason_code):
    assert reason_code in decision.reason_codes, (
        f"missing {reason_code}; got {decision.reason_codes}")


def run_checks():
    probe = RiskLogicProbe()
    lego = DetectionInput("lego", (10, 10, 60, 60), 0.8)
    actions = []
    for _ in range(3):
        decision = _assert_single(probe.evaluate_frame([lego]))
        actions.append(decision.action)
    assert actions == ["NONE", "NONE", "REMOVE"], actions
    assert decision.risk_class == "lego"
    assert decision.stable is True
    _assert_contains(decision, "STABLE_TRACK_3_FRAMES")

    probe = RiskLogicProbe()
    decision = _assert_single(
        probe.evaluate_frame([DetectionInput("C2x1x1", (10, 10, 60, 60), 0.8)]))
    assert decision.risk_class == "lego"
    assert decision.action == "NONE"
    _assert_contains(decision, "CLASS_ALIAS:c2x1x1->lego")

    probe = RiskLogicProbe()
    decision = _assert_single(probe.evaluate_frame(
        [DetectionInput("lego", (100, 100, 140, 140), 0.9)],
        mouth=(120, 120),
        mouth_width_px=30,
    ))
    assert decision.action == "EMERGENCY_STOP"
    _assert_contains(decision, "OBJECT_NEAR_MOUTH")

    probe = RiskLogicProbe()
    decision = _assert_single(
        probe.evaluate_frame([DetectionInput("unknown_chip", (20, 20, 60, 60), 0.7)]))
    assert decision.risk_class == "unknown_small_object"
    assert decision.risk_level == 2
    assert decision.action == "NONE"
    _assert_contains(decision, "UNKNOWN_SMALL_OBJECT")

    probe = RiskLogicProbe()
    decision = _assert_single(
        probe.evaluate_frame([DetectionInput("toy_box", (10, 10, 210, 210), 0.7)]))
    assert decision.risk_class == "unknown_large_object"
    assert decision.risk_level == 1
    assert decision.action == "NONE"
    _assert_contains(decision, "UNKNOWN_LARGE_OBJECT")


def main():
    run_checks()
    print("risk logic unit checks: PASS")
    print("- lego single-class policy: PASS")
    print("- legacy lego alias compatibility: PASS")
    print("- mouth proximity emergency stop: PASS")
    print("- unknown small object mapping: PASS")
    print("- unknown large object observe-only mapping: PASS")


if __name__ == "__main__":
    main()
