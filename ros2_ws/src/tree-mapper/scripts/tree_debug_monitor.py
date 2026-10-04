#!/usr/bin/env python3

import csv
import math
import os

import rclpy

from tree_mapper.msg import ObjectDetectionArray

from tree_mapper_ros2 import TreeMapperNode, spin_node


class TreeDebugMonitor(TreeMapperNode):
    """Compare live tree-mapper streams against ground truth and print summary stats."""

    def __init__(self):
        super().__init__("tree_debug_monitor")

        self.truth_csv_path = self.param("truth_csv_path", "")
        self.truth_offset_x = float(self.param("truth_offset_x", 0.0))
        self.truth_offset_y = float(self.param("truth_offset_y", 0.0))
        self.summary_period_sec = float(self.param("summary_period_sec", 2.0))
        self.match_warn_dist = float(self.param("match_warn_dist", 0.50))
        self.duplicate_gate_dist = float(self.param("duplicate_gate_dist", 0.60))

        self.detections_topic = self.param("detections_topic", "/tree_mapper/internal/detections_full")
        self.tracked_topic = self.param("tracked_topic", "/tree_mapper/internal/tracked_full")
        self.candidates_topic = self.param("candidates_topic", "/tree_map_candidate_full")
        self.confirmed_topic = self.param("confirmed_topic", "/tree_map_full")

        self.truth_points = self._load_truth_points(self.truth_csv_path)
        self.latest = {
            "detections": [],
            "tracked": [],
            "candidates": [],
            "confirmed": [],
        }
        self.frame_ids = {
            "detections": "",
            "tracked": "",
            "candidates": "",
            "confirmed": "",
        }

        if not self.truth_points:
            self.logwarn("tree_debug_monitor: no truth points loaded from %s", self.truth_csv_path)
        else:
            self.loginfo(
                "tree_debug_monitor: loaded %d truth points from %s with offset=(%.3f, %.3f)",
                len(self.truth_points),
                self.truth_csv_path,
                self.truth_offset_x,
                self.truth_offset_y,
            )

        self.detections_sub = self.create_subscription(
            ObjectDetectionArray, self.detections_topic, self._make_cb("detections"), 1
        )
        self.tracked_sub = self.create_subscription(ObjectDetectionArray, self.tracked_topic, self._make_cb("tracked"), 1)
        self.candidates_sub = self.create_subscription(
            ObjectDetectionArray, self.candidates_topic, self._make_cb("candidates"), 1
        )
        self.confirmed_sub = self.create_subscription(
            ObjectDetectionArray, self.confirmed_topic, self._make_cb("confirmed"), 1
        )

        self.summary_timer = self.create_timer(max(self.summary_period_sec, 0.5), self._timer_cb)

    @staticmethod
    def _load_truth_points(csv_path):
        if not csv_path or not os.path.exists(csv_path):
            return []

        points = []
        with open(csv_path, "r") as handle:
            reader = csv.DictReader(handle)
            for row in reader:
                try:
                    kind = str(row.get("kind", "tree")).strip().lower()
                    if kind and kind != "tree":
                        continue
                    points.append(
                        {
                            "id": int(row.get("tree_id", len(points) + 1)),
                            "x": float(row["x"]),
                            "y": float(row["y"]),
                        }
                    )
                except Exception:
                    continue
        return points

    def _make_cb(self, key):
        def _cb(msg):
            items = []
            for det in msg.detections:
                items.append(
                    {
                        "id": int(det.id),
                        "x": float(det.pose.position.x),
                        "y": float(det.pose.position.y),
                        "diameter": float(det.diameter),
                        "circle_fit_score": float(det.circle_fit_score),
                        "geometric_score": float(det.geometric_score),
                    }
                )
            self.latest[key] = items
            self.frame_ids[key] = msg.header.frame_id if msg.header.frame_id else ""

        return _cb

    @staticmethod
    def _nearest_truth(point, truth_points):
        best = None
        best_dist = None
        for truth in truth_points:
            dx = point["x"] - truth["x"]
            dy = point["y"] - truth["y"]
            dist = math.sqrt(dx * dx + dy * dy)
            if best_dist is None or dist < best_dist:
                best = truth
                best_dist = dist
        return best, best_dist

    def _summarize(self, name, items):
        frame_id = self.frame_ids.get(name, "") or "(no-frame)"
        if not items:
            return "%s[%s]: empty" % (name, frame_id)
        if not self.truth_points:
            return "%s[%s]: count=%d (no ground truth loaded)" % (name, frame_id, len(items))

        dists = []
        truth_hits = {}
        worst = None
        dxs = []
        dys = []

        for item in items:
            shifted_item = {
                "x": item["x"] - self.truth_offset_x,
                "y": item["y"] - self.truth_offset_y,
            }
            truth, dist = self._nearest_truth(shifted_item, self.truth_points)
            if dist is None:
                continue
            dists.append(dist)
            dxs.append(item["x"] - truth["x"])
            dys.append(item["y"] - truth["y"])
            truth_hits.setdefault(truth["id"], []).append((item["id"], dist))
            if worst is None or dist > worst[1]:
                worst = (item, dist, truth)

        if not dists:
            return "%s: count=%d no truth matches" % (name, len(items))

        far_count = sum(1 for dist in dists if dist > self.match_warn_dist)
        duplicate_truths = []
        for truth_id, hits in truth_hits.items():
            close_hits = [hit for hit in hits if hit[1] <= self.duplicate_gate_dist]
            if len(close_hits) > 1:
                duplicate_truths.append((truth_id, len(close_hits)))

        mean_dist = sum(dists) / float(len(dists))
        min_dist = min(dists)
        max_dist = max(dists)
        mean_dx = sum(dxs) / float(len(dxs))
        mean_dy = sum(dys) / float(len(dys))

        text = (
            "%s[%s]: count=%d mean=%.2fm min=%.2fm max=%.2fm far>%0.2fm=%d mean_dx=%.2fm mean_dy=%.2fm"
            % (
                name,
                frame_id,
                len(items),
                mean_dist,
                min_dist,
                max_dist,
                self.match_warn_dist,
                far_count,
                mean_dx,
                mean_dy,
            )
        )

        if duplicate_truths:
            dup_text = ", ".join(["T%d x%d" % (truth_id, count) for truth_id, count in duplicate_truths[:4]])
            text += " dup=%s" % dup_text

        if worst is not None:
            item, dist, truth = worst
            text += " worst=%s_%d->T%d(%.2fm)" % (name, item["id"], truth["id"], dist)

        return text

    def _timer_cb(self):
        self.loginfo(self._summarize("detections", self.latest["detections"]))
        self.loginfo(self._summarize("tracked", self.latest["tracked"]))
        self.loginfo(self._summarize("candidates", self.latest["candidates"]))
        self.loginfo(self._summarize("confirmed", self.latest["confirmed"]))


def main(args=None):
    rclpy.init(args=args)
    node = TreeDebugMonitor()
    spin_node(node)


if __name__ == "__main__":
    main()
