#!/usr/bin/env python3

import csv
import json
import math
import os
import threading

import rclpy
from rclpy.duration import Duration
from rclpy.time import Time
import tf2_ros
from geometry_msgs.msg import Pose, PoseArray
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, Int32MultiArray, String
from visualization_msgs.msg import Marker, MarkerArray

from tree_mapper.msg import ObjectDetection, ObjectDetectionArray

from tree_mapper_ros2 import TreeMapperNode, as_bool, spin_node
from tree_mapper_runtime import decode_reset_payload, resolve_output_file, resolve_runtime_context


class ObjectMapFuser(TreeMapperNode):
    """Generic map fuser for classified vegetation objects."""

    def __init__(self):
        super().__init__("object_map_fuser")

        self.runtime_state_namespace = self.param("runtime_state_namespace", "/tree_mapper_runtime")
        self.requested_run_output_dir = self.param("run_output_dir", "")
        self.requested_run_id = self.param("run_id", "")
        self.runtime = resolve_runtime_context(
            self,
            package_name="tree_mapper",
            requested_run_output_dir=self.requested_run_output_dir,
            requested_run_id=self.requested_run_id,
            state_namespace=self.runtime_state_namespace,
        )

        self.class_label = str(self.param("class_label", "tree")).strip().lower()
        self.array_topic = self.param("array_topic", "/tree_mapper/internal/tracked_full")
        self.output_topic = self.param("output_topic", "/tree_map_poses")
        self.output_id_topic = self.param("output_id_topic", "/tree_map_ids")
        self.output_array_topic = self.param("output_array_topic", "/tree_map_full")
        self.candidate_topic = self.param("candidate_topic", "/tree_map_candidates")
        self.candidate_id_topic = self.param("candidate_id_topic", "/tree_map_candidate_ids")
        self.candidate_array_topic = self.param("candidate_array_topic", "/tree_map_candidate_full")
        self.marker_topic = self.param("marker_topic", "/tree_map_markers")

        self.association_dist = float(self.param("association_dist", 0.70))
        self.id_reuse_max_dist = float(self.param("id_reuse_max_dist", 0.95))
        self.max_diameter_delta = float(self.param("max_diameter_delta", 0.45))
        self.diameter_assoc_weight = float(self.param("diameter_assoc_weight", 0.35))
        self.min_confirmations = int(self.param("min_confirmations", 3))
        self.max_std_xy = float(self.param("max_std_xy", 0.70))
        self.max_std_diameter = float(self.param("max_std_diameter", 0.28))
        self.min_confirmed_score = float(self.param("min_confirmed_score", 0.0))
        self.prune_unconfirmed_after_sec = float(self.param("prune_unconfirmed_after_sec", 60.0))

        self.publish_rate_hz = float(self.param("publish_rate_hz", 2.0))
        self.publish_candidates = as_bool(self.param("publish_candidates", True))

        self.default_height = float(self.param("default_height", 2.2))
        self.default_radius = float(self.param("default_radius", 0.35))
        self.marker_ns = self.param("marker_ns", "%s_map" % self.class_label)
        self.marker_text_ns = self.param("marker_text_ns", "%s_map_text" % self.class_label)
        self.marker_label_prefix = self.param("marker_label_prefix", self.class_label)

        self.auto_export_period_sec = float(self.param("auto_export_period_sec", 5.0))
        self.export_now_topic = self.param("export_now_topic", "~/export_now")
        self.log_detections_topic = self.param("log_detections_topic", "/tree_mapper/log_detections_now")
        self.reset_run_topic = self.param("reset_run_topic", "/tree_mapper/reset_run_token")
        self.observer_pose_topic = self.param("observer_pose_topic", "")
        # Frame do mapa vazio publicado antes da 1a detecção (sem last_header).
        # Deve ser o frame do planner, senão o mapa vazio é descartado por ele.
        self.default_frame_id = str(self.param("default_frame_id", "world")).strip() or "world"
        # Frame do mapa para onde a pose do observador é transformada via TF.
        # Vazio = usa a pose como veio (MRS: odom já no frame do mapa). Nav2:
        # /odom vem em 'odom' e o mapa em 'map' -> sem isto a cobertura
        # angular é sempre descartada por mismatch de frame.
        self.observer_target_frame = str(self.param("observer_target_frame", "")).strip()
        self.angular_bin_size_deg = float(self.param("angular_bin_size_deg", 15.0))
        self.angular_min_range_m = float(self.param("angular_min_range_m", 0.75))
        self.csv_output_path_param = self.param("csv_output_path", "")
        self.json_output_path_param = self.param("json_output_path", "")
        self.history_csv_output_path_param = self.param("history_csv_output_path", "")
        self.csv_output_path = resolve_output_file(
            self.csv_output_path_param,
            self.runtime["run_dir"],
            "%s_map_final.csv" % self.class_label,
        )
        self.json_output_path = resolve_output_file(
            self.json_output_path_param,
            self.runtime["run_dir"],
            "%s_map_final.json" % self.class_label,
        )
        self.history_csv_output_path = resolve_output_file(
            self.history_csv_output_path_param,
            self.runtime["run_dir"],
            "%s_map_history.csv" % self.class_label,
        )

        self.lock = threading.Lock()
        self.next_map_id = 1
        self.entries = {}
        self.source_to_map = {}
        self.last_header = None
        self.published_marker_ids = set()
        self.latest_observer_xy = None
        self.latest_observer_frame_id = ""

        self.angular_bin_size_deg = max(1.0, min(180.0, self.angular_bin_size_deg))
        self.angular_bin_count = max(1, int(round(360.0 / self.angular_bin_size_deg)))
        self.angular_bin_size_rad = (2.0 * math.pi) / float(self.angular_bin_count)

        self.pose_pub = self.create_publisher(PoseArray, self.output_topic, 1)
        self.id_pub = self.create_publisher(Int32MultiArray, self.output_id_topic, 1)
        self.array_pub = self.create_publisher(ObjectDetectionArray, self.output_array_topic, 1)
        self.candidate_pose_pub = self.create_publisher(PoseArray, self.candidate_topic, 1)
        self.candidate_id_pub = self.create_publisher(Int32MultiArray, self.candidate_id_topic, 1)
        self.candidate_array_pub = self.create_publisher(ObjectDetectionArray, self.candidate_array_topic, 1)
        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, 1)

        self.create_subscription(ObjectDetectionArray, self.array_topic, self.array_cb, 1)
        self.create_timer(0.5 if self.publish_rate_hz <= 0.0 else 1.0 / self.publish_rate_hz, self.publish_timer_cb)
        if self.auto_export_period_sec > 0.0:
            self.create_timer(self.auto_export_period_sec, self.export_timer_cb)
        self.create_subscription(Empty, self.export_now_topic, self.export_now_cb, 1)
        self.create_subscription(Empty, self.log_detections_topic, self.log_detections_cb, 1)
        self.create_subscription(String, self.reset_run_topic, self.reset_run_cb, 1)
        self._observer_tf_buffer = None
        if self.observer_pose_topic and self.observer_target_frame:
            self._observer_tf_buffer = tf2_ros.Buffer()
            self._observer_tf_listener = tf2_ros.TransformListener(self._observer_tf_buffer, self)
        if self.observer_pose_topic:
            self.create_subscription(Odometry, self.observer_pose_topic, self.observer_pose_cb, 1)
        self.on_shutdown(self.on_shutdown_cb)

        self.loginfo(
            "object_map_fuser(%s) listening on %s observer=%s angular_bin=%.1fdeg",
            self.class_label,
            self.array_topic,
            self.observer_pose_topic if self.observer_pose_topic else "(disabled)",
            self.angular_bin_size_deg,
        )

    def _refresh_runtime_outputs(self, reset_payload):
        new_run_dir = reset_payload.get("run_dir", "")
        new_run_id = reset_payload.get("run_id", "")
        self.runtime = resolve_runtime_context(
            self,
            package_name="tree_mapper",
            requested_run_output_dir=new_run_dir if new_run_dir else self.requested_run_output_dir,
            requested_run_id=new_run_id if new_run_id else self.requested_run_id,
            state_namespace=self.runtime_state_namespace,
        )
        self.csv_output_path = resolve_output_file(
            self.csv_output_path_param,
            self.runtime["run_dir"],
            "%s_map_final.csv" % self.class_label,
        )
        self.json_output_path = resolve_output_file(
            self.json_output_path_param,
            self.runtime["run_dir"],
            "%s_map_final.json" % self.class_label,
        )
        self.history_csv_output_path = resolve_output_file(
            self.history_csv_output_path_param,
            self.runtime["run_dir"],
            "%s_map_history.csv" % self.class_label,
        )

    @staticmethod
    def _dist_sq_xy(first, second):
        dx = first[0] - second[0]
        dy = first[1] - second[1]
        return dx * dx + dy * dy

    @staticmethod
    def _linear_sum_assignment(cost):
        n = len(cost)
        if n == 0:
            return []
        m = len(cost[0])
        if m == 0:
            return [-1] * n

        transposed = False
        work = cost
        n_work = n
        m_work = m
        if n > m:
            transposed = True
            n_work = m
            m_work = n
            work = [[cost[j][i] for j in range(n)] for i in range(m)]

        inf = 1e18
        u = [0.0] * (n_work + 1)
        v = [0.0] * (m_work + 1)
        p = [0] * (m_work + 1)
        way = [0] * (m_work + 1)

        for i in range(1, n_work + 1):
            p[0] = i
            j0 = 0
            minv = [inf] * (m_work + 1)
            used = [False] * (m_work + 1)
            while True:
                used[j0] = True
                i0 = p[j0]
                delta = inf
                j1 = 0
                for j in range(1, m_work + 1):
                    if used[j]:
                        continue
                    cur = work[i0 - 1][j - 1] - u[i0] - v[j]
                    if cur < minv[j]:
                        minv[j] = cur
                        way[j] = j0
                    if minv[j] < delta:
                        delta = minv[j]
                        j1 = j
                for j in range(0, m_work + 1):
                    if used[j]:
                        u[p[j]] += delta
                        v[j] -= delta
                    else:
                        minv[j] -= delta
                j0 = j1
                if p[j0] == 0:
                    break
            while True:
                j1 = way[j0]
                p[j0] = p[j1]
                j0 = j1
                if j0 == 0:
                    break

        assignment_work = [-1] * n_work
        for j in range(1, m_work + 1):
            if p[j] != 0:
                assignment_work[p[j] - 1] = j - 1

        if not transposed:
            return assignment_work

        assignment = [-1] * n
        for col_idx in range(len(assignment_work)):
            row_idx = assignment_work[col_idx]
            if row_idx >= 0:
                assignment[row_idx] = col_idx
        return assignment

    def _entry_std_xy(self, entry):
        if entry["hits"] < 2:
            return 0.0
        variance_x = max(entry["m2_x"] / float(entry["hits"] - 1), 0.0)
        variance_y = max(entry["m2_y"] / float(entry["hits"] - 1), 0.0)
        return math.sqrt(variance_x + variance_y)

    def _entry_std_diameter(self, entry):
        if entry["hits"] < 2:
            return 0.0
        variance = max(entry["m2_d"] / float(entry["hits"] - 1), 0.0)
        return math.sqrt(variance)

    def _entry_score(self, entry):
        if entry["hits"] <= 0:
            return 0.0
        return float(entry["classification_score_sum"]) / float(entry["hits"])

    def _is_confirmed(self, entry):
        if entry["hits"] < self.min_confirmations:
            return False
        if self.max_std_xy > 0.0 and self._entry_std_xy(entry) > self.max_std_xy:
            return False
        if self.max_std_diameter > 0.0 and self._entry_std_diameter(entry) > self.max_std_diameter:
            return False
        if self.min_confirmed_score > 0.0 and self._entry_score(entry) < self.min_confirmed_score:
            return False
        return True

    def _entry_radius(self, entry):
        if entry["diameter"] > 0.0:
            return max(0.5 * entry["diameter"], 0.0)
        return max(self.default_radius, 0.0)

    def _entry_console_line(self, map_id, entry, confirmed):
        status = "confirmed" if confirmed else "candidate"
        angular_ratio = self._entry_angular_coverage_ratio(entry)
        return (
            "object_map_fuser(%s) %s id=%d pos=(%.2f, %.2f, %.2f) "
            "d=%.2fm ext=%.2fm hits=%d score=%.2f tree=%.2f shrub=%.2f "
            "slices=%d h=%.2fm std_xy=%.3f std_d=%.3f ang=%.1fdeg (%d bins/%d obs)"
        ) % (
            self.class_label,
            status,
            map_id,
            entry["x"],
            entry["y"],
            entry["z"],
            max(entry["diameter"], 0.0),
            max(entry["extent_diameter"], 0.0),
            int(entry["hits"]),
            self._entry_score(entry),
            float(entry["tree_score"]),
            float(entry["shrub_score"]),
            int(entry["slice_count"]),
            float(entry["height_support"]),
            self._entry_std_xy(entry),
            self._entry_std_diameter(entry),
            360.0 * angular_ratio,
            self._entry_angular_bin_count(entry),
            int(entry["angular_observation_count"]),
        )

    def _log_detection_snapshot(self):
        with self.lock:
            entries = sorted(self.entries.items(), key=lambda item: item[0])
        self.loginfo("========== object_map_fuser(%s) detection snapshot start ==========", self.class_label)
        self.loginfo(
            "object_map_fuser(%s): total=%d",
            self.class_label,
            len(entries),
        )
        if not entries:
            self.loginfo("========== object_map_fuser(%s) detection snapshot end ==========", self.class_label)
            return
        for map_id, entry in entries:
            self.loginfo(self._entry_console_line(map_id, entry, self._is_confirmed(entry)))
        self.loginfo("========== object_map_fuser(%s) detection snapshot end ==========", self.class_label)

    @staticmethod
    def _normalize_angle_positive(angle_rad):
        while angle_rad < 0.0:
            angle_rad += 2.0 * math.pi
        while angle_rad >= 2.0 * math.pi:
            angle_rad -= 2.0 * math.pi
        return angle_rad

    def _angular_bin_index(self, angle_rad):
        normalized = self._normalize_angle_positive(angle_rad)
        index = int(math.floor(normalized / self.angular_bin_size_rad))
        return max(0, min(self.angular_bin_count - 1, index))

    def _entry_angular_bin_count(self, entry):
        return len(entry["angular_bin_hits"])

    def _entry_angular_coverage_ratio(self, entry):
        return float(self._entry_angular_bin_count(entry)) / float(max(self.angular_bin_count, 1))

    def _record_angular_observation(self, entry, object_x, object_y, header_frame_id):
        if not self.latest_observer_xy:
            return
        if header_frame_id and self.latest_observer_frame_id and header_frame_id != self.latest_observer_frame_id:
            self.logwarn_throttle(
                5.0,
                "object_map_fuser(%s): observer frame (%s) differs from map frame (%s); angular coverage skipped",
                self.class_label,
                self.latest_observer_frame_id,
                header_frame_id,
            )
            return
        dx = float(self.latest_observer_xy[0]) - float(object_x)
        dy = float(self.latest_observer_xy[1]) - float(object_y)
        range_xy = math.hypot(dx, dy)
        if range_xy < self.angular_min_range_m:
            return
        angle_rad = math.atan2(dy, dx)
        bin_index = self._angular_bin_index(angle_rad)
        entry["angular_bin_hits"].add(bin_index)
        entry["angular_observation_count"] += 1

    def _entry_cost(self, detection, map_id, entry):
        distance = math.sqrt(self._dist_sq_xy((detection["x"], detection["y"]), (entry["x"], entry["y"])))
        source_id = detection.get("source_id")
        if source_id is not None and self.source_to_map.get(source_id) == map_id:
            if distance <= self.id_reuse_max_dist:
                return 0.1 * distance
            return None
        if distance > self.association_dist:
            return None
        detection_diameter = max(float(detection.get("diameter", 0.0)), 0.0)
        entry_diameter = max(float(entry.get("diameter", 0.0)), 0.0)
        diameter_delta = 0.0
        if detection_diameter > 0.0 and entry_diameter > 0.0:
            diameter_delta = abs(detection_diameter - entry_diameter)
            if diameter_delta > self.max_diameter_delta:
                return None
        return distance + self.diameter_assoc_weight * diameter_delta

    def _associate_batch(self, detections):
        map_ids = sorted(self.entries.keys())
        detection_to_map = {}
        invalid_cost = 1e6
        if detections and map_ids:
            cost = []
            for detection in detections:
                row = []
                for map_id in map_ids:
                    value = self._entry_cost(detection, map_id, self.entries[map_id])
                    row.append(invalid_cost if value is None else float(value))
                cost.append(row)
            assignment = self._linear_sum_assignment(cost)
            for detection_index, column in enumerate(assignment):
                if column < 0 or cost[detection_index][column] >= invalid_cost * 0.5:
                    continue
                detection_to_map[detection_index] = map_ids[column]
        return detection_to_map

    def _create_entry(self, detection, now):
        map_id = self.next_map_id
        self.next_map_id += 1
        entry = {
            "class_label": self.class_label,
            "x": detection["x"],
            "y": detection["y"],
            "z": detection["z"],
            "diameter": max(detection["diameter"], 0.0),
            "extent_diameter": max(detection["extent_diameter"], 0.0),
            "density_proxy": max(detection["density_proxy"], 0.0),
            "height_support": max(detection["height_support"], 0.0),
            "radius_std": max(detection["radius_std"], 0.0),
            "center_std": max(detection["center_std"], 0.0),
            "slice_count": max(detection["slice_count"], 0),
            "tree_score": max(detection["tree_score"], 0.0),
            "shrub_score": max(detection["shrub_score"], 0.0),
            "circle_fit_score": max(detection["circle_fit_score"], 0.0),
            "geometric_score": max(detection["geometric_score"], 0.0),
            "hits": 1,
            "m2_x": 0.0,
            "m2_y": 0.0,
            "m2_d": 0.0,
            "classification_score_sum": max(detection["classification_score"], 0.0),
            "last_seen": now,
            "source_ids": set(),
            "angular_bin_hits": set(),
            "angular_observation_count": 0,
        }
        source_id = detection.get("source_id")
        if source_id is not None:
            entry["source_ids"].add(int(source_id))
            self.source_to_map[int(source_id)] = map_id
        self._record_angular_observation(entry, detection["x"], detection["y"], detection.get("frame_id", ""))
        self.entries[map_id] = entry

    def _update_entry(self, map_id, detection, now):
        entry = self.entries[map_id]
        entry["hits"] += 1
        count = float(entry["hits"])

        dx = detection["x"] - entry["x"]
        entry["x"] += dx / count
        entry["m2_x"] += dx * (detection["x"] - entry["x"])

        dy = detection["y"] - entry["y"]
        entry["y"] += dy / count
        entry["m2_y"] += dy * (detection["y"] - entry["y"])

        dz = detection["z"] - entry["z"]
        entry["z"] += dz / count

        detection_diameter = max(float(detection.get("diameter", 0.0)), 0.0)
        if detection_diameter > 0.0:
            delta = detection_diameter - entry["diameter"]
            entry["diameter"] += delta / count
            entry["m2_d"] += delta * (detection_diameter - entry["diameter"])

        entry["extent_diameter"] = max(float(entry["extent_diameter"]), float(detection["extent_diameter"]))
        entry["density_proxy"] = max(float(entry["density_proxy"]), float(detection["density_proxy"]))
        entry["height_support"] = max(float(entry["height_support"]), float(detection["height_support"]))
        entry["radius_std"] = max(float(entry["radius_std"]), float(detection["radius_std"]))
        entry["center_std"] = max(float(entry["center_std"]), float(detection["center_std"]))
        entry["slice_count"] = max(int(entry["slice_count"]), int(detection["slice_count"]))
        entry["tree_score"] = max(float(entry["tree_score"]), float(detection["tree_score"]))
        entry["shrub_score"] = max(float(entry["shrub_score"]), float(detection["shrub_score"]))
        entry["circle_fit_score"] = max(float(entry["circle_fit_score"]), float(detection["circle_fit_score"]))
        entry["geometric_score"] = max(float(entry["geometric_score"]), float(detection["geometric_score"]))
        entry["classification_score_sum"] += float(max(detection.get("classification_score", 0.0), 0.0))
        entry["last_seen"] = now
        source_id = detection.get("source_id")
        if source_id is not None:
            source_id = int(source_id)
            entry["source_ids"].add(source_id)
            self.source_to_map[source_id] = map_id
        self._record_angular_observation(entry, detection["x"], detection["y"], detection.get("frame_id", ""))

    def _prune_unconfirmed(self, now):
        if self.prune_unconfirmed_after_sec <= 0.0:
            return
        expired = []
        for map_id, entry in self.entries.items():
            if self._is_confirmed(entry):
                continue
            if (now - entry["last_seen"]) > self.prune_unconfirmed_after_sec:
                expired.append(map_id)
        for map_id in expired:
            for source_id in list(self.entries[map_id]["source_ids"]):
                if self.source_to_map.get(source_id) == map_id:
                    del self.source_to_map[source_id]
            del self.entries[map_id]

    def _consume_detections(self, header, detections):
        now = self.now_sec()
        with self.lock:
            self.last_header = header
            new_count = 0
            update_count = 0
            detection_to_map = self._associate_batch(detections)
            for index, detection in enumerate(detections):
                map_id = detection_to_map.get(index)
                if map_id is None:
                    self._create_entry(detection, now)
                    new_count += 1
                else:
                    self._update_entry(map_id, detection, now)
                    update_count += 1
            self._prune_unconfirmed(now)

    def array_cb(self, msg):
        detections = []
        for detection in msg.detections:
            source_id = int(detection.id) if int(detection.id) > 0 else None
            diameter = float(detection.diameter)
            if diameter <= 0.0 and float(detection.radius) > 0.0:
                diameter = 2.0 * float(detection.radius)
            detections.append(
                {
                    "x": float(detection.pose.position.x),
                    "y": float(detection.pose.position.y),
                    "z": float(detection.pose.position.z),
                    "diameter": max(diameter, 0.0),
                    "extent_diameter": float(detection.extent_diameter),
                    "density_proxy": float(detection.density_proxy),
                    "classification_score": float(detection.classification_score),
                    "tree_score": float(detection.tree_score),
                    "shrub_score": float(detection.shrub_score),
                    "height_support": float(detection.height_support),
                    "radius_std": float(detection.radius_std),
                    "center_std": float(detection.center_std),
                    "slice_count": int(detection.slice_count),
                    "circle_fit_score": float(detection.circle_fit_score),
                    "geometric_score": float(detection.geometric_score),
                    "source_id": source_id,
                    "frame_id": msg.header.frame_id if msg.header.frame_id else "",
                }
            )
        self._consume_detections(msg.header, detections)

    def observer_pose_cb(self, msg):
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        frame_id = msg.header.frame_id if msg.header.frame_id else ""
        if self._observer_tf_buffer is not None and frame_id and frame_id != self.observer_target_frame:
            try:
                transform = self._observer_tf_buffer.lookup_transform(
                    self.observer_target_frame,
                    frame_id,
                    Time(),
                    timeout=Duration(seconds=0.05),
                )
            except Exception as exc:
                self.logwarn_throttle(
                    5.0,
                    "object_map_fuser(%s): observer TF %s -> %s failed: %s",
                    self.class_label,
                    frame_id,
                    self.observer_target_frame,
                    exc,
                )
                return
            q = transform.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            t = transform.transform.translation
            x, y = (
                t.x + math.cos(yaw) * x - math.sin(yaw) * y,
                t.y + math.sin(yaw) * x + math.cos(yaw) * y,
            )
            frame_id = self.observer_target_frame
        self.latest_observer_xy = (x, y)
        self.latest_observer_frame_id = frame_id

    def _entry_to_detection(self, map_id, entry):
        detection = ObjectDetection()
        detection.id = int(map_id)
        detection.class_label = self.class_label
        detection.pose.position.x = entry["x"]
        detection.pose.position.y = entry["y"]
        detection.pose.position.z = entry["z"]
        detection.pose.orientation.w = 1.0
        detection.radius = float(self._entry_radius(entry))
        detection.diameter = float(max(entry["diameter"], 0.0))
        detection.extent_diameter = float(max(entry["extent_diameter"], 0.0))
        detection.density_proxy = float(max(entry["density_proxy"], 0.0))
        detection.cluster_label = -1
        detection.cluster_points = int(entry["hits"])
        detection.fit_error = float(self._entry_std_xy(entry))
        detection.circle_fit_score = float(entry["circle_fit_score"])
        detection.geometric_score = float(entry["geometric_score"])
        detection.classification_score = float(self._entry_score(entry))
        detection.tree_score = float(entry["tree_score"])
        detection.shrub_score = float(entry["shrub_score"])
        detection.height_support = float(entry["height_support"])
        detection.radius_std = float(entry["radius_std"])
        detection.center_std = float(entry["center_std"])
        detection.slice_count = int(entry["slice_count"])
        return detection

    def _build_marker(self, header, map_id, entry, confirmed):
        radius = self._entry_radius(entry)
        is_tree = self.class_label == "tree"

        marker = Marker()
        marker.header = header
        marker.ns = self.marker_ns
        marker.id = map_id
        marker.type = Marker.CYLINDER if is_tree else Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = entry["x"]
        marker.pose.position.y = entry["y"]
        marker.pose.position.z = entry["z"] + max(entry["height_support"], 0.3) * 0.5
        marker.pose.orientation.w = 1.0
        marker.scale.z = max(float(entry["height_support"]), self.default_height if is_tree else 0.6)
        marker.scale.x = max(2.0 * radius, float(entry["extent_diameter"]))
        marker.scale.y = max(2.0 * radius, float(entry["extent_diameter"]))
        marker.color.a = 0.5
        if confirmed:
            marker.color.r = 0.1 if is_tree else 0.85
            marker.color.g = 0.75 if is_tree else 0.55
            marker.color.b = 0.2 if is_tree else 0.12
        else:
            marker.color.r = 0.95
            marker.color.g = 0.65
            marker.color.b = 0.12

        text = Marker()
        text.header = header
        text.ns = self.marker_text_ns
        text.id = 100000 + map_id
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = entry["x"]
        text.pose.position.y = entry["y"]
        text.pose.position.z = entry["z"] + marker.scale.z + 0.30
        text.pose.orientation.w = 1.0
        text.scale.z = 0.30
        text.color.r = 1.0
        text.color.g = 1.0
        text.color.b = 1.0
        text.color.a = 0.95
        text.text = "%s_%d h=%d s=%.2f" % (
            self.marker_label_prefix,
            map_id,
            entry["hits"],
            self._entry_score(entry),
        )
        return marker, text

    def publish_timer_cb(self):
        with self.lock:
            entries = sorted(self.entries.items(), key=lambda item: item[0])
            header = self.last_header

        if header is None:
            header = PoseArray().header
            header.frame_id = self.default_frame_id
        header.stamp = self.now_msg()

        confirmed_pose_array = PoseArray()
        confirmed_pose_array.header = header
        confirmed_ids = Int32MultiArray()
        confirmed_array = ObjectDetectionArray()
        confirmed_array.header = header

        candidate_pose_array = PoseArray()
        candidate_pose_array.header = header
        candidate_ids = Int32MultiArray()
        candidate_array = ObjectDetectionArray()
        candidate_array.header = header

        markers = MarkerArray()
        current_marker_ids = set()

        for map_id, entry in entries:
            confirmed = self._is_confirmed(entry)
            pose = Pose()
            pose.position.x = entry["x"]
            pose.position.y = entry["y"]
            pose.position.z = entry["z"]
            pose.orientation.w = 1.0
            detection = self._entry_to_detection(map_id, entry)
            if confirmed:
                confirmed_pose_array.poses.append(pose)
                confirmed_ids.data.append(map_id)
                confirmed_array.detections.append(detection)
            elif self.publish_candidates:
                candidate_pose_array.poses.append(pose)
                candidate_ids.data.append(map_id)
                candidate_array.detections.append(detection)
            marker, text = self._build_marker(header, map_id, entry, confirmed)
            markers.markers.append(marker)
            markers.markers.append(text)
            current_marker_ids.add(map_id)

        for map_id in self.published_marker_ids - current_marker_ids:
            delete_marker = Marker()
            delete_marker.header = header
            delete_marker.ns = self.marker_ns
            delete_marker.id = map_id
            delete_marker.action = Marker.DELETE
            markers.markers.append(delete_marker)

            delete_text = Marker()
            delete_text.header = header
            delete_text.ns = self.marker_text_ns
            delete_text.id = 100000 + map_id
            delete_text.action = Marker.DELETE
            markers.markers.append(delete_text)

        self.published_marker_ids = current_marker_ids

        self.pose_pub.publish(confirmed_pose_array)
        self.id_pub.publish(confirmed_ids)
        self.array_pub.publish(confirmed_array)
        self.candidate_pose_pub.publish(candidate_pose_array)
        self.candidate_id_pub.publish(candidate_ids)
        self.candidate_array_pub.publish(candidate_array)
        self.marker_pub.publish(markers)

    @staticmethod
    def _ensure_parent_dir(path):
        if not path:
            return
        parent = os.path.dirname(path)
        if parent and not os.path.exists(parent):
            os.makedirs(parent)

    def _snapshot_rows(self):
        with self.lock:
            now = self.now_sec()
            frame_id = self.last_header.frame_id if self.last_header is not None and self.last_header.frame_id else self.default_frame_id
            rows = []
            for map_id, entry in sorted(self.entries.items(), key=lambda item: item[0]):
                rows.append(
                    {
                        "map_id": map_id,
                        "class_label": self.class_label,
                        "x": entry["x"],
                        "y": entry["y"],
                        "z": entry["z"],
                        "diameter_m": max(entry["diameter"], 0.0),
                        "extent_diameter": max(entry["extent_diameter"], 0.0),
                        "density_proxy": max(entry["density_proxy"], 0.0),
                        "hits": entry["hits"],
                        "std_xy": self._entry_std_xy(entry),
                        "std_diameter": self._entry_std_diameter(entry),
                        "classification_score": self._entry_score(entry),
                        "tree_score": entry["tree_score"],
                        "shrub_score": entry["shrub_score"],
                        "height_support": entry["height_support"],
                        "radius_std": entry["radius_std"],
                        "center_std": entry["center_std"],
                        "slice_count": entry["slice_count"],
                        "angular_bin_count": self._entry_angular_bin_count(entry),
                        "angular_coverage_ratio": self._entry_angular_coverage_ratio(entry),
                        "angular_coverage_deg": 360.0 * self._entry_angular_coverage_ratio(entry),
                        "angular_observation_count": int(entry["angular_observation_count"]),
                        "confirmed": self._is_confirmed(entry),
                        "age_sec": now - entry["last_seen"],
                        "last_seen_sec": entry["last_seen"],
                        "source_ids": sorted(list(entry["source_ids"])),
                    }
                )
        return frame_id, rows

    def _append_history_csv(self, reason, frame_id, exported_at_sec, rows):
        self._ensure_parent_dir(self.history_csv_output_path)
        write_header = (not os.path.exists(self.history_csv_output_path)) or (
            os.path.getsize(self.history_csv_output_path) == 0
        )
        count_confirmed = len([row for row in rows if row["confirmed"]])
        export_seq = int(round(exported_at_sec * 1000.0))
        fields = [
            "export_seq",
            "exported_at_sec",
            "reason",
            "frame_id",
            "count_total",
            "count_confirmed",
            "map_id",
            "class_label",
            "x",
            "y",
            "z",
            "diameter_m",
            "extent_diameter",
            "density_proxy",
            "hits",
            "std_xy",
            "std_diameter",
            "classification_score",
            "tree_score",
            "shrub_score",
            "height_support",
            "radius_std",
            "center_std",
            "slice_count",
            "angular_bin_count",
            "angular_coverage_ratio",
            "angular_coverage_deg",
            "angular_observation_count",
            "confirmed",
            "age_sec",
            "last_seen_sec",
            "source_ids",
        ]
        with open(self.history_csv_output_path, "a") as handle:
            writer = csv.writer(handle)
            if write_header:
                writer.writerow(fields)
            if not rows:
                writer.writerow([export_seq, "%.3f" % exported_at_sec, reason, frame_id, 0, 0, "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", ""])
                return
            for row in rows:
                writer.writerow(
                    [
                        export_seq,
                        "%.3f" % exported_at_sec,
                        reason,
                        frame_id,
                        len(rows),
                        count_confirmed,
                        row["map_id"],
                        row["class_label"],
                        "%.6f" % row["x"],
                        "%.6f" % row["y"],
                        "%.6f" % row["z"],
                        "%.6f" % row["diameter_m"],
                        "%.6f" % row["extent_diameter"],
                        "%.6f" % row["density_proxy"],
                        row["hits"],
                        "%.6f" % row["std_xy"],
                        "%.6f" % row["std_diameter"],
                        "%.6f" % row["classification_score"],
                        "%.6f" % row["tree_score"],
                        "%.6f" % row["shrub_score"],
                        "%.6f" % row["height_support"],
                        "%.6f" % row["radius_std"],
                        "%.6f" % row["center_std"],
                        row["slice_count"],
                        row["angular_bin_count"],
                        "%.6f" % row["angular_coverage_ratio"],
                        "%.6f" % row["angular_coverage_deg"],
                        row["angular_observation_count"],
                        int(row["confirmed"]),
                        "%.3f" % row["age_sec"],
                        "%.3f" % row["last_seen_sec"],
                        ";".join([str(item) for item in row["source_ids"]]),
                    ]
                )

    def _export_files(self, reason):
        frame_id, rows = self._snapshot_rows()
        exported_at_sec = self.now_sec()
        exported = False
        if self.csv_output_path:
            try:
                self._ensure_parent_dir(self.csv_output_path)
                with open(self.csv_output_path, "w") as handle:
                    writer = csv.writer(handle)
                    writer.writerow(
                        [
                            "map_id", "class_label", "x", "y", "z", "diameter_m", "extent_diameter",
                            "density_proxy", "hits", "std_xy", "std_diameter", "classification_score",
                            "tree_score", "shrub_score", "height_support", "radius_std", "center_std",
                            "slice_count", "angular_bin_count", "angular_coverage_ratio",
                            "angular_coverage_deg", "angular_observation_count",
                            "confirmed", "age_sec", "last_seen_sec", "source_ids",
                        ]
                    )
                    for row in rows:
                        writer.writerow(
                            [
                                row["map_id"],
                                row["class_label"],
                                "%.6f" % row["x"],
                                "%.6f" % row["y"],
                                "%.6f" % row["z"],
                                "%.6f" % row["diameter_m"],
                                "%.6f" % row["extent_diameter"],
                                "%.6f" % row["density_proxy"],
                                row["hits"],
                                "%.6f" % row["std_xy"],
                                "%.6f" % row["std_diameter"],
                                "%.6f" % row["classification_score"],
                                "%.6f" % row["tree_score"],
                                "%.6f" % row["shrub_score"],
                                "%.6f" % row["height_support"],
                                "%.6f" % row["radius_std"],
                                "%.6f" % row["center_std"],
                                row["slice_count"],
                                row["angular_bin_count"],
                                "%.6f" % row["angular_coverage_ratio"],
                                "%.6f" % row["angular_coverage_deg"],
                                row["angular_observation_count"],
                                int(row["confirmed"]),
                                "%.3f" % row["age_sec"],
                                "%.3f" % row["last_seen_sec"],
                                ";".join([str(item) for item in row["source_ids"]]),
                            ]
                        )
                exported = True
            except Exception as exc:
                self.logwarn("object_map_fuser(%s) failed CSV export: %s", self.class_label, exc)
        if self.json_output_path:
            try:
                self._ensure_parent_dir(self.json_output_path)
                payload = {
                    "frame_id": frame_id,
                    "class_label": self.class_label,
                    "exported_at_sec": exported_at_sec,
                    "count_total": len(rows),
                    "count_confirmed": len([row for row in rows if row["confirmed"]]),
                    "objects": rows,
                }
                with open(self.json_output_path, "w") as handle:
                    json.dump(payload, handle, indent=2, sort_keys=True)
                exported = True
            except Exception as exc:
                self.logwarn("object_map_fuser(%s) failed JSON export: %s", self.class_label, exc)
        if self.history_csv_output_path:
            try:
                self._append_history_csv(reason, frame_id, exported_at_sec, rows)
                exported = True
            except Exception as exc:
                self.logwarn("object_map_fuser(%s) failed history export: %s", self.class_label, exc)
        if exported and reason != "periodic":
            self.loginfo("object_map_fuser(%s) export(%s): %d objects", self.class_label, reason, len(rows))

    def _publish_reset_empty(self, header):
        confirmed_pose_array = PoseArray()
        confirmed_pose_array.header = header
        confirmed_array = ObjectDetectionArray()
        confirmed_array.header = header
        candidate_pose_array = PoseArray()
        candidate_pose_array.header = header
        candidate_array = ObjectDetectionArray()
        candidate_array.header = header
        self.pose_pub.publish(confirmed_pose_array)
        self.id_pub.publish(Int32MultiArray())
        self.array_pub.publish(confirmed_array)
        self.candidate_pose_pub.publish(candidate_pose_array)
        self.candidate_id_pub.publish(Int32MultiArray())
        self.candidate_array_pub.publish(candidate_array)

        markers = MarkerArray()
        marker = Marker()
        marker.header = header
        marker.action = Marker.DELETEALL
        markers.markers.append(marker)
        self.marker_pub.publish(markers)

    def reset_run_cb(self, msg):
        with self.lock:
            self.entries = {}
            self.source_to_map = {}
            self.next_map_id = 1
            self.published_marker_ids = set()
            self.latest_observer_xy = None
            self.latest_observer_frame_id = ""
            header = self.last_header
        if header is None:
            header = PoseArray().header
            header.frame_id = self.default_frame_id
        header.stamp = self.now_msg()
        self._refresh_runtime_outputs(decode_reset_payload(msg.data))
        self._publish_reset_empty(header)
        self.loginfo(
            "object_map_fuser(%s): run reset applied run_dir=%s",
            self.class_label,
            self.runtime["run_dir"],
        )

    def export_timer_cb(self):
        self._export_files("periodic")

    def export_now_cb(self, _msg):
        self._export_files("manual")

    def log_detections_cb(self, _msg):
        self._log_detection_snapshot()

    def on_shutdown_cb(self):
        self._export_files("shutdown")


def main(args=None):
    rclpy.init(args=args)
    node = ObjectMapFuser()
    spin_node(node)


if __name__ == "__main__":
    main()
