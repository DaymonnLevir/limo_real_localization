#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Int32MultiArray, String
from visualization_msgs.msg import Marker, MarkerArray

from tree_mapper.msg import ObjectDetection, ObjectDetectionArray

from tree_mapper_ros2 import TreeMapperNode, duration_msg, spin_node


class ObjectIdTracker(TreeMapperNode):
    """Generic tracker for classified vegetation objects."""

    def __init__(self):
        super().__init__("object_id_tracker")

        self.input_array_topic = self.param("input_array_topic", "/tree_mapper/internal/detections_full")
        self.output_topic = self.param("output_topic", "/tree_mapper/internal/tracked_legacy")
        self.output_array_topic = self.param("output_array_topic", "/tree_mapper/internal/tracked_full")
        self.id_topic = self.param("id_topic", "/tree_mapper/internal/tracked_ids")
        self.marker_topic = self.param("marker_topic", "/tree_mapper/internal/tracked_markers")
        self.class_label = str(self.param("class_label", "tree")).strip().lower()

        self.max_match_dist = float(self.param("max_match_dist", 0.60))
        self.max_diameter_delta = float(self.param("max_diameter_delta", 0.45))
        self.diameter_weight = float(self.param("diameter_weight", 0.40))
        self.max_missed_time = float(self.param("max_missed_time", 25.0))
        self.position_alpha = float(self.param("position_alpha", 0.45))
        self.diameter_alpha = float(self.param("diameter_alpha", 0.38))
        self.default_radius = float(self.param("default_radius", 0.3))
        self.default_height = float(self.param("default_height", 2.0))
        self.marker_lifetime = float(self.param("marker_lifetime", 0.8))
        self.min_detection_score = float(self.param("min_detection_score", 0.0))
        self.reset_run_topic = self.param("reset_run_topic", "/tree_mapper/reset_run_token")

        self.next_id = 1
        self.tracks = {}
        self.last_header = None

        self.pose_pub = self.create_publisher(PoseArray, self.output_topic, 1)
        self.array_pub = self.create_publisher(ObjectDetectionArray, self.output_array_topic, 1)
        self.id_pub = self.create_publisher(Int32MultiArray, self.id_topic, 1)
        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, 1)

        self.create_subscription(ObjectDetectionArray, self.input_array_topic, self.array_callback, 1)
        self.create_subscription(String, self.reset_run_topic, self.reset_run_cb, 1)
        self.loginfo("object_id_tracker(%s) listening on %s", self.class_label, self.input_array_topic)

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

    def _prune_old_tracks(self, now):
        expired = [track_id for track_id, track in self.tracks.items() if (now - track["t_last"]) > self.max_missed_time]
        for track_id in expired:
            del self.tracks[track_id]

    def _build_cost(self, detection, track):
        dx = detection["x"] - track["x"]
        dy = detection["y"] - track["y"]
        distance_xy = (dx * dx + dy * dy) ** 0.5
        if distance_xy > self.max_match_dist:
            return None
        detection_diameter = max(float(detection.get("diameter", 0.0)), 0.0)
        track_diameter = max(float(track.get("diameter", 0.0)), 0.0)
        diameter_delta = 0.0
        if detection_diameter > 0.0 and track_diameter > 0.0:
            diameter_delta = abs(detection_diameter - track_diameter)
            if diameter_delta > self.max_diameter_delta:
                return None
        return float(distance_xy + self.diameter_weight * diameter_delta)

    def _associate(self, detections, now):
        track_ids = sorted(self.tracks.keys())
        detection_to_id = {}
        invalid_cost = 1e6

        if detections and track_ids:
            cost = []
            for detection in detections:
                row = []
                for track_id in track_ids:
                    value = self._build_cost(detection, self.tracks[track_id])
                    row.append(invalid_cost if value is None else value)
                cost.append(row)
            assignment = self._linear_sum_assignment(cost)
            for detection_index, column in enumerate(assignment):
                if column < 0 or cost[detection_index][column] >= invalid_cost * 0.5:
                    continue
                detection_to_id[detection_index] = track_ids[column]

        for detection_index, detection in enumerate(detections):
            if detection_index in detection_to_id:
                continue
            track_id = self.next_id
            self.next_id += 1
            self.tracks[track_id] = {
                "x": detection["x"],
                "y": detection["y"],
                "z": detection["z"],
                "radius": detection["radius"],
                "diameter": detection["diameter"],
                "extent_diameter": detection["extent_diameter"],
                "density_proxy": detection["density_proxy"],
                "classification_score": detection["classification_score"],
                "tree_score": detection["tree_score"],
                "shrub_score": detection["shrub_score"],
                "height_support": detection["height_support"],
                "radius_std": detection["radius_std"],
                "center_std": detection["center_std"],
                "slice_count": detection["slice_count"],
                "circle_fit_score": detection["circle_fit_score"],
                "geometric_score": detection["geometric_score"],
                "hits": 1,
                "t_last": now,
            }
            detection_to_id[detection_index] = track_id

        for detection_index, track_id in detection_to_id.items():
            detection = detections[detection_index]
            track = self.tracks[track_id]
            track["x"] = (1.0 - self.position_alpha) * track["x"] + self.position_alpha * detection["x"]
            track["y"] = (1.0 - self.position_alpha) * track["y"] + self.position_alpha * detection["y"]
            track["z"] = (1.0 - self.position_alpha) * track["z"] + self.position_alpha * detection["z"]
            if detection["diameter"] > 0.0:
                track["diameter"] = (1.0 - self.diameter_alpha) * track["diameter"] + self.diameter_alpha * detection["diameter"]
                track["radius"] = max(0.5 * track["diameter"], 0.0)
            track["extent_diameter"] = max(float(track["extent_diameter"]), float(detection["extent_diameter"]))
            track["density_proxy"] = max(float(track["density_proxy"]), float(detection["density_proxy"]))
            track["classification_score"] = max(float(track["classification_score"]), float(detection["classification_score"]))
            track["tree_score"] = max(float(track["tree_score"]), float(detection["tree_score"]))
            track["shrub_score"] = max(float(track["shrub_score"]), float(detection["shrub_score"]))
            track["height_support"] = max(float(track["height_support"]), float(detection["height_support"]))
            track["radius_std"] = max(float(track["radius_std"]), float(detection["radius_std"]))
            track["center_std"] = max(float(track["center_std"]), float(detection["center_std"]))
            track["slice_count"] = max(int(track["slice_count"]), int(detection["slice_count"]))
            track["circle_fit_score"] = max(float(track["circle_fit_score"]), float(detection["circle_fit_score"]))
            track["geometric_score"] = max(float(track["geometric_score"]), float(detection["geometric_score"]))
            track["hits"] += 1
            track["t_last"] = now
        return detection_to_id

    def _track_to_detection(self, track_id, track, source_detection):
        pose = Pose()
        pose.position.x = track["x"]
        pose.position.y = track["y"]
        pose.position.z = track["z"]
        pose.orientation.w = 1.0

        item = ObjectDetection()
        item.id = int(track_id)
        item.class_label = self.class_label
        item.pose = pose
        item.radius = float(max(track["radius"], 0.0))
        item.diameter = float(max(track["diameter"], 0.0))
        item.extent_diameter = float(max(track["extent_diameter"], 0.0))
        item.density_proxy = float(max(track["density_proxy"], 0.0))
        item.cluster_label = int(source_detection.get("cluster_label", -1))
        item.cluster_points = int(source_detection.get("cluster_points", 0))
        item.fit_error = float(source_detection.get("fit_error", 0.0))
        item.circle_fit_score = float(max(track["circle_fit_score"], source_detection.get("circle_fit_score", 0.0)))
        item.geometric_score = float(max(track["geometric_score"], source_detection.get("geometric_score", 0.0)))
        item.classification_score = float(max(track["classification_score"], source_detection.get("classification_score", 0.0)))
        item.tree_score = float(max(track["tree_score"], source_detection.get("tree_score", 0.0)))
        item.shrub_score = float(max(track["shrub_score"], source_detection.get("shrub_score", 0.0)))
        item.height_support = float(max(track["height_support"], source_detection.get("height_support", 0.0)))
        item.radius_std = float(max(track["radius_std"], source_detection.get("radius_std", 0.0)))
        item.center_std = float(max(track["center_std"], source_detection.get("center_std", 0.0)))
        item.slice_count = int(max(track["slice_count"], source_detection.get("slice_count", 0)))
        return item

    def _publish(self, header, detections, detection_to_id):
        pose_array = PoseArray()
        pose_array.header = header
        id_array = Int32MultiArray()
        detection_array = ObjectDetectionArray()
        detection_array.header = header
        markers = MarkerArray()

        is_tree = self.class_label == "tree"
        marker_type = Marker.CYLINDER if is_tree else Marker.SPHERE
        marker_rgb = (0.10, 0.70, 0.10) if is_tree else (0.82, 0.45, 0.10)

        for index, detection in enumerate(detections):
            track_id = detection_to_id[index]
            track = self.tracks[track_id]
            pose = Pose()
            pose.position.x = track["x"]
            pose.position.y = track["y"]
            pose.position.z = track["z"]
            pose.orientation.w = 1.0
            pose_array.poses.append(pose)
            id_array.data.append(track_id)

            item = self._track_to_detection(track_id, track, detection)
            detection_array.detections.append(item)

            marker = Marker()
            marker.header = header
            marker.ns = "%s_tracks" % self.class_label
            marker.id = track_id
            marker.type = marker_type
            marker.action = Marker.ADD
            marker.pose = pose
            marker.scale.x = max(float(item.extent_diameter), 2.0 * self.default_radius)
            marker.scale.y = max(float(item.extent_diameter), 2.0 * self.default_radius)
            marker.scale.z = max(float(item.height_support), self.default_height if is_tree else 0.6)
            marker.color.r = marker_rgb[0]
            marker.color.g = marker_rgb[1]
            marker.color.b = marker_rgb[2]
            marker.color.a = 0.42
            marker.lifetime = duration_msg(self.marker_lifetime)
            markers.markers.append(marker)

            text = Marker()
            text.header = header
            text.ns = "%s_track_ids" % self.class_label
            text.id = 100000 + track_id
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = track["x"]
            text.pose.position.y = track["y"]
            text.pose.position.z = track["z"] + marker.scale.z + 0.25
            text.pose.orientation.w = 1.0
            text.scale.z = 0.30
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.color.a = 0.95
            text.text = "%s_%d" % (self.class_label, track_id)
            text.lifetime = duration_msg(self.marker_lifetime)
            markers.markers.append(text)

        self.pose_pub.publish(pose_array)
        self.array_pub.publish(detection_array)
        self.id_pub.publish(id_array)
        self.marker_pub.publish(markers)

    def _publish_empty(self, header):
        pose_array = PoseArray()
        pose_array.header = header
        detection_array = ObjectDetectionArray()
        detection_array.header = header
        self.pose_pub.publish(pose_array)
        self.array_pub.publish(detection_array)
        self.id_pub.publish(Int32MultiArray())
        self.marker_pub.publish(MarkerArray())

    def _delete_all_markers(self, header):
        markers = MarkerArray()
        marker = Marker()
        marker.header = header
        marker.action = Marker.DELETEALL
        markers.markers.append(marker)
        self.marker_pub.publish(markers)

    def reset_run_cb(self, _msg):
        self.tracks = {}
        self.next_id = 1
        header = self.last_header if self.last_header is not None else PoseArray().header
        if not header.frame_id:
            header.frame_id = "world"
        header.stamp = self.now_msg()
        self._publish_empty(header)
        self._delete_all_markers(header)
        self.loginfo("object_id_tracker(%s): run state reset", self.class_label)

    def array_callback(self, msg):
        self.last_header = msg.header
        detections = []
        for detection in msg.detections:
            score = float(detection.classification_score)
            if score < self.min_detection_score:
                continue
            diameter = float(detection.diameter)
            radius = float(detection.radius)
            if diameter <= 0.0 and radius > 0.0:
                diameter = 2.0 * radius
            if radius <= 0.0 and diameter > 0.0:
                radius = 0.5 * diameter
            detections.append(
                {
                    "x": float(detection.pose.position.x),
                    "y": float(detection.pose.position.y),
                    "z": float(detection.pose.position.z),
                    "radius": max(radius, 0.0),
                    "diameter": max(diameter, 0.0),
                    "extent_diameter": float(detection.extent_diameter),
                    "density_proxy": float(detection.density_proxy),
                    "cluster_label": int(detection.cluster_label),
                    "cluster_points": int(detection.cluster_points),
                    "fit_error": float(detection.fit_error),
                    "circle_fit_score": float(detection.circle_fit_score),
                    "geometric_score": float(detection.geometric_score),
                    "classification_score": score,
                    "tree_score": float(detection.tree_score),
                    "shrub_score": float(detection.shrub_score),
                    "height_support": float(detection.height_support),
                    "radius_std": float(detection.radius_std),
                    "center_std": float(detection.center_std),
                    "slice_count": int(detection.slice_count),
                }
            )

        now = self.now_sec()
        self._prune_old_tracks(now)
        if not detections:
            self._publish_empty(msg.header)
            return

        detection_to_id = self._associate(detections, now)
        self._publish(msg.header, detections, detection_to_id)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectIdTracker()
    spin_node(node)


if __name__ == "__main__":
    main()
