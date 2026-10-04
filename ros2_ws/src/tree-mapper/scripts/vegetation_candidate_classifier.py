#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray

from tree_mapper.msg import ObjectDetection, ObjectDetectionArray, VegetationCandidateArray

from tree_mapper_ros2 import TreeMapperNode, duration_msg, spin_node


class VegetationCandidateClassifier(TreeMapperNode):
    """Classify shared candidates into tree and shrub streams."""

    def __init__(self):
        super().__init__("vegetation_candidate_classifier")

        self.input_array_topic = self.param("input_array_topic", "/vegetation_mapper/internal/candidates_full")

        self.tree_output_topic = self.param("tree_output_topic", "/tree_mapper/internal/detections_legacy")
        self.tree_output_array_topic = self.param("tree_output_array_topic", "/tree_mapper/internal/detections_full")
        self.tree_marker_topic = self.param("tree_marker_topic", "/tree_mapper/internal/detections_raw_markers")
        self.tree_radius_topic = self.param("tree_radius_topic", "/tree_mapper/internal/detection_radii")

        self.shrub_output_topic = self.param("shrub_output_topic", "/shrub_mapper/internal/detections_legacy")
        self.shrub_output_array_topic = self.param(
            "shrub_output_array_topic", "/shrub_mapper/internal/detections_full"
        )
        self.shrub_marker_topic = self.param("shrub_marker_topic", "/shrub_mapper/internal/detections_raw_markers")
        self.shrub_radius_topic = self.param("shrub_radius_topic", "/shrub_mapper/internal/detection_radii")

        self.marker_lifetime = float(self.param("marker_lifetime", 0.8))

        self.tree_min_slice_count = int(self.param("tree_min_slice_count", 3))
        self.tree_min_height_support = float(self.param("tree_min_height_support", 0.35))
        self.tree_max_center_std = float(self.param("tree_max_center_std", 0.18))
        self.tree_max_radius_std = float(self.param("tree_max_radius_std", 0.05))
        self.tree_min_circle_fit_score = float(self.param("tree_min_circle_fit_score", 0.60))
        self.tree_min_classification_score = float(self.param("tree_min_classification_score", 0.55))
        self.tree_hard_min_height_support = float(
            self.param("tree_hard_min_height_support", max(self.tree_min_height_support, 0.60))
        )
        self.tree_hard_min_slice_count = int(self.param("tree_hard_min_slice_count", self.tree_min_slice_count))

        self.shrub_min_extent_diameter = float(self.param("shrub_min_extent_diameter", 0.35))
        self.shrub_max_extent_diameter = float(self.param("shrub_max_extent_diameter", 2.80))
        self.shrub_min_density_proxy = float(self.param("shrub_min_density_proxy", 6.0))
        self.shrub_min_classification_score = float(self.param("shrub_min_classification_score", 0.45))
        self.shrub_height_reference = float(self.param("shrub_height_reference", 0.80))
        self.class_margin = float(self.param("class_margin", 0.08))

        self.tree_pose_pub = self.create_publisher(PoseArray, self.tree_output_topic, 1)
        self.tree_array_pub = self.create_publisher(ObjectDetectionArray, self.tree_output_array_topic, 1)
        self.tree_marker_pub = self.create_publisher(MarkerArray, self.tree_marker_topic, 1)
        self.tree_radius_pub = self.create_publisher(Float32MultiArray, self.tree_radius_topic, 1)

        self.shrub_pose_pub = self.create_publisher(PoseArray, self.shrub_output_topic, 1)
        self.shrub_array_pub = self.create_publisher(ObjectDetectionArray, self.shrub_output_array_topic, 1)
        self.shrub_marker_pub = self.create_publisher(MarkerArray, self.shrub_marker_topic, 1)
        self.shrub_radius_pub = self.create_publisher(Float32MultiArray, self.shrub_radius_topic, 1)

        self.create_subscription(VegetationCandidateArray, self.input_array_topic, self.array_callback, 1)
        self.loginfo("vegetation_candidate_classifier listening on %s", self.input_array_topic)

    @staticmethod
    def _clamp(value):
        return float(max(0.0, min(1.0, value)))

    def _tree_score(self, candidate):
        slice_term = self._clamp(float(candidate.slice_count) / 5.0)
        height_term = self._clamp(float(candidate.height_support) / max(self.tree_min_height_support * 2.0, 1e-6))
        center_term = 1.0 - self._clamp(float(candidate.center_std) / max(self.tree_max_center_std * 2.0, 1e-6))
        radius_term = 1.0 - self._clamp(float(candidate.radius_std) / max(self.tree_max_radius_std * 2.0, 1e-6))
        circle_term = self._clamp(float(candidate.circle_fit_score))
        return self._clamp(
            0.24 * slice_term + 0.18 * height_term + 0.22 * center_term + 0.16 * radius_term + 0.20 * circle_term
        )

    def _shrub_score(self, candidate):
        extent_term = self._clamp(
            (float(candidate.extent_diameter) - self.shrub_min_extent_diameter)
            / max(self.shrub_max_extent_diameter - self.shrub_min_extent_diameter, 1e-6)
        )
        density_term = self._clamp(float(candidate.density_proxy) / max(self.shrub_min_density_proxy * 2.0, 1e-6))
        low_height_term = 1.0 - self._clamp(float(candidate.height_support) / max(self.shrub_height_reference, 1e-6))
        instability_term = 0.5 * self._clamp(float(candidate.center_std) / max(self.tree_max_center_std, 1e-6))
        instability_term += 0.5 * self._clamp(float(candidate.radius_std) / max(self.tree_max_radius_std, 1e-6))
        slice_term = 1.0 - self._clamp(float(candidate.slice_count - 1) / 4.0)
        return self._clamp(
            0.24 * extent_term + 0.22 * density_term + 0.20 * low_height_term + 0.20 * instability_term + 0.14 * slice_term
        )

    def _classify_candidate(self, candidate):
        tree_score = self._tree_score(candidate)
        shrub_score = self._shrub_score(candidate)

        tree_gate = (
            candidate.slice_count >= self.tree_min_slice_count
            and candidate.slice_count >= self.tree_hard_min_slice_count
            and candidate.height_support >= self.tree_min_height_support
            and candidate.height_support >= self.tree_hard_min_height_support
            and candidate.center_std <= self.tree_max_center_std
            and candidate.radius_std <= self.tree_max_radius_std
            and candidate.circle_fit_score >= self.tree_min_circle_fit_score
            and tree_score >= self.tree_min_classification_score
            and tree_score >= shrub_score + self.class_margin
        )
        if tree_gate:
            return "tree", tree_score, tree_score, shrub_score

        shrub_gate = (
            candidate.extent_diameter >= self.shrub_min_extent_diameter
            and candidate.extent_diameter <= self.shrub_max_extent_diameter
            and candidate.density_proxy >= self.shrub_min_density_proxy
            and shrub_score >= self.shrub_min_classification_score
            and shrub_score >= tree_score + self.class_margin
        )
        if shrub_gate:
            return "shrub", shrub_score, tree_score, shrub_score

        return "", max(tree_score, shrub_score), tree_score, shrub_score

    @staticmethod
    def _candidate_to_detection(candidate, class_label, classification_score, tree_score, shrub_score):
        pose = Pose()
        pose.position.x = float(candidate.pose.position.x)
        pose.position.y = float(candidate.pose.position.y)
        pose.position.z = float(candidate.pose.position.z)
        pose.orientation.w = 1.0

        item = ObjectDetection()
        item.id = int(candidate.id)
        item.class_label = class_label
        item.pose = pose
        item.radius = float(candidate.radius)
        item.diameter = float(candidate.diameter)
        item.extent_diameter = float(candidate.extent_diameter)
        item.density_proxy = float(candidate.density_proxy)
        item.cluster_label = int(candidate.id - 1)
        item.cluster_points = int(candidate.cluster_points)
        item.fit_error = float(candidate.fit_error)
        item.circle_fit_score = float(candidate.circle_fit_score)
        item.geometric_score = float(candidate.geometric_score)
        item.classification_score = float(classification_score)
        item.tree_score = float(tree_score)
        item.shrub_score = float(shrub_score)
        item.height_support = float(candidate.height_support)
        item.radius_std = float(candidate.radius_std)
        item.center_std = float(candidate.center_std)
        item.slice_count = int(candidate.slice_count)
        return item

    def _build_outputs(self, header, detections):
        pose_array = PoseArray()
        pose_array.header = header
        detection_array = ObjectDetectionArray()
        detection_array.header = header
        radius_msg = Float32MultiArray()
        for detection in detections:
            pose_array.poses.append(detection.pose)
            detection_array.detections.append(detection)
            radius_msg.data.append(float(detection.radius))
        return pose_array, detection_array, radius_msg

    def _build_markers(self, header, detections, namespace, rgb, marker_type):
        markers = MarkerArray()
        for index, detection in enumerate(detections):
            marker = Marker()
            marker.header = header
            marker.ns = namespace
            marker.id = index
            marker.type = marker_type
            marker.action = Marker.ADD
            marker.pose = detection.pose
            marker.scale.x = max(float(detection.extent_diameter), 0.12)
            marker.scale.y = max(float(detection.extent_diameter), 0.12)
            marker.scale.z = max(float(detection.height_support), 0.25)
            marker.color.r = rgb[0]
            marker.color.g = rgb[1]
            marker.color.b = rgb[2]
            marker.color.a = 0.78
            marker.lifetime = duration_msg(self.marker_lifetime)
            markers.markers.append(marker)
        return markers

    def array_callback(self, msg):
        tree_detections = []
        shrub_detections = []

        for candidate in msg.candidates:
            class_label, classification_score, tree_score, shrub_score = self._classify_candidate(candidate)
            if class_label == "tree":
                tree_detections.append(
                    self._candidate_to_detection(candidate, class_label, classification_score, tree_score, shrub_score)
                )
            elif class_label == "shrub":
                shrub_detections.append(
                    self._candidate_to_detection(candidate, class_label, classification_score, tree_score, shrub_score)
                )

        tree_pose_array, tree_array, tree_radii = self._build_outputs(msg.header, tree_detections)
        shrub_pose_array, shrub_array, shrub_radii = self._build_outputs(msg.header, shrub_detections)

        self.tree_pose_pub.publish(tree_pose_array)
        self.tree_array_pub.publish(tree_array)
        self.tree_radius_pub.publish(tree_radii)
        self.tree_marker_pub.publish(
            self._build_markers(msg.header, tree_detections, "tree_candidates", (0.15, 0.75, 0.20), Marker.CYLINDER)
        )

        self.shrub_pose_pub.publish(shrub_pose_array)
        self.shrub_array_pub.publish(shrub_array)
        self.shrub_radius_pub.publish(shrub_radii)
        self.shrub_marker_pub.publish(
            self._build_markers(msg.header, shrub_detections, "shrub_candidates", (0.88, 0.52, 0.10), Marker.SPHERE)
        )


def main(args=None):
    rclpy.init(args=args)
    node = VegetationCandidateClassifier()
    spin_node(node)


if __name__ == "__main__":
    main()
