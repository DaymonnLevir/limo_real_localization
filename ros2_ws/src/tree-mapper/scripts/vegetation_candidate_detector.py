#!/usr/bin/env python3

import csv
import json
import math
import os

import numpy as np
import rclpy
import sensor_msgs_py.point_cloud2 as pc2
import tf2_ros
from geometry_msgs.msg import Pose, PoseArray
from rclpy.duration import Duration
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Int32MultiArray, String

from tree_mapper.msg import VegetationCandidate, VegetationCandidateArray

from tree_mapper_ros2 import TreeMapperNode, as_bool, spin_node, stamp_to_sec
from tree_mapper_runtime import decode_reset_payload, resolve_output_file, resolve_runtime_context

try:
    from sklearn.cluster import DBSCAN

    SKLEARN_AVAILABLE = True
except Exception:
    SKLEARN_AVAILABLE = False


class VegetationCandidateDetector(TreeMapperNode):
    """Detect woody vegetation candidates using multi-slice circle evidence."""

    def __init__(self):
        super().__init__("vegetation_candidate_detector")

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

        self.input_cloud_topic = self.param("input_cloud_topic", "/camera/depth/points")
        self.target_frame = self.param("target_frame", "map")
        self.output_array_topic = self.param("output_array_topic", "/vegetation_mapper/internal/candidates_full")
        self.cluster_points_topic = self.param("cluster_points_topic", "/vegetation_mapper/internal/cluster_points")
        self.cluster_labels_topic = self.param("cluster_labels_topic", "/vegetation_mapper/internal/cluster_labels")
        self.debug_transformed_cloud_topic = self.param(
            "debug_transformed_cloud_topic", "/vegetation_candidate_debug/transformed_cloud"
        )
        self.debug_band_cloud_topic = self.param(
            "debug_band_cloud_topic", "/vegetation_candidate_debug/band_cloud"
        )
        self.debug_publish_clouds = as_bool(self.param("debug_publish_clouds", True))
        self.history_csv_output_path_param = self.param("history_csv_output_path", "")
        self.history_csv_output_path = resolve_output_file(
            self.history_csv_output_path_param,
            self.runtime["run_dir"],
            "tree_detection_history.csv",
        )
        self.export_slice_debug = as_bool(self.param("export_slice_debug", False))
        self.slice_debug_json_output_path_param = self.param("slice_debug_json_output_path", "")
        self.slice_debug_json_output_path = resolve_output_file(
            self.slice_debug_json_output_path_param,
            self.runtime["run_dir"],
            "tree_detection_slice_debug.jsonl",
        )
        self.reset_run_topic = self.param("reset_run_topic", "/tree_mapper/reset_run_token")

        self.max_points = int(self.param("max_points", 180000))
        self.max_points_per_candidate = int(self.param("max_points_per_candidate", 20000))
        self.band_z_min = float(self.param("band_z_min", 0.35))
        self.band_z_max = float(self.param("band_z_max", 2.10))
        self.slice_thickness = float(self.param("slice_thickness", 0.20))
        self.slice_step = float(self.param("slice_step", 0.12))
        self.min_points_per_slice = int(self.param("min_points_per_slice", 10))
        self.min_diameter = float(self.param("min_diameter", 0.10))
        self.max_diameter = float(self.param("max_diameter", 0.60))
        self.min_candidate_slices = int(self.param("min_candidate_slices", 1))
        self.association_max_dist = float(self.param("association_max_dist", 0.30))
        self.association_max_radius_delta = float(self.param("association_max_radius_delta", 0.10))
        self.fit_method = str(self.param("fit_method", "ransac")).strip().lower()
        self.enable_blob_candidates = as_bool(self.param("enable_blob_candidates", True))
        self.blob_min_points = int(self.param("blob_min_points", max(self.min_points_per_slice, 12)))
        self.blob_min_extent_diameter = float(
            self.param("blob_min_extent_diameter", max(self.min_diameter, 0.20))
        )
        self.blob_max_extent_diameter = float(self.param("blob_max_extent_diameter", 2.50))
        self.blob_max_pca_ratio = float(self.param("blob_max_pca_ratio", 2.80))
        self.blob_min_compactness = float(self.param("blob_min_compactness", 0.30))

        self.dbscan_eps = float(self.param("dbscan_eps", 0.28))
        self.dbscan_min_samples = int(self.param("dbscan_min_samples", 8))
        self.clustering_mode = str(self.param("clustering_mode", "dbscan")).strip().lower()
        self.cell_size = float(self.param("cell_size", 0.20))
        self.min_points_per_cell = int(self.param("min_points_per_cell", 3))
        self.min_cells_per_cluster = int(self.param("min_cells_per_cluster", 3))
        self.max_cells_per_cluster = int(self.param("max_cells_per_cluster", 500))

        self.ransac_iterations = int(self.param("ransac_iterations", 60))
        self.ransac_distance_threshold = float(self.param("ransac_distance_threshold", 0.015))
        self.ransac_min_inlier_ratio = float(self.param("ransac_min_inlier_ratio", 0.45))
        self.rlts_trim_percentile = float(self.param("rlts_trim_percentile", 75.0))

        # Limites x_min,x_max,y_min,y_max [m] no target_frame; pontos da banda fora deles
        # sao descartados (vazio = sem filtro). Mesmo formato do map_bounds do informative_planner.
        self.map_bounds = self._parse_map_bounds(self.param("map_bounds", ""))
        self.exclude_self_points = as_bool(self.param("exclude_self_points", True))
        self.self_filter_frame = str(self.param("self_filter_frame", "")).strip()
        self.self_filter_xy_radius = float(self.param("self_filter_xy_radius", 0.85))
        self.self_filter_z_margin = float(self.param("self_filter_z_margin", 0.80))

        self.array_pub = self.create_publisher(VegetationCandidateArray, self.output_array_topic, 1)
        self.cluster_points_pub = self.create_publisher(PoseArray, self.cluster_points_topic, 1)
        self.cluster_labels_pub = self.create_publisher(Int32MultiArray, self.cluster_labels_topic, 1)
        self.debug_transformed_cloud_pub = self.create_publisher(
            PointCloud2, self.debug_transformed_cloud_topic, self.sensor_qos()
        )
        self.debug_band_cloud_pub = self.create_publisher(
            PointCloud2, self.debug_band_cloud_topic, self.sensor_qos()
        )

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(PointCloud2, self.input_cloud_topic, self.cloud_callback, self.sensor_qos())
        self.create_subscription(String, self.reset_run_topic, self.reset_run_cb, 1)

        if self.clustering_mode != "grid_cc" and not SKLEARN_AVAILABLE:
            self.logwarn("vegetation_candidate_detector: sklearn unavailable, using grid_cc")
            self.clustering_mode = "grid_cc"
        if self.fit_method not in ("ransac", "rlts"):
            self.fit_method = "ransac"

        self.loginfo(
            "vegetation_candidate_detector: input=%s frame=%s band=[%.2f, %.2f] slice=%.2f/%.2f map_bounds=%s",
            self.input_cloud_topic,
            self.target_frame,
            self.band_z_min,
            self.band_z_max,
            self.slice_thickness,
            self.slice_step,
            "off" if self.map_bounds is None else "[%.1f, %.1f] x [%.1f, %.1f]" % self.map_bounds,
        )

    @staticmethod
    def _ensure_parent_dir(path):
        if not path:
            return
        parent = os.path.dirname(path)
        if parent and not os.path.exists(parent):
            os.makedirs(parent)

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
        self.history_csv_output_path = resolve_output_file(
            self.history_csv_output_path_param,
            self.runtime["run_dir"],
            "tree_detection_history.csv",
        )
        self.slice_debug_json_output_path = resolve_output_file(
            self.slice_debug_json_output_path_param,
            self.runtime["run_dir"],
            "tree_detection_slice_debug.jsonl",
        )

    def reset_run_cb(self, msg):
        self._refresh_runtime_outputs(decode_reset_payload(msg.data))
        self.loginfo("vegetation_candidate_detector: run reset applied run_dir=%s", self.runtime["run_dir"])

    @staticmethod
    def _stamp_sec(header):
        if header is not None and stamp_to_sec(header.stamp) > 0.0:
            return stamp_to_sec(header.stamp)
        return 0.0

    def _append_history_csv(self, header, cloud_points, band_points, candidates):
        if not self.history_csv_output_path:
            return
        self._ensure_parent_dir(self.history_csv_output_path)
        write_header = (not os.path.exists(self.history_csv_output_path)) or (
            os.path.getsize(self.history_csv_output_path) == 0
        )
        fields = [
            "stamp_sec",
            "frame_id",
            "cloud_points",
            "band_points",
            "candidate_count",
            "candidate_id",
            "x",
            "y",
            "z",
            "diameter_m",
            "extent_diameter",
            "density_proxy",
            "fit_error",
            "circle_fit_score",
            "geometric_score",
            "height_support",
            "radius_std",
            "center_std",
            "slice_count",
            "cluster_points",
        ]
        stamp_sec = self._stamp_sec(header) or self.now_sec()
        frame_id = header.frame_id if header and header.frame_id else ""

        with open(self.history_csv_output_path, "a") as handle:
            writer = csv.writer(handle)
            if write_header:
                writer.writerow(fields)
            if not candidates:
                writer.writerow(
                    ["%.3f" % stamp_sec, frame_id, cloud_points, band_points, 0, "", "", "", "", "", "", "", "", "", "", "", "", "", "", ""]
                )
                return
            for candidate in candidates:
                writer.writerow(
                    [
                        "%.3f" % stamp_sec,
                        frame_id,
                        int(cloud_points),
                        int(band_points),
                        len(candidates),
                        int(candidate["id"]),
                        "%.6f" % candidate["x"],
                        "%.6f" % candidate["y"],
                        "%.6f" % candidate["z"],
                        "%.6f" % candidate["diameter"],
                        "%.6f" % candidate["extent_diameter"],
                        "%.6f" % candidate["density_proxy"],
                        "%.6f" % candidate["fit_error"],
                        "%.6f" % candidate["circle_fit_score"],
                        "%.6f" % candidate["geometric_score"],
                        "%.6f" % candidate["height_support"],
                        "%.6f" % candidate["radius_std"],
                        "%.6f" % candidate["center_std"],
                        int(candidate["slice_count"]),
                        int(candidate["cluster_points"]),
                    ]
                )

    def _append_slice_debug_json(self, header, cloud_points, band_points, candidates):
        if (not self.export_slice_debug) or (not self.slice_debug_json_output_path):
            return

        self._ensure_parent_dir(self.slice_debug_json_output_path)
        payload = {
            "stamp_sec": self._stamp_sec(header) or self.now_sec(),
            "frame_id": header.frame_id if header and header.frame_id else "",
            "cloud_points": int(cloud_points),
            "band_points": int(band_points),
            "candidate_count": int(len(candidates)),
            "candidates": [],
        }
        for candidate in candidates:
            payload["candidates"].append(
                {
                    "id": int(candidate["id"]),
                    "x": float(candidate["x"]),
                    "y": float(candidate["y"]),
                    "z": float(candidate["z"]),
                    "radius": float(candidate["radius"]),
                    "diameter": float(candidate["diameter"]),
                    "extent_diameter": float(candidate["extent_diameter"]),
                    "density_proxy": float(candidate["density_proxy"]),
                    "cluster_points": int(candidate["cluster_points"]),
                    "fit_error": float(candidate["fit_error"]),
                    "circle_fit_score": float(candidate["circle_fit_score"]),
                    "geometric_score": float(candidate["geometric_score"]),
                    "height_support": float(candidate["height_support"]),
                    "radius_std": float(candidate["radius_std"]),
                    "center_std": float(candidate["center_std"]),
                    "slice_count": int(candidate["slice_count"]),
                    "slice_metrics": candidate.get("slice_metrics", []),
                }
            )

        with open(self.slice_debug_json_output_path, "a") as handle:
            handle.write(json.dumps(payload, sort_keys=True) + "\n")

    def _to_target_frame(self, msg):
        if not self.target_frame or msg.header.frame_id == self.target_frame:
            return msg
        try:
            source_time = Time.from_msg(msg.header.stamp) if stamp_to_sec(msg.header.stamp) > 0.0 else Time()
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                msg.header.frame_id,
                source_time,
                timeout=Duration(seconds=0.1),
            )
            return self._transform_cloud_xyz(msg, transform)
        except Exception as exc:
            self.logwarn_throttle(
                2.0,
                "vegetation_candidate_detector TF %s -> %s failed: %s",
                msg.header.frame_id,
                self.target_frame,
                exc,
            )
            return None

    @staticmethod
    def _quaternion_to_rotation_matrix(qx, qy, qz, qw):
        xx = qx * qx
        yy = qy * qy
        zz = qz * qz
        xy = qx * qy
        xz = qx * qz
        yz = qy * qz
        wx = qw * qx
        wy = qw * qy
        wz = qw * qz
        return np.asarray(
            [
                [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
                [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
                [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
            ],
            dtype=float,
        )

    def _transform_cloud_xyz(self, msg, transform):
        points = self._read_xyz_points(msg)
        header = msg.header
        header.frame_id = self.target_frame
        if points.shape[0] == 0:
            return pc2.create_cloud_xyz32(header, [])
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        rot = self._quaternion_to_rotation_matrix(rotation.x, rotation.y, rotation.z, rotation.w)
        trans = np.asarray([translation.x, translation.y, translation.z], dtype=float)
        transformed = (points @ rot.T) + trans
        transformed = transformed[np.all(np.isfinite(transformed), axis=1)]
        return pc2.create_cloud_xyz32(header, transformed.astype(np.float32).tolist())

    @staticmethod
    def _read_xyz_points(msg):
        raw = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        array = np.asarray(raw)
        if array.size == 0:
            return np.empty((0, 3), dtype=float)
        if array.dtype.names:
            points = np.column_stack(
                (
                    array["x"].astype(float, copy=False),
                    array["y"].astype(float, copy=False),
                    array["z"].astype(float, copy=False),
                )
            )
        elif array.ndim == 1:
            points = np.asarray(list(raw), dtype=float)
        else:
            points = array.astype(float, copy=False)
        if points.size == 0:
            return np.empty((0, 3), dtype=float)
        return points[np.all(np.isfinite(points), axis=1)]

    @staticmethod
    def _namespace_from_frame(frame_id):
        text = str(frame_id).strip().lstrip("/")
        if not text or "/" not in text:
            return ""
        return text.split("/", 1)[0]

    def _resolve_self_filter_frame(self, source_frame_id):
        if self.self_filter_frame:
            return self.self_filter_frame
        namespace = self._namespace_from_frame(source_frame_id)
        if namespace:
            return "%s/fcu" % namespace
        return "base_link"

    def _parse_map_bounds(self, raw):
        """Converte "x_min,x_max,y_min,y_max" (ou lista) em tupla; None = sem filtro."""
        if raw is None:
            return None
        if isinstance(raw, str):
            raw = raw.strip()
            if not raw:
                return None
            raw = raw.split(",")
        try:
            values = tuple(float(v) for v in raw)
        except (TypeError, ValueError):
            self.logwarn("vegetation_candidate_detector: map_bounds invalido (%r); filtro desligado", raw)
            return None
        if len(values) != 4 or values[0] >= values[1] or values[2] >= values[3]:
            self.logwarn("vegetation_candidate_detector: map_bounds invalido (%r); filtro desligado", raw)
            return None
        return values

    def _exclude_out_of_bounds_points(self, points_xyz):
        """Mantem so os pontos dentro de map_bounds (x/y no target_frame)."""
        if self.map_bounds is None or points_xyz.shape[0] == 0:
            return points_xyz
        x_min, x_max, y_min, y_max = self.map_bounds
        keep = (
            (points_xyz[:, 0] >= x_min) & (points_xyz[:, 0] <= x_max)
            & (points_xyz[:, 1] >= y_min) & (points_xyz[:, 1] <= y_max)
        )
        return points_xyz[keep]

    def _exclude_self_points(self, header, points_xyz, source_frame_id):
        if (
            (not self.exclude_self_points)
            or (not self.target_frame)
            or self.self_filter_xy_radius <= 0.0
            or points_xyz.shape[0] == 0
        ):
            return points_xyz
        filter_frame = self._resolve_self_filter_frame(source_frame_id)
        try:
            stamp = Time.from_msg(header.stamp) if header and stamp_to_sec(header.stamp) > 0.0 else Time()
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                filter_frame,
                stamp,
                timeout=Duration(seconds=0.05),
            )
        except Exception as exc:
            self.logwarn_throttle(
                2.0,
                "vegetation_candidate_detector self-filter TF %s -> %s failed: %s",
                filter_frame,
                self.target_frame,
                exc,
            )
            return points_xyz

        translation = transform.transform.translation
        center = np.asarray([translation.x, translation.y, translation.z], dtype=float)
        xy_dist = np.linalg.norm(points_xyz[:, :2] - center[:2], axis=1)
        if self.self_filter_z_margin > 0.0:
            z_dist = np.abs(points_xyz[:, 2] - center[2])
            keep_mask = (xy_dist > self.self_filter_xy_radius) | (z_dist > self.self_filter_z_margin)
        else:
            keep_mask = xy_dist > self.self_filter_xy_radius
        return points_xyz[keep_mask]

    def _fit_circle_ransac(self, points_xy):
        if points_xy.shape[0] < 3:
            return None
        best = None
        best_inliers = 0
        best_fit_error = None
        for _ in range(self.ransac_iterations):
            try:
                sample_idx = np.random.choice(points_xy.shape[0], 3, replace=False)
                p1, p2, p3 = points_xy[sample_idx]
                a_mat = np.array(
                    [
                        [p2[0] - p1[0], p2[1] - p1[1]],
                        [p3[0] - p2[0], p3[1] - p2[1]],
                    ]
                ) * 2.0
                b_vec = np.array(
                    [
                        [p2[0] ** 2 - p1[0] ** 2 + p2[1] ** 2 - p1[1] ** 2],
                        [p3[0] ** 2 - p2[0] ** 2 + p3[1] ** 2 - p2[1] ** 2],
                    ]
                )
                if abs(np.linalg.det(a_mat)) < 1e-8:
                    continue
                center = np.linalg.solve(a_mat, b_vec).T[0]
                radius = float(np.linalg.norm(p1 - center))
                diameter = 2.0 * radius
                if diameter < self.min_diameter or diameter > self.max_diameter:
                    continue
                residuals = np.abs(np.linalg.norm(points_xy - center, axis=1) - radius)
                inlier_mask = residuals < self.ransac_distance_threshold
                inliers = int(np.sum(inlier_mask))
                if inliers <= 0:
                    continue
                fit_error = float(np.mean(residuals[inlier_mask]))
                if inliers > best_inliers:
                    best_inliers = inliers
                    best_fit_error = fit_error
                    best = (float(center[0]), float(center[1]), radius)
            except Exception:
                continue
        if best is None:
            return None
        inlier_ratio = float(best_inliers) / float(max(points_xy.shape[0], 1))
        if inlier_ratio < self.ransac_min_inlier_ratio:
            return None
        return best[0], best[1], float(best[2]), float(best_fit_error), float(inlier_ratio)

    def _fit_circle_rlts(self, points_xy):
        if points_xy.shape[0] < 5:
            return None
        trim_percentile = max(0.0, min(100.0, self.rlts_trim_percentile))
        centroid = np.mean(points_xy, axis=0)
        centroid_distances = np.linalg.norm(points_xy - centroid, axis=1)
        keep_distance = float(np.percentile(centroid_distances, trim_percentile))
        trimmed_points = points_xy[centroid_distances <= keep_distance]
        if trimmed_points.shape[0] < 3:
            return None
        x_vals = trimmed_points[:, 0]
        y_vals = trimmed_points[:, 1]
        a_mat = np.column_stack((x_vals, y_vals, np.ones(x_vals.shape[0], dtype=float)))
        b_vec = -(x_vals ** 2 + y_vals ** 2)
        try:
            params, _, _, _ = np.linalg.lstsq(a_mat, b_vec, rcond=None)
        except np.linalg.LinAlgError:
            return None
        d_term, e_term, f_term = params
        center_x = float(-d_term / 2.0)
        center_y = float(-e_term / 2.0)
        radius_sq = float(center_x ** 2 + center_y ** 2 - f_term)
        if radius_sq <= 0.0:
            return None
        radius = math.sqrt(radius_sq)
        diameter = 2.0 * radius
        if diameter < self.min_diameter or diameter > self.max_diameter:
            return None
        residuals = np.abs(np.linalg.norm(trimmed_points - np.array([center_x, center_y]), axis=1) - radius)
        inlier_mask = residuals < self.ransac_distance_threshold
        inlier_fraction = float(np.mean(inlier_mask)) if residuals.size else 0.0
        kept_fraction = float(trimmed_points.shape[0]) / float(max(points_xy.shape[0], 1))
        inlier_ratio = kept_fraction * inlier_fraction
        if inlier_ratio < self.ransac_min_inlier_ratio:
            return None
        fit_error = float(np.mean(residuals[inlier_mask])) if np.any(inlier_mask) else float(np.mean(residuals))
        return center_x, center_y, float(radius), fit_error, float(inlier_ratio)

    def _fit_circle(self, points_xy):
        if self.fit_method == "rlts":
            return self._fit_circle_rlts(points_xy)
        return self._fit_circle_ransac(points_xy)

    @staticmethod
    def _cluster_extent(points_xy):
        center = np.mean(points_xy, axis=0)
        radius = float(np.max(np.linalg.norm(points_xy - center, axis=1)))
        return float(center[0]), float(center[1]), max(2.0 * radius, 0.0)

    @staticmethod
    def _pca_shape_stats(points_xy):
        if points_xy.shape[0] < 3:
            return 1.0, 1.0
        centered = points_xy - np.mean(points_xy, axis=0)
        covariance = np.cov(centered.T)
        eigenvalues = np.linalg.eigvalsh(covariance)
        eigenvalues = np.sort(np.maximum(eigenvalues, 0.0))[::-1]
        if eigenvalues.shape[0] < 2:
            return 1.0, 1.0
        major_sigma = math.sqrt(max(float(eigenvalues[0]), 1e-12))
        minor_sigma = math.sqrt(max(float(eigenvalues[1]), 1e-12))
        pca_ratio = major_sigma / minor_sigma
        compactness = minor_sigma / major_sigma
        return float(pca_ratio), float(compactness)

    def _circle_fit_score(self, inlier_ratio, fit_error):
        if inlier_ratio is None or fit_error is None:
            return 0.0
        err_term = math.exp(-float(fit_error) / max(2.0 * self.ransac_distance_threshold, 1e-6))
        return float(max(0.0, min(1.0, 0.65 * float(inlier_ratio) + 0.35 * float(err_term))))

    def _cluster_cells(self, occupied_cells):
        visited = set()
        components = []
        for start in occupied_cells:
            if start in visited:
                continue
            queue = [start]
            visited.add(start)
            component = []
            while queue:
                cell = queue.pop()
                component.append(cell)
                cx, cy = cell
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        if dx == 0 and dy == 0:
                            continue
                        neighbor = (cx + dx, cy + dy)
                        if neighbor in occupied_cells and neighbor not in visited:
                            visited.add(neighbor)
                            queue.append(neighbor)
            components.append(component)
        return components

    def _slice_clusters_grid(self, slice_points):
        cells = {}
        for point in slice_points:
            key = (int(np.floor(point[0] / self.cell_size)), int(np.floor(point[1] / self.cell_size)))
            cells.setdefault(key, []).append(point)
        occupied = {cell for cell, pts in cells.items() if len(pts) >= self.min_points_per_cell}
        components = self._cluster_cells(occupied)
        clusters = []
        for component in components:
            if len(component) < self.min_cells_per_cluster or len(component) > self.max_cells_per_cluster:
                continue
            cluster_points = []
            for cell in component:
                cluster_points.extend(cells[cell])
            cluster_points = np.asarray(cluster_points, dtype=float)
            if cluster_points.shape[0] >= self.min_points_per_slice:
                clusters.append(cluster_points)
        return clusters

    def _slice_clusters_dbscan(self, slice_points):
        points_xy = slice_points[:, :2]
        try:
            labels = DBSCAN(eps=self.dbscan_eps, min_samples=self.dbscan_min_samples).fit_predict(points_xy)
        except Exception as exc:
            self.logwarn_throttle(2.0, "vegetation_candidate_detector DBSCAN failed: %s", exc)
            return []
        clusters = []
        for label in sorted(set(labels.tolist())):
            if label < 0:
                continue
            cluster_idx = np.where(labels == label)[0]
            cluster_points = slice_points[cluster_idx]
            if cluster_points.shape[0] >= self.min_points_per_slice:
                clusters.append(cluster_points)
        return clusters

    def _detect_slice_objects(self, slice_points, z_center):
        if slice_points.shape[0] < self.min_points_per_slice:
            return []
        if self.clustering_mode == "grid_cc":
            clusters = self._slice_clusters_grid(slice_points)
        else:
            clusters = self._slice_clusters_dbscan(slice_points)

        detections = []
        for cluster_points in clusters:
            points_xy = cluster_points[:, :2]
            extent_center_x, extent_center_y, extent_diameter = self._cluster_extent(points_xy)
            fit = self._fit_circle(points_xy)
            if fit is not None:
                cx, cy, radius, fit_error, inlier_ratio = fit
                detections.append(
                    {
                        "x": float(cx),
                        "y": float(cy),
                        "z": float(z_center),
                        "radius": float(radius),
                        "diameter": float(2.0 * radius),
                        "extent_diameter": float(extent_diameter),
                        "cluster_points": int(cluster_points.shape[0]),
                        "fit_error": float(fit_error),
                        "circle_fit_score": self._circle_fit_score(inlier_ratio, fit_error),
                        "fit_model": "circle_%s" % self.fit_method,
                        "fit_success": True,
                        "inlier_ratio": float(inlier_ratio),
                        "pca_ratio": None,
                        "compactness": None,
                        "points": cluster_points,
                    }
                )
                continue

            if (not self.enable_blob_candidates) or cluster_points.shape[0] < self.blob_min_points:
                continue

            pca_ratio, compactness = self._pca_shape_stats(points_xy)
            if extent_diameter < self.blob_min_extent_diameter or extent_diameter > self.blob_max_extent_diameter:
                continue
            if pca_ratio > self.blob_max_pca_ratio or compactness < self.blob_min_compactness:
                continue

            detections.append(
                {
                    "x": float(extent_center_x),
                    "y": float(extent_center_y),
                    "z": float(z_center),
                    "radius": float(0.5 * extent_diameter),
                    "diameter": float(extent_diameter),
                    "extent_diameter": float(extent_diameter),
                    "cluster_points": int(cluster_points.shape[0]),
                    "fit_error": 0.0,
                    "circle_fit_score": float(max(0.10, min(0.35, 0.35 * compactness))),
                    "fit_model": "blob_pca",
                    "fit_success": False,
                    "inlier_ratio": None,
                    "pca_ratio": float(pca_ratio),
                    "compactness": float(compactness),
                    "points": cluster_points,
                }
            )
        return detections

    def _slice_centers(self):
        if self.band_z_max <= self.band_z_min or self.slice_thickness <= 0.0 or self.slice_step <= 0.0:
            return []
        start = self.band_z_min + 0.5 * self.slice_thickness
        stop = self.band_z_max - 0.5 * self.slice_thickness
        if stop < start:
            return []
        values = []
        current = start
        while current <= stop + 1e-6:
            values.append(float(current))
            current += self.slice_step
        return values

    @staticmethod
    def _new_candidate(detection):
        return {
            "observations": [detection],
            "support_points": [detection["points"]],
            "last_x": detection["x"],
            "last_y": detection["y"],
            "last_radius": detection["radius"],
            "last_z": detection["z"],
        }

    def _associate_detection(self, candidates, detection):
        best_idx = -1
        best_cost = None
        for index, candidate in enumerate(candidates):
            if detection["z"] <= candidate["last_z"]:
                continue
            dist = math.hypot(detection["x"] - candidate["last_x"], detection["y"] - candidate["last_y"])
            radius_delta = abs(detection["radius"] - candidate["last_radius"])
            if dist > self.association_max_dist or radius_delta > self.association_max_radius_delta:
                continue
            cost = dist + 0.5 * radius_delta
            if best_cost is None or cost < best_cost:
                best_cost = cost
                best_idx = index
        if best_idx < 0:
            candidates.append(self._new_candidate(detection))
            return
        candidate = candidates[best_idx]
        candidate["observations"].append(detection)
        candidate["support_points"].append(detection["points"])
        candidate["last_x"] = detection["x"]
        candidate["last_y"] = detection["y"]
        candidate["last_radius"] = detection["radius"]
        candidate["last_z"] = detection["z"]

    @staticmethod
    def _rms(values):
        if not values:
            return 0.0
        return math.sqrt(sum(item * item for item in values) / float(len(values)))

    def _finalize_candidate(self, candidate, candidate_id):
        observations = candidate["observations"]
        observations = sorted(observations, key=lambda item: float(item["z"]))
        xs = [item["x"] for item in observations]
        ys = [item["y"] for item in observations]
        zs = [item["z"] for item in observations]
        radii = [item["radius"] for item in observations]
        circle_scores = [item["circle_fit_score"] for item in observations]
        fit_errors = [item["fit_error"] for item in observations]
        cluster_points = [item["cluster_points"] for item in observations]
        extents = [item["extent_diameter"] for item in observations]

        mean_x = float(np.mean(xs))
        mean_y = float(np.mean(ys))
        mean_z = float(np.mean(zs))
        mean_radius = float(np.mean(radii))
        mean_diameter = 2.0 * mean_radius
        radius_std = float(np.std(radii)) if len(radii) > 1 else 0.0
        center_std = self._rms([math.hypot(x_val - mean_x, y_val - mean_y) for x_val, y_val in zip(xs, ys)])
        height_support = (max(zs) - min(zs) + self.slice_thickness) if zs else 0.0

        support_points = np.vstack(candidate["support_points"]) if candidate["support_points"] else np.empty((0, 3))
        if support_points.shape[0] > self.max_points_per_candidate:
            sample_idx = np.random.choice(support_points.shape[0], self.max_points_per_candidate, replace=False)
            support_points = support_points[sample_idx]
        extent_diameter = max(float(np.max(extents)) if extents else mean_diameter, mean_diameter)
        if support_points.shape[0] > 0:
            _, _, broad_extent = self._cluster_extent(support_points[:, :2])
            extent_diameter = max(extent_diameter, float(broad_extent))
        density_area = max(math.pi * (0.5 * extent_diameter) ** 2 * max(height_support, self.slice_thickness), 1e-6)
        density_proxy = float(sum(cluster_points)) / density_area

        slice_count = len(observations)
        circle_fit_score = float(np.mean(circle_scores)) if circle_scores else 0.0
        fit_error = float(np.mean(fit_errors)) if fit_errors else 0.0
        support_term = min(float(slice_count) / 5.0, 1.0)
        height_term = min(height_support / 0.8, 1.0)
        center_term = 1.0 - min(center_std / 0.35, 1.0)
        radius_term = 1.0 - min(radius_std / 0.08, 1.0)
        geometric_score = float(
            max(
                0.0,
                min(1.0, 0.30 * support_term + 0.20 * height_term + 0.25 * center_term + 0.15 * radius_term + 0.10 * circle_fit_score),
            )
        )
        slice_metrics = []
        for slice_index, item in enumerate(observations):
            slice_metrics.append(
                {
                    "slice_index": int(slice_index),
                    "z": float(item["z"]),
                    "z_min": float(item["z"] - 0.5 * self.slice_thickness),
                    "z_max": float(item["z"] + 0.5 * self.slice_thickness),
                    "x": float(item["x"]),
                    "y": float(item["y"]),
                    "radius": float(item["radius"]),
                    "diameter": float(item["diameter"]),
                    "extent_diameter": float(item["extent_diameter"]),
                    "cluster_points": int(item["cluster_points"]),
                    "fit_model": str(item.get("fit_model", "")),
                    "fit_success": bool(item.get("fit_success", False)),
                    "fit_error": float(item["fit_error"]),
                    "inlier_ratio": None if item.get("inlier_ratio") is None else float(item.get("inlier_ratio")),
                    "circle_fit_score": float(item["circle_fit_score"]),
                    "pca_ratio": None if item.get("pca_ratio") is None else float(item.get("pca_ratio")),
                    "compactness": None if item.get("compactness") is None else float(item.get("compactness")),
                }
            )

        return {
            "id": int(candidate_id),
            "x": mean_x,
            "y": mean_y,
            "z": mean_z,
            "radius": mean_radius,
            "diameter": mean_diameter,
            "extent_diameter": extent_diameter,
            "density_proxy": density_proxy,
            "cluster_points": int(sum(cluster_points)),
            "fit_error": fit_error,
            "circle_fit_score": circle_fit_score,
            "geometric_score": geometric_score,
            "height_support": float(height_support),
            "radius_std": radius_std,
            "center_std": center_std,
            "slice_count": int(slice_count),
            "slice_metrics": slice_metrics,
            "support_points": support_points,
        }

    def _detect_candidates(self, band_points):
        slice_centers = self._slice_centers()
        candidates = []
        for z_center in slice_centers:
            z_min = z_center - 0.5 * self.slice_thickness
            z_max = z_center + 0.5 * self.slice_thickness
            slice_points = band_points[(band_points[:, 2] >= z_min) & (band_points[:, 2] < z_max)]
            detections = self._detect_slice_objects(slice_points, z_center)
            for detection in detections:
                self._associate_detection(candidates, detection)

        finalized = []
        for index, candidate in enumerate(candidates, start=1):
            if len(candidate["observations"]) < self.min_candidate_slices:
                continue
            finalized.append(self._finalize_candidate(candidate, index))
        return finalized

    def _publish_debug_clusters(self, header, candidates):
        pose_array = PoseArray()
        pose_array.header = header
        labels = []
        for candidate in candidates:
            for point in candidate["support_points"]:
                pose = Pose()
                pose.position.x = float(point[0])
                pose.position.y = float(point[1])
                pose.position.z = float(point[2])
                pose.orientation.w = 1.0
                pose_array.poses.append(pose)
                labels.append(int(candidate["id"]) - 1)
        labels_msg = Int32MultiArray()
        labels_msg.data = labels
        self.cluster_points_pub.publish(pose_array)
        self.cluster_labels_pub.publish(labels_msg)

    def _publish_debug_cloud(self, publisher, header, points_xyz):
        if not self.debug_publish_clouds:
            return
        msg = pc2.create_cloud_xyz32(header, np.asarray(points_xyz, dtype=np.float32).tolist())
        publisher.publish(msg)

    def _build_candidate_array(self, header, candidates):
        out = VegetationCandidateArray()
        out.header = header
        for candidate in candidates:
            pose = Pose()
            pose.position.x = candidate["x"]
            pose.position.y = candidate["y"]
            pose.position.z = candidate["z"]
            pose.orientation.w = 1.0

            item = VegetationCandidate()
            item.id = int(candidate["id"])
            item.pose = pose
            item.radius = float(candidate["radius"])
            item.diameter = float(candidate["diameter"])
            item.extent_diameter = float(candidate["extent_diameter"])
            item.density_proxy = float(candidate["density_proxy"])
            item.cluster_points = int(candidate["cluster_points"])
            item.fit_error = float(candidate["fit_error"])
            item.circle_fit_score = float(candidate["circle_fit_score"])
            item.geometric_score = float(candidate["geometric_score"])
            item.height_support = float(candidate["height_support"])
            item.radius_std = float(candidate["radius_std"])
            item.center_std = float(candidate["center_std"])
            item.slice_count = int(candidate["slice_count"])
            out.candidates.append(item)
        return out

    def _publish_empty(self, header):
        out = VegetationCandidateArray()
        out.header = header
        self.array_pub.publish(out)
        pose_array = PoseArray()
        pose_array.header = header
        self.cluster_points_pub.publish(pose_array)
        self.cluster_labels_pub.publish(Int32MultiArray())

    def cloud_callback(self, msg):
        source_frame_id = msg.header.frame_id
        msg = self._to_target_frame(msg) # Hoje transforma para o ground truth. Na prática, vai transformar para o frame do SLAM
        if msg is None:
            return
        try:
            points = self._read_xyz_points(msg)

        except Exception as exc:
            self.logwarn_throttle(2.0, "vegetation_candidate_detector point read failed: %s", exc)
            return
        if points.shape[0] < 20:
            self._publish_empty(msg.header)
            return
        if points.shape[0] > self.max_points:
            sample_idx = np.random.choice(points.shape[0], self.max_points, replace=False)
            points = points[sample_idx]
        self._publish_debug_cloud(self.debug_transformed_cloud_pub, msg.header, points)

        band_points = points[(points[:, 2] >= self.band_z_min) & (points[:, 2] <= self.band_z_max)]
        band_points = self._exclude_self_points(msg.header, band_points, source_frame_id)
        band_points = self._exclude_out_of_bounds_points(band_points)
        self._publish_debug_cloud(self.debug_band_cloud_pub, msg.header, band_points)
        if band_points.shape[0] < 20:
            self._append_history_csv(msg.header, points.shape[0], band_points.shape[0], [])
            self._publish_empty(msg.header)
            return

        candidates = self._detect_candidates(band_points)
        self._append_history_csv(msg.header, points.shape[0], band_points.shape[0], candidates)
        self._append_slice_debug_json(msg.header, points.shape[0], band_points.shape[0], candidates)
        self.array_pub.publish(self._build_candidate_array(msg.header, candidates))
        self._publish_debug_clusters(msg.header, candidates)


def main(args=None):
    rclpy.init(args=args)
    node = VegetationCandidateDetector()
    spin_node(node)


if __name__ == "__main__":
    main()
