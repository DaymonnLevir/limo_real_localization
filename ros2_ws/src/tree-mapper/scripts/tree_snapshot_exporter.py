#!/usr/bin/env python3

import csv
import json
import os
import shutil

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, Int32MultiArray, String

from tree_mapper.msg import ObjectDetectionArray
from tree_mapper.srv import ExportSnapshot

from tree_mapper_ros2 import TreeMapperNode, as_bool, spin_node
from tree_mapper_runtime import (
    decode_reset_payload,
    ensure_dir,
    ensure_parent_dir,
    normalize_optional_path,
    resolve_output_file,
    resolve_runtime_context,
    sanitize_token,
    timestamp_token,
)


class TreeSnapshotExporter(TreeMapperNode):
    """Export tree-mapper runtime artifacts on demand without requiring the live plotter."""

    LEGEND_ANCHOR = (1.02, 1.0)
    METRICS_ANCHOR = (1.02, 0.34)
    RIGHT_MARGIN = 0.66

    def __init__(self):
        super().__init__("tree_snapshot_exporter")

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

        self.cluster_points_topic = self.param("cluster_points_topic", "/tree_mapper/internal/cluster_points")
        self.cluster_labels_topic = self.param("cluster_labels_topic", "/tree_mapper/internal/cluster_labels")
        self.map_array_topic = self.param("map_array_topic", "/tree_map_full")
        self.map_candidate_array_topic = self.param("map_candidate_array_topic", "/tree_map_candidate_full")
        self.route_pose_topic = self.param("route_pose_topic", "")

        self.fixed_axes = as_bool(self.param("fixed_axes", False))
        self.x_min = float(self.param("x_min", -10.0))
        self.x_max = float(self.param("x_max", 10.0))
        self.y_min = float(self.param("y_min", -8.0))
        self.y_max = float(self.param("y_max", 7.0))
        self.axis_margin = float(self.param("axis_margin", 5.0))
        self.use_boundary_bounds = as_bool(self.param("use_boundary_bounds", False))
        self.tick_step = float(self.param("tick_step", 5.0))
        self.cluster_point_size = float(self.param("cluster_point_size", 12.0))
        self.cluster_alpha = float(self.param("cluster_alpha", 0.62))
        self.show_route_trace = as_bool(self.param("show_route_trace", True))
        self.show_ground_truth = as_bool(self.param("show_ground_truth", False))
        self.show_ground_truth_labels = as_bool(self.param("show_ground_truth_labels", True))
        self.ground_truth_csv_path = normalize_optional_path(self.param("ground_truth_csv_path", ""))
        self.reset_run_topic = self.param("reset_run_topic", "/tree_mapper/reset_run_token")

        self.tree_map_export_now_topic = self.param("tree_map_export_now_topic", "/tree_map_fuser/export_now")
        self.tree_map_csv_output_path_param = self.param("tree_map_csv_output_path", "")
        self.tree_map_csv_output_path = resolve_output_file(
            self.tree_map_csv_output_path_param,
            self.runtime["run_dir"],
            "tree_map_final.csv",
        )
        self.tree_map_json_output_path_param = self.param("tree_map_json_output_path", "")
        self.tree_map_json_output_path = resolve_output_file(
            self.tree_map_json_output_path_param,
            self.runtime["run_dir"],
            "tree_map_final.json",
        )
        self.tree_map_history_output_path_param = self.param("tree_map_history_output_path", "")
        self.tree_map_history_output_path = resolve_output_file(
            self.tree_map_history_output_path_param,
            self.runtime["run_dir"],
            "tree_map_history.csv",
        )
        self.tree_detection_history_output_path_param = self.param("tree_detection_history_output_path", "")
        self.tree_detection_history_output_path = resolve_output_file(
            self.tree_detection_history_output_path_param,
            self.runtime["run_dir"],
            "tree_detection_history.csv",
        )

        self.snapshot_topic = self.param("snapshot_topic", "~/save_snapshot")
        self.snapshot_prefix = self.param("snapshot_prefix", "tree_snapshot")
        self.save_on_shutdown = as_bool(self.param("save_on_shutdown", True))
        self.shutdown_snapshot_basename = self.param("shutdown_snapshot_basename", "tree_cluster_state_final")

        self.cluster_points = []
        self.cluster_labels = []
        self.map_detections = []
        self.map_candidate_detections = []
        self.route_points = []
        self.route_frame_id = ""
        self.data_frame_id = ""
        self.ground_truth = self._load_ground_truth()
        self.palette = plt.cm.get_cmap("tab20", 20)
        self.export_now_pub = self.create_publisher(Empty, self.tree_map_export_now_topic, 1)

        self.cluster_points_sub = self.create_subscription(PoseArray, self.cluster_points_topic, self.cluster_points_cb, 1)
        self.cluster_labels_sub = self.create_subscription(
            Int32MultiArray, self.cluster_labels_topic, self.cluster_labels_cb, 1
        )
        self.map_sub = self.create_subscription(ObjectDetectionArray, self.map_array_topic, self.map_array_cb, 1)
        self.map_candidate_sub = self.create_subscription(
            ObjectDetectionArray, self.map_candidate_array_topic, self.map_candidate_array_cb, 1
        )
        self.route_pose_sub = None
        if self.show_route_trace and self.route_pose_topic:
            self.route_pose_sub = self.create_subscription(Odometry, self.route_pose_topic, self.route_cb, 1)

        self.export_service = self.create_service(ExportSnapshot, "~/export_snapshot", self.handle_export_snapshot)
        self.snapshot_sub = self.create_subscription(Empty, self.snapshot_topic, self.default_snapshot_cb, 1)
        self.reset_sub = self.create_subscription(String, self.reset_run_topic, self.reset_run_cb, 1)
        self.on_shutdown(self.handle_shutdown)

        self.loginfo(
            "tree_snapshot_exporter: run_dir=%s snapshots=%s service=%s topic=%s",
            self.runtime["run_dir"],
            self.runtime["snapshots_dir"],
            self.resolve_name("~/export_snapshot"),
            self.resolve_name(self.snapshot_topic),
        )
        if self.show_route_trace and self.route_pose_topic:
            self.loginfo("tree_snapshot_exporter: route topic=%s", self.route_pose_topic)

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
        self.tree_map_csv_output_path = resolve_output_file(
            self.tree_map_csv_output_path_param,
            self.runtime["run_dir"],
            "tree_map_final.csv",
        )
        self.tree_map_json_output_path = resolve_output_file(
            self.tree_map_json_output_path_param,
            self.runtime["run_dir"],
            "tree_map_final.json",
        )
        self.tree_map_history_output_path = resolve_output_file(
            self.tree_map_history_output_path_param,
            self.runtime["run_dir"],
            "tree_map_history.csv",
        )
        self.tree_detection_history_output_path = resolve_output_file(
            self.tree_detection_history_output_path_param,
            self.runtime["run_dir"],
            "tree_detection_history.csv",
        )

    def reset_run_cb(self, msg):
        self.cluster_points = []
        self.cluster_labels = []
        self.map_detections = []
        self.map_candidate_detections = []
        self.route_points = []
        self.route_frame_id = ""
        self.data_frame_id = ""
        self._refresh_runtime_outputs(decode_reset_payload(msg.data))
        self.loginfo("tree_snapshot_exporter: run reset applied run_dir=%s", self.runtime["run_dir"])

    def _load_ground_truth(self):
        if (not self.show_ground_truth) or (not self.ground_truth_csv_path) or (not os.path.exists(self.ground_truth_csv_path)):
            return []
        entries = []
        try:
            with open(self.ground_truth_csv_path, "r") as handle:
                reader = csv.DictReader(handle)
                for row in reader:
                    label = str(row.get("type", "tree")).strip().lower()
                    entries.append(
                        {
                            "type": "bush" if "bush" in label else "tree",
                            "x": float(row.get("x", 0.0)),
                            "y": float(row.get("y", 0.0)),
                        }
                    )
        except Exception as exc:
            self.logwarn("tree_snapshot_exporter: failed reading ground truth %s: %s", self.ground_truth_csv_path, exc)
            return []
        return entries

    @staticmethod
    def _to_detection_list(msg):
        detections = []
        for det in msg.detections:
            detections.append(
                {
                    "id": int(det.id),
                    "x": float(det.pose.position.x),
                    "y": float(det.pose.position.y),
                    "z": float(det.pose.position.z),
                    "diameter": float(det.diameter),
                    "circle_fit_score": float(det.circle_fit_score),
                    "geometric_score": float(det.geometric_score),
                    "cluster_label": int(det.cluster_label),
                }
            )
        return detections

    @staticmethod
    def _align_labels(count, labels):
        aligned = [-1] * count
        limit = min(count, len(labels))
        for index in range(limit):
            aligned[index] = int(labels[index])
        return aligned

    @staticmethod
    def _snapshot_basename(name_value):
        raw = str(name_value).strip()
        if raw.lower().endswith(".png"):
            raw = raw[:-4]
        return sanitize_token(raw, "tree_cluster_state")

    def _cluster_colors(self, labels):
        colors = []
        for label in labels:
            if label < 0:
                rgba = [0.25, 0.25, 0.25, self.cluster_alpha * 0.65]
            else:
                rgba = list(self.palette(int(label) % 20))
                rgba[3] = self.cluster_alpha
            colors.append(rgba)
        return np.asarray(colors, dtype=float)

    def _apply_boundary_bounds_if_available(self, axis):
        if not self.fixed_axes or not self.use_boundary_bounds:
            return
        try:
            boundary_min = self.param("boundary.min", [self.x_min, self.y_min])
            boundary_max = self.param("boundary.max", [self.x_max, self.y_max])
            x_min = float(boundary_min[0]) - self.axis_margin
            y_min = float(boundary_min[1]) - self.axis_margin
            x_max = float(boundary_max[0]) + self.axis_margin
            y_max = float(boundary_max[1]) + self.axis_margin
        except Exception:
            x_min = self.x_min
            y_min = self.y_min
            x_max = self.x_max
            y_max = self.y_max
        axis.set_xlim(x_min, x_max)
        axis.set_ylim(y_min, y_max)
        if self.tick_step > 0.0:
            axis.set_xticks(np.arange(x_min, x_max + self.tick_step, self.tick_step))
            axis.set_yticks(np.arange(y_min, y_max + self.tick_step, self.tick_step))

    def cluster_points_cb(self, msg):
        if msg.header.frame_id:
            self.data_frame_id = msg.header.frame_id
        self.cluster_points = [(pose.position.x, pose.position.y) for pose in msg.poses]

    def cluster_labels_cb(self, msg):
        self.cluster_labels = list(msg.data)

    def map_array_cb(self, msg):
        if msg.header.frame_id:
            self.data_frame_id = msg.header.frame_id
        self.map_detections = self._to_detection_list(msg)

    def map_candidate_array_cb(self, msg):
        if msg.header.frame_id:
            self.data_frame_id = msg.header.frame_id
        self.map_candidate_detections = self._to_detection_list(msg)

    def _append_route_point(self, point, route_frame_id):
        self.route_points.append(point)
        if len(self.route_points) > 4000:
            self.route_points = self.route_points[-4000:]
        if route_frame_id:
            self.route_frame_id = route_frame_id

    def route_cb(self, msg):
        point = (float(msg.pose.pose.position.x), float(msg.pose.pose.position.y))
        route_frame_id = msg.header.frame_id if msg.header.frame_id else ""
        self._append_route_point(point, route_frame_id)

    def _trigger_map_export(self):
        if self.export_now_pub.get_subscription_count() <= 0:
            return
        self.export_now_pub.publish(Empty())
        self.sleep(0.25)

    def _copy_if_exists(self, source_path, dest_dir):
        normalized = normalize_optional_path(source_path)
        if (not normalized) or (not os.path.exists(normalized)):
            return ""
        ensure_dir(dest_dir)
        dest_path = os.path.join(dest_dir, os.path.basename(normalized))
        if os.path.abspath(normalized) == os.path.abspath(dest_path):
            return normalized
        shutil.copy2(normalized, dest_path)
        return dest_path

    def _plot_snapshot(self, png_path):
        cluster_xy = list(self.cluster_points)
        plot_labels = self._align_labels(len(cluster_xy), self.cluster_labels)
        confirmed_xy = [(item["x"], item["y"]) for item in self.map_detections]
        candidate_xy = [(item["x"], item["y"]) for item in self.map_candidate_detections]
        route_xy = list(self.route_points)
        gt_tree_xy = [(entry["x"], entry["y"]) for entry in self.ground_truth if entry["type"] == "tree"]
        gt_bush_xy = [(entry["x"], entry["y"]) for entry in self.ground_truth if entry["type"] == "bush"]

        fig, axis = plt.subplots(figsize=(9, 9))
        fig.subplots_adjust(right=self.RIGHT_MARGIN)
        axis.set_title("Tree Mapper Snapshot")
        axis.set_xlabel("X (m)")
        axis.set_ylabel("Y (m)")
        axis.grid(True, alpha=0.35)
        axis.set_aspect("equal", "box")

        if cluster_xy:
            cluster_arr = np.asarray(cluster_xy, dtype=float)
            axis.scatter(
                cluster_arr[:, 0],
                cluster_arr[:, 1],
                s=self.cluster_point_size,
                c=self._cluster_colors(plot_labels),
                linewidths=0,
                label="cluster",
            )
        if confirmed_xy:
            confirmed_arr = np.asarray(confirmed_xy, dtype=float)
            axis.scatter(
                confirmed_arr[:, 0],
                confirmed_arr[:, 1],
                c="#1f8f3a",
                s=84,
                marker="x",
                linewidths=1.8,
                label="confirmed",
            )
        if candidate_xy:
            candidate_arr = np.asarray(candidate_xy, dtype=float)
            axis.scatter(
                candidate_arr[:, 0],
                candidate_arr[:, 1],
                c="#e67e22",
                s=64,
                marker="^",
                alpha=0.90,
                label="candidate",
            )
        if self.show_ground_truth and gt_tree_xy:
            gt_tree_arr = np.asarray(gt_tree_xy, dtype=float)
            axis.scatter(
                gt_tree_arr[:, 0],
                gt_tree_arr[:, 1],
                s=44,
                marker="s",
                facecolors="none",
                edgecolors="#2e8b57",
                linewidths=1.5,
                alpha=0.90,
                label="gt tree",
            )
        if self.show_ground_truth and gt_bush_xy:
            gt_bush_arr = np.asarray(gt_bush_xy, dtype=float)
            axis.scatter(
                gt_bush_arr[:, 0],
                gt_bush_arr[:, 1],
                s=44,
                marker="D",
                facecolors="none",
                edgecolors="#b8860b",
                linewidths=1.5,
                alpha=0.90,
                label="gt bush",
            )
        if self.show_route_trace and route_xy:
            axis.plot(
                [point[0] for point in route_xy],
                [point[1] for point in route_xy],
                color="#444444",
                linewidth=1.8,
                alpha=0.85,
                label="route",
            )
            axis.scatter([route_xy[-1][0]], [route_xy[-1][1]], c="#111111", s=24, marker="o", alpha=0.9)

        if self.fixed_axes:
            self._apply_boundary_bounds_if_available(axis)
        else:
            all_xy = cluster_xy + confirmed_xy + candidate_xy + route_xy + gt_tree_xy + gt_bush_xy
            if all_xy:
                xs = [point[0] for point in all_xy]
                ys = [point[1] for point in all_xy]
                axis.set_xlim(min(xs) - 2.0, max(xs) + 2.0)
                axis.set_ylim(min(ys) - 2.0, max(ys) + 2.0)

        metrics_lines = [
            "cluster_points=%d" % len(cluster_xy),
            "clusters=%d" % len(set([label for label in plot_labels if label >= 0])),
            "confirmed=%d" % len(confirmed_xy),
            "candidates=%d" % len(candidate_xy),
        ]
        if self.show_route_trace:
            metrics_lines.append("route_points=%d" % len(route_xy))
        axis.text(
            self.METRICS_ANCHOR[0],
            self.METRICS_ANCHOR[1],
            "\n".join(metrics_lines),
            transform=axis.transAxes,
            ha="left",
            va="top",
            fontsize=9,
            bbox=dict(boxstyle="round,pad=0.25", facecolor="white", alpha=0.68),
            clip_on=False,
        )

        if self.show_ground_truth and self.show_ground_truth_labels:
            for point in gt_tree_xy[:90]:
                axis.text(point[0] + 0.10, point[1] + 0.10, "tree", fontsize=7.8, color="#1f5d3a")
            for point in gt_bush_xy[:90]:
                axis.text(point[0] + 0.10, point[1] + 0.10, "bush", fontsize=7.8, color="#8a6a00")

        handles, labels = axis.get_legend_handles_labels()
        if labels:
            axis.legend(loc="upper left", bbox_to_anchor=self.LEGEND_ANCHOR, borderaxespad=0.0)

        ensure_parent_dir(png_path)
        fig.savefig(png_path, dpi=180, bbox_inches="tight")
        plt.close(fig)

    def _build_metadata(self, output_dir, snapshot_png_path, copied_paths, reason):
        return {
            "saved_at_sec": self.now_sec(),
            "reason": reason,
            "run_dir": self.runtime["run_dir"],
            "output_dir": output_dir,
            "frame_id": self.data_frame_id if self.data_frame_id else "world",
            "route_frame_id": self.route_frame_id,
            "counts": {
                "cluster_points": len(self.cluster_points),
                "clusters": len(set([label for label in self._align_labels(len(self.cluster_points), self.cluster_labels) if label >= 0])),
                "confirmed_map": len(self.map_detections),
                "candidate_map": len(self.map_candidate_detections),
                "route_points": len(self.route_points),
            },
            "files": {
                "snapshot_png": snapshot_png_path,
                "tree_map_csv": copied_paths.get("tree_map_csv", ""),
                "tree_map_json": copied_paths.get("tree_map_json", ""),
                "tree_map_history_csv": copied_paths.get("tree_map_history_csv", ""),
                "tree_detection_history_csv": copied_paths.get("tree_detection_history_csv", ""),
            },
        }

    def _export_snapshot_dir(
        self,
        output_dir,
        tag,
        include_history,
        include_map_outputs,
        reason,
        snapshot_basename="tree_cluster_state",
    ):
        resolved_output_dir = normalize_optional_path(output_dir)
        if not resolved_output_dir:
            safe_tag = sanitize_token(tag, timestamp_token()) if tag else timestamp_token()
            resolved_output_dir = os.path.join(self.runtime["snapshots_dir"], safe_tag)
        ensure_dir(resolved_output_dir)

        self._trigger_map_export()

        snapshot_png_path = os.path.join(
            resolved_output_dir,
            self._snapshot_basename(snapshot_basename) + ".png",
        )
        self._plot_snapshot(snapshot_png_path)

        copied_paths = {
            "tree_map_csv": "",
            "tree_map_json": "",
            "tree_map_history_csv": "",
            "tree_detection_history_csv": "",
        }
        if include_map_outputs:
            copied_paths["tree_map_csv"] = self._copy_if_exists(self.tree_map_csv_output_path, resolved_output_dir)
            copied_paths["tree_map_json"] = self._copy_if_exists(self.tree_map_json_output_path, resolved_output_dir)
        if include_history:
            copied_paths["tree_map_history_csv"] = self._copy_if_exists(
                self.tree_map_history_output_path, resolved_output_dir
            )
            copied_paths["tree_detection_history_csv"] = self._copy_if_exists(
                self.tree_detection_history_output_path, resolved_output_dir
            )

        metadata_path = os.path.join(resolved_output_dir, "tree_snapshot_metadata.json")
        ensure_parent_dir(metadata_path)
        with open(metadata_path, "w") as handle:
            json.dump(
                self._build_metadata(
                    output_dir=resolved_output_dir,
                    snapshot_png_path=snapshot_png_path,
                    copied_paths=copied_paths,
                    reason=reason,
                ),
                handle,
                indent=2,
                sort_keys=True,
            )

        return {
            "output_dir": resolved_output_dir,
            "snapshot_png_path": snapshot_png_path,
            "tree_map_csv_path": copied_paths["tree_map_csv"],
            "tree_map_json_path": copied_paths["tree_map_json"],
            "tree_map_history_path": copied_paths["tree_map_history_csv"],
            "tree_detection_history_path": copied_paths["tree_detection_history_csv"],
            "metadata_json_path": metadata_path,
        }

    def handle_export_snapshot(self, request, response):
        try:
            exported = self._export_snapshot_dir(
                output_dir=request.output_dir,
                tag=request.tag,
                include_history=bool(request.include_history),
                include_map_outputs=bool(request.include_map_outputs),
                reason="service",
                snapshot_basename="tree_cluster_state",
            )
            response.success = True
            response.output_dir_resolved = exported["output_dir"]
            response.run_dir = self.runtime["run_dir"]
            response.snapshot_png_path = exported["snapshot_png_path"]
            response.tree_map_csv_path = exported["tree_map_csv_path"]
            response.tree_map_json_path = exported["tree_map_json_path"]
            response.tree_map_history_path = exported["tree_map_history_path"]
            response.tree_detection_history_path = exported["tree_detection_history_path"]
            response.metadata_json_path = exported["metadata_json_path"]
            response.message = "snapshot exported"
        except Exception as exc:
            self.logwarn("tree_snapshot_exporter: snapshot export failed: %s", exc)
            response.success = False
            response.output_dir_resolved = ""
            response.run_dir = self.runtime["run_dir"]
            response.snapshot_png_path = ""
            response.tree_map_csv_path = ""
            response.tree_map_json_path = ""
            response.tree_map_history_path = ""
            response.tree_detection_history_path = ""
            response.metadata_json_path = ""
            response.message = str(exc)
        return response

    def default_snapshot_cb(self, _):
        try:
            self._export_snapshot_dir(
                output_dir="",
                tag=self.snapshot_prefix + "_" + timestamp_token(),
                include_history=True,
                include_map_outputs=True,
                reason="topic",
                snapshot_basename="tree_cluster_state",
            )
        except Exception as exc:
            self.logwarn("tree_snapshot_exporter: default snapshot failed: %s", exc)

    def handle_shutdown(self):
        if not self.save_on_shutdown:
            return
        try:
            self._export_snapshot_dir(
                output_dir=self.runtime["run_dir"],
                tag=self.shutdown_snapshot_basename,
                include_history=True,
                include_map_outputs=True,
                reason="shutdown",
                snapshot_basename=self.shutdown_snapshot_basename,
            )
        except Exception as exc:
            self.logwarn("tree_snapshot_exporter: shutdown export failed: %s", exc)


def main(args=None):
    rclpy.init(args=args)
    node = TreeSnapshotExporter()
    spin_node(node)


if __name__ == "__main__":
    main()
